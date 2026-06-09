# Design: Physics-rule fixes & inertia/second-moment type safety

- **Date:** 2026-06-09
- **Status:** Approved (design); implementation plan to follow
- **Repo:** `tonton` submodule (code) + `tonton-example` (test target)

## Context

A scientific/mathematical review of `src/Rules/` (2026-06-09) found that many outputs
are "plausible-looking but wrong": energy-conservation violations, formulas that
algebraically cancel to a constant, wrong allometric exponents, a normalized field
stored in the wrong units, and a recurring pattern of clamps masking non-physical
intermediate values. Separately, the geometry layer carries a latent footgun: rintintin
now returns **second moment of volume** (it used to return unit-density inertia), both
are symmetric 3×3 matrices, and `EigenDecomposition` is fed both kinds with nothing
distinguishing them — the eigenvalue ordering reverses between the two, so a mixup
silently inverts projection axes.

A GTest plausibility suite (`tonton-tests`, in `tonton-example`) now loads 7 sample
models and asserts biologically-grounded ranges. It is currently 5 green / 10 red; the
red tests pin the bugs this spec fixes (plus some out-of-scope clade bugs).

## Goals

1. Eliminate the inertia / second-moment confusion **at compile time** via distinct
   types, so the wrong tensor cannot be passed to a consumer.
2. Fix every **CRITICAL** and **MAJOR** finding from the review, replacing degenerate
   "clamp-rescued" formulas with physically-grounded models so morphology inputs
   actually affect the result.
3. Anchor each fix to a test (existing red test going green, or a new targeted test).

## Non-goals (explicitly out of scope for this spec)

- MINOR / NIT tuning notes from the review — **except** a few low-cost MINOR fixes that
  sit in the exact lines a MAJOR fix already touches (tagged `MINOR→fix` below); those are
  folded in opportunistically, the way you'd tidy code you're already editing.
- Clade-detection bugs surfaced by the test suite (treefrog→MAMMALIA, bat→no-wings/
  ectotherm). These live in the wordlist/builder, not `src/Rules/`; separate spec.
- Deferred measurement-estimation item #5 (hardcoded forewing separation).
- **Penguin propulsion classification** (paddle vs underwater-flight). The engine emits
  *both* paddling and flying because it cannot disambiguate the morphology — a
  classification-confidence problem, not a physics bug. `Species.PenguinSwims` (which
  asserts `MEDIAN_PAIRED_FIN`) remains a documented known-gap until that separate work.

## Guiding principle

Clamps stay as **safety nets**, not as the answer. After each fix the morphology inputs
must measurably change the output (verified by a "varies with input" test where relevant).

---

## Part A — Inertia / second-moment strong types

**Invariant for this part: it is behavior-preserving.** No numeric output changes; the
plausibility suite must produce identical results before and after Part A. It is a safety
refactor only.

### Current state (all consistent with "second moment of volume")

- Per-joint `SkinnedMesh::covariance[i]` and `Analysis_Physical::covariance_restPose` are
  the second moment of volume `∫ xᵢxⱼ dV` (units m⁵). Confirmed by the comment at
  `tonton_skinnedmesh.cpp:241-243`.
- `GetCovariance` transforms it correctly via the congruence map `M·C·Mᵀ`
  (`tonton_skinnedmesh.cpp:148`).
- Conversion to inertia `I = tr(C)·𝟙 − C` is hand-rolled in **three** places:
  `Analysis_Physical::inertia_restPose()` (`tonton_analysis.h:107`), and both
  `GetInertia` overloads (`tonton_skinnedmesh.cpp:166, 177`).
- `EigenDecomposition` (`tonton_eigen.cpp:204`) sorts eigenvalues ascending and is called
  on a **covariance** at `tonton_skinnedmesh.cpp:259` (EstimateCrossSection) and
  `tonton_rigsolver.cpp:370` (capsule fit), and on an **inertia** at
  `tonton_skinnedmeshmemo.cpp:87` (GetProjectionMatrix) and `tonton_skinnedmesh.cpp:584`
  (stalk detection). Polarity is reversed between the two (covariance Small = thin axis;
  inertia Small = long axis), but each site is currently self-consistent.
- `GetProjectionMatrix(EigenValue,quat)` / `GetProjectionDirection`
  (`tonton_skinnedmeshmemo.cpp:33, 65`) carry comments describing **covariance** polarity
  ("smallest eigenvalue = thinnest direction") while their only live input is an
  **inertia** decomposition. The code works; the comments lie.

### Design

Introduce two thin wrapper types (header, e.g. `tonton_tensors.hpp`):

```cpp
struct SecondMomentTensor { glm::dmat3 C; };   // ∫ xᵢxⱼ dV  (units m⁵, about centroid)
struct InertiaTensor      { glm::dmat3 I; };    // tr(C)·𝟙 − C  (unit-density inertia)

InertiaTensor ToInertia(const SecondMomentTensor& c);   // the ONE conversion site
```

- Replace the three hand-rolled `tr(C)𝟙−C` blocks with `ToInertia(...)`.
- `GetCovariance*` return `SecondMomentTensor`; `GetInertia*` return `InertiaTensor`.
  `LimbMetrics::unitInertia` becomes `InertiaTensor`.
- `EigenDecomposition` gets two typed entry points (or one taking a tagged type):
  - `EigenDecomposition(InertiaTensor)` — documented: eigenvalues ascending,
    **Small = principal/long axis**.
  - `EigenDecomposition(SecondMomentTensor)` — documented: **Small = thinnest axis**.
  - Passing the wrong tensor is a compile error.
- Fix the comments in `GetProjectionMatrix`/`GetProjectionDirection` to state they consume
  an **inertia** decomposition and what each `EigenValue` selects physically.
- The `std::array<float,6>` storage on the wire (`covariance`, `covariance_restPose`) is
  unchanged; only the in-memory tensor passed between functions is typed.

### Risk & mitigation

Breadth: the change touches the geometry layer that feeds every rule. Mitigation: it is a
no-op numerically — the **regression guard is the plausibility suite staying byte-identical**
through Part A, plus new unit tests (Part C).

---

## Part B — Rules physics fixes (CRITICAL + MAJOR)

Each item: location → the bug → the fix.

### `tonton_aerial.cpp`

- **A1 (MAJOR):** Pennycuick wingbeat frequency (~line 196) uses span exponent −1/2, area
  −3/8, drops the air-density term, and K=3.87. → Implement the full relation
  `f = K·m^(3/8)·g^(1/2)·b^(−23/24)·S^(−1/3)·ρ^(−3/8)` with K≈1.08 (or refit K to the full
  exponents). Restores correct span/area exponents and the ρ dependence (matters for the
  thin/dense-atmosphere presets).
- **A2 (MAJOR):** Strouhal cruise speed (~lines 113-116, 384) uses single-sided amplitude;
  `St = f·A/U` requires **peak-to-peak** A. → Use `A_pp = 2·amplitude` consistently.
- **A3 (CRITICAL):** Inertial power is modeled two inconsistent ways; the
  `inertial_fraction ∝ (I/1e-6)^0.3` curve (~lines 475-478) over-predicts large-flyer power
  (>100% of aero power). → Use a single physical model: `P_inertial = k·8π²·I·Φ²·f³` (or the
  cited Van Den Berg & Rayner 11-15% band), delete the `I^0.3` heuristic, and re-check the
  wing-inertia coefficient (0.33 → measured ~0.1-0.15).
- **A4 (MAJOR):** Effective-gravity buoyancy reduction is dropped from stall/lift weight for
  near-neutrally-buoyant (underwater) flyers (~line 79-81 vs 278). → Use the buoyancy-reduced
  weight consistently.
- **A5 (MINOR→keep):** Guard `√(load_factor²−1)` in turn radius (~line 174).

### `tonton_aquatic.cpp`

- **Q1 (CRITICAL):** Strouhal cruise speed (line 421) single-sided amplitude. →
  `U = f·2·tail_amplitude/St`.
- **Q2 (CRITICAL):** Power→amplitude inversion (lines 271-335) equates mean available power
  to **peak** drag power. → Insert the cycle-mean factor `⟨v³⟩ = (4/3π)·v_peak³`.
- **Q3 (MAJOR):** Jet velocity (lines 594-597) uses a total-momentum snapshot. → Derive from
  a thrust = drag balance (`ṁ·v_jet = ½ρv_body²·Cd·A`).
- **Q4 (MAJOR):** Heterocercal-tail asymmetry threshold (lines 70-74) of 0.01 spuriously
  gives sharks a swim bladder. → Raise to ~0.10-0.15; gate the `sink_rate` path on
  `!has_swim_bladder` so swim-bladdered fish don't get a spurious sink rate.
  **(Flips `Species.GreatWhiteShark`.)**
- **Q5 (MAJOR):** `lift_per_meter_N` (line 708) stores a force in an N/m field. → Divide by a
  characteristic glide length, or rename the field.
- **Q6 (MAJOR):** C-start acceleration (lines 499-509) derived from steady burst swim speed
  (itself mana/clade-scaled). → Use fast-start kinematics (body length + curvature), not
  steady speed.
- Ensure burst speed is sane in-environment (the 181 m/s seen in air was the wrong-env
  artifact; Q1/Q2 plus the burst formula keep it realistic). **(Supports
  `Species.PenguinSwims` burst bound and shark burst range.)**

### `tonton_terrestrial.cpp`

- **T1 (CRITICAL):** Catapult jump multiplies takeoff velocity by `√(power_amplification)·0.5`
  (lines 423-426), fabricating ~2.5× kinetic energy; frog branch adds ×1.3 (line 556). →
  Remove both. Power amplification changes *rate*, not total work (already captured by
  `W=F·d`). **(New test: jump KE ≤ muscle work.)**
- **T2 (CRITICAL):** Elastic energy uses spring extension 2L/3L (lines 488, 550) = 200-300%
  strain. → Use `x = ε·L` (cuticle ε≈0.02-0.03, tendon ε≈0.08-0.10). Removes reliance on the
  rescue clamp.
- **T3 (MAJOR):** Running speed is gravity-blind (only mention of Froude is a comment). →
  Multiply by `√(g/9.81)` so low-gravity presets behave; optionally expose Froude gait
  transitions.
- **T4 (CRITICAL-ish guard):** Zero-gravity / zero-mass produce inf turning radius and
  inf leg-correction (lines 87-94, 125). → Guard divisors; sentinel for g≈0.
- **T5 (MAJOR):** Q10 rises monotonically to a 50 °C cliff (lines 164-179). → Peaked thermal
  performance curve.
- **T6 (MAJOR):** Garland speed intercept overestimates small-animal speed ~2-3×; arthropod
  size penalty uses M^(−1) (lines 53-67, 190-201). → Recalibrate intercept to the published
  regression in the chosen unit; arthropod penalty M^(−1/3).
- **T7 (MINOR→fix):** Jump-height clamp `< 50× body length` (line 561) contradicts Borelli's
  law and truncates small jumpers. → Raise/remove for small masses.

### `tonton_sensory.cpp`

- **S1 (CRITICAL):** `binocular_overlap` stored in radians but documented/consumed as a 0..1
  fraction (lines 24-29; `tonton_analysis.h:618`). → Compute
  `clamp((2·fov − sep)/(2·fov), 0, 1)`; decide the field's type (fraction). **(Flips 6
  invariant tests.)**
- **S2 (CRITICAL):** `activity_pattern` convention inverted vs `tonton_input.h:39`
  (1=nocturnal): acuity bonus, color/night vision (lines 56-65), and `nocturnal_bonus =
  1−activity_pattern` (lines 408, 624) are all backwards. → Audit all 8 uses; align to the
  header.
- **S3 (MAJOR):** Eyestalk horizon bonus (line 88) is dimensionally meaningless. → Real
  horizon `≈ 3.57·√h` km, or drop the pretense.
- **S4 (MINOR→fix):** `PredictedEyeDiameter` NONE/default branch (line 375) computes a value
  but never adds it to the accumulator. → Add `accumulator += eye_diameter;`.

### `tonton_takeoffanalysis.cpp`

- **K1 (CRITICAL):** `takeoff_power_fraction = hovering_power_W / available_muscle_power_W`
  (line 62) divides metabolic by mechanical power (off by 1/efficiency ≈ 4-10×). → Compare
  like-for-like (mechanical-to-mechanical or metabolic-to-metabolic).
- **K2 (MAJOR):** Max lift uses tip-velocity² over total wing area (lines 167-178),
  overestimating ~3×. → Use radius-of-gyration / blade-element mean (≈⅓ factor).
- **K3 (MAJOR):** Ground-effect bonus applied to the jet-thrust term (lines 37-49). → Apply
  only to the lift term.
- **K4 (guard):** `RequiredJumpVelocity` indexes `wings[0]` with no empty check (line 245);
  add `body_mass<=0` early guard (line 151).

### `tonton_serpentine.cpp`

- **P1 (CRITICAL):** Terrestrial undulation uses wave speed `λ·f` as ground speed (lines
  188-194); no slip factor (the aquatic branch uses 0.8). → Apply a terrestrial slip/advance
  ratio (~0.3-0.6).
- **P2 (MAJOR):** Friction anisotropy used as a linear speed multiplier with a 0.3 floor. →
  Model as the enabling thrust condition: net thrust → 0 as anisotropy → 1.

### `tonton_specialized.cpp`

- **D1 (CRITICAL):** Dig speed (lines 146-150) cancels to the constant `stroke_len·freq`;
  cross-section divides out and all soil/force terms are unused. → Drive from `P = F·v`
  against soil penetration resistance so morphology matters. **(New test: dig speed varies
  with forelimb force / body scale.)**
- **D2 (MAJOR):** Constriction pressure scales linearly with coil count (lines 262-265),
  ~13× over empirical, rescued by a 60 kPa clamp. → Laplace's law `P ≈ σ·(t/r)`; coil count
  drives total grip force/coverage, not local pressure.
- **D3 (MINOR→fix):** Hardcoded `9.81` (line 102) where the rest uses
  `environment.gravity_m_s2`. → Use the environment.

### `tonton_metabolic.cpp`

- **B1 (MAJOR/verify):** Confirm AVES/PISCES/REPTILIA RMR coefficients are in **Watts**, not
  kcal/day or mlO₂/hr (lines 85-96). The AVES `6.25·M^0.72` looks like a kcal/day regression
  (~20× too high in W). → Re-express in W (mammal `4.18·M^0.75` is the correct reference).
- **B2 (MAJOR):** Endotherm upgrade `*= 5.0` (line 323) can stack past the mammalian level. →
  `rmr_coefficient = max(rmr_coefficient, 4.18f)` (target a floor, not a blind multiplier).
- **B3 (MINOR→fix):** Q10 for muscle *power* (lines 422-424) uses the metabolic 2.5. → Use a
  gentler power Q10 (~1.8) / cap cold impairment.

### `tonton_scratch.cpp`

- **C-scr1 (MAJOR):** Tail sway frequency (line 234) divides by `π`; pendulum `f = √(g/L)/(2π)`.
  → Use `/(2π)` and an effective pendulum length to the swinging segment's CoM.
- **C-scr2 (MINOR→fix):** Guard `body_volume_m3 == 0` in the density check (line 108).

---

## Part C — Verification

Extend `tonton-example/tests/tonton_plausibility_tests.cpp` and add focused unit tests
(in the tonton repo where pure-math).

### New unit tests (tonton repo)
- `ToInertia(C)` equals `tr(C)·𝟙 − C` for known matrices; round-trips a diagonal case.
- `EigenDecomposition` polarity: for a synthetic long-thin body, inertia Small = long axis,
  covariance Small = thin axis.

### Plausibility-suite assertions (per fix)
- **Behavior-preserving check (Part A):** suite results identical before/after Part A.
- Jump: takeoff KE ≤ muscle work × efficiency (T1); no `power_amplification` energy gain.
- Dig: speed differs between two morphologies / scales (D1).
- Gravity: terrestrial sprint scales with environment gravity (T3).

### Red → green map (existing suite)
- S1 (binocular_overlap) → 6 `Invariants` tests green.
- Q4 (heterocercal/swim-bladder) → `Species.GreatWhiteShark` green.
- Q1/Q2 (aquatic speed) → keeps `Species.PenguinSwims` burst bound satisfiable and shark
  burst in [5,20]. (Penguin propulsion-mode assertion stays red — out of scope.)

---

## Sequencing

1. **Part A** (inertia strong types) — first, behavior-preserving; suite must stay unchanged;
   land the new unit tests.
2. **Part B**, file-by-file in this order: aerial → aquatic → terrestrial → sensory →
   takeoff → serpentine → specialized → metabolic → scratch. Each fix lands with its test.
3. **Part C** assertions added alongside the fix that makes them pass.

## Open risks

- Inertia refactor breadth (mitigated by the no-op invariant).
- B1 (metabolic units) may require pinning a literature value for the avian/fish/reptile
  RMR coefficients in Watts.
- Some "correct" values are ranges, not numbers; tests assert ranges, not point values.
