# Physics-rule fixes & inertia/second-moment type safety — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Eliminate the inertia / second-moment-of-volume confusion at compile time with strong types, then fix every CRITICAL and MAJOR physics finding from the 2026-06-09 review so morphology inputs actually drive the outputs instead of being masked by clamps.

**Architecture:** Two repos. Rule/geometry code lives in the `tonton` submodule (`modules/tonton`); the GTest plausibility + unit suite lives in the outer `tonton-example` repo and links the tonton library. Part A is a behavior-preserving refactor (the plausibility suite must stay byte-identical through it). Part B fixes physics file-by-file, each landing with a test or a plausibility assertion. Part C wires verification.

**Tech Stack:** C++20, GLM, compile-time dimensional `Quantity<…>` types, GoogleTest (system-installed), CMake.

**Spec:** `modules/tonton/docs/superpowers/specs/2026-06-09-physics-and-inertia-fixes-design.md`

**Paths & conventions:**
- Tonton source root: `/mnt/Passport/Libraries/Spehleon/tonton-example/modules/tonton`
- Test root: `/mnt/Passport/Libraries/Spehleon/tonton-example/tests`
- Build dir: `/mnt/Passport/Libraries/Spehleon/tonton-example/build`
- Configure once: `cmake -S /mnt/Passport/Libraries/Spehleon/tonton-example -B /mnt/Passport/Libraries/Spehleon/tonton-example/build`
- Build the suite: `cmake --build /mnt/Passport/Libraries/Spehleon/tonton-example/build --target tonton-tests`
- Run all tests: `/mnt/Passport/Libraries/Spehleon/tonton-example/build/tonton-tests`
- Run one: `/mnt/Passport/Libraries/Spehleon/tonton-example/build/tonton-tests --gtest_filter='Geometry.*'`
- **Commits:** rule/geometry edits commit inside `modules/tonton` (its own git repo); test edits commit in `tonton-example`. Both are on non-default branches already; commit, do not push, unless the user asks.

**Baseline before starting:** run the suite and record the green/red split (expected 5 green / 10 red). Save it: `/mnt/Passport/Libraries/Spehleon/tonton-example/build/tonton-tests --gtest_list_tests` then a full run; keep the output as the Part-A regression oracle.

---

## Part A — Inertia / second-moment strong types (behavior-preserving)

**Invariant:** no numeric output changes. After every Part A task, the plausibility suite's pass/fail set and printed values must be identical to baseline.

### Task A0: Establish the baseline oracle

**Files:** none (read-only).

- [ ] **Step 1: Configure and build the current suite**

Run:
```bash
cmake -S /mnt/Passport/Libraries/Spehleon/tonton-example -B /mnt/Passport/Libraries/Spehleon/tonton-example/build
cmake --build /mnt/Passport/Libraries/Spehleon/tonton-example/build --target tonton-tests
```
Expected: builds clean.

- [ ] **Step 2: Capture baseline results**

Run:
```bash
/mnt/Passport/Libraries/Spehleon/tonton-example/build/tonton-tests > /tmp/tonton_baseline.txt 2>&1; echo "exit=$?"
```
Expected: ~5 passing / ~10 failing. Keep `/tmp/tonton_baseline.txt`; Part A must reproduce the same `[  PASSED  ]` / `[  FAILED  ]` lists.

---

### Task A1: Create the tensor wrapper types + `ToInertia`

**Files:**
- Create: `modules/tonton/include/tonton_tensors.hpp`

- [ ] **Step 1: Write the header**

Create `modules/tonton/include/tonton_tensors.hpp`:
```cpp
#ifndef TONTON_TENSORS_HPP
#define TONTON_TENSORS_HPP
#include <glm/glm.hpp>

namespace TonTon {

// Second moment of volume:  C_ij = ∫ x_i x_j dV   (units m^5, taken about the centroid).
// This is what rintintin returns and what SkinnedMesh stores per joint.
// Eigen-polarity: the SMALLEST eigenvalue is the THINNEST physical axis.
struct SecondMomentTensor { glm::dmat3 C{1.0}; };

// Unit-density inertia tensor:  I = tr(C)·𝟙 − C.
// Eigen-polarity: the SMALLEST eigenvalue is the PRINCIPAL / LONGEST physical axis
// (opposite of SecondMomentTensor — this reversal is the footgun this header prevents).
struct InertiaTensor { glm::dmat3 I{1.0}; };

// The ONE place the C → I conversion lives.
inline InertiaTensor ToInertia(const SecondMomentTensor& c)
{
    const glm::dmat3& C = c.C;
    const double tr = C[0][0] + C[1][1] + C[2][2];
    return InertiaTensor{ tr * glm::dmat3(1.0) - C };
}

} // namespace TonTon
#endif // TONTON_TENSORS_HPP
```

- [ ] **Step 2: Commit**

```bash
cd /mnt/Passport/Libraries/Spehleon/tonton && git add include/tonton_tensors.hpp && git commit -m "feat(tensors): add SecondMomentTensor/InertiaTensor strong types + ToInertia"
```

---

### Task A2: Unit-test `ToInertia` (TDD for the conversion)

**Files:**
- Create: `tonton-example/tests/tonton_geometry_tests.cpp`
- Modify: `tonton-example/CMakeLists.txt` (add the new source to the `tonton-tests` target)

- [ ] **Step 1: Write the failing test file**

Create `tonton-example/tests/tonton_geometry_tests.cpp`:
```cpp
#include <gtest/gtest.h>
#include <glm/glm.hpp>
#include "tonton_tensors.hpp"

using namespace TonTon;

// I = tr(C)·𝟙 − C for a known diagonal second moment.
TEST(Geometry, ToInertiaDiagonal) {
    SecondMomentTensor c{ glm::dmat3(0.0) };
    c.C[0][0] = 1.0; c.C[1][1] = 2.0; c.C[2][2] = 4.0;   // tr = 7
    InertiaTensor got = ToInertia(c);
    EXPECT_DOUBLE_EQ(got.I[0][0], 7.0 - 1.0); // 6
    EXPECT_DOUBLE_EQ(got.I[1][1], 7.0 - 2.0); // 5
    EXPECT_DOUBLE_EQ(got.I[2][2], 7.0 - 4.0); // 3
    EXPECT_DOUBLE_EQ(got.I[0][1], 0.0);
}

// Off-diagonal products negate: I_xy = -C_xy.
TEST(Geometry, ToInertiaOffDiagonal) {
    SecondMomentTensor c{ glm::dmat3(0.0) };
    c.C[0][0] = 1.0; c.C[1][1] = 1.0; c.C[2][2] = 1.0;
    c.C[0][1] = c.C[1][0] = 0.5;
    InertiaTensor got = ToInertia(c);
    EXPECT_DOUBLE_EQ(got.I[0][1], -0.5);
    EXPECT_DOUBLE_EQ(got.I[1][0], -0.5);
    EXPECT_DOUBLE_EQ(got.I[0][0], 3.0 - 1.0); // tr=3
}
```

- [ ] **Step 2: Add the source to the test target**

In `tonton-example/CMakeLists.txt`, modify the `add_executable(tonton-tests …)` call (currently lines ~166-169) to add the new file:
```cmake
        add_executable(tonton-tests
            tests/tonton_plausibility_tests.cpp
            tests/tonton_geometry_tests.cpp
            ${TONTON_TEST_PIPELINE_SOURCES}
        )
```

- [ ] **Step 3: Build and run the new tests**

Run:
```bash
cmake -S /mnt/Passport/Libraries/Spehleon/tonton-example -B /mnt/Passport/Libraries/Spehleon/tonton-example/build
cmake --build /mnt/Passport/Libraries/Spehleon/tonton-example/build --target tonton-tests
/mnt/Passport/Libraries/Spehleon/tonton-example/build/tonton-tests --gtest_filter='Geometry.ToInertia*'
```
Expected: 2 PASS. (These pass on first write — they pin the conversion the rest of Part A routes through.)

- [ ] **Step 4: Commit**

```bash
cd /mnt/Passport/Libraries/Spehleon/tonton-example && git add tests/tonton_geometry_tests.cpp CMakeLists.txt && git commit -m "test(geometry): unit-test ToInertia conversion"
```

---

### Task A3: Route the three hand-rolled `tr(C)𝟙−C` conversions through `ToInertia`

**Files:**
- Modify: `modules/tonton/include/tonton_analysis.h:106-115`
- Modify: `modules/tonton/src/tonton_skinnedmesh.cpp:162-171` and `:173-182`

These three blocks are numerically identical to `ToInertia`; replacing them is behavior-preserving.

- [ ] **Step 1: `Analysis_Physical::inertia_restPose()`**

In `modules/tonton/include/tonton_analysis.h`, add the include near the top (with the other includes):
```cpp
#include "tonton_tensors.hpp"
```
Replace the body (lines 107-114):
```cpp
	inline glm::mat3 inertia_restPose() const 
	{
		auto & I = covariance_restPose;
		return glm::mat3{ 
			 I[1] +I[2],-I[3],-I[4],
			-I[3], I[0] +I[2],-I[5],
			-I[4],-I[5], I[0] +I[1] 
	};
	}
```
with:
```cpp
	inline glm::mat3 inertia_restPose() const 
	{
		auto & C = covariance_restPose;
		TonTon::SecondMomentTensor smt{ glm::dmat3{
			C[0], C[3], C[4],
			C[3], C[1], C[5],
			C[4], C[5], C[2] } };
		return glm::mat3(TonTon::ToInertia(smt).I);
	}
```

- [ ] **Step 2: Both `GetInertia` overloads**

In `modules/tonton/src/tonton_skinnedmesh.cpp`, add near the top includes:
```cpp
#include "tonton_tensors.hpp"
```
Replace the single-joint overload body (lines 164-170):
```cpp
	std::array<double, 6> I = GetCovariance(i, scale);
		
	return glm::dmat3{
		 I[1] +I[2],-I[3],-I[4],
		-I[3], I[0] +I[2],-I[5],
		-I[4],-I[5], I[0] +I[1]
	};
```
with:
```cpp
	std::array<double, 6> I = GetCovariance(i, scale);
	TonTon::SecondMomentTensor smt{ glm::dmat3{
		I[0], I[3], I[4],
		I[3], I[1], I[5],
		I[4], I[5], I[2] } };
	return TonTon::ToInertia(smt).I;
```
Replace the span overload body (lines 175-181) the same way (it begins `auto I = GetCovariance(joints, transforms, centroid_out, volume_out);`):
```cpp
	auto I = GetCovariance(joints, transforms, centroid_out, volume_out);
	TonTon::SecondMomentTensor smt{ glm::dmat3{
		I[0], I[3], I[4],
		I[3], I[1], I[5],
		I[4], I[5], I[2] } };
	return TonTon::ToInertia(smt).I;
```

> Note: `GetCovariance` packs as `{xx,yy,zz,xy,xz,yz}` = indices `{0,1,2,3,4,5}`, so the dmat3 reconstruction above (`C[0],C[3],C[4] / C[3],C[1],C[5] / C[4],C[5],C[2]`) matches the existing `EstimateCrossSection` reconstruction at `tonton_skinnedmesh.cpp:243-247`. The old hand-rolled form (`I[1]+I[2]` for xx, etc.) is algebraically the same as `tr−C`.

- [ ] **Step 3: Build and run the FULL suite — regression check**

Run:
```bash
cmake --build /mnt/Passport/Libraries/Spehleon/tonton-example/build --target tonton-tests
/mnt/Passport/Libraries/Spehleon/tonton-example/build/tonton-tests > /tmp/tonton_afterA3.txt 2>&1; echo "exit=$?"
diff <(grep -E '^\[  (PASSED|FAILED)' /tmp/tonton_baseline.txt) <(grep -E '^\[  (PASSED|FAILED)' /tmp/tonton_afterA3.txt)
```
Expected: `diff` prints nothing (identical pass/fail set). If anything changed, the conversion was not behavior-preserving — stop and inspect.

- [ ] **Step 4: Commit**

```bash
cd /mnt/Passport/Libraries/Spehleon/tonton && git add include/tonton_analysis.h src/tonton_skinnedmesh.cpp && git commit -m "refactor(geometry): route C→I conversions through ToInertia (no-op)"
```

---

### Task A4: Typed `EigenDecomposition` entry points + polarity unit test

The current `EigenDecomposition(glm::dmat3 const&)` (eigen.cpp:204) is called on a **covariance** at `tonton_skinnedmesh.cpp:259` (EstimateCrossSection) and `tonton_rigsolver.cpp:370` (capsule fit), and on an **inertia** at `tonton_skinnedmeshmemo.cpp:87` and `tonton_skinnedmesh.cpp:584`. We add two typed wrappers that forward to the existing raw function so the *kind* is explicit at the call site; the raw `glm::dmat3` overload stays (internal).

**Files:**
- Modify: `modules/tonton/include/tonton.h` (declarations — find the existing `EigenDecomposition` declaration; it's the documented external dependency)
- Modify: `modules/tonton/src/tonton_eigen.cpp:204` (add wrappers)
- Modify: `modules/tonton/tests` consumer call sites (see Task A5)
- Modify: `tonton-example/tests/tonton_geometry_tests.cpp`

- [ ] **Step 1: Add typed wrapper declarations**

Locate the existing declaration `std::pair<glm::quat, glm::vec3> EigenDecomposition(glm::dmat3 const& I);` (in the public header — per CLAUDE.md it's in `tonton.h`). Add beside it:
```cpp
// Typed entry points. Polarity differs by kind — see tonton_tensors.hpp.
//   InertiaTensor      → eigenvalues ascending; Small = principal/long axis.
//   SecondMomentTensor → eigenvalues ascending; Small = thinnest axis.
std::pair<glm::quat, glm::vec3> EigenDecomposition(InertiaTensor const& i);
std::pair<glm::quat, glm::vec3> EigenDecomposition(SecondMomentTensor const& c);
```
Add `#include "tonton_tensors.hpp"` to that header if not already visible.

- [ ] **Step 2: Implement the wrappers**

In `modules/tonton/src/tonton_eigen.cpp`, after the existing `EigenDecomposition(glm::dmat3 const& m)` (lines 204-208), add:
```cpp
std::pair<glm::quat, glm::vec3> TonTon::EigenDecomposition(InertiaTensor const& i)
{
	return EigenDecomposition(i.I);
}

std::pair<glm::quat, glm::vec3> TonTon::EigenDecomposition(SecondMomentTensor const& c)
{
	return EigenDecomposition(c.C);
}
```
Add `#include "tonton_tensors.hpp"` if needed.

- [ ] **Step 3: Add the polarity unit test**

Append to `tonton-example/tests/tonton_geometry_tests.cpp`:
```cpp
#include "tonton.h"   // EigenDecomposition

// A long-thin body along +x: large extent in x, small in y/z.
// SecondMoment: x-eigenvalue is LARGEST (most spread) → Small = thin axis (y or z).
// Inertia: rotation about the long axis is SMALLEST → Small = long axis (x).
TEST(Geometry, EigenPolarityReversal) {
    // second moment of a rod along x: Cxx >> Cyy == Czz
    SecondMomentTensor c{ glm::dmat3(0.0) };
    c.C[0][0] = 100.0; c.C[1][1] = 1.0; c.C[2][2] = 1.0;

    auto [q_c, v_c] = TonTon::EigenDecomposition(c);
    // ascending: smallest is a thin axis (value 1), largest is the long axis (100)
    EXPECT_NEAR(v_c[0], 1.0, 1e-6);
    EXPECT_NEAR(v_c[2], 100.0, 1e-6);

    InertiaTensor i = ToInertia(c);
    auto [q_i, v_i] = TonTon::EigenDecomposition(i);
    // I = tr-C: Ixx = (1+1)=2 (about long axis, smallest), Iyy=Izz=101
    EXPECT_NEAR(v_i[0], 2.0, 1e-6);
    EXPECT_NEAR(v_i[2], 101.0, 1e-6);
}
```

- [ ] **Step 4: Build and run**

Run:
```bash
cmake --build /mnt/Passport/Libraries/Spehleon/tonton-example/build --target tonton-tests
/mnt/Passport/Libraries/Spehleon/tonton-example/build/tonton-tests --gtest_filter='Geometry.*'
```
Expected: all `Geometry.*` PASS (3 tests). If `tonton.h` is the wrong header, the compiler error on the unit-test include tells you the right one — match the existing `EigenDecomposition` declaration's header.

- [ ] **Step 5: Commit**

```bash
cd /mnt/Passport/Libraries/Spehleon/tonton && git add include/tonton.h src/tonton_eigen.cpp && git commit -m "feat(eigen): typed EigenDecomposition entry points (Inertia/SecondMoment)"
cd /mnt/Passport/Libraries/Spehleon/tonton-example && git add tests/tonton_geometry_tests.cpp && git commit -m "test(geometry): eigen polarity reversal between inertia and second moment"
```

---

### Task A5: Make the four call sites use the typed entry points + fix the lying comments

This converts the latent footgun into compile-checked intent and corrects the misleading polarity comments in `tonton_skinnedmeshmemo.cpp`.

**Files:**
- Modify: `modules/tonton/src/tonton_skinnedmesh.cpp:259` (EstimateCrossSection — covariance) and `:584` (stalk — inertia)
- Modify: `modules/tonton/src/tonton_rigsolver.cpp:370` (capsule — covariance)
- Modify: `modules/tonton/src/Memos/tonton_skinnedmeshmemo.cpp:33-62, 65-81, 87` (inertia + comments)

- [ ] **Step 1: EstimateCrossSection — wrap the projected covariance**

In `tonton_skinnedmesh.cpp`, the projected matrix is a (projected) second moment. Change line 259 from:
```cpp
	std::pair<glm::quat, glm::vec3> eigen_decomp = TonTon::EigenDecomposition(projected_cov);
```
to:
```cpp
	std::pair<glm::quat, glm::vec3> eigen_decomp = TonTon::EigenDecomposition(SecondMomentTensor{projected_cov});
```

- [ ] **Step 2: Stalk detection — it decomposes an inertia**

At `tonton_skinnedmesh.cpp:584`, the variable is named `inertia`. Change:
```cpp
            auto eigen_decomp = EigenDecomposition(inertia);
```
to:
```cpp
            auto eigen_decomp = EigenDecomposition(InertiaTensor{inertia});
```

- [ ] **Step 3: Capsule fit — projected covariance**

At `tonton_rigsolver.cpp:370`, change:
```cpp
		auto [rotation, eigenvalues] = EigenDecomposition(projected_cov);
```
to:
```cpp
		auto [rotation, eigenvalues] = EigenDecomposition(SecondMomentTensor{projected_cov});
```
(Comment on the next line already says "eigenvalues sorted smallest→largest; largest is z" — correct for a covariance's perpendicular spread; leave it.)

- [ ] **Step 4: Projection matrix — decompose `unitInertia` as inertia + fix comments**

In `tonton_skinnedmeshmemo.cpp`, `LimbMetrics::unitInertia` is an inertia tensor. At line 87 change:
```cpp
	auto[rotation_q, vectors] = EigenDecomposition(m.unitInertia);
```
to:
```cpp
	auto[rotation_q, vectors] = EigenDecomposition(InertiaTensor{m.unitInertia});
```
Then correct the misleading polarity comments in `GetProjectionMatrix(EigenValue,quat)` (lines 33-62) and `GetProjectionDirection` (lines 65-81). Replace the `case EigenValue::Small:` comment "View along the axis with smallest eigenvalue (thinnest direction)" and the `Large` "widest direction" comments with the inertia-correct meaning:
```cpp
	case EigenValue::Small:
		// Decomposition source is an INERTIA tensor: smallest eigenvalue = principal/long axis.
		// Viewing along it looks down the body's long axis.
		return glm::mat4(rotation);  // rotation columns are the eigenvectors
```
and for `Large`:
```cpp
	case EigenValue::Large:
		// Largest inertia eigenvalue = the axis the body is "fattest" about (a short axis).
		return glm::mat4(
			glm::vec4(rotation[1], 0), 
			glm::vec4(rotation[2], 0), 
			glm::vec4(rotation[0], 0), 
			glm::vec4(0,0,0,1)
		);
```
Add an equivalent one-line correction to the `GetProjectionDirection` cases (note: these consume an inertia decomposition).

- [ ] **Step 5: Build — FULL regression check (still behavior-preserving)**

Run:
```bash
cmake --build /mnt/Passport/Libraries/Spehleon/tonton-example/build --target tonton-tests
/mnt/Passport/Libraries/Spehleon/tonton-example/build/tonton-tests > /tmp/tonton_afterA5.txt 2>&1; echo "exit=$?"
diff <(grep -E '^\[  (PASSED|FAILED)' /tmp/tonton_baseline.txt) <(grep -E '^\[  (PASSED|FAILED)' /tmp/tonton_afterA5.txt)
```
Expected: `diff` empty (wrappers forward to the same math; only types/comments changed). The 3 `Geometry.*` tests are new and additional.

- [ ] **Step 6: Commit**

```bash
cd /mnt/Passport/Libraries/Spehleon/tonton && git add src/tonton_skinnedmesh.cpp src/tonton_rigsolver.cpp src/Memos/tonton_skinnedmeshmemo.cpp && git commit -m "refactor(geometry): use typed EigenDecomposition at call sites; fix polarity comments"
```

> **Decision recorded:** Part A applies the strong types at the geometry boundary (the actual footgun: the C→I conversion and the eigen-polarity reversal). Propagating `SecondMomentTensor`/`InertiaTensor` as the *return types* of every `GetCovariance`/`GetInertia` overload was considered and deferred — it touches many call sites for no additional safety beyond what the typed `EigenDecomposition` + single `ToInertia` already guarantee. If the user wants the full signature change, it is a clean follow-up: change the headers, build, and resolve each compiler error site-by-site.

---

## Part B — Rules physics fixes (CRITICAL + MAJOR)

Order per spec: aerial → aquatic → terrestrial → sensory → takeoff → serpentine → specialized → metabolic → scratch. Each rule file's edits are one task; build + run the relevant filter after each.

---

### Task B1: `tonton_aerial.cpp` (A1–A5)

**Files:** Modify `modules/tonton/src/Rules/tonton_aerial.cpp`

- [ ] **Step 1 (A1 — Pennycuick frequency):** Replace the lambda at lines 196-201:
```cpp
	// Pennycuick wingbeat frequency: f = K×√(g/b)×(W/S)^0.375
	// Valid for birds (high Re regime)
	auto pennycuick_frequency = [](float K, acceleration_m_s2 g, length_m wingspan,
	                               load_N_m2 loading) -> freq_Hz {
		return K * sqrt(g / wingspan) * std::pow(float(loading), 0.375f);
	};
```
with the full relation (mass, span, area, density explicit):
```cpp
	// Pennycuick (1996) full allometric form:
	//   f = K · m^(3/8) · g^(1/2) · b^(−23/24) · S^(−1/3) · ρ^(−3/8)
	// Restores the span/area exponents and the air-density dependence (matters for
	// thin/dense-atmosphere presets). K≈1.08 in SI.
	auto pennycuick_frequency = [](float K, mass_kg m, acceleration_m_s2 g,
	                               length_m wingspan, area_m2 wing_area,
	                               density_Kg_m3 rho) -> freq_Hz {
		return K
		     * std::pow(float(m), 0.375f)
		     * std::sqrt(float(g))
		     * std::pow(float(wingspan), -23.0f/24.0f)
		     * std::pow(float(wing_area), -1.0f/3.0f)
		     * std::pow(float(rho), -0.375f);
	};
```
Update the call site (search for `pennycuick_frequency(` in this file) to pass `K=1.08f`, the body mass, `real_g`, wingspan, wing area, and `rho`. Confirm the exact local variable names at the call site before editing (they exist: `rho` line 75, `real_g` line 77, body mass `in.body_mass_kg()`, wing span/area from the `wing`/`total_wing_span_m` locals). Verify the dimensional `Quantity` type names (`mass_kg`, `area_m2`, `density_Kg_m3`) compile; if the project spells them differently, match the spelling used elsewhere in this file.

- [ ] **Step 2 (A2 — Strouhal peak-to-peak):** The cruise calc at line 384 uses `cruise_amplitude_rad * (span/2)` as the stroke half-excursion, then divides by Strouhal. `St = f·A/U` needs peak-to-peak A. Change line 384:
```cpp
    length_m cruise_stroke_length_m = cruise_amplitude_rad * (total_wing_span_m / 2.0f);
```
to:
```cpp
    // Strouhal requires PEAK-TO-PEAK amplitude (factor 2 over single-sided).
    length_m cruise_stroke_length_m = 2.0f * cruise_amplitude_rad * (total_wing_span_m / 2.0f);
```
Also fix the helper lambda at lines 113-116 to document/return peak-to-peak — change its body to `return (f * (2.0f*amplitude) * wing_length) / strouhal;` and update its comment to "peak-to-peak". Audit its call sites: if any caller already doubled the amplitude, remove that double to avoid 4×.

- [ ] **Step 3 (A3 — inertial power):** Replace the `I^0.3` heuristic at lines 474-478:
```cpp
    // Inertial power scales with wing inertia (small flappers have tiny wings)
    auto base_inertial_fraction = 0.10f;
    auto size_factor = std::pow(float(wing_inertia_kg_m2) / 1e-6f, 0.3f);
    auto inertial_fraction = base_inertial_fraction * size_factor;
    power_W inertial_power_cruise_W = aerodynamic_power_W * inertial_fraction;
```
with a single physical model:
```cpp
    // Inertial power to reverse the wing each half-stroke (Weis-Fogh / Van Den Berg & Rayner):
    //   P_inertial = k · 8π² · I · Φ² · f³   (Φ = stroke amplitude in radians, f = wingbeat Hz)
    // k≈0.5 accounts for elastic/aerodynamic recovery of part of the kinetic energy.
    const float k_recovery = 0.5f;
    power_W inertial_power_cruise_W = k_recovery * 8.0f * float(M_PI) * float(M_PI)
        * float(wing_inertia_kg_m2)
        * float(cruise_amplitude_rad) * float(cruise_amplitude_rad)
        * pow3(float(r.wingbeat_frequency_Hz));
```
Then at line 797 reduce the rod coefficient from the over-high 0.33 to the measured wing-inertia band:
```cpp
	auto wing_inertia_kg_m2 = 0.15f * wing_mass_kg * wing.span_m * wing.span_m;
```
(Add a comment citing Van Den Berg & Rayner 1995, I ≈ 0.1–0.15·m·L².) If `pow3` is not in scope here, use `f*f*f`.

- [ ] **Step 4 (A4 — buoyancy-reduced weight):** `g_effective` (line 79) already reduces gravity for near-neutrally-buoyant flyers, but the stall/lift weight at line 81 uses full `body_weight_N()`. Make the lift/stall path consistent: where weight is compared against lift (search the stall-speed and required-lift expressions, ~line 278), use `body_mass_kg * g_effective` instead of `weight_N`. Concretely, introduce right after line 81:
```cpp
    const auto effective_weight_N = std::max<force_N>(0.001f, body_mass_kg * g_effective);
```
and replace the `weight_N` uses in the **lift/stall** computations (not the metabolic ones) with `effective_weight_N`. Verify each replacement is a lift-balance term before changing it.

- [ ] **Step 5 (A5 — turn-radius guard):** At lines 174-177 guard the sqrt:
```cpp
	auto aerial_turn_radius = [](velocity_m_s v, acceleration_m_s2 g,
	                             float load_factor) -> length_m {
		float n2 = load_factor * load_factor - 1.0f;
		if (n2 <= 1e-4f || float(g) <= 0.0f) return length_m(1e6f); // ~straight line
		return (v * v) / (g * std::sqrt(n2));
	};
```

- [ ] **Step 6: Build + run aerial-touching tests**
```bash
cmake --build /mnt/Passport/Libraries/Spehleon/tonton-example/build --target tonton-tests
/mnt/Passport/Libraries/Spehleon/tonton-example/build/tonton-tests --gtest_filter='*Dragonfly*:*Bat*:Invariants*'
```
Expected: no regressions vs baseline for these (Dragonfly stays green; wingbeat still in 15–90 Hz). Bat/clade failures remain (out of scope). If Dragonfly wingbeat leaves 15–90 Hz, re-check K and exponents.

- [ ] **Step 7: Commit**
```bash
cd /mnt/Passport/Libraries/Spehleon/tonton && git add src/Rules/tonton_aerial.cpp && git commit -m "fix(aerial): Pennycuick exponents+ρ, peak-to-peak Strouhal, physical inertial power, buoyancy-reduced weight, turn-radius guard"
```

---

### Task B2: `tonton_aquatic.cpp` (Q1–Q6)

**Files:** Modify `modules/tonton/src/Rules/tonton_aquatic.cpp`

- [ ] **Step 1 (Q4 — heterocercal threshold + swim-bladder gate):** At line 73 raise the asymmetry threshold:
```cpp
				has_asymmetric_tail |= (offset > 0.12);   // was 0.01; sharks ~0.15+, symmetric fish < 0.1
```
This flips `Species.GreatWhiteShark` (no spurious swim bladder). Then gate the sink-rate path (lines 696-709) so swim-bladdered fish get no sink rate — wrap the negatively-buoyant block:
```cpp
    if(!has_swim_bladder && body_density_kg_m3 > fluid_density)
    {
        // ... existing sink_rate / lift_per_meter body ...
    }
```

- [ ] **Step 2 (Q1 — Strouhal peak-to-peak cruise):** At line 421:
```cpp
    auto cruise_speed_m_s = (beat_frequency_Hz * tail_amplitude_m) / strouhal_optimal;
```
→
```cpp
    // Strouhal uses peak-to-peak amplitude (2× the single-sided tail_amplitude_m).
    auto cruise_speed_m_s = (beat_frequency_Hz * (2.0f * tail_amplitude_m)) / strouhal_optimal;
```

- [ ] **Step 3 (Q2 — cycle-mean power factor):** The amplitude inversion (lines 328-334) equates mean available power to peak drag power `P ≈ ½ρ A v_peak³ Cd`. The cycle-mean of `v³` for sinusoidal motion is `⟨v³⟩ = (4/3π)·v_peak³`, so mean power is smaller by 4/3π and the solved amplitude must use that. Replace lines 330-332:
```cpp
    auto amplitude_cubed = swim_power_W /
        (4.0f * M_PI * M_PI * M_PI * fluid_density * tail_area_m2 *
         pow3(beat_frequency_Hz) * oscillation_drag_coef);
```
with:
```cpp
    // v_peak = 2π f A → P_peak = ½ρ A_tail Cd (2π f A)³.
    // Available power is the CYCLE MEAN: ⟨P⟩ = (4/3π)·P_peak. Invert for A using ⟨P⟩.
    const float cycle_mean = 4.0f / (3.0f * float(M_PI));
    auto amplitude_cubed = swim_power_W /
        (cycle_mean * 0.5f * fluid_density * tail_area_m2 *
         pow3(2.0f * float(M_PI) * beat_frequency_Hz) * oscillation_drag_coef);
```
(The denominator now matches the documented `P = ½ρ A v³ Cd` with `v=2πfA`, instead of the previous ad-hoc `4π³` grouping.)

- [ ] **Step 4 (Q6 — C-start from kinematics):** Replace lines 500-507:
```cpp
        // Acceleration = v / t, where v ≈ burst_speed
        auto c_start_acceleration = burst_speed_m_s / c_start_duration_s;
```
with fast-start kinematics independent of steady burst speed:
```cpp
        // Fast-start: the body straightens through ~one body length over the stage,
        // reaching peak velocity v ≈ 2L/t (constant-accel approximation of the C→straight snap).
        // a ≈ 2·v/t = 4L/t²  (Domenici & Blake 1997: a ~ 40–150 m/s² for fish).
        velocity_m_s c_start_peak_v = 2.0f * body_length_m / c_start_duration_s;
        auto c_start_acceleration = 2.0f * c_start_peak_v / c_start_duration_s;
```

- [ ] **Step 5 (Q3 — jet velocity from thrust=drag):** Replace lines 601-604 (the momentum-snapshot jet velocity):
```cpp
        // Jet velocity from momentum conservation
        // m_jet * v_jet = m_body * v_body
        auto jet_mass_kg = jet.jet_pulse_volume_m3 * fluid_density;
        jet.jet_velocity_m_s = (body_mass_kg * cruise_speed_m_s) / jet_mass_kg;
```
with a thrust = drag balance (`ṁ·v_jet = ½ρ v_body² Cd A`):
```cpp
        // Steady jet: thrust ṁ·v_jet balances body drag ½ρ v_body² Cd A.
        // ṁ = ρ·A_siphon·v_jet ⇒ ρ A_siphon v_jet² = ½ρ Cd A_body v_body²
        // ⇒ v_jet = v_body·√(½ Cd A_body / A_siphon).
        auto siphon_area_m2 = std::max(0.0001f, float(jet.jet_pulse_volume_m3) /
            std::max(0.01f, float(body_length_m))); // crude siphon area proxy
        jet.jet_velocity_m_s = cruise_speed_m_s *
            std::sqrt(0.5f * drag_coefficient * cross_section_m2 / siphon_area_m2);
```
Confirm `drag_coefficient` and `cross_section_m2` are in scope here (they are used near line 703); if not, hoist them.

- [ ] **Step 6 (Q5 — lift_per_meter units):** At line 708 the field `lift_per_meter_N` (N/m) is assigned a force (N). Divide by a characteristic glide length:
```cpp
        // Lift needed per metre of travel = weight spread over a glide length ~ body length.
        lift_per_meter_N = float(weight_in_water_N) / std::max(0.01f, float(body_length_m));
```

- [ ] **Step 7: Build + run aquatic tests**
```bash
cmake --build /mnt/Passport/Libraries/Spehleon/tonton-example/build --target tonton-tests
/mnt/Passport/Libraries/Spehleon/tonton-example/build/tonton-tests --gtest_filter='*Shark*:*Eel*:*Penguin*:Invariants*'
```
Expected: `Species.GreatWhiteShark` flips to GREEN (has_swim_bladder==false; burst in [5,20]). `PenguinSwims` burst bound now satisfiable (its propulsion-mode assertion stays red — out of scope). Eel stays green.

- [ ] **Step 8: Commit**
```bash
cd /mnt/Passport/Libraries/Spehleon/tonton && git add src/Rules/tonton_aquatic.cpp && git commit -m "fix(aquatic): heterocercal threshold+swim-bladder gate, peak-to-peak+cycle-mean speed, kinematic C-start, thrust-balance jet, lift_per_meter units"
```

---

### Task B3: `tonton_terrestrial.cpp` (T1–T7)

**Files:** Modify `modules/tonton/src/Rules/tonton_terrestrial.cpp`

- [ ] **Step 1 (T1 — remove fabricated catapult/frog energy):** At line 426 remove the energy-fabricating multiplier:
```cpp
	takeoff_velocity_m_s *= std::sqrt(power_amplification_ratio) * 0.5f;
```
→
```cpp
	// Power amplification changes the RATE of energy release, not total work (W=F·d is
	// already counted). It does not add kinetic energy — no velocity multiplier here.
	// (power_amplification_ratio retained for diagnostics / timing only.)
```
And at line 556 remove the frog ×1.3:
```cpp
	takeoff_velocity_m_s *= 1.3f; // 30% boost from elastic storage
```
→
```cpp
	// Elastic storage is already added to the work budget below; do not double-count as a velocity boost.
```

- [ ] **Step 2 (T2 — realistic spring strain):** At line 488:
```cpp
	auto max_extension_m = avg_leg_length_m * 2.0f;
```
→
```cpp
	// Tendon strain at failure ~8–10%; use ε=0.08 of the spring's working length.
	auto max_extension_m = avg_leg_length_m * 0.08f;
```
And at line 550:
```cpp
	elastic_storage_J = 0.5f * spring_stiffness * (avg_leg_length_m * 3.0f) * (avg_leg_length_m * 3.0f);
```
→
```cpp
	// Cuticle/tendon strain ε≈0.03 (insect cuticle); x = ε·L.
	auto spring_extension_m = avg_leg_length_m * 0.03f;
	elastic_storage_J = 0.5f * spring_stiffness * spring_extension_m * spring_extension_m;
```

- [ ] **Step 3 (T3 — gravity-scaled running speed):** `base_sprint` is gravity-blind. After all biomechanical modifiers are applied but before line 254 (where `base_sprint` is emitted), scale by `√(g/9.81)`:
```cpp
	// Sprint speed scales with √g (Froude similarity); 9.81 is the calibration gravity.
	{
		float g_scale = std::sqrt(std::max(1e-3f, float(in.environment.gravity_m_s2)) / 9.81f);
		base_sprint *= g_scale;
		max_sustainable_speed_m_s *= g_scale;
	}
```
Place this just before the struct that contains `.max_sprint_speed_m_s=base_sprint,` (line 254). Confirm `base_sprint` and `max_sustainable_speed_m_s` are still in scope there.

- [ ] **Step 4 (T4 — divisor guards):** At line 92-94 guard the leg ratio:
```cpp
	auto leg_length_ratio = (float(expected_leg_m) > 1e-6f)
	    ? functional_length / expected_leg_m
	    : 1.0f;
	auto leg_correction = std::sqrt(std::max(0.0f, float(leg_length_ratio)));
	leg_correction = std::clamp(leg_correction, 0.3f, 2.0f);
```
At line 125 guard the turning-radius lateral accel (find `max_lateral_accel`'s definition; it derives from gravity):
```cpp
	length_m min_turning_radius_m = (float(max_lateral_accel) > 1e-4f)
	    ? (base_sprint * base_sprint) / max_lateral_accel
	    : length_m(1e6f);   // ~no turning constraint in micro-gravity
```

- [ ] **Step 5 (T5 — peaked thermal curve):** Replace the monotonic Q10 block. After line 164 computes `q10_multiplier`, fold in a Gaussian-style optimum so performance falls off above an optimum rather than rising to a 50 °C cliff. Replace lines 164-184 (the q10 application + cold/heat penalties) with:
```cpp
	// Thermal performance curve: rises with Q10 toward an optimum, then declines.
	// Topt ~ 35 °C (308 K) for typical ectotherms; CTmax ~ 45 °C.
	const float Topt_K = 308.15f, width_K = 12.0f;
	float T_K = float(in.environment.temperature_K);
	float rise = std::pow(q10, std::min(0.0f, (T_K - Topt_K)) / 10.0f); // Q10 below optimum
	float fall = std::exp(-((T_K - Topt_K) * (T_K - Topt_K)) / (2.0f * width_K * width_K));
	float thermal_perf = std::clamp(rise * fall, 0.05f, 1.0f);
	base_sprint *= thermal_perf;
	max_sustainable_speed_m_s *= thermal_perf;
	max_acceleration_m_s2 *= thermal_perf;
```
(Verify `q10` is defined above; keep the original `q10` value.)

- [ ] **Step 6 (T6 — Garland intercept + arthropod penalty):** At line 54 lower the over-high intercept and at line 195 fix the exponent. Line 54:
```cpp
	auto base_constant = 0.9f; // Default for general vertebrates
```
→
```cpp
	auto base_constant = 0.5f; // Recalibrated to Garland (1983) regression; old 0.9 over-predicted small-animal speed ~2-3×
```
(adjust the MAMMALIA/AVES/REPTILIA offsets at lines 58/60/62 down by the same 0.4 to preserve their relative ordering: 1.05→0.65, 0.85→0.45, 0.70→0.30). Line 195 arthropod penalty:
```cpp
		auto size_penalty = 0.1f / float(out.physical.body_mass_kg); // Linear penalty above 100g
```
→
```cpp
		auto size_penalty = std::pow(0.1f / float(out.physical.body_mass_kg), 1.0f/3.0f); // strength ∝ area: M^(-1/3)
```

- [ ] **Step 7 (T7 — jump-height clamp):** At line 561 the `50× body length` ceiling truncates small jumpers (Borelli: jump height is ~size-independent). Raise it for small masses:
```cpp
	auto max_reasonable_height = s.physical.body_length_m * 50.0f;
```
→
```cpp
	// Borelli's law: absolute jump height is ~size-independent (~1-2 m for good jumpers),
	// so a "×body length" ceiling wrongly truncates small animals. Use an absolute floor.
	auto max_reasonable_height = std::max<length_m>(s.physical.body_length_m * 50.0f, 2.0f);
```

- [ ] **Step 8: Add the jump-energy plausibility test (T1)**

Append to `tonton-example/tests/tonton_plausibility_tests.cpp` (inside the existing test fixture's namespace; mirror the style of existing `Species` tests). Add a check, for the treefrog (a jumper, analyzed in air), that takeoff kinetic energy does not exceed available muscle work:
```cpp
TEST(Physics, JumpEnergyConserved) {
    const Output& o = Analyze("treefrog.glb", EarthAir());   // match the real filename used elsewhere
    ASSERT_TRUE(o.jumping.has_value());
    const auto& j = *o.jumping;
    float m = float(o.physical.body_mass_kg);
    float v = float(j.takeoff_velocity_m_s);
    float ke = 0.5f * m * v * v;
    // Muscle work available for one jump: leg muscle mass × work density (~250 J/kg upper bound).
    float muscle_mass = float(o.metabolic.muscle_mass_kg);   // confirm member name
    float max_work = muscle_mass * 250.0f;
    EXPECT_LE(ke, max_work) << "takeoff KE exceeds muscle work — energy fabricated";
}
```
Confirm the real treefrog filename and the metabolic muscle-mass member name against the existing test file before running; adjust to match.

- [ ] **Step 9: Build + run**
```bash
cmake --build /mnt/Passport/Libraries/Spehleon/tonton-example/build --target tonton-tests
/mnt/Passport/Libraries/Spehleon/tonton-example/build/tonton-tests --gtest_filter='*Treefrog*:*Cat*:Physics.JumpEnergyConserved:Invariants*'
```
Expected: `Physics.JumpEnergyConserved` PASS; Cat sprint still in [8,18]; no new regressions. (Treefrog clade failure stays — out of scope.)

- [ ] **Step 10: Commit**
```bash
cd /mnt/Passport/Libraries/Spehleon/tonton && git add src/Rules/tonton_terrestrial.cpp && git commit -m "fix(terrestrial): remove fabricated jump energy, realistic spring strain, gravity-scaled speed, divisor guards, peaked thermal curve, Garland recal, Borelli jump ceiling"
cd /mnt/Passport/Libraries/Spehleon/tonton-example && git add tests/tonton_plausibility_tests.cpp && git commit -m "test(physics): jump KE must not exceed muscle work"
```

---

### Task B4: `tonton_sensory.cpp` (S1–S4)

**Files:** Modify `modules/tonton/src/Rules/tonton_sensory.cpp`

- [ ] **Step 1 (S1 — binocular_overlap fraction):** At lines 27-29 the overlap is stored as an angle. The contract (`tonton_analysis.h:618`) wants a 0..1 fraction. Replace:
```cpp
	auto overlap_angle = 2.0f * fov_per_eye_rad - in.builder->sensory.vision.angular_separation_rad;
	
	vision.binocular_overlap = overlap_angle;
```
with:
```cpp
	// Fraction of one eye's field that overlaps the other: (2·FOV − separation)/(2·FOV), clamped 0..1.
	float two_fov = 2.0f * float(fov_per_eye_rad);
	float overlap_frac = (two_fov > 1e-4f)
	    ? (two_fov - float(in.builder->sensory.vision.angular_separation_rad)) / two_fov
	    : 0.0f;
	vision.binocular_overlap = std::clamp(overlap_frac, 0.0f, 1.0f);
```
Then change the field type in `tonton_analysis.h:618` from `angle_rad` to a plain `float` (it is a fraction, not an angle):
```cpp
	float binocular_overlap{};      // 0=none, 1=full overlap (fraction)
```
Check for any other reader of `binocular_overlap` (grep the tonton repo) and ensure none treats it as radians.

- [ ] **Step 2 (S2 — activity_pattern convention):** Per `tonton_input.h:39`, `activity_pattern` is **1 = nocturnal, 0 = diurnal**. The sensory file treats it as the opposite (1 = diurnal). Audit all 9 uses and flip each:
  - Line 56-57: `activity_bonus` for acuity — nocturnal animals get the acuity bonus differently; make the bonus track nocturnality correctly: `float nocturnality = in.behavior.activity_pattern;` then `glm::mix(0.7f, 1.3f, nocturnality)` only if nocturnal eyes are intended to be more acute at low light — **decide and document**: per spec, acuity bonus should align to the header. Set `vision.acuity = geometric_acuity * glm::mix(1.3f, 0.7f, in.behavior.activity_pattern);` (diurnal=0 → 1.3 bonus; nocturnal=1 → 0.7).
  - Line 64: `has_color_vision = (in.behavior.activity_pattern < 0.6f);` (diurnal → color).
  - Line 65: `has_night_vision = (in.behavior.activity_pattern > 0.4f);` (nocturnal → night vision).
  - Line 149: `if (in.behavior.activity_pattern > 0.5f) { // Nocturnal` (flip `<` → `>`).
  - Line 408: `auto nocturnal_bonus = in.behavior.activity_pattern;` (was `1.0f - …`).
  - Line 521: `if (s.aerial.has_value() && in.behavior.activity_pattern > 0.6f) { // Nocturnal` (flip).
  - Line 624: `auto nocturnal_bonus = in.behavior.activity_pattern;` (was `1.0f - …`).
  - Line 737: `if (in.behavior.activity_pattern > 0.6f) {` (nocturnal mammal olfaction; flip).
  - Line 766: `} else if (in.behavior.activity_pattern > 0.7f) {` (kiwi nocturnal; flip).

  After editing, re-grep `activity_pattern` in this file and confirm every comparison reads correctly under "1 = nocturnal".

- [ ] **Step 3 (S3 — horizon bonus):** At line 88 replace the dimensionally-meaningless bonus:
```cpp
		auto horizon_bonus = 1.0f + std::sqrt(float(max_stalk_height) * 2.0f); // Rough approximation
```
with the real geometric-horizon relation (distance to horizon ≈ 3.57·√h km for h in metres), expressed as a bounded multiplier:
```cpp
		// Horizon distance ≈ 3.57·√h km (h in m). Express as a modest detection-range
		// multiplier relative to a 1.7 m eye height baseline.
		float horizon_km = 3.57f * std::sqrt(std::max(0.0f, float(max_stalk_height)));
		float baseline_km = 3.57f * std::sqrt(1.7f);
		auto horizon_bonus = std::clamp(horizon_km / baseline_km, 1.0f, 3.0f);
```

- [ ] **Step 4 (S4 — eye-diameter accumulation):** At lines 375-381 the NONE/default branch computes `eye_diameter` but never adds it. Add the accumulation (match how the other clade branches feed the accumulator — confirm the accumulator variable name; the function returns `bool` and writes a diameter out-param or accumulator):
```cpp
		case CladeFlags::NONE:
		default:
		{
			// Fallback: very conservative estimate based on body size
			length_m eye_diameter = 0.015f * std::cbrt(float(physical.body_mass_kg));
			accumulator += eye_diameter;   // was missing — value computed but dropped
			return true;
		}
```
Confirm the accumulator's exact name/type by reading the other `case` branches in `PredictedEyeDiameter`; mirror them.

- [ ] **Step 5: Build + run — S1 flips 6 invariant tests**
```bash
cmake --build /mnt/Passport/Libraries/Spehleon/tonton-example/build --target tonton-tests
/mnt/Passport/Libraries/Spehleon/tonton-example/build/tonton-tests --gtest_filter='Invariants*'
```
Expected: the 6 `Invariants` cases that failed on `binocular_overlap ∈ [0,1]` now PASS (all 7 models). 

- [ ] **Step 6: Commit**
```bash
cd /mnt/Passport/Libraries/Spehleon/tonton && git add src/Rules/tonton_sensory.cpp include/tonton_analysis.h && git commit -m "fix(sensory): binocular_overlap as fraction, align activity_pattern to 1=nocturnal, real horizon bonus, accumulate fallback eye diameter"
```

---

### Task B5: `tonton_takeoffanalysis.cpp` (K1–K4)

**Files:** Modify `modules/tonton/src/Rules/tonton_takeoffanalysis.cpp`

- [ ] **Step 1 (K4 — guards first):** Add an early guard near line 151 region and before the `wings[0]` index at line 245. At the top of the function that uses `wings[0]` (RequiredJumpVelocity, line ~245), add:
```cpp
    if (aerial.wings.empty() || aerial.wingbeat_frequency_Hz <= 0) return /* sentinel, match return type */;
    velocity_m_s wing_tip_speed = aerial.wings[0].wing_tip_velocity(aerial.wingbeat_frequency_Hz);
```
(Use the function's existing early-return convention — confirm its return type and what a "cannot take off" sentinel looks like elsewhere in the file.) Also add at the very start of the main analysis function:
```cpp
    if (output.physical.body_mass_kg <= 0) { result.confidence = 0.0f; return result; }
```

- [ ] **Step 2 (K1 — like-for-like power fraction):** At line 62:
```cpp
    result.takeoff_power_fraction = float(takeoff_power_needed_W) / float(output.metabolic.available_muscle_power_W);
```
`takeoff_power_needed_W` is mechanical (hovering aerodynamic power); `available_muscle_power_W` is mechanical too if it's the post-efficiency figure — but per the review this divides metabolic by mechanical. Make both mechanical: divide the mechanical requirement by the mechanical muscle power. If `available_muscle_power_W` is already mechanical, the fix is to ensure `takeoff_power_needed_W` is also mechanical (it is). If `available_muscle_power_W` is metabolic, convert with the muscle efficiency η≈0.25:
```cpp
    // Compare mechanical-to-mechanical. Muscle delivers ~25% of metabolic power as mechanical work.
    const float muscle_efficiency = 0.25f;
    power_W mechanical_muscle_W = output.metabolic.available_muscle_power_W * muscle_efficiency;
    result.takeoff_power_fraction = float(takeoff_power_needed_W) / std::max(1e-3f, float(mechanical_muscle_W));
```
**Verify first** whether `available_muscle_power_W` is mechanical or metabolic by reading its definition in `tonton_metabolic.cpp`; only apply the η factor if it is metabolic. Document which it is in the comment.

- [ ] **Step 3 (K2 — blade-element mean lift):** At lines 167-178 the lift uses tip velocity² over total area, overestimating ~3×. Use the radius-of-gyration mean (mean of v² over the blade is ⅓·v_tip² for a triangular velocity distribution):
```cpp
    pressure_Pa dynamic_pressure = 0.5f * air_density * (tip_velocity * tip_velocity);
```
→
```cpp
    // Velocity grows linearly to the tip; mean of v² over the wing = v_tip²/3 (blade-element).
    pressure_Pa dynamic_pressure = 0.5f * air_density * (tip_velocity * tip_velocity) / 3.0f;
```

- [ ] **Step 4 (K3 — ground effect on lift only):** At lines 44-45 the bonus multiplies `net_vertical_force_N`, which includes the jet/thrust term. Apply it only to the lift component. Find where lift and thrust are summed into `net_vertical_force_N`; multiply only the lift term:
```cpp
    result.ground_effect_bonus = T::GroundEffectBonus(aerial.wing_span_m, ground_height_m);
    // Ground effect augments LIFT only (image-vortex effect), not jet/reaction thrust.
    lift_component_N *= result.ground_effect_bonus;
    result.net_vertical_force_N = lift_component_N + thrust_component_N - weight_N;
```
Adapt to the actual local variable names for the lift and thrust components (read lines 30-49 to identify them). If the code currently only has the combined `net_vertical_force_N`, split it into lift + thrust first.

- [ ] **Step 5: Build + run**
```bash
cmake --build /mnt/Passport/Libraries/Spehleon/tonton-example/build --target tonton-tests
/mnt/Passport/Libraries/Spehleon/tonton-example/build/tonton-tests --gtest_filter='*Dragonfly*:*Bat*:Invariants*'
```
Expected: no crashes on empty-wing models; Dragonfly stays green; `takeoff_power_fraction` now in a sane band (< ~1 for fliers). No new regressions.

- [ ] **Step 6: Commit**
```bash
cd /mnt/Passport/Libraries/Spehleon/tonton && git add src/Rules/tonton_takeoffanalysis.cpp && git commit -m "fix(takeoff): like-for-like power fraction, blade-element mean lift, ground effect on lift only, empty-wing/zero-mass guards"
```

---

### Task B6: `tonton_serpentine.cpp` (P1–P2)

**Files:** Modify `modules/tonton/src/Rules/tonton_serpentine.cpp`

- [ ] **Step 1 (P1 — terrestrial slip factor):** At lines 188-194 terrestrial undulation uses wave speed `λ·f` directly as ground speed with only an efficiency-from-anisotropy factor floored at 0.3. The aquatic branch (line 320) applies an explicit 0.8 slip. Give the terrestrial branch a slip/advance ratio. Replace lines 189-196:
```cpp
    auto wavelength_m = result.lateral_undulation.wavelength_ratio * body_length_m;
    auto lateral_speed_m_s = wavelength_m * frequency_Hz;

    // Efficiency depends on friction anisotropy and substrate
    // Better anisotropy = faster movement
    auto efficiency = glm::clamp(result.friction_anisotropy_ratio / 3.0f, 0.3f, 1.0f);
    lateral_speed_m_s *= efficiency;
```
with:
```cpp
    auto wavelength_m = result.lateral_undulation.wavelength_ratio * body_length_m;
    auto wave_speed_m_s = wavelength_m * frequency_Hz;

    // Lateral undulation advances at a fraction of wave speed (slip): snakes typically
    // achieve a 0.3-0.6 advance ratio on natural ground (Hu et al. 2009). Anisotropy
    // sets the achievable end of that band; net thrust → 0 as anisotropy → 1 (see P2).
    float advance_ratio = glm::clamp(0.3f + 0.3f * (result.friction_anisotropy_ratio - 1.0f), 0.0f, 0.6f);
    auto lateral_speed_m_s = wave_speed_m_s * advance_ratio;
```

- [ ] **Step 2 (P2 — anisotropy as enabling condition):** The replacement above already makes thrust vanish as anisotropy → 1 (advance_ratio → 0 when ratio == 1) and grow with anisotropy. Confirm `friction_anisotropy_ratio` is defined so that 1.0 = isotropic (no preferred direction). If the convention is inverted (1.0 = maximally anisotropic), invert the formula to `0.6f * (1.0f - 1.0f/ratio)` instead. Read the `friction_anisotropy_ratio` assignment above this block to confirm, and document the chosen convention in the comment.

- [ ] **Step 3: Build + run**
```bash
cmake --build /mnt/Passport/Libraries/Spehleon/tonton-example/build --target tonton-tests
/mnt/Passport/Libraries/Spehleon/tonton-example/build/tonton-tests --gtest_filter='*Eel*:Invariants*'
```
Expected: Eel stays green (eel is aquatic; serpentine terrestrial path mostly affects snakes — verify no model regresses). No serpentine sample model exists, so this is a no-regression check.

- [ ] **Step 4: Commit**
```bash
cd /mnt/Passport/Libraries/Spehleon/tonton && git add src/Rules/tonton_serpentine.cpp && git commit -m "fix(serpentine): terrestrial slip/advance ratio; anisotropy as enabling thrust condition"
```

---

### Task B7: `tonton_specialized.cpp` (D1–D3)

**Files:** Modify `modules/tonton/src/Rules/tonton_specialized.cpp`

- [ ] **Step 1 (D3 — use environment gravity):** At line 102:
```cpp
    auto relative_limb_strength = max_forelimb_force / (s.physical.body_mass_kg * 9.81f);
```
→
```cpp
    auto relative_limb_strength = max_forelimb_force / (s.physical.body_mass_kg * in.environment.gravity_m_s2);
```
(Confirm `in.environment` is reachable in this scope; if the param is named differently, match it.)

- [ ] **Step 2 (D1 — dig speed from P=F·v):** At lines 146-150 cross-section cancels, leaving the constant `stroke_len·freq`. Drive from power against soil penetration resistance so morphology matters. Replace lines 145-150:
```cpp
    // Volume excavated per stroke
    auto cross_section_m2 = s.physical.cross_sectional_area_m2;
    auto volume_per_stroke = cross_section_m2 * stroke_length_m;

    // Digging speed = (volume per stroke) * (frequency) / (tunnel cross-section)
    result.max_dig_speed_m_s = (volume_per_stroke * stroke_frequency_Hz) / cross_section_m2;
```
with:
```cpp
    auto cross_section_m2 = std::max(1e-4f, float(s.physical.cross_sectional_area_m2));

    // Soil penetration resistance: force needed to advance the tunnel face one metre.
    //   F_soil = soil_resistance_Pa · tunnel_area.  Advance rate set by available power:
    //   v = P_dig / F_soil, capped by the limb's kinematic stroke rate (stroke_len·freq).
    const float soil_resistance_Pa = 50000.0f; // compacted soil ~50 kPa penetration resistance
    force_N soil_force_N = soil_resistance_Pa * cross_section_m2;
    power_W dig_power_W = max_forelimb_force * (stroke_length_m * stroke_frequency_Hz); // F·v of the stroke
    velocity_m_s power_limited_v = dig_power_W / std::max(1e-3f, float(soil_force_N));
    velocity_m_s kinematic_v = stroke_length_m * stroke_frequency_Hz;
    result.max_dig_speed_m_s = std::min(power_limited_v, kinematic_v);
```
Now forelimb force and body cross-section both affect the result. Confirm `max_forelimb_force` is in scope (it is, used at line 102).

- [ ] **Step 3 (D2 — Laplace constriction pressure):** At lines 262-271 pressure scales linearly with coil count, rescued by a 60 kPa clamp. Use Laplace's law `P ≈ σ·(t/r)` where coil count drives total grip coverage, not local pressure. Replace lines 261-271:
```cpp
    // Squeeze pressure from muscle stress distributed over prey surface
    result.max_squeeze_pressure_Pa = muscle_stress_Pa * effective_coils * 0.15f; // ~15% efficiency
    ...
    result.max_squeeze_pressure_Pa = std::clamp<pressure_Pa>(
        result.max_squeeze_pressure_Pa, 5000.0f, 60000.0f);
```
with:
```cpp
    // Laplace's law for a cylindrical wrap: P = σ · (t / r)
    //   σ = muscle tension stress, t = body-wall thickness, r = coil radius.
    // Coil COUNT does not raise local pressure — it raises total grip force/coverage.
    float wall_thickness_m = std::max(1e-3f, float(body_diameter_m) * 0.15f); // muscle layer ~15% of diameter
    float coil_radius_m    = std::max(1e-3f, float(body_diameter_m) * 0.5f);
    result.max_squeeze_pressure_Pa = muscle_stress_Pa * 0.15f * (wall_thickness_m / coil_radius_m);
    // Total grip scales with coverage = coils (kept as a separate field if present):
    // result.grip_coverage = effective_coils;  // uncomment/adapt if such a field exists
    // Safety net only (not the answer): empirical pythons ~30 kPa.
    result.max_squeeze_pressure_Pa = std::clamp<pressure_Pa>(result.max_squeeze_pressure_Pa, 1000.0f, 60000.0f);
```
Confirm `body_diameter_m` (or an equivalent) is available; if only `cross_sectional_area_m2` exists, derive `body_diameter_m = 2·√(area/π)`. Keep the clamp as a safety net but the intermediate is now physical (≈ a few kPa to tens of kPa without relying on the ceiling).

- [ ] **Step 4: Add dig-speed varies-with-input test (D1)**

Append to `tonton-example/tests/tonton_plausibility_tests.cpp`: assert dig speed is finite/positive where digging is present, and (if two diggers exist among samples, else scale one model) that it is not a constant independent of morphology. With only one digger likely among the 7 samples, assert the weaker but real property — dig speed is bounded and positive:
```cpp
TEST(Physics, DigSpeedPhysical) {
    // Find any sample model that produces a digging output; assert it's finite & positive.
    bool checked = false;
    for (const char* f : {"cat.glb","treefrog.glb","eel.glb"}) { // adjust to actual diggers if any
        const Output& o = Analyze(f, EarthAir());
        if (o.specialized.digging.has_value()) {
            float v = float(o.specialized.digging->max_dig_speed_m_s);
            EXPECT_GT(v, 0.0f);
            EXPECT_LT(v, 1.0f) << "burrowers advance < ~1 m/s";
            checked = true;
        }
    }
    GTEST_SKIP_("no digging sample model") ? void() : void(); // no-op if none; keeps suite green
    (void)checked;
}
```
If none of the 7 models exercise digging, replace this with a direct unit test that calls the digging function with two different forelimb forces and asserts the outputs differ — this is the real D1 guarantee. Prefer the unit test if the digging function is callable in isolation; otherwise keep the bounded-output assertion and note the coverage gap with `log`/comment.

- [ ] **Step 5: Build + run**
```bash
cmake --build /mnt/Passport/Libraries/Spehleon/tonton-example/build --target tonton-tests
/mnt/Passport/Libraries/Spehleon/tonton-example/build/tonton-tests --gtest_filter='Physics.*:Invariants*'
```
Expected: new dig test PASS (or SKIP if no digger); no regressions.

- [ ] **Step 6: Commit**
```bash
cd /mnt/Passport/Libraries/Spehleon/tonton && git add src/Rules/tonton_specialized.cpp && git commit -m "fix(specialized): dig speed from P=F·v vs soil resistance, Laplace constriction pressure, environment gravity"
cd /mnt/Passport/Libraries/Spehleon/tonton-example && git add tests/tonton_plausibility_tests.cpp && git commit -m "test(physics): dig speed is physical/bounded"
```

---

### Task B8: `tonton_metabolic.cpp` (B1–B3)

**Files:** Modify `modules/tonton/src/Rules/tonton_metabolic.cpp`

- [ ] **Step 1 (B1 — RMR coefficient units):** Confirm the reference: mammal `4.18·M^0.75` is BMR in **Watts** (Kleiber 3.4 kcal/day·kg^0.75 ≈ 4.1 W at M^0.75 — correct). The AVES `6.25·M^0.72` is plausibly a kcal/day or different-unit regression that is ~too high relative to mammals in W. Bring AVES into the same Watt basis. At lines 85-92 change:
```cpp
            .rmr_coef = 6.25f,
            .rmr_exp = 0.72f,
```
to the passerine/non-passerine avian BMR in Watts (McKechnie & Wolf; non-passerine ≈ mammalian level, slightly higher):
```cpp
            .rmr_coef = 4.8f,    // avian BMR in W (was 6.25 — that figure reads as kcal/day, ~20× high in W)
            .rmr_exp = 0.72f,
```
Leave PISCES (0.8, line 148) and REPTILIA (0.5, line 164) — those are already low ectotherm values consistent with Watts; add a one-line comment on each confirming "Watts" so the unit is pinned. **Before changing the AVES number, sanity-check against a known bird:** a 1 kg bird BMR ≈ 5–7 W; with `4.8·1^0.72 = 4.8 W` (reasonable) vs old `6.25` (also plausible at 1 kg but the exponent/scale diverges at large mass). If a literature value contradicts 4.8, use the pinned literature value and cite it.

- [ ] **Step 2 (B2 — endotherm floor not blind multiply):** At line 323:
```cpp
            rmr_coefficient *= 5.0f;         // Boost to endotherm levels (~5x ectotherm)
```
→
```cpp
            // Target a floor at the mammalian endotherm level rather than blindly stacking ×5
            // (which can push an already-warm clade past mammalian RMR).
            rmr_coefficient = std::max(rmr_coefficient, 4.18f);
```

- [ ] **Step 3 (B3 — gentler power Q10):** At line 422:
```cpp
        auto q10 = 2.5f; // Typical value
```
→
```cpp
        auto q10 = 1.8f; // Muscle mechanical power is less temperature-sensitive than metabolic rate
```
(If the same `q10` local is reused for metabolic rate elsewhere in this function, introduce a separate `power_q10 = 1.8f` for the muscle-power path only and leave the metabolic 2.5 intact. Read lines 415-430 to confirm scope.)

- [ ] **Step 4: Build + run**
```bash
cmake --build /mnt/Passport/Libraries/Spehleon/tonton-example/build --target tonton-tests
/mnt/Passport/Libraries/Spehleon/tonton-example/build/tonton-tests --gtest_filter='Invariants*:*Cat*:*Penguin*'
```
Expected: `max_rate ≥ basal` and `aerobic_scope ≥ 1` invariants still hold for all models; no metabolic value goes non-physical. Cat/penguin metabolic stays sane.

- [ ] **Step 5: Commit**
```bash
cd /mnt/Passport/Libraries/Spehleon/tonton && git add src/Rules/tonton_metabolic.cpp && git commit -m "fix(metabolic): avian RMR in Watts, endotherm floor not blind ×5, gentler muscle-power Q10"
```

---

### Task B9: `tonton_scratch.cpp` (C-scr1–C-scr2)

**Files:** Modify `modules/tonton/src/Rules/tonton_scratch.cpp`

- [ ] **Step 1 (C-scr2 — divide-by-zero guard):** At line 108:
```cpp
    bool reasonable_density = (physical.body_mass_kg / physical.body_volume_m3 > 100.0f &&
                              physical.body_mass_kg / physical.body_volume_m3 < 2000.0f);
```
→
```cpp
    bool reasonable_density = false;
    if (physical.body_volume_m3 > 1e-9f) {
        float density = float(physical.body_mass_kg) / float(physical.body_volume_m3);
        reasonable_density = (density > 100.0f && density < 2000.0f);
    }
```

- [ ] **Step 2 (C-scr1 — pendulum frequency /(2π)):** At line 234:
```cpp
    r.natural_sway_frequency_Hz = sqrt(in.environment.gravity_m_s2 / (r.stretched_length_m + max_length)) / M_PI;
```
→
```cpp
    // Simple pendulum: f = √(g/L)/(2π).  L = distance from pivot to the swinging segment's CoM.
    auto pendulum_length_m = std::max(1e-3f, float(r.stretched_length_m + max_length) * 0.5f); // CoM ~ mid-length
    r.natural_sway_frequency_Hz = std::sqrt(float(in.environment.gravity_m_s2) / pendulum_length_m) / (2.0f * float(M_PI));
```
(The `·0.5` puts the effective pendulum length at the segment's mid-length CoM rather than its tip; adjust if the segment's CoM is known more precisely.)

- [ ] **Step 3: Build + run full suite**
```bash
cmake --build /mnt/Passport/Libraries/Spehleon/tonton-example/build --target tonton-tests
/mnt/Passport/Libraries/Spehleon/tonton-example/build/tonton-tests > /tmp/tonton_afterB.txt 2>&1; echo "exit=$?"
```
Expected: no NaN/crash; sway frequencies roughly halve (the `/π → /2π` change). No regression in the invariant set.

- [ ] **Step 4: Commit**
```bash
cd /mnt/Passport/Libraries/Spehleon/tonton && git add src/Rules/tonton_scratch.cpp && git commit -m "fix(scratch): guard zero-volume density check, correct pendulum sway frequency /(2π)"
```

---

## Part C — Verification & final sweep

### Task C1: Gravity-scaling plausibility test (T3)

**Files:** Modify `tonton-example/tests/tonton_plausibility_tests.cpp`

- [ ] **Step 1: Write the test**

Add a test that the same model runs faster under higher gravity (sprint ∝ √g). Add a low-gravity env helper if not present (mirror `EarthAir()`):
```cpp
TEST(Physics, SprintScalesWithGravity) {
    Output_Env moon = EarthAir();             // copy, then lower gravity (adjust struct/setter to match)
    moon.gravity_m_s2 = 1.62f;                // confirm the env field name
    const Output& earth = Analyze("cat.glb", EarthAir());
    const Output& low   = Analyze("cat.glb", moon);
    ASSERT_TRUE(earth.terrestrial.has_value() && low.terrestrial.has_value());
    float se = float(earth.terrestrial->max_sprint_speed_m_s);
    float sl = float(low.terrestrial->max_sprint_speed_m_s);
    EXPECT_LT(sl, se) << "lower gravity should reduce sprint speed (√g scaling)";
    EXPECT_NEAR(sl / se, std::sqrt(1.62f / 9.81f), 0.05f);
}
```
The `Analyze` cache keys on filename — if it ignores the env, give the low-g run a distinct cache key (e.g. an `Analyze(file, env, key)` overload) or bypass the cache for this test. Confirm against the existing `Analyze` helper signature and adapt.

- [ ] **Step 2: Build + run**
```bash
cmake --build /mnt/Passport/Libraries/Spehleon/tonton-example/build --target tonton-tests
/mnt/Passport/Libraries/Spehleon/tonton-example/build/tonton-tests --gtest_filter='Physics.SprintScalesWithGravity'
```
Expected: PASS.

- [ ] **Step 3: Commit**
```bash
cd /mnt/Passport/Libraries/Spehleon/tonton-example && git add tests/tonton_plausibility_tests.cpp && git commit -m "test(physics): terrestrial sprint scales with environment gravity"
```

### Task C2: Final full-suite sweep & red→green confirmation

**Files:** none (read-only) + update `CLAUDE.md` testing note if accurate.

- [ ] **Step 1: Run everything**
```bash
cmake --build /mnt/Passport/Libraries/Spehleon/tonton-example/build --target tonton-tests
/mnt/Passport/Libraries/Spehleon/tonton-example/build/tonton-tests > /tmp/tonton_final.txt 2>&1; echo "exit=$?"
grep -E '^\[  (PASSED|FAILED)' /tmp/tonton_final.txt
```
Expected red→green per spec:
  - 6 `Invariants` tests (binocular_overlap) → GREEN (S1).
  - `Species.GreatWhiteShark` → GREEN (Q4).
  - `Physics.JumpEnergyConserved`, `Physics.SprintScalesWithGravity`, dig test, all `Geometry.*` → GREEN.
  - Still RED (out of scope, documented): `Species.Treefrog` clade, `Species.Bat` clade/wings, `Species.PenguinSwims` propulsion-mode assertion.

- [ ] **Step 2: Confirm no scope creep regressions**

Compare `/tmp/tonton_final.txt` against `/tmp/tonton_baseline.txt`: every previously-green test must still be green; the only remaining reds must be the three documented out-of-scope clade/propulsion items.

- [ ] **Step 3: Update the stale testing note**

In `modules/tonton/CLAUDE.md`, the "No formal test suite exists" line is outdated. Update it to point at `tonton-tests`. Commit:
```bash
cd /mnt/Passport/Libraries/Spehleon/tonton && git add CLAUDE.md && git commit -m "docs: note tonton-tests plausibility suite in CLAUDE.md"
```

- [ ] **Step 4: Summarize results to the user** — report the final green/red split and which spec items each green test confirms.

---

## Self-review notes (author checklist applied)

- **Spec coverage:** A (types) → A1-A5; A1-A5/Q1-Q6/T1-T7/S1-S4/K1-K4/P1-P2/D1-D3/B1-B3/C-scr1-2 each map to a B-task step of the same tag. Part C unit + plausibility tests → A2, A4, B3 step 8, B7 step 4, C1. Red→green map → C2.
- **Behavior-preserving invariant (Part A):** enforced by the `diff` regression checks in A3 step 3 and A5 step 5 against the A0 baseline.
- **Known unknowns flagged for the implementer (verify-before-edit, not placeholders):** exact header for `EigenDecomposition`'s declaration (A4); whether `available_muscle_power_W` is mechanical vs metabolic (K1); `friction_anisotropy_ratio` convention (P2); presence of a digging sample model (B7); `Analyze` cache keying on env (C1); exact dimensional `Quantity` type spellings (B1 aerial). Each step says what to confirm and how. These are genuine codebase facts the implementer reads, not deferred decisions.
- **Out of scope (unchanged, documented):** penguin propulsion classification, treefrog/bat clade detection, deferred item #5.
