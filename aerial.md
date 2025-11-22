# Aerial Flight Analysis - Refactoring Plan

## Current State: What's Wrong

The `tonton_aerial.cpp` implementation has fundamental physics errors that accumulated over AI-assisted iterations:

1. **Power costs 28-112× too high** (563 W/N vs expected 5-20 W/N for insects)
2. **Speed calculations absurd** (dragonfly: 150 km/h cruise vs reality: 18-36 km/h)
3. **`adjust_for_power_budget()` is a hack** - reduces frequency to fit power budget without checking if lift = weight is satisfied
4. **Scaling violations** - 100× scaled dragonfly would still claim it can fly (violates square-cube law)
5. **Disk area uncertainty** - unclear if π·span² or π·span²/4 is correct for multi-wing configurations
6. **Beat amplitude units confusion** - mixed angular (radians) and linear (meters) inconsistently

## Ground Truth: Real Animal Data for Validation

### Dragonfly (Anax junius - Common Green Darner)
**Must pass this test case:**
- Body mass: 0.6-1.0 g
- Wingspan: 9-11 cm
- Wingbeat frequency: 25-40 Hz
- Forward flight speed: 5-10 m/s (18-36 km/h)
- Can hover: YES
- Wing loading: ~5-10 N/m²
- Power loading: ~50-100 W/kg

**Sources to verify:**
- May, M. L. (1991). "Dragonfly flight: power requirements at high speed and acceleration." *Journal of Experimental Biology* 158(1): 325-342.
- Alexander, D. E. (1984). "Unusual phase relationships between the forewings and hindwings in flying dragonflies." *Journal of Experimental Biology* 109(1): 379-383.

### Hummingbird (Archilochus colubris - Ruby-throated)
**Should also validate:**
- Body mass: 3-4 g
- Wingspan: 8-11 cm
- Wingbeat frequency: 50-80 Hz
- Hovering: EXCELLENT (primary mode)
- Forward speed: 10-15 m/s
- Power loading: ~80-120 W/kg

### Large Bird (Columba livia - Rock Pigeon)
- Body mass: 300-500 g
- Wingspan: 60-70 cm
- Wingbeat frequency: 5-8 Hz
- Forward speed: 15-20 m/s
- Hovering: NO
- Wing loading: 30-50 N/m²

## Step-by-Step Verification Plan

### Phase 1: Literature Review & Equation Validation

#### 1.1 Check bibliography.md for relevant papers
**Action:** Read and extract key equations from:
- [ ] Pennycuick (1996) - Wingbeat frequency scaling
- [ ] Ellington (1984) - Insect flight mechanics
- [ ] Norberg (1990) - Vertebrate flight
- [ ] Dickinson et al. (1999) - LEV (Leading Edge Vortex) effects at low Re
- [ ] Any papers on momentum theory / actuator disk theory

**Extract:**
- Exact formulas with all coefficients
- Units and variable definitions
- Range of validity (body size, Reynolds number)
- Known limitations

#### 1.2 Web search for standard aerodynamics
**Search topics:**
- [ ] "Momentum theory helicopter rotor induced power" - verify disk area formula
- [ ] "Actuator disk theory" - confirm π·R² vs π·D²/4
- [ ] "Flapping flight Strouhal number" - typical range, how to apply
- [ ] "Pennycuick 1996 wingbeat frequency equation" - exact formula
- [ ] "Ellington 1984 induced power factor" - what is k_induced really?
- [ ] "Reynolds number insect flight" - typical values, effects on drag
- [ ] "Wing loading limits takeoff" - what's maximum for vertical launch?

#### 1.3 Resolve disk area question
**Key uncertainty:** Is swept area π·span² or π·span²/4?

**Research:**
- [ ] For helicopters: A = π·R² where R is blade radius (from center to tip)
- [ ] For flapping flight: What's the "effective radius"?
  - Single pair: span/2 (half-span is radius) → A = π·(span/2)² = π·span²/4
  - Dragonfly 4-wing: Do wings overlap? Independent disks? Additive?
- [ ] Check Ellington papers on multi-wing efficiency
- [ ] Check if Pennycuick addresses this

**Validation test:** Calculate induced power for known dragonfly, compare to measured values.

### Phase 2: Core Physics Equations (Clean Slate)

#### 2.1 Lift generation (MUST be satisfied for flight)
```
Steady flight: L = W
L = 0.5 × ρ × v² × S × CL

Minimum speed (stall):
v_min = sqrt(2·W / (ρ·S·CL_max))
```

**Variables:**
- W = weight (N)
- ρ = fluid density (kg/m³)
- v = flight speed (m/s)
- S = wing area (m²)
- CL = lift coefficient (dimensionless)
- CL_max ≈ 1.0-1.8 (depends on wing shape, feathers, Re)

**Critical:** Before claiming flight is possible, verify v > v_min AND CL < CL_max

#### 2.2 Power requirements (forward flight)

**Induced power** (from momentum theory):
```
P_ind = k_ind × W² / (2·ρ·A·v)
```
- A = disk area (need to resolve π·span² vs π·span²/4)
- k_ind ≈ 1.1-1.2 (induced power factor, accounts for non-uniform inflow)
- NOTE: This assumes actuator disk model - may not be accurate for flapping flight at low Re

**Profile power** (drag on wings):
```
P_pro = 0.5 × ρ × v³ × S × CD_pro
```
- CD_pro ≈ 0.01-0.05 (profile drag coefficient)
- Varies with Reynolds number (higher at low Re)

**Parasite power** (body drag):
```
P_par = 0.5 × ρ × v³ × S_body × CD_body
```
- S_body = cross-sectional area (frontal area)
- CD_body ≈ 0.1-0.4 (depends on streamlining)

**Inertial power** (accelerating wings each stroke):
```
P_inert = (2/π) × f × I_wing × A_beat²
```
- f = wingbeat frequency (Hz)
- I_wing = wing moment of inertia (kg·m²)
- A_beat = beat amplitude (radians)
- May be partially recovered via elastic storage (insects: ~50%)

**Total mechanical power:**
```
P_mech = P_ind + P_pro + P_par + P_inert
```

**Metabolic power:**
```
P_met = P_mech / η_muscle
```
- η_muscle ≈ 0.20-0.25 (muscle efficiency)

#### 2.3 Hovering power

Different model needed (no forward velocity):

**Induced power (momentum theory):**
```
v_induced = sqrt(W / (2·ρ·A))
P_ind = W × v_induced = sqrt(W³ / (2·ρ·A))
```

**Profile power:**
Sum over all wings:
```
P_pro = Σ(0.5 × ρ × v_tip³ × S_wing × CD_pro)
v_tip = 2 × f × A_beat × R_wing
```

**Key difference:** Induced power dominates hovering, scales as W^1.5

#### 2.4 Wingbeat frequency

**Pennycuick (1996) for birds:**
```
f = 3.87 × M^(-0.33)  [Hz, M in kg]
```
OR based on natural frequency:
```
f = (1/2π) × sqrt(k/I_wing)
```
where k is muscle/tendon stiffness

**For insects:** May use different scaling (asynchronous muscle)

**Power-limited frequency:**
```
P_inertial ∝ f × I × A²
f_max = sqrt(P_available / (I × A²)) / (2π)
```

#### 2.5 Strouhal number constraint

For efficient oscillatory propulsion:
```
St = f × A / v ≈ 0.2-0.4
```
- A = stroke amplitude (linear distance, meters)
- f = frequency (Hz)
- v = forward velocity (m/s)

**IMPORTANT:** A here is LINEAR amplitude, not angular
```
A_linear = 2 × R_wing × sin(A_angular / 2)  [for arc]
A_linear ≈ 2 × R_wing × A_angular  [for small angles]
```

### Phase 3: Implementation Strategy

#### 3.1 Separate concerns (break into functions)

**Proposed structure:**
```cpp
struct FlightPhysics {
    // Geometry
    float wing_area_m2;
    float wing_span_m;
    float disk_area_m2;  // Properly calculated
    float body_frontal_area_m2;

    // Kinematics
    float wingbeat_freq_Hz;
    float beat_amplitude_rad;
    float stroke_amplitude_m;  // Linear distance

    // Forces
    float weight_N;
    float available_power_W;
};

struct FlightRegime {
    float speed_m_s;
    float lift_coefficient;
    float power_required_W;
    bool is_valid;  // true if CL < CL_max and P < P_available
};

// Step 1: Calculate minimum speed (stall)
float CalculateStallSpeed(FlightPhysics const& p, Environment const& env);

// Step 2: Calculate power for a given speed
float CalculateForwardPower(FlightPhysics const& p, float speed_m_s, Environment const& env);

// Step 3: Find viable flight regime
FlightRegime FindViableForwardFlight(FlightPhysics const& p, Environment const& env);

// Step 4: Calculate hovering power
float CalculateHoveringPower(FlightPhysics const& p, Environment const& env);

// Step 5: Determine capabilities
Output_Aerial AnalyzeFlight(FlightPhysics const& p, Environment const& env);
```

#### 3.2 Validation at each step

**For each function:**
1. Write the function with proper physics
2. Unit test with known animal (dragonfly)
3. Check output against expected values
4. Add diagnostic warnings if values are suspicious
5. Don't proceed until that step validates

#### 3.3 Handle edge cases properly

**Reynolds number effects:**
- Re < 1,000: Very high drag, Stokes flow
- Re = 1,000-10,000: Insect regime, LEV effects important
- Re > 10,000: Standard aerodynamics

**Multi-wing configurations:**
- 2 wings (bilateral): Standard
- 4 wings (dragonfly): Efficiency < 2× due to interference
- 6+ wings (insects): Rare, need special handling

**Flight modes:**
- Forward flight: Speed-dependent power curve (U-shape)
- Hovering: High power, independent of forward speed
- Gliding: Zero power (not implemented yet)

### Phase 4: Refactoring Execution

#### 4.1 Create new file (clean start)
- [ ] `tonton_aerial_v2.cpp` - don't modify existing until validated
- [ ] Implement functions from Phase 3.1 one at a time
- [ ] Each function has unit test with dragonfly data

#### 4.2 Validation checklist (dragonfly)
- [ ] Stall speed: 2-4 m/s
- [ ] Cruise speed: 5-10 m/s
- [ ] Wingbeat frequency: 25-40 Hz
- [ ] Can sustain forward flight: YES
- [ ] Can hover: YES
- [ ] Power cost: 10-30 W/N (not 563 W/N!)
- [ ] Wing loading: 5-10 N/m²

#### 4.3 Validation checklist (scaling)
- [ ] 10× scaled dragonfly (1cm → 10cm wingspan):
  - Mass scales as L³ → 1000× heavier
  - Wing area scales as L² → 100× larger
  - Wing loading 10× higher → should struggle or fail
- [ ] 100× scaled dragonfly:
  - Should definitely fail (square-cube law)
  - Wing loading 100× higher
  - Power requirements scale differently than muscle capacity

#### 4.4 Replace old implementation
- [ ] Once v2 validates, replace `ComputeAerial` in `tonton_aerial.cpp`
- [ ] Keep old version in git history
- [ ] Update comments with paper citations

### Phase 5: Documentation

#### 5.1 Code comments
For each equation, add:
```cpp
// Induced power (momentum theory, Ellington 1984 Eq. 12)
// P_ind = k × W² / (2·ρ·A·v)
// Valid for Re > 10,000; underestimates for insects (LEV effects)
float induced_W = 1.15f * weight_N * weight_N /
                  (2.0f * rho * disk_area_m2 * speed_m_s);
```

#### 5.2 Bibliography additions
If new papers are found, add to `bibliography.md`:
- Full citation
- Key equations used
- Page numbers
- DOI/URL

#### 5.3 Validation notes
Document test cases in `aerial.md`:
- Animal tested
- Expected vs actual values
- Acceptable error range
- Date validated

## Investigation Progress

### Phase 1.1 - Bibliography Review: ✓ COMPLETED

**Key findings:**
1. **Pennycuick (1996)**: f = 3.87 × M^(-0.33) for birds
2. **Ellington (1984)**: k_ind = 1.18 (not 1.15!), disk area = πR²
3. **Van Den Berg & Rayner (1995)**: I_wing = 0.33 × m_wing × L²
4. **Tobalske (2007)**: Stroke amplitude 60° hover, 120° forward; muscle 15-35% body mass
5. **Taylor et al. (2003)**: Strouhal 0.2-0.4 (confirmed)

**Current code vs literature:**
- ✓ Disk area formula correct: π × span² / 4
- ✗ k_induced = 1.15 (should be 1.18)
- ✗ Dragonfly 4-wing: adds disk areas (likely ~2× too large)
- ? Beat amplitude: 1.05-2.44 rad (60°-140°) - needs validation
- ? Muscle power: 200-400 W/kg matches Ellington (1990)

### Phase 1.2 - Web Search: PARTIAL

**Confirmed:**
- Actuator disk: A = πR² (helicopter/rotor theory)
- Forward flight power: P_ind = k × W² / (2ρAv) from Glauert's work
- Momentum theory applies to flapping flight with corrections

**Found - Real dragonfly data:**
- Cruise speed: 10-15 m/s (36-54 km/h)
- Max speed: 15-25 m/s (54-90 km/h)
- Wingbeat frequency: 20-50 Hz typical (range 17-90 Hz)
- Takeoff acceleration: up to 20 m/s²

**Still needed:**
- Dragonfly-specific 4-wing tandem configuration data
- Measured dragonfly power requirements for validation

### Phase 1.3 - Debug Analysis: ✓ CRITICAL FINDINGS

**Actual values from code execution (dragonfly):**
```
amplitude_rad: 1.745 rad (100°)  ← Should be ~1.05 rad (60°) for cruise
frequency_Hz: 87.8 Hz            ← Should be 30-50 Hz
stroke_length_m: 0.179 m         ← Correct given wrong inputs
cruise_speed: 41.8 m/s          ← Should be 10-15 m/s
```

**ROOT CAUSES IDENTIFIED:**

1. **Amplitude formula wrong for forward flight** (line 141)
   ```cpp
   amplitude_rad = 1.05f + 1.39f * in.stability_vs_speed;
   // If stability_vs_speed = 0.5 → 100° (hovering mode!)
   // Should be 60° for cruise (stability_vs_speed = 0.0)
   ```

2. **Frequency formula is for BIRDS, not insects** (lines 136-138)
   ```cpp
   base_frequency_Hz = K * sqrt(g/L) * wing_loading^0.375
   // Pennycuick (1996): ONLY VALID FOR BIRDS
   // Insects use different scaling (not M^-0.33)
   ```

3. **Cascade effect:**
   - Wrong amplitude: 1.67× too large → stroke length 1.67× too long
   - Wrong frequency: 2.2× too high → speeds 2.2× too high
   - Combined error: 1.67 × 2.2 = **3.7× too high** ✓ Matches observation!

**The adjust_for_power_budget hack:**
- Reduces frequency to fit power budget
- But doesn't check if resulting speed can generate lift
- Allows impossible scaling (100× dragonfly still "flies")

## Open Questions to Resolve

### Q1: Disk area for dragonfly 4-wing configuration
**RESOLVED - Single bilateral pair:**
- Disk area = π × R² where R = span/2 (semi-span)
- Equivalent to: π × span² / 4 ✓
- Source: Ellington (1984), actuator disk theory
- **Induced power factor: k_ind = 1.18** (not 1.15 - Ellington mean for 8 insects)

**STILL OPEN - Dragonfly 4-wing:**
- Options:
  - A) Sum of two independent disks (fore + hind pairs) - current code does this
  - B) Overlapping disks with efficiency penalty
  - C) Single disk with span = max(fore_span, hind_span)
- No specific dragonfly tandem-wing paper found in bibliography
- **Conservative approach:** Use max(fore_span, hind_span) to avoid overestimating disk area
- **Current code issue:** Adds both disk areas, likely overestimates by ~50-100%

### Q2: LEV (Leading Edge Vortex) contribution
Low Reynolds number insects generate extra lift via LEV. How much?
- Dickinson et al. (1999): 2-3× quasi-steady lift
- But this is for specific conditions (high AoA, rapid rotation)
- When to apply? Always at Re < 10,000? Hovering only?

### Q3: Inertial power recovery
Insects store elastic energy in tendons/cuticle. How much is recovered?
- Estimates: 50-70% for some species
- Depends on anatomy (resonant frequency matching)
- Conservative: assume 0% recovery (worst case)
- Optimistic: 50% recovery

### Q4: Muscle power density
What's realistic for insects vs birds?
- Insects (asynchronous): 100-400 W/kg
- Birds (synchronous): 100-200 W/kg
- Current code: 200-400 W/kg (seems reasonable)
- But needs validation against measured values

### Q5: Metabolism efficiency
`in.metabolic_efficiency` - what does this represent?
- Muscle efficiency (mechanical/metabolic)? ~20-25%
- Or something else?
- Check how it's used elsewhere in codebase

## Red Flags to Watch For

During refactoring, stop and investigate if:
- [ ] Power costs > 100 W/N for any flyer
- [ ] Cruise speed > 30 m/s for insects
- [ ] Stall speed < 1 m/s for large birds
- [ ] Wingbeat frequency > 200 Hz (except tiny insects)
- [ ] Creature with wing loading > 80 N/m² can hover
- [ ] Lift coefficient required > 2.0 for sustained flight
- [ ] 10× scaled creature has same performance as 1× (should degrade)

## Success Criteria

The refactoring is complete when:
- [ ] Dragonfly validates (all 6 checks pass)
- [ ] Hummingbird validates
- [ ] Pigeon validates
- [ ] 10× scaled dragonfly shows realistic degradation
- [ ] 100× scaled dragonfly fails to fly
- [ ] No diagnostics warnings for valid flyers
- [ ] Clear diagnostics explain why non-flyers can't fly
- [ ] Code has citations for every equation
- [ ] All magic numbers replaced with named constants + comments

## IMMEDIATE ACTION PLAN (Priority Order)

### Priority 1: Fix Frequency Calculation (BLOCKS EVERYTHING)
**Problem:** Using Pennycuick (birds) for insects
**Fix:**
```cpp
// OLD (line 136-138):
base_frequency_Hz = K * sqrt(g/L) * pow(wing_loading, 0.375);

// NEW: For insects, use Ellington or empirical scaling
if (HasFlag(clade, CF::INSECTA)) {
    // Insects: f scales differently, typically 20-100 Hz range
    // Conservative: use measured range or allometric scaling
    base_frequency_Hz = 40.0f;  // Dragonfly typical
} else {
    // Birds: Pennycuick
    base_frequency_Hz = 3.87f * pow(body_mass_kg, -0.33f);
}
```
**Impact:** Fixes 2.2× speed error

### Priority 2: Fix Amplitude for Flight Mode
**Problem:** Using hovering amplitude for cruise flight
**Fix:**
```cpp
// OLD (line 141):
amplitude_rad = 1.05f + 1.39f * in.stability_vs_speed;

// NEW: Tobalske (2007): 60° cruise, 120° hover
float cruise_amplitude_rad = 1.05f;  // 60°
float hover_amplitude_rad = 2.09f;   // 120°
amplitude_rad = glm::mix(cruise_amplitude_rad, hover_amplitude_rad, in.stability_vs_speed);
```
**Impact:** Fixes 1.67× stroke length error

### Priority 3: Fix 4-Wing Disk Area
**Problem:** Adding overlapping disk areas (2× overestimate)
**Fix:**
```cpp
// OLD (lines 230-242): Sums all disk areas
// NEW: Use max span for multi-group configs
if (wing_params.num_groups > 1) {
    float max_span = 0.0f;
    auto groups = GetGaitGroupSpan(wings.data(), wings.size());
    for (const auto& group : groups) {
        max_span = std::max(max_span, group.total_span_m);
    }
    disk_area = M_PI * max_span * max_span / 4.0f;
} else {
    disk_area = M_PI * total_span_m * total_span_m / 4.0f;
}
```
**Impact:** Reduces power requirements by ~50% (disk area 2× too large → power 2× too high)

### Priority 4: Fix k_induced Constant
**Problem:** Using 1.15, should be 1.18
**Fix:** Change line 252: `float k_induced = 1.18f;`
**Impact:** Minor (~2.5% error)

### Priority 5: Add Lift = Weight Validation
**Problem:** No check if flight regime can generate lift
**Fix:** After calculating cruise speed, verify:
```cpp
float CL_required = (2.0f * weight_N) / (rho * v² * S);
float CL_max = 1.2f + 0.6f * in.feather_quality;
if (CL_required > CL_max) {
    // Cannot generate enough lift at this speed
    r.can_sustain_level_flight = false;
    diagnostics.warning("Speed too low, stall imminent");
}
```

### Priority 6: Remove adjust_for_power_budget Hack
**Problem:** Fudges frequency without checking lift validity
**Fix:** Replace with proper power-limited analysis:
- If power insufficient, mark as "can glide only" or "parachuting descent"
- Don't artificially reduce frequency and pretend flight works

## Combined Impact of Fixes 1-3:
- Frequency: 87.8 Hz → 40 Hz (2.2× reduction)
- Amplitude: 100° → 60° (1.67× reduction in stroke length)
- Disk area: 2× reduction in power requirements
- **Total speed correction:** 41.8 m/s → ~11-15 m/s ✓
- **Total power correction:** ~8× reduction in requirements

## Timeline / Priorities

**Critical path (do in order):**
1. Fix frequency calculation (Priority 1) - enables correct kinematics
2. Fix amplitude (Priority 2) - completes kinematic correction
3. Fix disk area (Priority 3) - corrects power model
4. Add lift validation (Priority 5) - prevents invalid results
5. Validate dragonfly - confirms fixes work
6. Test scaling behavior - confirms no hidden hacks remain

**Lower priority:**
- k_induced correction (Priority 4) - minor improvement
- Remove power budget hack (Priority 6) - cleanup, doesn't block validation
- LEV effects (can approximate conservatively)
- Inertial power recovery (assume worst case)
- Specialized modes (soaring, gliding)

---

**Notes:**
- This document is a living plan - update as new information is found
- Add test results inline as validation proceeds
- Keep old broken code in git history, don't delete
- Every equation change should cite a source or explain reasoning
