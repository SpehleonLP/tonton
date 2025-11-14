# Clade-Based vs. Locomotion-Mode Analysis: Unification Strategy

## Overview

The old system (src/Rules/old/) organized knowledge by **taxonomic clade** (arthropods, birds, fish, mammals, mollusks, herpatology).
The new system (src/Rules/) organizes by **locomotion mode** (aerial, terrestrial, aquatic, serpentine, climbing).

**Key insight:** Both approaches are valuable and complementary. Clade-specific knowledge should **refine and constrain** the general locomotion models.

---

## Current State

**What's already implemented:**
- `Output_Physical.clade` field populated from bone name analysis (CladeFlags from tonton_wordlist.h)
- Available clade flags: CHORDATA, AMPHIBIA, REPTILIA, CHELONIA, AVES, MAMMALIA, UNGULATA, EQUIDAE, CETACEA, PISCES, ARTHROPODA, INSECTA, ARACHNIDA, CRUSTACEA, MOLLUSCA, CEPHALOPODA
- Locomotion modes work independently of clade (good generalization)

**What's missing:**
- Clade-specific refinements to locomotion models
- Physiological constraints unique to clades
- Metabolic scaling differences between clades

---

## Priority 1: Metabolic Scaling (CRITICAL)

### Problem
Current implementation uses generic metabolic scaling. Different clades have **fundamentally different** metabolic rates and scaling exponents.

### Old system knowledge:

**Arthropods** (metrics_arthropod.cpp:17-21):
```cpp
// RMR = 4.14M^0.66 (R² = 0.90, Addo-Bediako et al. 2002)
// FMR = 35.08M^1.10 for flying insects (R² = 0.95, Niven & Scharlemann 2005)
```

**Fish** (metrics_icthyology.cpp:449-459):
```cpp
// Sustained red muscle: ~200 W/kg (Rome et al. 1988)
// Peak white muscle: 250–500 W/kg (Wardle 1975)
```

**Birds** (metrics_birdstrouhal.cpp:44-48):
```cpp
// Muscle power density: 200-400 W/kg (Ellington et al. 1990)
// Most birds operate at 200-300 W/kg sustainable
```

### Recommendation:
**Add clade-specific metabolic functions in `src/Rules/tonton_metabolic.cpp` (NEW FILE):**

```cpp
namespace TonTon {

struct MetabolicScaling {
    float rmr_coefficient;    // a in RMR = a*M^b
    float rmr_exponent;       // b in RMR = a*M^b
    float mmr_coefficient;    // max metabolic rate
    float mmr_exponent;
    float muscle_power_W_kg;  // sustainable muscle power density
};

MetabolicScaling GetMetabolicScaling(CladeFlags clade) {
    // ARTHROPODA: lower basal rate, lower muscle power
    if (HasFlag(clade, CladeFlags::ARTHROPODA)) {
        return {
            .rmr_coefficient = 4.14f,
            .rmr_exponent = 0.66f,
            .mmr_coefficient = HasFlag(clade, CladeFlags::INSECTA) ? 35.08f : 33.0f,
            .mmr_exponent = 1.10f,
            .muscle_power_W_kg = 100.0f  // Lower than vertebrates
        };
    }

    // AVES: highest metabolic rates
    if (HasFlag(clade, CladeFlags::AVES)) {
        return {
            .rmr_coefficient = 6.25f,  // ~1.5x mammal rate
            .rmr_exponent = 0.72f,
            .mmr_coefficient = 78.0f,
            .mmr_exponent = 0.72f,
            .muscle_power_W_kg = 250.0f  // High-performance flight muscle
        };
    }

    // MAMMALIA: standard endotherm
    if (HasFlag(clade, CladeFlags::MAMMALIA)) {
        return {
            .rmr_coefficient = 4.18f,  // Kleiber's law base
            .rmr_exponent = 0.75f,
            .mmr_coefficient = 33.5f,
            .mmr_exponent = 0.75f,
            .muscle_power_W_kg = 200.0f
        };
    }

    // PISCES: ectotherm with good muscle performance
    if (HasFlag(clade, CladeFlags::PISCES)) {
        return {
            .rmr_coefficient = 0.8f,  // Low ectotherm basal
            .rmr_exponent = 0.80f,
            .mmr_coefficient = 6.5f,
            .mmr_exponent = 0.80f,
            .muscle_power_W_kg = 200.0f  // Red muscle, 400 W/kg white burst
        };
    }

    // REPTILIA/AMPHIBIA: lower ectotherm rates
    if (HasFlag(clade, CladeFlags::REPTILIA) || HasFlag(clade, CladeFlags::AMPHIBIA)) {
        return {
            .rmr_coefficient = 0.5f,
            .rmr_exponent = 0.80f,
            .mmr_coefficient = 4.0f,
            .mmr_exponent = 0.80f,
            .muscle_power_W_kg = 150.0f
        };
    }

    // MOLLUSCA: very low metabolic rate
    if (HasFlag(clade, CladeFlags::MOLLUSCA)) {
        return {
            .rmr_coefficient = 0.3f,
            .rmr_exponent = 0.75f,
            .mmr_coefficient = 2.0f,
            .mmr_exponent = 0.75f,
            .muscle_power_W_kg = 100.0f
        };
    }

    // Default: generic ectotherm
    return {
        .rmr_coefficient = 1.0f,
        .rmr_exponent = 0.75f,
        .mmr_coefficient = 8.0f,
        .mmr_exponent = 0.75f,
        .muscle_power_W_kg = 150.0f
    };
}

} // namespace TonTon
```

**Impact:** This would immediately improve accuracy of all speed/endurance predictions by 20-300% for different clades.

---

## Priority 2: Arthropod-Specific Constraints

### Problem
Arthropods have unique physiological limits not captured in current locomotion models.

### Missing constraints from metrics_arthropod.cpp:

**1. Oxygen Diffusion Limits** (lines 98-111):
```cpp
// Tracheal system scales POORLY with size (Kaiser et al. 2007)
// criticalPO2 = basePO2 * M^0.15 * sqrt(MMR/RMR)
// Clamp: 5-15 kPa
```

**2. Size Limits** (lines 128-146):
```cpp
// Square-cube law crush depth
// maxSize = (compressiveStrength / safetyFactor) / (density * gravity)
// Exoskeleton Young's modulus: 1-20 GPa
```

**3. Thermal Effects** (lines 46-75):
```cpp
// Q10 = 2-3 (temperature doubles reaction rates per 10K)
// Smaller arthropods have HIGHER Q10 due to thermal inertia
```

**4. Jumping Performance** (lines 300-329):
```cpp
// Elastic energy storage in cuticle (Burrows 2006)
// jumpRange = v²/g where v = sqrt(2*elasticEnergy/mass)
// Elastic modulus determines stored energy
```

### Recommendation:
**Modify `src/Rules/tonton_terrestrial.cpp` ComputeJumping():**

```cpp
// AFTER line 117: if(body_mass_kg < 0.1f)
if (HasFlag(s.physical.clade, CladeFlags::ARTHROPODA)) {
    // Arthropods use elastic catapult mechanism (Burrows 2006)
    mechanism = Output_Jumping::MechanismType::ELASTIC_CATAPULT;

    // Cuticle elastic modulus from exoskeleton quality
    float cuticle_modulus_Pa = glm::mix(1e9f, 20e9f, in.structure_vs_weight);

    // Elastic energy = 0.5 * k * x² where k = EA/L
    float leg_cross_section = avg_leg_length_m * avg_leg_length_m * 0.01f;
    float leg_stiffness = (cuticle_modulus_Pa * leg_cross_section) / avg_leg_length_m;
    float extension_distance = avg_leg_length_m * 2.0f; // Can extend 2x

    float elastic_energy_J = 0.5f * leg_stiffness * extension_distance * extension_distance;
    takeoff_velocity_m_s = std::sqrt(2.0f * elastic_energy_J / body_mass_kg);
    takeoff_velocity_m_s = std::min(takeoff_velocity_m_s, 5.0f); // Cap at 5 m/s
}
```

**Add environmental limits check in `src/Rules/tonton_specialized.cpp`:**

```cpp
std::optional<Output_EnvironmentalLimits> ComputeEnvironmentalLimits(Input const& in, Scratch & s) {
    Output_EnvironmentalLimits limits;

    if (HasFlag(s.physical.clade, CladeFlags::ARTHROPODA)) {
        // Oxygen diffusion limit (Harrison et al. 2006, Kaiser et al. 2007)
        float criticalPO2_Pa = 6000.0f * std::pow(s.physical.body_mass_kg * 1000.0f, 0.15f);
        limits.min_O2_pressure_Pa = std::clamp(criticalPO2_Pa, 5000.0f, 15000.0f);

        // Size limit from square-cube law (Haldane 1926)
        float exoskeleton_strength_Pa = 100e6f * in.structure_vs_weight;  // 10-100 MPa
        float max_size_m = exoskeleton_strength_Pa / (1150.0f * in.environment.gravity_m_s2 * 3.0f);
        limits.max_viable_size_m = std::clamp(max_size_m, 0.001f, 0.5f);

        // Temperature effects (Q10 = 2-3)
        float temp_diff_K = std::abs(in.environment.temperature_K - 298.15f);
        limits.performance_multiplier = std::pow(2.5f, temp_diff_K / 10.0f);
    }

    // Similar blocks for other clades...

    return limits;
}
```

---

## Priority 3: Bird-Specific Flight Refinements

### Problem
Current aerial implementation is generic. Birds have specific constraints from Van Den Berg & Rayner (1995) and Pennycuick (1996).

### Missing from metrics_birdstrouhal.cpp:

**1. Wing Inertia** (lines 33-36):
```cpp
// I = 0.33 × m_wing × L_wing² (R² = 0.997)
// Inertial power = 11-15% of total flight power
```

**2. Pennycuick Frequency** (lines 38-41):
```cpp
// f = 3.87 × M^(-0.33) [universal scaling]
// Power-limited: f = (2P/IA²)^(1/3)
```

**3. Hovering Threshold** (lines 129-142):
```cpp
// Wing loading < 80 N/m² for sustained hovering
// Requires power_margin > 2.0 and frequency > 50 Hz
```

### Recommendation:
**Enhance `src/Rules/tonton_aerial.cpp` ComputeAerial():**

```cpp
// AFTER line 104: if (!has_wings) return {};

// BIRD-SPECIFIC REFINEMENTS
if (HasFlag(sk_memo->GetSemanticFlags()[wing_root], SF::AVIAN) ||
    HasFlag(s.physical.clade, CladeFlags::AVES)) {

    // Pennycuick universal scaling (1996)
    float pennycuick_freq_Hz = 3.87f * std::pow(body_mass_kg, -0.33f);

    // Wing moment of inertia (Van Den Berg & Rayner 1995)
    float wing_span = wing_length * 2.0f;
    float wing_inertia = 0.33f * wing_mass * wing_span * wing_span;

    // Power-limited frequency
    float omega = 2.0f * M_PI * flap_frequency_Hz;
    float inertial_power_W = 0.5f * wing_inertia * omega * omega * omega *
                             stroke_amplitude * stroke_amplitude;

    // Inertial power should be 11-15% of total (Van Den Berg & Rayner)
    if (inertial_power_W / flight_power_W > 0.15f) {
        // Wings too heavy/fast - reduce frequency
        flap_frequency_Hz *= std::sqrt(0.12f / (inertial_power_W / flight_power_W));
    }

    // Apply Pennycuick constraint
    flap_frequency_Hz = std::min(flap_frequency_Hz, pennycuick_freq_Hz);

    // Hovering capability (Altshuler et al. 2004)
    float wing_loading_N_m2 = weight_N / total_wing_area;
    if (wing_loading_N_m2 < 80.0f && power_margin > 2.0f && flap_frequency_Hz > 50.0f) {
        output.can_hover = true;
        output.hovering_power_W = flight_power_W * 1.5f; // 50% more for hover
    }
}
```

---

## Priority 4: Fish-Specific Aquatic Refinements

### Problem
Current aquatic implementation doesn't distinguish between fish, cephalopods, and marine mammals.

### Missing from metrics_icthyology.cpp:

**1. Caudal Aspect Ratio Relationship** (lines 208-226):
```cpp
// Sambilay (1990): V = 0.59 * exp(0.42 * AR)
// Speed in body lengths per second
```

**2. Muscle Fiber Types** (lines 449-459):
```cpp
// Red muscle (sustained): 200 W/kg
// White muscle (burst): 250-500 W/kg
```

**3. Swim Bladder** (lines 342-356):
```cpp
// Adjustment time = 30 * M^0.33 seconds
// Depth range limited if has swim bladder
```

**4. C-Start Escape** (already implemented! ✓)

### Recommendation:
**Refine `src/Rules/tonton_aquatic.cpp`:**

```cpp
// AFTER line 93: analysis.physiology.body_mass_kg

if (HasFlag(s.physical.clade, CladeFlags::PISCES)) {
    // Fish-specific: caudal aspect ratio → speed (Sambilay 1990)
    // Already using similar logic, but can refine:

    // Muscle fiber type affects burst capability
    float white_muscle_fraction = glm::mix(0.3f, 0.7f, 1.0f - in.behavior.endurance_vs_power);
    float burst_power_W = s.metabolic.muscle_mass_kg *
                         (white_muscle_fraction * 400.0f +  // White: 400 W/kg
                          (1.0f - white_muscle_fraction) * 200.0f); // Red: 200 W/kg

    // Burst speed = sustained * sqrt(burst_power / sustained_power)
    burst_speed_m_s = cruise_speed_m_s * std::sqrt(burst_power_W / swim_power_W);
    burst_speed_m_s = std::min(burst_speed_m_s, cruise_speed_m_s * 3.5f); // Cap at 3.5x
}

// CETACEA: air-breathing marine mammals
if (HasFlag(s.physical.clade, CladeFlags::CETACEA)) {
    has_swim_bladder = false;  // Mammals don't have swim bladders
    requires_constant_motion = false; // But can control buoyancy via lungs

    // Cetaceans are neutrally buoyant through lung volume control
    neutral_buoyancy_density = body_density_kg_m3;

    // Depth diving capability from body structure
    crush_depth_m *= 5.0f; // Cetaceans can go much deeper than fish

    // Must surface to breathe
    float lung_volume_L = body_volume_m3 * 0.05f * 1000.0f; // ~5% body volume
    float O2_consumption_L_per_min = s.metabolic.max_rate_W * 0.05f; // ~0.05 L/W/min
    float max_dive_time_min = lung_volume_L / O2_consumption_L_per_min;

    output.max_dive_duration_s = max_dive_time_min * 60.0f;
    output.must_surface_for_air = true;
}
```

---

## Priority 5: Clade-Specific Behavior Hints

### Problem
Current behavior system doesn't use clade knowledge for AI archetype selection.

### Old system patterns:
- **Arthropods**: Territory size ∝ M^1.37, predator-prey ratio M^0.8
- **Birds**: Formation flying from aspect ratio, mobbing behavior from sociality
- **Fish**: Schooling from eye size + body plan

### Recommendation:
**Add clade refinements to `src/Rules/tonton_behavior.cpp`:**

```cpp
Output_Behavior ComputeBehavior(Input const& in, Scratch & s) {
    // ... existing behavior computation ...

    // CLADE-SPECIFIC REFINEMENTS

    if (HasFlag(s.physical.clade, CladeFlags::ARTHROPODA)) {
        // Territory size scaling (Schoener 1968)
        output.territory_radius_m = 100.0f * std::pow(s.physical.body_mass_kg, 1.37f);

        // Arthropods have fast reaction times but limited learning
        output.reaction_time_s = 0.05f + s.physical.body_length_m * 0.02f;
        output.learning_capacity *= 0.5f; // Lower than vertebrates
    }

    if (HasFlag(s.physical.clade, CladeFlags::AVES)) {
        // Birds: formation flying if high aspect ratio
        if (s.aerial.has_value() && s.aerial->aspect_ratio > 8.0f) {
            output.formation_flying_tendency = 0.8f;
        }

        // Mobbing behavior for small social birds
        if (s.physical.body_mass_kg < 0.5f && output.sociability > 0.6f) {
            output.mob_behavior_tendency = 0.8f;
        }
    }

    if (HasFlag(s.physical.clade, CladeFlags::PISCES)) {
        // Fish schooling tendency from eye size (Kotrschal et al. 1998)
        if (s.sensory.has_value()) {
            float eye_score = s.sensory->eye_diameter_m / s.physical.body_length_m;
            output.schooling_tendency = std::clamp(eye_score * 20.0f, 0.0f, 1.0f);
        }
    }

    if (HasFlag(s.physical.clade, CladeFlags::REPTILIA)) {
        // Reptiles: sit-and-wait predators if large
        if (s.physical.body_mass_kg > 10.0f) {
            output.ambush_vs_pursuit = 0.8f; // High ambush tendency
            output.patience_factor = 0.9f;
        }
    }

    return output;
}
```

---

## Implementation Strategy

### Phase 1: Metabolic Foundation (Week 1)
1. Create `src/Rules/tonton_metabolic.cpp` with clade-specific scaling functions
2. Integrate into `tonton_scratch.cpp` metabolic calculations
3. Update all locomotion rules to use clade-corrected metabolic values
4. **Impact:** Immediate 20-300% accuracy improvement in speed/endurance

### Phase 2: Arthropod Constraints (Week 2)
1. Add environmental limits structure to `tonton_output.h`
2. Implement arthropod-specific constraints (O2, size, temperature)
3. Add elastic catapult jumping mechanism for insects
4. **Impact:** Arthropods become physically realistic (currently broken for small sizes)

### Phase 3: Avian Refinements (Week 3)
1. Add wing inertia calculations to aerial rules
2. Implement Pennycuick frequency constraints
3. Add hovering capability detection
4. **Impact:** Bird flight becomes quantitatively accurate vs. real data

### Phase 4: Aquatic Clade Specialization (Week 4)
1. Add muscle fiber type ratios for fish
2. Implement cetacean diving/surfacing mechanics
3. Refine swim bladder vs. lung buoyancy control
4. **Impact:** Marine mammals become distinct from fish

### Phase 5: Behavioral Integration (Week 5)
1. Add clade-specific behavior modifiers
2. Implement territory size scaling laws
3. Add social behavior patterns (schooling, mobbing, formation flying)
4. **Impact:** AI archetypes become clade-appropriate

---

## Testing Strategy

### Validation Cases
For each clade, test against known real-world examples:

**Arthropods:**
- Dragonfly (already in README): body length 24mm, wingspan 60mm, mass ~0.5g
- Expected: RMR ~0.02 W, max speed ~10 m/s, jump height ~0.1m

**Birds:**
- Hummingbird: mass 3g, wingspan 10cm, wingbeat 50 Hz
- Expected: can hover, wing loading ~60 N/m², power margin 2.5x

**Fish:**
- Tuna: body length 1m, mass 40kg, cruise speed 2 m/s (2 BL/s)
- Expected: caudal AR ~5, burst 7 m/s, no swim bladder

**Mammals:**
- Dolphin: body length 2m, mass 150kg, cruise speed 4 m/s
- Expected: must surface every 5-10 minutes, burst speed 12 m/s

---

## Code Organization

```
src/Rules/
├── tonton_metabolic.cpp        [NEW] Clade-specific metabolic scaling
├── tonton_environmental.cpp    [NEW] Environmental constraints by clade
├── tonton_aerial.cpp           [MODIFY] Add avian-specific refinements
├── tonton_aquatic.cpp          [MODIFY] Add fish/cetacean distinctions
├── tonton_terrestrial.cpp      [MODIFY] Add arthropod elastic jumping
├── tonton_behavior.cpp         [MODIFY] Add clade-specific behaviors
└── tonton_specialized.cpp      [MODIFY] Add clade-specific limits
```

Keep old rules as **reference documentation** in `src/Rules/old/` - they contain valuable citations and research context.

---

## Bibliography Integration

The old rules have EXTENSIVE citations (50+ papers per clade). These should be:
1. Preserved in comments near relevant code
2. Added to `bibliography.md`
3. Cited in code comments when using clade-specific formulas

Example:
```cpp
// Arthropod metabolic rate scaling (Addo-Bediako et al. 2002; Chown et al. 2007)
// RMR = 4.14M^0.66 (R² = 0.90, F₁,₅₉ = 544.9, p < 0.0001)
float rmr_W = 4.14f * std::pow(body_mass_kg, 0.66f);
```

---

## Conclusion

**The locomotion-mode organization is CORRECT and should be kept.** It provides good generalization and code reuse.

**Clade-specific knowledge should be layered ON TOP as refinements:**
- Use clade flags to select appropriate constants/exponents
- Apply clade-specific constraints after general calculations
- Add clade-specific behaviors as modifiers

**Priority order:**
1. **Metabolic scaling** (biggest impact, easiest implementation)
2. **Arthropod constraints** (currently broken for insects)
3. **Avian refinements** (birds are most studied, best data)
4. **Aquatic specialization** (fish vs. cetaceans)
5. **Behavioral patterns** (polish for AI)

This approach gets the best of both worlds: general locomotion physics with clade-specific biological realism.
