#include "tonton_aerial.h"
#include "../../include/tonton_skinnedmesh.h"
#include "../../include/tonton_input.h"
#include "../../include/tonton_builder.h"
#include "Rules/tonton_scratch.h"
#include <set>
#include <cmath>
#include <algorithm>

namespace TonTon
{
struct GaitGroupSpan
{
	uint16_t gaitGroup;
	uint16_t count;
	length_m total_span_m;
};

static std::vector<Analysis_Aerial::Wing> GetWings(Input const& in, bool &likely_feathers);
std::vector<GaitGroupSpan> GetGaitGroupSpan(Analysis_Aerial::Wing * data, size_t size);
// Forward declaration for GetGaitGroupCenters
}

std::optional<TonTon::Analysis_Aerial> TonTon::ComputeAerial(Input const& in, Scratch& s)
{
    TonTon::Analysis_Aerial r;

	bool likely_feathers = false;
    auto wings = shared_array<TonTon::Analysis_Aerial::Wing>::FromArray(GetWings(in, likely_feathers));
    r.wings = wings;

    if (r.wings.empty()) {
        return {}; // No wings, no aerial capability
    }

    // ============================================================================
    // WING GEOMETRY AGGREGATION
    // ============================================================================

    area_m2 total_wing_area_m2 = 0.0f;
    length_m total_wing_span_m = 0.0f;
    inertia_kgm2 wing_inertia_kg_m2 = 0.0f;
    length_m mean_chord_m = 0.0f;

    for (const auto& wing : r.wings) {
        total_wing_area_m2 += wing.wing_area_m2;
        total_wing_span_m += wing.span_m;
        wing_inertia_kg_m2 += wing.wing_inertia_kgm2;
        mean_chord_m += wing.chord_m;
    }

    mean_chord_m /= r.wings.size();
    // Wingspan = max tip-to-tip across any symmetrical pair
    // Each wing's span_m is one side, so a pair spans 2 * max(span_m)
    {
        length_m max_single_wing_span = 0.0f;
        for (const auto& wing : r.wings) {
            max_single_wing_span = std::max(max_single_wing_span, wing.span_m);
        }
        total_wing_span_m = max_single_wing_span * 2.0f;
    }
    wing_inertia_kg_m2 /= r.wings.size();

    r.wing_span_m = total_wing_span_m;
    r.wing_area_m2 = total_wing_area_m2;

    // Apply magical enhancement
    total_wing_span_m  *= std::exp2(in.mana.air);
    total_wing_area_m2 *= std::exp2(in.mana.air);

    // ============================================================================
    // ENVIRONMENT PARAMETERS
    // ============================================================================

    const auto rho = in.environment.fluidDensity_Kg_m3;
    const auto mu = in.environment.fluidViscosity_Pa_s;
    const auto real_g = in.environment.gravity_m_s2;
    // Effective gravity accounting for buoyancy (0 for neutrally buoyant)
    const auto g_effective = real_g * std::max(0.0f, 1.0f - float(rho / in.body_density()));
    const auto body_mass_kg = in.body_mass_kg();
    const auto weight_N = std::max<force_N>(0.001, in.body_weight_N());
    // Weight that lift must actually support, reduced by buoyancy (0 when neutrally buoyant).
    const auto effective_weight_N = std::max<force_N>(0.001f, body_mass_kg * g_effective);

    // ============================================================================
    // PURE PHYSICS LAMBDAS
    // These encode fundamental physical relationships with no biological reasoning
    // ============================================================================

    // --- FLUID DYNAMICS ---

    // Reynolds number: ratio of inertial to viscous forces
    // Re = ρvL/μ (dimensionless)
    auto reynolds_number = [](density_kg_m3 rho, velocity_m_s v, length_m L,
                              viscosity_Pa_s mu) -> float {
        return float((rho * v * L) / mu);
    };

    // --- AERODYNAMIC FORCES ---

    // Stall speed from lift equation: L = 0.5ρv²SCL → v = √(2W/ρSCL)
    auto stall_speed = [](force_N weight, density_kg_m3 rho, area_m2 S,
                          float CL_max) -> velocity_m_s {
        return sqrt((2.0f * weight) / (rho * S * CL_max));
    };

    // Wing loading: weight per unit wing area (returns N/m² = pressure)
    auto wing_loading = [](force_N weight, area_m2 S) -> load_N_m2 {
        return weight / S;
    };

    // --- OSCILLATORY PROPULSION ---

    // Strouhal velocity: v = f×A_pp×L/St (universal for efficient oscillation)
    // St = f·A_pp/U uses PEAK-TO-PEAK amplitude (factor 2 over single-sided).
    auto strouhal_velocity = [](freq_Hz f, angle_rad amplitude, length_m wing_length,
                                float strouhal) -> velocity_m_s {
        return (f * (2.0f * amplitude) * wing_length) / strouhal;
    };

    // Power-limited frequency: P = coef×f³×A² → f = ∛(P/(coef×A²))
    auto power_limited_frequency = [](power_W P, auto coef,
                                      angle_rad amplitude) -> freq_Hz {
        return cbrt(P / (coef * amplitude * amplitude));
    };

    // --- ADDED MASS (potential flow theory) ---

    // Added mass coefficient for oscillating plate: k = (π/4)×(1 + 2/AR)
    auto added_mass_coef = [](float aspect_ratio) -> float {
        return (M_PI / 4.0f) * (1.0f + 2.0f / std::max(1.0f, aspect_ratio));
    };

    // Added inertia: I = k×ρ×c²×b³/3 (from potential flow integration)
    auto added_inertia = [](float k, density_kg_m3 rho, length_m chord,
                            length_m span) -> inertia_kgm2 {
        return k * rho * (chord * chord) * pow3(span) / 3.0f;
    };

    // --- POWER REQUIREMENTS ---

    // Induced power (momentum theory): P = kW²/(2ρAv)
    auto induced_power = [](float k, force_N weight, density_kg_m3 rho,
                            area_m2 disk_area, velocity_m_s v) -> power_W {
        return k * weight * weight / (2.0f * rho * disk_area * v);
    };

    // Hover induced power: P = W×√(W/(2ρA))
    auto hover_induced_power = [](force_N weight, density_kg_m3 rho,
                                  area_m2 disk_area) -> power_W {
        auto v_induced = sqrt(weight / (2.0f * rho * disk_area));
        return weight * v_induced;
    };

    // Profile drag power: P = 0.5×ρ×v³×S×CD
    auto profile_power = [](density_kg_m3 rho, velocity_m_s v, area_m2 S,
                            float CD) -> power_W {
        return 0.5f * rho * pow3(v) * S * CD;
    };

    // Inertial power: P = 8π²×I×f³×A² (with elastic recovery factor)
    auto inertial_power = [](float elastic_recovery, inertia_kgm2 I, freq_Hz f,
                             angle_rad amplitude) -> power_W {
        return elastic_recovery * 8.0f * M_PI * M_PI * I * pow3(f) * amplitude * amplitude;
    };

    // Profile power coefficient: k = 0.5×ρ×(2π)³×A×R³×S×CD
    auto profile_power_coef = [](density_kg_m3 rho, angle_rad amplitude,
                                 length_m wing_length, area_m2 S, float CD) {
        constexpr float two_pi_cubed = (2.0f * M_PI) * (2.0f * M_PI) * (2.0f * M_PI);
        return 0.5f * rho * two_pi_cubed * amplitude * pow3(wing_length) * S * CD;
    };

    // --- TURNING DYNAMICS ---

    // Aerial turn radius: r = v²/(g×√(n²-1)) where n = load factor
    auto aerial_turn_radius = [](velocity_m_s v, acceleration_m_s2 g,
                                 float load_factor) -> length_m {
        float n2 = load_factor * load_factor - 1.0f;
        if (n2 <= 1e-4f || float(g) <= 0.0f) return length_m(1e6f); // ~straight line
        return (v * v) / (g * std::sqrt(n2));
    };

    // ============================================================================
    // REGIME TRANSITION LAMBDAS
    // These encode how aerodynamic behavior changes with Reynolds number
    // ============================================================================

    // Regime blend: smoothstep transition from insect (Re~5000) to bird (Re~50000)
    auto compute_regime_blend = [](float Re) -> float {
        auto log_Re = std::log10(std::max(100.0f, Re));
        auto blend = std::clamp((log_Re - 3.7f) / 1.3f, 0.0f, 1.0f);
        return blend * blend * (3.0f - 2.0f * blend);  // smoothstep
    };

    // ============================================================================
    // ALLOMETRIC SCALING LAMBDAS
    // These encode biological scaling relationships (empirical fits)
    // ============================================================================

    // Pennycuick (1996) full allometric form:
    //   f = K · m^(3/8) · g^(1/2) · b^(−23/24) · S^(−1/3) · ρ^(−3/8)
    // Restores span/area exponents and the air-density dependence (matters for thin/dense atmospheres).
    // Valid for birds (high Re regime).
    auto pennycuick_frequency = [](float K, mass_kg m, acceleration_m_s2 g,
                                   length_m wingspan, area_m2 wing_area,
                                   density_kg_m3 rho) -> freq_Hz {
        return K
             * std::pow(float(m), 0.375f)
             * std::sqrt(float(g))
             * std::pow(float(wingspan), -23.0f/24.0f)
             * std::pow(float(wing_area), -1.0f/3.0f)
             * std::pow(float(rho), -0.375f);
    };

    // Insect allometric frequency: f = 80×(m/0.001)^-0.24 Hz
    // Empirical fit across insect taxa
    auto insect_allometric_frequency = [](mass_kg m) -> freq_Hz {
        return 80.0f * std::pow(float(m) / 0.001f, -0.24f);
    };

    // ============================================================================
    // WING LOADING (pure physics)
    // ============================================================================

    r.wing_loading_N_m2 = wing_loading(weight_N, total_wing_area_m2);

    // ============================================================================
    // REGIME DETERMINATION (iterative solver)
    // Couples: Re ↔ regime ↔ frequency ↔ velocity
    // ============================================================================

    // Morphology-dependent constants
    auto CL_max_bird = 1.2f + 0.6f * in.feather_quality;
    auto CL_max_insect = 2.0f + 1.0f * in.feather_quality;
    // Pennycuick (1996) SI constant K ≈ 1.08, modulated by scaling strategy.
    auto K_pennycuick = 1.08f * (0.8f + 0.4f * in.scaling_strategy);

    // Blend between insect and bird frequency models based on regime
    auto compute_base_frequency = [&](float regime) -> freq_Hz {
        freq_Hz insect_freq = insect_allometric_frequency(body_mass_kg);
        freq_Hz bird_freq = pennycuick_frequency(K_pennycuick, body_mass_kg, real_g,
                                                  total_wing_span_m, total_wing_area_m2, rho);
        return glm::mix(float(insect_freq), float(bird_freq), regime);
    };

    // Strouhal number varies with regime: 0.45 (insects) → 0.25 (birds)
    auto regime_strouhal = [](float regime) -> float {
        return glm::mix(0.45f, 0.25f, regime);
    };

    // Velocity from frequency via Strouhal constraint
    auto compute_strouhal_velocity = [&](freq_Hz f, float regime) -> velocity_m_s {
        angle_rad cruise_amplitude = 1.05f;  // ~60° single-sided (strouhal_velocity doubles for peak-to-peak)
        return strouhal_velocity(f, cruise_amplitude, total_wing_span_m / 2.0f,
                                 regime_strouhal(regime));
    };

    // Initial velocity estimate (body-size based, avoids weight=0 issues)
    velocity_m_s v_estimate = velocity_m_s(std::sqrt(float(body_mass_kg)) * 0.5f);
    v_estimate = std::max(v_estimate, velocity_m_s(0.1f));

    // Iterative solve for self-consistent regime
    float regime_blend = 0.0f;
    freq_Hz base_frequency_Hz = insect_allometric_frequency(body_mass_kg);

    for (int iter = 0; iter < 10; ++iter) {
        float Re = reynolds_number(rho, v_estimate, mean_chord_m, mu);
        float new_regime = compute_regime_blend(Re);
        freq_Hz new_base_freq = compute_base_frequency(new_regime);
        velocity_m_s new_v = compute_strouhal_velocity(new_base_freq, new_regime);

        float v_change = std::abs(float(new_v - v_estimate)) / std::max(0.01f, float(v_estimate));

        regime_blend = new_regime;
        base_frequency_Hz = new_base_freq;
        v_estimate = new_v;

        if (v_change < 0.01f) break;
    }

    // ============================================================================
    // STALL SPEED (from converged regime)
    // ============================================================================

    auto CL_max = glm::mix(CL_max_insect, CL_max_bird, regime_blend);
    r.min_flight_speed_m_s = stall_speed(effective_weight_N, rho, total_wing_area_m2, CL_max);

    // ============================================================================
    // BEAT AMPLITUDE (morphological adaptation)
    // ============================================================================

    // Single-sided amplitudes — Cruise: ~60° for efficiency, Hover: ~140° for lift generation
    // (doubled to peak-to-peak where Strouhal / stroke-length require it)
    angle_rad cruise_amplitude_rad = 1.05f;
    angle_rad hover_amplitude_rad = 2.44f;
    angle_rad base_beat_amplitude_rad = glm::mix(cruise_amplitude_rad, hover_amplitude_rad,
                                                  in.stability_vs_speed);

    int noWingGroups = 1;
    for (auto& wing : wings) {
        wing.beat_amplitude_rad = base_beat_amplitude_rad;
        wing.stroke_plane_angle_rad = (1.0f - in.stability_vs_speed) * (M_PI / 4.0f);
    }

    // Multi-wing coordination (dragonflies)
    if (r.wings.size() > 2) {
        std::set<int32_t> gait_groups;
        for (const auto& wing : r.wings) {
            gait_groups.insert(wing.gait_group);
        }

        noWingGroups = gait_groups.size();
        if (gait_groups.size() == 2 && r.wings.size() == 4) {
            for (auto& wing : wings) {
                if (wing.gait_group == 0) {
                    wing.stroke_plane_angle_rad = 0.1f + (1.0f - in.stability_vs_speed) * 0.6f;
                    wing.phase_offset = 0.0f;
                } else {
                    wing.stroke_plane_angle_rad = 0.3f + (1.0f - in.stability_vs_speed) * 0.5f;
                    wing.phase_offset = in.stability_vs_speed * M_PI;
                }
            }
        }
    }

    // ============================================================================
    // AVAILABLE POWER (metabolic constraints)
    // ============================================================================

    // Sustained power limited by BOTH muscle capacity AND aerobic metabolism
    power_W available_power_W;
    if (s.metabolic.available_muscle_power_W < s.metabolic.max_rate_W) {
        // Metabolism can support muscle output (small animals/insects)
        available_power_W = s.metabolic.available_muscle_power_W;
    } else {
        // Metabolism is bottleneck (large endotherms)
        auto sustained_efficiency = 0.20f + 0.05f * in.feather_quality;
        available_power_W = s.metabolic.max_rate_W * sustained_efficiency;
    }

    // ============================================================================
    // PROFILE DRAG COEFFICIENT (regime-dependent)
    // ============================================================================

    // Base CD for thin airfoils: 0.02-0.03
    // At low Re, unsteady effects (LEV, wake capture) provide "free" thrust
    auto CD_base = 0.02f + 0.01f * (1.0f - in.feather_quality);
    auto CD_profile = glm::mix(CD_base * 0.3f, CD_base, regime_blend);  // 70% reduction for insects

    // ============================================================================
    // POWER-LIMITED FREQUENCY
    // ============================================================================

    // Average aspect ratio for added mass calculation
    float avg_aspect_ratio = 0.0f;
    for (const auto& wing : wings) {
        avg_aspect_ratio += wing.aspect_ratio();
    }
    avg_aspect_ratio = std::max(1.0f, avg_aspect_ratio / wings.size());

    // Added mass inertia from potential flow theory
    auto k = added_mass_coef(avg_aspect_ratio);
    auto single_wing_span = total_wing_span_m / 2.0f;
    inertia_kgm2 per_wing_added_inertia = added_inertia(k, rho, mean_chord_m, single_wing_span);
    // wing_inertia_kg_m2 is per-wing (averaged on line 54), so use per-wing added inertia
    auto inertial_coef = (wing_inertia_kg_m2 + per_wing_added_inertia) * float(wings.size());

    // Power amplitude for frequency calculation
    angle_rad power_limit_amplitude_rad = glm::mix(cruise_amplitude_rad, base_beat_amplitude_rad, 0.5f);

    // Profile power coefficient
    auto profile_coef = profile_power_coef(rho, power_limit_amplitude_rad,
                                           single_wing_span, total_wing_area_m2, CD_profile);

    // Total power coefficient and frequency limit
    auto total_power_coef = inertial_coef + profile_coef;
    freq_Hz power_limited_freq_Hz = power_limited_frequency(available_power_W, total_power_coef,
                                                             power_limit_amplitude_rad);

    // Neural bandwidth limit (Sponberg et al. 2015)
    freq_Hz neural_limit_Hz = 200.0f;

    r.wingbeat_frequency_Hz = std::min<freq_Hz>({base_frequency_Hz, power_limited_freq_Hz, neural_limit_Hz});

    // --- STROUHAL NUMBER CONSTRAINT ---
    // Universal for efficient oscillatory locomotion: St = f·A/v ≈ 0.2-0.4
    // BUT: optimal St varies by Reynolds regime:
    // - Birds (high Re): St ≈ 0.2-0.3 at cruise
    // - Insects (low Re): St ≈ 0.3-0.5 due to unsteady effects (LEV, wake capture)
    // Stroke length = wingtip excursion = arc length = half_amplitude × wing_radius × 2
    // (factor of 2 for peak-to-peak excursion, but Strouhal uses peak-to-peak)
    // Use CRUISE amplitude for cruise speed calculation, not morphological average
    // Strouhal requires PEAK-TO-PEAK amplitude (factor 2 over single-sided).
    length_m cruise_stroke_length_m = 2.0f * cruise_amplitude_rad * (total_wing_span_m / 2.0f);

    // Blend Strouhal based on regime (insects operate at higher St)
    auto strouhal_bird = 0.25f;   // Birds cruise near lower end of efficient range
    auto strouhal_insect = 0.45f; // Insects operate higher due to unsteady aero
    auto strouhal_optimal = glm::mix(strouhal_insect, strouhal_bird, regime_blend);

    // Kinematic cruise speed from Strouhal constraint
    velocity_m_s strouhal_cruise_m_s = (r.wingbeat_frequency_Hz * cruise_stroke_length_m) / strouhal_optimal;

    // Save the true Strouhal-limited speed before any overrides
    // This represents what the creature can actually achieve kinematically
    velocity_m_s true_strouhal_speed_m_s = strouhal_cruise_m_s;

    // Ensure cruise speed is above stall (for reporting purposes)
    strouhal_cruise_m_s = std::max(strouhal_cruise_m_s, r.min_flight_speed_m_s * 1.2f);

    // Power-limited cruise speed: can't fly faster than power allows
    // Approximate: P_required ∝ v³ for parasite drag at high speed
    // At power-optimal speed, induced and parasite power are roughly equal
    // v_mp ≈ (2 * W² / (ρ * S * CD * π * e * AR))^(1/4) - simplified
    // For now, use available power to cap the maximum sustainable cruise
    auto drag_coefficient = 0.1f + 0.05f * (1.0f - regime_blend);  // Higher CD for insects
    velocity_m_s power_limited_cruise = cbrt(available_power_W /
        (0.5f * rho * s.physical.cross_sectional_area_m2 * drag_coefficient));

    // Use the lower of Strouhal-derived and power-limited speeds
    r.cruise_speed_m_s = std::min(strouhal_cruise_m_s, power_limited_cruise);

    // Maximum speed at higher Strouhal (less efficient, more power)
    auto strouhal_max = glm::mix(0.25f, 0.20f, regime_blend);  // Birds can push lower
    r.max_flight_speed_m_s = (r.wingbeat_frequency_Hz * cruise_stroke_length_m) / strouhal_max;
    
    // --- REYNOLDS NUMBER ---
    // Re = (ρ * v * c) / μ - determines aerodynamic regime
    // ============================================================================
    // AERODYNAMIC POWER REQUIREMENTS
    // ============================================================================

    // Disk area (swept by wings) - used for induced power calculations
    auto disk_area_m2 = M_PI * r.wing_span_m * r.wing_span_m / 4.0f;

    if (noWingGroups > 1) {
        // Multiple wing pairs: sum their swept areas
        disk_area_m2 = 0.0f;
        auto groups = GetGaitGroupSpan(wings.data(), wings.size());
        for (const auto& group : groups) {
            if (group.count == 2) {
                disk_area_m2 += M_PI * group.total_span_m * group.total_span_m / 4.0f;
            } else {
                auto avg = group.total_span_m / group.count;
                disk_area_m2 += group.count * M_PI * avg * avg / 4.0f;
            }
        }
    }

    // Coefficients for power calculations
    auto k_induced = 1.15f;      // Non-ideal induced flow factor
    auto CD_body = 0.1f + 0.05f * (1.0f - in.structure_vs_weight);
    auto elastic_recovery = 0.6f;  // ~40% elastic energy storage in tendons

    // --- POWER AT STALL SPEED ---
    // Direct lift check: can we generate enough power at minimum flight speed?
    auto induced_power_at_stall_W = induced_power(k_induced, effective_weight_N, rho,
                                                   disk_area_m2, r.min_flight_speed_m_s);
    auto profile_power_at_stall_W = profile_power(rho, r.min_flight_speed_m_s,
                                                   total_wing_area_m2, CD_profile);
    auto parasite_power_at_stall_W = profile_power(rho, r.min_flight_speed_m_s,
                                                    s.physical.cross_sectional_area_m2, CD_body);
    auto inertial_power_at_stall_W = inertial_power(elastic_recovery, wing_inertia_kg_m2,
                                                     r.wingbeat_frequency_Hz, cruise_amplitude_rad);

    power_W aero_power_at_stall_W = induced_power_at_stall_W +
                                    profile_power_at_stall_W +
                                    parasite_power_at_stall_W;
    power_W power_required_at_stall_W = aero_power_at_stall_W + inertial_power_at_stall_W;

    // Need 50% margin for control authority
    bool has_power_for_lift = available_power_W >= power_required_at_stall_W * 1.5f;

    // --- POWER AT CRUISE SPEED ---
    auto induced_power_cruise_W = induced_power(k_induced, effective_weight_N, rho,
                                                 disk_area_m2, r.cruise_speed_m_s);
    auto profile_power_cruise_W = profile_power(rho, r.cruise_speed_m_s,
                                                 total_wing_area_m2, CD_profile);
    auto parasite_power_cruise_W = profile_power(rho, r.cruise_speed_m_s,
                                                  s.physical.cross_sectional_area_m2, CD_body);

    auto aerodynamic_power_W = induced_power_cruise_W + profile_power_cruise_W + parasite_power_cruise_W;

    // Inertial power to reverse the wing each half-stroke (Weis-Fogh / Van Den Berg & Rayner):
    //   P_inertial = k · 8π² · I · Φ² · f³   (Φ = stroke amplitude rad, f = wingbeat Hz)
    // k≈0.5: part of the kinetic energy is recovered elastically/aerodynamically.
    // Replaces the old I^0.3 heuristic with a single physical model (matches inertial_power lambda).
    const float k_recovery = 0.5f;
    power_W inertial_power_cruise_W = inertial_power(k_recovery, wing_inertia_kg_m2,
                                                     r.wingbeat_frequency_Hz, cruise_amplitude_rad);

    power_W mechanical_power_W = aerodynamic_power_W + inertial_power_cruise_W;

    // --- METABOLIC CONVERSION ---
    // Muscle efficiency: 10% (insects) → 23% (birds)
    auto reynolds_cruise = reynolds_number(rho, r.cruise_speed_m_s, mean_chord_m, mu);
    auto cruise_regime_blend = compute_regime_blend(reynolds_cruise);
    auto muscle_efficiency = 0.10f + 0.13f * cruise_regime_blend;
    power_W metabolic_power_W = mechanical_power_W / muscle_efficiency;

	// Report metabolic power for forward flight (raw Watts, universally meaningful)
	r.flapping_power_W = metabolic_power_W;

	// Flapping capability: compare MECHANICAL power (apples to apples)
	// available_power_W is muscle mechanical output capacity
	// mechanical_power_W is mechanical work needed to fly
	r.flapping_efficiency = std::min<float>(1.0f, available_power_W / mechanical_power_W);
	if (r.flapping_efficiency < 0.5f) {
		// Below 50% power margin = marginal flight capability
	} else {
		// Normalize 0.5-1.0 range to 0-1 for "comfortable" flight
		r.flapping_efficiency = (r.flapping_efficiency - 0.5f) * 2.0f;
	}
    
	// so if we have 1 pair of wings then 1 center
	// if we have 2 then 2 centers
	// if we have 5 then 5 centers etc.
	// centers represnent a group of wings that are symmetrical, bilaterially, penta- whatever.	
	std::span<TonTon::Analysis_Aerial::Wing> as_span(wings);
    auto centers = shared_array<glm::vec3>::Build(in.builder->gait_group_centers.size(), [&](int i)
    {
		return glm::vec3(in.builder->gait_group_centers[i]) * float(in.scale);
    });
    
    // HOVERING FLIGHT
	// Use disk area (swept area) not wing planform area

	// Induced power from momentum theory: P = W×√(W/(2ρA))
	auto hover_induced_power_W = hover_induced_power(effective_weight_N, rho, disk_area_m2);

	// --- HOVER AMPLITUDE ---
	// Dragonflies hovering use SMALL amplitudes - they look nearly stationary!
	// Hovering specialists (high stability_vs_speed) use smaller, controlled motions.
	// Forward flight uses larger sweeps for thrust generation.
	// Typical hovering amplitude: 30-60° peak-to-peak (vs 100-140° for hummingbirds)
	angle_rad hover_amplitude_actual_rad = glm::mix(0.7f, 1.4f, in.stability_vs_speed);  // 40-80° p2p
	angle_rad hover_half_amp = hover_amplitude_actual_rad / 2.0f;

	// --- LEV (Leading Edge Vortex) BENEFIT ---
	// At low Reynolds numbers, unsteady aerodynamics with LEV HELPS by enhancing CL
	// Ellington (1984): LEV can provide 2/3 of total lift in hovering insects
	// This is a benefit, not a penalty!
	auto lev_benefit = 1.0f;
	if (float(reynolds_cruise) < 10000.0f) {
		// LEV provides up to 40% power reduction at very low Re
		auto re_factor = float(reynolds_cruise) / 10000.0f;
		lev_benefit = 1.0f + 0.4f * (1.0f - re_factor);  // 1.0 to 1.4
	}

	// --- PROFILE POWER ---
	// Power to overcome wing drag during hovering strokes
	power_W hover_profile_power_W = 0.0f;

	for (const auto& wing : wings) {
		// Peak tip velocity = 2πf × half_amplitude × wing_length
		auto wing_tip_velocity = 2.0f * M_PI * r.wingbeat_frequency_Hz *
								 hover_half_amp * wing.span_m;
		auto mean_velocity = 0.7f * wing_tip_velocity;  // Mean over sinusoidal stroke

		// Profile power for this wing (CD higher in hover due to higher AoA)
		auto wing_CD = CD_profile * 1.5f;
		auto wing_profile_power = 0.5f * rho *
								  pow3(mean_velocity) *
								  wing.wing_area_m2 * wing_CD;

		hover_profile_power_W += wing_profile_power;
	}

	// --- MULTI-WING BENEFITS ---
	// 4+ wings (dragonflies, damselflies) can phase independently:
	// - Counter-phasing reduces peak torque demands
	// - Fore/hind wing interaction can recover energy from wake
	// - More continuous thrust = less acceleration/deceleration
	auto multi_wing_hover_benefit = 1.0f;
	if (noWingGroups > 1) {
		// Each additional wing pair adds ~15% efficiency gain
		multi_wing_hover_benefit = 1.0f + 0.15f * (noWingGroups - 1);
	}

	// --- INERTIAL POWER ---
	// Must recalculate from hover parameters, not reuse forward flight value
	// Inertial power = energy to reverse wing direction each half-stroke
	// For small amplitudes, this is much lower than forward flight
	// Physical model (Weis-Fogh): P_inertial = k·8π²·I·Φ²·f³ at the hover amplitude.
	// Lower hover amplitude => lower inertial cost falls out naturally from Φ².
	power_W hover_inertial_power_W = inertial_power(k_recovery, wing_inertia_kg_m2,
	                                                r.wingbeat_frequency_Hz, hover_half_amp);

	// --- TOTAL MECHANICAL POWER ---
	// Apply LEV benefit and multi-wing benefit
	auto hover_mechanical_W = (hover_induced_power_W + hover_profile_power_W +
							   hover_inertial_power_W) / (lev_benefit * multi_wing_hover_benefit);

	// Convert to metabolic power for reporting (same efficiency as forward flight)
	auto hover_metabolic_W = hover_mechanical_W / muscle_efficiency;
	r.hovering_power_W = hover_metabolic_W;

	// Hovering capability: compare MECHANICAL power (apples to apples)
	// Hovering requires more power margin than forward flight for stability
	auto hover_mechanical_ratio = available_power_W / hover_mechanical_W;
	bool hover_power_ok = hover_mechanical_ratio > 1.5f;  // Need 50% margin for control

	if (hover_power_ok) {
		// Map 1.5x-3x mechanical margin to 0-1 efficiency
		r.hovering_efficiency = std::min(1.0f, (hover_mechanical_ratio - 1.5f) / 1.5f);
	} else {
		r.hovering_efficiency = 0.0f;
	}

    
    // ============================================================================
    // FINAL FLIGHT CAPABILITIES EVALUATION
    // ============================================================================
    // Re-evaluate flight capabilities based on final power ratios, after all
    // adjustments and clade-specific refinements.
    
    // Note: flapping_power_W and hovering_power_W are METABOLIC power (raw Watts)
    // available_power_W is MECHANICAL capacity
    // For flight capability, compare mechanical vs mechanical using the efficiency values
    // we already computed (r.flapping_efficiency, r.hovering_efficiency)

    // Flight requires ALL of:
    // 1. Kinematic check: can flapping achieve stall speed? (Strouhal constraint)
    // 2. Lift/power check: enough power to fly at stall speed? (direct lift check)
    // 3. Sustained flight check: power margin for cruise? (flapping_efficiency)
    bool can_reach_flight_speed = true_strouhal_speed_m_s >= r.min_flight_speed_m_s;

    r.can_sustain_level_flight = can_reach_flight_speed && has_power_for_lift;

    // Generate warnings explaining WHY flight fails
    using Warning = Analysis_Diagnostics::Warning;
    using Severity = Warning::Severity;

    if (!can_reach_flight_speed) {
        char buf[256];
        float speed_deficit_pct = 100.0f * (1.0f - float(true_strouhal_speed_m_s / r.min_flight_speed_m_s));
        float freq_needed = float(r.wingbeat_frequency_Hz) * float(r.min_flight_speed_m_s / true_strouhal_speed_m_s);
        snprintf(buf, sizeof(buf),
            "Cannot sustain level flight: wingbeat too slow. "
            "Achievable speed %.1f m/s < stall speed %.1f m/s (%.0f%% deficit). "
            "Would need %.1f Hz wingbeat (have %.1f Hz). "
            "Reduce mass, increase wing area, or lower gravity.",
            float(true_strouhal_speed_m_s), float(r.min_flight_speed_m_s),
            speed_deficit_pct, freq_needed, float(r.wingbeat_frequency_Hz));
        s.diagnostics.warnings.push_back({Severity::ERROR, buf});
    }

    if (!has_power_for_lift) {
        char buf[256];
        float power_deficit_pct = 100.0f * (1.0f - float(available_power_W / (power_required_at_stall_W * 1.5f)));
        snprintf(buf, sizeof(buf),
            "Cannot sustain level flight: insufficient power for lift. "
            "Available %.1f W < required %.1f W (%.0f%% deficit). "
            "Need %.1f W aerodynamic + %.1f W inertial with 50%% margin. "
            "Reduce mass, increase air density, or lower gravity.",
            float(available_power_W), float(power_required_at_stall_W * 1.5f),
            power_deficit_pct, float(aero_power_at_stall_W), float(inertial_power_at_stall_W));
        s.diagnostics.warnings.push_back({Severity::ERROR, buf});
    }
    // can_slow_descent: need enough power to partially counteract gravity
    // Requires ~50% of level flight power to meaningfully slow descent
    // Use power_required_at_stall for consistency with level flight check
    r.can_slow_descent = (available_power_W >= power_required_at_stall_W * 0.5f);
    // Logical constraint: if you can sustain level flight, you can slow descent
    if (r.can_sustain_level_flight) r.can_slow_descent = true;

    r.can_hover = (r.hovering_efficiency > 0.0f);

    // --- TURNING PERFORMANCE ---
    // In air: minimum radius limited by structural g-loads (typically 2-9g max)
    // In water (neutrally buoyant): limited by hydrodynamic control authority
    auto max_load_factor = glm::mix(3.0f, 9.0f, in.structure_vs_weight);
    if (g_effective > 0.1f) {
        // Standard aerial turning: r = v² / (g × sqrt(n² - 1))
        r.min_turning_radius_m = aerial_turn_radius(r.cruise_speed_m_s, g_effective, max_load_factor);
    } else {
        // Hydrodynamic turning for neutrally buoyant creatures
        // Most aquatic animals can turn within 1-3 body lengths
        auto body_lengths_per_turn = 1.0f + 2.0f * (1.0f - in.stability_vs_speed);
        r.min_turning_radius_m = s.physical.body_length_m * body_lengths_per_turn;
    }
    
    auto GetInertia = [I=in.inertia_restPose()](glm::vec3 const& axis) -> inertia_kgm2
    {
		return glm::dot(axis, I * axis);
    };
		
	// Roll/pitch rates limited by wing inertia and control authority
	auto lift_coeff = 0.5; // rough estimate
	//auto v_tip_m_s = 2.0 * M_PI * base_frequency_Hz * (total_wing_span_m/2.0) * base_beat_amplitude_rad;
	
	// Total force from all wings
	auto total_wing_force = 0.5 * rho * (r.cruise_speed_m_s*r.cruise_speed_m_s) * (total_wing_area_m2/4.0) * lift_coeff;
	
	// Differential control forces (one side/pair vs the other)
	auto differential_force = total_wing_force * 0.5; // max difference when fully asymmetric
	
	// Roll: left vs right wings (moment arm = half wingspan)
	torque_rad_s2 roll_torque = differential_force * (total_wing_span_m / 2.0);
		
	// Pitch: fore vs aft wings (moment arm = fore-aft separation)
	// Compute from gait group centers: wings in different gait groups (e.g. dragonfly fore/aft)
	// have measurable separation. Single-pair fliers get 0 pitch torque from this.
	length_m forewing_aft_separation = 0;
	{
		std::set<uint16_t> wing_gait_groups;
		for (const auto& wing : r.wings) {
			wing_gait_groups.insert(wing.gait_group);
		}
		for (auto it1 = wing_gait_groups.begin(); it1 != wing_gait_groups.end(); ++it1) {
			for (auto it2 = std::next(it1); it2 != wing_gait_groups.end(); ++it2) {
				auto dist = glm::distance(centers[*it1], centers[*it2]);
				forewing_aft_separation = std::max(forewing_aft_separation, length_m(dist));
			}
		}
	}
	torque_rad_s2 pitch_torque = differential_force * (forewing_aft_separation);
	
	// Yaw: typically weaker, dominated by drag forces
	// Dragonflies use counter-rotation and body drag
	torque_rad_s2 yaw_torque = differential_force * (total_wing_span_m / 2.0); // rough estimate, typically smaller

	r.max_roll_rate_rad_s = (roll_torque / GetInertia({0, 0, 1})) / r.wingbeat_frequency_Hz;
	r.max_pitch_rate_rad_s = (pitch_torque / GetInertia({1, 0, 0})) / r.wingbeat_frequency_Hz;
	r.max_yaw_rate_rad_s = (yaw_torque / GetInertia({0, 1, 0})) / r.wingbeat_frequency_Hz;
    
    
    // ============================================================================
    // EMPIRICAL / ALLOMETRIC SCALING (R² territory)
    // ============================================================================
    
    // --- ALTITUDE LIMIT ---
    // Allometric: scales with metabolic capacity and oxygen delivery
    // Typical birds: sea level to ~5000m, exceptional to ~9000m
    auto altitude_metabolic_factor = in.metabolic_efficiency * in.muscle_quality;
    r.max_altitude_m = 3000.0f + 6000.0f * altitude_metabolic_factor; // 3000-9000m range
    
    // --- ENDURANCE ---
    auto flight_metabolic_rate_W = glm::mix(s.metabolic.basal_rate_W, s.metabolic.max_rate_W, in.metabolic_efficiency);
    
    // Fat reserves: 10-20% for migrants, 5-10% for residents
    // Energy density of fat: 39 MJ/kg
    auto fat_fraction = 0.05f + 0.15f * in.behavior.endurance_vs_power;
    auto fat_mass_kg = body_mass_kg * fat_fraction;
    energy_J energy_available_J = float(fat_mass_kg) * 39.0e6f;
    
    r.max_flight_duration_s = energy_available_J / flight_metabolic_rate_W;
    
    // --- MANEUVERABILITY TUNING ---
    // Empirically, smaller, lighter birds are more maneuverable
    // Scale rates by size (inverse relationship)
 /*   auto size_maneuver_factor = std::max(0.3f, 1.0f / std::sqrt(body_mass_kg));
    r.max_roll_rate_rad_s *= size_maneuver_factor;
    r.max_pitch_rate_rad_s *= size_maneuver_factor;*/
    
    // Activity level affects cruise speed choice
    // Soaring birds fly slower, flapping birds faster
    r.cruise_speed_m_s *= (0.8f + 0.4f * in.activity_level);
    
    
    // ============================================================================
    // VALIDATION & RETURN
    // ============================================================================
    
    // Sanity checks
    // Check if ANY flight mode is viable
    if (r.flapping_efficiency == 0.0f && 
        r.hovering_efficiency == 0.0f) {
        return {}; // Not a flyer
    }
    
    return r;
}

std::vector<TonTon::Analysis_Aerial::Wing> TonTon::GetWings(Input const& in, bool & likely_feathers)
{
using SF = SemanticFlags;
	
	std::vector<TonTon::Analysis_Aerial::Wing> r;
	r.reserve(in.builder->appendages.size());
	
	auto body_density= in.body_density();
	
	auto area_scale = in.surface_area_scale();
	auto volume_scale = in.volume_scale();
	auto inertia_scale = in.inertia_scale();
	
	for(auto & appendage : in.builder->appendages)
	{
		if(!HasFlag(appendage.semantic_flags, SF::WING))
			continue;
			
		TonTon::Analysis_Aerial::Wing wing;
		appendage.copy_into(wing, in.scale);
				
		wing.span_m = wing.stretched_length_m + scale_to<0>(appendage.distance_to_parent, in.scale);
		wing.wing_area_m2 = scale_to<0>(appendage.surface.area, area_scale);
		wing.chord_m = scale_to<0>(appendage.surface.chord, in.scale);
					
		wing.wing_mass_kg = scale_to<0>(appendage.volume, volume_scale) * body_density;
		wing.wing_inertia_kgm2 = scale_to<0>(appendage.surface.unit_inertia, inertia_scale) * body_density;
		
		// --- WING INERTIA --- 
		// Van Den Berg & Rayner (1995): I = k × m_wing × L² (R^2 = 0.97)
		// Wing mass typically 10-15% of body mass (more for hovering specialists)
		// only applies to birds.
		auto wing_mass_fraction = 0.10f + 0.05f * in.stability_vs_speed;
		auto wing_mass_kg = in.body_mass_kg() * wing_mass_fraction;
		// Van Den Berg & Rayner (1995): I ≈ 0.1–0.15·m·L² for a real (tapered) wing,
		// not 0.33 (uniform rod about one end).
		auto wing_inertia_kg_m2 = 0.15f * wing_mass_kg * wing.span_m * wing.span_m;
		
	// indicates feathers	
		if(wing_mass_kg < wing.wing_mass_kg)	
		{
			wing.wing_mass_kg = wing_mass_kg;
			wing.wing_inertia_kgm2 = wing_inertia_kg_m2;
			likely_feathers = true;
		}
		
		r.push_back(wing);
	}
	
	return r;
}


/*
struct GaitGroupSpan
{
	uint16_t gaitGroup;
	uint16_t count;
	auto total_span_m;
};
*/

std::vector<TonTon::GaitGroupSpan> TonTon::GetGaitGroupSpan(Analysis_Aerial::Wing * data, size_t size)
{
	std::vector<GaitGroupSpan> group_span;
	group_span.reserve(2);
	
	for(auto i = 0u; i < size; ++i)
	{
		Analysis_Aerial::Wing * p = &data[i];
	
		for(auto j = 0u; j < group_span.size(); ++j)
		{
			if(group_span[j].gaitGroup == p->gait_group)
			{
				group_span[j].total_span_m += p->span_m;
				group_span[j].count += 1;
				goto found; 
			}
			
			if(group_span[j].gaitGroup > p->gait_group)
			{
				group_span.insert(group_span.begin()+j, {p->gait_group,1, p->span_m});
				goto found; 
			}
		}

		group_span.push_back({p->gait_group,1, p->span_m});
		
	found:
		(void)0;
	}
	
	return group_span;
}
