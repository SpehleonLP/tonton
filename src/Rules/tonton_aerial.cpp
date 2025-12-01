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

static std::vector<Analysis_Aerial::Wing> GetWings(Input const& in);
std::vector<GaitGroupSpan> GetGaitGroupSpan(Analysis_Aerial::Wing * data, size_t size);
// Forward declaration for GetGaitGroupCenters
}

std::optional<TonTon::Analysis_Aerial> TonTon::ComputeAerial(Input const& in, Scratch& s)
{
    TonTon::Analysis_Aerial r;
    
    auto wings = shared_array<TonTon::Analysis_Aerial::Wing>::FromArray(GetWings(in));
    r.wings = wings;
    
    if (r.wings.empty()) {
        return {}; // No wings, no aerial capability
    }
    
    // Calculate total wing parameters
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
    
    struct WingDistanceMemo
    {
		uint16_t gait_group_front;
		uint16_t gait_group_back;
		uint16_t count;
		length_m distance;
    };
            
    
    mean_chord_m /= r.wings.size();
    total_wing_span_m = total_wing_span_m * 2.0 / r.wings.size();
    wing_inertia_kg_m2 /= r.wings.size();
    
    r.wing_span_m = total_wing_span_m;
    r.wing_area_m2 = total_wing_area_m2;
    
    total_wing_span_m  *= std::exp2(in.mana.air);
    total_wing_area_m2 *= std::exp2(in.mana.air);
    
    // ============================================================================
    // DETERMINISTIC PHYSICS-BASED CALCULATIONS
    // ============================================================================
    
    const auto rho = in.environment.fluidDensity_Kg_m3;
    const auto mu = in.environment.fluidViscosity_Pa_s;
    const auto g = in.environment.gravity_m_s2 * std::max(0.0f, 1.0f - float(rho / in.body_density()));
    const auto body_mass_kg = in.body_mass_kg();
    // can be negative due to bouyancy.
    const auto weight_N = std::max<force_N>(0.001, in.body_weight_N());
    
    // --- WING LOADING ---
    // Pure physics: weight per unit wing area
    r.wing_loading_N_m2 = weight_N / (total_wing_area_m2);
    
    // --- STALL SPEED (minimum flight speed) ---
    // From lift equation: L = 0.5 * rho * v² * S * CL
    // Solving for v at L = Weight and CL = CL_max
    // CL_max varies by flight regime: birds ~1.2-1.8, insects with LEV ~1.5-3.5
    auto CL_max_bird = 1.2f + 0.6f * in.feather_quality;
    auto CL_max_insect = 2.0f + 1.0f * in.feather_quality;  // LEV enhancement

    // Initial estimate using bird CL_max (will refine after Re calculation)
    velocity_m_s stall_speed_estimate = sqrt((2.0f * weight_N) / (rho * total_wing_area_m2 * CL_max_bird));

    // --- REYNOLDS NUMBER AT STALL ---
    // Re = (ρ * v * c) / μ - determines aerodynamic regime
    // Calculate early to determine which frequency/lift model to use
    auto Re_stall = (rho * stall_speed_estimate * mean_chord_m) / mu;

    // --- REGIME BLENDING ---
    // Use log(Re) to smoothly transition between insect and bird models
    // Insect regime: Re < 5,000 (LEV-dominated, unsteady aerodynamics)
    // Bird regime: Re > 50,000 (quasi-steady aerodynamics)
    // Transition zone: 5,000 - 50,000
    auto log_Re = std::log10(std::max(100.0f, float(Re_stall)));
    auto regime_blend = std::clamp((log_Re - 3.7f) / 1.3f, 0.0f, 1.0f);  // 5000 to 50000
    regime_blend = regime_blend * regime_blend * (3.0f - 2.0f * regime_blend);  // smoothstep

    // Blend CL_max based on regime
    auto CL_max = glm::mix(CL_max_insect, CL_max_bird, regime_blend);
    r.min_flight_speed_m_s = sqrt((2.0f * weight_N) / (rho * total_wing_area_m2 * CL_max));

    // --- WINGBEAT FREQUENCY ---
    // Two models blended by Reynolds regime:

    // 1. Pennycuick (1996) - birds: f ∝ sqrt(g/L) * (wing_loading)^(3/8)
    auto K = 3.87f * (0.8f + 0.4f * in.scaling_strategy); // 3.1 - 4.6 range
    freq_Hz bird_frequency_Hz = K * sqrt(g / total_wing_span_m) *
                                std::pow(float(r.wing_loading_N_m2), 0.375f);

    // 2. Insect allometry - Dudley (2000): f ∝ M^(-0.24)
    // Base coefficient adjusted for synchronous muscle insects
    // Odonata (dragonflies): typically 25-40 Hz, averaging ~30 Hz
    // Small synchronous: butterflies ~10-20 Hz
    // Asynchronous (flies, bees): 100-1000 Hz
    freq_Hz insect_frequency_Hz = 80.0f * std::pow(float(body_mass_kg) / 0.001f, -0.24f);

    // Blend frequencies based on Reynolds regime
    freq_Hz base_frequency_Hz = glm::mix(float(insect_frequency_Hz), float(bird_frequency_Hz), regime_blend);
    
    

	// --- BEAT AMPLITUDE ---
	// Different amplitudes for different flight modes:
	// - Cruise: smaller amplitude (~60°, 1.05 rad peak-to-peak) for efficiency
	// - Hover: larger amplitude (~140°, 2.44 rad peak-to-peak) for lift generation
	// stability_vs_speed is a MORPHOLOGICAL trait (what creature evolved for),
	// NOT the current operating mode. Use mode-specific amplitudes for calculations.
	// NOTE: These are PEAK-TO-PEAK angles. For velocity calculations, use half-amplitude.
	angle_rad cruise_amplitude_rad = 1.05f;   // 60° peak-to-peak for forward flight
	angle_rad hover_amplitude_rad = 2.44f;    // 140° peak-to-peak (for wing data storage, not power calc)
	angle_rad cruise_half_amplitude = cruise_amplitude_rad / 2.0f;
	// Note: actual hover amplitude is computed dynamically in hovering section based on morphology

	// For wing data storage, use a weighted average based on morphological adaptation
	// (creature optimized for hover will tend toward higher amplitude)
	angle_rad base_beat_amplitude_rad = glm::mix(cruise_amplitude_rad, hover_amplitude_rad, in.stability_vs_speed);
    
	int noWingGroups = 1;
	for (auto& wing : wings) {
		wing.beat_amplitude_rad = base_beat_amplitude_rad;
			
		// --- STROKE PLANE ANGLE ---
		// Hovering: ~0° (horizontal), forward flight: ~45-60° (angled back)
		wing.stroke_plane_angle_rad = (1.0f - in.stability_vs_speed) * (M_PI / 4.0f);
		// phase_offset already set in the wing data for left/right
	}
	
	if(r.wings.size() > 2)
	{
		// Count unique gait groups
		std::set<int32_t> gait_groups;
		for (const auto& wing : r.wings) {
			gait_groups.insert(wing.gait_group);
		}
		
		noWingGroups = gait_groups.size();
		if (gait_groups.size() == 2 && r.wings.size() == 4) {
			// DRAGONFLY MODE: front and hind wings
			
			// Strategy depends on flight mode:
			// - Hovering: counter-stroking (180° phase, different planes)
			// - Fast forward: synchronous (0° phase, same plane)
			// - Maneuvering: independent control
			
			for (auto& wing : wings) {
				if (wing.gait_group == 0) {
					// Front wings - more horizontal for hovering
					wing.stroke_plane_angle_rad = 0.1f + (1.0f - in.stability_vs_speed) * 0.6f;
					wing.phase_offset = 0.0f;
				} else {
					// Hind wings - steeper angle
					wing.stroke_plane_angle_rad = 0.3f + (1.0f - in.stability_vs_speed) * 0.5f;
					
					// Phase offset: counter-stroking for hovering, sync for speed
					wing.phase_offset = in.stability_vs_speed * M_PI; // 0° to 180°
				}
			}
		}
	}
    
    // Power density: 200-400 W/kg (Ellington et al. 1990)
    power_W available_power_W = s.metabolic.available_muscle_power_W;

    // Profile drag coefficient (used in multiple calculations)
    auto CD_profile = 0.02f + 0.01f * (1.0f - in.feather_quality);

    // --- FREQUENCY LIMITS ---
    // Power-limited frequency from TOTAL power: P = P_inertial + P_profile
    // P_inertial = k_i × I × f³ × A² (accelerating wing mass)
    // P_profile = k_p × ρ × S × c × A³ × f³ × C_D (drag on moving wing)
    // Total: P = (k_i×I + k_p×ρ×S×c×A×C_D) × f³ × A²

    // Inertial power coefficient
    auto inertial_coef = wing_inertia_kg_m2;

    // Profile drag power coefficient
    // Wing tip velocity: v = 2πfAR (A = amplitude rad, R = wing span/2)
    // P_profile = 0.5 × ρ × v³ × S × C_D = 0.5 × ρ × (2πfAR)³ × S × CD
    //           = 0.5 × ρ × (2π)³ × f³ × A³ × R³ × S × CD
    // Factor as: P = k × f³ × A², so k = 0.5 × ρ × (2π)³ × A × R³ × S × CD
    auto wing_length_m = total_wing_span_m / 2.0f;  // Half-span = one wing length
    constexpr float two_pi_cubed = (2.0f * M_PI) * (2.0f * M_PI) * (2.0f * M_PI);
    auto profile_coef = 0.5f * rho * two_pi_cubed * base_beat_amplitude_rad *
                        pow3(wing_length_m) * total_wing_area_m2 * CD_profile;

    // Total power coefficient (both inertial and aerodynamic)
    auto total_power_coef = inertial_coef + profile_coef;

    freq_Hz power_limited_freq_Hz = cbrt(available_power_W /
        (total_power_coef * base_beat_amplitude_rad * base_beat_amplitude_rad));
    
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
    length_m cruise_stroke_length_m = cruise_amplitude_rad * (total_wing_span_m / 2.0f);

    // Blend Strouhal based on regime (insects operate at higher St)
    auto strouhal_bird = 0.25f;   // Birds cruise near lower end of efficient range
    auto strouhal_insect = 0.45f; // Insects operate higher due to unsteady aero
    auto strouhal_optimal = glm::mix(strouhal_insect, strouhal_bird, regime_blend);

    // Kinematic cruise speed from Strouhal constraint
    velocity_m_s strouhal_cruise_m_s = (r.wingbeat_frequency_Hz * cruise_stroke_length_m) / strouhal_optimal;

    // Ensure cruise speed is above stall
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
    auto reynolds_cruise = (rho * r.cruise_speed_m_s * mean_chord_m) / mu;
    

    // --- AERODYNAMIC POWER REQUIREMENTS ---
		
	// FLAPPING FLIGHT (forward)
	auto disk_area_m2 = M_PI * r.wing_span_m * r.wing_span_m / 4.0f;
	
	if (noWingGroups > 1) {
		// Multiple wing pairs: sum their swept areas
		disk_area_m2 = 0.0f;
		auto groups = GetGaitGroupSpan(wings.data(), wings.size());
		
		for (const auto& group : groups) {
			if (group.count == 2) {
				disk_area_m2 += M_PI * group.total_span_m * group.total_span_m / 4.0f;  // bilateral
			} else {
				auto avg = group.total_span_m / group.count;
				disk_area_m2 += group.count * M_PI * avg * avg / 4.0f;  // radial
			}
		}
	}
	
	// Induced power (momentum theory for forward flight)
	auto k_induced = 1.15f;  // non-ideal flow factor
	auto induced_power_W = k_induced * weight_N * weight_N / 
						   (2.0f * rho * disk_area_m2 * r.cruise_speed_m_s);
	
	// Profile drag power: wings (CD_profile defined earlier)
	auto profile_power_W = 0.5f * rho * pow3(r.cruise_speed_m_s) *
							total_wing_area_m2 * CD_profile;
	
	// Parasite drag power: body
	auto CD_body = 0.1f + 0.05f * (1.0f - in.structure_vs_weight);
	auto parasite_power_W = 0.5f * rho * pow3(r.cruise_speed_m_s) * 
							 s.physical.cross_sectional_area_m2 * CD_body;
	
	// Inertial power (approximate as fraction of aerodynamic power)
	auto aerodynamic_power_W = induced_power_W + profile_power_W + parasite_power_W;
	
	// Small, fast flappers actually have LOWER inertial costs relative to aerodynamic
	// because they have tiny wing inertia
	auto base_inertial_fraction = 0.10f;
	auto size_factor = std::pow(float(wing_inertia_kg_m2) / 1e-6, 0.3f);  // Scale with inertia
	auto inertial_fraction = base_inertial_fraction * size_factor;

	power_W inertial_power_W = aerodynamic_power_W * inertial_fraction;
	
	// Total mechanical power
	power_W mechanical_power_W = aerodynamic_power_W + inertial_power_W;
	
	// Convert mechanical power to metabolic cost (for energy budget reporting)
	// Muscle efficiency: mechanical work out / metabolic energy in
	// Literature: 9-13% for dragonflies (Wakeling & Ellington), 20-25% for birds
	auto muscle_efficiency = 0.10f + 0.13f * regime_blend;  // 10% insects → 23% birds
	power_W metabolic_power_W = mechanical_power_W / muscle_efficiency;

	// Report metabolic cost per unit weight (useful for energy budgets)
	r.flapping_cost_W_per_N = metabolic_power_W / weight_N;

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

	// Induced power: P = T * v_induced (momentum theory)
	// This is the fundamental cost of generating lift by accelerating air downward
	auto v_induced = sqrt(weight_N / (2.0f * rho * disk_area_m2));
	auto hover_induced_power_W = weight_N * v_induced;

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
	auto hover_aero_power = hover_induced_power_W + hover_profile_power_W;
	auto hover_inertial_fraction = inertial_fraction * 0.8f;  // Lower amplitude = less inertia cost
	auto hover_inertial_power_W = hover_aero_power * hover_inertial_fraction;

	// --- TOTAL MECHANICAL POWER ---
	// Apply LEV benefit and multi-wing benefit
	auto hover_mechanical_W = (hover_induced_power_W + hover_profile_power_W +
							   hover_inertial_power_W) / (lev_benefit * multi_wing_hover_benefit);

	// Convert to metabolic power for reporting (same efficiency as forward flight)
	auto hover_metabolic_W = hover_mechanical_W / muscle_efficiency;
	r.hovering_cost_W_per_N = hover_metabolic_W / weight_N;

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

   #if 0
    // HOVERING FLIGHT
    // Hovering requires 2-3x power due to no forward velocity benefit
    // Momentum theory: P_hover = T × sqrt(T / (2 * rho * A))
    auto vortex_interaction_benefit = noWingGroups > 1? 0.2 : 0.0;
    auto hover_induced_power_W = weight_N * std::sqrt(weight_N / (2.0f * rho * total_wing_area_m2));
    auto hover_profile_power_W = profile_power_W * 1.5f * (1.0f - vortex_interaction_benefit); // Higher angle of attack
    auto hover_inertial_power_W = inertial_power_W * 1.2f; // Figure-8 pattern costs more
    
    // Reynolds-dependent efficiency for hovering (LEV contribution)
    auto hover_reynolds_efficiency = 1.0f;
    if (reynolds_cruise < 10000.0f) {
        // Leading edge vortices help at low Reynolds
        hover_reynolds_efficiency = 0.85f + 0.15f * (reynolds_cruise / 10000.0f);
        // Need larger amplitude for LEV stabilization
        for (auto& wing : wings) {
            wing.beat_amplitude_rad *= 1.4f; // ~140-160° full figure-8
        }
    }
    
    auto hovering_power_W = (hover_induced_power_W + hover_profile_power_W + hover_inertial_power_W) / 
                             hover_reynolds_efficiency;
    r.hovering_cost_W_per_N = hovering_power_W / weight_N;
    
    // --- POWER LOADING ---
    r.power_loading_W_N = available_power_W / weight_N;
    //auto power_margin = available_power_W / flapping_power_W;
    
    // --- EFFICIENCY CALCULATIONS ---
    // Efficiency = ability to sustain mode (0 = impossible, 1 = optimal)
    

    // Hovering efficiency: needs >2x power margin + other constraints
    bool hover_power_ok = (available_power_W / hovering_power_W) > 2.0f;
    bool hover_loading_ok = r.wing_loading_N_m2 < 80.0f;
    bool hover_frequency_ok = r.wingbeat_frequency_Hz > 50.0f;
    bool hover_reynolds_ok = reynolds_cruise > 1000.0f;
    
    if (hover_power_ok && hover_loading_ok && hover_frequency_ok && hover_reynolds_ok) {
        r.hovering_efficiency = std::min(1.0f, (available_power_W / hovering_power_W) / 3.0f);
    } else {
        r.hovering_efficiency = 0.0f;
    }
    #endif
    
    // ============================================================================
    // FINAL FLIGHT CAPABILITIES EVALUATION
    // ============================================================================
    // Re-evaluate flight capabilities based on final power ratios, after all
    // adjustments and clade-specific refinements.
    
    // Note: flapping_cost_W_per_N and hovering_cost_W_per_N are METABOLIC costs
    // available_power_W is MECHANICAL capacity
    // For flight capability, compare mechanical vs mechanical using the efficiency values
    // we already computed (r.flapping_efficiency, r.hovering_efficiency)

    r.can_sustain_level_flight = (r.flapping_efficiency > 0.0f);
    r.can_slow_descent = (available_power_W / mechanical_power_W) >= 0.5f;
    r.can_hover = (r.hovering_efficiency > 0.0f);
    
    // --- TURNING PERFORMANCE ---
    // Minimum radius limited by structural loads (typically 2-4g max)
    auto max_load_factor = glm::mix(3.0, 9.0, in.structure_vs_weight); // 2.0-3.5g
    r.min_turning_radius_m = (r.cruise_speed_m_s * r.cruise_speed_m_s) / 
                             (g * std::sqrt(max_load_factor * max_load_factor - 1.0f));
    
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
	length_m forewing_aft_separation = 0.0056; // distance between fore and aft wing roots
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
    r.cruise_speed_m_s *= (0.8f + 0.4f * (1.0f - in.activity_level));
    
    
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

std::vector<TonTon::Analysis_Aerial::Wing> TonTon::GetWings(Input const& in)
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
		auto wing_inertia_kg_m2 = 0.33f * wing_mass_kg * wing.span_m * wing.span_m * 4.0;
		
	// indicates feathers	
		if(wing_mass_kg < wing.wing_mass_kg)	
		{
			wing.wing_mass_kg = wing_mass_kg;
			wing.wing_inertia_kgm2 = wing_inertia_kg_m2;
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
