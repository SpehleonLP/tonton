#include "tonton_aerial.h"
#include "../../include/tonton_skinnedmesh.h"
#include "../../include/tonton_input.h"
#include "../../include/tonton_builder.h"
#include <set>
#include <cmath>
#include <algorithm>

#define pow3(x) ((x)*(x)*(x))

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
std::vector<glm::vec3> GetGaitGroupCenters(Input const& in, std::span<Analysis_Aerial::Wing> wings);
}

std::optional<TonTon::Analysis_Aerial> TonTon::ComputeAerial(Input const& in, Scratch&)
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
        
    
    // ============================================================================
    // DETERMINISTIC PHYSICS-BASED CALCULATIONS
    // ============================================================================
    
    const auto g = in.environment.gravity_m_s2;
    const auto rho = in.environment.fluidDensity_Kg_m3;
    const auto mu = in.environment.fluidViscosity_Pa_s;
    const auto body_mass_kg = in.body_mass_kg();
    const auto weight_N = in.body_weight_N();
    
    // --- WING LOADING ---
    // Pure physics: weight per unit wing area
    r.wing_loading_N_m2 = weight_N / total_wing_area_m2;
    
    // --- STALL SPEED (minimum flight speed) ---
    // From lift equation: L = 0.5 * rho * v² * S * CL
    // Solving for v at L = Weight and CL = CL_max
    // CL_max varies with feather quality: poor feathers ~1.2, excellent ~1.8
    auto CL_max = 1.2f + 0.6f * in.feather_quality;
    r.min_flight_speed_m_s = sqrt((2.0f * weight_N) / (rho * total_wing_area_m2 * CL_max));
    
    // --- WINGBEAT FREQUENCY ---
    // Pennycuick (1996): f ∝ sqrt(g/L) * (wing_loading)^(3/8)
    // Base coefficient ~3.87 for birds, adjusted by scaling strategy
    auto K = 3.87f * (0.8f + 0.4f * in.scaling_strategy); // 3.1 - 4.6 range
    auto base_frequency_Hz = K * sqrt(g / total_wing_span_m) * 
                              std::pow(float(r.wing_loading_N_m2), 0.375f);
    
    

	// --- BEAT AMPLITUDE ---
	// Geometric constraints: hovering needs large amplitude, fast flight smaller
	// Typical range: 60° (1.05 rad) for cruising to 140° (2.44 rad) for hovering
    auto amplitude_factor = 0.3f + 0.5f * in.stability_vs_speed; // hovering increases amplitude
    angle_rad base_beat_amplitude_rad = amplitude_factor * (total_wing_span_m / 2.0f);
    
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

    // --- MUSCLE POWER BUDGET ---
    // Muscle mass: 15-35% of body mass, higher for hovering
    auto muscle_mass_fraction = 0.15f + 0.20f * in.stability_vs_speed;
    auto muscle_mass_kg = body_mass_kg * muscle_mass_fraction;
    
    // Power density: 200-400 W/kg (Ellington et al. 1990)
    cost_W_kg power_density_W_kg = 200.0f + 200.0f * in.muscle_quality;
    power_W available_power_W = muscle_mass_kg * power_density_W_kg * in.metabolic_efficiency;
    
    // --- FREQUENCY LIMITS ---
    // Power-limited frequency from inertial power: P = 0.5 * I * ω³ * A²
    // Solving: ω = (2P / IA²)^(1/3)
    freq_Hz power_limited_freq_Hz = cbrt((2 * available_power_W) / 
        (wing_inertia_kg_m2 * base_beat_amplitude_rad * base_beat_amplitude_rad));
    
    // Neural bandwidth limit (Sponberg et al. 2015)
    freq_Hz neural_limit_Hz = 200.0f;
    
    r.wingbeat_frequency_Hz = std::min<freq_Hz>({base_frequency_Hz, power_limited_freq_Hz, neural_limit_Hz});
    
    // --- STROUHAL NUMBER CONSTRAINT ---
    // Universal for efficient oscillatory locomotion: St = f·A/v ≈ 0.2-0.4
    // Use optimal Strouhal (0.3) to determine cruise speed
    length_m stroke_length_m = base_beat_amplitude_rad; // simplification
    auto strouhal_optimal = 0.3f;
    r.cruise_speed_m_s = (r.wingbeat_frequency_Hz * stroke_length_m) / strouhal_optimal;
    
    // Ensure cruise speed is above stall
    r.cruise_speed_m_s = std::max(r.cruise_speed_m_s, r.min_flight_speed_m_s * 1.2f);
    
    // Maximum speed at higher Strouhal (less efficient, more power)
    auto strouhal_max = 0.2f;
    r.max_flight_speed_m_s = (r.wingbeat_frequency_Hz * stroke_length_m) / strouhal_max;
    
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
	
	// Profile drag power: wings
	auto CD_profile = 0.02f + 0.01f * (1.0f - in.feather_quality);
	auto profile_power_W = 0.5f * rho * pow3(r.cruise_speed_m_s) * 
							total_wing_area_m2 * CD_profile;
	
	// Parasite drag power: body
	auto CD_body = 0.1f + 0.05f * (1.0f - in.structure_vs_weight);
	auto parasite_power_W = 0.5f * rho * pow3(r.cruise_speed_m_s) * 
							 in.cross_sectional_area_m2() * CD_body;
	
	// Inertial power (approximate as fraction of aerodynamic power)
	auto aerodynamic_power_W = induced_power_W + profile_power_W + parasite_power_W;
	
	// Small, fast flappers actually have LOWER inertial costs relative to aerodynamic
	// because they have tiny wing inertia
	auto base_inertial_fraction = 0.10f;
	auto size_factor = std::pow(float(wing_inertia_kg_m2) / 1e-6, 0.3f);  // Scale with inertia
	auto inertial_fraction = base_inertial_fraction * size_factor;

	power_W inertial_power_W = aerodynamic_power_W + inertial_fraction;
	
	// Total mechanical power
	power_W mechanical_power_W = aerodynamic_power_W + inertial_power_W;
	
	// Convert to metabolic cost
	auto muscle_efficiency = 0.20f + 0.03f * in.feather_quality;
	power_W flapping_power_W = mechanical_power_W / muscle_efficiency;
	
	r.flapping_cost_W_per_N = flapping_power_W / weight_N;
		    // Flapping efficiency: can we sustain it?
    r.flapping_efficiency = std::min<float>(1.0f, available_power_W / flapping_power_W);
    if (r.flapping_efficiency < 0.5f) {
    //    r.flapping_efficiency = 0.0f; // Below 50% power margin = not viable
    } else {
        // Normalize 0.5-1.0 range to 0-1
        r.flapping_efficiency = (r.flapping_efficiency - 0.5f) * 2.0f;
    }
    
	// so if we have 1 pair of wings then 1 center
	// if we have 2 then 2 centers
	// if we have 5 then 5 centers etc.
	// centers represnent a group of wings that are symmetrical, bilaterially, penta- whatever.	
	std::span<TonTon::Analysis_Aerial::Wing> as_span(wings);
    auto centers = TonTon::GetGaitGroupCenters(in, as_span); // Added TonTon:: prefix
    
    // HOVERING FLIGHT
	// Use disk area (swept area) not wing planform area
	
	// Induced power: P = T * v_induced
	auto v_induced = sqrt(weight_N / (2.0f * rho * disk_area_m2));
	auto hover_induced_power_W = weight_N * v_induced;
	
	// For hovering profile power with multiple wings
	power_W hover_profile_power_W = 0.0f;
	
	for (const auto& wing : wings) {
		auto wing_tip_velocity = 2.0f * M_PI * r.wingbeat_frequency_Hz * 
								 wing.beat_amplitude_rad * wing.span_m;
		auto mean_velocity = 0.7f * wing_tip_velocity;
		
		// Profile power for this wing
		auto wing_CD = CD_profile * 1.5f;  // Higher AoA in hover
		auto wing_profile_power = 0.5f * rho * 
								  pow3(mean_velocity) * 
								  wing.wing_area_m2 * wing_CD;
		
		hover_profile_power_W += wing_profile_power;
	}
	
	if (noWingGroups > 1) {
		hover_profile_power_W *= (1.0f - 0.2f * (noWingGroups - 1) / 3.0f);
	}
	
	// Inertial power: figure-8 pattern is more complex
	auto hover_inertial_power_W = inertial_power_W * 1.3f;
	
	// Reynolds-dependent efficiency (LEV contribution)
	auto hover_reynolds_efficiency = 1.0f;
	if (reynolds_cruise < 10000.0f) {
		hover_reynolds_efficiency = 0.85f + 0.15f * (reynolds_cruise / 10000.0f);
		// Larger amplitude for LEV stabilization
		for (auto& wing : wings) {
			wing.beat_amplitude_rad *= 1.4f;
		}
	}
	
	// Total mechanical power
	auto hover_mechanical_W = (hover_induced_power_W + hover_profile_power_W + 
							   hover_inertial_power_W) / hover_reynolds_efficiency;
	
	// Muscle efficiency (worse than forward flight)
	auto hovering_power_W = hover_mechanical_W / muscle_efficiency;
	
	r.hovering_cost_W_per_N = hovering_power_W / weight_N;
	
	// Hovering viability
	bool hover_power_ok = (available_power_W / hovering_power_W) > 2.0f;
	
	if (hover_power_ok) {
		auto power_ratio = available_power_W / hovering_power_W;
		r.hovering_efficiency = std::min(1.0f, (power_ratio - 2.0f) / 2.0f);  // 2-4x = 0-1
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
    
    auto final_forward_power_ratio = available_power_W / (r.flapping_cost_W_per_N * weight_N);
    auto final_hover_power_ratio = available_power_W / (r.hovering_cost_W_per_N * weight_N);

    r.can_sustain_level_flight = (final_forward_power_ratio >= 1.0f);
    r.can_slow_descent = (final_forward_power_ratio >= 0.5f && !r.can_sustain_level_flight);
    r.can_hover = (final_hover_power_ratio > 1.0f); // Hovering often requires a greater power margin
    
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
    // Kleiber's law: BMR = 3.5 * M^0.75
    power_W BMR_W = 3.5f * std::pow(float(body_mass_kg), 0.75f);
    auto flight_metabolic_multiplier = 10.0f + 5.0f * (1.0f - in.metabolic_efficiency); // 10-15x
    auto flight_metabolic_rate_W = BMR_W * flight_metabolic_multiplier;
    
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
	
	auto area_scale = in.area_scale();
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
		wing.wing_inertia_kgm2 = scale_to<0>(appendage.unit_inertia, inertia_scale) * body_density;
		
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
