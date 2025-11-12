#include "tonton_aerial.h"
#include "Memos/tonton_armaturememo.h"
#include "Memos/tonton_skinnedmeshmemo.h"
#include "Rules/tonton_scratch.h"
#include "dodeedum.h"
#include "../../include/tonton_input.h"
#include <iostream>
#include <set>
#include <cmath>
#include <algorithm>


struct GaitGroupSpan
{
	uint16_t gaitGroup;
	uint16_t count;
	float total_span_m;
};

namespace TonTon
{
static std::vector<Output_Aerial::Wing> GetWings(Input const& in, Scratch &out);
std::vector<GaitGroupSpan> GetGaitGroupSpan(Output_Aerial::Wing * data, size_t size);
}

std::optional<TonTon::Output_Aerial> TonTon::ComputeAerial(Input const& in, Scratch &out)
{
    TonTon::Output_Aerial r;
    
    auto wings = shared_array<TonTon::Output_Aerial::Wing>::FromArray(GetWings(in, out));
    r.wings = wings;
    
    if (r.wings.empty()) {
        return {}; // No wings, no aerial capability
    }
    
    // Calculate total wing parameters
    float total_wing_area_m2 = 0.0f;
    float total_wing_span_m = 0.0f;
    float wing_inertia_kg_m2 = 0.0f;
    float mean_chord_m = 0.0f;
        
    for (const auto& wing : r.wings) {
        total_wing_area_m2 += wing.area_m2;
        total_wing_span_m += wing.span_m; 
        wing_inertia_kg_m2 += wing.inertia_kgm2; 
        mean_chord_m += wing.chord_m;
    }
    
    struct WingDistanceMemo
    {
		uint16_t gait_group_front;
		uint16_t gait_group_back;
		uint16_t count;
		float   distance;
    };
            
    
    mean_chord_m /= r.wings.size();
    total_wing_span_m = total_wing_span_m * 2.0 / r.wings.size();
    wing_inertia_kg_m2 /= r.wings.size();
    
    r.wing_span_m = total_wing_span_m;
    r.wing_area_m2 = total_wing_area_m2;
        
    
    // ============================================================================
    // DETERMINISTIC PHYSICS-BASED CALCULATIONS
    // ============================================================================
    
    const float g = in.environment.gravity_m_s2;
    const float rho = in.environment.fluidDensity_Kg_m3;
    const float mu = in.environment.fluidViscosity_Pa_s;
    const float body_mass_kg = out.physical.body_mass_kg;
    const float weight_N = body_mass_kg * g;
    
    // --- WING LOADING ---
    // Pure physics: weight per unit wing area
    r.wing_loading_N_m2 = weight_N / total_wing_area_m2;
    
    // --- STALL SPEED (minimum flight speed) ---
    // From lift equation: L = 0.5 * rho * v² * S * CL
    // Solving for v at L = Weight and CL = CL_max
    // CL_max varies with feather quality: poor feathers ~1.2, excellent ~1.8
    float CL_max = 1.2f + 0.6f * in.feather_quality;
    r.min_flight_speed_m_s = std::sqrt((2.0f * weight_N) / (rho * total_wing_area_m2 * CL_max));
    
    // --- WINGBEAT FREQUENCY ---
    // Pennycuick (1996): f ∝ sqrt(g/L) * (wing_loading)^(3/8)
    // Base coefficient ~3.87 for birds, adjusted by scaling strategy
    float K = 3.87f * (0.8f + 0.4f * in.scaling_strategy); // 3.1 - 4.6 range
    float base_frequency_Hz = K * std::sqrt(g / total_wing_span_m) * 
                              std::pow(r.wing_loading_N_m2, 0.375f);
    
    

	// --- BEAT AMPLITUDE ---
	// Geometric constraints: hovering needs large amplitude, fast flight smaller
	// Typical range: 60° (1.05 rad) for cruising to 140° (2.44 rad) for hovering
    float amplitude_factor = 0.3f + 0.5f * in.stability_vs_speed; // hovering increases amplitude
    float base_beat_amplitude_rad = amplitude_factor * (total_wing_span_m / 2.0f);
    
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
    float muscle_mass_fraction = 0.15f + 0.20f * in.stability_vs_speed;
    float muscle_mass_kg = body_mass_kg * muscle_mass_fraction;
    
    // Power density: 200-400 W/kg (Ellington et al. 1990)
    float power_density_W_kg = 200.0f + 200.0f * in.muscle_quality;
    float available_power_W = muscle_mass_kg * power_density_W_kg * in.metabolic_efficiency;
    
    // --- FREQUENCY LIMITS ---
    // Power-limited frequency from inertial power: P = 0.5 * I * ω³ * A²
    // Solving: ω = (2P / IA²)^(1/3), but using sqrt approximation for efficiency
    float power_limited_freq_Hz = std::sqrt(available_power_W / 
        (wing_inertia_kg_m2 * base_beat_amplitude_rad * base_beat_amplitude_rad));
    
    // Neural bandwidth limit (Sponberg et al. 2015)
    float neural_limit_Hz = 200.0f;
    
    r.wingbeat_frequency_Hz = std::min({base_frequency_Hz, power_limited_freq_Hz, neural_limit_Hz});
    
    // --- STROUHAL NUMBER CONSTRAINT ---
    // Universal for efficient oscillatory locomotion: St = f·A/v ≈ 0.2-0.4
    // Use optimal Strouhal (0.3) to determine cruise speed
    float stroke_length_m = base_beat_amplitude_rad; // simplification
    float strouhal_optimal = 0.3f;
    r.cruise_speed_m_s = (r.wingbeat_frequency_Hz * stroke_length_m) / strouhal_optimal;
    
    // Ensure cruise speed is above stall
    r.cruise_speed_m_s = std::max(r.cruise_speed_m_s, r.min_flight_speed_m_s * 1.2f);
    
    // Maximum speed at higher Strouhal (less efficient, more power)
    float strouhal_max = 0.2f;
    r.max_flight_speed_m_s = (r.wingbeat_frequency_Hz * stroke_length_m) / strouhal_max;
    
    // --- REYNOLDS NUMBER ---
    // Re = (ρ * v * c) / μ - determines aerodynamic regime
    float reynolds_cruise = (rho * r.cruise_speed_m_s * mean_chord_m) / mu;
    

    // --- AERODYNAMIC POWER REQUIREMENTS ---
		
	// FLAPPING FLIGHT (forward)
	float disk_area_m2 = M_PI * r.wing_span_m * r.wing_span_m;
	
	if (noWingGroups > 1) {
		// Multiple wing pairs: sum their swept areas
		disk_area_m2 = 0.0f;
		auto groups = GetGaitGroupSpan(wings.data(), wings.size());
		
		for (const auto& group : groups) {
			if (group.count == 2) {
				disk_area_m2 += M_PI * group.total_span_m * group.total_span_m;  // bilateral
			} else {
				float avg = group.total_span_m / group.count;
				disk_area_m2 += group.count * M_PI * avg * avg;  // radial
			}
		}
	}
	
	// Induced power (momentum theory for forward flight)
	float k_induced = 1.15f;  // non-ideal flow factor
	float induced_power_W = k_induced * weight_N * weight_N / 
						   (2.0f * rho * disk_area_m2 * r.cruise_speed_m_s);
	
	// Profile drag power: wings
	float CD_profile = 0.02f + 0.01f * (1.0f - in.feather_quality);
	float profile_power_W = 0.5f * rho * std::pow(r.cruise_speed_m_s, 3.0f) * 
							total_wing_area_m2 * CD_profile;
	
	// Parasite drag power: body
	float CD_body = 0.1f + 0.05f * (1.0f - in.structure_vs_weight);
	float parasite_power_W = 0.5f * rho * std::pow(r.cruise_speed_m_s, 3.0f) * 
							 out.physical.cross_sectional_area_m2 * CD_body;
	
	// Inertial power (approximate as fraction of aerodynamic power)
	float aerodynamic_power_W = induced_power_W + profile_power_W + parasite_power_W;
	
	// Small, fast flappers actually have LOWER inertial costs relative to aerodynamic
	// because they have tiny wing inertia
	float base_inertial_fraction = 0.10f;
	float size_factor = std::pow(wing_inertia_kg_m2 / 1e-6, 0.3f);  // Scale with inertia
	float inertial_fraction = base_inertial_fraction * size_factor;

	float inertial_power_W = aerodynamic_power_W * inertial_fraction;
	
	// Total mechanical power
	float mechanical_power_W = aerodynamic_power_W + inertial_power_W;
	
	// Convert to metabolic cost
	float muscle_efficiency = 0.20f + 0.03f * in.feather_quality;
	float flapping_power_W = mechanical_power_W / muscle_efficiency;
	
	r.flapping_cost_W_per_N = flapping_power_W / weight_N;
		    // Flapping efficiency: can we sustain it?
    r.flapping_efficiency = std::min(1.0f, available_power_W / flapping_power_W);
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
	std::span<TonTon::Output_Aerial::Wing> as_span(wings);
    auto centers = GetGaitGroupCenters(in, as_span);
    
    // HOVERING FLIGHT
	// Use disk area (swept area) not wing planform area
	
	// Induced power: P = T * v_induced
	float v_induced = std::sqrt(weight_N / (2.0f * rho * disk_area_m2));
	float hover_induced_power_W = weight_N * v_induced;
	
	// For hovering profile power with multiple wings
	float hover_profile_power_W = 0.0f;
	
	for (const auto& wing : wings) {
		float wing_tip_velocity = 2.0f * M_PI * r.wingbeat_frequency_Hz * 
								 wing.beat_amplitude_rad * wing.span_m;
		float mean_velocity = 0.7f * wing_tip_velocity;
		
		// Profile power for this wing
		float wing_CD = CD_profile * 1.5f;  // Higher AoA in hover
		float wing_profile_power = 0.5f * rho * 
								  std::pow(mean_velocity, 3.0f) * 
								  wing.area_m2 * wing_CD;
		
		hover_profile_power_W += wing_profile_power;
	}
	
	if (noWingGroups > 1) {
		hover_profile_power_W *= (1.0f - 0.2f * (noWingGroups - 1) / 3.0f);
	}
	
	// Inertial power: figure-8 pattern is more complex
	float hover_inertial_power_W = inertial_power_W * 1.3f;
	
	// Reynolds-dependent efficiency (LEV contribution)
	float hover_reynolds_efficiency = 1.0f;
	if (reynolds_cruise < 10000.0f) {
		hover_reynolds_efficiency = 0.85f + 0.15f * (reynolds_cruise / 10000.0f);
		// Larger amplitude for LEV stabilization
		for (auto& wing : wings) {
			wing.beat_amplitude_rad *= 1.4f;
		}
	}
	
	// Total mechanical power
	float hover_mechanical_W = (hover_induced_power_W + hover_profile_power_W + 
							   hover_inertial_power_W) / hover_reynolds_efficiency;
	
	// Muscle efficiency (worse than forward flight)
	float hovering_power_W = hover_mechanical_W / muscle_efficiency;
	
	r.hovering_cost_W_per_N = hovering_power_W / weight_N;
	
	// Hovering viability
	bool hover_power_ok = (available_power_W / hovering_power_W) > 2.0f;
	
	if (hover_power_ok) {
		float power_ratio = available_power_W / hovering_power_W;
		r.hovering_efficiency = std::min(1.0f, (power_ratio - 2.0f) / 2.0f);  // 2-4x = 0-1
	} else {
		r.hovering_efficiency = 0.0f;
	}

   #if 0
    // HOVERING FLIGHT
    // Hovering requires 2-3x power due to no forward velocity benefit
    // Momentum theory: P_hover = T × sqrt(T / (2 * rho * A))
    float vortex_interaction_benefit = noWingGroups > 1? 0.2 : 0.0;
    float hover_induced_power_W = weight_N * std::sqrt(weight_N / (2.0f * rho * total_wing_area_m2));
    float hover_profile_power_W = profile_power_W * 1.5f * (1.0f - vortex_interaction_benefit); // Higher angle of attack
    float hover_inertial_power_W = inertial_power_W * 1.2f; // Figure-8 pattern costs more
    
    // Reynolds-dependent efficiency for hovering (LEV contribution)
    float hover_reynolds_efficiency = 1.0f;
    if (reynolds_cruise < 10000.0f) {
        // Leading edge vortices help at low Reynolds
        hover_reynolds_efficiency = 0.85f + 0.15f * (reynolds_cruise / 10000.0f);
        // Need larger amplitude for LEV stabilization
        for (auto& wing : wings) {
            wing.beat_amplitude_rad *= 1.4f; // ~140-160° full figure-8
        }
    }
    
    float hovering_power_W = (hover_induced_power_W + hover_profile_power_W + hover_inertial_power_W) / 
                             hover_reynolds_efficiency;
    r.hovering_cost_W_per_N = hovering_power_W / weight_N;
    
    // --- POWER LOADING ---
    r.power_loading_W_N = available_power_W / weight_N;
    //float power_margin = available_power_W / flapping_power_W;
    
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
    
    // --- TURNING PERFORMANCE ---
    // Minimum radius limited by structural loads (typically 2-4g max)
    float max_load_factor = glm::mix(3.0, 9.0, in.structure_vs_weight); // 2.0-3.5g
    r.min_turning_radius_m = (r.cruise_speed_m_s * r.cruise_speed_m_s) / 
                             (g * std::sqrt(max_load_factor * max_load_factor - 1.0f));
    
    auto GetInertia = [I=out.physical.inertia_restPose()](glm::vec3 const& axis)
    {
		return glm::dot(axis, I * axis);
    };
		
	// Roll/pitch rates limited by wing inertia and control authority
	auto lift_coeff = 0.5; // rough estimate
	//auto v_tip_m_s = 2.0 * M_PI * base_frequency_Hz * (total_wing_span_m/2.0) * base_beat_amplitude_rad;
	
	// Total force from all wings
	float total_wing_force = 0.5 * rho * (r.cruise_speed_m_s*r.cruise_speed_m_s) * (total_wing_area_m2/4.0) * lift_coeff;
	
	// Differential control forces (one side/pair vs the other)
	float differential_force = total_wing_force * 0.5; // max difference when fully asymmetric
	
	// Roll: left vs right wings (moment arm = half wingspan)
	float roll_torque = differential_force * (total_wing_span_m / 2.0);
		
	// Pitch: fore vs aft wings (moment arm = fore-aft separation)
	float forewing_aft_separation = 0.0056; // distance between fore and aft wing roots
	float pitch_torque = differential_force * (forewing_aft_separation);
	
	// Yaw: typically weaker, dominated by drag forces
	// Dragonflies use counter-rotation and body drag
	float yaw_torque = differential_force * (total_wing_span_m / 2.0); // rough estimate, typically smaller
	
	r.max_roll_rate_rad_s = roll_torque / GetInertia({0, 0, 1});
	r.max_pitch_rate_rad_s = pitch_torque / GetInertia({1, 0, 0});
	r.max_yaw_rate_rad_s = yaw_torque / GetInertia({0, 1, 0});
    
    
    // ============================================================================
    // EMPIRICAL / ALLOMETRIC SCALING (R² territory)
    // ============================================================================
    
    // --- ALTITUDE LIMIT ---
    // Allometric: scales with metabolic capacity and oxygen delivery
    // Typical birds: sea level to ~5000m, exceptional to ~9000m
    float altitude_metabolic_factor = in.metabolic_efficiency * in.muscle_quality;
    r.max_altitude_m = 3000.0f + 6000.0f * altitude_metabolic_factor; // 3000-9000m range
    
    // --- ENDURANCE ---
    // Kleiber's law: BMR = 3.5 * M^0.75
    float BMR_W = 3.5f * std::pow(body_mass_kg, 0.75f);
    float flight_metabolic_multiplier = 10.0f + 5.0f * (1.0f - in.metabolic_efficiency); // 10-15x
    float flight_metabolic_rate_W = BMR_W * flight_metabolic_multiplier;
    
    // Fat reserves: 10-20% for migrants, 5-10% for residents
    // Energy density of fat: 39 MJ/kg
    float fat_fraction = 0.05f + 0.15f * in.behavior.endurance_vs_power;
    float fat_mass_kg = body_mass_kg * fat_fraction;
    float energy_available_J = fat_mass_kg * 39.0e6f;
    
    r.max_flight_duration_s = energy_available_J / flight_metabolic_rate_W;
    
    // --- MANEUVERABILITY TUNING ---
    // Empirically, smaller, lighter birds are more maneuverable
    // Scale rates by size (inverse relationship)
 /*   float size_maneuver_factor = std::max(0.3f, 1.0f / std::sqrt(body_mass_kg));
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

std::vector<TonTon::Output_Aerial::Wing> TonTon::GetWings(Input const& in, Scratch & out)
{
using SF = SemanticFlags;
	auto & sk = *in.armature;
	auto * sk_memo = sk.armature->memo();
	auto position = sk.armature->position.data();
	
	auto appendages = GetAppendages(in, GetChainsFromRoot(in, SF::WING));
	std::vector<TonTon::Output_Aerial::Wing> r;
	r.resize(appendages.size());
	
	double body_density=glm::mix(700.0, 1050.0, in.average_density);
	
	for(auto i = 0u; i < r.size(); ++i)
	{
		(Output_Chain&)r[i] = appendages[i];
		r[i].span_m = r[i].stretched_length_m;
		
		SkinnedMesh::LimbMetrics limb_metrics;
		std::pair<glm::quat, glm::vec3> eigen_decomposition;
		
		auto relevant_joints = sk_memo->GetAllChildrenOfRoot(r[i].root);
		auto which = std::span<uint16_t>(relevant_joints.data(), relevant_joints.size());
		auto projection = in.armature->memo()->GetProjectionMatrix(EigenValue::Small, in.behavior.scale, which, &limb_metrics, &eigen_decomposition);
		auto metrics = in.armature->memo()->GetSilhouettes(projection, in.behavior.scale, which);
		
		r[i].area_m2 = metrics.area;
		r[i].chord_m = metrics.MeasureWidth_Segment(position[r[i].root], position[r[i].tip]).length;
		
		auto axis = GetTangentAxis(EigenValue::Small, eigen_decomposition.first, in.position(r[i].root), in.position(r[i].tip));
		
		r[i].mass_kg = limb_metrics.volume * body_density;
		r[i].inertia_kgm2 = limb_metrics.GetInertia(position[r[i].root], body_density, axis);
			
		// --- WING INERTIA --- 
		// Van Den Berg & Rayner (1995): I = k × m_wing × L² (R^2 = 0.97)
		// Wing mass typically 10-15% of body mass (more for hovering specialists)
		// only applies to birds.
		float wing_mass_fraction = 0.10f + 0.05f * in.stability_vs_speed;
		float wing_mass_kg = out.physical.body_mass_kg * wing_mass_fraction;
		float wing_inertia_kg_m2 = 0.33f * wing_mass_kg * r[i].span_m * r[i].span_m * 4.0;
		
	// indicates feathers	
		if(wing_mass_kg < r[i].mass_kg)	
		{
			r[i].mass_kg = wing_mass_kg;
			r[i].inertia_kgm2 = wing_inertia_kg_m2;
		}
	}
	
	for(auto i = 0u; i < r.size(); )
	{
		glm::vec3 accumulator{0.0};
		
		int begin = i;
		for(; i < r.size(); ++i)
		{
			if(r[begin].gait_group != r[i].gait_group)
				break;
			
			accumulator += in.position(i);
		}
		
		accumulator /= (i - begin);
		
		for(i=begin; i < r.size(); ++i)
		{
			r[i].span_m += glm::distance(in.position(r[i].root), accumulator);
		}
	}
	
	return r;
}

std::vector<glm::vec3> TonTon::GetGaitGroupCenters(Input const& in, Output_Appendage * data, size_t size, size_t stride)
{
	std::vector<glm::vec3> positions;
	std::vector<std::pair<int, int>> count;
	positions.reserve(2);
	count.reserve(2);
	
	for(auto i = 0u; i < size; ++i)
	{
		Output_Appendage * p = (Output_Appendage*)(((uint8_t*)data) + stride);
	
		for(auto j = 0u; j < count.size(); ++j)
		{
			if(count[j].first == p->gait_group)
			{
				positions[j] += in.position(p->root);
				count[j].second += 1;
				goto found;
			}
			
			if(count[j].first > p->gait_group)
			{
				count.insert(count.begin()+j, {	int(p->gait_group),1});
				positions.insert(positions.begin()+j, in.position(p->root));
				goto found;
			}
		}

		count.push_back({int(p->gait_group),1});
		positions.push_back(in.position(p->root));
		
	found:
		(void)0;
	}

	for(auto j = 0u; j < count.size(); ++j)
	{
		positions[j] *= 1.0 / count[j].second;
	}
	
	return positions;
}
/*
struct GaitGroupSpan
{
	uint16_t gaitGroup;
	uint16_t count;
	float total_span_m;
};
*/

std::vector<GaitGroupSpan> TonTon::GetGaitGroupSpan(Output_Aerial::Wing * data, size_t size)
{
	std::vector<GaitGroupSpan> group_span;
	group_span.reserve(2);
	
	for(auto i = 0u; i < size; ++i)
	{
		Output_Aerial::Wing * p = &data[i];
	
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
