#include "tonton_terrestrial.h"
#include "../../include/tonton_output.h"
#include "Memos/tonton_armaturememo.h"
#include "Rules/tonton_climbing.h"
#include "Rules/tonton_scratch.h"
#include "tonton_input.h"
#include <cfloat>

using SF = TonTon::SemanticFlags;


std::optional<TonTon::Output_Terrestrial>  TonTon::ComputeTerrestrial(Input const& in, Scratch &out)
{
	SF constexpr NOT_LIMB_FLAGS = SF(
		int64_t(SF::HEAD)|
		int64_t(SF::NECK)|
		int64_t(SF::SPINE)|
		int64_t(SF::ABDOMEN)
	);
	
	auto position = in.skinnedMesh->skin->position.data();
	auto children = in.skinnedMesh->skin->memo()->GetChildren();
	auto appendages = GetAppendages(in, GetChainsFromTip(in, SF::CONTACT, NOT_LIMB_FLAGS));
	auto legs = ComputeManipulation(in, appendages);
	
	if(legs.empty())
		return {};
	
	float functional_length{FLT_MAX};
	float posture = 0.0f;
	
	for(auto const& leg : legs)
	{
		functional_length = std::min(functional_length, leg.stretched_length_m);
		
		if(children[leg.root].size())
		{
			glm::vec3 accumulator{0};
			for(auto child : children[leg.root])
			{
				accumulator += position[child];
			}
			
			auto tip = accumulator / float(children.size());
			auto vec = glm::normalize(tip - position[leg.root]); 
			
			posture = std::max(posture, 1.f - std::abs(vec.y));
		}
	}

	// Basic speed scaling (from actual data, not Froude)
	// These are empirical fits from Garland 1983, Reilly et al. 2007
	float upright_base_speed = 10.0f * pow(out.physical.body_mass_kg, 0.17f); // m/s
	float sprawling_base_speed = 2.5f * pow(out.physical.body_mass_kg, 0.25f); // m/s
	
	float base_sprint = glm::mix(sprawling_base_speed, upright_base_speed, posture);
	
	// Posture affects sustained speed more than sprint
	auto max_sustainable_speed_m_s = base_sprint * glm::mix(0.3f, 0.6f, posture);
		
	float max_sprint_duration_s = -1;
	float recovery_time_s = -1;
	if (posture < 0.5f) { // sprawling
		// Carrier's constraint: lateral bending -> breathing conflict
		float mass_factor = glm::min(1.0f, out.physical.body_mass_kg / 50.0f); // worse for heavier animals
		
		max_sprint_duration_s = glm::mix(30.0f, 3.0f, mass_factor); // heavy lizards tire fast
		recovery_time_s = max_sprint_duration_s * 2.0f;
	}  
		
	// Turning radius limited by centripetal force
	// F_centripetal = m*v²/r, limited by friction coefficient * weight
	float friction_coeff = glm::mix(0.6f, 0.8f, posture); // sprawling has lower CoM
	float max_lateral_accel = friction_coeff * 9.81f;
	
	float min_turning_radius_m = (base_sprint * base_sprint) / max_lateral_accel;
	
	// Forward acceleration limited by muscle force
	// Very rough: force ~ cross_sectional_area of muscles ~ mass^(2/3)
	float force_to_mass_ratio = 15.0f * std::pow(out.physical.body_mass_kg, -0.33f); // N/kg
	float max_acceleration_m_s2 = force_to_mass_ratio * glm::mix(0.7f, 1.0f, posture);

	return Output_Terrestrial
	{
		.legs=shared_array<Output_Manipulator>::FromArray(legs),
		.posture=posture,
		.max_sprint_speed_m_s=base_sprint,
		.max_sustainable_speed_m_s=max_sustainable_speed_m_s,
		.optimal_speed_m_s=max_sustainable_speed_m_s*0.7f,
		.min_turning_radius_m=min_turning_radius_m,
		.max_acceleration_m_s2=max_acceleration_m_s2,
		
		.max_sprint_duration_s=max_sprint_duration_s,
		.recovery_time_s=recovery_time_s	
	};
}

std::optional<TonTon::Output_Jumping>  TonTon::ComputeJumping(Input const& in, Scratch & s)
{
	// Jumping requires terrestrial locomotion
	if(!s.terrestrial.has_value())
		return {};

	auto const& terrestrial = s.terrestrial.value();

	if(terrestrial.legs.empty())
		return {};

	float body_mass_kg = s.physical.body_mass_kg;
	float body_weight_N = body_mass_kg * in.environment.gravity_m_s2;

	// 1. DETERMINE JUMP MECHANISM
	Output_Jumping::MechanismType mechanism = Output_Jumping::MechanismType::MUSCLE_DIRECT;

	// Elastic catapult for small animals or specialized jumpers
	// Insects, frogs, fleas use elastic energy storage
	bool is_small = body_mass_kg < 0.1f; // < 100g

	// Check for specialized anatomy (can't detect from skeleton alone)
	// Default to muscle direct for vertebrates, elastic for small animals
	if(is_small)
	{
		mechanism = Output_Jumping::MechanismType::ELASTIC_CATAPULT;
	}

	// Hydraulic for very small arthropods (< 1g)
	if(body_mass_kg < 0.001f)
	{
		mechanism = Output_Jumping::MechanismType::HYDRAULIC;
	}

	// 2. CALCULATE AVAILABLE JUMP FORCE
	// Force comes from leg extension (quadriceps, gastrocnemius equivalents)

	float total_leg_force_N = 0;
	float avg_leg_length_m = 0;
	int leg_count = terrestrial.legs.size();

	for(auto const& leg : terrestrial.legs)
	{
		// Use lift force as proxy for leg extension force
		total_leg_force_N += leg.max_lift_force_N;
		avg_leg_length_m += leg.stretched_length_m;
	}
	avg_leg_length_m /= leg_count;

	// Not all legs contribute equally - use ~70% of total force
	float effective_force_N = total_leg_force_N * 0.7f;

	// 3. CALCULATE TAKEOFF VELOCITY
	// Work-energy: F*d = 0.5*m*v²
	// Stroke distance ≈ leg_length * extension_ratio

	float extension_ratio = 0.7f; // Typical leg extension during jump
	float stroke_distance_m = avg_leg_length_m * extension_ratio;

	// Work done = force * distance
	float work_J = effective_force_N * stroke_distance_m;

	// Kinetic energy at takeoff = work - losses
	float efficiency = 0.65f; // ~35% losses to heat, internal work

	if(mechanism == Output_Jumping::MechanismType::ELASTIC_CATAPULT)
	{
		// Elastic storage is more efficient
		efficiency = 0.85f;
	}
	else if(mechanism == Output_Jumping::MechanismType::HYDRAULIC)
	{
		// Hydraulic is very efficient but limited by fluid volume
		efficiency = 0.75f;
	}

	float kinetic_energy_J = work_J * efficiency;

	// v = sqrt(2 * KE / m)
	float takeoff_velocity_m_s = std::sqrt(2.0f * kinetic_energy_J / body_mass_kg);

	// Apply muscle quality scaling
	takeoff_velocity_m_s *= glm::mix(0.7f, 1.3f, in.muscle_quality);

	// 4. CALCULATE JUMP HEIGHT AND DISTANCE
	// Optimal angle for maximum distance: 45 degrees
	// Optimal angle for maximum height: 90 degrees
	// Most animals compromise: 60-70 degrees

	float takeoff_angle_rad = M_PI / 3.0f; // 60 degrees

	// Vertical component of velocity
	float v_vertical = takeoff_velocity_m_s * std::sin(takeoff_angle_rad);

	// Max height: h = v_y² / (2*g)
	float max_jump_height_m = (v_vertical * v_vertical) / (2.0f * in.environment.gravity_m_s2);

	// Horizontal component
	float v_horizontal = takeoff_velocity_m_s * std::cos(takeoff_angle_rad);

	// Flight time: t = 2 * v_y / g
	float flight_time_s = 2.0f * v_vertical / in.environment.gravity_m_s2;

	// Horizontal distance
	float max_jump_distance_m = v_horizontal * flight_time_s;

	// 5. ELASTIC ENERGY STORAGE (for catapult mechanism)
	float elastic_storage_J = 0;
	float power_amplification_ratio = 1.0f;

	if(mechanism == Output_Jumping::MechanismType::ELASTIC_CATAPULT)
	{
		// Elastic tendons can store energy over longer prep time
		// Then release it rapidly for high power output

		elastic_storage_J = work_J; // Total energy stored in tendons

		// Power amplification = stored_energy / (muscle_power * contraction_time)
		// Typical: 2-20x for various animals
		float muscle_contraction_time_s = 0.1f; // ~100ms for direct muscle
		float tendon_release_time_s = 0.01f;     // ~10ms for elastic release

		power_amplification_ratio = muscle_contraction_time_s / tendon_release_time_s;

		// Enhance jump performance for catapult mechanism
		takeoff_velocity_m_s *= std::sqrt(power_amplification_ratio) * 0.5f;
		max_jump_height_m = (v_vertical * v_vertical) / (2.0f * in.environment.gravity_m_s2);
		max_jump_distance_m = v_horizontal * flight_time_s;
	}

	// 6. RECOVERY TIME
	// Time to recharge for next jump
	// Based on metabolic recovery + muscle relaxation

	float recovery_time_s = 2.0f; // Default: ~2 seconds

	if(mechanism == Output_Jumping::MechanismType::MUSCLE_DIRECT)
	{
		// Direct muscle: fast recovery if aerobic
		recovery_time_s = 1.0f + (kinetic_energy_J / s.metabolic.available_muscle_power_W);
	}
	else if(mechanism == Output_Jumping::MechanismType::ELASTIC_CATAPULT)
	{
		// Elastic storage: slower recovery (need to re-stretch tendons)
		recovery_time_s = 3.0f;
	}
	else if(mechanism == Output_Jumping::MechanismType::HYDRAULIC)
	{
		// Hydraulic: slow recovery (refill hemolymph pressure)
		recovery_time_s = 5.0f;
	}

	// Scale by endurance_vs_power
	recovery_time_s *= glm::mix(0.6f, 1.4f, 1.0f - in.behavior.endurance_vs_power);

	// 7. SANITY CHECKS
	// Jump height should be reasonable (< 50x body length for most animals)
	float max_reasonable_height = s.physical.body_length_m * 50.0f;
	if(max_jump_height_m > max_reasonable_height)
	{
		// Scale back unrealistic jumps
		float scale = max_reasonable_height / max_jump_height_m;
		max_jump_height_m *= scale;
		max_jump_distance_m *= scale;
		takeoff_velocity_m_s *= std::sqrt(scale);
	}

	return Output_Jumping{
		.mechanism = mechanism,
		.max_jump_height_m = max_jump_height_m,
		.max_jump_distance_m = max_jump_distance_m,
		.takeoff_velocity_m_s = takeoff_velocity_m_s,
		.takeoff_angle_rad = takeoff_angle_rad,
		.elastic_storage_J = elastic_storage_J,
		.power_amplification_ratio = power_amplification_ratio,
		.recovery_time_s = recovery_time_s
	};
}
