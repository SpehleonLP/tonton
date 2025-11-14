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

	// ========== CLADE-SPECIFIC REFINEMENTS ==========
	// Applied after generic terrestrial locomotion physics

	using CF = CladeFlags;

	// ARTHROPODA: Temperature-dependent performance (Q10 effects)
	if (HasFlag(out.physical.clade, CF::ARTHROPODA)) {
		// Q10 temperature coefficient (Angilletta et al. 2002)
		// Arthropod performance changes 2-3x per 10K temperature change
		// Smaller arthropods have HIGHER Q10 due to lower thermal inertia

		float q10 = 2.5f; // Typical mid-range (Huey & Kingsolver 1989)

		// Thermal mass effect: smaller = faster heating/cooling
		if (out.physical.body_mass_kg < 0.001f) {
			q10 = 3.0f; // Small insects respond faster to temperature
		}

		// Temperature difference from reference (25°C)
		float reference_temp_K = 298.15f;
		float temp_diff_K = in.environment.temperature_K - reference_temp_K;

		// Performance multiplier from Q10 (Huey & Stevenson 1979)
		float q10_multiplier = std::pow(q10, temp_diff_K / 10.0f);

		// Apply to all speed/acceleration metrics
		base_sprint *= q10_multiplier;
		max_sustainable_speed_m_s *= q10_multiplier;
		max_acceleration_m_s2 *= q10_multiplier;

		// Clamp to reasonable limits (arthropods inactive below ~5°C, above ~50°C)
		if (in.environment.temperature_K < 278.15f) { // Below 5°C
			float cold_penalty = (in.environment.temperature_K - 273.15f) / 5.0f;
			cold_penalty = std::max(cold_penalty, 0.1f);
			base_sprint *= cold_penalty;
			max_sustainable_speed_m_s *= cold_penalty;
		}

		if (in.environment.temperature_K > 323.15f) { // Above 50°C
			float heat_penalty = (323.15f - in.environment.temperature_K) / 10.0f;
			heat_penalty = std::clamp(heat_penalty, 0.1f, 1.0f);
			base_sprint *= heat_penalty;
			max_sustainable_speed_m_s *= heat_penalty;
		}

		// Exoskeleton scaling constraint (Full 1989)
		// Exoskeleton mass scales as M^1.0 (not M^0.67 like internal skeleton)
		// This creates size limit for terrestrial arthropods

		if (out.physical.body_mass_kg > 0.1f) {
			// Harrison et al. (2010): Giant insects extinct after Carboniferous
			// Square-cube law: strength ∝ area (M^0.67), weight ∝ volume (M^1.0)
			// Large arthropods become strength-limited

			float size_penalty = 0.1f / out.physical.body_mass_kg; // Linear penalty above 100g
			size_penalty = std::clamp(size_penalty, 0.1f, 1.0f);

			base_sprint *= size_penalty;
			max_sustainable_speed_m_s *= size_penalty;
			max_acceleration_m_s2 *= size_penalty;
		}

		// Oxygen diffusion limit (Kaiser et al. 2007; Harrison et al. 2006)
		// Tracheal system (passive diffusion) limits oxygen delivery
		// Critical PO2 increases with body size and metabolic rate

		// Estimate oxygen demand from metabolic rate
		float O2_demand_ml_per_min = out.metabolic.max_rate_W * 12.0f; // ~12 ml O2 per watt-min

		// Tracheal system efficiency decreases with size (Kaiser et al. 2007)
		// criticalPO2 = basePO2 × M^0.15 × sqrt(MMR/RMR)
		float base_critical_PO2_Pa = 5000.0f; // ~5 kPa minimum for small insects
		float size_factor = std::pow(out.physical.body_mass_kg * 1000.0f, 0.15f); // M in grams
		float activity_factor = std::sqrt(out.metabolic.aerobic_scope);
		float critical_O2_Pa = base_critical_PO2_Pa * size_factor * activity_factor;

		// If atmospheric O2 is below critical, performance degrades
		float atmospheric_O2_Pa = in.environment.pressure_Pa * 0.21f; // Assume 21% O2
		if (atmospheric_O2_Pa < critical_O2_Pa) {
			float hypoxia_penalty = atmospheric_O2_Pa / critical_O2_Pa;
			hypoxia_penalty = std::max(hypoxia_penalty, 0.2f); // Can't drop below 20%

			max_sustainable_speed_m_s *= hypoxia_penalty;
			max_acceleration_m_s2 *= hypoxia_penalty;
			// Sprint unaffected (anaerobic)
		}
	}

	// REPTILIA: Carrier's constraint (Carrier 1987)
	if (HasFlag(out.physical.clade, CF::REPTILIA)) {
		// Carrier's constraint: lateral bending during locomotion → breathing conflict
		// Already captured in sprawling vs upright posture (lines 63-68)
		// Low posture (<0.5) = sprawling = Carrier's constraint active

		// Just document here - constraint already applied in generic code
	}

	// MAMMALIA/AVES: Endotherm sustained performance
	if (HasFlag(out.physical.clade, CF::MAMMALIA) || HasFlag(out.physical.clade, CF::AVES)) {
		// Endotherms can sustain high speeds much longer than ectotherms
		// Already reflected in metabolic scaling and sprint duration
		// Just ensure consistency

		if (max_sprint_duration_s > 0.0f && max_sprint_duration_s < 60.0f) {
			// Endotherms can sprint for minutes, not seconds
			max_sprint_duration_s = std::max(max_sprint_duration_s, 60.0f);
		}
	}

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

	// ========== CLADE-SPECIFIC REFINEMENTS ==========
	// Applied after generic physics to add biological constraints

	using CF = CladeFlags;

	// ARTHROPODA: Insect jumping uses elastic energy storage (catapult mechanism)
	if (HasFlag(s.physical.clade, CF::ARTHROPODA)) {
		// Burrows (2006, 2009): Insects use elastic protein (resilin) in cuticle
		// Bennet-Clark & Lucey (1967): Resilin stores energy with 97% efficiency
		// Alexander (1988): Elastic storage enables power amplification 10-100x

		// Force mechanism to elastic catapult for arthropods
		mechanism = Output_Jumping::MechanismType::ELASTIC_CATAPULT;

		// Cuticle elastic modulus (Vincent & Wegst 2004)
		// Resilin: 0.6-2 MPa (very elastic)
		// Cuticle: 1-20 GPa (stiff for leverage)
		float cuticle_modulus_Pa = glm::mix(1e9f, 20e9f, in.structure_vs_weight);

		// Leg cross-section scales with (body_length)²
		float leg_cross_section_m2 = avg_leg_length_m * avg_leg_length_m * 0.01f;

		// Spring stiffness k = EA/L where E=modulus, A=area, L=length
		float leg_stiffness_N_m = (cuticle_modulus_Pa * leg_cross_section_m2) / avg_leg_length_m;

		// Maximum extension limited by joint geometry (typically 2-3x resting length)
		float max_extension_m = avg_leg_length_m * 2.0f;

		// Elastic energy stored: E = 0.5 * k * x²
		elastic_storage_J = 0.5f * leg_stiffness_N_m * max_extension_m * max_extension_m;

		// Bennet-Clark (1975): Fleas store ~10 μJ, achieve 50cm jumps from 1.5mm body
		// Power amplification = energy_release_time / energy_storage_time
		float storage_time_s = 0.5f; // Slow muscle contraction to load spring
		float release_time_s = 0.001f; // Latch mechanism releases in ~1ms (Burrows 2006)
		power_amplification_ratio = storage_time_s / release_time_s; // ~500x typical

		// Takeoff velocity from elastic energy (97% efficient)
		float efficiency = 0.97f; // Resilin efficiency (Bennet-Clark & Lucey 1967)
		takeoff_velocity_m_s = std::sqrt(2.0f * elastic_storage_J * efficiency / body_mass_kg);

		// Geometric constraint: can't exceed cuticle yield stress
		// Wainwright et al. (1976): Arthropod cuticle yields at ~100-400 MPa
		float max_stress_Pa = 200e6f; // Conservative mid-range
		float max_force_N = max_stress_Pa * leg_cross_section_m2;
		float stress_limited_energy_J = 0.5f * max_force_N * max_extension_m;
		elastic_storage_J = std::min(elastic_storage_J, stress_limited_energy_J);

		// Recalculate velocity with stress limit
		takeoff_velocity_m_s = std::sqrt(2.0f * elastic_storage_J * efficiency / body_mass_kg);

		// Cap at realistic values (Burrows 2006: locusts achieve ~3.5 m/s)
		takeoff_velocity_m_s = std::min(takeoff_velocity_m_s, 5.0f);

		// Recalculate jump height and distance
		float v_vertical = takeoff_velocity_m_s * std::sin(takeoff_angle_rad);
		max_jump_height_m = (v_vertical * v_vertical) / (2.0f * in.environment.gravity_m_s2);

		float v_horizontal = takeoff_velocity_m_s * std::cos(takeoff_angle_rad);
		float flight_time_s = 2.0f * v_vertical / in.environment.gravity_m_s2;
		max_jump_distance_m = v_horizontal * flight_time_s;

		// Recovery time longer for elastic catapult (need to reload spring)
		recovery_time_s = storage_time_s + 0.5f; // Prep time + reset time
	}

	// AMPHIBIA: Frogs also use elastic catapult but with different implementation
	if (HasFlag(s.physical.clade, CF::AMPHIBIA)) {
		// Frogs use elastic tendons (not cuticle like insects)
		// Marsh & John-Alder (1994): Frog jump performance scales M^0.17

		// If small amphibian (<100g), likely uses elastic storage
		if (body_mass_kg < 0.1f) {
			mechanism = Output_Jumping::MechanismType::ELASTIC_CATAPULT;

			// Frog tendon elastic modulus ~1-2 GPa (less than arthropod cuticle)
			float tendon_modulus_Pa = 1.5e9f;
			float tendon_area_m2 = avg_leg_length_m * avg_leg_length_m * 0.005f;
			float spring_stiffness = (tendon_modulus_Pa * tendon_area_m2) / avg_leg_length_m;

			// Frogs achieve ~70-90% efficiency (lower than insects)
			float efficiency = 0.80f;
			elastic_storage_J = 0.5f * spring_stiffness * (avg_leg_length_m * 3.0f) * (avg_leg_length_m * 3.0f);

			// Peplowski & Marsh (1997): Frogs achieve 10-20x power amplification
			power_amplification_ratio = 15.0f;

			// Enhance jump performance
			takeoff_velocity_m_s *= 1.3f; // 30% boost from elastic storage
		}
	}

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
