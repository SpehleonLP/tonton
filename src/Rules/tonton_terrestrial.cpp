#include "tonton_terrestrial.h"
#include "../../include/tonton_analysis.h"
#include "Memos/tonton_armaturememo.h"
#include "Rules/tonton_climbing.h"
#include "Rules/tonton_scratch.h"
#include "tonton_input.h"
#include "tonton_skinnedmesh.h"
#include <cfloat>

using SF = TonTon::SemanticFlags;

namespace TonTon
{
static std::vector<Analysis_Aerial::Wing> GetLegs(Input const& in);
}

std::optional<TonTon::Analysis_Terrestrial>  TonTon::ComputeTerrestrial(Input const& in, Scratch &out)
{	
	auto appendages = GetAppendages(in, GetChainsFromRoot(in, SF::LIMB, SF::CONTACT));
	auto legs = ComputeManipulation(in, appendages);
	
	if(legs.empty())
		return {};
	
	length_m functional_length{FLT_MAX};
	auto posture = 0.0f;

	for(auto const& leg : legs)
	{
		functional_length = std::min(functional_length, leg.stretched_length_m);
		auto vec = glm::normalize(position[leg.tip] - position[leg.root]);
		posture = std::max(posture, 1.f - std::abs(vec.y));
	}

	// HYBRID SPEED SCALING: Mass allometry + leg length correction
	//
	// Base speed from empirical allometric scaling (Garland 1983):
	// log10(speed) = 0.17 × log10(mass) + C
	// where C varies by clade and lifestyle:
	// - Cursorial mammals: C ≈ 1.1 (fast runners: cats, dogs, ungulates)
	// - General mammals: C ≈ 0.9
	// - Reptiles: C ≈ 0.7
	// - Aquatic-adapted (flippers/fins): C ≈ 0.4 (penguins, seals - poor land locomotion)
	//
	// Then correct for leg length deviation from expected proportions

	using CF = CladeFlags;

	auto mass_exponent = 0.17f;
	auto base_constant = 0.9f; // Default for general vertebrates

	// Clade-specific adjustments
	if (HasFlag(out.physical.clade, CF::MAMMALIA)) {
		base_constant = 1.05f; // Mammals are generally fast runners
	} else if (HasFlag(out.physical.clade, CF::AVES)) {
		base_constant = 0.85f; // Ground birds vary widely
	} else if (HasFlag(out.physical.clade, CF::REPTILIA)) {
		base_constant = 0.70f; // Reptiles generally slower
	}

	// Base speed from mass allometry (in m/s)
	auto allometric_speed_m_s = std::pow(10.0f,
		mass_exponent * std::log10(float(out.physical.body_mass_kg)) + base_constant);

	// Expected leg length from body mass (assuming isometric scaling)
	// leg_length ∝ mass^(1/3) for isometrically scaled animals
	// Typical mammals: leg_length ≈ 0.3 × mass^(1/3)
	
	// Scale Effects between Body Size and Limb Design in Quadrupedal Mammals
	// Brandon M. Kilbourne ,Louwrens C. Hoffman  
	/*
	 * Both fore- and hindlimb length are positively allometric with respect to body mass, (P<0.05) with slopes of
	 * 0.40 and 0.37, respectively (Figs. 4A and 5A; Table 3). For mammalian subgroups, forelimb slopes range from 
	 * 0.30 (Rodentia) to 0.42 (Carnivora), while hindlimb slopes range from 0.27 (Rodentia) to 0.42 (Carnivora). 
	 * Apart from cursorial mammals, fore- and hindlimb slopes for each of the subgroups have wide confidence limits, 
	 * likely due in part to their smaller sample sizes. For each of these groups, the slopes do not significantly 
	 * depart from geometric similarity (P>0.05). For cursors, forelimb length is isometric with body mass, while 
	 * hindlimb length is negatively allometric with body mass. 
	 * 
	 * (carnivoria R^2 = 0.78) 
	 */ 

	length_m expected_leg_m = 0.151f * std::pow(float(out.physical.body_mass_kg), 0.42);

	// Leg length correction factor
	// Short legs (penguins, seals): functional_length < expected → reduce speed
	// Long legs (kangaroos, ostriches): functional_length > expected → increase speed
	auto leg_length_ratio = functional_length / expected_leg_m;
	auto leg_correction = std::sqrt(leg_length_ratio); // sqrt because speed ∝ sqrt(leg_length) from Froude
	leg_correction = std::clamp(leg_correction, 0.3f, 2.0f); // Don't go crazy with extremes

	auto base_sprint = allometric_speed_m_s * leg_correction;

	// Aquatic-adapted animals with short legs: poor terrestrial locomotion
	if (has_flippers && leg_length_ratio < 0.6f) {
		// Penguins, seals waddle slowly on land
		base_sprint *= 0.35f; // Massive penalty for flipper-based waddling
	}

	// Posture penalty for sprawling (reduces efficiency, not just stride length)
	base_sprint *= glm::mix(1.0f, 0.7f, posture);
	
	// Posture affects sustained speed more than sprint
	auto max_sustainable_speed_m_s = base_sprint * glm::mix(0.6f, 0.3f,  posture);
		
	time_s max_sprint_duration_s = -1;
	time_s recovery_time_s = -1;
	if (posture > 0.5f) { // sprawling
		// Carrier's constraint: lateral bending -> breathing conflict
		auto mass_factor = glm::min(1.0f, float(out.physical.body_mass_kg) / 50.0f); // worse for heavier animals
		
		max_sprint_duration_s = glm::mix(30.0f, 3.0f, mass_factor); // heavy lizards tire fast
		recovery_time_s = max_sprint_duration_s * 2.0f;
	}  
		
	// Turning radius limited by centripetal force
	// F_centripetal = m*v²/r, limited by friction coefficient * weight
	auto friction_coeff = glm::mix(0.8f, 0.6f, posture); // sprawling has lower CoM
	auto max_lateral_accel = friction_coeff * 9.81f;
	
	length_m min_turning_radius_m = (base_sprint * base_sprint) / max_lateral_accel;
	
	// Forward acceleration limited by muscle force
	// Very rough: force ~ cross_sectional_area of muscles ~ mass^(2/3)
	auto force_to_mass_ratio = 15.0f * std::pow(float(out.physical.body_mass_kg), -0.33f); // N/kg
	acceleration_m_s2 max_acceleration_m_s2 = force_to_mass_ratio * glm::mix(1.0f, 0.7f, posture);

	// ========== CLADE-SPECIFIC REFINEMENTS ==========
	// Applied after generic terrestrial locomotion physics

	using CF = CladeFlags;

	// ARTHROPODA: Temperature-dependent performance (Q10 effects)
	if (HasFlag(out.physical.clade, CF::ARTHROPODA)) {
		// Q10 temperature coefficient (Angilletta et al. 2002)
		// Arthropod performance changes 2-3x per 10K temperature change
		// Smaller arthropods have HIGHER Q10 due to lower thermal inertia

		auto q10 = 2.5f; // Typical mid-range (Huey & Kingsolver 1989)

		// Thermal mass effect: smaller = faster heating/cooling
		if (out.physical.body_mass_kg < 0.001f) {
			q10 = 3.0f; // Small insects respond faster to temperature
		}

		// Temperature difference from reference (25°C)
		temp_K reference_temp_K = 298.15f;
		auto temp_diff_K = in.environment.temperature_K - reference_temp_K;

		// Performance multiplier from Q10 (Huey & Stevenson 1979)
		auto q10_multiplier = std::pow(q10, float(temp_diff_K) / 10.0f);

		// Apply to all speed/acceleration metrics
		base_sprint *= q10_multiplier;
		max_sustainable_speed_m_s *= q10_multiplier;
		max_acceleration_m_s2 *= q10_multiplier;

		// Clamp to reasonable limits (arthropods inactive below ~5°C, above ~50°C)
		if (in.environment.temperature_K < 278.15f) { // Below 5°C
			auto cold_penalty = float(in.environment.temperature_K - 273.15f) / 5.0f;
			cold_penalty = std::max(cold_penalty, 0.1f);
			base_sprint *= cold_penalty;
			max_sustainable_speed_m_s *= cold_penalty;
		}

		if (in.environment.temperature_K > 323.15f) { // Above 50°C
			auto heat_penalty = (323.15f - float(in.environment.temperature_K)) / 10.0f;
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

			auto size_penalty = 0.1f / float(out.physical.body_mass_kg); // Linear penalty above 100g
			size_penalty = std::clamp(size_penalty, 0.1f, 1.0f);

			base_sprint *= size_penalty;
			max_sustainable_speed_m_s *= size_penalty;
			max_acceleration_m_s2 *= size_penalty;
		}

		// Oxygen diffusion limit (Kaiser et al. 2007; Harrison et al. 2006)
		// Tracheal system (passive diffusion) limits oxygen delivery
		// Critical PO2 increases with body size and metabolic rate

		// Estimate oxygen demand from metabolic rate
		auto O2_demand_ml_per_min = out.metabolic.max_rate_W * 12.0f; // ~12 ml O2 per watt-min

		// Tracheal system efficiency decreases with size (Kaiser et al. 2007)
		// criticalPO2 = basePO2 × M^0.15 × sqrt(MMR/RMR)
		pressure_Pa base_critical_PO2_Pa = 5000.0f; // ~5 kPa minimum for small insects
		auto size_factor = std::pow(float(out.physical.body_mass_kg) * 1000.0f, 0.15f); // M in grams
		auto activity_factor = std::sqrt(out.metabolic.aerobic_scope());
		pressure_Pa critical_O2_Pa = base_critical_PO2_Pa * size_factor * activity_factor;

		// If atmospheric O2 is below critical, performance degrades
		auto atmospheric_O2_Pa = in.environment.fluidPressure_Pa * 0.21f; // Assume 21% O2
		if (atmospheric_O2_Pa < critical_O2_Pa) {
			auto hypoxia_penalty = atmospheric_O2_Pa / critical_O2_Pa;
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
		// Low posture (>0.5) = sprawling = Carrier's constraint active

		// Just document here - constraint already applied in generic code
	}

	// MAMMALIA/AVES: Endotherm sustained performance
	if (HasFlag(out.physical.clade, CF::MAMMALIA) || HasFlag(out.physical.clade, CF::AVES)) {
		// Endotherms can sustain high speeds much longer than ectotherms
		// Already reflected in metabolic scaling and sprint duration
		// Just ensure consistency

		if (max_sprint_duration_s > 0.0f && max_sprint_duration_s < 60.0f) {
			// Endotherms can sprint for minutes, not seconds
			max_sprint_duration_s = std::max(max_sprint_duration_s, time_s(60.0f));
		}
	}

	return Analysis_Terrestrial
	{
		.legs=shared_array<Analysis_Manipulator>::FromArray(legs),
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

std::optional<TonTon::Analysis_Jumping>  TonTon::ComputeJumping(Input const& in, Scratch & s)
{
	// Jumping requires terrestrial locomotion
	if(!s.terrestrial.has_value())
		return {};

	auto const& terrestrial = s.terrestrial.value();

	if(terrestrial.legs.empty())
		return {};

	auto body_mass_kg = s.physical.body_mass_kg;
//	auto body_weight_N = body_mass_kg * in.environment.gravity_m_s2;

	// 1. DETERMINE JUMP MECHANISM
	Analysis_Jumping::MechanismType mechanism = Analysis_Jumping::MechanismType::MUSCLE_DIRECT;

	// 2. CALCULATE AVAILABLE JUMP FORCE
	// Force comes from leg extension (quadriceps, gastrocnemius equivalents)
	// NOTE: max_lift_force_N is calculated for tip force (long lever arm)
	// For jumping, we need leg extension force which is much higher
	// because the effective moment arm is much shorter (push through ankle/knee)

	force_N total_leg_force_N = 0;
	length_m avg_leg_length_m = 0;
	int leg_count = terrestrial.legs.size();
	length_m min_rest_length = FLT_MAX;
	length_m max_stretched_length = -FLT_MAX;
	length_m min_stretched_length = FLT_MAX;
	length_m max_rest_length = -FLT_MAX;
	auto total_compression_ratio = 0;

	for(auto const& leg : terrestrial.legs)
	{
		// For jumping, use grip force as proxy for leg extension force
		// Grip force = muscle_cross_section * muscle_stress
		// Leg extensors (quadriceps, gastrocnemius) are typically ~1.5-2x stronger than flexors
		auto leg_extension_force_N = leg.max_grip_force_N * 1.5f;

		total_leg_force_N += leg_extension_force_N;
		avg_leg_length_m += leg.stretched_length_m;

		min_rest_length = std::min(min_rest_length, leg.rest_length_m);
		max_stretched_length = std::max(max_stretched_length, leg.stretched_length_m);
		min_stretched_length = std::min(min_stretched_length, leg.stretched_length_m);
		max_rest_length = std::max(max_rest_length, leg.rest_length_m);

		auto compression_ratio = leg.stretched_length_m / leg.rest_length_m;
		total_compression_ratio += compression_ratio;
	}
	avg_leg_length_m /= leg_count;
	auto avg_compression_ratio = total_compression_ratio / leg_count;

	auto leg_length_asymmetry = max_stretched_length / min_stretched_length;
	
	// DETERMINE JUMP MECHANISM based on morphology
	// Elastic catapult mechanism requires high leg compression for energy storage
	// Frogs: compression ratio ~2.0-3.0 (legs fold to 33-50% of stretched length)
	// Cats/humans: compression ratio ~1.05-1.2 (minimal compression)
	// Grasshoppers/fleas: compression ratio >2.0

	using CF = CladeFlags;

	if(avg_compression_ratio > 1.8f)
	{
		// High compression indicates elastic energy storage capability
		mechanism = Analysis_Jumping::MechanismType::ELASTIC_CATAPULT;
	}
	else
	{
		mechanism = Analysis_Jumping::MechanismType::MUSCLE_DIRECT;
	}

	// Hydraulic for very small arthropods (< 1g)
	// Jumping spiders use hydraulic leg extension
	if(body_mass_kg < 0.001f && HasFlag(s.physical.clade, CF::ARTHROPODA))
	{
		mechanism = Analysis_Jumping::MechanismType::HYDRAULIC;
	}

	// Not all legs contribute equally - use ~70% of total force
	auto effective_force_N = total_leg_force_N * 0.7f;

	// 3. CALCULATE TAKEOFF VELOCITY
	// Work-energy: F*d = 0.5*m*v²
	// Stroke distance ≈ leg_length * extension_ratio

	// Typical leg extension during jump
	auto stroke_distance_m =  max_stretched_length - (min_rest_length * 0.3f);
	
	// Work done = force * distance
	auto work_J = effective_force_N * stroke_distance_m;

	// Kinetic energy at takeoff = work - losses
	auto efficiency = 0.65f; // ~35% losses to heat, internal work

	if(mechanism == Analysis_Jumping::MechanismType::ELASTIC_CATAPULT)
	{
		// Elastic storage is more efficient
		efficiency = 0.85f;
	}
	else if(mechanism == Analysis_Jumping::MechanismType::HYDRAULIC)
	{
		// Hydraulic is very efficient but limited by fluid volume
		efficiency = 0.75f;
	}

	energy_J kinetic_energy_J = work_J * efficiency;
	// v = sqrt(2 * KE / m)
	velocity_m_s takeoff_velocity_m_s = sqrt(2.0f * kinetic_energy_J / body_mass_kg);

	// Apply muscle quality scaling
	takeoff_velocity_m_s *= glm::mix(0.7f, 1.3f, in.muscle_quality);

	// 4. CALCULATE JUMP HEIGHT AND DISTANCE
	// Optimal angle for maximum distance: 45 degrees
	// Optimal angle for maximum height: 90 degrees
	// Most animals compromise: 60-70 degrees

	auto takeoff_angle_rad = M_PI / 3.0f; // 60 degrees

	// Vertical component of velocity
	velocity_m_s v_vertical = takeoff_velocity_m_s * std::sin(takeoff_angle_rad);

	// Max height: h = v_y² / (2*g)
	length_m max_jump_height_m = (v_vertical * v_vertical) / (2.0f * in.environment.gravity_m_s2);

	// Horizontal component
	velocity_m_s v_horizontal = takeoff_velocity_m_s * std::cos(takeoff_angle_rad);

	// Flight time: t = 2 * v_y / g
	time_s flight_time_s = 2.0f * v_vertical / in.environment.gravity_m_s2;

	// Horizontal distance
	length_m max_jump_distance_m = v_horizontal * flight_time_s;

	// 5. ELASTIC ENERGY STORAGE (for catapult mechanism)
	energy_J elastic_storage_J = 0;
	auto power_amplification_ratio = 1.0f;

	if(mechanism == Analysis_Jumping::MechanismType::ELASTIC_CATAPULT)
	{
		// Elastic tendons can store energy over longer prep time
		// Then release it rapidly for high power output

		elastic_storage_J = work_J; // Total energy stored in tendons

		// Power amplification = stored_energy / (muscle_power * contraction_time)
		// Typical: 2-20x for various animals
		auto muscle_contraction_time_s = 0.1f; // ~100ms for direct muscle
		auto tendon_release_time_s = 0.01f;     // ~10ms for elastic release

		power_amplification_ratio = muscle_contraction_time_s / tendon_release_time_s;

		// Enhance jump performance for catapult mechanism
		takeoff_velocity_m_s *= std::sqrt(power_amplification_ratio) * 0.5f;
		max_jump_height_m = (v_vertical * v_vertical) / (2.0f * in.environment.gravity_m_s2);
		max_jump_distance_m = v_horizontal * flight_time_s;
	}

	// 6. RECOVERY TIME
	// Time to recharge for next jump
	// Based on metabolic recovery + muscle relaxation

	time_s recovery_time_s = 2.0f; // Default: ~2 seconds

	if(mechanism == Analysis_Jumping::MechanismType::MUSCLE_DIRECT)
	{
		// Direct muscle: fast recovery if aerobic
		recovery_time_s = time_s(1.0f) + (kinetic_energy_J / s.metabolic.available_muscle_power_W);
	}
	else if(mechanism == Analysis_Jumping::MechanismType::ELASTIC_CATAPULT)
	{
		// Elastic storage: slower recovery (need to re-stretch tendons)
		recovery_time_s = 3.0f;
	}
	else if(mechanism == Analysis_Jumping::MechanismType::HYDRAULIC)
	{
		// Hydraulic: slow recovery (refill hemolymph pressure)
		recovery_time_s = 5.0f;
	}

	// Scale by endurance_vs_power
	recovery_time_s *= glm::mix(0.6f, 1.4f, 1.0f - in.behavior.endurance_vs_power);

	// ========== CLADE-SPECIFIC REFINEMENTS ==========
	// Applied after generic physics to add biological constraints

	// ARTHROPODA: Insect jumping uses elastic energy storage (catapult mechanism)
	if (HasFlag(s.physical.clade, CF::ARTHROPODA) && mechanism == Analysis_Jumping::MechanismType::ELASTIC_CATAPULT) {
		// Burrows (2006, 2009): Insects use elastic protein (resilin) in cuticle
		// Bennet-Clark & Lucey (1967): Resilin stores energy with 97% efficiency
		// Alexander (1988): Elastic storage enables power amplification 10-100x

		// Cuticle elastic modulus (Vincent & Wegst 2004)
		// Resilin: 0.6-2 MPa (very elastic)
		// Cuticle: 1-20 GPa (stiff for leverage)
		pressure_Pa cuticle_modulus_Pa = glm::mix(1e9f, 20e9f, in.structure_vs_weight);

		// Leg cross-section scales with (body_length)²
		auto leg_cross_section_m2 = avg_leg_length_m * avg_leg_length_m * 0.01f;

		// Spring stiffness k = EA/L where E=modulus, A=area, L=length
		auto leg_stiffness_N_m = (cuticle_modulus_Pa * leg_cross_section_m2) / avg_leg_length_m;

		// Maximum extension limited by joint geometry (typically 2-3x resting length)
		auto max_extension_m = avg_leg_length_m * 2.0f;

		// Elastic energy stored: E = 0.5 * k * x²
		elastic_storage_J = 0.5f * leg_stiffness_N_m * max_extension_m * max_extension_m;

		// Bennet-Clark (1975): Fleas store ~10 μJ, achieve 50cm jumps from 1.5mm body
		// Power amplification = energy_release_time / energy_storage_time
		auto storage_time_s = 0.5f; // Slow muscle contraction to load spring
		auto release_time_s = 0.001f; // Latch mechanism releases in ~1ms (Burrows 2006)
		power_amplification_ratio = storage_time_s / release_time_s; // ~500x typical

		// Takeoff velocity from elastic energy (97% efficient)
		auto efficiency = 0.97f; // Resilin efficiency (Bennet-Clark & Lucey 1967)
		takeoff_velocity_m_s = sqrt(2.0f * elastic_storage_J * efficiency / body_mass_kg);

		// Geometric constraint: can't exceed cuticle yield stress
		// Wainwright et al. (1976): Arthropod cuticle yields at ~100-400 MPa
		pressure_Pa max_stress_Pa = 200e6f; // Conservative mid-range
		force_N max_force_N = max_stress_Pa * leg_cross_section_m2;
		energy_J stress_limited_energy_J = 0.5f * max_force_N * max_extension_m;
		elastic_storage_J = std::min(elastic_storage_J, stress_limited_energy_J);

		// Recalculate velocity with stress limit
		takeoff_velocity_m_s = sqrt(2.0f * elastic_storage_J * efficiency / body_mass_kg);

		// Cap at realistic values (Burrows 2006: locusts achieve ~3.5 m/s)
		takeoff_velocity_m_s = std::min(takeoff_velocity_m_s, velocity_m_s(5.0f));

		// Recalculate jump height and distance
		auto v_vertical = takeoff_velocity_m_s * std::sin(takeoff_angle_rad);
		max_jump_height_m = (v_vertical * v_vertical) / (2.0f * in.environment.gravity_m_s2);

		auto v_horizontal = takeoff_velocity_m_s * std::cos(takeoff_angle_rad);
		auto flight_time_s = 2.0f * v_vertical / in.environment.gravity_m_s2;
		max_jump_distance_m = v_horizontal * flight_time_s;

		// Recovery time longer for elastic catapult (need to reload spring)
		recovery_time_s = storage_time_s + 0.5f; // Prep time + reset time
	}

	// AMPHIBIA: Frogs use elastic catapult when morphology supports it
	if (HasFlag(s.physical.clade, CF::AMPHIBIA) && mechanism == Analysis_Jumping::MechanismType::ELASTIC_CATAPULT) {
		// Frogs use elastic tendons (not cuticle like insects)
		// Marsh & John-Alder (1994): Frog jump performance scales M^0.17

		// Frog tendon elastic modulus ~1-2 GPa (less than arthropod cuticle)
		pressure_Pa tendon_modulus_Pa = 1.5e9f;
		area_m2 tendon_area_m2 = avg_leg_length_m * avg_leg_length_m * 0.005f;
		auto spring_stiffness = (tendon_modulus_Pa * tendon_area_m2) / avg_leg_length_m;

		// Frogs achieve ~70-90% efficiency (lower than insects)
		auto efficiency = 0.80f;
		elastic_storage_J = 0.5f * spring_stiffness * (avg_leg_length_m * 3.0f) * (avg_leg_length_m * 3.0f);

		// Peplowski & Marsh (1997): Frogs achieve 10-20x power amplification
		power_amplification_ratio = 15.0f;

		// Enhance jump performance
		takeoff_velocity_m_s *= 1.3f; // 30% boost from elastic storage
	}

	// 7. SANITY CHECKS
	// Jump height should be reasonable (< 50x body length for most animals)
	auto max_reasonable_height = s.physical.body_length_m * 50.0f;
	if(max_jump_height_m > max_reasonable_height)
	{
		// Scale back unrealistic jumps
		auto scale = max_reasonable_height / max_jump_height_m;
		max_jump_height_m *= scale;
		max_jump_distance_m *= scale;
		takeoff_velocity_m_s *= std::sqrt(scale);
	}

	return Analysis_Jumping{
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
