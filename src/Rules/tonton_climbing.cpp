#include "tonton_climbing.h"
#include "../../include/tonton_skinnedmesh.h"
#include "../../include/tonton_analysis.h"
#include "Memos/tonton_armaturememo.h"
#include "Memos/tonton_skinnedmeshmemo.h"
#include "Rules/tonton_scratch.h"
#include "dodeedum.h"
#include "tonton_builder.h"
#include "tonton_input.h"
#include <cfloat>

using SF = TonTon::SemanticFlags;

std::optional<TonTon::Analysis_Climbing>  TonTon::ComputeClimbing(Input const& in, Scratch & s)
{
	SF constexpr NOT_LIMB_FLAGS = SF(
		int64_t(SF::HEAD)|
		int64_t(SF::NECK)|
		int64_t(SF::SPINE)|
		int64_t(SF::ABDOMEN)
	);

// re-get appendages because there may be overlap between manipulators and legs.
	auto limbs = ComputeManipulation(in, SF::GRASPER|SF::CONTACT);

	if(limbs.empty())
		return {};

	// 1. CLIMBING CAPABILITY CHECK
	// Need sufficient grip/adhesion force to support body weight
	// Use all limbs (not just contact limbs) since climbing uses hands/feet
	// BUT exclude wings/flippers - they can't grip surfaces!
	auto body_weight_N = s.physical.body_mass_kg * in.environment.gravity_m_s2;

	force_N total_grip_force = 0;
	force_N total_adhesion_force = 0;
	bool any_claws = false;
	bool any_wet_grip = false;
	bool any_setae = false;
	bool any_suckers = false;

	int limb_count = 0;

	for(auto const& limb : limbs)
	{
		// Exclude wings and fins - they're for flight/swimming, not climbing
		// Exception: birds with zygodactyl feet (parrots, woodpeckers) can climb
		bool is_wing = HasFlag(limb.subtree_flags, SF::WING);
		bool is_fin = HasFlag(limb.subtree_flags, SF::FIN);
		//
		bool is_grasper = HasFlag(limb.subtree_flags, SF::GRASPER|SF::DIGIT);

		if((is_wing || is_fin) && !(is_grasper)) {
			continue; // Skip flippers and wings
		}

		total_grip_force += limb.max_grip_force_N;
		total_adhesion_force += limb.max_adhesion_force_N;
		any_claws |= limb.has_claws;
		any_wet_grip |= limb.has_wet_grip;
		any_setae |= limb.has_setae;
		any_suckers |= limb.has_suckers;
		limb_count++;
	}

	// If no non-wing limbs, can't climb
	if(limb_count == 0)
		return {};

	// Total support force (assuming 3-point contact, 1 limb moving)
	int contact_limbs = std::max(1, limb_count - 1);
	auto effective_grip = (total_grip_force / limb_count) * contact_limbs;
	auto effective_adhesion = (total_adhesion_force / limb_count) * contact_limbs;
	auto total_support = effective_grip + effective_adhesion;

	// Need at least 1.5x safety margin for climbing
	auto safety_margin = total_support / body_weight_N;

	if(safety_margin < 1.5f)
		return {};  // Cannot climb safely

	// 2. CLIMB SPEED
	// Based on Alexander (2003) - climbing speed ~ limb_length * frequency
	// Frequency limited by force-to-weight ratio and limb reach

	length_m avg_limb_length = 0;
	for(auto const& limb : limbs)
		avg_limb_length += limb.stretched_length_m;
	avg_limb_length /= limb_count;

	// Climbing frequency (cycles/second) based on muscle power
	// Hauling body weight up requires work = m*g*h per stroke
	auto stroke_height_m = avg_limb_length * 0.5f; // half limb reach per cycle
	auto work_per_stroke_J = body_weight_N * stroke_height_m;

	// Available power from Input::metabolic_efficiency and muscle quality
	auto muscle_power_W = s.metabolic.available_muscle_power_W * in.climbing_ability;

	// Climbing efficiency ~25% (lots of isometric holds)
	auto climbing_efficiency = 0.25f;
	auto effective_power_W = muscle_power_W * climbing_efficiency;

	// Max frequency = power / work_per_stroke
	auto max_frequency_Hz = effective_power_W / work_per_stroke_J;

	// But also limited by limb dynamics (can't move faster than ~3 Hz for most animals)
	max_frequency_Hz = std::min<freq_Hz>(max_frequency_Hz, 3.0f);

	velocity_m_s max_climb_speed_m_s = stroke_height_m * max_frequency_Hz;

	// Adjust for climbing ability input
	max_climb_speed_m_s *= glm::mix(0.3f, 1.0f, in.climbing_ability);

	// 3. MAX CLIMB ANGLE
	// Vertical climbing requires safety_margin >= 2.0
	// Horizontal requires safety_margin >= 1.5
	// Interpolate between
	angle_rad max_angle_rad = M_PI / 2.0f; // vertical

	if(safety_margin < 2.0f)
	{
		// Clamp max angle based on available force
		// At angle θ from horizontal: normal_force = weight * cos(θ)
		// Need: grip_force >= weight * sin(θ) + friction_needed
		// Simplified: max angle when total_force = safety_margin * weight

		float angle_factor = (safety_margin - 1.5f) / 0.5f; // 0 at 1.5, 1 at 2.0
		angle_factor = glm::clamp(angle_factor, 0.0f, 1.0f);
		max_angle_rad = glm::mix(M_PI / 4.0f, M_PI / 2.0f, angle_factor);
	}

	// 4. HEAD-FIRST DESCENT
	// Requires either:
	// - Strong claws that can hook (most common)
	// - Very high adhesion force (gecko, spider)
	// - Prehensile limbs with high grip strength

	bool can_descend_head_first = false;

	if(any_claws)
	{
		can_descend_head_first = true;
	}
	else if(any_setae || any_suckers)
	{
		// Need very high adhesion for head-first descent without claws
		if(total_adhesion_force > body_weight_N * 3.0f)
			can_descend_head_first = true;
	}
	else if(total_grip_force > body_weight_N * 4.0f)
	{
		// Strong grippers (primates) can descend slowly
		can_descend_head_first = true;
	}

	// 5. SUBSTRATE REQUIREMENTS
	auto min_roughness_required = 0.0f;
	bool requires_dry_surface = false;

	if(any_claws)
	{
		// Claws need rough surface to penetrate/hook
		// Roughness scale: 0=smooth glass, 0.5=wood, 1.0=rough bark
		min_roughness_required = 0.3f;

		// Larger claws need more roughness
		auto avg_claw_force = total_grip_force / limb_count;
		auto body_weight_per_limb = body_weight_N / limb_count;

		if(avg_claw_force < body_weight_per_limb * 2.0f)
		{
			// Weak claws need rougher surfaces
			min_roughness_required = 0.6f;
		}
	}
	else if(any_setae)
	{
		// Gecko setae work on very smooth surfaces
		min_roughness_required = 0.0f;
		requires_dry_surface = true; // Van der Waals forces weakened by water
	}
	else if(any_wet_grip)
	{
		// Tree frog adhesion works best on smooth surfaces
		min_roughness_required = 0.0f;
		requires_dry_surface = false; // Actually needs moisture!
	}
	else if(any_suckers)
	{
		// Suction cups need smooth, non-porous surfaces
		min_roughness_required = 0.0f;
		requires_dry_surface = false;
	}
	else
	{
		// Pure friction grip - needs some texture
		min_roughness_required = 0.4f;
	}

	Analysis_Climbing result{
		.limbs = shared_array<Analysis_Manipulator>::FromArray(limbs),
		.max_climb_speed_m_s = max_climb_speed_m_s,
		.max_climb_angle_rad = max_angle_rad,
		.can_descend_head_first = can_descend_head_first,
		.min_roughness_required = min_roughness_required,
		.requires_dry_surface = requires_dry_surface
	};

	// ========== CLADE-SPECIFIC REFINEMENTS ==========
	using CF = CladeFlags;
	auto clade = s.physical.clade;

	// MAMMALIA (Primates): Exceptional grippers with high intelligence
	if (HasFlag(clade, CF::MAMMALIA)) {
		// Primates have excellent grip strength and prehensile abilities
		// Hanna et al. (2017): Primates can support 2-3x body weight per hand

		bool has_graspers = false;
		for(auto const& limb : limbs) {
			if (limb.has_thumb || limb.max_grip_force_N > body_weight_N) {
				has_graspers = true;
				break;
			}
		}

		if (has_graspers) {
			// Primates are excellent climbers
			// Fleagle (1976): Primates climb at 0.3-0.8 m/s in trees
			result.max_climb_speed_m_s *= 1.4f;
			result.can_descend_head_first = true;

			// Can climb smooth-ish surfaces (tree bark)
			result.min_roughness_required *= 0.7f;

			// Smaller primates (squirrel monkeys, gibbons) are exceptional
			if (s.physical.body_mass_kg < 10.0f) {
				result.max_climb_speed_m_s *= 1.3f;
			}
		}

		// Cats (clawed climbers)
		// Taylor (1970): Cats climb using retractable claws
		if (any_claws && s.physical.body_mass_kg < 50.0f) {
			result.can_descend_head_first = true; // Cats descend (though awkwardly)
			result.min_roughness_required = 0.4f; // Need bark texture
		}

		// Bears and large mammals
		// Beringer et al. (2005): Black bears climb trees for refuge
		if (any_claws && s.physical.body_mass_kg > 50.0f) {
			// Large mammals climb slowly but powerfully
			result.max_climb_speed_m_s *= 0.7f; // Slower due to mass
			result.min_roughness_required = 0.5f; // Need deep bark grooves
		}
	}

	// REPTILIA (Squamata): Geckos and lizards
	if (HasFlag(clade, CF::REPTILIA)) {
		// Geckos have setae (van der Waals adhesion)
		if (any_setae) {
			// Autumn et al. (2000): Gecko setae generate 10 N/cm²
			// Can climb ANY solid surface (even glass)
			result.min_roughness_required = 0.0f; // Smooth surfaces OK
			result.requires_dry_surface = true; // Water disrupts van der Waals

			// Geckos are exceptional climbers
			// Autumn et al. (2006): Tokay gecko climbs 1 m/s vertically
			result.max_climb_speed_m_s = std::max<velocity_m_s>(result.max_climb_speed_m_s, 0.8f);
			result.max_climb_angle_rad = M_PI / 2.0f; // Can climb overhangs
			result.can_descend_head_first = true;

			// Geckos can run upside-down on ceilings
			// Autumn et al. (2006): 6.2x safety margin with 4 feet
			if (safety_margin > 4.0f) {
				result.can_climb_inverted = true;
			}
		}

		// Non-gecko lizards (clawed climbers)
		// Zaaf et al. (1999): Lizards use claws and lateral undulation
		if (any_claws && !any_setae) {
			result.max_climb_speed_m_s *= 1.2f; // Agile climbers
			result.min_roughness_required = 0.4f; // Need texture for claws
		}

		// Snakes (concertina climbing)
		if (!s.terrestrial.has_value() || s.terrestrial->legs.empty()) {
			// Snakes climb via concertina and lateral wedging
			// Astley & Jayne (2007): Snakes climb at 0.05-0.15 m/s
			result.max_climb_speed_m_s = s.physical.body_length_m / time_s(10.0f);
			result.min_roughness_required = 0.6f; // Need rough bark
			result.can_descend_head_first = true; // Excellent control

			// Tree snakes have specialized ventral scales
			// Lillywhite & Henderson (1993): Keeled ventral scales
			if (s.physical.body_mass_kg < 2.0f) {
				result.max_climb_speed_m_s *= 1.5f; // Lighter = faster
			}
		}
	}

	// AMPHIBIA: Tree frogs
	if (HasFlag(clade, CF::AMPHIBIA)) {
		// Tree frogs have wet adhesion (capillary + mucus)
		if (any_wet_grip) {
			// Federle et al. (2006): Tree frog toe pads 1-5 N/cm²
			// Excellent on smooth wet surfaces (leaves)
			result.min_roughness_required = 0.0f; // Smooth OK
			result.requires_dry_surface = false; // Actually NEEDS moisture!

			// Tree frogs are excellent jumpers while climbing
			// Nauwelaerts & Aerts (2006): Tree frogs leap between branches
			result.max_climb_speed_m_s *= 1.3f; // Jumping-assisted climbing
			result.can_climb_smooth_wet_surfaces = true;

			// Small tree frogs can climb anything organic
			if (s.physical.body_mass_kg < 0.1f) {
				result.max_climb_angle_rad = M_PI / 2.0f; // Vertical easy
			}
		}
	}

	// ARTHROPODA: Insects and spiders
	if (HasFlag(clade, CF::ARTHROPODA)) {
		// Most arthropods are excellent climbers
		// Dai et al. (2002): Insects climb via diverse mechanisms

		if (any_setae) {
			// Spider/insect setae (similar to geckos but different scale)
			// Gorb (2001): Fly setae ~100 nN per spatula
			result.min_roughness_required = 0.0f;
			result.max_climb_angle_rad = M_PI / 2.0f;
			result.can_descend_head_first = true;

			// Small arthropods can climb inverted
			if (s.physical.body_mass_kg < 0.001f) {
				result.can_climb_inverted = true; // Flies walk on ceilings
			}
		}

		if (any_claws) {
			// Insect claws hook into microscopic surface features
			// Bullock & Federle (2011): Claws effective on rough surfaces
			result.min_roughness_required = 0.2f; // Need some texture
			result.max_climb_speed_m_s *= 1.5f; // Very fast for size
		}

		if (any_suckers) {
			// Beetles and some insects use suction
			// Dixon et al. (1990): Suction pads on smooth surfaces
			result.min_roughness_required = 0.0f;
			result.requires_dry_surface = false;
			result.can_descend_head_first = true;
		}

		// Spiders (ARACHNIDA subset)
		// Would benefit from ARACHNIDA clade flag if added
		if (limb_count >= 8) { // Spider proxy (8 legs)
			// Spiders are exceptional climbers
			// Wolff & Gorb (2012): Spiders combine claws + setae
			result.max_climb_speed_m_s *= 1.8f; // Very fast
			result.can_descend_head_first = true;
			result.can_climb_inverted = true; // Spiders walk on ceilings
		}

		// Ants (social climbing)
		// Wilson (1971): Ants form bridges and chains
		if (s.physical.body_mass_kg < 0.0001f && in.behavior.social_tendency > 0.7f) {
			result.max_climb_angle_rad = M_PI; // Can hang from anything
			result.can_form_living_bridges = true; // Social cooperation
		}
	}

	// AVES: Birds climbing (parrots, woodpeckers)
	if (HasFlag(clade, CF::AVES)) {
		// Most birds don't "climb" in traditional sense (they fly)
		// But parrots and woodpeckers do

		if (any_claws && limb_count >= 4) {
			// Parrots use beak + feet (zygodactyl toes)
			// Sustaita et al. (2013): Parrots use beak as 3rd limb
			result.max_climb_speed_m_s *= 0.8f; // Slower (not specialized)
			result.min_roughness_required = 0.4f;

			// Woodpeckers (tail-braced climbing)
			// Spring (1965): Woodpecker tail feathers support weight
			if (s.appendages.tails.size() > 0) {
				result.max_climb_speed_m_s *= 1.2f; // Tail brace helps
				result.can_descend_head_first = false; // Must descend tail-first
			}
		}
	}

	// MOLLUSCA (Cephalopoda): Octopus climbing
	if (HasFlag(clade, CF::CEPHALOPODA)) {
		// Octopuses climb using suckers + flexibility
		// Tramacere et al. (2014): Octopus suckers + soft body

		if (any_suckers) {
			result.min_roughness_required = 0.0f; // Smooth surfaces OK
			result.requires_dry_surface = false; // Aquatic
			result.max_climb_angle_rad = M_PI / 2.0f;
			result.can_climb_inverted = true; // Octopi walk on ceilings underwater

			// Octopuses are slow but extremely dexterous climbers
			result.max_climb_speed_m_s *= 0.6f;
			result.requires_aquatic_environment = true;
		}
	}

	return result;
}

std::optional<TonTon::Analysis_Brachiation>  TonTon::ComputeBrachiation(Input const& in, Scratch & s)
{
	// Brachiation requires anterior limbs with graspers
	auto manipulators = ComputeManipulation(in, SF::GRASPER);

	if(manipulators.empty())
		return {};

	// Filter to only anterior (forelimb) manipulators
	std::vector<Analysis_Manipulator> arms;
	for(auto const& manip : manipulators)
	{
		if(HasFlag(manip.subtree_flags, SF::FORELIMB) || HasFlag(manip.subtree_flags, SF::ANTERIOR))
		{
			arms.push_back(manip);
		}
	}

	if(arms.empty())
		return {};

	// 1. BRACHIATION CAPABILITY CHECK
	// Requires:
	// - Long arms relative to body size
	// - Sufficient grip strength to support body weight with one arm
	// - Prehensile hands (thumb or strong grip)

	force_N body_weight_N = s.physical.body_mass_kg * in.environment.gravity_m_s2;
	length_m body_length = s.physical.body_length_m;

	length_m avg_arm_length = 0;
	force_N min_grip_strength = FLT_MAX;
	bool has_prehensile_hands = false;

	for(auto const& arm : arms)
	{
		avg_arm_length += arm.stretched_length_m;
		min_grip_strength = std::min(min_grip_strength, arm.max_grip_force_N);
		has_prehensile_hands |= arm.has_thumb;
	}
	avg_arm_length /= arms.size();

	// Need arms at least 0.8x body length for effective brachiation
	auto arm_to_body_ratio = avg_arm_length / body_length;
	if(arm_to_body_ratio < 0.8f)
		return {};  // Arms too short

	// Need grip strength to support body weight with one arm (2x safety margin)
	if(min_grip_strength < body_weight_N * 2.0f)
		return {};  // Insufficient grip strength

	if(!has_prehensile_hands)
		return {};  // Need grasping ability

	// 2. PENDULUM DYNAMICS
	// Brachiation is essentially inverted pendulum motion
	// Center of mass falls and rises with each swing

	// Effective pendulum length = arm_length + distance from shoulder to CoM
	// Approximate body CoM at 0.5 * body_length from shoulder
	auto shoulder_to_com = body_length * 0.4f;
	auto pendulum_length_m = avg_arm_length + shoulder_to_com;

	// Natural frequency: f = (1/2π) * sqrt(g/L)
	auto natural_frequency_Hz = (1.0f / (2.0f * M_PI)) * sqrt(in.environment.gravity_m_s2 / pendulum_length_m);

	// Actual swing frequency is typically 70-90% of natural frequency
	auto swing_frequency_Hz = natural_frequency_Hz * 0.8f;

	// 3. SWING SPEED
	// Maximum speed at bottom of swing using energy conservation
	// PE at top = KE at bottom
	// m*g*h = 0.5*m*v²
	// v = sqrt(2*g*h)

	// Height drop during swing ≈ pendulum_length * (1 - cos(swing_angle))
	// Typical swing angle for brachiation: 30-45 degrees
	angle_rad swing_angle_rad = M_PI / 6.0f; // 30 degrees
	auto height_drop_m = pendulum_length_m * (1.0f - std::cos(float(swing_angle_rad)));

	auto max_swing_speed_m_s = sqrt(2.0f * in.environment.gravity_m_s2 * height_drop_m);

	// Adjust for muscle quality (affects ability to maintain speed)
	max_swing_speed_m_s *= glm::mix(0.7f, 1.3f, in.muscle_quality);

	// 4. MAX GAP DISTANCE
	// Limited by arm reach and forward momentum
	// Gap = 2 * arm_reach + forward_projection

	auto max_reach_m = avg_arm_length * 2.0f; // Both arms extended

	// Forward projection from swing momentum
	// At release: horizontal velocity ≈ v * sin(release_angle)
	// Ballistic flight time: t = 2*v_y/g
	// Horizontal distance: x = v_x * t

	auto release_angle_rad = M_PI / 4.0f; // 45 degrees optimal
	auto v_horizontal = max_swing_speed_m_s * std::cos(release_angle_rad);
	auto v_vertical = max_swing_speed_m_s * std::sin(release_angle_rad);

	auto flight_time_s = 2.0f * v_vertical / in.environment.gravity_m_s2;
	auto ballistic_distance_m = v_horizontal * flight_time_s;

	auto max_gap_distance_m = max_reach_m + ballistic_distance_m;

	// Safety factor - don't use full reach
	max_gap_distance_m *= 0.75f;

	// 5. BUILD ARM STRUCTURES
	std::vector<Analysis_Brachiation::Arm> output_arms;
	output_arms.reserve(arms.size());

	for(auto const& arm : arms)
	{
		Analysis_Brachiation::Arm brachiation_arm;
		static_cast<Analysis_Appendage&>(brachiation_arm) = arm;

		brachiation_arm.reach_m = arm.stretched_length_m;
		brachiation_arm.grip_strength_N = arm.max_grip_force_N;
		brachiation_arm.swing_speed_m_s = max_swing_speed_m_s;

		output_arms.push_back(brachiation_arm);
	}

	return Analysis_Brachiation{
		.arms = shared_array<Analysis_Brachiation::Arm>::FromArray(output_arms),
		.max_swing_speed_m_s = max_swing_speed_m_s,
		.max_gap_distance_m = max_gap_distance_m,
		.pendulum_length_m = pendulum_length_m,
		.swing_frequency_Hz = swing_frequency_Hz,
		.arm_phase_offset = 0.5f  // Alternating arms are 180° out of phase
	};
}


std::vector<TonTon::Analysis_Manipulator>   TonTon::ComputeManipulation(TonTon::Input const& in, TonTon::SemanticFlags requirements)
{
	std::vector<TonTon::Analysis_Manipulator> r;
	r.reserve(in.builder->appendages.size());
	
	for(auto i = 0u; i < in.builder->appendages.size(); ++i)
	{
		if(HasFlag(in.builder->appendages[i].semantic_flags, requirements))
		{
			auto app = ComputeManipulation(in, i);
			
			if(app != std::nullopt)
			{
				r.push_back(std::move(*app));
			}
		}
	}

	return r;
}

std::optional<TonTon::Analysis_Manipulator> TonTon::ComputeManipulation(const Input &in, uint32_t idx) 
{
	auto & appendage = in.builder->appendages[idx];
	
	if(appendage.contact.joint < 0)
		return {};
	
	Analysis_Manipulator out;
	appendage.copy_into(out, in.scale);
	
	out.tip = appendage.contact.joint;
	
	out.rest_length_m = scale_to<0>(appendage.contact.rest_length, in.scale);
	out.stretched_length_m = scale_to<0>(appendage.contact.stretched_length, in.scale);
	
	out.subtree_flags = appendage.contact.subtree_flags;
	out.surface_area_m2 = scale_to<0>(appendage.contact.area, in.area_scale());
	out.surface_normal  = appendage.contact.normal;
		
	area_m2 avg_area = scale_to<0>(appendage.avgCrossSection, in.area_scale());
	auto avg_moment = scale_to<0>(appendage.avgMoment, in.area_scale()*in.area_scale());
	
//	if(counter)
	{
		// 1. MUSCLE VOLUME ESTIMATION
		// Assume ~40-60% of limb volume is muscle (depends on type)
		auto muscle_fraction = glm::mix(0.35f, 0.65f, in.behavior.endurance_vs_power);
	//	double muscle_volume_m3 = metrics.volume * muscle_fraction;
	//	double muscle_mass_kg = muscle_volume_m3 * glm::mix(700.0, 1050.0, in.average_density);
		
		// 2. GRIP FORCE (from finger/digit flexors)
		// Peak muscle stress: 20-40 N/cm² (200,000-400,000 Pa)
		// Quality affects max stress achievable
		pressure_Pa muscle_stress_Pa = glm::mix(200000.0f, 400000.0f, in.muscle_quality);
		
		// Effective cross-sectional area (perpendicular to force direction)
		// For grip, this is roughly the cross-section of the muscles
		area_m2 grip_muscle_area_m2 = avg_area * muscle_fraction;
		force_N base_grip_force_N = muscle_stress_Pa * grip_muscle_area_m2;

		// Grip force depends on mechanical advantage / limb geometry
		// Short thick limbs (penguin legs) have poor leverage despite high muscle mass
		// Compare limb length to thickness (sqrt of cross-sectional area)
		auto limb_thickness_m = sqrt(avg_area); // Characteristic thickness
		auto aspect_ratio = out.stretched_length_m / std::max<length_m>(0.001f, limb_thickness_m);
		// Typical limb: aspect_ratio ~5-15 (length is 5-15x thickness)
		// Penguin legs: aspect_ratio ~2-3 (short and thick), poor mechanical advantage
		// Primate arms: aspect_ratio ~10-20 (long and slender), good leverage
		auto length_factor = glm::clamp(aspect_ratio / 10.0f, 0.3f, 1.2f); // 0.3x at stubby, 1.0x at normal

		out.max_grip_force_N = base_grip_force_N * length_factor;
		
		// 3. LIFT FORCE (constrained by joint torque limits)
		// Torque = Force × moment_arm
		// Max torque ≈ muscle_PCSA × muscle_stress × moment_arm
		length_m moment_arm_m = sqrt(avg_moment / avg_area); // Radius of gyration
		auto max_torque_Nm = muscle_stress_Pa * grip_muscle_area_m2 * moment_arm_m;
		
		// Convert torque to force at tip (lever arm = chain length)
		auto lever_arm_m = out.stretched_length_m;
		if(lever_arm_m > 0.001f) {
			out.max_lift_force_N = max_torque_Nm / lever_arm_m;
		} else {
			out.max_lift_force_N = 0.0f;
		}
		
		// Apply structure vs weight scaling
		// More robust structure = can handle more force
		auto structure_bonus = glm::mix(0.7f, 1.3f, in.structure_vs_weight);
		out.max_lift_force_N *= structure_bonus;
		out.max_grip_force_N *= structure_bonus;
	}
	
	// 4. ADHESION FORCE (for specialized structures)
	out.max_adhesion_force_N = 0.0f;
		
	out.has_claws=appendage.contact.has_claws;
	out.has_suckers=appendage.contact.has_suckers;
	out.has_setae=appendage.contact.has_setae;
	out.has_thumb=appendage.contact.has_thumb;
	out.has_wet_grip=appendage.contact.has_wet_grip;
	
	if(out.has_suckers) {
		// Suction force: F = ΔP × Area
		// Assume can create ~0.8 atm (80 kPa) pressure differential
		pressure_Pa suction_pressure_Pa = 80000.0f;
		
		// Estimate sucker area from surface area
		auto sucker_coverage = 0.3f; // ~30% of surface is actual suction cups
		auto effective_sucker_area = out.surface_area_m2 * sucker_coverage;
		
		out.max_adhesion_force_N = suction_pressure_Pa * effective_sucker_area;
	}
	
	if(out.has_setae) {
		// Van der Waals adhesion (gecko-like)
		// ~10 N/cm² for optimal setae density
		pressure_Pa setae_stress_Pa = 100000.0f; // 10 N/cm²
		
		// Assume setae cover the contact surface
		out.max_adhesion_force_N += 
			setae_stress_Pa * out.surface_area_m2;
	}
		
	if(out.has_wet_grip) {
		// Tree frog adhesion: capillary forces + mucus adhesion
		// ~1-5 N/cm² depending on surface wetness
		// Formula: F = 2πRγ (capillary) + μ×Area (mucus)
		pressure_Pa wet_adhesion_Pa = 30000.0f; // ~3 N/cm² typical
		
		// Pad coverage (tree frogs have ~60% of toe surface as pad)
		auto pad_coverage = 0.6f;
		auto effective_pad_area = out.surface_area_m2 * pad_coverage;
		
		out.max_adhesion_force_N = 
			wet_adhesion_Pa * effective_pad_area;
		
		// Requires wet/humid surface - note this in metadata somewhere?
	}
	
	if(out.has_friction_pads()) {
		// Primate friction grip (no true adhesion, just high friction)
		// Not really "adhesion" but contributes to grip force
		// Coefficient of friction ~0.5-1.5 for primate skin
		auto friction_coef = 1.0f;
		
		// This multiplies the normal force (which comes from grip force)
		// So it enhances grip_force rather than being separate
		out.max_grip_force_N *= (1.0f + friction_coef * 0.3f);
	}

	// 5. SCALING ADJUSTMENTS
	// Square-cube law: Force scales with cross-section (area), not volume
	auto size_scale = in.area_scale(); // Already accounts for anisotropic scaling
	
	// Apply conservative allometric scaling
	// Smaller animals have relatively stronger muscles (force/mass ratio)
	auto allometric_factor = std::pow(float(size_scale), 0.67f); // Between area (1.0) and volume (0.67)
	
	out.max_lift_force_N *= allometric_factor;
	out.max_grip_force_N *= allometric_factor;
	out.max_adhesion_force_N *= allometric_factor;
	
	// 6. TYPE-SPECIFIC ADJUSTMENTS
	if(HasFlag(out.subtree_flags, SF::TENTACLE)) {
		// Tentacles are hydrostatic - different force characteristics
		// Generally weaker in grip but good adhesion
		out.max_grip_force_N *= 0.6f;
		if(out.has_suckers) {
			out.max_adhesion_force_N *= 1.5f; // Octopus-level suction
		}
	}
	
	// trunks and monkey tails.
	if(HasFlag(out.subtree_flags, SF::FACIAL|SF::TAIL)) {
		// Elephant trunks: excellent lift, moderate grip
		out.max_lift_force_N *= 1.4f;
		out.max_grip_force_N *= 0.8f;
	}
	
	if(out.max_lift_force_N == 0
	|| out.max_grip_force_N == 0)
		return {};
	
	return out;
}


#if ORIGINAL

std::optional<TonTon::Analysis_Manipulator> TonTon::ComputeManipulation(const Input &in, uint32_t idx) 
{
	auto & appendage = in.builder->appendages[idx];
	
	Analysis_Manipulator out;
	appendage.copy_into(out, );
	static_cast<Analysis_Appendage&>(	out) = appendages[i];
	auto idx = out.tip;
	
	int relevant_root = out.tip;
	for(auto j = out.tip; j >= out.root; j = parents[j])
	{
		if(HasFlag(semantic_flags[j], SF::CONTACT|SF::GRASPER))
			relevant_root = j;		
	}
	
	auto relevant_joints = sk_memo->GetAllChildrenOfRoot(relevant_root);
	// get all *excluding grasper itself* (just fingers)
	auto which = std::span<uint16_t>(relevant_joints.data(), relevant_joints.size());
		
	auto metrics = in.skinnedMesh->GetMetrics(which, in.behavior.scale);
	TonTon::Silhouette silhouette;
	
	out.subtree_flags = relative_flags[idx].child_flags|semantic_flags[idx];
	
	// it is a spike.. or trunk.. or something, no fingers!
	// trunks have fingers but in an armature they probably don't.
	if(metrics.volume == 0.0)
	{
		auto tip = out.tip;
	
		out.surface_area_m2 = sk.EstimateCrossSection(tip, in.behavior.scale, position[tip] - position[parents[tip]]);
		out.surface_normal  = glm::normalize(position[tip] - position[parents[tip]]);
	}
	else
	{
	// get all *including grasper itself*
		which = std::span<uint16_t>(relevant_joints.data(), relevant_joints.size());
				
		std::pair<glm::quat, glm::vec3> eigen_decomp;
		auto projection = in.skinnedMesh->memo()->GetProjectionMatrix(EigenValue::Small, in.behavior.scale, which, &metrics, &eigen_decomp);
		silhouette = in.skinnedMesh->memo()->GetSilhouettes(projection, in.behavior.scale, which);
		
		// get projection direction.
		out.surface_area_m2 = silhouette.area;
		out.surface_normal =  glm::mat3(eigen_decomp.first)[2];
	}
	
	auto counter = 0;
	area_m2 accumulator = 0;
	moment_m4 accumulator_2nd_moment = 0;
	for(auto j = out.tip; j != out.root; j = parents[j])
	{
		auto p =  parents[j];
		if(p < 0) break;
		double secondMoment = 0;
		auto area_m2 = sk.EstimateCrossSection(p, in.behavior.scale, position[j] - position[p], &secondMoment);
	
		accumulator += area_m2;
		accumulator_2nd_moment += secondMoment;
		++counter;
	}
	
	if(counter)
	{
		area_m2 avg_area = accumulator / counter;
		auto avg_moment = accumulator_2nd_moment / counter;
		
		// 1. MUSCLE VOLUME ESTIMATION
		// Assume ~40-60% of limb volume is muscle (depends on type)
		auto muscle_fraction = glm::mix(0.35f, 0.65f, in.behavior.endurance_vs_power);
	//	double muscle_volume_m3 = metrics.volume * muscle_fraction;
	//	double muscle_mass_kg = muscle_volume_m3 * glm::mix(700.0, 1050.0, in.average_density);
		
		// 2. GRIP FORCE (from finger/digit flexors)
		// Peak muscle stress: 20-40 N/cm² (200,000-400,000 Pa)
		// Quality affects max stress achievable
		pressure_Pa muscle_stress_Pa = glm::mix(200000.0f, 400000.0f, in.muscle_quality);
		
		// Effective cross-sectional area (perpendicular to force direction)
		// For grip, this is roughly the cross-section of the muscles
		area_m2 grip_muscle_area_m2 = avg_area * muscle_fraction;
		force_N base_grip_force_N = muscle_stress_Pa * grip_muscle_area_m2;

		// Grip force depends on mechanical advantage / limb geometry
		// Short thick limbs (penguin legs) have poor leverage despite high muscle mass
		// Compare limb length to thickness (sqrt of cross-sectional area)
		auto limb_thickness_m = sqrt(avg_area); // Characteristic thickness
		auto aspect_ratio = out.stretched_length_m / std::max(0.001f, limb_thickness_m);
		// Typical limb: aspect_ratio ~5-15 (length is 5-15x thickness)
		// Penguin legs: aspect_ratio ~2-3 (short and thick), poor mechanical advantage
		// Primate arms: aspect_ratio ~10-20 (long and slender), good leverage
		auto length_factor = glm::clamp(aspect_ratio / 10.0f, 0.3f, 1.2f); // 0.3x at stubby, 1.0x at normal

		out.max_grip_force_N = base_grip_force_N * length_factor;
		
		// 3. LIFT FORCE (constrained by joint torque limits)
		// Torque = Force × moment_arm
		// Max torque ≈ muscle_PCSA × muscle_stress × moment_arm
		length_m moment_arm_m = sqrt(avg_moment / avg_area); // Radius of gyration
		auto max_torque_Nm = muscle_stress_Pa * grip_muscle_area_m2 * moment_arm_m;
		
		// Convert torque to force at tip (lever arm = chain length)
		auto lever_arm_m = out.stretched_length_m;
		if(lever_arm_m > 0.001f) {
			out.max_lift_force_N = max_torque_Nm / lever_arm_m;
		} else {
			out.max_lift_force_N = 0.0f;
		}
		
		// Apply structure vs weight scaling
		// More robust structure = can handle more force
		auto structure_bonus = glm::mix(0.7f, 1.3f, in.structure_vs_weight);
		out.max_lift_force_N *= structure_bonus;
		out.max_grip_force_N *= structure_bonus;
	}
	
	// 4. ADHESION FORCE (for specialized structures)
	out.max_adhesion_force_N = 0.0f;
	
// grab each thing in the subtree		
	bool has_suckers = false;
	bool has_setae = false;
	bool has_claws = false;
	bool has_thumb = false;
	bool has_wet_grip = false; // what word do i look for for a treefrog?		
	
	for(auto node : relevant_joints)
	{
		for(auto word : sk.skin->tags[node])
		{
			switch(word)
			{
			default: break;
			case Word::sucker:
				has_suckers = true;
				break;
			case Word::setae:
				has_setae = true;
				break;		
			case Word::claw:
			case Word::talon:
				has_claws = true;
				break;		
			case Word::thumb:
				has_thumb = true;
				break;	
			case Word::pad:
			case Word::adhesive:
				has_wet_grip = true;
				break;
			}
		}
	}
	
	out.has_claws=has_claws;
	out.has_suckers=has_suckers;
	out.has_setae=has_setae;
	out.has_thumb=has_thumb;
	out.has_wet_grip=has_wet_grip;
	
	if(has_suckers) {
		// Suction force: F = ΔP × Area
		// Assume can create ~0.8 atm (80 kPa) pressure differential
		pressure_Pa suction_pressure_Pa = 80000.0f;
		
		// Estimate sucker area from surface area
		auto sucker_coverage = 0.3f; // ~30% of surface is actual suction cups
		auto effective_sucker_area = out.surface_area_m2 * sucker_coverage;
		
		out.max_adhesion_force_N = suction_pressure_Pa * effective_sucker_area;
	}
	
	if(has_setae) {
		// Van der Waals adhesion (gecko-like)
		// ~10 N/cm² for optimal setae density
		pressure_Pa setae_stress_Pa = 100000.0f; // 10 N/cm²
		
		// Assume setae cover the contact surface
		out.max_adhesion_force_N += 
			setae_stress_Pa * out.surface_area_m2;
	}
		
	if(has_wet_grip) {
		// Tree frog adhesion: capillary forces + mucus adhesion
		// ~1-5 N/cm² depending on surface wetness
		// Formula: F = 2πRγ (capillary) + μ×Area (mucus)
		pressure_Pa wet_adhesion_Pa = 30000.0f; // ~3 N/cm² typical
		
		// Pad coverage (tree frogs have ~60% of toe surface as pad)
		auto pad_coverage = 0.6f;
		auto effective_pad_area = out.surface_area_m2 * pad_coverage;
		
		out.max_adhesion_force_N = 
			wet_adhesion_Pa * effective_pad_area;
		
		// Requires wet/humid surface - note this in metadata somewhere?
	}
	
	if(out.has_friction_pads()) {
		// Primate friction grip (no true adhesion, just high friction)
		// Not really "adhesion" but contributes to grip force
		// Coefficient of friction ~0.5-1.5 for primate skin
		auto friction_coef = 1.0f;
		
		// This multiplies the normal force (which comes from grip force)
		// So it enhances grip_force rather than being separate
		out.max_grip_force_N *= (1.0f + friction_coef * 0.3f);
	}

	// 5. SCALING ADJUSTMENTS
	// Square-cube law: Force scales with cross-section (area), not volume
	auto size_scale = in.area_scale(); // Already accounts for anisotropic scaling
	
	// Apply conservative allometric scaling
	// Smaller animals have relatively stronger muscles (force/mass ratio)
	auto allometric_factor = std::pow(float(size_scale), 0.67f); // Between area (1.0) and volume (0.67)
	
	out.max_lift_force_N *= allometric_factor;
	out.max_grip_force_N *= allometric_factor;
	out.max_adhesion_force_N *= allometric_factor;
	
	// 6. TYPE-SPECIFIC ADJUSTMENTS
	if(HasFlag(out.subtree_flags, SF::TENTACLE)) {
		// Tentacles are hydrostatic - different force characteristics
		// Generally weaker in grip but good adhesion
		out.max_grip_force_N *= 0.6f;
		if(has_suckers) {
			out.max_adhesion_force_N *= 1.5f; // Octopus-level suction
		}
	}
	
	// trunks and monkey tails.
	if(HasFlag(out.subtree_flags, SF::FACIAL|SF::TAIL)) {
		// Elephant trunks: excellent lift, moderate grip
		out.max_lift_force_N *= 1.4f;
		out.max_grip_force_N *= 0.8f;
	}
	
	return out;
}

#endif
