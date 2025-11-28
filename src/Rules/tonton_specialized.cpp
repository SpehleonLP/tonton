#include "tonton_specialized.h"
#include "Rules/tonton_climbing.h"
#include "tonton_builder.h"
#include "tonton_scratch.h"
#include "../include/tonton_input.h"
#include "../include/tonton_analysis.h"
#include "Memos/tonton_armaturememo.h"

using namespace TonTon;
using SF = TonTon::SemanticFlags;

// ============================================================================
// DIGGING / FOSSORIAL LOCOMOTION
// ============================================================================
// References:
// - Hildebrand 1985: Fossorial adaptations (long claws)
// - Casinos et al. 1993: Allometry in digging rodents
// - Hopkins 2007: Fossorial morphology indicators
// - Biewener 1990: Muscle stress (~300 kPa maximum)

std::optional<Analysis_Digging> TonTon::ComputeDigging(Input const& in, Scratch & s)
{
	// Need manipulators (forelimbs) to dig
	if(s.appendages.manipulation.empty())
		return {};

	// Analyze forelimbs for digging adaptations
	bool has_digging_claws = false;
	bool has_strong_forelimbs = false;
	bool has_incisor_teeth = in.builder->semanticAnalyisis.has_incisor_teeth;
	force_N max_forelimb_force = 0.0f;
	area_m2 avg_forelimb_area = 0.0f;
	int forelimb_count = 0;

	// Check manipulators for digging capability
	for(auto i = 0u; i < in.builder->appendages.size(); ++i)
	{
		// Check if this is a forelimb (anterior)
		bool is_forelimb = HasFlag(in.builder->appendages[i].semantic_flags, SF::ANTERIOR | SF::LIMB);

		if(is_forelimb)
		{
			auto manip = ComputeManipulation(in, i);
		
			if(manip)
			{		
				++forelimb_count;
				max_forelimb_force = std::max(max_forelimb_force, manip->max_lift_force_N);
				avg_forelimb_area += manip->contact_area_m2;
	
				// Claws are critical for scratch digging
				if(manip->has_claws)
				{
					has_digging_claws = true;
				}
	
				// Strong forelimbs indicate digging capability
				// Fossorial mammals have ~2-3x stronger forelimbs than cursorial species
				auto force_to_weight = manip->max_lift_force_N / (s.physical.body_mass_kg * in.environment.gravity_m_s2);
				if(force_to_weight > 0.3f) // Can lift 30% of body weight
				{
					has_strong_forelimbs = true;
				}
			}
		}
	}

	if(forelimb_count > 0)
	{
		avg_forelimb_area /= forelimb_count;
	}

	// Determine if this creature can dig
	bool can_dig = has_digging_claws || has_strong_forelimbs || has_incisor_teeth;

	if(!can_dig)
		return {};

	// ========================================================================
	// DIGGING METHOD DETERMINATION
	// ========================================================================

	Analysis_Digging result;

	// Default to scratch digging (most common)
	result.method = Analysis_Digging::Method::SCRATCH;

	// Head-lift method (turtles, some lizards) - uses head as wedge
	if(!has_digging_claws && !has_incisor_teeth && s.physical.body_mass_kg > 0.5f)
	{
		result.method = Analysis_Digging::Method::HEAD_LIFT;
	}

	// Incisor digging (rodents, pocket gophers)
	if(has_incisor_teeth && has_strong_forelimbs)
	{
		result.method = Analysis_Digging::Method::INCISOR;
	}

	// Humeral rotation (moles, golden moles) - highly specialized
	// Indicated by very powerful forelimbs relative to body size
	auto relative_limb_strength = max_forelimb_force / (s.physical.body_mass_kg * 9.81f);
	if(relative_limb_strength > 0.5f && avg_forelimb_area > 0.0f)
	{
		// Check if body is cylindrical (fineness ratio < 5 = stocky digger)
		if(s.physical.fineness_ratio() < 5.0f)
		{
			result.method = Analysis_Digging::Method::HUMERAL_ROTATION;
		}
	}

	// ========================================================================
	// DIGGING SPEED CALCULATION
	// ========================================================================
	// From Hildebrand 1985 and empirical measurements:
	// - Scratch diggers: ~0.1-1.0 m/hour in hard soil
	// - Moles: ~3-6 m/hour in soft soil
	// - Pocket gophers: ~30-100 m/day (1-4 m/hour)

	// Base digging rate from muscle power
	// Assume ~25% muscle efficiency (Currey 2002)
	//auto muscle_efficiency = 0.25f * glm::mix(0.7f, 1.3f, in.muscle_quality);

	// Available power for digging (from forelimbs)
	// Muscle stress: 200-400 kPa (Biewener 1990)
	//auto muscle_stress_Pa = glm::mix(200000.0f, 400000.0f, in.muscle_quality);

	// Estimate muscle volume in forelimbs (~40-60% of limb)
	//auto muscle_fraction = glm::mix(0.35f, 0.65f, in.behavior.endurance_vs_power);

	// Power available = force * velocity
	// For digging: P = F * v, where v is stroke velocity
	// Typical digging stroke: ~0.5-2.0 Hz frequency
	auto stroke_frequency_Hz = 1.0f;
	auto stroke_length_m = 0.1f; // ~10cm per stroke

	// Soil resistance force (varies with soil type)
	// Hard soil: ~100-500 kPa penetration resistance
	// Soft soil: ~10-50 kPa
	//auto soil_resistance_Pa = 100000.0f; // Assume medium-hard soil

	// Effective digging force
	result.soil_force_N = max_forelimb_force * forelimb_count;

	// Volume excavated per stroke
	auto cross_section_m2 = s.physical.cross_sectional_area_m2;
	auto volume_per_stroke = cross_section_m2 * stroke_length_m;

	// Digging speed = (volume per stroke) * (frequency) / (tunnel cross-section)
	result.max_dig_speed_m_s = (volume_per_stroke * stroke_frequency_Hz) / cross_section_m2;

	// Method-specific adjustments
	switch(result.method)
	{
		case Analysis_Digging::Method::HUMERAL_ROTATION:
			// Moles are fastest diggers
			result.max_dig_speed_m_s *= 3.0f;
			break;

		case Analysis_Digging::Method::INCISOR:
			// Rodents moderately fast
			result.max_dig_speed_m_s *= 1.5f;
			break;

		case Analysis_Digging::Method::HEAD_LIFT:
			// Slowest method
			result.max_dig_speed_m_s *= 0.5f;
			break;

		case Analysis_Digging::Method::SCRATCH:
		default:
			// Default speed
			break;
	}

	// Allometric scaling: smaller animals dig relatively faster
	// From Casinos et al. 1993: digging rate ∝ M^(-0.2)
	auto size_factor = std::pow(float(s.physical.body_mass_kg), -0.2f);
	result.max_dig_speed_m_s *= size_factor;

	// Apply muscle quality and endurance factors
	result.max_dig_speed_m_s *= glm::mix(0.5f, 1.5f, in.muscle_quality);
	result.max_dig_speed_m_s *= glm::mix(0.8f, 1.2f, in.behavior.endurance_vs_power);

	// Clamp to realistic values (0.0001-0.01 m/s = 0.36-36 m/hour)
	result.max_dig_speed_m_s = std::clamp<velocity_m_s>(result.max_dig_speed_m_s, 0.0001f, 0.01f);

	// ========================================================================
	// TUNNEL DIAMETER
	// ========================================================================
	// Tunnel is typically 1.2-1.5x body diameter to allow turning
	auto body_diameter_m = sqrt(s.physical.cross_sectional_area_m2 / 3.14159f) * 2.0f;
	result.tunnel_diameter_m = body_diameter_m * 1.3f;

	return result;
}

// ============================================================================
// CONSTRICTION (Snakes, Large Constrictors, Octopi)
// ============================================================================
// References:
// - Hu et al. 2009: Friction requirements in snake locomotion
// - Biewener 1990: Muscle stress (~300 kPa maximum)
// - Moon 2000: Python constriction pressures (can reach 30 kPa)
// - Boback et al. 2012: Constriction in snakes

std::optional<Analysis_Constriction> TonTon::ComputeConstriction(Input const& in, Scratch & s)
{
	// Check for elongated body (snakes, eels, some amphisbaenians)
	// Fineness ratio = length / diameter
	// Snakes typically have fineness ratio > 15-20

	bool is_elongate = s.physical.fineness_ratio() > 10.0f;
	bool has_limbs = !s.appendages.manipulation.empty();

	// Limbless and elongate = likely constrictor candidate
	if(!is_elongate && has_limbs)
		return {};

	// Check if serpentine locomotion exists (strong indicator)
	bool is_serpentine = s.serpentine.has_value();

	// Tentacles can also constrict
	bool has_tentacles = false;
	for(auto const& manip : s.appendages.manipulation)
	{
		if(HasFlag(manip.subtree_flags, SF::TENTACLE))
		{
			has_tentacles = true;
			break;
		}
	}

	if(!is_serpentine && !has_tentacles)
		return {};

	// ========================================================================
	// CONSTRICTION FORCE CALCULATION
	// ========================================================================

	Analysis_Constriction result;

	// Muscle stress: 200-400 kPa (Biewener 1990)
	auto muscle_stress_Pa = glm::mix(200000.0f, 400000.0f, in.muscle_quality);

	// For elongate bodies, muscle makes up ~40-60% of body volume
	//auto muscle_fraction = glm::mix(0.4f, 0.6f, in.structure_vs_weight);
	//auto muscle_volume_m3 = s.physical.body_volume_m3 * muscle_fraction;

	// Effective cross-sectional area for constriction
	// Constriction uses axial muscles in a ring around prey
	//auto body_cross_section = s.physical.cross_sectional_area_m2;

	// Constriction pressure = (muscle stress) × (muscle fraction) × (coil overlap factor)
	// Multiple coils increase pressure
	auto typical_coils = std::max<float>(3.0f, float(s.physical.body_length_m / (2.0f * s.physical.cross_sectional_area_m2)));
	typical_coils = std::min(typical_coils, 12.0f); // Cap at ~12 coils

	// Each coil adds pressure, but with diminishing returns
	auto coil_efficiency = 0.7f; // Subsequent coils less effective
	auto effective_coils = 1.0f + (typical_coils - 1.0f) * coil_efficiency;

	// Squeeze pressure from muscle stress distributed over prey surface
	result.max_squeeze_pressure_Pa = muscle_stress_Pa * effective_coils * 0.15f; // ~15% efficiency

	// Empirical data: Python constriction reaches ~30 kPa (Boback et al. 2012)
	// Adjust our calculation to match empirical range
	result.max_squeeze_pressure_Pa = std::clamp<pressure_Pa>(
		result.max_squeeze_pressure_Pa,
		5000.0f,   // Min: 5 kPa (weak constrictors)
		60000.0f   // Max: 60 kPa (exceptional constrictors, >2× python)
	);

	// Apply quality factors
	result.max_squeeze_pressure_Pa *= glm::mix(0.6f, 1.4f, in.muscle_quality);
	result.max_squeeze_pressure_Pa *= glm::mix(0.8f, 1.2f, in.structure_vs_weight);

	// ========================================================================
	// COIL DIAMETER RANGE
	// ========================================================================
	// Minimum diameter: typically can't constrict prey much smaller than body diameter
	// Maximum diameter: limited by body length and flexibility

	length_m body_diameter_m = sqrt(s.physical.cross_sectional_area_m2 / 3.14159f) * 2.0f;

	// Minimum coil: ~1-2× body diameter
	result.coil_diameter_range_min_m = body_diameter_m * 1.5f;

	// Maximum coil: depends on body length
	// Circumference per coil ≈ π × D
	// Total length = N × π × D, so D_max = L / (N × π)
	auto min_coils_needed = 2.0f; // Need at least 2 coils to be effective
	result.coil_diameter_range_max_m = s.physical.body_length_m / (min_coils_needed * 3.14159f);

	// Can constrict up to ~3-5× body diameter typically
	auto practical_max = body_diameter_m * 4.0f;
	result.coil_diameter_range_max_m = std::min(result.coil_diameter_range_max_m, practical_max);

	// Tentacles have different constraints
	if(has_tentacles)
	{
		// Tentacles more flexible, can handle wider range
		result.coil_diameter_range_min_m *= 0.5f;
		result.coil_diameter_range_max_m *= 1.5f;

		// But generally weaker pressure (hydrostatic vs axial muscle)
		result.max_squeeze_pressure_Pa *= 0.7f;
	}

	return result;
}
