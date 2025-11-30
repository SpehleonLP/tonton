#include "tonton_scratch.h"
#include "Rules/tonton_aerial.h"
#include "Rules/tonton_sensory.h"
#include "Rules/tonton_specialized.h"
#include "Rules/tonton_climbing.h"
#include "Rules/tonton_serpentine.h"
#include "Rules/tonton_metabolic.h"
#include "../include/tonton_input.h"

#include "Rules/tonton_behavior.h"
#include "Rules/tonton_aquatic.h"
#include "Rules/tonton_terrestrial.h"
#include "tonton_builder.h"
#include <cfloat>

using Warning = TonTon::Analysis_Diagnostics::Warning::Severity;
using SF = TonTon::SemanticFlags;

namespace TonTon
{
static	Analysis_Physical ComputePhysical(Input const& in, Scratch & s);

static	shared_array<Analysis_Tail>   ComputeTails(Input const& in);

}

namespace TonTon
{
Analysis_TakeoffAnalysis TakeoffAnalysis_Compute(Input const& in, const Scratch& output);
}

TonTon::Scratch::Scratch(Input const& in)
{
	appendages.manipulation = shared_array<Analysis_Manipulator>::FromArray(ComputeManipulation(in, SF::GRASPER));
	appendages.tails = ComputeTails(in);

	physical    = ComputePhysical(in, *this);
	sensory     = ComputeSensory(in, *this);
	
	// Metabolic rates computed early (after physical, before locomotion)
	// Locomotion modes need metabolic budget, but metabolic needs clade flags from physical
	// For multi-clade creatures (pegasus), metabolic blends all clade contributions
	metabolic   = ComputeMetabolic(in, *this);

	terrestrial = ComputeTerrestrial(in, *this);
	serpentine  = ComputeSerpentine(in, *this);
	jumping     = ComputeJumping(in, *this);

	aerial      = ComputeAerial(in, *this);
	aquatic     = ComputeAquatic(in, *this);

	climbing    = ComputeClimbing(in, *this);
	brachiation = ComputeBrachiation(in, *this);

	specialized.digging = ComputeDigging(in, *this);
	specialized.constriction = ComputeConstriction(in, *this);


	if(aerial.has_value())
	{
		aerial->takeoff = TakeoffAnalysis_Compute(in, *this);
	}

	behavior =  ComputeBehavior(in, *this);


	// ========================================================================
	// DIAGNOSTICS - Physics validation and confidence estimation
	// ========================================================================

	// 1. POWER BUDGET CHECK
	// Check if the most demanding locomotion mode is feasible
	// We check PEAK power (muscle) for flight, since it's the most demanding
	// Terrestrial/aquatic locomotion is already constrained by muscle force in their respective rules
	power_W peak_power_required_W = 0.0f;
	const force_N weight_N = physical.body_mass_kg * in.environment.gravity_m_s2;
	bool has_demanding_mode = false;

	if(aerial.has_value())
	{
		// Flight is the most power-demanding locomotion mode
		// Check sustained flight power vs available muscle power
		power_W flight_power = weight_N * aerial->flapping_cost_W_per_N;
		peak_power_required_W = std::max(peak_power_required_W, flight_power);
		has_demanding_mode = true;
	}

	if(aerial.has_value() && aerial->can_hover)
	{
		// Hovering is even more demanding
		power_W hover_power = weight_N * aerial->hovering_cost_W_per_N;
		peak_power_required_W = std::max(peak_power_required_W, hover_power);
	}

	// Power budget passes if:
	// 1. No demanding modes (terrestrial/aquatic are self-regulating)
	// 2. Peak power required <= available muscle power
	diagnostics.passes_power_budget_check =
		!has_demanding_mode ||
		(peak_power_required_W <= metabolic.available_muscle_power_W);

	// 2. MASS BUDGET CHECK
	// Sum appendage masses and compare to body mass
	mass_kg appendage_mass_kg = 0.0f;

	for(auto const& tail : appendages.tails)
	{
		appendage_mass_kg += tail.tail_mass_kg;
	}

	if(aerial.has_value())
	{
		for(auto const& wing : aerial->wings)
		{
			appendage_mass_kg += wing.wing_mass_kg;
		}
	}

	// Appendages should be < 80% of body mass (limbs, organs, etc make up the rest)
	diagnostics.passes_mass_budget_check =
		(appendage_mass_kg < physical.body_mass_kg * 0.8f);

	// 3. PHYSICAL PLAUSIBILITY
	// Check for obviously wrong values
	bool reasonable_mass = (physical.body_mass_kg > 0.0001f && physical.body_mass_kg < 200000.0f);
	bool reasonable_length = (physical.body_length_m > 0.001f && physical.body_length_m < 50.0f);
	bool reasonable_density = (physical.body_mass_kg / physical.body_volume_m3 > 100.0f &&
	                          physical.body_mass_kg / physical.body_volume_m3 < 2000.0f);
	bool reasonable_metabolic = (metabolic.basal_rate_W > 0.0f && metabolic.max_rate_W > metabolic.basal_rate_W);

	diagnostics.is_physically_plausible =
		reasonable_mass && reasonable_length && reasonable_density && reasonable_metabolic &&
		diagnostics.passes_power_budget_check && diagnostics.passes_mass_budget_check;

	// 4. OVERALL CONFIDENCE
	// Based on number of checks passed and locomotion mode diversity
	int checks_passed = 0;
	if(diagnostics.passes_power_budget_check) ++checks_passed;
	if(diagnostics.passes_mass_budget_check) ++checks_passed;
	if(reasonable_mass) ++checks_passed;
	if(reasonable_length) ++checks_passed;
	if(reasonable_density) ++checks_passed;
	if(reasonable_metabolic) ++checks_passed;

	// More locomotion modes analyzed = more confident
	int locomotion_modes = 0;
	if(terrestrial.has_value()) ++locomotion_modes;
	if(aerial.has_value()) ++locomotion_modes;
	if(aquatic.has_value()) ++locomotion_modes;
	if(serpentine.has_value()) ++locomotion_modes;
	if(climbing.has_value()) ++locomotion_modes;

	// Confidence: (checks_passed / 6) * 0.7 + (locomotion_modes / 3) * 0.3
	diagnostics.overall_confidence =
		(float(checks_passed) / 6.0f) * 0.7f +
		(std::min(locomotion_modes, 3) / 3.0f) * 0.3f;

	// 5. WARNINGS
	if(!diagnostics.passes_power_budget_check)
	{
		diagnostics.warnings.push_back({
			.level = Warning::CAUTION,
			.message = "Power budget exceeded: locomotion requires more power than available muscle can provide"
		});
	}

	if(!diagnostics.passes_mass_budget_check)
	{
		diagnostics.warnings.push_back({
			.level = Warning::CAUTION,
			.message = "Mass budget suspicious: appendages account for >80% of body mass"
		});
	}

	if(!reasonable_density)
	{
		diagnostics.warnings.push_back({
			.level = Warning::ERROR,
			.message = "Body density outside plausible range (100-2000 kg/m³)"
		});
	}

	if(physical.body_mass_kg < 0.001f)
	{
		diagnostics.warnings.push_back({
			.level = Warning::INFO,
			.message = "Very small creature (<1g) - some allometric laws may not apply"
		});
	}
	else if(physical.body_mass_kg > 10000.0f)
	{
		diagnostics.warnings.push_back({
			.level = Warning::INFO,
			.message = "Very large creature (>10 tons) - some physics may need adjustment"
		});
	}
};

static	TonTon::Analysis_Physical TonTon::ComputePhysical(Input const& in, Scratch & s)
{
	auto & ph = in.builder->physical;
	auto body_volume_m3 = scale_to<0>(ph.body_volume, in.volume_scale());
	auto & I = ph.covariance_restPose;
	float cov_scale = float(in.inertia_scale()) * float(in.body_density()); 
	
	return {
		.body_mass_kg=body_volume_m3 * in.body_density(),
		.body_length_m=scale_to<0>(ph.body_length, in.scale),
		.body_volume_m3=body_volume_m3,
		.tail_length_m=scale_to<0>(ph.tail_length, in.scale),
		
		.surface_area_m2=scale_to<0>(ph.surface_area, in.area_scale()),
		.cross_sectional_area_m2=scale_to<0>(ph.cross_section_area, in.area_scale()),
	
		.spine_root=ph.spine_root,
		.upright=ph.upright,
		.clade=ph.clade,
		.niche=ph.niche,
		
		.covariance_restPose ={
			I[0]*cov_scale,
			I[1]*cov_scale,
			I[2]*cov_scale,
			I[3]*cov_scale,
			I[4]*cov_scale,
			I[5]*cov_scale
		}
	};
}

static	TonTon::Analysis_Tail ComputeTail(TonTon::Input const& in, TonTon::Builder_Tail const& it)
{
	TonTon::Analysis_Tail r;
	it.copy_into(r, in.scale);
	
	r.common_ancestor=it.commonAncestor;
	r.tail_mass_kg=scale_to<0>(it.volume, in.volume_scale()) * in.body_density();
	r.max_cross_section_m2=scale_to<0>(it.maxCrossSection, in.area_scale());
	r.min_cross_section_m2=scale_to<0>(it.minCrossSection, in.area_scale());
	
	r.used_for=it.used_for;
	auto branches = shared_array<TonTon::Analysis_Tail>(it.branches.size());
	
	TonTon::length_m max_length = 0;
	for(auto i = 0u; i < branches.size(); ++i)
	{
		branches[i] = ComputeTail(in, it.branches[i]);
		max_length = std::max(max_length, branches[i].stretched_length_m);
	}
	
	r.branches = branches;
	r.natural_sway_frequency_Hz = sqrt(in.environment.gravity_m_s2 / (r.stretched_length_m + max_length)) / M_PI;
	
	return r;
} 

static	shared_array<TonTon::Analysis_Tail>   TonTon::ComputeTails(Input const& in)
{
	auto branches = in.builder->tails;
	auto r = shared_array<TonTon::Analysis_Tail>(branches.size());
	
	for(auto i = 0u; i < branches.size(); ++i)
	{
		r[i] = ComputeTail(in, branches[i]);
	}
	
	return r;
}


