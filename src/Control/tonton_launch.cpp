#include "Control/tonton_launch.h"
#include "tonton_analysis.h"
#include <algorithm>

namespace TonTon {
namespace {

BlockingReason FirstFailedConstraint(const LaunchFacts& f)
{
	if (!f.wing_loading_ok)  return BlockingReason::WING_LOADING;
	if (!f.power_loading_ok) return BlockingReason::POWER_LOADING;
	if (!f.aspect_ratio_ok)  return BlockingReason::ASPECT_RATIO;
	if (!f.leg_strength_ok)  return BlockingReason::LEG_STRENGTH;
	// Reachable: ClassifyMode (tonton_takeoffanalysis.cpp:324-380) returns
	// IMPOSSIBLE from threshold misses -- power-to-weight, force margin -- with
	// every constraint flag true. Returning NONE there described an infeasible
	// plan as unobstructed.
	return BlockingReason::TAKEOFF_IMPOSSIBLE;
}

// A launch that pushes off the substrate needs a substrate that pushes back.
// Open water bears neither a leg thrust nor a standing wingbeat, unless the
// analysis flagged this creature as one of the water-taxiing flyers.
bool SubstrateBearsWeight(const LaunchFacts& f)
{
	return f.substrate != Substrate::WATER || f.can_use_water_taxi;
}

} // namespace

LaunchPlan PlanLaunch(const LaunchFacts& f)
{
	using TM = Analysis_TakeoffAnalysis::TakeoffMode;

	LaunchPlan p;

	p.required_airspeed_m_s      = f.required_airspeed_m_s;
	p.required_jump_velocity_m_s = f.required_jump_velocity_m_s;
	p.jump_direction             = glm::vec3(0.f, 1.f, 0.f);

	// A required jump velocity of exactly 0 means the analysis never computed
	// one -- `required <= available` is trivially true there, which reported
	// every creature with a jumping section as cleared for a jump launch
	// (measured: the dragonfly requires 0 against 3.896 m/s available). A
	// missing requirement is unknown, not satisfied.
	const bool jump_quantified = (f.required_jump_velocity_m_s > 0.f);
	p.jump_feasible = jump_quantified
		&& f.has_jump_analysis
		&& f.required_jump_velocity_m_s <= f.available_jump_velocity_m_s;

	switch (f.mode) {
	case TM::VERTICAL_LAUNCH:
		// Pure wing power -- but the creature is still standing on something
		// while it spools up, and open water is not something.
		p.feasible  = SubstrateBearsWeight(f);
		p.readiness = p.feasible ? 1.f : 0.f;
		if (!p.feasible) p.blocking_reason = BlockingReason::NEEDS_SOLID_SUBSTRATE;
		break;

	case TM::JUMP_LAUNCH:
		// Descriptive only: the caller plays the crouch and applies the
		// delta-v at whatever frame its animation says.
		if (!SubstrateBearsWeight(f)) {
			p.blocking_reason = BlockingReason::NEEDS_SOLID_SUBSTRATE;
		} else if (!jump_quantified) {
			p.blocking_reason = BlockingReason::JUMP_REQUIREMENT_UNKNOWN;
		} else {
			p.feasible  = p.jump_feasible;
			p.readiness = p.jump_feasible ? 1.f : 0.f;
			if (!p.jump_feasible) p.blocking_reason = BlockingReason::LEG_STRENGTH;
		}
		break;

	case TM::RUNNING_TAKEOFF: {
		// A water surface counts as runway for a taxiing flyer (pelicans).
		const bool runway_ok = (f.substrate == Substrate::GROUND)
			|| (f.substrate == Substrate::WATER && f.can_use_water_taxi);
		p.feasible = runway_ok;
		p.accelerate_along_heading = runway_ok;
		// Measured in AIRSPEED, so a headwind shortens the runway with no
		// special case at all.
		p.readiness = (f.required_airspeed_m_s > 0.f && runway_ok)
			? std::clamp(f.airspeed_m_s / f.required_airspeed_m_s, 0.f, 1.f) : 0.f;
		if (p.readiness < 1.f) p.blocking_reason = BlockingReason::NEEDS_RUNWAY_SPEED;
		break;
	}

	case TM::CLIFF_LAUNCH:
		// The drop is what converts HEIGHT INTO AIRSPEED: falling h reaches
		// v = sqrt(2gh), so clearing the stall speed needs h = v_stall^2/(2g).
		// This used to report takeoff_run_distance_m, which tonton_analysis.h:236
		// documents as "Required runway length" -- a horizontal distance
		// reported in a field named for a vertical one. At g <= 0 there is
		// nothing to fall through and no drop can be derived, so report 0
		// rather than divide.
		p.required_drop_m = (f.gravity_m_s2 > 0.f && f.required_airspeed_m_s > 0.f)
			? (f.required_airspeed_m_s * f.required_airspeed_m_s)
			  / (2.f * f.gravity_m_s2)
			: 0.f;
		p.feasible  = (f.substrate == Substrate::CLIFF_EDGE);
		p.readiness = p.feasible ? 1.f : 0.f;
		if (!p.feasible) p.blocking_reason = BlockingReason::NEEDS_ELEVATION;
		break;

	case TM::ASSISTED_LAUNCH:
		p.feasible  = (f.substrate == Substrate::PERCH);
		p.readiness = p.feasible ? 1.f : 0.f;
		if (!p.feasible) p.blocking_reason = BlockingReason::NEEDS_PERCH;
		break;

	case TM::IMPOSSIBLE:
	default:
		p.feasible  = false;
		p.readiness = 0.f;
		p.blocking_reason = FirstFailedConstraint(f);
		break;
	}

	return p;
}

LaunchPlan PlanLaunch(const Output& analysis, const MyopicInput& in, float airspeed_m_s)
{
	if (!analysis.aerial.has_value()) {
		LaunchPlan p;
		p.blocking_reason = BlockingReason::NO_AERIAL_ANALYSIS;
		return p;
	}

	const auto& aerial = *analysis.aerial;
	const auto& t = aerial.takeoff;

	LaunchFacts f;
	f.mode                       = t.mode;
	f.required_airspeed_m_s      = float(aerial.min_flight_speed_m_s);
	f.airspeed_m_s               = airspeed_m_s;
	f.gravity_m_s2               = in.gravity_m_s2;
	f.required_jump_velocity_m_s = float(t.required_jump_velocity_m_s);
	f.has_jump_analysis          = analysis.jumping.has_value();
	f.available_jump_velocity_m_s = f.has_jump_analysis
		? float(analysis.jumping->takeoff_velocity_m_s) : 0.f;
	f.substrate                  = in.substrate;
	f.can_use_water_taxi         = t.can_use_water_taxi;
	f.wing_loading_ok            = t.constraints.wing_loading_ok;
	f.power_loading_ok           = t.constraints.power_loading_ok;
	f.aspect_ratio_ok            = t.constraints.aspect_ratio_ok;
	f.leg_strength_ok            = t.constraints.leg_strength_ok;

	return PlanLaunch(f);
}

} // namespace TonTon
