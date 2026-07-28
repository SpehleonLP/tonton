#include "Control/tonton_launch.h"
#include "tonton_analysis.h"
#include <algorithm>

namespace TonTon {
namespace {

BlockingReason FirstFailedConstraint(const Analysis_TakeoffAnalysis& t)
{
	if (!t.constraints.wing_loading_ok)  return BlockingReason::WING_LOADING;
	if (!t.constraints.power_loading_ok) return BlockingReason::POWER_LOADING;
	if (!t.constraints.aspect_ratio_ok)  return BlockingReason::ASPECT_RATIO;
	if (!t.constraints.leg_strength_ok)  return BlockingReason::LEG_STRENGTH;
	return BlockingReason::NONE;
}

} // namespace

LaunchPlan PlanLaunch(const Output& analysis, const MyopicInput& in, float airspeed_m_s)
{
	using TM = Analysis_TakeoffAnalysis::TakeoffMode;

	LaunchPlan p;
	if (!analysis.aerial.has_value()) {
		p.blocking_reason = BlockingReason::NO_AERIAL_ANALYSIS;
		return p;
	}

	const auto& aerial = *analysis.aerial;
	const auto& t = aerial.takeoff;

	p.required_airspeed_m_s      = float(aerial.min_flight_speed_m_s);
	p.required_jump_velocity_m_s = float(t.required_jump_velocity_m_s);
	p.jump_direction             = glm::vec3(0.f, 1.f, 0.f);
	p.jump_feasible = analysis.jumping.has_value()
		&& float(t.required_jump_velocity_m_s)
		   <= float(analysis.jumping->takeoff_velocity_m_s);

	switch (t.mode) {
	case TM::VERTICAL_LAUNCH:
		p.feasible  = true;
		p.readiness = 1.f;
		break;

	case TM::JUMP_LAUNCH:
		// Descriptive only: the caller plays the crouch and applies the
		// delta-v at whatever frame its animation says.
		p.feasible  = p.jump_feasible;
		p.readiness = p.jump_feasible ? 1.f : 0.f;
		if (!p.jump_feasible) p.blocking_reason = BlockingReason::LEG_STRENGTH;
		break;

	case TM::RUNNING_TAKEOFF: {
		// A water surface counts as runway for a taxiing flyer (pelicans).
		const bool runway_ok = (in.substrate == Substrate::GROUND)
			|| (in.substrate == Substrate::WATER && t.can_use_water_taxi);
		p.feasible = runway_ok;
		p.accelerate_along_heading = runway_ok;
		// Measured in AIRSPEED, so a headwind shortens the runway with no
		// special case at all.
		p.readiness = (p.required_airspeed_m_s > 0.f && runway_ok)
			? std::clamp(airspeed_m_s / p.required_airspeed_m_s, 0.f, 1.f) : 0.f;
		if (p.readiness < 1.f) p.blocking_reason = BlockingReason::NEEDS_RUNWAY_SPEED;
		break;
	}

	case TM::CLIFF_LAUNCH:
		p.required_drop_m = float(t.takeoff_run_distance_m);
		p.feasible  = (in.substrate == Substrate::CLIFF_EDGE);
		p.readiness = p.feasible ? 1.f : 0.f;
		if (!p.feasible) p.blocking_reason = BlockingReason::NEEDS_ELEVATION;
		break;

	case TM::ASSISTED_LAUNCH:
		p.feasible  = (in.substrate == Substrate::PERCH);
		p.readiness = p.feasible ? 1.f : 0.f;
		if (!p.feasible) p.blocking_reason = BlockingReason::NEEDS_PERCH;
		break;

	case TM::IMPOSSIBLE:
	default:
		p.feasible  = false;
		p.readiness = 0.f;
		p.blocking_reason = FirstFailedConstraint(t);
		break;
	}

	return p;
}

} // namespace TonTon
