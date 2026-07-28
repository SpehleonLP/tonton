#ifndef TONTON_LAUNCH_H
#define TONTON_LAUNCH_H

#include "tonton_myopic.h"
#include "tonton_units.hpp"

namespace TonTon {

struct Output;

struct LaunchPlan {
	bool           feasible{false};
	float          readiness{0.f};        // [0,1]
	BlockingReason blocking_reason{BlockingReason::NONE};

	float required_airspeed_m_s{0.f};
	float required_drop_m{0.f};

	// Descriptive only; the caller owns jump timing.
	float     required_jump_velocity_m_s{0.f};
	glm::vec3 jump_direction{0};
	bool      jump_feasible{false};

	bool accelerate_along_heading{false}; // true only for RUNNING_TAKEOFF
};

// `airspeed_m_s` must be AIRSPEED (|velocity - medium_velocity|), never ground
// speed: a runway is measured in the air the wing sees, which is what makes a
// headwind shorten it with no special case anywhere in this function.
LaunchPlan PlanLaunch(const Output& analysis, const MyopicInput& in, float airspeed_m_s);

} // namespace TonTon

#endif // TONTON_LAUNCH_H
