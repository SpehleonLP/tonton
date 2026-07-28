#ifndef TONTON_LAUNCH_H
#define TONTON_LAUNCH_H

#include "tonton_myopic.h"
#include "tonton_units.hpp"
#include "tonton_analysis.h"

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

// Exactly the facts the mode dispatch below reads, and nothing else.
//
// This exists so the dispatch can be exercised directly. `Output`'s constructor
// is private and only the analysis pipeline can build one, so every launch arm
// that the sample models do not happen to classify into was untestable, and a
// reviewer duly found all four of VERTICAL_LAUNCH, JUMP_LAUNCH, CLIFF_LAUNCH
// and ASSISTED_LAUNCH could be gutted to `feasible = false, readiness = 0` with
// the whole suite still green. Widening the public API to let tests fabricate
// an `Output` would have been a test hack; narrowing the dispatch to the fields
// it actually consumes is a real seam.
struct LaunchFacts {
	using TakeoffMode = Analysis_TakeoffAnalysis::TakeoffMode;

	TakeoffMode mode{TakeoffMode::IMPOSSIBLE};

	// AIRSPEED (|velocity - medium_velocity|) on both sides, never ground
	// speed: a runway is measured in the air the wing sees, which is what makes
	// a headwind shorten it with no special case anywhere below.
	float required_airspeed_m_s{0.f};   // stall / minimum level-flight speed
	float airspeed_m_s{0.f};
	float gravity_m_s2{9.81f};          // for the cliff drop; may be 0

	// 0 means "the analysis did not compute this", not "free". See
	// BlockingReason::JUMP_REQUIREMENT_UNKNOWN.
	float required_jump_velocity_m_s{0.f};
	float available_jump_velocity_m_s{0.f};
	bool  has_jump_analysis{false};

	Substrate substrate{Substrate::GROUND};
	bool      can_use_water_taxi{false};

	bool wing_loading_ok{false};
	bool power_loading_ok{false};
	bool aspect_ratio_ok{false};
	bool leg_strength_ok{false};
};

// The dispatch itself: pure, total over every TakeoffMode, no Output in sight.
LaunchPlan PlanLaunch(const LaunchFacts& facts);

// Thin adapter: reads the facts out of `Output` and defers. `airspeed_m_s` must
// be AIRSPEED, never ground speed (see LaunchFacts).
LaunchPlan PlanLaunch(const Output& analysis, const MyopicInput& in, float airspeed_m_s);

} // namespace TonTon

#endif // TONTON_LAUNCH_H
