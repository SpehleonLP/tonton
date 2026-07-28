#include "tonton_myopic.h"
#include "Control/tonton_envelope.h"
#include "Control/tonton_launch.h"
#include "Control/tonton_steer.h"
#include "tonton_analysis.h"

#include <glm/geometric.hpp>
#include <cmath>

namespace TonTon {
namespace {

// Negligible-length guard for a direction that is about to be normalised.
// Deliberately not shared with the speed/angle epsilons in tonton_steer.cpp:
// this one decides whether a DIRECTION exists at all, and it never appears in
// an arithmetic result.
constexpr float kDirEpsilon = 1e-4f;

// Horizontal (heading-plane) part of a world vector, normalised, or the zero
// vector when there is no horizontal part at all.
//
// A creature pointing straight up has NO heading. glm::normalize on a
// zero-length vector produces NaN, and NaN in `forward` poisons every field of
// MyopicOutput -- including `stability`, which the caller uses to decide
// whether to ragdoll.
glm::vec3 FlattenDirection(glm::vec3 v)
{
	v.y = 0.f;
	const float len = glm::length(v);
	return (len > kDirEpsilon) ? (v / len) : glm::vec3(0.f);
}

} // namespace

MyopicOutput ComputeMyopicControl(
	const Output& analysis, const MyopicInput& input, MyopicState& state)
{
	MyopicOutput out;

	// AIRSPEED DISCIPLINE: computed exactly once, used for every speed
	// comparison downstream. Ground speed is used only for runway distance.
	const glm::vec3 airspeed_vec = input.velocity_m_s - input.medium_velocity_m_s;
	const float airspeed = glm::length(airspeed_vec);

	auto env = ExtractEnvelope(analysis, input.mode, input.current_gait, input.gravity_m_s2);
	if (!env.has_value()) {
		// The caller named a mode this creature has no analysis for. Reported
		// through blocking_reason, NOT through a magic `stability = -1`:
		// stability legitimately reaches -1 (and far below) whenever the
		// demanded turn is about twice the available authority, so overloading
		// it would make a routine hard turn indistinguishable from a caller
		// error. Every other field keeps its default; no control was computed.
		out.blocking_reason = BlockingReason::MODE_UNAVAILABLE;
		return out;
	}

	// Launch planning: the ONLY transition carrying physics is * -> AERIAL,
	// because airspeed is the only precondition a creature must generate
	// itself. Every other target mode leaves steering unchanged.
	bool launch_run = false;
	if (input.target_mode == LocomotionMode::AERIAL
	    && input.mode != LocomotionMode::AERIAL) {
		LaunchPlan plan = PlanLaunch(analysis, input, airspeed);
		out.transition_readiness       = plan.readiness;
		out.blocking_reason            = plan.blocking_reason;
		out.required_jump_velocity_m_s = plan.required_jump_velocity_m_s;
		out.jump_direction             = plan.jump_direction;
		out.jump_feasible              = plan.jump_feasible;
		launch_run = plan.accelerate_along_heading;
	}

	// ONE forward, used for BOTH the heading error and the direction of the
	// commanded acceleration. The plan used the flattened forward for the
	// error and the unflattened one for the acceleration, which point in
	// different directions the moment a flyer pitches.
	//
	// The flattened one wins because this module is a heading-plane
	// controller end to end: the only angular channel it has is a turn rate
	// about world up, the target bearing it tracks is a horizontal angle, and
	// the envelope's max_accel/max_speed are level-flight figures. Emitting
	// acceleration along the pitched body axis would turn a pure speed command
	// into an uncommanded climb that no channel here ever budgeted for, and
	// pitch is explicitly out of scope (the analysis layer never populates
	// max_pitch_rate_rad_s -- see the note in tonton_steer.cpp). Climb and dive
	// belong to the caller.
	//
	// Fallback chain when the creature points straight up or straight down and
	// has no heading at all: the horizontal direction it is actually MOVING,
	// then nothing. "Nothing" is the honest answer -- there is no direction to
	// push in -- and it is a zero vector rather than a NaN one.
	glm::vec3 forward = FlattenDirection(input.orientation * glm::vec3(0.f, 0.f, 1.f));
	if (glm::length(forward) <= kDirEpsilon) {
		forward = FlattenDirection(airspeed_vec);
	}

	// Heading error toward the target, about the world up axis: the signed
	// rotation FROM the current heading TO the target bearing, so that a
	// positive error asks for a positive rotation about +Y. That is
	// atan2(dot(up, cross(f, t)), dot(f, t)), and with up = +Y the numerator is
	// f.z*t.x - f.x*t.z. NOTE the order: the plan wrote f.x*t.z - f.z*t.x, the
	// angle from the target back to the heading, which made the controller
	// steer away from its target at exactly the commanded rate.
	const glm::vec3 to_target = FlattenDirection(input.target_position - input.position);
	float angle_error = 0.f;
	if (glm::length(to_target) > kDirEpsilon && glm::length(forward) > kDirEpsilon) {
		angle_error = std::atan2(
			forward.z * to_target.x - forward.x * to_target.z,
			glm::dot(forward, to_target));
	}

	SteerCommand cmd;
	// Steering toward the launch precondition instead of the target: accelerate
	// along the current heading until we have made our own airspeed.
	cmd.angle_error_rad   = launch_run ? 0.f : angle_error;
	cmd.current_speed_m_s = airspeed;
	cmd.desired_speed_m_s = launch_run ? float(env->max_speed)
	                      : (input.desired_speed_m_s >= 0.f
	                             ? input.desired_speed_m_s
	                             : float(env->max_speed));
	cmd.dt_s          = input.dt_s;
	cmd.gravity_m_s2  = input.gravity_m_s2;

	// The state round-trip is field-for-field: MyopicState holds exactly the
	// scalars SteerState owns, in the same signed form. Reconstructing them
	// from world-space vectors (the plan's `glm::length(prev_linear_accel)`)
	// threw away the sign, so a braking creature was fed back into the slew as
	// if it had been accelerating forward just as hard.
	SteerState ss;
	ss.prev_turn_rate_rad_s = state.prev_turn_rate_rad_s;
	ss.prev_accel_m_s2      = state.prev_accel_m_s2;
	ss.bank_angle_rad       = state.bank_angle_rad;

	SteerResult r = Steer(*env, ss, cmd);

	state.prev_turn_rate_rad_s = ss.prev_turn_rate_rad_s;
	state.prev_accel_m_s2      = ss.prev_accel_m_s2;
	state.bank_angle_rad       = ss.bank_angle_rad;

	out.linear_acceleration_m_s2 = forward * r.accel_m_s2;
	out.angular_velocity_rad_s   = glm::vec3(0.f, r.turn_rate_rad_s, 0.f);
	out.stability           = r.stability;
	out.speed_headroom      = r.speed_headroom;
	out.turn_headroom       = r.turn_headroom;
	out.suggest_gait_change = r.suggest_gait_change;
	out.strategy            = r.strategy;
	out.bank_angle_rad      = r.bank_angle_rad;

	// Landing is a readout, not a steering goal: a pelican physically cannot
	// arrive slower than its stall speed, and the caller decides whether that
	// becomes a stumble or a ragdoll.
	if (input.mode == LocomotionMode::AERIAL) {
		out.touchdown_speed_m_s = float(env->min_speed);
	}

	return out;
}

} // namespace TonTon
