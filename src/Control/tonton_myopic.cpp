#include "tonton_myopic.h"
#include "Control/tonton_envelope.h"
#include "Control/tonton_launch.h"
#include "Control/tonton_steer.h"
#include "tonton_analysis.h"

#include <glm/geometric.hpp>
#include <algorithm>
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

// THE FRAME RULE. Read this before adding a mode.
//
// A locomotion mode's envelope -- its grip budget, its centripetal budget, its
// acceleration and its top speed -- describes force it can push through the
// thing it pushes AGAINST. So the speed those limits are evaluated at is the
// speed RELATIVE TO THAT THING, and nothing else:
//
//   TERRESTRIAL, CLIMBING, SERPENTINE, BRACHIATION push against ground, rock,
//   branches. Their budgets are GROUND-frame. A wind blowing past a standing
//   cat changes none of them.
//
//   AERIAL, AQUATIC push against the fluid. Their budgets are MEDIUM-relative:
//   |velocity - medium_velocity| is the only speed a wing or a fin sees.
//
// Feeding airspeed to the ground arm (which this module used to do
// unconditionally) is not a conservative approximation, it is wrong in both
// directions at once. Measured on cat.glb before the fix: a standing cat in a
// 10 m/s crosswind had its pivot rate collapse from 14.19 rad/s to 0.053 and
// reported stability -132.3, while a cat sprinting at 5 m/s downwind in a 5 m/s
// tailwind pivoted on the spot at 14.19 rad/s and was commanded 62.7 m/s^2 of
// acceleration as though it were stationary.
//
// The ONE exception is launch readiness, which is about generating LIFT and so
// is always measured in airspeed no matter which mode the creature is launching
// from. That is handled explicitly at its own use site below.
//
// Runway DISTANCE stays a ground-frame quantity: it is a length of ground.
enum class SpeedFrame : uint8_t { GROUND, MEDIUM };

SpeedFrame SpeedFrameOf(LocomotionMode mode)
{
	switch (mode) {
	case LocomotionMode::AERIAL:
	case LocomotionMode::AQUATIC:
		return SpeedFrame::MEDIUM;
	case LocomotionMode::TERRESTRIAL:
	case LocomotionMode::CLIMBING:
	case LocomotionMode::SERPENTINE:
	case LocomotionMode::BRACHIATION:
		return SpeedFrame::GROUND;
	}
	return SpeedFrame::GROUND;
}

} // namespace

MyopicOutput ComputeMyopicControl(
	const Output& analysis, const MyopicInput& input, MyopicState& state)
{
	MyopicOutput out;

	// Airspeed: what a WING sees. Used for launch readiness (which is about
	// generating lift) and, via the frame rule, for the fluid modes' envelopes.
	const glm::vec3 airspeed_vec = input.velocity_m_s - input.medium_velocity_m_s;
	const float airspeed = glm::length(airspeed_vec);

	// The speed the CURRENT mode's envelope is evaluated against. See the frame
	// rule at SpeedFrameOf: legs are ground-frame, wings and fins are not.
	const SpeedFrame frame = SpeedFrameOf(input.mode);
	const glm::vec3 mode_velocity =
		(frame == SpeedFrame::MEDIUM) ? airspeed_vec : input.velocity_m_s;
	const float mode_speed = glm::length(mode_velocity);

	auto env = ExtractEnvelope(analysis, input.mode, input.current_gait, input.gravity_m_s2);
	if (!env.has_value()) {
		// The caller named a mode this creature has no analysis for. Reported
		// through blocking_reason, NOT through a magic `stability = -1`:
		// stability legitimately reaches -1 (and far below) whenever the
		// demanded turn is about twice the available authority, so overloading
		// it would make a routine hard turn indistinguishable from a caller
		// error. Every other field keeps its default; no control was computed.
		out.blocking_reason = BlockingReason::MODE_UNAVAILABLE;
		// No envelope, no control, and -- the part that was wrong -- no
		// authority either. Leaving these at their defaults reported every
		// channel as fully comfortable for a mode the creature does not have.
		// See the contract note at MyopicOutput::stability.
		out.stability      = 0.f;
		out.speed_headroom = 0.f;
		out.turn_headroom  = 0.f;
		return out;
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
	// push in -- and it is a zero vector rather than a NaN one. The motion used
	// is the mode's own, by the frame rule: a flyer's nose follows its airspeed
	// vector, a runner's follows its ground track.
	glm::vec3 forward = FlattenDirection(input.orientation * glm::vec3(0.f, 0.f, 1.f));
	if (glm::length(forward) <= kDirEpsilon) {
		forward = FlattenDirection(mode_velocity);
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

	// Launch planning: the ONLY transition carrying physics is * -> AERIAL,
	// because airspeed is the only precondition a creature must generate
	// itself. Every other target mode leaves steering unchanged.
	//
	// Placed after `forward` because converting the launch's airspeed
	// requirement into the current mode's frame needs the heading.
	float launch_speed_floor = -1.f;  // < 0 == no launch run in progress
	bool  hold_heading       = false;
	if (input.target_mode == LocomotionMode::AERIAL
	    && input.mode != LocomotionMode::AERIAL) {
		// Readiness is measured in AIRSPEED even from a ground mode: it is
		// about generating lift, and lift does not care what the ground is
		// doing. This is the frame rule's one deliberate exception.
		LaunchPlan plan = PlanLaunch(analysis, input, airspeed);
		out.transition_readiness       = plan.readiness;
		out.blocking_reason            = plan.blocking_reason;
		out.required_jump_velocity_m_s = plan.required_jump_velocity_m_s;
		out.jump_direction             = plan.jump_direction;
		out.jump_feasible              = plan.jump_feasible;

		if (plan.accelerate_along_heading) {
			// COMMAND THE SPEED THE LAUNCH ACTUALLY REQUIRES.
			//
			// This used to command env->max_speed -- the TERRESTRIAL envelope's
			// top speed at the current gait -- while readiness was measured
			// against the AERIAL minimum flight speed. The two are unrelated
			// numbers, and for both sample flyers the ground figure is far
			// smaller (batto: 0.579/0.827/2.089 m/s across its three gaits
			// against a 7.867 m/s stall; dragonfly 0.139 against 2.631), so the
			// run converged at 5-27% readiness and stayed there forever. The
			// takeoff could never complete on either flyer.
			//
			// The requirement is an airspeed; the speed channel is in the
			// current mode's frame. For a ground mode they differ by the wind
			// along the heading -- airspeed_along = v_ground_along - wind_along
			// -- so reaching airspeed R needs ground speed R + wind_along. A
			// headwind (wind_along < 0) shortens the run, with no special case.
			float required = plan.required_airspeed_m_s;
			if (frame == SpeedFrame::GROUND) {
				required += glm::dot(input.medium_velocity_m_s, forward);
			}
			launch_speed_floor = std::max(0.f, required);

			// Steering is suppressed only while the precondition is UNMET.
			// Suppressing it unconditionally left a creature already at flight
			// speed flying straight past its target for as long as it stayed
			// in the launch run.
			hold_heading = (plan.readiness < 1.f);
		}
	}

	SteerCommand cmd;
	// Steering toward the launch precondition instead of the target: accelerate
	// along the current heading until we have made our own airspeed.
	cmd.angle_error_rad   = hold_heading ? 0.f : angle_error;
	cmd.current_speed_m_s = mode_speed;
	// The launch requirement is a FLOOR under the caller's own wish, not a
	// replacement for it: dropping back to env->max_speed the instant readiness
	// hit 1 would command a brake below flight speed and oscillate. When the
	// floor exceeds the gait's top speed, Steer's `suggest_gait_change` fires --
	// which is how the caller learns that THIS gait cannot reach flight speed,
	// instead of watching readiness stall silently.
	cmd.desired_speed_m_s = std::max(
		launch_speed_floor,
		(input.desired_speed_m_s >= 0.f ? input.desired_speed_m_s
		                                : float(env->max_speed)));
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
