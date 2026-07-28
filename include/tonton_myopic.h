#ifndef TONTON_MYOPIC_H
#define TONTON_MYOPIC_H

#include <cstdint>
#include <glm/vec3.hpp>
#include <glm/gtc/quaternion.hpp>

namespace TonTon {

struct Output;

enum class LocomotionMode : uint8_t {
	TERRESTRIAL, AERIAL, AQUATIC, SERPENTINE, CLIMBING, BRACHIATION,
	// JUMPING is a one-shot event, not a sustained mode.
};

enum class Substrate : uint8_t { GROUND, WATER, PERCH, CLIFF_EDGE };

enum class BlockingReason : uint8_t {
	NONE, NEEDS_RUNWAY_SPEED, NEEDS_ELEVATION, NEEDS_PERCH,
	WING_LOADING, POWER_LOADING, ASPECT_RATIO, LEG_STRENGTH,
	NO_AERIAL_ANALYSIS,
	// The mode the caller says the creature is CURRENTLY in has no analysis
	// section, so no envelope exists and no control was computed. This is a
	// caller error, not a creature limitation, and it deliberately does NOT
	// live in `stability`: a stability of -1 is a perfectly ordinary reading
	// for a creature asking for twice the turn it can deliver, so using it as
	// an error sentinel would make the two indistinguishable.
	MODE_UNAVAILABLE,

	// The analysis classified takeoff as IMPOSSIBLE, but not because any single
	// named constraint failed -- ClassifyMode (tonton_takeoffanalysis.cpp) also
	// reaches IMPOSSIBLE from threshold misses with every `*_ok` flag true.
	// Reporting NONE there described an infeasible plan as unobstructed, which
	// is uninterpretable for a caller; this says "the analysis says no, and it
	// did not blame one constraint".
	TAKEOFF_IMPOSSIBLE,
	// JUMP_LAUNCH, but Analysis_TakeoffAnalysis::required_jump_velocity_m_s is
	// exactly 0. That means the analysis never computed the requirement, NOT
	// that the jump is free -- `required <= available` is trivially true at 0
	// and would clear a creature for a launch it was never sized for.
	JUMP_REQUIREMENT_UNKNOWN,
	// The launch has to push off the substrate (legs, or wings spooling up
	// while standing) and the creature is in open water, which bears neither --
	// unless the analysis flagged it as a water-taxiing flyer.
	NEEDS_SOLID_SUBSTRATE,

	// The creature HAS an Analysis_Aerial section, but the aerial envelope it
	// produces is not one this module can control: no usable speed band, or no
	// mechanical power surplus over the cost of level flight
	// (ExtractEnvelope, src/Control/tonton_envelope.cpp).
	//
	// This is deliberately a THIRD reading, distinct from both of its
	// neighbours, because a caller has to act differently on each:
	//   NO_AERIAL_ANALYSIS      - not a flyer at all; never plan a takeoff.
	//   CANNOT_SUSTAIN_FLIGHT   - shaped like a flyer, but this creature at this
	//                             gravity cannot hold level flight. Do not run
	//                             it down a runway; it will reach flight speed
	//                             and there will be nothing there.
	//   MODE_UNAVAILABLE        - the caller asked for a mode that does not
	//                             exist for this creature: a caller error.
	//
	// Without it the launch planner cleared a takeoff (readiness 1.0,
	// launch_feasible true) into a mode ExtractEnvelope then refused to control,
	// so a caller doing exactly what it was told got MODE_UNAVAILABLE -- an
	// error code the module itself had produced. Measured on batto.glb, which
	// has a NEGATIVE mechanical surplus (152.6 W available against 280.1 W
	// required); that is honest propagation of the known Species.Bat clade
	// defect, not something to be relaxed away in ExtractEnvelope.
	CANNOT_SUSTAIN_FLIGHT,
};

// How the creature is turning THIS frame.
//  LATERAL - the turn is paid for out of the mode's own lateral acceleration
//            budget (Envelope::max_lateral_accel), whatever the creature is
//            pushing against: ground, rock, branch or water. It means "not a
//            banked or yawed AERIAL turn" and says nothing about the substrate.
//            It was spelled GROUND, which a caller reading `strategy` on a
//            swimming shark would have taken as a claim about the seabed. Only
//            AERIAL envelopes carry an AerialAuthority, so AQUATIC, SERPENTINE,
//            CLIMBING and BRACHIATION all report LATERAL.
//  YAW     - flat turn about the yaw axis: available instantly, usually weak.
//  BANK    - rolled turn: must be rolled into first, then delivers far more
//            turn rate, and loads the wings (see the load-factor stall coupling).
// Public because MyopicOutput reports it: a caller that cannot see the strategy
// cannot roll the mesh, which would make the whole bank model invisible.
enum class TurnStrategy : uint8_t { LATERAL, YAW, BANK };

// All members are plain floats with units in their names so the engine can
// fill this in without depending on TonTon's Quantity system.
struct MyopicInput {
	LocomotionMode mode{LocomotionMode::TERRESTRIAL};
	LocomotionMode target_mode{LocomotionMode::TERRESTRIAL};

	glm::vec3 position{0};
	glm::quat orientation{1, 0, 0, 0};
	glm::vec3 velocity_m_s{0};          // world space, relative to the ground
	glm::vec3 angular_velocity_rad_s{0}; // body space

	glm::vec3 target_position{0};
	float     desired_speed_m_s{-1.f};  // < 0 -> controller picks cruise

	glm::vec3 medium_velocity_m_s{0};   // wind or current, world space
	float     height_above_ground_m{0};
	Substrate substrate{Substrate::GROUND};

	// A world fact the caller owns, like the two above. Analysis_Physical has
	// no gravity field and Input::environment never reaches Output, so this is
	// the only honest source. Do NOT hardcode 9.81 anywhere in the module --
	// the library explicitly supports low-gravity settings (see the Titan
	// dragonfly example in README.md).
	float gravity_m_s2{9.81f};

	int   current_gait{0};              // terrestrial: 0 walk, 1 trot, 2 gallop
	float dt_s{1.f / 60.f};
};

struct MyopicOutput {
	glm::vec3 linear_acceleration_m_s2{0};   // world space

	// A RATE, not an acceleration: Steer's angular channel commands a turn
	// rate directly (see SteerResult::turn_rate_rad_s). The old name
	// `angular_acceleration_rad_s2` was a lie about both the dimension and the
	// quantity, and this codebase carries compile-time dimensional analysis
	// precisely because that class of mistake is expensive.
	// Only the world-up (y) component is ever populated: this module is a
	// heading-plane controller and has no pitch or roll channel.
	glm::vec3 angular_velocity_rad_s{0};

	// 1 comfortable, 0 at limits, < 0 exceeding.
	//
	// On blocking_reason == MODE_UNAVAILABLE these three read 0, not their
	// defaults: no envelope existed, so no control was computed and the
	// creature has NO authority on any channel. The defaults said "fully
	// comfortable on every axis" for a mode the creature does not possess,
	// which is a reassuring lie to any caller keyed on `stability` -- and
	// `stability` is exactly the field documented for that use. 0 ("at the
	// limit / no authority") is the least-wrong reading in the existing
	// convention; there is no separate "not applicable" value and inventing a
	// sentinel here is the very thing MODE_UNAVAILABLE exists to avoid.
	//
	// TODO(follow-up): `stability` does not currently ORDER correctly across
	// those two situations. MODE_UNAVAILABLE reads exactly 0, while a nominal,
	// successfully completing takeoff run reads -12.6 (batto, gait 0) to -327929
	// (dragonfly, gait 0) -- driven by u_speed = ground_speed/gait_max_speed
	// (~13.6 during a launch run, which is the adjudicated division of labour) and
	// by the degenerate tau_linear recorded at tonton_envelope.cpp. So a caller
	// keyed on `stability` sees "a mode you do not have" scoring strictly better
	// than "a textbook takeoff". Each half is individually adjudicated; it is
	// their INTERACTION that is new, and fixing it means deciding what the scale
	// means below 0 rather than adjusting either half.
	//
	// A SECOND, SEPARATE COLLISION WITH 0, for the same reason. Giving
	// `desired_speed_m_s < 0` makes the controller pick the mode's cruise speed,
	// and for three modes plus one gait the analysis states only ONE
	// characteristic speed, so cruise == max and u_speed is identically 1:
	//   SERPENTINE (eel 0.4136 m/s), CLIMBING (treefrog 0.1053), BRACHIATION,
	//   and TERRESTRIAL at gait 0, whose ceiling IS optimal_speed_m_s.
	// Those creatures read stability == 0 while doing nothing whatsoever unusual.
	// Not a defect in this layer -- there is no second number to use, and
	// fabricating a fraction of the first is exactly what this module refuses --
	// but a caller keyed on `stability == 0` must not read it as "at a limit"
	// without also checking `blocking_reason` and the mode.
	float stability{1.f};
	float speed_headroom{1.f};
	float turn_headroom{1.f};

	// "The speed being asked for exceeds what THIS GAIT can deliver -- try a
	// faster one." Meaningful ONLY for LocomotionMode::TERRESTRIAL, the one mode
	// with a gait concept (the one arm that reads MyopicInput::current_gait);
	// always false for every other mode.
	//
	// It used to be computed unconditionally, so a dragonfly in AERIAL and a
	// shark in AQUATIC were both told to change gait during a launch run --
	// uninterpretable, since neither caller has a gait to change, and carrying
	// no information beyond `speed_headroom < 0`.
	//
	// KNOWN OVERLOAD, on terrestrial envelopes: it means both "you asked for
	// more than this gait can do" and "this gait cannot reach flight speed"
	// (the launch floor at ComputeMyopicControl). Both are genuinely "this gait
	// is not enough", so they share the flag; a caller needing to tell them
	// apart reads target_mode.
	bool  suggest_gait_change{false};

	// Which mechanism produced the turn this frame, and the bank angle actually
	// achieved (signed; right-banked is right-turning). The caller needs both to
	// roll the mesh -- a banked flyer rendered wings-level is the whole Task 5
	// model made invisible.
	TurnStrategy strategy{TurnStrategy::LATERAL};
	float        bank_angle_rad{0.f};

	float          transition_readiness{0.f}; // [0,1]; meaningful when target != mode
	BlockingReason blocking_reason{BlockingReason::NONE};
	float          touchdown_speed_m_s{0.f};  // aerial only

	// How far this creature must FALL to reach flight speed: h = v_stall^2/(2g).
	// Populated whenever a launch plan exists at all (target_mode == AERIAL and
	// mode != AERIAL); 0 when gravity is 0, where no drop converts into airspeed.
	//
	// It is a readout, not a command -- this module has no vertical channel. But
	// without it a CLIFF_LAUNCH creature's caller learns only
	// blocking_reason == NEEDS_ELEVATION and has no way to find out HOW MUCH
	// elevation, which makes the reason unactionable: the number was derived,
	// lived in LaunchPlan, and reached nobody.
	float required_drop_m{0.f};

	// Whether the launch is possible AT ALL from here, as distinct from ready
	// NOW. These are genuinely two facts, and blocking_reason carries only the
	// second: a RUNNING_TAKEOFF creature reports NEEDS_RUNWAY_SPEED both while it
	// is accelerating down a perfectly good runway (readiness rising, feasible)
	// and while it stands on a perch it can never take off from (readiness pinned
	// at 0, not feasible). A caller that cannot tell those apart will either give
	// up on a run that was about to succeed or keep accelerating forever.
	// Meaningless (false) unless target_mode == AERIAL and mode != AERIAL.
	bool launch_feasible{false};

	// Descriptive only. The caller owns all jump timing: a jump has a crouch
	// pre-animation, so no single frame is the right one to apply an impulse.
	float     required_jump_velocity_m_s{0.f};
	glm::vec3 jump_direction{0};
	bool      jump_feasible{false};
};

// Caller-owned, one per creature. Zero-initialised is a valid cold start.
//
// These are exactly the three scalars the steering layer owns, stored in the
// same form it produced them. They were previously world-space vec3s that the
// entry point reconstructed each frame, which lost information: a magnitude is
// unsigned, so a BRAKING creature came back as if it had been accelerating
// forward just as hard, and the framerate-correct exponential slew -- the whole
// reason this module works where the 2025 PD attempt did not -- was being fed a
// sign-flipped history. Keep them scalar; do not "helpfully" re-vectorise.
struct MyopicState {
	float prev_turn_rate_rad_s{0}; // signed, about world up
	float prev_accel_m_s2{0};      // signed, along the heading (negative = braking)
	float bank_angle_rad{0};       // signed roll achieved so far
};

MyopicOutput ComputeMyopicControl(
	const Output& analysis, const MyopicInput& input, MyopicState& state);

} // namespace TonTon

#endif // TONTON_MYOPIC_H
