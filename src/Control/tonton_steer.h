#ifndef TONTON_STEER_H
#define TONTON_STEER_H

#include "Control/tonton_envelope.h"
#include <cstdint>

// NOTE: tonton_analysis.h must never be included here or in tonton_steer.cpp.
// Steer operates only on Envelope. This is what makes drowning-in-data
// structurally impossible rather than merely discouraged.

namespace TonTon {

// TurnStrategy is declared in tonton_myopic.h (reached via tonton_envelope.h):
// MyopicOutput has to report it, and the public header must not include this
// internal one.

struct SteerCommand {
	float angle_error_rad{0}; // signed, current heading -> desired heading

	// NON-NEGATIVE: a SPEED, i.e. the magnitude |velocity| in the frame the mode
	// pushes against. There is no reverse channel in this module -- the only
	// linear axis is "along the heading", and going backwards is expressed by
	// turning around, not by a negative speed. ComputeMyopicControl always
	// passes glm::length(...).
	//
	// The convention is stated because Steer used to read it BOTH ways: fabs for
	// the turn and stall channels, signed for u_speed and speed_error. A
	// negative value therefore counted as positive speed against the centripetal
	// and stall budgets while reporting a NEGATIVE u_speed -- i.e. "more
	// comfortable than standing still" -- on the same frame. Both mutants
	// confirmed it was unobserved (dropping the fabs, and adding one, each left
	// all 99 tests green), and Steer is a live internal API exercised directly by
	// ~40 of them. Every channel now reads the magnitude; a negative value is
	// treated exactly as its absolute value rather than being silently
	// meaningful. Pinned by MyopicSteer.SpeedIsAMagnitudeOnEveryChannel.
	float current_speed_m_s{0};

	// NON-NEGATIVE, same convention and for the same reason.
	float desired_speed_m_s{0};
	float dt_s{1.f / 60.f};
	// Banking trades weight for centripetal force, so the bank math needs the
	// local gravity explicitly rather than assuming Earth. TonTon supports
	// low-gravity worlds; see the zero-g note in Steer().
	float gravity_m_s2{9.81f};
};

struct SteerState {
	float prev_turn_rate_rad_s{0};
	float prev_accel_m_s2{0};
	// Roll-in is a process, not an instant: this is the bank angle actually
	// achieved so far, signed by the direction of the turn. Slewed at the
	// creature's own max_roll_rate, which IS the aerial angular time constant.
	float bank_angle_rad{0};
};

struct SteerResult {
	float turn_rate_rad_s{0};
	float accel_m_s2{0};

	float stability{1.f};
	float speed_headroom{1.f};
	float turn_headroom{1.f};
	bool  suggest_gait_change{false};

	TurnStrategy strategy{TurnStrategy::LATERAL};
	float bank_angle_rad{0}; // signed; 0 for LATERAL and for wings-level YAW
};

SteerResult Steer(const Envelope& env, SteerState& state, const SteerCommand& cmd);

} // namespace TonTon

#endif // TONTON_STEER_H
