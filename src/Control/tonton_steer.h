#ifndef TONTON_STEER_H
#define TONTON_STEER_H

#include "Control/tonton_envelope.h"

// NOTE: tonton_analysis.h must never be included here or in tonton_steer.cpp.
// Steer operates only on Envelope. This is what makes drowning-in-data
// structurally impossible rather than merely discouraged.

namespace TonTon {

struct SteerCommand {
	float angle_error_rad{0}; // signed, current heading -> desired heading
	float current_speed_m_s{0};
	float desired_speed_m_s{0};
	float dt_s{1.f / 60.f};
};

struct SteerState {
	float prev_turn_rate_rad_s{0};
	float prev_accel_m_s2{0};
};

struct SteerResult {
	float turn_rate_rad_s{0};
	float accel_m_s2{0};
};

SteerResult Steer(const Envelope& env, SteerState& state, const SteerCommand& cmd);

} // namespace TonTon

#endif // TONTON_STEER_H
