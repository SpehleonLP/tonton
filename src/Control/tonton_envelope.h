#ifndef TONTON_ENVELOPE_H
#define TONTON_ENVELOPE_H

#include "tonton_myopic.h"
#include "tonton_units.hpp"
#include <optional>

namespace TonTon {

struct Output;

// Populated only for LocomotionMode::AERIAL. Lives here rather than being read
// from Output at steer time so that Steer never sees Output, and so that tests
// can build one directly (Output's constructor is private).
struct AerialAuthority {
	omega_rad_s  max_roll_rate{};
	omega_rad_s  max_pitch_rate{};
	omega_rad_s  max_yaw_rate{};
	float        n_max{1.f};      // load-factor limit, >= 1
	velocity_m_s stall_speed{};   // at n = 1
	velocity_m_s cruise_speed{};
};

struct Envelope {
	velocity_m_s      max_speed{};
	velocity_m_s      min_speed{};          // 0 for ground, stall for air
	acceleration_m_s2 max_accel{};
	acceleration_m_s2 max_brake{};
	acceleration_m_s2 max_lateral_accel{};  // turn-rate budget; omega = a_lat / v
	length_m          min_turn_radius{};    // 0 = can pivot in place
	time_s            tau_linear{};         // slew time constant
	std::optional<AerialAuthority> aerial{};
};

// Returns nullopt when the requested mode has no analysis section.
// Called every frame; must stay cheap and free of hidden state.
std::optional<Envelope> ExtractEnvelope(
	const Output& analysis, LocomotionMode mode, int gait, float gravity_m_s2);

} // namespace TonTon

#endif // TONTON_ENVELOPE_H
