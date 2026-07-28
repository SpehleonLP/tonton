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
	// The speed this mode HOLDS when nobody is asking for anything in
	// particular, which is what MyopicInput::desired_speed_m_s < 0 means
	// ("controller picks cruise"). It is NOT max_speed: substituting the ceiling
	// made the modal default command "flat out forever" -- a shark held its
	// anaerobic burst speed (20.29 m/s against a 2.53 m/s cruise, 8.0x)
	// indefinitely, and every creature on defaults reported u_speed == 1, i.e.
	// stability 0, "at the limit", on the one field the public header names as
	// the one a caller keys on.
	//
	// Populated by EVERY arm. Where a mode's analysis states only ONE
	// characteristic speed (SERPENTINE, CLIMBING, BRACHIATION) this is that
	// speed and therefore equals max_speed -- said explicitly at each site, so
	// the equality reads as a fact about the analysis rather than as an
	// oversight.
	velocity_m_s      cruise_speed{};
	acceleration_m_s2 max_accel{};
	acceleration_m_s2 max_brake{};
	acceleration_m_s2 max_lateral_accel{};  // turn-rate budget; omega = a_lat / v
	// SENTINEL at 0: "this mode states no radius constraint", not "can turn
	// infinitely tightly". SERPENTINE, CLIMBING and BRACHIATION always report 0
	// because their analysis sections have no turning-radius field at all.
	// LateralBudget (tonton_envelope.cpp) is the only consumer and short-circuits
	// on <= 0, so the sentinel can neither divide by zero nor fabricate a bound.
	length_m          min_turn_radius{};
	time_s            tau_linear{};         // slew time constant

	// Does this mode have a GAIT CONCEPT at all -- discrete stride patterns the
	// caller can switch between, which is what MyopicInput::current_gait selects
	// and what MyopicOutput::suggest_gait_change advises about?
	//
	// True for TERRESTRIAL only. It is the only arm that reads `gait`, and no
	// other analysis section exposes anything gait-shaped. Steer gates
	// suggest_gait_change on this: emitting it for a mode with no gaits told a
	// dragonfly in AERIAL and a shark in AQUATIC to "change gait", which is not
	// an action either caller can take and which duplicates
	// speed_headroom < 0 exactly.
	bool              has_gaits{false};

	std::optional<AerialAuthority> aerial{};
};

// Returns nullopt when the requested mode has no analysis section.
// Called every frame; must stay cheap and free of hidden state.
std::optional<Envelope> ExtractEnvelope(
	const Output& analysis, LocomotionMode mode, int gait, float gravity_m_s2);

} // namespace TonTon

#endif // TONTON_ENVELOPE_H
