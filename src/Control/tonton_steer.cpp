#include "Control/tonton_steer.h"
#include <algorithm>
#include <cmath>

namespace TonTon {
namespace {

// Framerate-correct exponential slew. This exact form is why the 2025 PD
// attempt failed and this one does not: alpha depends on dt, so 16 Hz and
// 120 Hz reach the same place in the same wall-clock time. Never replace it
// with a bare lerp factor.
float SlewAlpha(float dt_s, float tau_s)
{
	if (!(tau_s > 0.f)) return 1.f; // degenerate tau -> no smoothing
	return 1.f - std::exp(-dt_s / tau_s);
}

float Approach(float prev, float demand, float alpha)
{
	return prev + (demand - prev) * alpha;
}

} // namespace

SteerResult Steer(const Envelope& env, SteerState& state, const SteerCommand& cmd)
{
	const float dt = (cmd.dt_s > 0.f) ? cmd.dt_s : 1.f / 60.f;

	// --- Turn -------------------------------------------------------------
	// Turn rate available at THIS speed. omega = a_lat / v, so a standing
	// animal pivots freely and a fast one is grip-limited. The guard keeps
	// v = 0 finite without introducing a tunable: it only bounds the demand,
	// which the greedy step below bounds again anyway.
	const float speed = std::fabs(cmd.current_speed_m_s);
	const float omega_max = (speed > 1e-3f)
		? float(env.max_lateral_accel) / speed
		: float(env.max_lateral_accel) / 1e-3f;

	// Greedy: if the error can be erased this frame, erase exactly it;
	// otherwise go flat out. This cannot overshoot, by construction.
	const float greedy = cmd.angle_error_rad / dt;
	const float turn_demand = std::clamp(greedy, -omega_max, omega_max);

	const float turn_alpha = SlewAlpha(dt, float(env.tau_linear));
	float turn_rate = Approach(state.prev_turn_rate_rad_s, turn_demand, turn_alpha);

	// Safety clamp: the slew carries momentum from previous frames, so once
	// the error has shrunk, a still-high slewed rate can carry PAST zero
	// error this frame -- the exact overshoot the 2025 PD attempt produced.
	// `greedy` (== error / dt) is the rate that exactly zeroes the CURRENT
	// error in this frame; never let the delivered rate exceed it, and never
	// let it flip past zero. This uses only values already in scope -- no
	// new tunable, just the same greedy bound applied after the slew instead
	// of only before it.
	if (greedy >= 0.f) turn_rate = std::clamp(turn_rate, 0.f, greedy);
	else                turn_rate = std::clamp(turn_rate, greedy, 0.f);

	state.prev_turn_rate_rad_s = turn_rate;

	// --- Speed ------------------------------------------------------------
	const float speed_error = cmd.desired_speed_m_s - cmd.current_speed_m_s;
	const float greedy_accel = speed_error / dt;
	const float accel_demand = std::clamp(greedy_accel,
	                                      -float(env.max_brake),
	                                       float(env.max_accel));

	const float accel_alpha = SlewAlpha(dt, float(env.tau_linear));
	float accel = Approach(state.prev_accel_m_s2, accel_demand, accel_alpha);

	// Same overshoot guard as the turn channel: never deliver more
	// acceleration than exactly zeroes the current speed error this frame.
	if (greedy_accel >= 0.f) accel = std::clamp(accel, 0.f, greedy_accel);
	else                      accel = std::clamp(accel, greedy_accel, 0.f);

	state.prev_accel_m_s2 = accel;

	SteerResult r;
	r.turn_rate_rad_s = turn_rate;
	r.accel_m_s2      = accel;
	return r;
}

} // namespace TonTon
