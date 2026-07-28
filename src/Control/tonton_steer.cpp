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

// Clamp `value` into [0, bound] or [bound, 0], matching bound's sign. Used to
// pin a delivered rate/accel so it cannot cross past the point it is meant to
// approach.
float ClampTowardZero(float value, float bound)
{
	return (bound >= 0.f) ? std::clamp(value, 0.f, bound)
	                      : std::clamp(value, bound, 0.f);
}

} // namespace

SteerResult Steer(const Envelope& env, SteerState& state, const SteerCommand& cmd)
{
	const float dt = (cmd.dt_s > 0.f) ? cmd.dt_s : 1.f / 60.f;

	// NOTE: tau_linear is deliberately reused here for the angular (turn)
	// channel as well as the linear (accel) channel -- there is no separate
	// angular time constant yet. A `tau_angular` may follow in a later task
	// once a mode supplies one; until then this is a known simplification,
	// not an oversight.

	// --- Turn -------------------------------------------------------------
	// Turn rate available at THIS speed. omega = a_lat / v: a fast animal is
	// grip-limited by its centripetal budget. At v = 0 there is no
	// centripetal requirement at all -- a standing animal pivots on the
	// spot -- so the lateral-accel limit does not apply and is skipped
	// entirely rather than evaluated against a fabricated denominator.
	// kSpeedEpsilon is a pure guard deciding whether the limit applies; it
	// must never appear inside an arithmetic result.
	constexpr float kSpeedEpsilon = 1e-3f;
	const float speed = std::fabs(cmd.current_speed_m_s);

	// omega_max is only meaningful once a centripetal budget applies (see
	// the note above kSpeedEpsilon); it stays 0 below the epsilon, which
	// downstream stability accounting reads as "no turning-authority ratio
	// to report" rather than a fabricated bound.
	float omega_max = 0.f;

	// Greedy: if the error can be erased this frame, erase exactly it;
	// otherwise go flat out. This is the pre-slew *demand* -- it is allowed
	// to be dt-dependent because it is only a target the slew approaches,
	// never the delivered value.
	const float greedy = cmd.angle_error_rad / dt;
	float turn_demand = greedy;
	if (speed > kSpeedEpsilon) {
		omega_max = float(env.max_lateral_accel) / speed;
		turn_demand = std::clamp(turn_demand, -omega_max, omega_max);
	}

	const float turn_alpha = SlewAlpha(dt, float(env.tau_linear));
	const float turn_rate_slewed = Approach(state.prev_turn_rate_rad_s, turn_demand, turn_alpha);
	// Store the unclamped slew value, not the clamped output: clamping is a
	// property of what we deliver this frame, not of the animal's actual
	// angular momentum. Storing the clamped value would reset momentum to
	// zero the instant the bound bites (e.g. the target crossing the
	// creature's nose), forcing counter-steer to restart from a standstill
	// instead of from where the slew actually is.
	state.prev_turn_rate_rad_s = turn_rate_slewed;

	// Stopping-angle bound: a rate w decaying with slew time constant tau
	// sweeps approximately w*tau before it dies out, so non-overshoot
	// requires |w| <= |error| / tau. This is dt-free -- it comes entirely
	// from the Envelope and the current error -- unlike clamping to
	// error/dt, which keeps the error trajectory dt-independent but makes
	// the delivered *signal* itself discontinuous at a rate that scales as
	// O(1/dt) (framerate-dependent jerk for anything downstream that
	// differentiates the output). Tighter than error/dt whenever dt < tau --
	// but ONLY then: when dt > tau (a coarse frame relative to a fast slew,
	// e.g. tau=0.025 driven at 16 Hz), error/tau alone permits rate*dt to
	// exceed the current error, i.e. a real overshoot in one step (measured:
	// 83% overshoot with 11 sign flips over three frames). Dividing by
	// max(tau, dt) takes whichever bound is tighter in each regime: dt-free
	// stopping-angle when dt < tau (where the jerk argument above applies),
	// hard error/dt non-overshoot when dt > tau (where the slew is already
	// near pass-through, so the jerk argument doesn't apply anyway). This
	// also subsumes the old tau<=0 special case for free: max(tau, dt) == dt
	// whenever tau is non-positive, since dt is always > 0 here.
	const float tau = float(env.tau_linear);
	const float turn_stop_bound = cmd.angle_error_rad / std::max(tau, dt);
	const float turn_rate = ClampTowardZero(turn_rate_slewed, turn_stop_bound);

	// --- Speed ------------------------------------------------------------
	const float speed_error = cmd.desired_speed_m_s - cmd.current_speed_m_s;
	const float greedy_accel = speed_error / dt;
	const float accel_demand = std::clamp(greedy_accel,
	                                      -float(env.max_brake),
	                                       float(env.max_accel));

	const float accel_alpha = SlewAlpha(dt, float(env.tau_linear));
	const float accel_slewed = Approach(state.prev_accel_m_s2, accel_demand, accel_alpha);
	// Same reasoning as the turn channel: keep the slew's own momentum in
	// state, clamp only the value handed back to the caller.
	state.prev_accel_m_s2 = accel_slewed;

	// Same stopping-distance bound as the turn channel (see the comment
	// above turn_stop_bound for why max(tau, dt), not tau alone), applied to
	// speed error instead of heading error.
	const float accel_stop_bound = speed_error / std::max(tau, dt);
	const float accel = ClampTowardZero(accel_slewed, accel_stop_bound);

	// --- Stability --------------------------------------------------------
	// Three pure demand/capacity ratios; the worst one wins. 1 = idle,
	// 0 = at a limit, negative = past it. No weights, nothing to tune.
	const float u_speed = (float(env.max_speed) > 0.f)
		? cmd.current_speed_m_s / float(env.max_speed) : 0.f;

	// omega_max is 0 below kSpeedEpsilon (see above): at near-zero speed
	// there is no centripetal budget to be a fraction of, so u_turn reports
	// 0 (idle) rather than dividing by a fabricated denominator.
	const float u_turn = (omega_max > 0.f)
		? std::fabs(turn_demand) / omega_max : 0.f;

	// Minimum-speed modes (stall in air, sharks, serpentine undulation floor).
	// Aerial replaces this in Task 5 with a bank-corrected stall speed.
	const float u_stall = (float(env.min_speed) > 0.f && speed > 1e-3f)
		? float(env.min_speed) / speed : 0.f;

	const float worst = std::max({u_speed, u_turn, u_stall});

	SteerResult r;
	r.turn_rate_rad_s = turn_rate;
	r.accel_m_s2      = accel;
	r.stability       = 1.f - worst;
	r.speed_headroom  = 1.f - u_speed;
	r.turn_headroom   = 1.f - u_turn;
	// Deliberately NOT clamping desired speed to the gait: the stability
	// metric communicates the problem and the external gait selector decides.
	r.suggest_gait_change = cmd.desired_speed_m_s > float(env.max_speed);
	return r;
}

} // namespace TonTon
