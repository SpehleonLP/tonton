#include "Control/tonton_steer.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace TonTon {
namespace {

// Speed below which a moving-frame constraint (centripetal budget, banked
// turn) does not apply at all. A pure guard deciding whether a limit is in
// force -- it must NEVER appear inside an arithmetic result, because dividing
// by it does not guard anything, it asserts a fabricated bound (an omega_max
// of ~8000 rad/s, in the case this constant replaced).
constexpr float kSpeedEpsilon = 1e-3f;

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

// Load factor actually AVAILABLE at this airspeed.
//
// AerialAuthority::n_max is evaluated once, at cruise, and stored. The true
// aerodynamic limit is not a constant: the stall boundary of the V-n diagram is
// n(v) = (v / V_stall)^2, so at the stall speed itself the available load
// factor is exactly 1 -- zero bank, zero banked turn rate -- and it rises as
// v^2 until the structural/aerodynamic ceiling n_max takes over. Using the
// cruise figure at low speed would let a near-stall flyer command a turn it
// physically cannot hold, i.e. a non-physical intermediate.
float LoadFactorAtSpeed(const AerialAuthority& a, float speed)
{
	const float n_max = std::max(1.f, a.n_max);
	const float v_stall = float(a.stall_speed);
	// No usable stall speed: the v^2 boundary is undefined, so fall back to the
	// stored ceiling rather than invent one.
	if (!(v_stall > 0.f)) return n_max;
	const float ratio = speed / v_stall;
	return std::clamp(ratio * ratio, 1.f, n_max);
}

// Bank angle the available load factor permits: n = 1/cos(phi).
float MaxBankAngle(const AerialAuthority& a, float speed)
{
	const float n = LoadFactorAtSpeed(a, speed);
	return (n > 1.f) ? std::acos(1.f / n) : 0.f;
}

// Choose the cheaper turn axis by TIME, not by rate. Yaw is available
// immediately; banking must first be rolled into, then delivers far more turn
// rate. Comparing times needs no invented horizon constant -- the old "0.5
// second" horizon in notes.md was a PD gain in disguise.
//
// Pitch authority is deliberately not consulted: Analysis_Aerial's
// max_pitch_rate_rad_s is never populated by the analysis layer (0 for every
// sample), and a decision built on it would be a decision built on nothing.
//
// out_phi_rad is SIGNED by the heading error, so a renderer can use it directly
// and a reversal rolls back through wings-level instead of teleporting.
TurnStrategy ChooseStrategy(const AerialAuthority& a, float angle_error_rad,
                            float speed, float gravity, float& out_phi_rad)
{
	const float err = std::fabs(angle_error_rad);
	out_phi_rad = 0.f;
	if (err < 1e-4f) return TurnStrategy::YAW; // wings level; nothing to decide

	const float yaw_rate = float(a.max_yaw_rate);
	const float t_yaw = (yaw_rate > 0.f) ? err / yaw_rate
	                                     : std::numeric_limits<float>::infinity();

	// Bank hard enough to serve the demand, capped by the load-factor limit
	// AVAILABLE AT THIS SPEED (not the stored cruise-time n_max).
	const float phi_max = MaxBankAngle(a, speed);
	const float roll_rate = float(a.max_roll_rate);
	float t_bank = std::numeric_limits<float>::infinity();
	// NOTE(task-5): the gravity > 0 guard leaves t_bank infinite at zero g, so a
	// zero-g flyer degrades to YAW. That is the honest answer for THIS model:
	// banking works by tilting the weight vector, and with no weight there is
	// nothing to tilt. A zero-g flyer needs a different lateral model (direct
	// thrust vectoring / wing side force), which is out of scope here; see the
	// matching note at tonton_envelope.cpp.
	if (phi_max > 0.f && roll_rate > 0.f && speed > kSpeedEpsilon && gravity > 0.f) {
		const float omega_bank = gravity * std::tan(phi_max) / speed;
		if (omega_bank > 0.f) {
			t_bank = (phi_max / roll_rate) + (err / omega_bank);
		}
	}

	if (t_bank < t_yaw) {
		out_phi_rad = std::copysign(phi_max, angle_error_rad);
		return TurnStrategy::BANK;
	}
	return TurnStrategy::YAW;
}

// Demand/capacity ratio for a minimum-speed mode (stall in air, sharks,
// serpentine undulation floor). Convention, shared with u_speed: 1 = at the
// limit, > 1 = past it -- but inverted, since this is a floor, not a ceiling.
//
// Above the floor this is simply v_min / v. Below the floor that ratio diverges
// as v -> 0, and the divergence is where the old `speed > 1e-3 ? ratio : 0`
// guard put its hole: a creature with a speed floor sitting at exactly zero
// speed -- the worst possible stall -- reported as perfectly idle. Both
// available repairs are wrong on their own: returning infinity poisons every
// downstream consumer of `stability`, and clamping to any finite constant makes
// standing still read BETTER than crawling, an inversion of the very ordering
// this number exists to express.
//
// So below the floor we continue the ratio along its own tangent at v = v_min:
//
//     u(v) = 1 + (v_min - v) / v_min        for v <= v_min
//
// "one, plus the fraction of the required speed you are missing". It is exactly
// the first-order expansion of v_min/v about v_min, so the two pieces agree in
// value AND slope there (C1-continuous, no kink); it is strictly decreasing in
// v everywhere, so it can never invert; and it is finite at v = 0, where it
// reads 2 -- unambiguously past the limit. No epsilon appears anywhere in it:
// the denominator is v_min, which this branch has already established is > 0.
float StallRatio(float v_min, float speed)
{
	if (!(v_min > 0.f)) return 0.f;   // no floor -> nothing to be below
	if (speed >= v_min) return v_min / speed;
	return 1.f + (v_min - speed) / v_min;
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
	// kSpeedEpsilon (file scope) is a pure guard deciding whether the limit
	// applies; it must never appear inside an arithmetic result.
	const float speed = std::fabs(cmd.current_speed_m_s);

	// omega_max is only meaningful once a turning budget applies; it stays 0
	// when none does, which downstream stability accounting reads as "no
	// turning-authority ratio to report" rather than a fabricated bound.
	float omega_max = 0.f;
	if (speed > kSpeedEpsilon) {
		omega_max = float(env.max_lateral_accel) / speed;
	}

	// --- Bank vs. yaw -----------------------------------------------------
	// For a flyer the ground-reaction budget above is the wrong model: the
	// turn rate available is set by which AXIS is being used, and by how far
	// the creature has actually rolled so far. Replace omega_max accordingly.
	TurnStrategy strategy = TurnStrategy::GROUND;
	if (env.aerial.has_value()) {
		const AerialAuthority& a = *env.aerial;

		float phi_target = 0.f;
		strategy = ChooseStrategy(a, cmd.angle_error_rad, speed,
		                          cmd.gravity_m_s2, phi_target);
		// A yawing (or wings-level) creature is rolling back OUT of any bank
		// it still carries; ChooseStrategy already returns phi_target == 0
		// for that case, so the slew below washes the bank out at the same
		// rate it rolled in.

		// Roll-in is a process. Slew the bank toward its target at the roll
		// rate -- this IS the aerial angular time constant, no estimator
		// needed. rate * dt makes it framerate-correct by construction.
		const float max_droll = std::max(0.f, float(a.max_roll_rate)) * dt;
		const float dphi = std::clamp(phi_target - state.bank_angle_rad,
		                              -max_droll, max_droll);
		state.bank_angle_rad += dphi;

		if (strategy == TurnStrategy::YAW) {
			// Yaw is available immediately and does not depend on airspeed or
			// on how far the creature has rolled.
			omega_max = float(a.max_yaw_rate);
		} else if (speed > kSpeedEpsilon) {
			// Turn rate available at the bank angle ACHIEVED SO FAR, not at
			// the target: mid-roll-in the creature genuinely cannot turn as
			// hard as it eventually will. omega_max is a magnitude; the sign
			// of the turn comes from the error, via the demand below.
			omega_max = std::fabs(cmd.gravity_m_s2
			                      * std::tan(state.bank_angle_rad) / speed);
		} else {
			omega_max = 0.f; // banking is a moving-frame constraint
		}
	}

	// Greedy: if the error can be erased this frame, erase exactly it;
	// otherwise go flat out. This is the pre-slew *demand* -- it is allowed
	// to be dt-dependent because it is only a target the slew approaches,
	// never the delivered value.
	const float greedy = cmd.angle_error_rad / dt;
	float turn_demand = greedy;
	if (omega_max > 0.f) {
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
	//
	// TWO ROLES, deliberately kept separable (see the u_turn comment below for
	// the other half):
	//  1. ANTI-OVERSHOOT CLAMP on the delivered rate, immediately below. This
	//     role depends only on the error, tau and dt -- never on omega_max --
	//     so it is completely unaffected by the bank/yaw choice above. Making
	//     omega_max a bank quantity cannot destabilise the clamp.
	//  2. NUMERATOR OF u_turn, the turn channel's demand/capacity ratio. Only
	//     the DENOMINATOR (omega_max) changed in this task, so u_turn's meaning
	//     sharpened from "fraction of the ground centripetal budget" to
	//     "fraction of the authority available about the axis actually being
	//     used, at the bank achieved so far". Both readings answer the same
	//     question; the second is simply the truthful one for a flyer.
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

	// omega_max is 0 when no turning budget applies at all (see above): at
	// near-zero speed there is no centripetal budget to be a fraction of, so
	// u_turn reports 0 (idle) rather than dividing by a fabricated denominator.
	//
	// For a flyer omega_max is now the BANK/YAW authority rather than the
	// ground centripetal budget, so this ratio reads "how much more turn than
	// the chosen axis can currently deliver" -- including, mid-roll-in, the
	// perfectly real fact that a creature that has not finished rolling cannot
	// yet turn as hard as it has been asked to.
	//
	// The numerator is deliberately turn_stop_bound, not turn_demand and not
	// the delivered turn_rate:
	//  - turn_demand is already clamped to +/-omega_max at the point it is
	//    computed above, so |turn_demand| <= omega_max always holds and a
	//    ratio built from it can never exceed 1 -- the turn channel would be
	//    structurally incapable of driving stability negative, no matter how
	//    much turn rate is actually being demanded.
	//  - turn_rate (the delivered, slew-lagged value) reads low immediately
	//    after a step input even while the *demand* is pegged at the limit,
	//    hiding exactly the saturation this ratio exists to surface.
	//  - turn_stop_bound (= angle_error_rad / max(tau, dt)) is the rate the
	//    creature actually wants to sustain to close the error over its own
	//    natural time constant. It is dt-free in the normal dt < tau regime,
	//    it is already computed above for the anti-overshoot clamp, and --
	//    critically -- it is NOT clamped to omega_max, so u_turn can exceed 1
	//    exactly when the demanded turn genuinely exceeds available authority.
	const float u_turn = (omega_max > 0.f)
		? std::fabs(turn_stop_bound) / omega_max : 0.f;

	// Minimum-speed modes (stall in air, sharks, serpentine undulation floor).
	// See StallRatio for the convention and for why the sub-floor branch is a
	// tangent continuation rather than the raw ratio.
	//
	// Load factor n = 1/cos(phi) raises the stall speed as sqrt(n): lift must
	// now support n times the weight, and lift goes as v^2. This is how real
	// flyers fall out of the sky -- turning hard near minimum speed stalls
	// you. Emergent, not scripted.
	float effective_min_speed = float(env.min_speed);
	if (env.aerial.has_value()) {
		// cos is even, so the signed bank angle needs no fabs here.
		const float cos_phi = std::cos(state.bank_angle_rad);
		if (cos_phi > kSpeedEpsilon && float(env.aerial->stall_speed) > 0.f) {
			effective_min_speed =
				float(env.aerial->stall_speed) / std::sqrt(cos_phi);
		}
	}
	const float u_stall = StallRatio(effective_min_speed, speed);

	const float worst = std::max({u_speed, u_turn, u_stall});

	SteerResult r;
	r.turn_rate_rad_s = turn_rate;
	r.accel_m_s2      = accel;
	r.stability       = 1.f - worst;
	r.speed_headroom  = 1.f - u_speed;
	r.turn_headroom   = 1.f - u_turn;
	r.strategy        = strategy;
	r.bank_angle_rad  = state.bank_angle_rad;
	// Deliberately NOT clamping desired speed to the gait: the stability
	// metric communicates the problem and the external gait selector decides.
	r.suggest_gait_change = cmd.desired_speed_m_s > float(env.max_speed);
	return r;
}

} // namespace TonTon
