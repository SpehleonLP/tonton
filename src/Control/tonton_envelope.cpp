#include "Control/tonton_envelope.h"
#include "tonton_analysis.h"
#include <algorithm>
#include <cmath>
#include <optional>

namespace TonTon {
namespace {

// Lateral acceleration budget, derived two independent ways and cross-checked:
//   - the grip/thrust budget the creature already has (max_accel)
//   - the budget implied by its own stated minimum turning radius at top speed
// Taking the min keeps us honest when the two disagree. Turn RATE is then
// omega = a_lat / v at steer time, which correctly lets a standing animal pivot
// and stops a galloping one from doing so.
acceleration_m_s2 LateralBudget(acceleration_m_s2 max_accel,
                                velocity_m_s max_speed,
                                length_m min_turn_radius)
{
	if (float(min_turn_radius) <= 0.f) return max_accel;
	const acceleration_m_s2 from_radius = (max_speed * max_speed) / min_turn_radius;
	return (float(from_radius) < float(max_accel)) ? from_radius : max_accel;
}

// Analysis_Aerial has no acceleration field, so derive it. P = F*v and F = m*a
// give a = P/(m*v), evaluated at cruise as a representative figure.
//
// `surplus` must be the *mechanical* power left over after paying for level
// flight -- P_available_mechanical - P_required_mechanical. Feeding in the
// metabolic cruise power instead is wrong twice over: it is inflated by 1/eta
// (eta ~ 0.10-0.23, tonton_aerial.cpp), and it is the power already spent
// holding cruise rather than the power free to change speed.
//
// Guards are written as !(x > 0) so that NaN (for which every comparison is
// false) is rejected too.
acceleration_m_s2 AccelFromPower(power_W surplus, mass_kg mass, velocity_m_s at_speed)
{
	const float p = float(surplus), m = float(mass), v = float(at_speed);
	if (!(p > 0.f) || !(m > 0.f) || !(v > 0.f)) return acceleration_m_s2{0.f};
	if (!std::isfinite(p) || !std::isfinite(m) || !std::isfinite(v)) return acceleration_m_s2{0.f};
	return surplus / (mass * at_speed);
}

// A mode with no usable acceleration is not a usable envelope: every downstream
// consumer divides by it (tau_linear here, the turn slew in Steer). Report
// absence rather than handing back an inf/NaN envelope. Written as !(x > 0) so
// NaN, for which every comparison is false, is rejected too.
bool UsableAccel(acceleration_m_s2 a)
{
	return (float(a) > 0.f) && std::isfinite(float(a));
}

// ...and a mode whose floor is at or above its ceiling is not a usable envelope
// either: every headroom/stability term downstream reads (v - min)/(max - min)
// or a difference of the two, which an inverted band makes meaningless or
// sign-flipped. Reported as absence, exactly as UsableAccel does.
//
// Applied to EVERY arm rather than only the one arm known to be able to invert
// it (see below), because it is a statement about what an Envelope IS, not
// about any one mode's derivation, and a future arm should inherit it for free.
// Verified against the whole sample set: no existing arm's band is inverted at
// the default inputs, so this gate changes no current behaviour -- in
// particular it does not change AERIAL, whose min_speed is a stall speed.
//
// The arm that can actually trip it today is AQUATIC: min_swim_speed_m_s is
// cruise*0.5 and is NOT scaled by exp2(mana.air) at tonton_aquatic.cpp:733,
// while cruise_speed_m_s and burst_speed_m_s both are. The mana axis is a
// supported input, so a sufficiently negative mana.air shrinks the ceiling past
// the fixed floor. That is an upstream inconsistency and is reported as such;
// this gate only stops a nonsensical envelope from reaching a consumer.
//
// Written as !(min < max) so NaN, for which every comparison is false, is
// rejected too.
bool UsableSpeedBand(velocity_m_s min_speed, velocity_m_s max_speed)
{
	const float lo = float(min_speed), hi = float(max_speed);
	if (!std::isfinite(lo) || !std::isfinite(hi)) return false;
	return lo < hi;
}

// Load-factor limit, derived two independent ways and cross-checked:
//   - aerodynamic (V-n) budget: level flight at v needs CL such that lift = W,
//     so at v the wing can generate at most (v/v_stall)^2 times its own weight
//     in lift before it stalls. n_aero = (v/v_stall)^2, evaluated at cruise.
//   - stated radius:  a coordinated turn of radius r at speed v needs
//                     tan(phi) = v^2 / (g*r), and n = 1/cos(phi)
// Taking the min keeps us honest. A large disagreement is a diagnostic about
// the analysis, not about the controller.
//
// NOT used here: Analysis_TakeoffAnalysis::max_instantaneous_lift_N. Despite the
// name it is a *hover* figure — EstimateMaxLift (tonton_takeoffanalysis.cpp:177)
// builds it from flapping tip velocity alone, with no forward-airspeed term. For
// the bat sample it comes out at 7.99 N against 18.7 N of weight, i.e. "cannot
// hover", which is true of most bats and says nothing about turning at 11.9 m/s.
// Feeding it into a cruise load-factor budget yields n < 1, which the floor below
// silently converts to n = 1 and hence to max_lateral_accel = 0 — a flyer that
// cannot turn. See the task-4 notes; this is a deviation from the plan's formula.
//
// Each budget is OPTIONAL. If a budget's inputs are unusable it contributes
// nothing at all rather than contributing 1.f -- contributing 1.f would win the
// min() and silently produce max_lateral_accel = 0, i.e. a flyer that cannot
// turn, which is the "clamp hides a non-physical intermediate" pattern.
float LoadFactorLimit(const Analysis_Aerial& a, float gravity_m_s2)
{
	std::optional<float> n_aero;
	const float v_stall = float(a.min_flight_speed_m_s);
	const float v = float(a.cruise_speed_m_s);
	if (v_stall > 0.f && v > 0.f && v >= v_stall) {
		const float ratio = v / v_stall;
		const float n = ratio * ratio;
		if (std::isfinite(n) && n >= 1.f) n_aero = n;
	}

	std::optional<float> n_radius;
	const float r = float(a.min_turning_radius_m);
	if (r > 0.f && v > 0.f && gravity_m_s2 > 0.f) {
		const float tan_phi = (v * v) / (gravity_m_s2 * r);
		const float n = std::sqrt(1.f + tan_phi * tan_phi); // = 1/cos(atan(tan_phi))
		if (std::isfinite(n) && n >= 1.f) n_radius = n;
	}

	if (n_aero && n_radius) return std::min(*n_aero, *n_radius);
	if (n_aero)   return *n_aero;
	if (n_radius) return *n_radius;
	// Neither budget is derivable: the analysis gave us no usable stall speed,
	// cruise speed or turning radius. Returning 1 here is a statement about
	// degenerate INPUT, not a physical claim that this flyer cannot bank.
	return 1.f;
}

} // namespace

std::optional<Envelope> ExtractEnvelope(
	const Output& analysis, LocomotionMode mode, int gait, float gravity_m_s2)
{
	switch (mode) {
	case LocomotionMode::TERRESTRIAL: {
		if (!analysis.terrestrial.has_value()) return std::nullopt;
		const auto& t = *analysis.terrestrial;

		Envelope e;
		e.max_speed = (gait <= 0) ? t.optimal_speed_m_s
		            : (gait == 1) ? t.max_sustainable_speed_m_s
		                          : t.max_sprint_speed_m_s;
		e.min_speed       = velocity_m_s{0.f};
		if (!UsableSpeedBand(e.min_speed, e.max_speed)) return std::nullopt;
		e.max_accel       = t.max_acceleration_m_s2;
		e.max_brake       = t.max_acceleration_m_s2; // legs brake with the same grip budget
		e.min_turn_radius = t.min_turning_radius_m;
		e.max_lateral_accel = LateralBudget(e.max_accel, e.max_speed, e.min_turn_radius);

		// tau = characteristic speed / acceleration. Dimensionally a time, built
		// from two numbers the plausibility suite already validates, adding no
		// new biological claim. See the spec's rejected-pendulum note.
		//
		// TODO(follow-up): this ratio is degenerate for small creatures, and it
		// is doing a second job it was never derived for.
		//  (a) DEGENERACY. The dragonfly's 41 m/s^2 of walking acceleration
		//      against a 3.6 cm/s optimal walk speed gives tau = 0.87 ms.
		//      Nothing about a dragonfly settles in under a millisecond; the
		//      ratio is small because the numerator is a walking gait's speed
		//      while the denominator is a whole-body acceleration, and the two
		//      are not measured on the same manoeuvre.
		//  (b) WRONG CHANNEL. Steer reuses tau_linear as the ANGULAR time
		//      constant (the u_turn numerator, the turn slew). A real angular
		//      time constant needs rotational inertia and available torque,
		//      neither of which Envelope carries. Extending Envelope with them
		//      and deriving tau_angular honestly is the fix.
		// Deliberately NOT patched with a floor or a bound: a fabricated clamp
		// would hide the non-physical intermediate rather than remove it (see
		// CLAUDE.md). The consequence is a large-magnitude `stability` for small
		// creatures -- framerate-independent since the u_turn split, but still
		// large.
		e.tau_linear = t.optimal_speed_m_s / t.max_acceleration_m_s2;
		return e;
	}
	case LocomotionMode::AERIAL: {
		if (!analysis.aerial.has_value()) return std::nullopt;
		const auto& a = *analysis.aerial;

		Envelope e;
		e.max_speed = a.max_flight_speed_m_s;
		e.min_speed = a.min_flight_speed_m_s;   // stall at n = 1
		if (!UsableSpeedBand(e.min_speed, e.max_speed)) return std::nullopt;

		// Accelerating power is the MECHANICAL SURPLUS: what the muscles can put
		// out mechanically, minus what level flight at cruise already consumes.
		// available_muscle_power_W is documented as mechanical muscle output
		// (tonton_analysis.h) and tonton_aerial.cpp:333 already uses it that way.
		const float surplus_W = std::max(0.f,
			float(analysis.metabolic.available_muscle_power_W)
			- float(a.flapping_power_mechanical_W));
		e.max_accel = AccelFromPower(power_W{surplus_W},
		                             analysis.physical.body_mass_kg,
		                             a.cruise_speed_m_s);
		e.max_brake = e.max_accel;

		if (!UsableAccel(e.max_accel)) return std::nullopt;

		AerialAuthority auth;
		auth.max_roll_rate  = a.max_roll_rate_rad_s;
		auth.max_pitch_rate = a.max_pitch_rate_rad_s;
		auth.max_yaw_rate   = a.max_yaw_rate_rad_s;
		auth.stall_speed    = a.min_flight_speed_m_s;
		auth.cruise_speed   = a.cruise_speed_m_s;
		auth.n_max = LoadFactorLimit(a, gravity_m_s2);
		e.aerial = auth;

		// a_lat = g*tan(phi_max) = g*sqrt(n_max^2 - 1)
		// TODO(task-5): this is identically 0 when gravity_m_s2 == 0 for ANY
		// n_max -- banking trades weight for centripetal force, so it is
		// undefined without weight. TonTon explicitly supports low/zero-gravity
		// worlds, so a zero-g flyer needs a different lateral model (direct
		// thrust-vectoring / wing side-force), not this one.
		const float lat = gravity_m_s2 * std::sqrt(std::max(0.f, auth.n_max * auth.n_max - 1.f));
		e.max_lateral_accel = acceleration_m_s2{lat};

		// Reconcile the stated radius with the lateral budget we just derived,
		// exactly as the terrestrial arm does via LateralBudget. A radius the
		// load factor forbids is worse than useless: a Task-5 consumer trusting
		// it would command a turn the creature cannot hold.
		e.min_turn_radius = a.min_turning_radius_m;
		if (float(e.max_lateral_accel) > 0.f && std::isfinite(float(e.max_lateral_accel))) {
			const float v_c = float(a.cruise_speed_m_s);
			const float r_from_lat = (v_c * v_c) / float(e.max_lateral_accel);
			if (std::isfinite(r_from_lat) && r_from_lat > float(e.min_turn_radius)) {
				e.min_turn_radius = length_m{r_from_lat};
			}
		}

		e.tau_linear = a.cruise_speed_m_s / e.max_accel;
		return e;
	}
	case LocomotionMode::AQUATIC: {
		if (!analysis.aquatic.has_value()) return std::nullopt;
		const auto& a = *analysis.aquatic;

		Envelope e;
		e.max_speed = a.burst_speed_m_s;
		// requires_constant_motion (sharks, tuna) is just a min_speed inside
		// the mode's own envelope -- not a transition constraint. It is 0 for a
		// creature that can hold station.
		e.min_speed = a.min_swim_speed_m_s;
		if (!UsableSpeedBand(e.min_speed, e.max_speed)) return std::nullopt;

		// NOT the same operation as the aerial arm above, despite the shape.
		//
		// Aerial subtracts a genuine aerodynamic DEMAND: flapping_power_mechanical_W
		// is derived from wing geometry and airspeed without reference to muscle
		// power, so it can and does exceed the supply (184% of it for the bat,
		// which correctly zeroes the surplus and makes the mode absent).
		//
		// Aquatic has no such demand figure. BOTH aquatic power numbers are
		// BUDGET ALLOCATIONS of the same available_muscle_power_W:
		// swim_power_mechanical_W is 0.08 of it and swim_power_burst_mechanical_W
		// is 0.4 of it (tonton_aquatic.cpp). Subtracting the 0.08 cruise budget
		// from the full muscle power would therefore be a fixed 0.92 * supply on
		// every sample forever -- an expression that can never zero, and one that
		// silently assumed 2.3x the power budget the burst SPEED bounding this
		// same envelope was derived from.
		//
		// So: spend out of the burst budget the rules layer itself considers
		// mechanically available, which is the budget max_speed = burst_speed_m_s
		// comes from. The envelope is then internally consistent -- one budget,
		// used for both ends of it.
		const float surplus_W = std::max(0.f,
			float(a.swim_power_burst_mechanical_W)
			- float(a.swim_power_mechanical_W));
		e.max_accel = AccelFromPower(power_W{surplus_W},
		                             analysis.physical.body_mass_kg,
		                             a.cruise_speed_m_s);
		if (!UsableAccel(e.max_accel)) return std::nullopt;

		// A conservative FLOOR on deceleration, not a derivation. Water does
		// brake a swimmer for free, so the true figure is thrust + drag and is
		// strictly larger than this -- but Analysis_Aquatic's drag_coefficient
		// applies to the streamlined cruise attitude, not to a fish flaring its
		// fins to stop, so the honest drag term is not available here. Using
		// max_accel understates braking; it never overstates it.
		e.max_brake = e.max_accel;

		e.min_turn_radius = a.min_turning_radius_m;
		e.max_lateral_accel = LateralBudget(e.max_accel, e.max_speed, e.min_turn_radius);
		e.tau_linear = a.cruise_speed_m_s / e.max_accel;
		return e;
	}

	case LocomotionMode::SERPENTINE: {
		if (!analysis.serpentine.has_value()) return std::nullopt;
		const auto& s = *analysis.serpentine;

		Envelope e;
		// Serpentine has a narrow band: undulation speed is close to both the
		// floor and the ceiling, so the envelope is deliberately tight and
		// stability drops fast when demands exceed it.
		e.max_speed = s.lateral_undulation_speed_m_s;

		// KNOWN GAP, shipped deliberately (adjudicated): the 0.5 factor is a
		// placeholder. Nothing in Analysis_Serpentine states a minimum
		// undulation speed, and a snake that stops undulating simply stops.
		e.min_speed = s.lateral_undulation_speed_m_s * 0.5f;
		if (!UsableSpeedBand(e.min_speed, e.max_speed)) return std::nullopt;

		// KNOWN GAP, shipped deliberately (adjudicated): the 1-second divisor is
		// a placeholder for a MISSING QUANTITY, not a tuning constant.
		// Analysis_Serpentine exposes no acceleration, no power, and -- checked
		// against the header -- no undulation FREQUENCY either. Its
		// Analysis_BodyWave carries wavelength_ratio and amplitude_ratio only;
		// the frequency tonton_serpentine.cpp:186 computes is never exported
		// (Rectilinear::frequency_Hz and SideWinding::frequency_Hz belong to
		// those other gaits and are both optional). With a frequency this would
		// be speed*f, the same honest form the brachiation arm below uses.
		e.max_accel = s.lateral_undulation_speed_m_s / time_s{1.f};
		if (!UsableAccel(e.max_accel)) return std::nullopt;
		e.max_brake = e.max_accel;

		// SENTINEL: 0 means "this mode states no radius constraint", NOT "can
		// turn infinitely tightly". Analysis_Serpentine has no turning-radius
		// field. LateralBudget is the only consumer of min_turn_radius and
		// treats <= 0 by falling through to the acceleration budget, so the
		// sentinel can neither divide by zero nor fabricate an infinite bound.
		e.min_turn_radius = length_m{0.f}; // a snake turns by undulating
		e.max_lateral_accel = LateralBudget(e.max_accel, e.max_speed, e.min_turn_radius);
		e.tau_linear = s.lateral_undulation_speed_m_s / e.max_accel;
		return e;
	}

	case LocomotionMode::CLIMBING: {
		if (!analysis.climbing.has_value()) return std::nullopt;
		const auto& c = *analysis.climbing;

		Envelope e;
		e.max_speed = c.max_climb_speed_m_s;
		e.min_speed = velocity_m_s{0.f};   // a climber can hang motionless
		if (!UsableSpeedBand(e.min_speed, e.max_speed)) return std::nullopt;

		// KNOWN GAP, shipped deliberately (adjudicated): same missing quantity
		// as SERPENTINE above. Analysis_Climbing exposes a speed, an angle and
		// a set of substrate booleans -- no acceleration, power or stride rate.
		e.max_accel = c.max_climb_speed_m_s / time_s{1.f};
		if (!UsableAccel(e.max_accel)) return std::nullopt;
		e.max_brake = e.max_accel;   // a climber stops by gripping

		// SENTINEL, as in the serpentine arm: no radius constraint is stated.
		e.min_turn_radius = length_m{0.f};
		e.max_lateral_accel = LateralBudget(e.max_accel, e.max_speed, e.min_turn_radius);
		e.tau_linear = c.max_climb_speed_m_s / e.max_accel;
		return e;
	}

	case LocomotionMode::BRACHIATION: {
		if (!analysis.brachiation.has_value()) return std::nullopt;
		const auto& b = *analysis.brachiation;

		Envelope e;
		e.max_speed = b.max_swing_speed_m_s;
		e.min_speed = velocity_m_s{0.f};   // a brachiator can hang motionless
		if (!UsableSpeedBand(e.min_speed, e.max_speed)) return std::nullopt;
		// A brachiator changes speed once per swing; the swing frequency is a
		// real measured quantity here, unlike terrestrial stride frequency. So
		// this arm, alone among the three non-fluid ones, needs no placeholder.
		e.max_accel = b.max_swing_speed_m_s * b.swing_frequency_Hz;
		if (!UsableAccel(e.max_accel)) return std::nullopt;
		e.max_brake = e.max_accel;

		// SENTINEL, as in the serpentine arm: no radius constraint is stated.
		e.min_turn_radius = length_m{0.f};
		e.max_lateral_accel = LateralBudget(e.max_accel, e.max_speed, e.min_turn_radius);
		e.tau_linear = b.max_swing_speed_m_s / e.max_accel;
		return e;
	}
	}
	// No `default:` above, deliberately: adding a LocomotionMode must be a
	// compile error here rather than a silent nullopt. That is enforced by
	// -Werror=switch on the `tonton` target (CMakeLists.txt) -- the omitted
	// `default:` alone does NOT make it true, since the project builds with no
	// -Wall and no -Werror and would otherwise fall straight through to the
	// return below with zero diagnostics.
	//
	// This line is reached only via a cast from outside the enum's value set.
	return std::nullopt;
}

} // namespace TonTon
