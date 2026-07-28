#include "Control/tonton_envelope.h"
#include "tonton_analysis.h"
#include <algorithm>
#include <cmath>

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
acceleration_m_s2 AccelFromPower(power_W available, mass_kg mass, velocity_m_s at_speed)
{
	if (float(mass) <= 0.f || float(at_speed) <= 0.f) return acceleration_m_s2{0.f};
	return available / (mass * at_speed);
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
float LoadFactorLimit(const Analysis_Aerial& a, mass_kg, float gravity_m_s2)
{
	float n_aero = 1.f;
	const float v_stall = float(a.min_flight_speed_m_s);
	if (v_stall > 0.f && float(a.cruise_speed_m_s) > 0.f) {
		const float ratio = float(a.cruise_speed_m_s) / v_stall;
		n_aero = ratio * ratio;
	}

	float n_radius = 1.f;
	const float r = float(a.min_turning_radius_m);
	const float v = float(a.cruise_speed_m_s);
	if (r > 0.f && v > 0.f && gravity_m_s2 > 0.f) {
		const float tan_phi = (v * v) / (gravity_m_s2 * r);
		n_radius = std::sqrt(1.f + tan_phi * tan_phi); // = 1/cos(atan(tan_phi))
	}

	const float n = std::min(n_aero, n_radius);
	return (n >= 1.f && std::isfinite(n)) ? n : 1.f;
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
		e.max_accel       = t.max_acceleration_m_s2;
		e.max_brake       = t.max_acceleration_m_s2; // legs brake with the same grip budget
		e.min_turn_radius = t.min_turning_radius_m;
		e.max_lateral_accel = LateralBudget(e.max_accel, e.max_speed, e.min_turn_radius);

		// tau = characteristic speed / acceleration. Dimensionally a time, built
		// from two numbers the plausibility suite already validates, adding no
		// new biological claim. See the spec's rejected-pendulum note.
		e.tau_linear = t.optimal_speed_m_s / t.max_acceleration_m_s2;
		return e;
	}
	case LocomotionMode::AERIAL: {
		if (!analysis.aerial.has_value()) return std::nullopt;
		const auto& a = *analysis.aerial;

		Envelope e;
		e.max_speed = a.max_flight_speed_m_s;
		e.min_speed = a.min_flight_speed_m_s;   // stall at n = 1
		e.max_accel = AccelFromPower(a.flapping_power_W,
		                             analysis.physical.body_mass_kg,
		                             a.cruise_speed_m_s);
		e.max_brake = e.max_accel;
		e.min_turn_radius = a.min_turning_radius_m;

		AerialAuthority auth;
		auth.max_roll_rate  = a.max_roll_rate_rad_s;
		auth.max_pitch_rate = a.max_pitch_rate_rad_s;
		auth.max_yaw_rate   = a.max_yaw_rate_rad_s;
		auth.stall_speed    = a.min_flight_speed_m_s;
		auth.cruise_speed   = a.cruise_speed_m_s;
		auth.n_max = LoadFactorLimit(a, analysis.physical.body_mass_kg, gravity_m_s2);
		e.aerial = auth;

		// a_lat = g*tan(phi_max) = g*sqrt(n_max^2 - 1)
		const float lat = gravity_m_s2 * std::sqrt(std::max(0.f, auth.n_max * auth.n_max - 1.f));
		e.max_lateral_accel = acceleration_m_s2{lat};

		e.tau_linear = a.cruise_speed_m_s / e.max_accel;
		return e;
	}
	default:
		return std::nullopt; // later tasks fill in the remaining modes
	}
}

} // namespace TonTon
