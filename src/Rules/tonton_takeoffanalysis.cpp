#include "tonton_takeoffanalysis.h"
#include "tonton_scratch.h"
#include "../include/tonton_analysis.h"
#include "../include/tonton_input.h"
#include "tonton_builder.h"

namespace TonTon {

using TakeoffAnalysis = Analysis_TakeoffAnalysis;

static TakeoffAnalysis::TakeoffMode ClassifyMode(const Scratch& output,
                                                 const TakeoffAnalysis& analysis);
                                                 
TakeoffAnalysis TakeoffAnalysis_Compute(Input const& in, const Scratch& output) {
    TakeoffAnalysis result;
    using TakeoffMode = TakeoffAnalysis::TakeoffMode;
    using T = TakeoffAnalysis;

    if (!output.aerial.has_value()) {
        result.mode = TakeoffMode::IMPOSSIBLE;
        result.confidence = 1.0f;
        return result;
    }

    // K4: zero/negative mass is degenerate (divides weight, power-to-weight, accel below).
    if (output.physical.body_mass_kg <= 0) {
        result.mode = TakeoffMode::IMPOSSIBLE;
        result.confidence = 0.0f;
        return result;
    }

    const auto& aerial = output.aerial.value();
    const auto body_mass_kg = output.physical.body_mass_kg;
    const auto weight_N = body_mass_kg * in.environment.gravity_m_s2;
    const auto air_density = in.environment.fluidDensity_Kg_m3;
    
    // ========================================================================
    // FORCE ANALYSIS
    // ========================================================================
    
    result.max_instantaneous_lift_N = T::EstimateMaxLift(aerial, body_mass_kg, air_density);
    result.max_instantaneous_thrust_N = T::EstimateMaxThrust(aerial, body_mass_kg, air_density);

    // Split the vertical force into a lift component and a jet/reaction-thrust component
    // (thrust contributes ~70% vertically during takeoff).
    force_N lift_component_N = result.max_instantaneous_lift_N;
    force_N thrust_component_N = result.max_instantaneous_thrust_N * 0.7f;

    // Apply ground effect bonus (height = max leg length, or 0.5m fallback)
    length_m ground_height_m = 0.5f;
    if (output.terrestrial.has_value()) {
        for (const auto& leg : output.terrestrial->legs) {
            ground_height_m = std::max(ground_height_m, leg.rest_length_m);
        }
    }
    result.ground_effect_bonus = T::GroundEffectBonus(aerial.wing_span_m, ground_height_m);
    // K3: ground effect augments LIFT only (image-vortex effect reduces induced drag);
    // it does not amplify the jet/reaction thrust term.
    lift_component_N *= result.ground_effect_bonus;
    result.net_vertical_force_N = lift_component_N + thrust_component_N;
    
    result.vertical_acceleration_m_s2 = (result.net_vertical_force_N - weight_N) / body_mass_kg;
    result.force_margin_percent = ((result.net_vertical_force_N / weight_N) - 1.0f) * 100.0f;
   
    // ========================================================================
    // POWER ANALYSIS
    // ========================================================================
    
    result.power_to_weight_W_kg = output.metabolic.available_muscle_power_W / body_mass_kg;
   
    // Estimate power needed for takeoff (hovering-like initially).
    // K1: compare mechanical-to-mechanical. Per tonton_aerial.cpp, hovering_power_W is
    // METABOLIC (mechanical / efficiency), while metabolic.available_muscle_power_W is the
    // MECHANICAL muscle output capacity (~200-400 W/kg, see tonton_metabolic.cpp). The old
    // code divided metabolic by mechanical (off by ~1/efficiency). Convert the metabolic
    // hovering requirement back to mechanical (efficiency) before forming the fraction.
    // NOTE: keep roughly in sync with the muscle efficiency tonton_aerial.cpp uses to
    // convert hover mechanical->metabolic (currently ~0.10-0.23); a mismatch skews the round-trip.
    const float muscle_efficiency = 0.20f;  // representative flapping-flight value (range 0.10-0.23)
    power_W takeoff_power_needed_W = aerial.hovering_power_W * muscle_efficiency;  // -> mechanical
    result.takeoff_power_fraction = float(takeoff_power_needed_W) /
                                    std::max(1e-3f, float(output.metabolic.available_muscle_power_W));
    
    // ========================================================================
    // CONSTRAINT CHECKS
    // ========================================================================
    
    result.constraints.wing_loading_ok = aerial.wing_loading_N_m2 < 80.0f;
    result.constraints.power_loading_ok = result.power_to_weight_W_kg > 50.0f;
    
    // Aspect ratio check: high AR (>12) is bad for flapping takeoff
    float avg_aspect_ratio = 0;
    if (!aerial.wings.empty()) {
        for (const auto& wing : aerial.wings) {
            avg_aspect_ratio += wing.aspect_ratio();
        }
        avg_aspect_ratio /= aerial.wings.size();
    }
    result.constraints.aspect_ratio_ok = avg_aspect_ratio < 15.0f;
    
    // Check if legs can provide jump assist
    if (output.jumping.has_value()) {
        result.required_jump_velocity_m_s = T::RequiredJumpVelocity(aerial, body_mass_kg,
                                                                 result.max_instantaneous_lift_N,
                                                                 in.environment.gravity_m_s2);
        result.constraints.leg_strength_ok = output.jumping->takeoff_velocity_m_s >=
                                            result.required_jump_velocity_m_s;
    } else if (output.terrestrial.has_value()) {
        // Estimate jump capability from leg strength
        result.required_jump_velocity_m_s = T::RequiredJumpVelocity(aerial, body_mass_kg,
                                                                 result.max_instantaneous_lift_N,
                                                                 in.environment.gravity_m_s2);
        
        // Estimate jump height from leg stroke distance (stretched - crouch length)
        // Work = F×d ≈ m×g×h when force ≈ body weight over the stroke
        length_m estimated_jump_height_m = 0;
        for (const auto& leg : output.terrestrial->legs) {
            auto crouch = scale_to<0>(in.builder->appendages[leg.id].crouch_length, in.scale);
            auto crouch_m = (crouch > 0) ? crouch : leg.rest_length_m * 0.3f;
            auto stroke = leg.stretched_length_m - crouch_m;
            estimated_jump_height_m = std::max(estimated_jump_height_m, stroke);
        }
        velocity_m_s estimated_jump_velocity = sqrt(2.0f * in.environment.gravity_m_s2 * estimated_jump_height_m);
        result.constraints.leg_strength_ok = estimated_jump_velocity >= result.required_jump_velocity_m_s;
    }
    
    // ========================================================================
    // RUNNING TAKEOFF ANALYSIS
    // ========================================================================
    
    if (output.terrestrial.has_value()) {
        result.takeoff_run_distance_m = T::RunwayDistance(aerial, &output.terrestrial.value(), body_mass_kg, in.environment.fluidDensity_Kg_m3);
        result.can_use_water_taxi = output.aquatic.has_value();  // Waterfowl can paddle-run
    }
    
    // ========================================================================
    // TIME TO FLIGHT
    // ========================================================================
    
    if (result.vertical_acceleration_m_s2 > 0) {
        // Time to reach min flight speed vertically
        result.time_to_flight_speed_s = aerial.min_flight_speed_m_s / 
                                       std::max(result.vertical_acceleration_m_s2, acceleration_m_s2(0.1f));
    } else {
        result.time_to_flight_speed_s = time_s(-1);  // Cannot accelerate vertically
    }
    
    // ========================================================================
    // ALTERNATIVE STRATEGIES
    // ========================================================================
    
    // High aspect ratio fliers benefit from wind
    result.can_use_wind_assist = avg_aspect_ratio > 10.0f;
    
    // Must drop if cannot generate vertical force
    result.requires_perch_drop = result.net_vertical_force_N < weight_N * 1.05f;
    
    // ========================================================================
    // CLASSIFY TAKEOFF MODE
    // ========================================================================
    
    result.mode = ClassifyMode(output, result);
    
    // ========================================================================
    // CONFIDENCE ESTIMATION
    // ========================================================================
    
    result.confidence = 0.7f;  // Base confidence
    if (aerial.wings.empty()) result.confidence *= 0.5f;
    if (aerial.wingbeat_frequency_Hz <= 0) result.confidence *= 0.7f;
    if (output.physical.body_mass_kg <= 0) result.confidence *= 0.5f;
    
    return result;
}


inline force_N TakeoffAnalysis::EstimateMaxLift(const Analysis_Aerial& aerial,
                                              mass_kg ,
                                              density_kg_m3 air_density) {
    if (aerial.wings.empty()) return force_N(0.0f);
    
    // Peak CL during power stroke (downstroke) can reach 1.5-2.5
    // Use conservative 2.0 for biological wings
    const float max_CL_power_stroke = 2.0f;
    
    // Wing tip velocity from flapping
    velocity_m_s tip_velocity;
    for (const auto& wing : aerial.wings) {
        tip_velocity += wing.wing_tip_velocity(aerial.wingbeat_frequency_Hz);
    }
    tip_velocity /= aerial.wings.size();  // Average
    
    // Dynamic pressure from tip velocity.
    // K2: velocity grows linearly to the tip, so the mean of v^2 over the wing span
    // = v_tip^2 / 3 (blade-element). Using tip v^2 over the whole area overestimates ~3x.
    pressure_Pa dynamic_pressure = 0.5f * air_density * (tip_velocity * tip_velocity) / 3.0f;
    
    // Total lift from all wings
    force_N max_lift_N = max_CL_power_stroke * dynamic_pressure * aerial.wing_area_m2;
    
    return max_lift_N;
}

inline force_N TakeoffAnalysis::EstimateMaxThrust(const Analysis_Aerial& aerial,
                                               mass_kg ,
                                               density_kg_m3 air_density) {
    if (aerial.wings.empty()) return force_N(0.0f);
    
    // Thrust from momentum theory: T = mass_flow * velocity_change
    
    // Wing disk area (swept by wings)
    auto disk_area = M_PI * aerial.wing_span_m * aerial.wing_span_m / 4.0f;
    
    // Average wing tip velocity
    velocity_m_s tip_velocity;
    for (const auto& wing : aerial.wings) {
        tip_velocity += wing.wing_tip_velocity(aerial.wingbeat_frequency_Hz);
    }
    tip_velocity /= aerial.wings.size();
    
    // Mass flow rate through disk per stroke
 //   float stroke_period_s = 1.0f / (2.0f * aerial.wingbeat_frequency_Hz);  // Half cycle
    auto mass_flow_rate = air_density * (disk_area * tip_velocity);
    
    // Downwash velocity (simplified: ~50% of tip velocity)
    velocity_m_s downwash_velocity = tip_velocity * 0.5f;
    
    // Thrust = mass flow * velocity change
    force_N thrust_N = mass_flow_rate * downwash_velocity;
    
    return thrust_N;
}

inline float TakeoffAnalysis::GroundEffectBonus(length_m wing_span_m, length_m height_m) {
    // Ground effect is significant within 1 wingspan of surface
    // Reduces induced drag and increases effective lift
    
    if (height_m >= wing_span_m) {
        return 1.0f;  // No ground effect
    }
    
    float height_ratio = height_m / wing_span_m;
    
    // Lift increase: 5-30% depending on height
    // Use empirical formula: bonus = 1 + 0.25 * (1 - h/b)
    float lift_bonus = 1.0f + 0.25f * (1.0f - height_ratio);
    
    return lift_bonus;
}


inline velocity_m_s TakeoffAnalysis::RequiredJumpVelocity(const Analysis_Aerial& aerial,
                                                   mass_kg body_mass_kg,
                                                   force_N max_lift_N,
                                                   acceleration_m_s2 gravity_m_s2) {
    const force_N weight_N = body_mass_kg * gravity_m_s2;

    // If wings can already provide > weight, no jump needed
    if (max_lift_N >= weight_N * 1.1f) {
        return velocity_m_s(0.0f);
    }

    // Otherwise, need to gain enough height/time for wings to build lift
    // Target: reach min flight speed or buy enough time for wing acceleration

    // K4: guard wings[0] and the 1/f used below; without wings/beat there is no
    // wing-driven forward accel, so no meaningful jump-assist velocity is defined.
    if (aerial.wings.empty() || aerial.wingbeat_frequency_Hz <= 0) return velocity_m_s(0.0f);
    velocity_m_s wing_tip_speed = aerial.wings[0].wing_tip_velocity(aerial.wingbeat_frequency_Hz);

    //   float lift_deficit_N = weight_N - max_lift_N;

    // Forward thrust ≈ some fraction of wing tip momentum per beat
    // Very rough estimate: 10-20% of tip speed translates to forward acceleration per wingbeat period
    time_s wingbeat_period = 1.0f / aerial.wingbeat_frequency_Hz;
    acceleration_m_s2 forward_accel = (wing_tip_speed * 0.15f) / wingbeat_period;

    time_s time_needed_s = std::max(time_s(0.3f), aerial.min_flight_speed_m_s / forward_accel);

    // Height needed = 0.5 * g * t²
    length_m height_needed_m = 0.5f * gravity_m_s2 * time_needed_s * time_needed_s;

    // Jump velocity: v = sqrt(2 * g * h)
    velocity_m_s required_v_m_s = sqrt(2.0f * gravity_m_s2 * height_needed_m);

    // Clamp to reasonable range
    return std::min(required_v_m_s, velocity_m_s(5.0f));  // Max 5 m/s jump (very strong)
}


inline length_m TakeoffAnalysis::RunwayDistance(const Analysis_Aerial& aerial,
                                            const Analysis_Terrestrial* terrestrial,
                                            mass_kg body_mass_kg,
                                            density_kg_m3 air_density) {
    if (!terrestrial) return length_m(-1.0f);  // Cannot run
    
//    const float weight_N = body_mass_kg * 9.81f;
    const velocity_m_s target_speed_m_s = aerial.min_flight_speed_m_s;
   
    // Acceleration from legs + wings
    acceleration_m_s2 leg_acceleration;
    if (terrestrial->max_acceleration_m_s2 > 0) {
        leg_acceleration = terrestrial->max_acceleration_m_s2 * 0.7f;  // 70% while flapping
    } else if (terrestrial->max_sprint_speed_m_s > 0) {
        // Estimate from sprint speed
        leg_acceleration = terrestrial->max_sprint_speed_m_s / time_s(2.0f);
    } else {
        leg_acceleration = acceleration_m_s2(2.0f);  // Default modest acceleration
    }
    
    // Wing thrust contribution (horizontal component)
    force_N wing_thrust_N = EstimateMaxThrust(aerial, body_mass_kg, air_density);
    acceleration_m_s2 wing_acceleration = (wing_thrust_N * 0.3f) / body_mass_kg;  // 30% horizontal
    
    acceleration_m_s2 total_acceleration = leg_acceleration + wing_acceleration;
    
    // Distance = v² / (2a)
    length_m distance_m = (target_speed_m_s * target_speed_m_s) / (2.0f * total_acceleration);
    
    return distance_m;
}


static TakeoffAnalysis::TakeoffMode ClassifyMode(const Scratch& output,
                                                 const TakeoffAnalysis& analysis) {
    using TakeoffMode = TakeoffAnalysis::TakeoffMode;
    
    const auto& aerial = output.aerial.value();
    
    // ========================================================================
    // VERTICAL LAUNCH: High power, low wing loading
    // ========================================================================
    if (analysis.power_to_weight_W_kg > 150.0f && 
        aerial.wing_loading_N_m2 < 25.0f &&
        analysis.force_margin_percent > 20.0f) {
        return TakeoffMode::VERTICAL_LAUNCH;
    }
    
    // ========================================================================
    // JUMP LAUNCH: Good power, wings need assist
    // ========================================================================
    if (analysis.power_to_weight_W_kg > 80.0f &&
        aerial.wing_loading_N_m2 < 50.0f &&
        analysis.constraints.leg_strength_ok &&
        analysis.force_margin_percent > 0.0f) {
        return TakeoffMode::JUMP_LAUNCH;
    }
    
    // ========================================================================
    // RUNNING TAKEOFF: Need runway to build speed
    // ========================================================================
    if (output.terrestrial.has_value() &&
        aerial.wing_loading_N_m2 < 80.0f &&
        analysis.takeoff_run_distance_m > 0 &&
        analysis.takeoff_run_distance_m < 50.0f) {  // Reasonable runway
        return TakeoffMode::RUNNING_TAKEOFF;
    }
    
    // ========================================================================
    // ASSISTED LAUNCH: Drop from perch
    // ========================================================================
    if (analysis.requires_perch_drop &&
        aerial.max_flight_speed_m_s > aerial.min_flight_speed_m_s * 1.5f) {
        return TakeoffMode::ASSISTED_LAUNCH;
    }
    
    // ========================================================================
    // CLIFF LAUNCH: Must gain speed by dropping
    // ========================================================================
    if (aerial.wing_loading_N_m2 > 80.0f &&
        analysis.force_margin_percent < 0 &&
        aerial.max_flight_speed_m_s > aerial.min_flight_speed_m_s * 2.0f) {
        return TakeoffMode::CLIFF_LAUNCH;
    }
    
    // ========================================================================
    // DEFAULT: IMPOSSIBLE
    // ========================================================================
    return TakeoffMode::IMPOSSIBLE;
}

}
