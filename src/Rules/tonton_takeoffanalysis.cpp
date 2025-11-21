#include "tonton_takeoffanalysis.h"
#include "tonton_scratch.h"
#include "../include/tonton_output.h"
#include "../include/tonton_input.h"

namespace TonTon {

using TakeoffAnalysis = Output_TakeoffAnalysis;

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

    const auto& aerial = output.aerial.value();
    const float body_mass_kg = output.physical.body_mass_kg;
    const float weight_N = body_mass_kg * in.environment.gravity_m_s2;
    const float air_density = in.environment.fluidDensity_Kg_m3;
    
    // ========================================================================
    // FORCE ANALYSIS
    // ========================================================================
    
    result.max_instantaneous_lift_N = T::EstimateMaxLift(aerial, body_mass_kg, air_density);
    result.max_instantaneous_thrust_N = T::EstimateMaxThrust(aerial, body_mass_kg, air_density);
    
    // Net vertical force (thrust contributes ~70% vertically during takeoff)
    result.net_vertical_force_N = result.max_instantaneous_lift_N + 
                                  result.max_instantaneous_thrust_N * 0.7f;
    
    // Apply ground effect bonus (assume starting at 0.5m height)
    result.ground_effect_bonus = T::GroundEffectBonus(aerial.wing_span_m, 0.5f);
    result.net_vertical_force_N *= result.ground_effect_bonus;
    
    result.vertical_acceleration_m_s2 = (result.net_vertical_force_N - weight_N) / body_mass_kg;
    result.force_margin_percent = ((result.net_vertical_force_N / weight_N) - 1.0f) * 100.0f;
    
    // ========================================================================
    // POWER ANALYSIS
    // ========================================================================
    
    result.power_to_weight_W_kg = output.metabolic.available_muscle_power_W / body_mass_kg;
    
    // Estimate power needed for takeoff (hovering-like initially)
    float takeoff_power_needed_W = aerial.hovering_cost_W_per_N * weight_N;
    result.takeoff_power_fraction = takeoff_power_needed_W / output.metabolic.available_muscle_power_W;
    
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
        
        float estimated_jump_height_m = body_mass_kg * 0.1f;  // Rough: 10% body mass -> 0.1m
        float estimated_jump_velocity = std::sqrt(2.0f * in.environment.gravity_m_s2 * estimated_jump_height_m);
        result.constraints.leg_strength_ok = estimated_jump_velocity >= result.required_jump_velocity_m_s;
    }
    
    // ========================================================================
    // RUNNING TAKEOFF ANALYSIS
    // ========================================================================
    
    if (output.terrestrial.has_value()) {
        result.takeoff_run_distance_m = T::RunwayDistance(aerial, &output.terrestrial.value(), body_mass_kg);
        result.can_use_water_taxi = output.aquatic.has_value();  // Waterfowl can paddle-run
    }
    
    // ========================================================================
    // TIME TO FLIGHT
    // ========================================================================
    
    if (result.vertical_acceleration_m_s2 > 0) {
        // Time to reach min flight speed vertically
        result.time_to_flight_speed_s = aerial.min_flight_speed_m_s / 
                                       std::max(result.vertical_acceleration_m_s2, 0.1f);
    } else {
        result.time_to_flight_speed_s = -1;  // Cannot accelerate vertically
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


inline float TakeoffAnalysis::EstimateMaxLift(const Output_Aerial& aerial,
                                              float ,
                                              float air_density) {
    if (aerial.wings.empty()) return 0.0f;
    
    // Peak CL during power stroke (downstroke) can reach 1.5-2.5
    // Use conservative 2.0 for biological wings
    const float max_CL_power_stroke = 2.0f;
    
    // Wing tip velocity from flapping
    float tip_velocity = 0;
    for (const auto& wing : aerial.wings) {
        tip_velocity += wing.wing_tip_velocity(aerial.wingbeat_frequency_Hz);
    }
    tip_velocity /= aerial.wings.size();  // Average
    
    // Dynamic pressure from tip velocity
    float dynamic_pressure = 0.5f * air_density * tip_velocity * tip_velocity;
    
    // Total lift from all wings
    float max_lift_N = max_CL_power_stroke * dynamic_pressure * aerial.wing_area_m2;
    
    return max_lift_N;
}


inline float TakeoffAnalysis::EstimateMaxThrust(const Output_Aerial& aerial,
                                               float ,
                                               float air_density) {
    if (aerial.wings.empty()) return 0.0f;
    
    // Thrust from momentum theory: T = mass_flow * velocity_change
    
    // Wing disk area (swept by wings)
    float disk_area = M_PI * aerial.wing_span_m * aerial.wing_span_m;
    
    // Average wing tip velocity
    float tip_velocity = 0;
    for (const auto& wing : aerial.wings) {
        tip_velocity += wing.wing_tip_velocity(aerial.wingbeat_frequency_Hz);
    }
    tip_velocity /= aerial.wings.size();
    
    // Mass flow rate through disk per stroke
 //   float stroke_period_s = 1.0f / (2.0f * aerial.wingbeat_frequency_Hz);  // Half cycle
    float mass_flow_rate = air_density * disk_area * tip_velocity;
    
    // Downwash velocity (simplified: ~50% of tip velocity)
    float downwash_velocity = tip_velocity * 0.5f;
    
    // Thrust = mass flow * velocity change
    float thrust_N = mass_flow_rate * downwash_velocity;
    
    return thrust_N;
}


inline float TakeoffAnalysis::GroundEffectBonus(float wing_span_m, float height_m) {
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


inline float TakeoffAnalysis::RequiredJumpVelocity(const Output_Aerial& aerial,
                                                   float body_mass_kg,
                                                   float max_lift_N,
                                                   float gravity_m_s2) {
    const float weight_N = body_mass_kg * gravity_m_s2;

    // If wings can already provide > weight, no jump needed
    if (max_lift_N >= weight_N * 1.1f) {
        return 0.0f;
    }

    // Otherwise, need to gain enough height/time for wings to build lift
    // Target: reach min flight speed or buy enough time for wing acceleration

 //   float lift_deficit_N = weight_N - max_lift_N;

    // Time needed to reach min flight speed (assuming wings can eventually support)
    float time_needed_s = std::max(0.3f, aerial.min_flight_speed_m_s / 5.0f);

    // Height needed = 0.5 * g * t²
    float height_needed_m = 0.5f * gravity_m_s2 * time_needed_s * time_needed_s;

    // Jump velocity: v = sqrt(2 * g * h)
    float required_v_m_s = std::sqrt(2.0f * gravity_m_s2 * height_needed_m);

    // Clamp to reasonable range
    return std::min(required_v_m_s, 5.0f);  // Max 5 m/s jump (very strong)
}


inline float TakeoffAnalysis::RunwayDistance(const Output_Aerial& aerial,
                                            const Output_Terrestrial* terrestrial,
                                            float body_mass_kg) {
    if (!terrestrial) return -1.0f;  // Cannot run
    
//    const float weight_N = body_mass_kg * 9.81f;
    const float target_speed_m_s = aerial.min_flight_speed_m_s;
    
    // Acceleration from legs + wings
    float leg_acceleration = 0;
    if (terrestrial->max_acceleration_m_s2 > 0) {
        leg_acceleration = terrestrial->max_acceleration_m_s2 * 0.7f;  // 70% while flapping
    } else if (terrestrial->max_sprint_speed_m_s > 0) {
        // Estimate from sprint speed
        leg_acceleration = terrestrial->max_sprint_speed_m_s / 2.0f;
    } else {
        leg_acceleration = 2.0f;  // Default modest acceleration
    }
    
    // Wing thrust contribution (horizontal component)
    float wing_thrust_N = EstimateMaxThrust(aerial, body_mass_kg, 1.225f);
    float wing_acceleration = (wing_thrust_N * 0.3f) / body_mass_kg;  // 30% horizontal
    
    float total_acceleration = leg_acceleration + wing_acceleration;
    
    // Distance = v² / (2a)
    float distance_m = (target_speed_m_s * target_speed_m_s) / (2.0f * total_acceleration);
    
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
