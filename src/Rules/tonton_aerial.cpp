#include "tonton_aerial.h"
#include "Memos/tonton_armaturememo.h"
#include "Memos/tonton_skinnedmeshmemo.h"
#include "Rules/tonton_scratch.h"
#include "dodeedum.h"
#include "../../include/tonton_input.h"
#include "../../include/tonton_output.h"
#include <iostream>
#include <set>
#include <cmath>
#include <algorithm>


struct GaitGroupSpan
{
	uint16_t gaitGroup;
	uint16_t count;
	float total_span_m;
};

namespace TonTon
{
static std::vector<Output_Aerial::Wing> GetWings(Input const& in, Scratch &out, bool & has_feathers);
std::vector<GaitGroupSpan> GetGaitGroupSpan(Output_Aerial::Wing * data, size_t size);
}

std::optional<TonTon::Output_Aerial> TonTon::ComputeAerial(Input const& in, Scratch &out)
{
    TonTon::Output_Aerial r;

	bool has_feathers = false;
    auto wings = shared_array<TonTon::Output_Aerial::Wing>::FromArray(GetWings(in, out, has_feathers));
    r.wings = wings;

    if (r.wings.empty()) {
        return {}; // No wings, no aerial capability
    }

    // ============================================================================
    // AGGREGATE WING PARAMETERS
    // ============================================================================

    struct WingAggregates {
        float total_area_m2;
        float total_span_m;  // Normalized to bilateral span
        float avg_inertia_kg_m2;
        float mean_chord_m;
        int num_groups;
    };

    auto compute_wing_aggregates = [&]() -> WingAggregates {
        float total_area = 0.0f;
        float total_span = 0.0f;
        float total_inertia = 0.0f;
        float total_chord = 0.0f;

        for (const auto& wing : r.wings) {
            total_area += wing.area_m2;
            total_span += wing.span_m;
            total_inertia += wing.inertia_kgm2;
            total_chord += wing.chord_m;
        }

        // Normalize span: for 4 wings or 8 wings, still get bilateral equivalent
        total_span = total_span * 2.0f / r.wings.size();

        // Count unique gait groups
        std::set<int32_t> gait_groups;
        for (const auto& wing : r.wings) {
            gait_groups.insert(wing.gait_group);
        }

        return {
            total_area,
            total_span,
            total_inertia / r.wings.size(),
            total_chord / r.wings.size(),
            static_cast<int>(gait_groups.size())
        };
    };

    auto wing_params = compute_wing_aggregates();
    r.wing_span_m = wing_params.total_span_m;
    r.wing_area_m2 = wing_params.total_area_m2;

    // ============================================================================
    // PHYSICAL CONSTANTS & BUOYANCY
    // ============================================================================

    const float g = in.environment.gravity_m_s2;
    const float rho = in.environment.fluidDensity_Kg_m3;
    const float mu = in.environment.fluidViscosity_Pa_s;
    const float body_mass_kg = out.physical.body_mass_kg;
    const float body_volume_m3 = out.physical.body_volume_m3;

    auto compute_effective_weight = [&]() -> float {
        // Account for buoyancy in dense fluids (critical for underwater "flight")
        // Effective weight = (body_mass - displaced_fluid_mass) * g
        const float displaced_fluid_mass_kg = body_volume_m3 * rho;
        float effective_weight_N = (body_mass_kg - displaced_fluid_mass_kg) * g;

        // Min weight: 1% of air weight to avoid singularities in flight equations
        const float min_weight_N = body_mass_kg * g * 0.01f;
        return std::max(effective_weight_N, min_weight_N);
    };

    const float weight_N = compute_effective_weight();

    // ============================================================================
    // WING LOADING & STALL SPEED
    // ============================================================================

    r.wing_loading_N_m2 = weight_N / wing_params.total_area_m2;

    auto compute_stall_speed = [&]() -> float {
        // From lift equation: L = 0.5 * rho * v² * S * CL
        // Solving for v at L = Weight and CL = CL_max
        float CL_max = 1.2f + 0.6f * in.feather_quality;
        return std::sqrt((2.0f * weight_N) / (rho * wing_params.total_area_m2 * CL_max));
    };

    r.min_flight_speed_m_s = compute_stall_speed();

    // ============================================================================
    // WINGBEAT KINEMATICS
    // ============================================================================

    struct WingbeatParams {
        float frequency_Hz;
        float cruise_amplitude_rad;
        float hover_amplitude_rad;
        float cruise_stroke_length_m;
        float hover_stroke_length_m;
        float available_power_W;
    };

    auto compute_wingbeat_params = [&]() -> WingbeatParams {
        // Pennycuick (1996): f ∝ sqrt(g/L) * (wing_loading)^(3/8)
        float K = 3.87f * (0.8f + 0.4f * in.scaling_strategy);
        float base_frequency_Hz = K * std::sqrt(g / wing_params.total_span_m) *
                                  std::pow(r.wing_loading_N_m2, 0.375f) ;

        // SEPARATE amplitudes for cruise vs hover flight modes
        // stability_vs_speed represents evolutionary optimization (swift vs hummingbird)
        // NOT which mode is currently being analyzed

        // Cruise amplitude: 50° (speed-optimized) to 75° (stability-optimized)
        float cruise_amplitude_rad = glm::mix(0.87f, 1.31f, in.stability_vs_speed);

        // Hover amplitude: 90° (speed specialist, minimal hover) to 180° (hummingbird, full stroke)
        float hover_amplitude_rad = glm::mix(1.57f, 3.14f, in.stability_vs_speed);

        // Muscle power budget
        float muscle_mass_fraction = 0.15f + 0.20f * in.stability_vs_speed;
        float muscle_mass_kg = body_mass_kg * muscle_mass_fraction;
        float power_density_W_kg = 200.0f + 200.0f * in.muscle_quality;
        float available_power_W = muscle_mass_kg * power_density_W_kg * in.metabolic_efficiency;

        // Frequency limits (use hover amplitude as it's more demanding)
        float power_limited_freq_Hz = std::sqrt(available_power_W /
            (wing_params.avg_inertia_kg_m2 * hover_amplitude_rad * hover_amplitude_rad)) / (2.0f * M_PI);
        // neural limit is considered as the CPG not the brain.
        float neural_limit_Hz = 200.0f;

        float frequency_Hz = std::min({base_frequency_Hz, power_limited_freq_Hz, neural_limit_Hz});

        // Stroke lengths from angular amplitudes
        float wing_radius_m = wing_params.total_span_m / 2.0f;
        float cruise_stroke_length_m = cruise_amplitude_rad * wing_radius_m;
        float hover_stroke_length_m =  hover_amplitude_rad * wing_radius_m;

        return {
			.frequency_Hz=frequency_Hz,
			.cruise_amplitude_rad=cruise_amplitude_rad,
			.hover_amplitude_rad=hover_amplitude_rad,
			.cruise_stroke_length_m=cruise_stroke_length_m,
			.hover_stroke_length_m=hover_stroke_length_m,
			.available_power_W=available_power_W
		};
    };

    auto wingbeat = compute_wingbeat_params();
    r.wingbeat_frequency_Hz = wingbeat.frequency_Hz;

    // Apply wingbeat parameters to individual wings
    auto configure_wing_kinematics = [&]() {
        for (auto& wing : wings) {
            // Use hover amplitude for wing kinematics (used in hover power calculation)
            wing.beat_amplitude_rad = wingbeat.hover_amplitude_rad;
            // Hovering: horizontal, forward flight: angled back 45-60°
            wing.stroke_plane_angle_rad = (1.0f - in.stability_vs_speed) * (M_PI / 4.0f);
        }

        // Dragonfly mode: 4 wings in 2 gait groups
        if (wing_params.num_groups == 2 && r.wings.size() == 4) {
            for (auto& wing : wings) {
                if (wing.gait_group == 0) {
                    // Front wings - more horizontal
                    wing.stroke_plane_angle_rad = 0.1f + (1.0f - in.stability_vs_speed) * 0.6f;
                    wing.phase_offset = 0.0f;
                } else {
                    // Hind wings - steeper, counter-stroking for hovering
                    wing.stroke_plane_angle_rad = 0.3f + (1.0f - in.stability_vs_speed) * 0.5f;
                    wing.phase_offset = in.stability_vs_speed * M_PI;
                }
            }
        }
    };

    configure_wing_kinematics();

    // ============================================================================
    // FLIGHT SPEEDS (Strouhal constraint)
    // ============================================================================

    auto compute_flight_speeds = [&]() -> std::pair<float, float> {
        // Strouhal number: St = f·A/v ≈ 0.2-0.4 for efficient oscillatory locomotion
        // Use CRUISE amplitude for forward flight speed calculations
        float strouhal_optimal = 0.3f;
        float cruise_speed = (wingbeat.frequency_Hz * wingbeat.cruise_stroke_length_m) / strouhal_optimal;
        cruise_speed = std::max(cruise_speed, r.min_flight_speed_m_s * 1.2f);

        float strouhal_max = 0.2f;
        float max_speed = (wingbeat.frequency_Hz * wingbeat.cruise_stroke_length_m) / strouhal_max;

        return std::make_pair(cruise_speed, max_speed);
    };

    const auto [cruise_speed_m_s, max_speed_m_s] = compute_flight_speeds();
    r.cruise_speed_m_s = cruise_speed_m_s;
    r.max_flight_speed_m_s = max_speed_m_s;

    const float reynolds_cruise = (rho * cruise_speed_m_s * wing_params.mean_chord_m) / mu;

    // ============================================================================
    // FORWARD FLIGHT POWER
    // ============================================================================

    struct PowerCosts {
        float flapping_W;
        float hovering_W;
        float forward_power_ratio;
        float hover_power_ratio;
    };

    auto compute_power_costs = [&]() -> PowerCosts {
        // Disk area: swept area by flapping wings
        auto compute_disk_area = [&]() -> float {
            float disk_area = M_PI * wing_params.total_span_m * wing_params.total_span_m / 4.0f;

            if (wing_params.num_groups > 1) {
                disk_area = 0.0f;
                auto groups = GetGaitGroupSpan(wings.data(), wings.size());
                for (const auto& group : groups) {
                    if (group.count == 2) {
                        // Bilateral pair
                        disk_area += M_PI * group.total_span_m * group.total_span_m / 4.0f;
                    } else {
                        // Radial symmetry
                        float avg_span = group.total_span_m / group.count;
                        disk_area += group.count * M_PI * avg_span * avg_span / 4.0f;
                    }
                }
            }
            return disk_area;
        };

        float disk_area_m2 = compute_disk_area();

        // Forward flight power components
        auto compute_forward_power = [&]() -> float {
            // Induced power (momentum theory)
            float k_induced = 1.18f; // (Ellington 1984)
            float induced_W = k_induced * weight_N * weight_N /
                            (2.0f * rho * disk_area_m2 * cruise_speed_m_s);

            // Profile drag: wings
            float CD_profile = 0.02f + 0.01f * (1.0f - in.feather_quality);
            float profile_W = 0.5f * rho * std::pow(cruise_speed_m_s, 3.0f) *
                            wing_params.total_area_m2 * CD_profile;

            // Parasite drag: body
            float CD_body = 0.1f + 0.05f * (1.0f - in.structure_vs_weight);
            float parasite_W = 0.5f * rho * std::pow(cruise_speed_m_s, 3.0f) *
                             out.physical.cross_sectional_area_m2 * CD_body;

            // Inertial power (scales with wing inertia)
            float aerodynamic_W = induced_W + profile_W + parasite_W;
            float size_factor = std::pow(wing_params.avg_inertia_kg_m2 / 1e-6, 0.3f);
            float inertial_W = aerodynamic_W * 0.10f * size_factor;

            // Convert to metabolic cost
            float mechanical_W = aerodynamic_W + inertial_W;
            float muscle_efficiency = 0.20f + 0.03f * in.feather_quality;
            return mechanical_W / muscle_efficiency;
        };

        // Hovering power components
        auto compute_hovering_power = [&]() -> float {
            // Induced power: P = T * v_induced
            float v_induced = std::sqrt(weight_N / (2.0f * rho * disk_area_m2));
            float induced_W = weight_N * v_induced;

            // Profile power: sum over all wings
            float CD_profile = 0.02f + 0.01f * (1.0f - in.feather_quality);
            float profile_W = 0.0f;
            for (const auto& wing : wings) {
                float wing_tip_velocity = 2.0f * wing.beat_amplitude_rad * wing.span_m *
                                         wingbeat.frequency_Hz;
                float mean_velocity = 0.7f * wing_tip_velocity;
                float wing_CD = CD_profile * 1.5f;  // Higher AoA in hover
                profile_W += 0.5f * rho * std::pow(mean_velocity, 3.0f) *
                           wing.area_m2 * wing_CD;
            }

            // Multi-wing efficiency bonus
            if (wing_params.num_groups > 1) {
                profile_W *= (1.0f - 0.2f * (wing_params.num_groups - 1) / 3.0f);
            }

            // Inertial power (figure-8 pattern more complex)
            float size_factor = std::pow(wing_params.avg_inertia_kg_m2 / 1e-6, 0.3f);
            float aerodynamic_W = induced_W + profile_W;
            float inertial_W = aerodynamic_W * 0.10f * size_factor * 1.3f;

            // Reynolds-dependent efficiency (LEV contribution at low Re)
            float reynolds_efficiency = 1.0f;
            if (reynolds_cruise < 10000.0f) {
                reynolds_efficiency = 0.85f + 0.15f * (reynolds_cruise / 10000.0f);
                // Require larger amplitude for LEV stabilization
                for (auto& wing : wings) {
                    wing.beat_amplitude_rad *= 1.4f;
                }
            }

            float mechanical_W = (induced_W + profile_W + inertial_W) / reynolds_efficiency;
            float muscle_efficiency = 0.20f + 0.03f * in.feather_quality;
            return mechanical_W / muscle_efficiency;
        };

        float flapping_W = compute_forward_power();
        float hovering_W = compute_hovering_power();

        return {
            flapping_W,
            hovering_W,
            wingbeat.available_power_W / flapping_W,
            wingbeat.available_power_W / hovering_W
        };
    };

    auto power = compute_power_costs();
    r.flapping_cost_W_per_N = power.flapping_W / weight_N;
    r.hovering_cost_W_per_N = power.hovering_W / weight_N;

    // Set preliminary flight capabilities (may be refined after frequency adjustment)
    r.can_sustain_level_flight = (power.forward_power_ratio >= 1.0f);
    r.can_slow_descent = (power.forward_power_ratio >= 0.5f && !r.can_sustain_level_flight);
    r.can_hover = (power.hover_power_ratio > 1.0f);
    r.hovering_efficiency = r.can_hover ? std::min(1.0f, (power.hover_power_ratio - 1.0f) / 2.0f) : 0.0f;

    // Get gait group centers for later use
    std::span<TonTon::Output_Aerial::Wing> as_span(wings);
    auto centers = GetGaitGroupCenters(in, as_span);

    // ============================================================================
    // POWER BUDGET ADJUSTMENT
    // ============================================================================

    auto adjust_for_power_budget = [&]() {
        // Use the better of forward or hovering power (specialists excel at one mode)
        float best_power_ratio = std::max(power.forward_power_ratio, power.hover_power_ratio);

        if (best_power_ratio >= 1.0f) {
            // Sufficient power, calculate efficiencies
            r.flapping_efficiency = (power.forward_power_ratio >= 1.0f) ?
                std::min(1.0f, (power.forward_power_ratio - 1.0f) / 0.5f) : 0.0f;
            return;
        }

        // Insufficient power - reduce wingbeat frequency
        // Power scales with frequency cubed (P ∝ f³)
        float frequency_scale = std::pow(best_power_ratio, 1.0f / 3.0f);
        r.wingbeat_frequency_Hz *= frequency_scale;

        // Recalculate speeds with reduced frequency (use CRUISE amplitude)
        float strouhal_optimal = 0.3f;
        r.cruise_speed_m_s = (r.wingbeat_frequency_Hz * wingbeat.cruise_stroke_length_m) / strouhal_optimal;
        r.cruise_speed_m_s = std::max(r.cruise_speed_m_s, r.min_flight_speed_m_s * 1.2f);

        float strouhal_max = 0.2f;
        r.max_flight_speed_m_s = (r.wingbeat_frequency_Hz * wingbeat.cruise_stroke_length_m) / strouhal_max;

        // Recalculate power costs (P ∝ f³)
        float freq_cube = frequency_scale * frequency_scale * frequency_scale;
        r.flapping_cost_W_per_N *= freq_cube;
        r.hovering_cost_W_per_N *= freq_cube;

        float new_forward_ratio = wingbeat.available_power_W / (r.flapping_cost_W_per_N * weight_N);
        float new_hover_ratio = wingbeat.available_power_W / (r.hovering_cost_W_per_N * weight_N);

        // Update flight capabilities
        r.can_sustain_level_flight = (new_forward_ratio >= 1.0f);
        r.can_slow_descent = (new_forward_ratio >= 0.5f && !r.can_sustain_level_flight);
        r.flapping_efficiency = (new_forward_ratio >= 1.0f) ?
            std::min(1.0f, (new_forward_ratio - 1.0f) / 0.5f) : 0.0f;

        r.can_hover = (new_hover_ratio > 1.0f);
        r.hovering_efficiency = r.can_hover ?
            std::min(1.0f, (new_hover_ratio - 1.0f) / 2.0f) : 0.0f;
    };

  //  adjust_for_power_budget();

    // ============================================================================
    // TURNING PERFORMANCE
    // ============================================================================

    auto compute_turning_performance = [&]() {
        // Minimum radius limited by structural loads
        float max_load_factor = glm::mix(3.0, 9.0, in.structure_vs_weight);
        r.min_turning_radius_m = (r.cruise_speed_m_s * r.cruise_speed_m_s) /
                                 (g * std::sqrt(max_load_factor * max_load_factor - 1.0f));

        // Roll/pitch/yaw rates limited by wing control authority and body inertia
        auto get_inertia_about_axis = [I=out.physical.inertia_restPose()](glm::vec3 const& axis) {
            return glm::dot(axis, I * axis);
        };

        float lift_coeff = 0.5f;
        float total_wing_force = 0.5f * rho * (r.cruise_speed_m_s * r.cruise_speed_m_s) *
                                (wing_params.total_area_m2 / 4.0f) * lift_coeff;
        float differential_force = total_wing_force * 0.5f;

        // Roll: left vs right wings
        float roll_torque = differential_force * (wing_params.total_span_m / 2.0f);
        r.max_roll_rate_rad_s = roll_torque / get_inertia_about_axis({0, 0, 1});

        // Pitch: fore vs aft wings
        float forewing_aft_separation = 0.0056f;  // Approximate for dragonfly-like config
        float pitch_torque = differential_force * forewing_aft_separation;
        r.max_pitch_rate_rad_s = pitch_torque / get_inertia_about_axis({1, 0, 0});

        // Yaw: counter-rotation and body drag
        float yaw_torque = differential_force * (wing_params.total_span_m / 2.0f);
        r.max_yaw_rate_rad_s = yaw_torque / get_inertia_about_axis({0, 1, 0});
    };

    compute_turning_performance();

    // ============================================================================
    // EMPIRICAL / ALLOMETRIC SCALING
    // ============================================================================

    auto apply_empirical_scaling = [&]() {
        // Altitude limit (metabolic capacity and oxygen delivery)
        float altitude_metabolic_factor = in.metabolic_efficiency * in.muscle_quality;
        r.max_altitude_m = 3000.0f + 6000.0f * altitude_metabolic_factor;

        // Endurance (Kleiber's law)
        float BMR_W = 3.5f * std::pow(body_mass_kg, 0.75f);
        float flight_metabolic_multiplier = 10.0f + 5.0f * (1.0f - in.metabolic_efficiency);
        float flight_metabolic_rate_W = BMR_W * flight_metabolic_multiplier;

        // Fat reserves: 5-20% depending on migration strategy
        float fat_fraction = 0.05f + 0.15f * in.behavior.endurance_vs_power;
        float fat_mass_kg = body_mass_kg * fat_fraction;
        float energy_available_J = fat_mass_kg * 39.0e6f;  // Fat energy density

        r.max_flight_duration_s = energy_available_J / flight_metabolic_rate_W;

        // Activity level: soaring birds fly slower
        r.cruise_speed_m_s *= (0.8f + 0.4f * (1.0f - in.activity_level));
    };

    apply_empirical_scaling();

    // ============================================================================
    // CLADE-SPECIFIC REFINEMENTS
    // ============================================================================

    auto apply_clade_refinements = [&]() {
        using CF = CladeFlags;

        // AVES: Bird-specific flight characteristics
        if (HasFlag(out.physical.clade, CF::AVES)) {
            // Pennycuick (1996): Universal wingbeat frequency f = 3.87 × M^(-0.33)
            float pennycuick_freq_Hz = 3.87f * std::pow(body_mass_kg, -0.33f);

            // Apply as constraint if significantly different
            if (std::abs(r.wingbeat_frequency_Hz - pennycuick_freq_Hz) > pennycuick_freq_Hz * 0.3f) {
                r.wingbeat_frequency_Hz = glm::mix(r.wingbeat_frequency_Hz, pennycuick_freq_Hz, 0.5f);
            }

            // Birds are endotherms (~40°C body temp) - can sustain high power output
            // Already reflected in metabolic scaling
        }

        // INSECTA: Insect-specific flight (fundamentally different aerodynamics)
        if (HasFlag(out.physical.clade, CF::INSECTA)) {
            // Insects operate at LOW Reynolds number (10-10,000)
            float insect_reynolds = (rho * r.cruise_speed_m_s * wing_params.mean_chord_m) / mu;

            if (insect_reynolds < 10000.0f) {
                // Leading edge vortex (LEV) enhanced lift
                // Dickinson et al. (1999): LEV generates 2-3x quasi-steady lift
                for (auto& wing : r.wings) {
                    const_cast<float&>(wing.beat_amplitude_rad) =
                        std::max(wing.beat_amplitude_rad, 2.0f);  // ~115° minimum
                }

                // Hovering is EASIER at low Re (LEV stabilization)
                r.hovering_efficiency = std::min(r.hovering_efficiency * 1.3f, 1.0f);
            }

            // Asynchronous muscle: high frequency but reduced control
            if (r.wingbeat_frequency_Hz > 100.0f) {
                r.min_turning_radius_m *= 1.5f;
            }
        }

        // CHIROPTERA: Bats (mammalian fliers)
        if (HasFlag(out.physical.clade, CF::MAMMALIA) && r.wings.size() >= 2) {
            // Membrane wings: higher drag but better control
            float membrane_drag_penalty = 1.2f;
            r.cruise_speed_m_s *= (1.0f / std::sqrt(membrane_drag_penalty)); // ~8% slower

            // Exceptional low-speed maneuverability (Norberg & Rayner 1987)
            r.min_turning_radius_m *= 0.6f;

            // Dynamic camber control enables slow flight without stall
            r.min_flight_speed_m_s *= 0.7f;
        }

        // ARTHROPODA: Exoskeleton mass constraint
        if (HasFlag(out.physical.clade, CF::ARTHROPODA)) {
            // Exoskeleton mass scaling M^1.0 limits maximum size
            // Max viable flying arthropod ~100g (historical dragonflies)
            if (body_mass_kg > 0.1f) {
                r.flapping_efficiency *= 0.3f;  // Severe penalty
                r.hovering_efficiency *= 0.3f;
            }
        }
    };

    apply_clade_refinements();

    // ============================================================================
    // VALIDATION & RETURN
    // ============================================================================

    // Add diagnostics if creature has wings but can't fly
    if (!r.can_sustain_level_flight && !r.can_hover && !r.can_slow_descent) {
        using Severity = Output_Diagnostics::Warning::Severity;

        // Calculate power ratio to explain the problem
        float power_needed_W = r.flapping_cost_W_per_N * weight_N;
        float power_ratio = wingbeat.available_power_W / power_needed_W;

        out.diagnostics.warnings.push_back({
            Severity::ERROR,
            "Flight impossible: insufficient muscle power. Need " +
            std::to_string(power_needed_W) + " W but only have " +
            std::to_string(wingbeat.available_power_W) + " W available (ratio: " +
            std::to_string(power_ratio) + ", need ≥1.0)"
        });

        // Break down where the power is going
        if (r.flapping_cost_W_per_N > 100.0f) {
            out.diagnostics.warnings.push_back({
                Severity::CAUTION,
                "Power cost extremely high (" + std::to_string(r.flapping_cost_W_per_N) +
                " W/N). Expected 5-20 W/N for insects, 10-40 W/N for birds. " +
                "Physics model may be miscalibrated."
            });
        }

        if (r.cruise_speed_m_s > 20.0f && body_mass_kg < 0.1f) {
            out.diagnostics.warnings.push_back({
                Severity::CAUTION,
                "Cruise speed suspiciously high (" + std::to_string(r.cruise_speed_m_s) +
                " m/s = " + std::to_string(r.cruise_speed_m_s * 3.6f) + " km/h) for small flyer. " +
                "Typical small insects: 2-10 m/s."
            });
        }
    }

    // If creature has wings, return analysis even if flight isn't viable
    // The boolean flags and efficiency values indicate capabilities
    return r;
}

std::vector<TonTon::Output_Aerial::Wing> TonTon::GetWings(Input const& in, Scratch & out, bool & has_feathers)
{
using SF = SemanticFlags;
	auto & sk = *in.skinnedMesh;
	auto * sk_memo = sk.skin->memo();
	auto position = sk.skin->position.data();
	
	auto appendages = GetAppendages(in, GetChainsFromRoot(in, SF::WING));
	std::vector<TonTon::Output_Aerial::Wing> r;
	r.resize(appendages.size());
	
	double body_density=glm::mix(700.0, 1050.0, in.average_density);
	
	for(auto i = 0u; i < r.size(); ++i)
	{
		(Output_Chain&)r[i] = appendages[i];
		r[i].span_m = r[i].stretched_length_m;
		
		SkinnedMesh::LimbMetrics limb_metrics;
		std::pair<glm::quat, glm::vec3> eigen_decomposition;
		
		auto relevant_joints = sk_memo->GetAllChildrenOfRoot(r[i].root);
		auto which = std::span<uint16_t>(relevant_joints.data(), relevant_joints.size());
		auto projection = in.skinnedMesh->memo()->GetProjectionMatrix(EigenValue::Small, in.behavior.scale, which, &limb_metrics, &eigen_decomposition);
		auto metrics = in.skinnedMesh->memo()->GetSilhouettes(projection, in.behavior.scale, which);
		
		r[i].area_m2 = metrics.area;
		r[i].chord_m = metrics.MeasureWidth_Segment(position[r[i].root], position[r[i].tip]).length;
		
		auto axis = GetTangentAxis(EigenValue::Small, eigen_decomposition.first, in.position(r[i].root), in.position(r[i].tip));
		
		r[i].mass_kg = limb_metrics.volume * body_density;
		r[i].inertia_kgm2 = limb_metrics.GetInertia(position[r[i].root], body_density, axis);
			
		// --- WING INERTIA --- 
		// Van Den Berg & Rayner (1995): I = k × m_wing × L² (R^2 = 0.97)
		// Wing mass typically 10-15% of body mass (more for hovering specialists)
		// only applies to birds.
		float wing_mass_fraction = 0.10f + 0.05f * in.stability_vs_speed;
		float wing_mass_kg = out.physical.body_mass_kg * wing_mass_fraction;
		float wing_inertia_kg_m2 = 0.33f * wing_mass_kg * r[i].span_m * r[i].span_m * 4.0;
		
	// indicates feathers (if its below this it must be a membrane)
		if(wing_mass_kg < r[i].mass_kg)	
		{
			r[i].mass_kg = wing_mass_kg;
			r[i].inertia_kgm2 = wing_inertia_kg_m2;
			has_feathers = true;
		}
	}
	
	for(auto i = 0u; i < r.size(); )
	{
		glm::vec3 accumulator{0.0};
		
		int begin = i;
		for(; i < r.size(); ++i)
		{
			if(r[begin].gait_group != r[i].gait_group)
				break;
			
			accumulator += in.position(r[i].root);
		}
		
		accumulator /= (i - begin);
		
		for(i=begin; i < r.size(); ++i)
		{
			if(r[begin].gait_group != r[i].gait_group)
				break;
				
			r[i].span_m += glm::distance(in.position(r[i].root), accumulator);
		}
	}
	
	return r;
}

/*
struct GaitGroupSpan
{
	uint16_t gaitGroup;
	uint16_t count;
	float total_span_m;
};
*/

std::vector<GaitGroupSpan> TonTon::GetGaitGroupSpan(Output_Aerial::Wing * data, size_t size)
{
	std::vector<GaitGroupSpan> group_span;
	group_span.reserve(2);
	
	for(auto i = 0u; i < size; ++i)
	{
		Output_Aerial::Wing * p = &data[i];
	
		for(auto j = 0u; j < group_span.size(); ++j)
		{
			if(group_span[j].gaitGroup == p->gait_group)
			{
				group_span[j].total_span_m += p->span_m;
				group_span[j].count += 1;
				goto found;
			}
			
			if(group_span[j].gaitGroup > p->gait_group)
			{
				group_span.insert(group_span.begin()+j, {p->gait_group,1, p->span_m});
				goto found;
			}
		}

		group_span.push_back({p->gait_group,1, p->span_m});
		
	found:
		(void)0;
	}
	
	return group_span;
}
