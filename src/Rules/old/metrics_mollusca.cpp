#include "metrics_mollusca.h"
#include "gltfmeshfile.h"
#include <cmath>
#include <algorithm>
#include <numeric>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace Mollusca;

// --- Physical Constants ---

// Mollusk tissue density (Love 1970)
constexpr float MOLLUSK_TISSUE_DENSITY = 1050.0f; // kg/m³

// Gastropod mucus properties (Denny & Gosline 1980, Denny 1984)
constexpr float MUCUS_WATER_CONTENT = 0.95f; // 91-98% typical
constexpr float MUCUS_SHEAR_MODULUS_BASE = 50.0f; // Pa
constexpr float MUCUS_KINETIC_FRICTION = 0.15f; // Dimensionless
constexpr float MUCUS_STATIC_FRICTION = 0.45f; // Dimensionless

// Scallop muscle performance (Cheng et al. 1996, Denny & Miller 2006)
constexpr float ADDUCTOR_PEAK_POWER_W_KG = 185.0f; // W/kg muscle
constexpr float ADDUCTOR_STRESS_PA = 1.06e5f; // Pa (~106 kPa)
constexpr float ADDUCTOR_CYCLIC_WORK_J_KG = 9.0f; // J/kg

// Pteropod aerodynamics (Murphy et al. 2016)
constexpr float PTEROPOD_LIFT_COEFFICIENT = 0.8f; // At 45-50° AoA
constexpr float PTEROPOD_DRAG_COEFFICIENT = 1.2f; // Including shell

float MorphologicalAnalysis::Prediction::final(float manual_adjustment, float weight) const {
    if (confidence > 0.9f) return value; // High confidence, use geometry
    
    float uncertainty = 1.0f - confidence;
    float manual_influence = uncertainty * weight;
    
    return value * (1.0f - manual_influence) + 
           manual_adjustment * manual_influence;
}

MorphologicalAnalysis MorphologicalAnalysis::AnalyzeMorphology(const MolluskTonTonInputs& inputs) {
    gltf::MeshFile_SkinnedMesh const& skinnedMesh = inputs.skinnedMesh();
    gltf::MeshFile_Skin const& skin = inputs.skin();
    
    MorphologicalAnalysis analysis = {};
    
    // ========== BASIC GEOMETRIC MEASUREMENTS ==========
    
    auto bounds = inputs.GetBoundingBox();
    float total_volume = skinnedMesh.volume * inputs.volume_scale();
    float total_surface_area = skinnedMesh.surfaceArea * inputs.area_scale();
    
    analysis.body_length = std::max({
        bounds.max.x - bounds.min.x,
        bounds.max.y - bounds.min.y,
        bounds.max.z - bounds.min.z
    });
    
    analysis.body_volume = total_volume;
    analysis.body_mass = total_volume * MOLLUSK_TISSUE_DENSITY;
    analysis.surface_area_to_volume = total_surface_area / total_volume;
    
    // Calculate mean diameter for fineness ratio
    float mean_diameter = std::sqrt(total_volume / (M_PI * analysis.body_length * 0.25f));
    analysis.fineness_ratio = analysis.body_length / mean_diameter;
    
    // ========== SHELL DETECTION AND CLASSIFICATION ==========
    
    float shell_volume = 0.0f;
    float soft_tissue_volume = 0.0f;
    glm::vec3 shell_com = glm::vec3(0);
    float shell_max_dimension = 0.0f;
    
    bool has_foot = false;
    bool has_wings = false;
    bool has_bivalve_shells = false;
    
    inputs.for_each_tag([&](int i, immutable_array<counted_string> tags) {
        // Shell detection: high density or "shell" tag
        bool is_shell = inputs.HasTag(tags, "shell") || 
                       inputs.HasTag(tags, "valve") ||
                       inputs.HasTag(tags, "carapace");
        
        // Soft tissue features
        bool is_foot = inputs.HasTag(tags, "foot") || 
                      inputs.HasTag(tags, "sole");
        bool is_wing = inputs.HasTag(tags, "wing") || 
                      inputs.HasTag(tags, "parapodia") ||
                      inputs.HasTag(tags, "parapodium");
        bool is_valve = inputs.HasTag(tags, "valve");
        
        // Position-based heuristics if no tags
        if (tags.empty()) {
            float sa_v_ratio = skinnedMesh.perBone.surfaceArea[i] / 
                              skinnedMesh.perBone.volume[i];
            glm::vec3 pos = skinnedMesh.perBone.centroid[i];
            
            // Shell-like: low SA/V ratio (thick structure)
            if (sa_v_ratio < 100.0f && !std::isnan(sa_v_ratio)) {
                is_shell = true;
            }
            
            // Foot-like: flat, ventral, high SA/V
            if (sa_v_ratio > 200.0f && pos.y < bounds.min.y + 0.3f * (bounds.max.y - bounds.min.y)) {
                is_foot = true;
            }
            
            // Wing-like: thin lateral structures
            if (sa_v_ratio > 300.0f && 
                std::abs(pos.x) > 0.6f * (bounds.max.x - bounds.min.x) * 0.5f) {
                is_wing = true;
            }
        }
        
        if (is_shell) {
            shell_volume += skinnedMesh.perBone.volume[i];
            shell_com += skinnedMesh.perBone.centroid[i] * skinnedMesh.perBone.volume[i];
            shell_max_dimension = std::max(shell_max_dimension,
                glm::length(skinnedMesh.perBone.centroid[i] - bounds.center()));
        } else {
            soft_tissue_volume += skinnedMesh.perBone.volume[i];
        }
        
        has_foot = has_foot || is_foot;
        has_wings = has_wings || is_wing;
        has_bivalve_shells = has_bivalve_shells || is_valve;
    });
    
    // Shell mass fraction (shell density ~2700 kg/m³ for aragonite/calcite)
    // Denny & Miller (2006): shell mass is substantial constraint on swimming
    float shell_mass = shell_volume * 2700.0f;
    analysis.shell_mass_fraction = shell_mass / (shell_mass + soft_tissue_volume * MOLLUSK_TISSUE_DENSITY);
    
    if (shell_volume > 0.0f) {
        analysis.shell_centroid = shell_com / shell_volume;
    }
    
    // ========== CLASS DETERMINATION ==========
    
    // Classification logic based on morphological features
    if (has_bivalve_shells) {
        analysis.detected_class = Bivalvia;
        analysis.primary_locomotion = (shell_volume / total_volume < 0.7f) ? JetEscape : Sessile;
    } else if (has_wings) {
        analysis.detected_class = Gastropoda; // Pteropods are gastropods
        analysis.primary_locomotion = PteropodFlight;
    } else if (has_foot) {
        analysis.detected_class = Gastropoda;
        analysis.primary_locomotion = PedalWaveCrawling;
    } else {
        // Default fallback based on geometry
        if (analysis.fineness_ratio > 1.5f && shell_volume / total_volume < 0.3f) {
            analysis.detected_class = Gastropoda;
            analysis.primary_locomotion = PedalWaveCrawling;
        } else {
            analysis.detected_class = Bivalvia;
            analysis.primary_locomotion = Sessile;
        }
    }
    
    // ========== GASTROPOD-SPECIFIC ANALYSIS ==========
    
    if (analysis.detected_class == Gastropoda && analysis.primary_locomotion == PedalWaveCrawling) {
        // Foot area estimation (ventral surface)
        analysis.foot_area = 0.0f;
        float foot_length = 0.0f;
        float foot_width = 0.0f;
        
        inputs.for_each_tag([&](int i, immutable_array<counted_string> tags) {
            bool is_foot = inputs.HasTag(tags, "foot") || inputs.HasTag(tags, "sole");
            
            if (!is_foot && tags.empty()) {
                glm::vec3 pos = skinnedMesh.perBone.centroid[i];
                float sa_v = skinnedMesh.perBone.surfaceArea[i] / skinnedMesh.perBone.volume[i];
                
                // Ventral + flat = likely foot
                if (pos.y < bounds.min.y + 0.3f * (bounds.max.y - bounds.min.y) && sa_v > 200.0f) {
                    is_foot = true;
                }
            }
            
            if (is_foot) {
                analysis.foot_area += skinnedMesh.perBone.surfaceArea[i];
                glm::vec3 pos = skinnedMesh.perBone.centroid[i];
                foot_length = std::max(foot_length, pos.z - bounds.min.z);
                foot_width = std::max(foot_width, std::abs(pos.x));
            }
        });
        
        // Fallback: estimate foot as ~30% of body surface (conservative)
        if (analysis.foot_area < 1e-6f) {
            analysis.foot_area = total_surface_area * 0.3f;
            foot_length = analysis.body_length * 0.7f;
            foot_width = analysis.body_length * 0.3f;
        }
        
        analysis.foot_aspect_ratio = foot_length / std::max(foot_width * 2.0f, 1e-6f);
    }
    
    // ========== BIVALVE-SPECIFIC ANALYSIS ==========
    
    if (analysis.detected_class == Bivalvia) {
        // Shell height (dorso-ventral dimension)
        analysis.shell_height = bounds.max.y - bounds.min.y;
        
        // Hinge line is typically the shortest dimension
        float dim_x = bounds.max.x - bounds.min.x;
        float dim_y = bounds.max.y - bounds.min.y;
        float dim_z = bounds.max.z - bounds.min.z;
        analysis.hinge_length = std::min({dim_x, dim_y, dim_z});
        
        // Gape area estimation (aperture when valves open)
        // Cheng & DeMont (1996): jet aperture critical for thrust
        analysis.shell_gape_area = analysis.shell_height * dim_z * 0.6f; // ~60% of projected area
        
        // Adductor attachment area (muscle scar)
        // Estimate as ~5% of shell surface (anatomical proportion)
        analysis.adductor_attachment_area = total_surface_area * 0.05f;
    }
    
    // ========== PTEROPOD-SPECIFIC ANALYSIS ==========
    
    if (analysis.primary_locomotion == PteropodFlight) {
        analysis.parapodial_area = 0.0f;
        float wing_span = 0.0f;
        
        inputs.for_each_tag([&](int i, immutable_array<counted_string> tags) {
            bool is_wing = inputs.HasTag(tags, "wing") || 
                          inputs.HasTag(tags, "parapodia");
            
            if (!is_wing && tags.empty()) {
                float sa_v = skinnedMesh.perBone.surfaceArea[i] / skinnedMesh.perBone.volume[i];
                glm::vec3 pos = skinnedMesh.perBone.centroid[i];
                
                // Lateral thin structures = wings
                if (sa_v > 300.0f && std::abs(pos.x) > 0.5f * (bounds.max.x - bounds.min.x) * 0.5f) {
                    is_wing = true;
                }
            }
            
            if (is_wing) {
                analysis.parapodial_area += skinnedMesh.perBone.surfaceArea[i];
                wing_span = std::max(wing_span, 
                    std::abs(skinnedMesh.perBone.centroid[i].x) * 2.0f);
            }
        });
        
        // Fallback: estimate from body size
        if (analysis.parapodial_area < 1e-6f) {
            // Murphy et al. (2016): wings ~2× body frontal area
            analysis.parapodial_area = analysis.body_length * analysis.body_length * 2.0f;
            wing_span = analysis.body_length * 2.0f;
        }
        
        // Wing aspect ratio: span²/area (Murphy et al. 2016)
        analysis.wing_aspect_ratio = (wing_span * wing_span) / std::max(analysis.parapodial_area, 1e-6f);
        
        // Shell drag (Murphy et al. 2016: shell is ~2/3 of body mass)
        analysis.shell_drag_coefficient = PTEROPOD_DRAG_COEFFICIENT;
    }
    
    // ========== MOMENT OF INERTIA ==========
    
    float total_inertia_y = 0.0f;
    for (size_t i = 0; i < skin.joints.size(); i++) {
        glm::mat3 global_inertia = transform_inertia(
            inputs.file.nodes._referencePose[i],
            skinnedMesh.perBone.inertia[i],
            skinnedMesh.perBone.volume[i]
        );
        total_inertia_y += global_inertia[1][1] * skinnedMesh.perBone.volume[i];
    }
    
    // Normalized for comparison (McHenry et al. 2003 method)
    analysis.moment_of_inertia_normalized = total_inertia_y / 
        (analysis.body_mass * analysis.body_length * analysis.body_length);
    
    // ========== PREDICTIONS WITH CONFIDENCE ==========
    
    // Locomotion mode score (already determined above)
    switch (analysis.primary_locomotion) {
        case PedalWaveCrawling:
            analysis.locomotion_mode_score = {0.0f, has_foot ? 0.85f : 0.60f};
            break;
        case JetEscape:
            analysis.locomotion_mode_score = {0.33f, has_bivalve_shells ? 0.80f : 0.50f};
            break;
        case PteropodFlight:
            analysis.locomotion_mode_score = {0.67f, has_wings ? 0.90f : 0.55f};
            break;
        case Sessile:
            analysis.locomotion_mode_score = {1.0f, 0.70f};
            break;
        default:
            analysis.locomotion_mode_score = {0.5f, 0.40f};
    }
    
    // Swimming capability (0=crawling only, 1=excellent swimmer)
    // Based on shell mass fraction and body plan
    float swim_score = 0.0f;
    float swim_confidence = 0.65f;
    
    if (analysis.primary_locomotion == PteropodFlight) {
        swim_score = 0.9f - analysis.shell_mass_fraction * 0.3f; // Shell is ballast
        swim_confidence = 0.85f;
    } else if (analysis.primary_locomotion == JetEscape) {
        // Cheng & DeMont (1996): shell mass critical constraint
        swim_score = 0.6f - analysis.shell_mass_fraction * 0.5f;
        swim_confidence = 0.80f;
    } else {
        swim_score = 0.1f; // Gastropod crawlers: poor swimmers
        swim_confidence = 0.75f;
    }
    
    analysis.swimming_capability = {
        std::clamp(swim_score, 0.0f, 1.0f),
        swim_confidence
    };
    
    // Escape performance (burst capability)
    // Guderley & Tremblay (2013): varies with shell mass and muscle capacity
    float escape_score = 0.0f;
    float escape_confidence = 0.50f;
    
    if (analysis.primary_locomotion == JetEscape) {
        // Denny & Miller (2006): low shell mass + high muscle ratio = good escape
        float muscle_fraction = 1.0f - analysis.shell_mass_fraction;
        escape_score = muscle_fraction * 0.8f + (1.0f - analysis.moment_of_inertia_normalized) * 0.2f;
        escape_confidence = 0.75f;
    } else if (analysis.primary_locomotion == PteropodFlight) {
        // Murphy et al. (2016): pteropods have moderate burst capability
        escape_score = 0.5f;
        escape_confidence = 0.65f;
    } else {
        // Gastropods: very limited burst escape (mostly hide in shell)
        escape_score = 0.1f;
        escape_confidence = 0.70f;
    }
    
    analysis.escape_performance = {
        std::clamp(escape_score, 0.0f, 1.0f),
        escape_confidence
    };
    
    // ========== FEATURE DETECTION ==========
    
    inputs.for_each_tag([&](int i, immutable_array<counted_string> tags) {
        if (inputs.HasTag(tags, "operculum")) analysis.has_operculum = true;
        if (inputs.HasTag(tags, "siphon")) analysis.has_siphon = true;
        if (inputs.HasTag(tags, "byssus") || inputs.HasTag(tags, "thread")) {
            analysis.has_byssal_threads = true;
        }
    });
    
    // ========== OVERALL CONFIDENCE ==========
    
    std::vector<float> confidences = {
        analysis.locomotion_mode_score.confidence,
        analysis.swimming_capability.confidence,
        analysis.escape_performance.confidence
    };
    
    analysis.overall_confidence = std::accumulate(confidences.begin(), 
        confidences.end(), 0.0f) / confidences.size();
    
    return analysis;
}

EnvironmentalLimits EnvironmentalLimits::Factory(
    const MolluskTonTonInputs& inputs,
    const MorphologicalAnalysis& analysis) {
    
    EnvironmentalLimits limits = {};
    const auto& env = inputs.environment;
    
    // ========== SUBSTRATE FRICTION REQUIREMENTS ==========
    
    if (analysis.primary_locomotion == PedalWaveCrawling) {
        // Denny (1981): mucus adhesion requires minimum friction
        // Below μ≈0.1, gastropod cannot generate forward thrust
        limits.min_substrate_friction = 0.1f + inputs.substrate_roughness * 0.15f;
        
        // Smoother substrates require more mucus, lower friction tolerance
        limits.min_substrate_friction = std::clamp(limits.min_substrate_friction, 0.08f, 0.30f);
    } else {
        limits.min_substrate_friction = 0.0f; // Swimmers don't need substrate
    }
    
    // ========== VISCOSITY LIMITS ==========
    
    if (analysis.primary_locomotion == PedalWaveCrawling) {
        // Denny & Gosline (1980): mucus must remain functional
        // At high viscosity, mucus production becomes prohibitively expensive
        limits.max_viscosity_for_mucus_Pas = 0.01f * std::exp(env.temperature_K / 100.0f);
        limits.max_viscosity_for_mucus_Pas = std::clamp(
            limits.max_viscosity_for_mucus_Pas, 0.001f, 0.1f);
    } else {
        // Swimmers: viscosity affects Reynolds number
        // Murphy et al. (2016): pteropods operate at Re=10-100
        float target_reynolds = 50.0f; // Midpoint of pteropod range
        float typical_speed = analysis.body_length * 2.0f; // 2 BL/s estimate
        
        limits.max_viscosity_for_mucus_Pas = (env.density_Kg_m3 * typical_speed * 
            analysis.body_length) / target_reynolds;
        limits.max_viscosity_for_mucus_Pas = std::clamp(
            limits.max_viscosity_for_mucus_Pas, 0.0001f, 0.05f);
    }
    
    // ========== FLOW VELOCITY LIMITS ==========
    
    if (analysis.primary_locomotion == PedalWaveCrawling) {
        // Denny (1984): adhesion force vs drag force
        float adhesion_force = analysis.foot_area * MUCUS_STATIC_FRICTION * 
            analysis.body_mass * env.gravity_m_s2;
        
        // Drag force: F_d = 0.5 * ρ * v² * C_d * A
        float frontal_area = analysis.body_length * analysis.body_length * 0.5f;
        float drag_coefficient = 1.2f; // Bluff body
        
        // Solve for max velocity before detachment
        limits.max_flow_velocity_ms = std::sqrt((2.0f * adhesion_force) / 
            (env.density_Kg_m3 * drag_coefficient * frontal_area));
        
        limits.max_flow_velocity_ms = std::clamp(limits.max_flow_velocity_ms, 0.1f, 10.0f);
    } else if (analysis.has_byssal_threads) {
        // Mussels: stronger attachment, can tolerate more flow
        limits.max_flow_velocity_ms = 5.0f + analysis.body_mass * 2.0f;
    } else {
        // Free swimmers: no attachment limit
        limits.max_flow_velocity_ms = 100.0f;
    }
    
    // ========== PRESSURE TOLERANCE ==========
    
    // Shell crush depth varies by thickness and structure
    // Denny & Miller (2006): scallops implode at species-specific depths
    float shell_thickness_estimate = analysis.shell_mass_fraction * analysis.body_length * 0.1f;
    
    if (analysis.detected_class == Bivalvia) {
        // Bivalves: moderately pressure-tolerant
        float base_pressure = 2.0e6f; // ~200m depth
        limits.pressure_tolerance_Pa = base_pressure * (1.0f + shell_thickness_estimate * 10.0f);
    } else if (analysis.primary_locomotion == PteropodFlight) {
        // Pteropods: thin shells, shallow water specialists
        limits.pressure_tolerance_Pa = 5.0e5f; // ~50m depth typical
    } else {
        // Gastropods: variable, typically shallow to moderate depth
        limits.pressure_tolerance_Pa = 1.0e6f * (1.0f + analysis.shell_mass_fraction * 2.0f);
    }
    
    limits.pressure_tolerance_Pa = std::clamp(limits.pressure_tolerance_Pa, 
        1.0e5f, 1.0e7f);
    
    // ========== THERMAL LIMITS ==========
    
    // Denny (1984): mucus viscosity temperature-dependent
    // Low temp: mucus too viscous, high temp: protein denaturation
    
    if (analysis.primary_locomotion == PedalWaveCrawling) {
        limits.min_temperature_K = 273.15f; // 0°C (mucus freezing)
        limits.max_temperature_K = 313.15f; // 40°C (protein damage)
        limits.optimal_temperature_K = 288.15f; // 15°C (temperate optimum)
    } else {
        // Marine mollusks: broader thermal tolerance
        limits.min_temperature_K = 271.15f; // -2°C (seawater freezing)
        limits.max_temperature_K = 303.15f; // 30°C (tropical max)
        limits.optimal_temperature_K = 285.15f; // 12°C (oceanic average)
    }
    
    // ========== ENERGETIC CONSTRAINTS ==========
    
    if (analysis.primary_locomotion == PedalWaveCrawling) {
        // Denny (1980a): cost of gastropod crawling
        // Mucus production is major energy sink
        float mucus_cost_J_per_m = 50.0f * analysis.body_mass; // Empirical
        float metabolic_rate_W = 0.5f * analysis.body_mass; // Low for ectotherms
        
        // Distance until mucus reserves depleted
        limits.max_continuous_distance_m = (metabolic_rate_W * 3600.0f) / mucus_cost_J_per_m;
        limits.max_continuous_distance_m = std::clamp(
            limits.max_continuous_distance_m, 1.0f, 1000.0f);
    } else if (analysis.primary_locomotion == JetEscape) {
        // Cheng et al. (1996): scallop escape limited by anaerobic capacity
        // ~5-10 claps before exhaustion typical
        float claps_available = 7.0f;
        float distance_per_clap = analysis.body_length * 1.5f;
        limits.max_continuous_distance_m = claps_available * distance_per_clap;
    } else if (analysis.primary_locomotion == PteropodFlight) {
        // Murphy et al. (2016): continuous swimming required to avoid sinking
        // Limited by aerobic metabolism
        limits.max_continuous_distance_m = 100.0f * analysis.body_length; // ~100 BL
    } else {
        limits.max_continuous_distance_m = 0.0f; // Sessile
    }
    
    // Recovery time (Guderley & Tremblay 2013)
    if (analysis.primary_locomotion == JetEscape) {
        limits.recovery_time_s = 30.0f + analysis.body_mass * 10.0f; // 30-60s typical
    } else {
        limits.recovery_time_s = 10.0f;
    }
    
    // Starvation time (rough estimate from metabolic rate)
    float metabolic_rate_W = 0.5f * analysis.body_mass;
    float energy_reserves_J = analysis.body_mass * 5000.0f; // ~5 kJ/kg reserves
    limits.starvation_time_s = energy_reserves_J / metabolic_rate_W;
    limits.starvation_time_s = std::clamp(limits.starvation_time_s, 3600.0f, 2592000.0f); // 1hr-30days
    
    return limits;
}

LocomotionPerformance ConstructLocomotionPerformance(
    const MolluskTonTonInputs& inputs,
    const MorphologicalAnalysis& analysis) {
    
    LocomotionPerformance perf = {};
    const auto& env = inputs.environment;
    
    // ========== GASTROPOD PEDAL WAVE CRAWLING ==========
    
    if (analysis.primary_locomotion == PedalWaveCrawling) {
        // Speed scaling (Pembury Smith & Ruxton 2021, Denny 1980a)
        // Typical range: 2-20 cm/min depending on species and substrate
        
        // Base speed from body size (Bainbridge-like scaling)
        float base_speed_cm_min = 10.0f * std::pow(analysis.body_length * 100.0f, 0.3f);
        
        // Substrate corrections (Pembury Smith & Ruxton 2021)
        float roughness_factor = 1.0f - inputs.substrate_roughness * 0.6f; // Rough = slower
        float inclination_factor = std::cos(inputs.substrate_inclination_deg * M_PI / 180.0f);
        
        perf.max_crawling_speed_ms = (base_speed_cm_min / 60.0f) * 0.01f * 
            roughness_factor * inclination_factor;
        
        // Clamp to observed range
        perf.max_crawling_speed_ms = std::clamp(perf.max_crawling_speed_ms, 
            0.0005f, 0.005f); // 0.03-0.3 cm/s = 1.8-18 cm/min
        
        perf.max_crawling_speed_BLs = perf.max_crawling_speed_ms / analysis.body_length;
        
        // Pedal wave parameters (Crozier & Pilz 1924, Lissmann 1945)
        // Wavelength typically 0.5-1.0× body length
        perf.pedal_wavelength_m = analysis.body_length * 0.7f;
        
        // Wave speed > body speed (Denny 1981: ratio typically 1.2-1.5)
        float wave_speed_ms = perf.max_crawling_speed_ms * 1.3f;
        perf.pedal_wave_frequency_Hz = wave_speed_ms / perf.pedal_wavelength_m;
        
        // Adhesion force (Denny 1981, Denny & Gosline 1980)
        perf.adhesion_force_N = analysis.foot_area * MUCUS_STATIC_FRICTION * 
            analysis.body_mass * env.gravity_m_s2;
        
        // Thrust force (limited by mucus hysteresis)
        perf.thrust_force_N = analysis.foot_area * MUCUS_KINETIC_FRICTION * 
            analysis.body_mass * env.gravity_m_s2 * 0.5f;
        
        // Mucus production rate (Denny 1980b)
        // Roughly 5-10% of body mass per day
        perf.mucus_production_rate_kg_s = analysis.body_mass * 0.07f / 86400.0f;
        
        // Cost of transport (Denny 1980a)
        // Gastropods are efficient: ~0.5-2 J/kg/m
        perf.metabolic_cost_J_per_m = 1.0f * analysis.body_mass;
        
        // No burst capability
        perf.escape_velocity_ms = perf.max_crawling_speed_ms * 1.2f;
        perf.muscle_power_output_W = perf.thrust_force_N * perf.max_crawperf.muscle_power_output_W = perf.thrust_force_N * perf.max_crawling_speed_ms;
        
        // Maximum inclination (Denny 1981)
        // Limited by adhesion vs gravity component
        perf.max_inclination_deg = std::atan(MUCUS_STATIC_FRICTION) * 180.0f / M_PI;
        perf.max_inclination_deg = std::clamp(perf.max_inclination_deg, 30.0f, 80.0f);
        
        // Minimum substrate friction requirement
        perf.minimum_substrate_friction = 0.1f;
        
        // Turning radius (generally very tight for crawlers)
        perf.turning_radius_BL = 0.5f;
        
        // Sustainable distance limited by mucus reserves
        perf.max_sustainable_distance_m = 10.0f * analysis.body_length;
    }
    
    // ========== SCALLOP JET ESCAPE ==========
    
    if (analysis.primary_locomotion == JetEscape) {
        // Shell clapping dynamics (Cheng & DeMont 1996, Cheng et al. 1996)
        
        // Estimate adductor muscle mass (~10-15% of soft tissue)
        float soft_tissue_mass = analysis.body_mass * (1.0f - analysis.shell_mass_fraction);
        float adductor_mass = soft_tissue_mass * 0.12f;
        
        // Clapping frequency (Cheng et al. 1996: typically 1-3 Hz depending on size)
        perf.shell_clap_frequency_Hz = 2.0f * std::pow(analysis.body_length, -0.5f);
        perf.shell_clap_frequency_Hz = std::clamp(perf.shell_clap_frequency_Hz, 0.5f, 4.0f);
        
        // Muscle power output (Denny & Miller 2006)
        perf.muscle_power_output_W = adductor_mass * ADDUCTOR_PEAK_POWER_W_KG;
        perf.adductor_stress_Pa = ADDUCTOR_STRESS_PA;
        
        // Jet velocity (Cheng & DeMont 1996)
        // v_jet = sqrt(2 * ΔP / ρ) where ΔP from muscle contraction
        float pressure_differential = perf.adductor_stress_Pa;
        float jet_velocity = std::sqrt(2.0f * pressure_differential / env.density_Kg_m3);
        
        // Escape velocity (body velocity from jet reaction)
        // Momentum conservation: m_body * v_body = m_water * v_jet
        float jet_volume = analysis.shell_gape_area * analysis.shell_height * 0.3f; // ~30% cavity volume
        float water_mass = jet_volume * env.density_Kg_m3;
        
        perf.escape_velocity_ms = (water_mass * jet_velocity) / analysis.body_mass;
        perf.escape_velocity_ms = std::clamp(perf.escape_velocity_ms, 0.1f, 3.0f);
        
        // Thrust force
        perf.thrust_force_N = water_mass * jet_velocity * perf.shell_clap_frequency_Hz;
        
        // Anaerobic capacity (Guderley & Tremblay 2013)
        // Scallops rely on anaerobic metabolism for escape
        perf.anaerobic_capacity_J = adductor_mass * ADDUCTOR_CYCLIC_WORK_J_KG * 7.0f; // ~7 claps
        
        // Cost of transport (high for jet propulsion)
        perf.metabolic_cost_J_per_m = analysis.body_mass * 10.0f; // Inefficient
        
        // Maximum distance (limited by anaerobic reserves)
        float distance_per_clap = perf.escape_velocity_ms / perf.shell_clap_frequency_Hz;
        perf.max_sustainable_distance_m = distance_per_clap * 7.0f; // 7 claps typical
        
        // Turning: poor (Cheng & DeMont 1996: asymmetric jet apertures needed)
        perf.turning_radius_BL = 3.0f + analysis.moment_of_inertia_normalized * 5.0f;
        
        // No crawling capability
        perf.max_crawling_speed_ms = 0.0f;
        perf.max_crawling_speed_BLs = 0.0f;
    }
    
    // ========== PTEROPOD UNDERWATER FLIGHT ==========
    
    if (analysis.primary_locomotion == PteropodFlight) {
        // Wing kinematics (Murphy et al. 2016, Chang & Yen 2012)
        
        // Beat frequency scaling (Murphy et al. 2016: 4-10 Hz)
        perf.wing_beat_frequency_Hz = 7.0f * std::pow(analysis.body_length, -0.3f);
        perf.wing_beat_frequency_Hz = std::clamp(perf.wing_beat_frequency_Hz, 4.0f, 10.0f);
        
        // Wing stroke amplitude (Murphy et al. 2016)
        perf.wing_stroke_amplitude_rad = (45.0f + 5.0f * analysis.wing_aspect_ratio) * M_PI / 180.0f;
        perf.wing_stroke_amplitude_rad = std::clamp(perf.wing_stroke_amplitude_rad, 
            40.0f * M_PI / 180.0f, 60.0f * M_PI / 180.0f);
        
        // Body pitch (Murphy et al. 2016: hyper-pitching up to ±60°)
        perf.body_pitch_angle_rad = 60.0f * M_PI / 180.0f;
        
        // Swimming speed (Chang & Yen 2012)
        // Temperate: 10-50 mm/s, sub-Arctic: up to 120 mm/s
        float base_speed_ms = 0.03f * std::pow(analysis.body_length * 1000.0f, 0.5f); // Size scaling
        
        // Adjust for wing loading (Murphy et al. 2016)
        float wing_loading = analysis.body_mass * env.gravity_m_s2 / analysis.parapodial_area;
        float wing_factor = 1.0f / std::sqrt(wing_loading * 0.01f + 1.0f);
        
        perf.sustained_swimming_speed_ms = base_speed_ms * wing_factor;
        perf.sustained_swimming_speed_ms = std::clamp(perf.sustained_swimming_speed_ms, 
            0.01f, 0.12f); // 10-120 mm/s
        
        // Escape velocity (moderate burst capability)
        perf.escape_velocity_ms = perf.sustained_swimming_speed_ms * 1.5f;
        
        // Lift and thrust forces (Murphy et al. 2016)
        // L = 0.5 * ρ * v² * CL * A
        float velocity_squared = perf.sustained_swimming_speed_ms * perf.sustained_swimming_speed_ms;
        float lift_force = 0.5f * env.density_Kg_m3 * velocity_squared * 
            PTEROPOD_LIFT_COEFFICIENT * analysis.parapodial_area;
        
        // Must generate lift > weight to avoid sinking
        float weight = analysis.body_mass * env.gravity_m_s2;
        perf.thrust_force_N = std::max(lift_force, weight * 1.2f); // 20% margin
        
        // Power output (from wing beat frequency and force)
        float stroke_length = analysis.parapodial_area * 0.5f * perf.wing_stroke_amplitude_rad;
        perf.muscle_power_output_W = perf.thrust_force_N * stroke_length * 
            perf.wing_beat_frequency_Hz;
        
        // Cost of transport (Murphy et al. 2016)
        // Intermediate between efficient swimmers and inefficient jet propulsion
        perf.metabolic_cost_J_per_m = analysis.body_mass * 5.0f;
        
        // Continuous swimming required (negatively buoyant)
        // Distance limited by aerobic capacity
        float metabolic_rate_W = analysis.body_mass * 2.0f; // Active metabolism
        float energy_reserves_J = analysis.body_mass * 2000.0f; // Limited reserves
        float swim_time_s = energy_reserves_J / metabolic_rate_W;
        perf.max_sustainable_distance_m = perf.sustained_swimming_speed_ms * swim_time_s;
        perf.max_sustainable_distance_m = std::clamp(perf.max_sustainable_distance_m, 
            10.0f, 1000.0f);
        
        // Turning (Murphy et al. 2016: good maneuverability)
        perf.turning_radius_BL = 1.0f + analysis.moment_of_inertia_normalized * 2.0f;
        
        // No substrate locomotion
        perf.max_crawling_speed_ms = 0.0f;
    }
    
    // ========== SESSILE/BURROWING ==========
    
    if (analysis.primary_locomotion == Sessile || analysis.primary_locomotion == Burrowing) {
        // No active locomotion
        perf.max_crawling_speed_ms = 0.0f;
        perf.escape_velocity_ms = 0.0f;
        perf.sustained_swimming_speed_ms = 0.0f;
        
        // Burrowing has slow substrate movement
        if (analysis.primary_locomotion == Burrowing) {
            perf.max_crawling_speed_ms = 0.0001f; // ~1 cm/min into substrate
        }
    }
    
    // ========== ENVIRONMENTAL ADJUSTMENTS ==========
    
    // Temperature effects (Denny 1984, general Q10 principles)
    float temperature_ratio = env.temperature_K / 298.15f; // Relative to 25°C
    float q10_factor = std::pow(2.0f, (temperature_ratio - 1.0f) * 10.0f / 10.0f); // Q10 ≈ 2
    
    perf.pedal_wave_frequency_Hz *= q10_factor;
    perf.shell_clap_frequency_Hz *= q10_factor;
    perf.wing_beat_frequency_Hz *= q10_factor;
    perf.max_crawling_speed_ms *= q10_factor;
    
    // Viscosity effects on swimming (Murphy et al. 2016: Reynolds number matters)
    float viscosity_ratio = env.viscosity_Pas / 0.001f;
    if (analysis.primary_locomotion == PteropodFlight) {
        // Higher viscosity helps at low Re (increased lift)
        perf.sustained_swimming_speed_ms *= std::pow(viscosity_ratio, -0.2f);
    }
    
    // Gravity effects
    float gravity_ratio = env.gravity_m_s2 / 9.81f;
    perf.escape_velocity_ms *= std::sqrt(gravity_ratio);
    perf.sustained_swimming_speed_ms *= std::sqrt(gravity_ratio);
    
    // Predation urgency modifier (user input)
    if (inputs.predation_urgency > 0.5f) {
        float urgency_multiplier = 1.0f + (inputs.predation_urgency - 0.5f) * 0.5f; // Up to 1.25×
        perf.escape_velocity_ms *= urgency_multiplier;
        perf.shell_clap_frequency_Hz *= urgency_multiplier;
        perf.wing_beat_frequency_Hz *= urgency_multiplier;
    }
    
    return perf;
}

MucusProperties ConstructMucusProperties(
    const MolluskTonTonInputs& inputs,
    const MorphologicalAnalysis& analysis) {
    
    MucusProperties mucus = {};
    
    if (analysis.primary_locomotion != PedalWaveCrawling) {
        return mucus; // No mucus for non-gastropod crawlers
    }
    
    const auto& env = inputs.environment;
    
    // ========== COMPOSITION (Denny & Gosline 1980) ==========
    
    mucus.water_content_fraction = MUCUS_WATER_CONTENT;
    
    // Glycoprotein concentration (Denny 1984)
    // Higher concentration = more viscous, better adhesion
    mucus.glycoprotein_concentration = (1.0f - MUCUS_WATER_CONTENT) * 50.0f; // ~2.5 g/L
    
    // ========== RHEOLOGICAL PROPERTIES (Denny 1981, Denny & Gosline 1980) ==========
    
    // Base shear modulus at 25°C
    mucus.shear_modulus_Pa = MUCUS_SHEAR_MODULUS_BASE;
    
    // Yield stress (minimum stress to initiate flow)
    // Denny (1981): ~10-100 Pa depending on species
    mucus.yield_stress_Pa = 30.0f + analysis.body_mass * 20.0f;
    mucus.yield_stress_Pa = std::clamp(mucus.yield_stress_Pa, 10.0f, 100.0f);
    
    // Friction coefficients (Denny 1981)
    mucus.kinetic_friction_coeff = MUCUS_KINETIC_FRICTION;
    mucus.static_friction_coeff = MUCUS_STATIC_FRICTION;
    
    // ========== TEMPERATURE DEPENDENCE (Denny 1984) ==========
    
    // Viscosity increases dramatically at low temperatures
    // Arrhenius-like relationship: η(T) = η₀ * exp(E_a / RT)
    float temperature_factor = std::exp(3000.0f * (1.0f / env.temperature_K - 1.0f / 298.15f));
    
    mucus.viscosity_at_temperature_Pas = 10.0f * temperature_factor; // Base ~10 Pa·s at 25°C
    mucus.viscosity_at_temperature_Pas = std::clamp(
        mucus.viscosity_at_temperature_Pas, 1.0f, 1000.0f);
    
    // Adhesion also temperature-dependent (less dramatic than viscosity)
    mucus.adhesion_at_temperature_N_m2 = 1000.0f / std::sqrt(temperature_factor);
    mucus.adhesion_at_temperature_N_m2 = std::clamp(
        mucus.adhesion_at_temperature_N_m2, 500.0f, 5000.0f);
    
    // ========== SUBSTRATE EFFECTS (Pembury Smith & Ruxton 2021) ==========
    
    // Rough substrates require more mucus, affect friction
    mucus.kinetic_friction_coeff *= (1.0f - inputs.substrate_roughness * 0.3f);
    mucus.static_friction_coeff *= (1.0f - inputs.substrate_roughness * 0.2f);
    
    return mucus;
}

ScallopDynamics ConstructScallopDynamics(
    const MolluskTonTonInputs& inputs,
    const MorphologicalAnalysis& analysis) {
    
    ScallopDynamics dynamics = {};
    
    if (analysis.primary_locomotion != JetEscape) {
        return dynamics; // Only for scallops
    }
    
    const auto& env = inputs.environment;
    
    // ========== MASS DISTRIBUTION (Cheng & DeMont 1996) ==========
    
    dynamics.shell_mass_kg = analysis.body_mass * analysis.shell_mass_fraction;
    
    float soft_tissue_mass = analysis.body_mass * (1.0f - analysis.shell_mass_fraction);
    dynamics.adductor_mass_kg = soft_tissue_mass * 0.12f; // ~12% is adductor
    
    // ========== HINGE MECHANICS (Denny & Miller 2006) ==========
    
    // Abductin elastic modulus (Denny & Miller 2006: resilience varies with temperature)
    // Stiffness: k = E * A / L where E = elastic modulus
    float abductin_modulus_Pa = 4.0e6f; // ~4 MPa typical
    
    // Temperature correction (Denny & Miller 2006: Arctic scallops maintain resilience)
    float temp_factor = env.temperature_K / 283.15f; // Relative to 10°C
    dynamics.hinge_stiffness_N_m = abductin_modulus_Pa * analysis.hinge_length * 0.01f * temp_factor;
    
    // Resilience (energy recovery fraction)
    // Denny & Miller (2006): 0.3-0.75 depending on species and temperature
    dynamics.hinge_resilience = 0.5f + (temp_factor - 1.0f) * 0.1f;
    dynamics.hinge_resilience = std::clamp(dynamics.hinge_resilience, 0.3f, 0.75f);
    
    // ========== JET PARAMETERS (Cheng & DeMont 1996) ==========
    
    // Jet volume: fraction of mantle cavity volume
    float mantle_volume = analysis.body_volume * (1.0f - analysis.shell_mass_fraction) * 0.6f;
    dynamics.jet_volume_m3 = mantle_volume * 0.7f; // ~70% expelled per clap
    
    // Jet velocity from pressure differential
    float pressure_differential = ADDUCTOR_STRESS_PA;
    dynamics.jet_velocity_ms = std::sqrt(2.0f * pressure_differential / env.density_Kg_m3);
    
    // Jet duration (Cheng et al. 1996: typically 0.1-0.3s)
    dynamics.jet_duration_s = 0.15f + analysis.body_length * 0.2f;
    dynamics.jet_duration_s = std::clamp(dynamics.jet_duration_s, 0.1f, 0.4f);
    
    // Aperture ratio (jet area / shell area)
    dynamics.aperture_ratio = analysis.shell_gape_area / 
        (analysis.shell_height * analysis.shell_height * 2.0f);
    dynamics.aperture_ratio = std::clamp(dynamics.aperture_ratio, 0.1f, 0.5f);
    
    // ========== HYDRODYNAMIC EFFICIENCY (Cheng & DeMont 1996) ==========
    
    // Froude efficiency: decreases with growth from ~0.5 to 0.3
    float size_factor = std::exp(-analysis.body_length * 2.0f); // Smaller = more efficient
    dynamics.froude_efficiency = 0.3f + size_factor * 0.2f;
    dynamics.froude_efficiency = std::clamp(dynamics.froude_efficiency, 0.25f, 0.55f);
    
    // Added mass coefficient (Cheng et al. 1996: ~10× valve mass)
    dynamics.added_mass_coefficient = 10.0f;
    
    // Optimal clap angle (Cheng & DeMont 1996: 6-12° from horizontal)
    dynamics.optimal_clap_angle_deg = 9.0f;
    
    // ========== POWER OUTPUT (Denny & Miller 2006) ==========
    
    dynamics.peak_power_W_kg = ADDUCTOR_PEAK_POWER_W_KG;
    dynamics.cyclic_work_J_kg = ADDUCTOR_CYCLIC_WORK_J_KG;
    
    return dynamics;
}

PteropodKinematics ConstructPteropodKinematics(
    const MolluskTonTonInputs& inputs,
    const MorphologicalAnalysis& analysis) {
    
    PteropodKinematics kine = {};
    
    if (analysis.primary_locomotion != PteropodFlight) {
        return kine; // Only for pteropods
    }
    
    const auto& env = inputs.environment;
    
    // ========== WING GEOMETRY (Murphy et al. 2016) ==========
    
    kine.wing_area_m2 = analysis.parapodial_area;
    
    // Estimate span and chord from aspect ratio
    // AR = span² / area
    kine.wing_span_m = std::sqrt(analysis.wing_aspect_ratio * kine.wing_area_m2);
    kine.wing_chord_m = kine.wing_area_m2 / kine.wing_span_m;
    
    // ========== STROKE PATTERN (Murphy et al. 2016) ==========
    
    // Beat frequency: 4-10 Hz (Murphy et al. 2016, Chang & Yen 2012)
    kine.beat_frequency_Hz = 7.0f * std::pow(analysis.body_length, -0.3f);
    kine.beat_frequency_Hz = std::clamp(kine.beat_frequency_Hz, 4.0f, 10.0f);
    
    // Stroke amplitude (Murphy et al. 2016: large angles for lift)
    kine.stroke_amplitude_deg = 50.0f + analysis.wing_aspect_ratio * 2.0f;
    kine.stroke_amplitude_deg = std::clamp(kine.stroke_amplitude_deg, 40.0f, 70.0f);
    
    // Angle of attack (Murphy et al. 2016: 45-50° for lift generation)
    kine.angle_of_attack_deg = 47.5f;
    
    // Body pitch range (Murphy et al. 2016: hyper-pitching ±60°)
    kine.body_pitch_range_deg = 60.0f;
    
    // ========== CONVERGENT EVOLUTION (Murphy et al. 2016) ==========
    
    // Figure-eight wingtip pattern (like insects)
    kine.figure_eight_pattern_ratio = kine.wing_span_m / kine.wing_chord_m;
    
    // Clap-and-fling efficiency (Weis-Fogh 1973, Murphy et al. 2016)
    // Enhances lift by ~30-50% over simple flapping
    kine.clap_fling_efficiency = 1.4f;
    
    // ========== PERFORMANCE (Murphy et al. 2016, Chang & Yen 2012) ==========
    
    // Lift coefficient at optimal angle of attack
    kine.lift_coefficient = PTEROPOD_LIFT_COEFFICIENT;
    
    // Drag coefficient (shell contributes significantly)
    kine.drag_coefficient = PTEROPOD_DRAG_COEFFICIENT;
    
    // Reynolds number (Murphy et al. 2016: Re=10-100 range)
    float swimming_speed = 0.03f; // Estimate ~30 mm/s typical
    kine.swimming_reynolds_number = (env.density_Kg_m3 * swimming_speed * kine.wing_chord_m) / 
        env.viscosity_Pas;
    kine.swimming_reynolds_number = std::clamp(kine.swimming_reynolds_number, 10.0f, 100.0f);
    
    // Sinking rate when not swimming (Murphy et al. 2016: negatively buoyant)
    // Terminal velocity: v_terminal = sqrt(2mg / (ρ C_d A))
    float weight = analysis.body_mass * env.gravity_m_s2;
    float frontal_area = analysis.body_length * analysis.body_length * 0.5f;
    kine.sinking_rate_ms = std::sqrt((2.0f * weight) / 
        (env.density_Kg_m3 * kine.drag_coefficient * frontal_area));
    kine.sinking_rate_ms = std::clamp(kine.sinking_rate_ms, 0.01f, 0.1f); // 10-100 mm/s
    
    return kine;
}

AnimationParameters ConstructAnimationParameters(
    const MolluskTonTonInputs& inputs,
    const MorphologicalAnalysis& analysis,
    const LocomotionPerformance& perf) {
    
    AnimationParameters params = {};
    
    // ========== GASTROPOD WAVE ANIMATION ==========
    
    if (analysis.primary_locomotion == PedalWaveCrawling) {
        // Wave speed ratio (Denny 1981: wave moves faster than body)
        params.wave_speed_ratio = 1.3f;
        
        // Wave amplitude (Lissmann 1945: typically small, ~5-10% of foot width)
        params.wave_amplitude_m = analysis.body_length * 0.02f;
        
        // Wave count (Crozier & Pilz 1924: typically 3-5 simultaneous waves)
        params.wave_count = 3.0f + analysis.foot_aspect_ratio * 1.5f;
        params.wave_count = std::clamp(params.wave_count, 2.0f, 6.0f);
        
        // Direct waves (most terrestrial gastropods use these)
        params.use_direct_waves = true;
        
        // Gastropods can remain stationary
        params.rest_pose_stability = 1.0f;
        params.requires_continuous_motion = false;
    }
    
    // ========== SCALLOP CLAPPING ANIMATION ==========
    
    if (analysis.primary_locomotion == JetEscape) {
        // Valve opening angle (Cheng & DeMont 1996)
        params.valve_opening_angle_deg = 30.0f + analysis.shell_gape_area * 100.0f;
        params.valve_opening_angle_deg = std::clamp(params.valve_opening_angle_deg, 20.0f, 50.0f);
        
        // Clap duration (adduction phase - fast)
        params.clap_duration_s = 0.1f / perf.shell_clap_frequency_Hz;
        
        // Refill duration (abduction phase - slower, elastic recoil)
        params.refill_duration_s = 0.6f / perf.shell_clap_frequency_Hz;
        
        // Can rest on substrate
        params.rest_pose_stability = 1.0f;
        params.requires_continuous_motion = false;
    }
    
    // ========== PTEROPOD FLIGHT ANIMATION ==========
    
    if (analysis.primary_locomotion == PteropodFlight) {
        // Wing stroke period
        params.wing_stroke_period_s = 1.0f / perf.wing_beat_frequency_Hz;
        
        // Upstroke vs downstroke ratio (Murphy et al. 2016: asymmetric)
        params.wing_upstroke_ratio = 0.45f; // Slightly faster upstroke
        
        // Body pitch amplitude (Murphy et al. 2016: extreme hyper-pitching)
        params.body_pitch_amplitude_deg = 60.0f;
        
        // Cannot rest - must swim continuously or sink
        params.rest_pose_stability = 0.0f;
        params.requires_continuous_motion = true;
    }
    
    // ========== SESSILE ==========
    
    if (analysis.primary_locomotion == Sessile) {
        params.rest_pose_stability = 1.0f;
        params.requires_continuous_motion = false;
    }
    
    return params;
}

/*
 * IMPLEMENTATION NOTES
 * 
 * This implementation follows the research-validated approach used in the
 * ichthyology module, with these mollusk-specific considerations:
 * 
 * 1. GASTROPOD CRAWLING: Based primarily on Mark Denny's extensive work
 *    (1980-1984) establishing the physics of mucus-based locomotion. The
 *    hysteresis property of mucus is the key enabling mechanism.
 * 
 * 2. SCALLOP JET PROPULSION: Follows Cheng & DeMont (1996) mathematical
 *    model integrating hinge mechanics, valve inertia, and fluid dynamics.
 *    The added mass coefficient (~10×) is critical for accurate prediction.
 * 
 * 3. PTEROPOD FLIGHT: Murphy et al. (2016) demonstrated remarkable convergent
 *    evolution with insect flight, including clap-and-fling mechanism and
 *    hyper-pitching. The low Reynolds number (10-100) regime is unique.
 * 
 * 4. SCALING: Unlike fish, mollusks show less consistent scaling relationships
 *    due to shell constraints and diverse body plans. Shell mass fraction is
 *    the dominant constraint on swimming performance.
 * 
 * 5. LIMITATIONS: This implementation focuses on PERFORMANCE PREDICTION rather
 *    than detailed animation curves. Most mollusks have relatively simple
 *    kinematics compared to fish, making this approach appropriate.
 */
