#ifndef METRICS_HERPETOLOGY_H
#define METRICS_HERPETOLOGY_H
#include "metrics_inputsbase.h"
#include <glm/vec3.hpp>
#include <vector>

/* HERTON: 
 * 
 * Herpetological Ecological & Reptilian Trait-Oriented Normalization
 * 
 */ 

namespace Herpetology
{

enum AI_Components
{
    AmbushPredator,
    ActiveForager,
    TerritorialDefender,
    SocialBasker,
    NocturnalHunter
};

enum Animation_Components
{
    LimbedQuadruped,
    LateralUndulation,
    RectilinearCrawler,
    BallisticJumper,
    ClimbingSpecialist
};

struct ThermalEnvironment {
    float ambient_temp_K{298.15};        // K (drives all metabolic rates)
    float substrate_temp_K{303.15};      // K (for behavioral thermoregulation)
    float humidity_percent{60.0};        // % (critical for amphibians)
    float wind_speed_m_s{0.5};          // m/s (affects evaporative cooling)
    float solar_radiation_W_m2{800.0};  // W/m² (basking opportunity)
};

struct TerrestrialEnvironment {
    float gravity_m_s2{9.81};           // m/s² (affects locomotor stress)
    float air_density_kg_m3{1.2};       // kg/m³ (minimal effect vs fish)
    float ground_friction_coef{0.6};    // μ (substrate-dependent)
    float substrate_roughness_m{0.01};  // m (affects snake locomotion)
};

struct HerpetologyTonTonInputs : public Metrics::InputsBase {
    ThermalEnvironment thermal;
    TerrestrialEnvironment terrain;
    
    // BEHAVIORAL FINE-TUNING (0-1 sliders, auto-computed defaults)
    float thermoregulation_strategy{0.5};  // 0=thermoconformer, 1=active regulator
    float activity_pattern{0.5};            // 0=nocturnal, 1=diurnal
    float burst_endurance_tradeoff{0.5};   // 0=pure sprinter, 1=marathon runner
    float climbing_specialization{0.5};     // 0=terrestrial only, 1=arboreal specialist
    
    // ENVIRONMENTAL CONTEXT (optional overrides)
    bool force_limbed_locomotion{false};
    bool force_limbless_locomotion{false};
    float preferred_body_temp_override{-1}; // -1 = auto-detect from morphology
};

struct MorphologicalAnalysis {
    static MorphologicalAnalysis AnalyzeMorphology(const HerpetologyTonTonInputs& inputs);

    // CORE GEOMETRIC MEASUREMENTS
    float body_length_m;                    // Snout-vent length (SVL)
    float body_mass_kg;                     // Volume × tissue density (~1050 kg/m³)
    float surface_area_to_volume;           // m⁻¹ (affects water loss & heat exchange)
    float limb_length_ratio;                // Limb length / SVL (0 for snakes)
    float femur_diameter_to_length;         // Bone robustness indicator
    float tail_length_ratio;                // Tail length / SVL
    
    // LIMB ANALYSIS (for tetrapods)
    struct LimbData {
        float length{0.0f};
        float diameter{0.0f};
        float moment_arm{0.0f};
        bool present{false};
    };
    LimbData forelimbs, hindlimbs;
    
    float duty_factor_predicted;            // Time foot on ground / stride period
    float sprawl_angle_degrees;             // Femur angle from horizontal
    
    // BODY PLAN CLASSIFICATION
    bool is_limbless;                       // Snake/amphisbaenian
    bool is_tetrapod;                       // Lizard/salamander/frog
    bool is_elongate;                       // Body length > 5× width
    bool has_reduced_limbs;                 // Limb reduction grade
    
    // SPECIALIZED STRUCTURES
    bool has_adhesive_pads;                 // Gecko-like toe pads detected
    float pad_total_area_m2;                // Total adhesive pad surface area
    float setae_density_estimated;          // Setae per m² (if pads present)
    
    bool has_heterodont_dentition;          // Varied tooth types (predator indicator)
    float gape_width_ratio;                 // Max jaw opening / head width
    
    // HEAD AND SENSORY
    float eye_diameter_ratio;               // Eye diameter / SVL
    float tympanum_size_ratio;              // Ear size / head width (if present)
    bool has_jacobsons_organ;               // Vomeronasal detection capability
    
    // TAIL ANALYSIS
    bool has_prehensile_tail;              // For arboreal species
    bool has_caudal_autotomy;              // Tail breakage adaptation
    float tail_cross_section_area;          // For sidewinding snakes
    
    // PREDICTIONS WITH CONFIDENCE
    struct Prediction {
        float value;      // 0-1 normalized
        float confidence; // R² from research
        float final(float manual_adjustment, float weight = 0.2) const;
    };
    
    Prediction locomotor_mode;              // 0=snake, 0.25=reduced limbs, 0.75=quadruped, 1=jumper
    Prediction sprint_capability;           // 0=slow, 1=fast burst speed
    Prediction endurance_capability;        // 0=anaerobic only, 1=sustained aerobic
    Prediction climbing_ability;            // 0=terrestrial, 1=vertical surfaces
    Prediction thermoregulation_precision;  // 0=wide tolerance, 1=narrow optimal range
    Prediction predatory_capability;        // 0=herbivore, 1=carnivore
    Prediction activity_temperature_range;  // Thermal performance breadth
    Prediction water_dependency;            // 0=xeric adapted, 1=requires moisture
    
    // CONFIDENCE ASSESSMENT
    float overall_confidence;
};

struct HerpetologyBehavioralProfile {
    // LOCOMOTION CHARACTERISTICS
    float max_sprint_speed_BL_s;           // Body lengths per second (burst)
    float sustained_speed_BL_s;            // Aerobic cruising speed
    float turning_radius_BL;               // Minimum turn radius in body lengths
    float acceleration_BL_s2;              // Maximum acceleration
    
    // LOCOMOTOR MODES (can be multi-modal)
    float lateral_undulation_efficiency;    // 0-1, primary for snakes
    float rectilinear_efficiency;          // 0-1, slow snake mode
    float sidewinding_efficiency;          // 0-1, desert specialists
    float concertina_efficiency;           // 0-1, climbing/tunneling
    float quadrupedal_efficiency;          // 0-1, lizards/salamanders
    float saltatory_efficiency;            // 0-1, jumping ability
    
    // CLIMBING CAPABILITY
    float vertical_climbing_speed_BL_s;    // Speed on vertical surfaces
    float adhesion_force_N;                // Total gecko-style adhesion
    float clinging_duration_s;             // Maximum hang time
    bool requires_roughness_for_climbing;  // vs adhesive climbing
    
    // THERMAL BIOLOGY
    float preferred_body_temp_C;           // Tset - behavioral target
    float critical_thermal_min_C;          // CTmin - lower lethal limit
    float critical_thermal_max_C;          // CTmax - upper lethal limit  
    float thermal_performance_breadth_C;   // Width of performance plateau
    float optimal_performance_temp_C;      // Topt - peak locomotor performance
    float basking_requirement;             // 0=nocturnal/tropical, 1=requires basking
    
    // ENERGETICS & METABOLISM
    float resting_metabolic_rate_W;        // At Tset, Q10-corrected
    float max_aerobic_metabolic_rate_W;    // During sustained activity
    float anaerobic_capacity_kJ;           // Burst energy available
    float cost_of_transport_J_kg_m;        // Net COT at optimal speed
    
    // HYDRATION PHYSIOLOGY (critical for amphibians)
    float water_loss_rate_g_h;             // Evaporative water loss
    float critical_dehydration_threshold;   // % body mass loss before performance decline
    float rehydration_rate_g_h;            // Water uptake through skin
    bool requires_free_water;               // vs metabolic water only
    
    // FEEDING & BEHAVIOR
    float ambush_vs_active_foraging;       // 0=sit-and-wait, 1=active hunter
    float territory_size_m2;               // Defended area
    float home_range_m2;                   // Total movement area
    float daily_movement_distance_m;       // Typical daily travel
    
    // SOCIAL & REPRODUCTIVE
    float social_tolerance;                // 0=territorial, 1=colonial
    bool exhibits_parental_care;
    float breeding_site_fidelity;          // Return to specific locations
    
    // ACTIVITY PATTERNS
    float diurnal_activity;                // 0=nocturnal, 0.5=crepuscular, 1=diurnal
    float seasonal_activity_months;        // Active months per year
    bool hibernates_or_estivates;
    
    // COMPONENT SELECTION
    AI_Components recommended_ai;
    Animation_Components recommended_animation;
    float simulation_priority;
};

struct EnvironmentalLimits {
    static EnvironmentalLimits Factory(
        HerpetologyTonTonInputs const& inputs,
        MorphologicalAnalysis const& analysis);

    // THERMAL LIMITS
    float min_activity_temp_C;             // Below this, too cold to move
    float max_activity_temp_C;             // Above this, heat stress
    float preferred_microhabitat_temp_C;   // Behavioral thermoregulation target
    float lethal_cold_exposure_hours;      // Time to CTmin death
    float lethal_heat_exposure_hours;      // Time to CTmax death
    
    // HYDRATION LIMITS (especially amphibians)
    float min_humidity_percent;            // Below this, desiccation risk
    float max_water_loss_percent_mass;     // Dehydration tolerance
    float time_to_critical_dehydration_hours; // At current conditions
    
    // LOCOMOTOR CONSTRAINTS
    float min_substrate_temperature_C;     // Too cold for locomotion
    float max_substrate_temperature_C;     // Burns/thermal damage
    float min_friction_coefficient;        // Can't generate traction
    float max_slope_angle_degrees;         // Gravitational limit
    
    // OXYGEN & ALTITUDE
    float max_elevation_m;                 // Respiratory capacity limit
    float min_oxygen_percent;              // Hypoxia threshold
    
    // SUBSTRATE REQUIREMENTS
    bool requires_complex_terrain;         // For ambush sites, burrows
    bool requires_basking_sites;           // Open solar exposure
    bool requires_water_access;            // For amphibians
    float min_canopy_cover_percent;        // Shade requirement
    float max_canopy_cover_percent;        // Light requirement
};

struct LocomotionPhysics {
    // FORCE GENERATION
    float max_muscle_stress_Pa;            // ~200,000 Pa typical
    float muscle_power_density_W_kg;       // ~200-400 W/kg
    float muscle_efficiency;               // ~0.25-0.30 typical
    
    // BONE LOADING
    float femur_peak_stress_Pa;            // Torsional for sprawling
    float bone_safety_factor;              // Actual strength / working stress
    bool torsion_dominated_loading;        // True for sprawling posture
    
    // SCALING RELATIONSHIPS
    float speed_scaling_exponent;          // v ∝ M^x (typically ~0.17 for lizards)
    float frequency_scaling_exponent;      // f ∝ M^x (typically ~-0.15)
    float stride_length_scaling;           // stride ∝ M^x (typically ~0.33)
    
    // FRICTION & TRACTION
    float required_friction_coefficient;    // For max acceleration
    float static_friction_available;        // From scales/skin
    float directional_friction_ratio;       // Forward/lateral (snakes)
    
    // ADHESION MECHANICS (geckos)
    float van_der_waals_force_per_seta_N;  // ~20-200 μN per seta
    float total_setae_count;                // Millions for large geckos
    float adhesive_stiffness_N_m;           // Spring constant of adhesive system
    float peel_angle_optimal_degrees;       // ~30° for maximum adhesion
};

struct ThermalPerformanceCurve {
    // CURVE SHAPE PARAMETERS
    float T_min;                           // Lower thermal limit (activity ceases)
    float T_opt;                           // Optimal performance temperature
    float T_max;                           // Upper thermal limit (activity ceases)
    float performance_breadth;             // T_max - T_min
    float plateau_width;                   // Width of 80%+ performance range
    
    // PERFORMANCE SCALING
    float max_performance_value;           // At T_opt (speed, force, etc.)
    float thermal_sensitivity;             // Slope of performance vs temp
    float Q10_resting;                     // Metabolic rate temperature dependence
    float Q10_active;                      // Active metabolic rate scaling
    
    // ACCLIMATION CAPACITY
    float acclimation_rate_C_per_day;      // Thermal plasticity
    float CTmax_acclimation_range_C;       // How much CTmax can shift
    float CTmin_acclimation_range_C;       // How much CTmin can shift
    
    // Calculate performance at given temperature
    float PerformanceAtTemp(float temp_C) const;
};

struct SnakeLocomotionParameters {
    // LATERAL UNDULATION (primary terrestrial mode)
    float undulation_frequency_Hz;         // 0.5-3 Hz typical
    float wavelength_body_lengths;         // ~0.6-0.8 BL typical
    float amplitude_body_widths;           // ~0.5-1.5 body widths
    float forward_friction_coef;           // Scales parallel to body
    float lateral_friction_coef;           // Scales perpendicular (higher)
    float friction_anisotropy_ratio;       // Lateral/forward (1.5-3.0)
    
    // RECTILINEAR (slow, stealthy)
    float rectilinear_speed_cm_s;          // 0.2-6 cm/s
    float rectilinear_wave_amplitude;      // 0.004-0.013 BL (very small)
    float rectilinear_frequency_Hz;        // 0.2-0.5 Hz (increases with size)
    float belly_scale_efficiency;          // Static friction utilization
    
    // SIDEWINDING (desert specialists)
    float sidewinding_speed_km_h;          // Up to 3.7 km/h for Crotalus cerastes
    float sidewinding_frequency_Hz;        // Increases with speed
    float sidewinding_amplitude_BL;        // 0.1-0.14 BL typical
    float contact_point_count;             // Usually 2-3 ground contacts
    float sidewinding_efficiency;          // Energy per distance (8 J/kg/m - most efficient)
    
    // CONCERTINA (climbing/tunneling)
    float concertina_speed_BL_s;           // Slowest mode
    float concertina_energy_cost;          // 170 J/kg/m (most expensive)
    float static_friction_requirement;     // Must exceed body weight component
    float accordion_ratio;                 // Compression/extension range
    
    // SUBSTRATE ADAPTATION
    float optimal_substrate_roughness_mm;  // Surface feature size for traction
    float min_push_point_spacing_BL;       // For lateral undulation
    bool can_swim;                         // Aquatic capability
    bool can_climb;                        // Vertical surface capability
};

struct JumpingParameters {
    // BALLISTIC JUMP MECHANICS (frogs/toads)
    float max_jump_distance_m;             // Horizontal
    float max_jump_height_m;               // Vertical
    float takeoff_velocity_m_s;            // Initial velocity
    float takeoff_angle_degrees;           // Optimal ~45° for distance
    
    // POWER GENERATION
    float muscle_shortening_percent;       // Before joint movement (~7.8% in frogs)
    float tendon_elastic_storage_J;        // Energy stored in plantaris tendon
    float catapult_power_multiplier;       // Power output / muscle power limit
    float jump_frequency_max_Hz;           // Recovery time limited
    
    // SCALING RELATIONSHIPS
    float jump_height_scaling;             // H/L ~ L^(-1/3) ~ Fr²
    float froude_number;                   // √(v²/gL) - dimensionless
    bool uses_elastic_storage;             // True for most anurans
    
    // LANDING MECHANICS
    float landing_impact_force_N;          // Peak force on touchdown
    float landing_g_force;                 // Acceleration in g's
    float energy_dissipation_efficiency;   // How well shock is absorbed
};

struct ClimbingParameters {
    // GECKO ADHESION SYSTEM
    float total_adhesive_force_N;          // Sum of all setae
    float adhesive_pressure_Pa;            // Force per contact area
    float adhesion_on_rough_surface;       // 60% at 3mm amplitude roughness
    float optimal_peel_angle_deg;          // ~30° for attach/detach
    
    // SYSTEM STIFFNESS (scales with body size)
    float adhesive_system_stiffness_N_m;   // Larger geckos = stiffer (2-250g range)
    float toe_pad_area_scaling;            // Explains 92% of variation with stiffness
    
    // CLIMBING PERFORMANCE
    float vertical_speed_cm_s;             // On smooth vertical surfaces
    float inverted_ceiling_capability;     // Can hang upside down
    float max_overhang_angle_degrees;      // Beyond vertical
    
    // ALTERNATIVE CLIMBING (non-gecko)
    float claw_penetration_depth_mm;       // For rough surface climbing
    float required_substrate_roughness_mm; // Minimum for traction
    bool uses_prehensile_tail;             // Additional support
    float grip_strength_N;                 // For branch grasping
};

};

#endif // METRICS_HERPETOLOGY_H
