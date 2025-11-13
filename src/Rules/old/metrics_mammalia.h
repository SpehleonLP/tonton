#ifndef METRICS_MAMMALIA_H
#define METRICS_MAMMALIA_H
#include "metrics_inputsbase.h"
#include <glm/vec3.hpp>
#include <vector>
#include <optional>

/* Tonton: 
 * 
 * Trait Oriented Normalization for Traits of Organismal Nature - Mammalia
 * 
 * Principled BSDF for mammalian locomotion across terrestrial, arboreal, 
 * fossorial, and aquatic modes. Spans 0.02 kg (shrew) to 5000 kg (elephant).
 */ 

namespace Mammalia
{

enum LocomotorMode
{
    Cursorial,           // Running specialists (wolves, horses, cheetahs)
    Saltatorial,         // Jumping/hopping specialists (kangaroos, jerboas)
    Arboreal,            // Tree-dwelling climbers (squirrels, primates)
    Fossorial,           // Burrowing specialists (moles, badgers)
    Scansorial,          // Occasional climbers (bears, raccoons)
    Aquatic,             // Swimming specialists (cetaceans, pinnipeds)
    SemiAquatic,         // Amphibious (otters, beavers, hippos)
    Volant               // Gliding/flying (bats, flying squirrels)
};

enum GaitType
{
    Walk,
    Trot,
    Pace,
    Bound,
    Gallop,
    Pronk,
    Hop,
    Brachiation
};

enum ThermalStrategy
{
    Thermoconformer,     // Minimal thermoregulation (some hibernators)
    ModerateRegulator,   // Standard mammalian thermoregulation
    PreciseRegulator     // Tight temperature control (humans, large mammals)
};

enum AI_Components
{
    HerdBehavior,
    PackHunter,
    SolitaryPredator,
    TerritorialDefender,
    OpportunisticForager,
    MigratoryHerder,
    ColonialBurrower
};

enum Animation_Components
{
    QuadrupedalWalker,
    CursorsSprinter,
    PowerJumper,
    ArborealClimber,
    AquaticSwimmer,
    BipedalHopper
};

struct TerrestrialEnvironment {
    float gravity_m_s2{9.81f};           // m/s² (affects jumping, gait transitions)
    float air_density_kg_m3{1.225f};     // kg/m³ (affects heat loss, drag)
    float temperature_K{298.15f};        // K (affects thermoregulation costs)
    float substrate_friction{0.6f};      // Coefficient (affects traction, climbing)
    float substrate_compliance{0.1f};    // 0=rigid, 1=soft (affects locomotion efficiency)
    float altitude_m{0.0f};              // Meters above sea level (affects O₂ availability)
};

struct MammalTonTonInputs : public Metrics::InputsBase {
    TerrestrialEnvironment environment;
    
    // BEHAVIORAL FINE-TUNING (0-1 sliders, auto-computed defaults)
    float social_adjustment = 0.5f;      // ±20% from sensory organ predictions
    float aggression_adjustment = 0.5f;  // ±20% from morphology predictions
    float activity_adjustment = 0.5f;    // ±20% from metabolic predictions
    
    // LOCOMOTOR CONTEXT (optional overrides)
    std::optional<LocomotorMode> force_locomotor_mode;  // Override geometry detection
    std::optional<float> preferred_speed_mps;           // Override speed predictions
    std::optional<ThermalStrategy> thermal_strategy;    // Override thermal inference
    
    // ENVIRONMENTAL CONTEXT
    bool nocturnal_override = false;     // Force nocturnal activity pattern
    bool fossorial_override = false;     // Force burrowing behavior
};

struct MorphologicalAnalysis {
    static MorphologicalAnalysis AnalyzeMorphology(const MammalTonTonInputs& inputs);
    
    // CORE GEOMETRIC MEASUREMENTS
    float body_length_m;                 // Snout-vent or head-body length
    float body_mass_kg;                  // Estimated from volume × tissue density (~1050 kg/m³)
    float limb_length_ratio;             // Limb length / body length
    float digit_length_ratio;            // Digit length / palm length (arboreal indicator)
    float tail_length_ratio;             // Tail length / body length
    float surface_area_to_volume;        // m⁻¹ (thermal exchange capacity)
    float body_compactness;              // Volume / (length × width × height)
    glm::vec3 moment_of_inertia;         // kg·m² (maneuverability, turning)
    
    // LIMB MORPHOLOGY
    float forelimb_robustness;           // Cross-sectional area / length (digging power)
    float hindlimb_length_ratio;         // Hindlimb / forelimb length (jumping indicator)
    float claw_curvature;                // Radians (climbing ability)
    float claw_length_ratio;             // Claw length / digit length
    float foot_pad_area_ratio;           // Pad area / foot area (grip vs speed)
    
    // HEAD AND SENSORY
    float eye_diameter_ratio;            // Eye diameter / skull length
    float ear_size_ratio;                // Ear area / head area
    float snout_length_ratio;            // Snout / skull length (fossorial indicator)
    float jaw_robustness;                // Jaw depth / length (bite force indicator)
    
    // SKELETAL FEATURES
    float bone_gracility;                // Average bone diameter / length (cursorial = gracile)
    float vertebral_flexibility;         // Estimated from torso length/height ratio
    float limb_posture_angle;            // Degrees from vertical (sprawling vs erect)
    
    // GEOMETRIC PREDICTIONS (with confidence from allometric studies)
    struct Prediction {
        float value;         // 0-1 normalized
        float confidence;    // R² from research (0.6-0.9+)
        float final(float manual_adjustment, float weight = 0.2f) const;
    };
    
    // Biewener (1989, 1990): posture scaling, safety factors 2-4
    Prediction locomotor_mode;           // Primary locomotion type from morphology
    
    // Alexander & Jayes (1983): Froude number gait transitions
    Prediction cursorial_capability;     // 0=poor runner, 1=sprint specialist
    
    // Alexander (1974, 1975): elastic energy storage scaling
    Prediction saltatorial_capability;   // 0=no jumping, 1=specialized hopper
    
    // Cartmill (1985), Jenkins & McClearn (1984): arboreal adaptations
    Prediction arboreal_capability;      // 0=terrestrial only, 1=specialized climber
    
    // Hopkins (2007), Casinos et al. (1993): fossorial morphology
    Prediction fossorial_capability;     // 0=surface only, 1=specialized digger
    
    // Fish & Baudinette (1999), Williams (1999): aquatic adaptations
    Prediction aquatic_capability;       // 0=terrestrial, 1=fully aquatic
    
    // Lovegrove (2000), Clarke & Rothery (2008): metabolic scaling
    Prediction metabolic_intensity;      // 0=low BMR, 1=high BMR relative to mass
    
    // Kotrschal et al. (1998), Healy & Rowe (2007): brain size correlates
    Prediction social_complexity;        // 0=solitary, 1=complex social structure
    
    // Muchlinski (2010), Kirk (2006): sensory ecology
    Prediction sensory_strategy;         // 0=vision-dominant, 1=olfaction-dominant
    
    // TAG-DERIVED HINTS
    bool has_flippers;                   // "flipper" tag detected
    bool has_wings;                      // "wing" tag for bats
    bool has_hooves;                     // "hoof" tag detected
    bool has_claws_retractile;           // "retractile" + "claw" tags
    bool has_prehensile_tail;            // "prehensile" + "tail" tags
    bool has_cheek_pouches;              // "pouch" tag on head
    bool has_horns_antlers;              // "horn" or "antler" tags
    bool has_blubber;                    // "blubber" tag (marine mammals)
    bool has_fur_dense;                  // "dense" + "fur" tags (insulation)
    
    // INFERRED SPECIALIZATIONS
    bool is_graviportal;                 // >1000 kg with columnar limbs
    bool is_glider;                      // Patagium detected from skin tags
    bool is_bipedal;                     // Hindlimb >> forelimb + tail balance
    
    // CONFIDENCE ASSESSMENTS
    float overall_confidence;            // Weighted average across predictions
};

struct MammalBehavioralProfile {
    // LOCOMOTION CHARACTERISTICS
    LocomotorMode primary_locomotion;
    std::optional<LocomotorMode> secondary_locomotion;  // Multi-modal species
    
    float max_sustainable_speed_BLs;     // Body lengths per second
    float max_sprint_speed_BLs;          // Short burst maximum (Garland, 1983)
    float preferred_speed_BLs;           // Optimal cost of transport speed
    float acceleration_capability;       // 0=slow, 1=explosive (m/s²)
    float maneuverability;               // 0=straight line, 1=agile turner
    float climbing_ability;              // 0=cannot climb, 1=vertical surfaces
    float jumping_ability;               // 0=no jump, 1=specialized (max height in BL)
    float swimming_ability;              // 0=sink, 1=aquatic specialist
    float digging_capability;            // 0=surface only, 1=extensive burrows
    
    // GAIT CHARACTERISTICS
    GaitType preferred_gait;             // At cruising speed
    float walk_trot_froude;              // Froude number for gait transition (~0.35)
    float trot_gallop_froude;            // Froude number for gait transition (~2-3)
    float duty_factor_walk;              // Fraction of stride in ground contact (>0.5)
    float duty_factor_trot;              // Typically ~0.4-0.5
    float stride_frequency_hz;           // At preferred speed (scales as M⁻⁰·¹⁴)
    float stride_length_BL;              // Stride / body length (typically 1-2)
    bool uses_gallop_rotary;             // Cheetah-style vs transverse (horse)
    bool uses_ricochetal_locomotion;     // Bipedal hopping like kangaroos
    
    // POSTURAL CHARACTERISTICS
    float limb_angle_degrees;            // 0=sprawling, 90=erect (Biewener, 1989)
    float spinal_flexion_range;          // Galloping flexibility (radians)
    bool uses_elastic_storage;           // Tendon springs for efficiency
    
    // THERMOREGULATION
    ThermalStrategy thermal_strategy;
    float thermal_neutral_zone_width;    // °C range where BMR unchanged
    float lower_critical_temp_C;         // Below this, increase metabolism
    float upper_critical_temp_C;         // Above this, active cooling required
    float insulation_quality;            // 0=poor, 1=excellent (fur/blubber)
    float surface_heat_loss_rate;        // W/m² at standard temp differential
    
    // METABOLIC CHARACTERISTICS
    float basal_metabolic_rate_W;        // Watts (Kleiber, 1947: ~3.5 M⁰·⁷⁵)
    float field_metabolic_rate_W;        // Active metabolism (Nagy et al., 1999)
    float aerobic_scope;                 // FMR / BMR (typically 2-10×)
    float cost_of_transport;             // J/(kg·m) (scales as M⁻⁰·³²)
    float endurance_factor;              // 0=burst only, 1=marathon capability
    
    // SOCIAL BEHAVIOR
    float gregariousness;                // 0=solitary, 1=obligate social
    float optimal_group_size;            // Mean group size in natural conditions
    float territoriality;                // 0=non-territorial, 1=strict territories
    float territorial_range_m;           // Defended area radius
    float dominance_hierarchy_strength;  // 0=egalitarian, 1=strict hierarchy
    
    // ACTIVITY PATTERNS
    float diurnal_preference;            // 0=nocturnal, 0.5=crepuscular, 1=diurnal
    float activity_level;                // 0=sedentary, 1=constantly active
    bool hibernates;                     // True torpor (Geiser, 2004)
    bool estivates;                      // Summer dormancy
    float daily_active_hours;            // Hours per day active
    
    // FEEDING ECOLOGY
    float predatory_capability;          // 0=herbivore, 1=apex predator
    float diet_specialization;           // 0=generalist, 1=specialist
    float foraging_range_km;             // Daily movement for food
    bool caches_food;                    // Food storage behavior
    
    // HABITAT PREFERENCES
    float canopy_preference;             // 0=ground, 1=treetop
    float water_dependency;              // 0=arid tolerant, 1=must stay near water
    float shelter_requirement;           // 0=open habitat, 1=needs complex cover
    float altitude_tolerance_m;          // Maximum elevation
    
    // REPRODUCTIVE CONSTRAINTS (affects behavior periodically)
    float breeding_seasonality;          // 0=continuous, 1=strict season
    float gestation_period_days;         // Pregnancy duration (scales with mass)
    float offspring_precocity;           // 0=altricial, 1=precocial
    
    // COMPONENT SELECTION HINTS
    AI_Components recommended_ai_component;
    Animation_Components recommended_animation;
    float simulation_priority;           // 0=background, 1=hero character
};

struct EnvironmentalLimits {
    static EnvironmentalLimits Factory(
        const MammalTonTonInputs& inputs, 
        const MorphologicalAnalysis& analysis);
    
    // LOCOMOTOR CONSTRAINTS
    float min_substrate_friction;        // Below this, cannot generate traction
    float max_slope_angle_deg;           // Steepest climbable slope
    float max_jump_height_m;             // Vertical jump capability
    float max_jump_distance_m;           // Horizontal jump capability
    float min_turning_radius_m;          // Minimum turn at speed
    float water_crossing_ability;        // 0=cannot swim, 1=crosses oceans
    
    // THERMAL CONSTRAINTS
    float min_ambient_temp_C;            // Lethal cold without shelter
    float max_ambient_temp_C;            // Lethal heat
    float max_sustained_activity_temp_C; // Cannot sustain exercise above this
    float metabolic_water_production;    // ml water/MJ energy (desert adaptation)
    
    // RESPIRATORY CONSTRAINTS
    float min_oxygen_partial_pressure;   // Pa (altitude limit)
    float max_sustained_speed_mps;       // Aerobic limit (Brett, 1964 analogue)
    float max_burst_duration_s;          // Anaerobic capacity
    
    // STRUCTURAL CONSTRAINTS
    float bone_stress_safety_factor;     // Typically 2-4 (Biewener, 1990)
    float max_impact_force_N;            // Landing/collision tolerance
    float burrow_soil_hardness_limit;    // Pa (fossorial species only)
    float climbing_grip_force_N;         // Maximum claw/pad adhesion
    
    // ENERGETIC CONSTRAINTS
    float starvation_time_days;          // Survival without food (fat reserves)
    float dehydration_time_days;         // Survival without water
    float max_daily_energy_budget_MJ;    // Sustainable daily expenditure
    float min_food_density_kg_km2;       // Minimum prey/forage density needed
};

struct LocomotionAnimationParameters {
    static LocomotionAnimationParameters Factory(
        const MammalTonTonInputs& inputs,
        const MorphologicalAnalysis& analysis,
        const MammalBehavioralProfile& behavior);
    
    // WALKING GAIT (Hildebrand, 1968)
    float walk_frequency_hz;             // Stride frequency (scales M⁻⁰·¹⁴)
    float walk_stride_length_m;          // Typically 1.5-2.0 BL
    float walk_duty_factor;              // >0.5 (Alexander & Jayes, 1983)
    float walk_lateral_sequence;         // Gait pattern descriptor
    
    // TROTTING GAIT
    float trot_frequency_hz;             // Higher than walk
    float trot_stride_length_m;          // Longer than walk
    float trot_duty_factor;              // ~0.4-0.5, aerial phase appears
    float trot_suspension_phase;         // Duration of aerial phase (s)
    
    // GALLOPING GAIT (Hildebrand, 1977)
    float gallop_frequency_hz;           // Maximum stride frequency
    float gallop_stride_length_m;        // Maximum extension
    float gallop_duty_factor;            // <0.4, extended flight phase
    float gallop_flexion_extension;      // Spinal contribution (radians)
    bool gallop_rotary_vs_transverse;    // Cheetah vs horse style
    
    // HOPPING/SALTATION (Alexander, 1974)
    std::optional<float> hop_frequency_hz;        // For saltatorial species
    std::optional<float> hop_ground_contact_time; // Brief contact
    std::optional<float> hop_aerial_time;         // Long flight phase
    std::optional<float> hop_energy_recovery;     // Elastic return (0-0.7)
    
    // CLIMBING KINEMATICS (Cartmill, 1985)
    std::optional<float> climb_speed_BLs;         // Vertical climbing rate
    std::optional<float> climb_stride_frequency;  // Steps per second
    std::optional<float> descent_speed_ratio;     // Descent/ascent speed
    std::optional<bool> can_descend_head_first;   // Ankle rotation ability
    
    // SWIMMING KINEMATICS (Fish & Baudinette, 1999)
    std::optional<float> swim_speed_mps;          // For aquatic/semi-aquatic
    std::optional<float> paddle_frequency_hz;     // Limb strokes
    std::optional<float> dive_duration_s;         // Breath-hold capacity
    std::optional<float> dive_depth_m;            // Maximum depth
    
    // BODY ARTICULATION
    float neck_flexibility_deg;          // Head movement range
    float torso_lateral_flex_deg;        // Side bending (turning)
    float torso_dorsoventral_flex_deg;   // Spinal flexion (galloping)
    float tail_articulation_segments;    // Tail flexibility
    
    // LIMB KINEMATICS
    float forelimb_protraction_deg;      // Forward reach angle
    float forelimb_retraction_deg;       // Backward push angle
    float hindlimb_protraction_deg;      // Forward swing
    float hindlimb_retraction_deg;       // Power stroke
    float elbow_flexion_range_deg;       // Joint ROM
    float knee_flexion_range_deg;        // Joint ROM
    
    // FOOT/PAW ARTICULATION
    float digit_splay_deg;               // Toe spreading (grip)
    float wrist_ankle_flexibility_deg;   // Joint articulation
    std::optional<float> claw_extension_mm; // For retractile claws
    
    // TURNING DYNAMICS
    float min_turn_radius_BL;            // Minimum turning circle
    float turn_initiation_time_s;        // Reaction time
    float pivot_turn_capability;         // 0=wide turns, 1=pirouette
    
    // GAIT TRANSITION PARAMETERS (Alexander & Jayes, 1983)
    float froude_walk_trot;              // ~0.35 for most mammals
    float froude_trot_gallop;            // ~2.0-3.0 typical range
    float hysteresis_bandwidth;          // Transition overlap zone
    
    // ENVIRONMENTAL SCALING
    float gravity_scaling_exponent;      // How performance scales with g
    float friction_requirement;          // Minimum μ for gait
    float substrate_stiffness_optimal;   // Preferred ground compliance
};

} // namespace Mammalia

#endif // METRICS_MAMMALIA_H
