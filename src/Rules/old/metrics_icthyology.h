#ifndef METRICS_ICTHYOLOGY_H
#define METRICS_ICTHYOLOGY_H
#include "metrics_inputsbase.h"
#include <glm/vec3.hpp>
#include <vector>

/* Tonton: 
 * 
 * Trait Oriented Normalization for Traits of Organismal Nature  
 * 
 */ 

namespace Ichthyology
{

enum AI_Components
{
	SchoolingBoids,
	AmbushPredator,
	ActiveHunter,
	TerritorialFish,
	GeneralistFish
};

enum Animation_Components
{
	ContinuousSwimmer,
	BurstSpecialist,
	ManeuverableSwimmer,
	StandardSwimmer,
};

struct FluidEnvironment {
    float density_Kg_m3{1000};       // kg/m³ (water=1000, air=1.2, liquid methane=422)
    float viscosity_Pas{0.00096};         // Pa·s (affects Reynolds numbers)
    float gravity_m_s2{9.81};           // m/s² (affects buoyancy requirements)
    float pressure_Pa{3068.167};          // Pa (affects swim bladder function)
    float temperature_K{298.15};       // K (affects metabolic rates)
};

struct IchthyologyTonTonInputs : public Metrics::InputsBase {
    FluidEnvironment environment;     // Water density, viscosity, gravity, etc.
    
    // BEHAVIORAL FINE-TUNING (0-1 sliders, auto-computed defaults)
    float social_adjustment = 0.5;       // ±20% from eye size prediction
    float aggression_adjustment = 0.5;   // ±20% from jaw geometry prediction  
    float activity_adjustment = 0.5;     // ±20% from body plan prediction
    
    // ENVIRONMENTAL CONTEXT (optional overrides)
    bool force_schooling = false;         // Override geometry-based social prediction
    bool force_solitary = false;         // Override geometry-based social prediction
    float preferred_depth_override = -1; // -1 = auto-detect from morphology
};

struct MorphologicalAnalysis {
	static MorphologicalAnalysis AnalyzeMorphology(const IchthyologyTonTonInputs& inputs);

	float liftCoefficient() const;
	float swimBladderVolume(float) const;

    // CORE GEOMETRIC MEASUREMENTS
    float fineness_ratio;              // Length / (mean of depth & width)
    float caudal_aspect_ratio;         // height² / surface area  
    float surface_area_to_volume;      // cm⁻¹
    float pectoral_fin_ratio;          // pectoral area / body area
    float eye_size_ratio;              // eye diameter / standard length
    float jaw_protrusion_capability;   // max jaw extension / head length
    glm::vec3 moment_of_inertia_normalized; // I / (mass × length²)
    float aabb_utilization;            // fish volume / bounding box volume
    
    // GEOMETRIC PREDICTIONS (with confidence)
    struct Prediction {
        float value;      // 0-1 normalized
        float confidence; // R² from research (0.6-0.9+)
		float final(float manual_adjustment, float weight = 0.2) const;
    };
    
    Prediction swimming_speed_class;     // 0=slow, 1=fast (AR formula, R²=0.85)
    Prediction metabolic_strategy;       // 0=low energy, 1=high energy (SA/V)
    Prediction visual_acuity;           // 0=poor vision, 1=excellent (eye size)
    Prediction social_tendency;         // 0=solitary, 1=schooling (eye + body)
    Prediction feeding_strategy;        // 0=filter, 0.5=generalist, 1=predator
    Prediction maneuverability;         // 0=straight line, 1=highly agile (inertia)
    Prediction activity_level;          // 0=sit-and-wait, 1=continuous (SA/V + fins)
    
    // TAG-DERIVED HINTS
    bool has_sucker_mouth;             // "sucker" tag found
    bool has_beak_mouth;               // "beak" tag found  
    bool has_large_eyes;               // "large" tag on eyes
    bool is_flatfish;                  // Both eyes on one side detected
    bool has_heterocercal_tail;        // Asymmetric tail detected
    
    // CONFIDENCE ASSESSMENTS
    float overall_confidence;          // Weighted average of all predictions
 //   std::vector<string> low_confidence_traits; // Traits needing manual review
  //  std::vector<string> tag_confirmations;     // Tags that confirmed geometric predictions
   // std::vector<string> tag_contradictions;    // Tags that contradict geometry
};

struct FishBehavioralProfile {
    // LOCOMOTION CHARACTERISTICS (for animation system)
    float swimming_speed_preference;    // 0=slow cruise, 1=fast cruise
    float burst_capability;            // 0=poor acceleration, 1=explosive starts
    float maneuverability;             // 0=straight line, 1=tight turns
    float endurance;                   // 0=short bursts only, 1=continuous swimming
    bool requires_constant_motion;      // True for sharks, fast pelagics
    
    // SOCIAL BEHAVIOR (for AI flocking system)  
    float schooling_tendency;          // 0=always solitary, 1=always schools
    float optimal_group_size;          // Number of individuals (1=solitary)
    float personal_space_radius;       // Body lengths of preferred spacing
    float leadership_tendency;         // 0=follower, 1=leads group movement
    
    // FEEDING & AGGRESSION (for AI decision-making)
    float predatory_capability;        // 0=herbivore/filter, 1=apex predator
    float ambush_vs_pursuit;          // 0=active hunting, 1=sit-and-wait
    float territorial_radius;          // Body lengths of defended area
    float aggression_level;            // 0=passive, 1=highly aggressive
    
    // HABITAT PREFERENCES (for AI environmental response)
    float depth_preference_min;        // Meters (0=surface only)
    float depth_preference_max;        // Meters  
    float flow_preference;             // 0=still water, 1=fast current
    float structure_dependency;        // 0=open water, 1=needs complex habitat
    
    // ACTIVITY PATTERNS (for AI scheduling)
    float diurnal_preference;          // 0=nocturnal, 0.5=crepuscular, 1=diurnal
    float activity_level;              // 0=mostly stationary, 1=constantly active
    float energy_conservation;         // 0=high metabolism, 1=low energy lifestyle
    
    // PERFORMANCE ENVELOPE (for physics constraints)
    float max_sustainable_speed;       // Body lengths per second
    float max_burst_speed;             // Body lengths per second (short duration)
    float min_turning_radius;          // Body lengths
    float max_acceleration;            // Body lengths per second²
    
    
    // COMPONENT SELECTION HINTS
    AI_Components recommended_ai_component;    // "SchoolingBoids", "AmbushPredator", etc.
    Animation_Components recommended_animation;       // "ContinuousSwimmer", "BurstSpecialist", etc.
    float simulation_priority;         // 0=background, 1=hero character
};

// Things we approximate but can't measure precisely:
struct UnmeasurableTraits {
    float red_white_muscle_ratio;      // Estimate from fin ratios + body shape
    float bone_density;                // Estimate from depth preferences
    float scale_roughness;             // Assume smooth unless tagged
    float lateral_line_sensitivity;    // Estimate from eye size inverse
    float swim_bladder_volume;         // Estimate from tail asymmetry + depth
    float gill_surface_area;           // Estimate from activity level prediction
};

struct EnvironmentalLimits {
	static EnvironmentalLimits Factory(IchthyologyTonTonInputs const& inputs, MorphologicalAnalysis const& );

    float min_fluid_density;          // Below this, can't maintain buoyancy
    float max_fluid_viscosity;        // Above this, can't generate enough thrust
    float optimal_reynolds_range;     // Where this morphology works best
    float pressure_crush_depth;       // Structural failure point
    float minimum_swim_speed;         // Stall speed in this fluid
    float min_speed_for_lift;
    float maximum_sustainable_speed; // Metabolic limit
};

struct SwimmingAnimationParameters {
    // BASIC SWIMMING GAIT
    float base_frequency;              // Tail beats per second
    float amplitude_ratio;             // Tail amplitude / body length
    float wavelength_ratio;            // Body wave length / body length  
    float wave_speed_ratio;            // Wave speed / forward speed
    
    // BODY UNDULATION
    float body_flexibility;            // 0=rigid like tuna, 1=flexible like eel
    float anterior_stiffness;          // Head region stiffness
    float posterior_flexibility;       // Tail region flexibility
    std::vector<float> segment_amplitudes;  // Per-segment amplitude multipliers
    
    // FIN MOTION
    float pectoral_frequency_ratio;    // Pectoral freq / tail freq
    float pectoral_amplitude;          // Pectoral fin stroke amplitude
    float dorsal_undulation;           // Dorsal fin wave participation
    
    // GAIT TRANSITIONS
    float cruise_to_burst_threshold;   // Speed ratio for gait change
    float burst_frequency_multiplier;  // Freq increase in burst mode
    float burst_amplitude_multiplier;  // Amplitude change in burst
    
    // TURNING DYNAMICS
    float c_start_duration;            // Duration of C-start escape (seconds)
    float max_body_curvature;          // Maximum C-start bend (radians)
    float turn_wave_frequency;         // Frequency during turning
    
    // SPECIALIZED BEHAVIORS
    bool uses_pectoral_propulsion;     // Wrasse-like swimming
    bool uses_dorsal_anal_undulation;  // Knife fish swimming
    float hovering_capability;         // 0=can't hover, 1=perfect station keeping
    
        // VERTICAL MANEUVERING
    float depth_change_capability;     // 0=surface only, 1=full water column
    float buoyancy_control_speed;      // How quickly can adjust depth (BL/s vertical)
    float swim_bladder_response_time;  // Seconds to adjust buoyancy (0 if no bladder)
    float pitch_angle_range;           // Max pitch up/down (radians)
    float vertical_thrust_efficiency;  // How well pectorals/body generate vertical force
};

};

#endif // METRICS_ICTHYOLOGY_H
