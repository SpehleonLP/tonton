#ifndef METRICS_BIRDSTROUHAL_H
#define METRICS_BIRDSTROUHAL_H
#include "gltf/metrics_inputsbase.h"
#include <glm/vec3.hpp>


namespace Onothopter
{

struct OrnithopterInput : public Metrics::InputsBase 
{
// {root, tip} pairs
	std::vector<std::pair<int, int>> wings;

    // Environment (actual units)
    float gravity_m_s2 = 9.81f;
    float air_density_kg_m3 = 1.225f;
    float air_viscosity_pa_s = 1.81e-5f;
    float temperature_k = 288.15f;
    
    // Normalized artistic parameters (0-1)
    float average_density = 0.5;		  // lerps to 700 kg / m^3 -> 900 kg / m^3
    float bone_quality = 0.5f;           // 0=heavy/weak, 1=optimal hollow/strong
    float muscle_quality = 0.5f;         // 0=weak muscle, 1=peak biological performance
    float feather_quality = 0.5f;        // 0=poor aerodynamics, 1=optimal feathers
    float metabolic_efficiency = 0.5f;   // 0=poor ATP production, 1=optimal metabolism
    float endurance_vs_power = 0.5f;     // 0=sprint muscle, 1=endurance muscle
    float structure_vs_weight = 0.5f;    // 0=lightweight/fragile, 1=robust/heavy
    float hovering_adaptation = 0.5f;    // 0=pure forward flight, 1=hovering optimized
    float soaring_adaptation = 0.5f;     // 0=flapping only, 1=thermal soaring optimized
    
    // Additional behavioral inputs (things geometry can't determine)
    float social_tendency = 0.5f;          // 0=solitary, 1=highly social
    float seasonal_behavior = 0.5f;        // 0=resident, 1=migratory  
    float activity_pattern = 0.5f;         // 0=diurnal, 1=nocturnal
    float tool_use_capability = 0.0f;      // 0=none, 1=advanced tool use
    
    // Coloration profile (from texture analysis)
    bool is_cryptic = false;
    bool is_aposematic = false;
    bool has_sexual_dimorphism = false;
    
    float calculateWingspan() const;
    float calculateWingArea() const;
    float estimateWingMass() const;
};


struct OrnithopterOutput {
    // Geometric properties
    float wingspan_m;
    float wing_area_m2;
    float aspect_ratio;
    float mean_chord_m;
    
    // Mass properties
    float total_mass_kg;
    float wing_mass_kg;
    float body_mass_kg;
    float muscle_mass_kg;
    
    // Inertial properties
    float wing_moment_inertia_kg_m2;
    
    // Flight performance
    float wingbeat_frequency_hz;
    float wingbeat_amplitude_rad;
    float wing_loading_n_m2;
    float required_power_w;
    float available_power_w;
    
    // Flight capability
    bool can_fly;
    float stall_speed_m_s;
    float cruise_speed_m_s;
    float max_flight_duration_s;
    float power_margin;  // available/required ratio
    
    // Performance envelope
    float reynolds_number;
    float strouhal_number;
    
    
    // Derived behavioral indicators
    float hovering_capability;    // 0-1, calculated from flight characteristics
    float soaring_capability;     // 0-1, calculated from flight characteristics
    float pursuit_capability;     // 0-1, calculated from flight characteristics
};

struct BehaviorProfile {
	static BehaviorProfile InferBehavior(const OrnithopterOutput& flight,
                             const OrnithopterInput& input);
                             
    // Aggression & Social Behavior (0-1 scales)
    float aggression_level;        // 0=passive, 1=highly aggressive
    float territoriality;          // 0=non-territorial, 1=fiercely territorial  
    float sociability;             // 0=solitary, 1=highly social/flocking
    float curiosity;               // 0=cautious, 1=investigative
    
    // Foraging Behavior
    float foraging_patience;       // 0=opportunistic, 1=sit-and-wait predator
    float ground_vs_aerial;        // 0=ground forager, 1=aerial hunter
    float prey_size_preference;    // 0=tiny prey, 1=large prey
    float food_specialization;     // 0=generalist, 1=specialist
    
    // Movement Patterns  
    float stealth_preference;      // 0=obvious movement, 1=cryptic/sneaky
    float formation_flying;        // 0=independent, 1=tight formations
    float thermal_seeking;         // 0=powered flight only, 1=soaring specialist
    float maneuverability_usage;   // 0=straight flight, 1=complex aerobatics
    
    // Threat Response
    float freeze_vs_flee;          // 0=immediate escape, 1=freeze first
    float mob_behavior;            // 0=individual response, 1=group mobbing
    float warning_calls;           // 0=silent, 1=loud alarm systems
    
    // Environmental Usage
    float canopy_vs_open;          // 0=open sky, 1=dense vegetation
    float height_preference;       // 0=ground level, 1=high altitude
    float weather_tolerance;       // 0=fair weather only, 1=all conditions
    
    // Derived AI Triggers
    bool is_apex_predator;
    bool uses_tools;               // from tags like "manipulator"
    bool is_aposematic;            // from coloration analysis
    bool is_nocturnal;             // from tags + eye size analysis
};

OrnithopterOutput CalculateOrnithopterPerformance(const OrnithopterInput& input);

}

#endif // METRICS_BIRDSTROUHAL_H
