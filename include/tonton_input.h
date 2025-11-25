#ifndef TONTON_INPUT_H
#define TONTON_INPUT_H
#include "tonton_units.hpp"
#include "tonton_counted_ptr.hpp"
#include <glm/common.hpp>
#include <glm/fwd.hpp>

namespace TonTon
{

struct Builder;
struct Environment
{
    density_kg_m3     fluidDensity_Kg_m3{1.225f};       // kg/m³ (water=1000, air=1.2, liquid methane=422)
    viscosity_Pa_s    fluidViscosity_Pa_s{1.81e-5f};   // Pa·s (affects Reynolds numbers)
    acceleration_m_s2 gravity_m_s2{9.81};             // m/s² (affects buoyancy requirements)
    pressure_Pa       pressure_Pa{3068.167};          // Pa (affects swim bladder function)
    temp_K            temperature_K{298.15};          // K (affects metabolic rates)
};

struct Input 
{
	counted_ptr<const Builder> builder;
	Environment environment;
	
	struct Behavior
	{
		float coloration{}; // -1 -> camoflauge, +1 -> aposematism
		
		// BEHAVIORAL FINE-TUNING (0-1 sliders, auto-computed defaults)
		float aggression_adjustment = 0.5;   // ±20% from jaw geometry prediction  
		float activity_adjustment = 0.5;     // ±20% from body plan prediction
		float endurance_vs_power = 0.5f;     // 0=sprint muscle, 1=endurance muscle
		float risk_tolerance = 0.5f;
		float social_tendency = 0.5f;        // 0=solitary, 1=highly social	
		
		// Additional behavioral inputs (things geometry can't determine)
		float seasonal_behavior = 0.5f;        // 0=resident, 1=migratory  
		float activity_pattern = 0.5f;         // 0=diurnal, 1=nocturnal
		float adaptability = 0.0f;      // 0=none, 1=advanced tool use	
	
	//	inline float area_scale() const { return std::sqrt(scale.x * scale.y) * std::sqrt(scale.y * scale.z) * std::sqrt(scale.z * scale.x); }
	} behavior;
	
	// Normalized artistic parameters (0-1)
	float average_density = 0.5;		  // lerps to 700 kg / m^3 -> 1050 kg / m^3
	float structure_vs_weight = 0.5f;    // 0=lightweight/fragile, 1=robust/heavy
	float muscle_quality = 0.5f;         // 0=weak muscle, 1=peak biological performance
	float feather_quality = 0.5f;        // 0=poor aerodynamics, 1=optimal feathers
	float metabolic_efficiency = 0.5f;   // 0=poor ATP production, 1=optimal metabolism
	float stability_vs_speed = 0.5f;     // 0=pure forward, 1=hovering optimized
	float activity_level = 0.5f;         // e.g. 0=flapping, 1=thermal soaring
	float scaling_strategy = 0.5f;       // how aggressively to combat size penalties
	float climbing_ability = 0.5;        // 0=none, 1=vertical surfaces
	
//	inline glm::vec3 position(int index) const { return skinnedMesh? skinnedMesh->skin->position[index] * behavior.scale : glm::vec3(0); };
	length_s scale = length_s(1.f);
	
	inline density_kg_m3 body_density() const { return density_kg_m3(glm::mix(700.0, 1050.0, average_density)); } 
	mass_kg body_mass_kg() const;
	force_N body_weight_N() const;
	area_m2 cross_sectional_area_m2() const;
	glm::mat3 inertia_restPose() const;
	
	inline area_s area_scale() const { return scale * scale; }
	inline volume_s volume_scale() const { return scale * scale * scale; }
	inline length5_s inertia_scale() const { return (scale * scale * scale) * (scale * scale); }
};

};


#endif // TONTON_INPUT_H
