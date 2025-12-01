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
    density_kg_m3     fluidDensity_Kg_m3{1.225f};     // kg/m³ (water=1000, air=1.2, liquid methane=422)
    viscosity_Pa_s    fluidViscosity_Pa_s{1.81e-5f};  // Pa·s (affects Reynolds numbers)
    pressure_Pa       fluidPressure_Pa{3068.167};     // Pa (affects swim bladder function)
    acceleration_m_s2 gravity_m_s2{9.81};             // m/s² (affects buoyancy requirements)
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
	
	struct Mana
	{
		// WATER - Fluidity and adaptability (Aristotle: takes shape of container, flows and changes)
		// Affects: turning radius, gait transitions, control authority, balance recovery
		// Higher water = more agile, smoother state changes, tighter maneuvers
		float water = 0.0;
		
		// FIRE - Internal heat and vital energy (Aristotle: rises upward, source of motion and life)
		// Affects: metabolic efficiency, power output, burst speed/strength
		// Higher fire = stronger muscles, more energetic, higher heat budget (breath weapons)
		float fire = 0.0;
		
		// EARTH - Solidity and resistance (Aristotle: falls downward, seeks center, heavy and stable)
		// Affects: structural integrity, cross-sectional area multiplier, bone/muscle strength
		// Higher earth = denser/stronger tissues, can support impossible proportions
		float earth = 0.0;
		
		// AIR - Medium cooperation (Aristotle: medium actively pushes objects along their path)
		// Affects: thrust efficiency in any medium (air/water), lift coefficient, reduced drag
		// Higher air = environment helps propulsion, easier flight/swimming
		float air = 0.0;
		
		// AETHER - The celestial element (Aristotle: unchanging, perfect, defies earthly physics)
		// Affects: weight_N directly, propagates through all weight-dependent calculations
		// Higher aether = lighter/floatier, reduced gravity coupling, more "magical"
		float aether = 0.0;
		
		// SHADOW - Surface affinity (shadows stick to surfaces, conform to terrain)
		// Affects: surface adhesion, grip strength, friction coefficients, wall-climbing
		// Higher shadow = better grip, can stick to vertical/inverted surfaces
		float shadow = 0.0;
	} mana;
	
	struct StyleCorrections
	{
		// TODO: art style corrections for stylized models
		// - toon models have oversized eyes/heads
		// - anime models have elongated limbs
		// - chibi models are weird
		// Problem: "stylization" is too broad, need better taxonomy
		// For now: assume realistic proportions
		// PRs welcome lol
	} style;
		
//	inline glm::vec3 position(int index) const { return skinnedMesh? skinnedMesh->skin->position[index] * behavior.scale : glm::vec3(0); };
	length_b_to_m scale = length_b_to_m(1.f);
	
	inline density_kg_m3 body_density() const { return density_kg_m3(glm::mix(700.0, 1050.0, average_density)); } 
	mass_kg body_mass_kg() const;
	force_N body_weight_N() const;
	area_m2 cross_sectional_area_m2() const;
	glm::mat3 inertia_restPose() const;
	
	inline auto surface_area_scale() const { return scale * scale; }
	inline auto cross_section_area_scale() const { return (scale * scale) * std::exp2(mana.earth); }
	inline auto volume_scale() const { return scale * scale * scale; }
	inline auto inertia_scale() const { return (scale * scale * scale) * (scale * scale); }
};

};


#endif // TONTON_INPUT_H
