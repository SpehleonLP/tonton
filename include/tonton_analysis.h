#ifndef TONTON_ANALYSIS_H
#define TONTON_ANALYSIS_H
#include "tonton_units.hpp"
#include "tonton_counted_ptr.hpp"
#include "tonton_shared_array.hpp"
#include "tonton_optional.hpp"
#include "tonton_wordlist.h"
#include "tonton_tensors.hpp"
#include <atomic>
#include <glm/vec3.hpp>
#include <glm/mat3x3.hpp>
#include <optional>
#include <vector>
#include <cstdint>
#include <string>
#include <array>

namespace TonTon {

struct Input;

// support
struct Analysis_Chain
{
	// stop ik chain here
	uint16_t root{};     
	// ik target
	uint16_t tip{};
	
	int noJoints{};
	
	length_m stretched_length_m{};
	length_m rest_length_m{};
};

struct Analysis_Appendage : public Analysis_Chain
{
// everything in the group is symmetric about some bone. so if the gait group has 2 memebers its bilaterally symmetrical.
// common ancestor of the group
	int16_t common_ancestor{}; 
	uint16_t gait_group{}; 
	uint16_t id{}; // index in appendage array in builder.
	float   phase_offset{};
};

// Venom delivery
struct Analysis_Venom {
	int32_t delivery_joint{};       // fang, stinger, spine
	float injection_volume_ml{};
	float strike_speed_m_s{};
};
		
// ============================================================================
// MANIPULATION - limb end point: trunks, tentacles, hands
// ============================================================================
struct Analysis_Manipulator : public Analysis_Appendage {
	
	SemanticFlags subtree_flags{}; // TENTACLE, DIGIT, etc.
	
	force_N max_lift_force_N{};
	force_N max_grip_force_N{};
	force_N max_adhesion_force_N{};
	
	area_m2 contact_area_m2{};
	glm::vec3 surface_normal{}; // sign undefined
	
	bool has_suckers : 1;
	bool has_setae : 1;
	bool has_claws : 1;
	bool has_thumb : 1;
	bool has_wet_grip : 1; // frog!	
	
	bool has_friction_pads() const
	{
		return
				has_suckers == false
			&& has_suckers == false
			&& has_setae == false
			&& has_wet_grip == false
			&& has_thumb == true;
	}
};

// ============================================================================
// CORE PHYSICAL PROPERTIES (always present)
// ============================================================================
struct Analysis_Physical {
	mass_kg body_mass_kg{};
	length_m body_length_m{};
	volume_m3 body_volume_m3{};
	length_m tail_length_m{};
	
	auto svl_m() const { return body_length_m - tail_length_m; }
	
	// Body plan characteristics
	area_m2 surface_area_m2{};
	area_m2 cross_sectional_area_m2{};
	
	length_m cross_sectional_diameter_m() const { return sqrt(cross_sectional_area_m2 / M_PI)*2.0; }
	float fineness_ratio() const { auto dia = cross_sectional_diameter_m(); return (dia != 0? body_length_m / dia : 0.f); };  // length / mean_diameter
	
	int16_t		spine_root{};
	bool		upright{};
	CladeFlags  clade{CladeFlags::NONE};
	NicheFlags  niche{NicheFlags::NONE};
	
	std::array<float, 6>  covariance_restPose{1.f, 1.f, 1.f, 0.f, 0.f, 0.f};
	inline glm::mat3 inertia_restPose() const
	{
		auto & C = covariance_restPose;
		TonTon::SecondMomentTensor smt{ glm::dmat3{
			C[0], C[3], C[4],
			C[3], C[1], C[5],
			C[4], C[5], C[2] } };
		return glm::mat3(TonTon::ToInertia(smt).I);
	}
	
};

// ============================================================================
// METABOLIC & ENERGETICS (always present)
// ============================================================================
struct Analysis_Metabolic {
	power_W basal_rate_W{};
	power_W max_rate_W{};
	float aerobic_scope() const { return max_rate_W / basal_rate_W; };              // max/basal ratio

	mass_kg muscle_mass_kg{};
	power_W available_muscle_power_W{};   // ~400 W/kg theoretical max

	// Thermal strategy
	temp_K body_temperature_K{-1};
	temp_K thermal_neutral_zone_min_K{-1};
	temp_K thermal_neutral_zone_max_K{-1};
	
	bool is_endotherm() const { return float(body_temperature_K) > 0; } 
};


// Body undulation (for BCF swimmers)
struct Analysis_BodyWave : public Analysis_Chain {			
	float wavelength_ratio{};        // wavelength / body_length
	float amplitude_ratio{};         // tail_amplitude / body_length
	float body_flexibility{};        // 0=rigid (tuna), 1=eel-like

	struct Parabola
	{
		float constant{}, linear{}, geometric{};
	} segment_amplitude;
};
	
// ============================================================================
// TERRESTRIAL LOCOMOTION (optional)
// ============================================================================
struct Analysis_Terrestrial {
	immutable_array<Analysis_Manipulator> legs{};
	
	// 0 upright -> 1 sprawling
	float posture{};

	// Speed capabilities
	velocity_m_s max_sprint_speed_m_s{};
	velocity_m_s max_sustainable_speed_m_s{};  // aerobic limit
	velocity_m_s optimal_speed_m_s{};          // minimum cost of transport

	// Maneuverability
	length_m min_turning_radius_m{};
	acceleration_m_s2 max_acceleration_m_s2{};

	// Endurance limits (especially important for sprawling posture)
	time_s max_sprint_duration_s{-1};  // before exhaustion
	time_s recovery_time_s{-1};        // after sprint
};

// Serpentine locomotion (for SERPENTINE posture)
struct Analysis_Serpentine {
	enum class Mode {
		LATERAL_UNDULATION = 1 << 0,  // Primary terrestrial snake mode
		RECTILINEAR = 1 << 1,         // Slow, stealthy crawling
		SIDEWINDING = 1 << 2,         // Desert specialists
		CONCERTINA = 1 << 3          // Climbing/tunneling
	};
	Mode capable_modes{};

	// Friction properties (critical for snake locomotion)
	float forward_friction_coef{};       // Parallel to body
	float lateral_friction_coef{};       // Perpendicular to body
	float friction_anisotropy_ratio{};   // lateral/forward (1.5-3.0 typical)
	velocity_m_s lateral_undulation_speed_m_s{};
	
	// Lateral undulation parameters
	Analysis_BodyWave lateral_undulation{};

	struct Rectilinear
	{
		velocity_m_s speed_m_s{};
		freq_Hz frequency_Hz{};
	};

	struct SideWinding : public Rectilinear
	{
		float amplitude_ratio{};
		int32_t contact_points{};
	};

	struct Concertina 
	{
		velocity_m_s speed_m_s{};
		float compression_ratio{};
	};
	
	// Rectilinear parameters
	std::optional<Rectilinear> rectilinear{};
	std::optional<SideWinding> sidewinding{};
	std::optional<Concertina>  concertina{};
};

// ============================================================================
// AERIAL LOCOMOTION (optional)
// ============================================================================
struct Analysis_Aerial;
struct Analysis_TakeoffAnalysis {
    
    enum class TakeoffMode {
        VERTICAL_LAUNCH,        // Pure wing power (hummingbird, dragonfly)
        JUMP_LAUNCH,            // Leg jump + wings (most birds)
        RUNNING_TAKEOFF,        // Needs runway for speed (ducks, geese)
        CLIFF_LAUNCH,           // Must drop to gain speed (albatross, large pterosaurs)
        ASSISTED_LAUNCH,        // Needs external help (some bats drop from roost)
        IMPOSSIBLE              // Cannot take off from level ground
    };
    
    TakeoffMode mode{TakeoffMode::IMPOSSIBLE};
    
    // Takeoff performance metrics
    length_m takeoff_run_distance_m{-1};           // Required runway length (0 for vertical)
    velocity_m_s required_jump_velocity_m_s{0};        // Initial upward velocity needed from legs
    time_s time_to_flight_speed_s{-1};           // Time to reach min_flight_speed
    acceleration_m_s2 vertical_acceleration_m_s2{0};        // Net upward accel during takeoff
    
    // Force analysis
    force_N max_instantaneous_lift_N{0};          // Peak lift during power stroke
    force_N max_instantaneous_thrust_N{0};        // Peak thrust from wing acceleration
    force_N net_vertical_force_N{0};              // Total upward force available
    float force_margin_percent{0};                // Safety margin above weight
    
    // Efficiency factors
    cost_W_kg power_to_weight_W_kg{0};           // Critical for vertical launch
    float takeoff_power_fraction{0};            // Fraction of max power needed
    float ground_effect_bonus{1.0f};            // Lift bonus near surface (1.0-1.3)
    
    // Alternative strategies
    bool can_use_wind_assist{false};            // Takeoff into headwind
    bool can_use_water_taxi{false};             // Run on water surface (pelicans)
    bool requires_perch_drop{false};            // Must drop from elevated position
    
    // Physical constraints
    struct Constraints {
        bool wing_loading_ok{false};            // < 80 N/m² for ground takeoff
        bool power_loading_ok{false};           // > 50 W/kg minimum
        bool aspect_ratio_ok{false};            // Good for flapping vs gliding
        bool leg_strength_ok{false};            // Can generate jump velocity
    } constraints{};
    
    // Confidence in analysis
    float confidence{0.5f};                     // 0-1, based on available data
    
    
    // Core force estimations
    static force_N EstimateMaxLift(const Analysis_Aerial& aerial, 
                                mass_kg body_mass_kg,
                                density_kg_m3 air_density);
    
    static force_N EstimateMaxThrust(const Analysis_Aerial& aerial,
                                  mass_kg body_mass_kg,
                                  density_kg_m3 air_density);
    
    // Ground effect modeling
    static float GroundEffectBonus(length_m wing_span_m, length_m height_above_ground_m);
    
    // Required jump velocity from legs
    static velocity_m_s RequiredJumpVelocity(const Analysis_Aerial& aerial,
                                     mass_kg body_mass_kg,
                                     force_N max_lift_N,
                                     acceleration_m_s2 gravity_m_s2);
    
    // Running takeoff distance
	static length_m RunwayDistance(const Analysis_Aerial& aerial,
								   const Analysis_Terrestrial* terrestrial,
								   mass_kg body_mass_kg, density_kg_m3 air_density);
    
    // Classification
 //   static TakeoffMode ClassifyMode(const Output& output, 
  //                                 const TakeoffAnalysis& analysis);
};

struct Analysis_Aerial {
	struct Wing : public Analysis_Appendage {				
		length_m span_m{};
		area_m2  wing_area_m2{};
		length_m chord_m{};
		// span is half span b/c only one wing.
		// so span² * 2² / (area*2)
		float aspect_ratio() const { return (span_m*span_m / wing_area_m2) * 2.0; };   // span² / area

		// Animation parameters
		angle_rad beat_amplitude_rad{};    // stroke angle
		angle_rad stroke_plane_angle_rad{}; // relative to body
		
		mass_kg wing_mass_kg{};
		inertia_kgm2 wing_inertia_kgm2{};
		
		
		velocity_m_s wing_tip_velocity(freq_Hz wingbeat_frequency_Hz) const { 
			return (3.14159265 * float(beat_amplitude_rad)) * (wingbeat_frequency_Hz * span_m); };
	};
	immutable_array<Wing> wings{};

	length_m wing_span_m{};
	area_m2 wing_area_m2{};

	// Performance envelope
	freq_Hz wingbeat_frequency_Hz{};
	velocity_m_s min_flight_speed_m_s{};       // stall speed
	velocity_m_s cruise_speed_m_s{};           // optimal
	velocity_m_s max_flight_speed_m_s{};

	pressure_Pa wing_loading_N_m2{};          // weight / wing_area
	cost_W_N power_loading_W_N{};          // power / weight

	// Specialized capabilities
	length_m max_altitude_m{};             // power/oxygen limit
	time_s max_flight_duration_s{};      // endurance limit

	// Maneuverability
	length_m min_turning_radius_m{};
	
	// computed with rest pose inertia. 
	omega_rad_s max_roll_rate_rad_s{};
	omega_rad_s max_pitch_rate_rad_s{}; 
	omega_rad_s max_yaw_rate_rad_s{}; 
	
	power_W flapping_power_W{};            // Metabolic power for forward flight
	power_W hovering_power_W{};            // Metabolic power for hovering
			
	// Relative efficiency (0-1, higher = better strategy for this animal)
	float flapping_efficiency{};
	float hovering_efficiency{};
	bool can_hover{};

	// Flight capability diagnostics (based on muscle power budget)
	bool can_sustain_level_flight{};  // Sufficient power for sustained flight
	bool can_slow_descent{};          // Can flutter to slow fall, but not gain altitude
			
	// Steady glide: Lift = Weight
	float gliding_CL(force_N weight_N, velocity_m_s speed_m_s, density_kg_m3 air_density) const;
	float flapping_CL_effective(force_N weight_N,
								velocity_m_s forward_speed_m_s, 
								freq_Hz wingbeat_freq_Hz,
								angle_rad beat_amplitude_rad,
								density_kg_m3 air_density) const;
	load_N_m2 hovering_disk_loading_N_m2(force_N weight_N) const;		
	power_W hovering_power_ideal_W(force_N weight_N, density_kg_m3 air_density) const;

	Analysis_TakeoffAnalysis takeoff;
};

// ============================================================================
// AQUATIC LOCOMOTION (optional)
// ============================================================================
struct Analysis_Aquatic  {
	enum class PropulsionMode : uint8_t {
		BODY_CAUDAL_FIN,      // BCF - fish tail
		MEDIAN_PAIRED_FIN,    // MPF - pectoral/dorsal fins
		JET_PROPULSION,       // Squid/octopus
		PADDLE_LIMBS,         // Turtles/seals
		DORSOVENTRAL_FLUKES   // Whales/dolphins
	};

	struct Fin  : public Analysis_Appendage {
		length_m chord_m{};
		area_m2 fin_area_m2{};

		// Animation - for undulating fins
		freq_Hz beat_frequency_Hz{0};
		angle_rad beat_amplitude_rad{0};
		float wave_speed_ratio{0};  // wave_speed / swim_speed
		
		glm::vec3 normal_vector{0, 0, 0}; // sign is not defined. 

		SemanticFlags type{};  // FIN, WING (for flying fish), LIMB
	};
	immutable_array<Fin> propulsors{};

	std::optional<Analysis_BodyWave> body_wave{};

	// Speed capabilities
	velocity_m_s cruise_speed_m_s{};
	velocity_m_s burst_speed_m_s{};
	velocity_m_s min_swim_speed_m_s{};         // to maintain lift/control

	// Hydrodynamics
	lift_N_per_m lift_per_meter_swam_N{};      // For negatively buoyant animals
	velocity_m_s sink_rate_m_s{};              // How fast they sink when stationary
	float reynolds_number{};            // Swimming regime indicator
	float drag_coefficient{};           // Cd for streamlining assessment
	length_m tail_amplitude_m{};           // Actual tail oscillation amplitude
	freq_Hz beat_frequency_Hz{};          // Tail beat frequency

	// Buoyancy control
	density_kg_m3 neutral_buoyancy_density_kg_m3{};
	time_s swim_bladder_adjust_time_s{-1};
	PropulsionMode primary_mode{};
	bool has_swim_bladder{};

	// Maneuverability
	bool can_hover{};                   // station-keeping
	bool requires_constant_motion{};    // sharks, tuna
	length_m min_turning_radius_m{};

	// Depth capabilities
	length_m preferred_depth_min_m{};
	length_m preferred_depth_max_m{};
	length_m crush_depth_m{-1};


	// C-start escape response
	struct CStartResponse {
		time_s duration_s{};
		angle_rad max_body_curvature_rad{};
		acceleration_m_s2 c_acceleration_m_s2{};
	};
	std::optional<CStartResponse> c_start{};
	
	struct JetPropulsion {
		freq_Hz mantle_contraction_frequency_Hz{};
		volume_m3 jet_pulse_volume_m3{};
		velocity_m_s jet_velocity_m_s{};
		int32_t siphon_joint{};
		angle_rad siphon_articulation_range_rad{};
	};
	std::optional<JetPropulsion> jet_propulsion{};
};

// ============================================================================
// CLIMBING (optional)
// ============================================================================
struct Analysis_Climbing {
	immutable_array<Analysis_Manipulator> limbs{};

	velocity_m_s max_climb_speed_m_s{};
	angle_rad max_climb_angle_rad{};        // from horizontal
	bool can_descend_head_first{};
	bool can_climb_inverted{};
	bool can_climb_smooth_wet_surfaces{};
	bool can_form_living_bridges{};
	bool requires_aquatic_environment{};

	// Substrate requirements
	float min_roughness_required{};     // for claw penetration
	bool requires_dry_surface{};        // for some adhesion modes
};


// ============================================================================
// BRACHIATION (optional) - arm-swinging locomotion (gibbons, spider monkeys)
// ============================================================================
struct Analysis_Brachiation {
	struct Arm  : public Analysis_Appendage {
		length_m reach_m{};
		force_N grip_strength_N{};
		velocity_m_s swing_speed_m_s{};
	};
	immutable_array<Arm> arms{};

	velocity_m_s max_swing_speed_m_s{};
	length_m max_gap_distance_m{};      // max distance between handholds
	length_m pendulum_length_m{};       // effective pendulum for energy

	// Alternating arm pattern
	freq_Hz swing_frequency_Hz{};
	float arm_phase_offset{};        // phase difference between arms (usually 0.5)
};

// ============================================================================
// JUMPING (optional)
// ============================================================================
struct Analysis_Jumping {
	enum class MechanismType {
		MUSCLE_DIRECT,      // Direct muscle power
		ELASTIC_CATAPULT,   // Tendon energy storage
		HYDRAULIC          // Hemolymph pressure (insects)
	};
	MechanismType mechanism{};

	length_m max_jump_height_m{};
	length_m max_jump_distance_m{};
	velocity_m_s takeoff_velocity_m_s{};
	angle_rad takeoff_angle_rad{};

	// Energy storage
	energy_J elastic_storage_J{0};
	float power_amplification_ratio{0};

	// Recovery
	time_s recovery_time_s{};            // between jumps
};

// ============================================================================
// SPECIALIZED BEHAVIORS
// ============================================================================
// Burrowing/fossorial
struct Analysis_Digging {
	enum class Method { SCRATCH, HEAD_LIFT, INCISOR, HUMERAL_ROTATION };
	Method method{};

	velocity_m_s max_dig_speed_m_s{};
	length_m tunnel_diameter_m{};
	force_N soil_force_N{};
};

// Constriction (snakes, tentacles)
struct Analysis_Constriction {
	pressure_Pa max_squeeze_pressure_Pa{};
	length_m coil_diameter_range_min_m{};
	length_m coil_diameter_range_max_m{};
};


struct  Analysis_Tail : public Analysis_Chain // or tips for branching
{       
	int     common_ancestor{};
	mass_kg tail_mass_kg{};
	area_m2 max_cross_section_m2{};
	area_m2 min_cross_section_m2{};

	// Animation
	freq_Hz natural_sway_frequency_Hz{};
	
	enum Flags : uint8_t
	{
	//	Balance = 1 << 0,		// in biology seems to be a catch all for "we don't know what this does"
	//	Propulsion = 1 << 2,   // not determinable at the point in time we compute the tail. 
		Grasping = 1 << 3,
	//	Display = 1 << 4,		// unsure how to detect right now
		Combat = 1 << 5,
	};

	Flags used_for{0};
	
	// Branching support (for mythological creatures!)
	immutable_array<Analysis_Tail> branches{};  // empty for single tail
    std::optional<Analysis_Venom> venom{}; // how?
};

// ============================================================================
// BEHAVIORAL PROFILE (always present)
// ============================================================================
struct Analysis_Behavior {
	// Derived AI hints
	enum class AIArchetype : uint8_t {
		SOLITARY_AMBUSH_HUNTER,
		PACK_COORDINATOR,
		SOCIAL_FORAGER,
		TERRITORIAL_DEFENDER,
		OPPORTUNISTIC_SCAVENGER,
		AERIAL_PREDATOR,
		SCHOOLING_PREY,
		APEX_PREDATOR
	};
	
	// Personality traits (0-1 scales)
	float aggression{};                 // 0=passive, 1=highly aggressive
	float social_tendency{};            // 0=solitary, 1=highly social
	float activity_level{};             // 0=sit-and-wait, 1=constantly active
	float curiosity{};                  // 0=cautious, 1=investigative
	float territoriality{};             // 0=nomadic, 1=fiercely territorial

	// Activity patterns
	float diurnal_preference{};         // 0=nocturnal, 0.5=crepuscular, 1=diurnal
	bool is_migratory{};

	AIArchetype suggested_archetype{};
	
	// Threat response
	bool uses_alarm_calls{};
	bool uses_mobbing_behavior{};
	float fight_vs_flight{};            // 0=flee immediately, 1=stand ground
	
	// Foraging strategy
	float ambush_vs_pursuit{};          // 0=active hunter, 1=sit-and-wait
	float prey_size_preference{};       // 0=tiny prey, 1=large prey relative to self
	float dietary_specialization{};     // 0=generalist, 1=specialist

	// Social behavior
	float optimal_group_size{-1};
	length_m personal_space_radius_m{-1};
	length_m territory_radius_m{-1};

	// Environmental preferences
	struct HabitatPreference {
		temp_K temperature_min_K{};
		temp_K temperature_max_K{};
		float humidity_preference{};    // 0=arid, 1=humid

		// Structural habitat
		float open_vs_dense{};          // 0=open terrain, 1=dense cover
		length_m canopy_height_preference_m{-1};
	};
	HabitatPreference habitat{};
};


// ============================================================================
// SENSORY SYSTEMS (always present, but may have low values)
// ============================================================================
struct Analysis_Vision {
	float acuity{};                 // 0=poor, 1=excellent
	angle_rad binocular_overlap{};      // 0=none, 1=full overlap
	length_m detection_range_m{};
	float centering{}; // how centered are the eyes? -1 or +1 indicates flatfish
	
	float motion_sensitivity_bonus{};
	length_m thermal_detection_range_m{};
	temp_K thermal_sensitivity_K{};
	
	bool has_color_vision{};
	bool has_night_vision{};
	bool has_thermal_vision{};
	bool has_uv_vision{};
};

struct Analysis_Hearing {
	float sensitivity{};            // 0=poor, 1=excellent
	freq_Hz frequency_range_Hz_min{};
	freq_Hz frequency_range_Hz_max{};
	length_m detection_range_m{};
	
	bool has_echolocation{};
	length_m echolocation_range_m{};
	angle_rad directional_accuracy_rad{};
	float substrate_vibration_sensitivity{};
};

struct Analysis_Olfaction {
	float sensitivity{};            // 0=poor, 1=excellent
	length_m detection_range_m{};
	int odor_discrimination_count{};
	float directional_acuity{};
};
		
template<template<typename> typename OPTIONAL>
struct Analysis_Sensory {
	OPTIONAL<Analysis_Vision> vision{};
	OPTIONAL<Analysis_Hearing> hearing{};
	OPTIONAL<Analysis_Olfaction> olfaction{};
	immutable_array<Analysis_Chain> antennae{};
};

// ============================================================================
// CONFIDENCE & WARNINGS
// ============================================================================
struct Analysis_Diagnostics {
	float overall_confidence{};         // 0-1, weighted average of predictions
	
	// Physics checks
	bool passes_power_budget_check{};   // total power usage <= available
	bool passes_mass_budget_check{};    // segments sum to total
	bool is_physically_plausible{};     // all major checks passed

	struct Warning {
		enum class Severity { INFO, CAUTION, ERROR };
		Severity level{};
		std::string message{};
	};
	std::vector<Warning> warnings{};

};
	
struct Output  {
using Sensory = Analysis_Sensory<optional>;

	static counted_ptr<const Output> Factory(Input const& in);
//	static counted_ptr<const Output> Scale(Output const& in, float scale);
	

	void AddRef() const { ++_refCount; };
	void Release() const { if(--_refCount == 0) delete this; }
	
	Analysis_Physical physical{};
	Analysis_Metabolic metabolic{};
	Analysis_Behavior  behavior{};
	Analysis_Sensory<optional> sensory{};
	
	struct Appendages {
		using Tail = Analysis_Tail;
		immutable_array<Analysis_Tail> tails{};  // usually 1, but kitsune has 9!
		immutable_array<Analysis_Manipulator> manipulation{};
	} appendages{};
	
	optional<Analysis_Terrestrial> terrestrial{};
	optional<Analysis_Serpentine> serpentine{};
	optional<Analysis_Aerial> aerial{};
	optional<Analysis_Aquatic> aquatic{};
	optional<Analysis_Climbing> climbing{};
	optional<Analysis_Brachiation> brachiation{};
	optional<Analysis_Jumping> jumping{};
	
	struct SpecializedBehaviors {		
		optional<Analysis_Digging> digging{};
		optional<Analysis_Constriction> constriction{};
	} specialized{};

	Analysis_Diagnostics diagnostics;
	
private:
	mutable std::atomic<int> _refCount{1};
	
	Output() = default;
};

}; // namespace TonTon

#endif // TONTON_ANALYSIS_H
