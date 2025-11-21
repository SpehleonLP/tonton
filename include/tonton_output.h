#ifndef TONTON_OUTPUT_H
#define TONTON_OUTPUT_H
#include "tonton_counted_ptr.hpp"
#include "tonton_shared_array.hpp"
#include "tonton_optional.hpp"
#include "tonton_wordlist.h"
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
struct Output_Chain
{
	// stop ik chain here
	uint16_t root{};     
	// ik target
	uint16_t tip{};
	
	int noJoints{};
	
	float stretched_length_m{};
	float rest_length_m{};
};

struct Output_Appendage : public Output_Chain
{
// everything in the group is symmetric about some bone. so if the gait group has 2 memebers its bilaterally symmetrical.
// common ancestor of the group
	int16_t common_ancestor{}; 
	uint16_t gait_group{}; 
	float   phase_offset{};
};

// Venom delivery
struct Output_Venom {
	int32_t delivery_joint{};       // fang, stinger, spine
	float injection_volume_ml{};
	float strike_speed_m_s{};
};
		
// ============================================================================
// MANIPULATION - limb end point: trunks, tentacles, hands
// ============================================================================
struct Output_Manipulator : public Output_Appendage {
	
	SemanticFlags subtree_flags{}; // TENTACLE, DIGIT, etc.
	
	float max_lift_force_N{};
	float max_grip_force_N{};
	float max_adhesion_force_N{};
	
	float surface_area_m2{};
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
struct Output_Physical {
	float body_mass_kg{};
	float body_length_m{};
	float body_volume_m3{};
	float tail_length_m{};
	
	float svl_m() const { return body_length_m - tail_length_m; }
	
	// Body plan characteristics
	float surface_area_m2{};
	float cross_sectional_area_m2{};
	float fineness_ratio{};  // length / mean_diameter
	
	int16_t		spine_root{};
	bool		upright{};
	CladeFlags  clade{CladeFlags::NONE};
	
	std::array<float, 6>  covariance_restPose{1.f, 1.f, 1.f, 0.f, 0.f, 0.f};
	inline glm::mat3 inertia_restPose() const 
	{
		auto & I = covariance_restPose;
		return glm::mat3{ 
			 I[1] +I[2],-I[3],-I[4],
			-I[3], I[0] +I[2],-I[5],
			-I[4],-I[5], I[0] +I[1] 
	};
	}
	
};

// ============================================================================
// METABOLIC & ENERGETICS (always present)
// ============================================================================
struct Output_Metabolic {
	float basal_rate_W{};
	float max_rate_W{};
	float aerobic_scope{};              // max/basal ratio

	float muscle_mass_kg{};
	float available_muscle_power_W{};   // ~400 W/kg theoretical max

	// Thermal strategy
	float body_temperature_K{-1};
	float thermal_neutral_zone_min_K{-1};
	float thermal_neutral_zone_max_K{-1};
	
	bool is_endotherm() const { return body_temperature_K > 0; } 
};


// Body undulation (for BCF swimmers)
struct Output_BodyWave : public Output_Chain {			
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
struct Output_Terrestrial {
	immutable_array<Output_Manipulator> legs{};
	
	// 0 upright -> 1 sprawling
	float posture{};

	// Speed capabilities
	float max_sprint_speed_m_s{};
	float max_sustainable_speed_m_s{};  // aerobic limit
	float optimal_speed_m_s{};          // minimum cost of transport

	// Maneuverability
	float min_turning_radius_m{};
	float max_acceleration_m_s2{};

	// Endurance limits (especially important for sprawling posture)
	float max_sprint_duration_s{-1};  // before exhaustion
	float recovery_time_s{-1};        // after sprint
};

// Serpentine locomotion (for SERPENTINE posture)
struct Output_Serpentine {
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
	float lateral_undulation_speed_m_s{};
	
	// Lateral undulation parameters
	Output_BodyWave lateral_undulation{};

	struct Rectilinear
	{
		float speed_m_s{};
		float frequency_Hz{};
	};

	struct SideWinding : public Rectilinear
	{
		float amplitude_ratio{};
		int32_t contact_points{};
	};

	struct Concertina 
	{
		float speed_m_s{};
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
struct Output_Aerial;
struct Output_TakeoffAnalysis {
    
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
    float takeoff_run_distance_m{-1};           // Required runway length (0 for vertical)
    float required_jump_velocity_m_s{0};        // Initial upward velocity needed from legs
    float time_to_flight_speed_s{-1};           // Time to reach min_flight_speed
    float vertical_acceleration_m_s2{0};        // Net upward accel during takeoff
    
    // Force analysis
    float max_instantaneous_lift_N{0};          // Peak lift during power stroke
    float max_instantaneous_thrust_N{0};        // Peak thrust from wing acceleration
    float net_vertical_force_N{0};              // Total upward force available
    float force_margin_percent{0};              // Safety margin above weight
    
    // Efficiency factors
    float power_to_weight_W_kg{0};              // Critical for vertical launch
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
    static float EstimateMaxLift(const Output_Aerial& aerial, 
                                float body_mass_kg,
                                float air_density = 1.225f);
    
    static float EstimateMaxThrust(const Output_Aerial& aerial,
                                  float body_mass_kg,
                                  float air_density = 1.225f);
    
    // Ground effect modeling
    static float GroundEffectBonus(float wing_span_m, float height_above_ground_m);
    
    // Required jump velocity from legs
    static float RequiredJumpVelocity(const Output_Aerial& aerial,
                                     float body_mass_kg,
                                     float max_lift_N,
                                     float gravity_m_s2);
    
    // Running takeoff distance
    static float RunwayDistance(const Output_Aerial& aerial,
                               const Output_Terrestrial* terrestrial,
                               float body_mass_kg);
    
    // Classification
 //   static TakeoffMode ClassifyMode(const Output& output, 
  //                                 const TakeoffAnalysis& analysis);
};

struct Output_Aerial {
	struct Wing : public Output_Appendage {			
		float span_m{};
		float area_m2{};
		float chord_m{};
		// span is half span b/c only one wing.
		// so span² * 2² / (area*2)
		float aspect_ratio() const { return span_m*span_m*2.0 / area_m2; };   // span² / area

		// Animation parameters
		float beat_amplitude_rad{};    // stroke angle
		float stroke_plane_angle_rad{}; // relative to body
		
		float mass_kg{};
		float inertia_kgm2{};
		
		
		float wing_tip_velocity(float wingbeat_frequency_Hz) const { 
			return 3.14159265 * wingbeat_frequency_Hz * beat_amplitude_rad * span_m; };
	};
	immutable_array<Wing> wings{};

	float wing_span_m{};
	float wing_area_m2{};

	// Performance envelope
	float wingbeat_frequency_Hz{};
	float min_flight_speed_m_s{};       // stall speed
	float cruise_speed_m_s{};           // optimal
	float max_flight_speed_m_s{};

	float wing_loading_N_m2{};          // weight / wing_area
	float power_loading_W_N{};          // power / weight

	// Specialized capabilities
	float max_altitude_m{};             // power/oxygen limit
	float max_flight_duration_s{};      // endurance limit

	// Maneuverability
	float min_turning_radius_m{};
	
	// computed with rest pose inertia. 
	float max_roll_rate_rad_s{};
	float max_pitch_rate_rad_s{}; 
	float max_yaw_rate_rad_s{}; 
	
	float flapping_cost_W_per_N{};      // Power per unit weight (lower = better)
	float hovering_cost_W_per_N{};      // Extremely expensive
			
	// Relative efficiency (0-1, higher = better strategy for this animal)
	float flapping_efficiency{};
	float hovering_efficiency{};
	bool can_hover{};

	// Flight capability diagnostics (based on muscle power budget)
	bool can_sustain_level_flight{};  // Sufficient power for sustained flight
	bool can_slow_descent{};          // Can flutter to slow fall, but not gain altitude
			
	// Steady glide: Lift = Weight
	float gliding_CL(float weight_N, float speed_m_s, float air_density = 1.225f) const;
	float flapping_CL_effective(float weight_N,
								float forward_speed_m_s, 
								float wingbeat_freq_Hz,
								float beat_amplitude_rad,
								float air_density = 1.225f) const;
	float hovering_disk_loading_N_m2(float weight_N) const;		
	float hovering_power_ideal_W(float weight_N, float air_density = 1.225f) const;
	
	Output_TakeoffAnalysis takeoff;
};

// ============================================================================
// AQUATIC LOCOMOTION (optional)
// ============================================================================
struct Output_Aquatic  {
	enum class PropulsionMode : uint8_t {
		BODY_CAUDAL_FIN,      // BCF - fish tail
		MEDIAN_PAIRED_FIN,    // MPF - pectoral/dorsal fins
		JET_PROPULSION,       // Squid/octopus
		PADDLE_LIMBS,         // Turtles/seals
		DORSOVENTRAL_FLUKES   // Whales/dolphins
	};

	struct Fin  : public Output_Appendage {
		float chord_m{};
		float area_m2{};

		// Animation - for undulating fins
		float beat_frequency_Hz{0};
		float beat_amplitude_rad{0};
		float wave_speed_ratio{0};  // wave_speed / swim_speed
		
		glm::vec3 normal_vector{0, 0, 0}; // sign is not defined. 

		SemanticFlags type{};  // FIN, WING (for flying fish), LIMB
	};
	immutable_array<Fin> propulsors{};

	std::optional<Output_BodyWave> body_wave{};

	// Speed capabilities
	float cruise_speed_m_s{};
	float burst_speed_m_s{};
	float min_swim_speed_m_s{};         // to maintain lift/control

	// Hydrodynamics
	float lift_per_meter_swam_N{};      // For negatively buoyant animals
	float sink_rate_m_s{};              // How fast they sink when stationary
	float reynolds_number{};            // Swimming regime indicator
	float drag_coefficient{};           // Cd for streamlining assessment
	float tail_amplitude_m{};           // Actual tail oscillation amplitude
	float beat_frequency_Hz{};          // Tail beat frequency

	// Buoyancy control
	float neutral_buoyancy_density_kg_m3{};
	float swim_bladder_adjust_time_s{-1};
	PropulsionMode primary_mode{};
	bool has_swim_bladder{};

	// Maneuverability
	bool can_hover{};                   // station-keeping
	bool requires_constant_motion{};    // sharks, tuna
	float min_turning_radius_m{};

	// Depth capabilities
	float preferred_depth_min_m{};
	float preferred_depth_max_m{};
	float crush_depth_m{-1};


	// C-start escape response
	struct CStartResponse {
		float duration_s{};
		float max_body_curvature_rad{};
		float acceleration_m_s2{};
	};
	std::optional<CStartResponse> c_start{};
	
	struct JetPropulsion {
		float mantle_contraction_frequency_Hz{};
		float jet_pulse_volume_m3{};
		float jet_velocity_m_s{};
		int32_t siphon_joint{};
		float siphon_articulation_range_rad{};
	};
	std::optional<JetPropulsion> jet_propulsion{};
};

// ============================================================================
// CLIMBING (optional)
// ============================================================================
struct Output_Climbing {
	immutable_array<Output_Manipulator> limbs{};

	float max_climb_speed_m_s{};
	float max_climb_angle_rad{};        // from horizontal
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
struct Output_Brachiation {
	struct Arm  : public Output_Appendage {
		float reach_m{};
		float grip_strength_N{};
		float swing_speed_m_s{};
	};
	immutable_array<Arm> arms{};

	float max_swing_speed_m_s{};
	float max_gap_distance_m{};      // max distance between handholds
	float pendulum_length_m{};       // effective pendulum for energy

	// Alternating arm pattern
	float swing_frequency_Hz{};
	float arm_phase_offset{};        // phase difference between arms (usually 0.5)
};

// ============================================================================
// JUMPING (optional)
// ============================================================================
struct Output_Jumping {
	enum class MechanismType {
		MUSCLE_DIRECT,      // Direct muscle power
		ELASTIC_CATAPULT,   // Tendon energy storage
		HYDRAULIC          // Hemolymph pressure (insects)
	};
	MechanismType mechanism{};

	float max_jump_height_m{};
	float max_jump_distance_m{};
	float takeoff_velocity_m_s{};
	float takeoff_angle_rad{};

	// Energy storage
	float elastic_storage_J{0};
	float power_amplification_ratio{0};

	// Recovery
	float recovery_time_s{};            // between jumps
};

// ============================================================================
// SPECIALIZED BEHAVIORS
// ============================================================================
// Burrowing/fossorial
struct Output_Digging {
	enum class Method { SCRATCH, HEAD_LIFT, INCISOR, HUMERAL_ROTATION };
	Method method{};

	float max_dig_speed_m_s{};
	float tunnel_diameter_m{};
	float soil_force_N{};
};

// Constriction (snakes, tentacles)
struct Output_Constriction {
	float max_squeeze_pressure_Pa{};
	float coil_diameter_range_min_m{};
	float coil_diameter_range_max_m{};
};


struct  Output_Tail : public Output_Appendage // or tips for branching
{       
	float mass_kg{};
	float max_cross_section_m2{};
	float min_cross_section_m2{};

	// Animation
	float natural_sway_frequency_Hz{};
	
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
	immutable_array<Output_Tail> branches{};  // empty for single tail
    std::optional<Output_Venom> venom{}; // how?
};

// ============================================================================
// BEHAVIORAL PROFILE (always present)
// ============================================================================
struct Output_Behavior {
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
	float personal_space_radius_m{-1};
	float territory_radius_m{-1};

	// Environmental preferences
	struct HabitatPreference {
		float temperature_min_K{};
		float temperature_max_K{};
		float humidity_preference{};    // 0=arid, 1=humid

		// Structural habitat
		float open_vs_dense{};          // 0=open terrain, 1=dense cover
		float canopy_height_preference_m{-1};
	};
	HabitatPreference habitat{};
};


// ============================================================================
// SENSORY SYSTEMS (always present, but may have low values)
// ============================================================================
struct Output_Vision {
	float acuity{};                 // 0=poor, 1=excellent
	float binocular_overlap{};      // 0=none, 1=full overlap
	float detection_range_m{};
	float centering{}; // how centered are the eyes? -1 or +1 indicates flatfish
	
	float motion_sensitivity_bonus{};
	float thermal_detection_range_m{};
	float thermal_sensitivity_K{};
	
	bool has_color_vision{};
	bool has_night_vision{};
	bool has_thermal_vision{};
	bool has_uv_vision{};
};

struct Output_Hearing {
	float sensitivity{};            // 0=poor, 1=excellent
	float frequency_range_Hz_min{};
	float frequency_range_Hz_max{};
	float detection_range_m{};
	
	bool has_echolocation{};
	float echolocation_range_m{};
	float directional_accuracy_deg{};
	float substrate_vibration_sensitivity{};
};

struct Output_Olfaction {
	float sensitivity{};            // 0=poor, 1=excellent
	float detection_range_m{};
	int odor_discrimination_count{};
	float directional_acuity{};
};
		
template<template<typename> typename OPTIONAL>
struct Output_Sensory {
	OPTIONAL<Output_Vision> vision{};
	OPTIONAL<Output_Hearing> hearing{};
	OPTIONAL<Output_Olfaction> olfaction{};
	immutable_array<Output_Chain> antennae{};
};

// ============================================================================
// CONFIDENCE & WARNINGS
// ============================================================================
struct Output_Diagnostics {
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
using Sensory = Output_Sensory<optional>;

	static counted_ptr<const Output> Factory(Input const& in);
//	static counted_ptr<const Output> Scale(Output const& in, float scale);
	

	void AddRef() const { ++_refCount; };
	void Release() const { if(--_refCount == 0) delete this; }
	
	Output_Physical physical{};
	Output_Metabolic metabolic{};
	Output_Behavior  behavior{};
	Output_Sensory<optional> sensory{};
	
	struct Appendages {
		using Tail = Output_Tail;
		immutable_array<Output_Tail> tails{};  // usually 1, but kitsune has 9!
		immutable_array<Output_Manipulator> manipulation{};
	} appendages{};
	
	optional<Output_Terrestrial> terrestrial{};
	optional<Output_Serpentine> serpentine{};
	optional<Output_Aerial> aerial{};
	optional<Output_Aquatic> aquatic{};
	optional<Output_Climbing> climbing{};
	optional<Output_Brachiation> brachiation{};
	optional<Output_Jumping> jumping{};
	
	struct SpecializedBehaviors {		
		optional<Output_Digging> digging{};
		optional<Output_Constriction> constriction{};
	} specialized{};

	Output_Diagnostics diagnostics;
	
private:
	mutable std::atomic<int> _refCount{1};
	
	Output() = default;
};

}; // namespace TonTon

#endif // TONTON_OUTPUT_H
