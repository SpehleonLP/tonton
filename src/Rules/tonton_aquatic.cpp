#include "tonton_aquatic.h"
#include "dodeedum.h"
#include "tonton_output.h"
#include "tonton_input.h"
#include "Rules/tonton_scratch.h"
#include "Memos/tonton_armaturememo.h"
#include "Memos/tonton_skinnedmeshmemo.h"
#include <cfloat>
#include <cmath>
#include <iostream>
#include <sstream>

#define DEBUG_AQUATIC 0

#if DEBUG_AQUATIC
#define AQUATIC_DBG(x) std::cout << "[AQUATIC] " << x << "\n"
#else
#define AQUATIC_DBG(x)
#endif

using SF = TonTon::SemanticFlags;

namespace TonTon
{
std::vector<TonTon::Output_Aquatic::Fin> Compute_Fins(TonTon::Input const& in, TonTon::Scratch & s);
};

std::optional<TonTon::Output_Aquatic>   TonTon::ComputeAquatic(Input const& in, Scratch & s)
{
	AQUATIC_DBG("Starting aquatic analysis, fluid_density=" << in.environment.fluidDensity_Kg_m3 << " kg/m3");

	auto & sk = *in.skinnedMesh;
	auto * sk_memo = sk.skin->memo();

	auto fins = Compute_Fins(in, s);
	auto semantic_flags = sk_memo->GetSemanticFlags();
//	auto relative_flags = sk_memo->GetRelativeFlags();

	// 1. DETECT PROPULSION MODE
	// Check for fins, flukes, limbs, or body flexibility
	
	bool has_fins = false;
	bool is_tail_horizontal = false;
	bool has_paddle_limbs = false;
	bool has_assymetric_tail = false;
	
	std::vector<bool> is_tail_fin(fins.size(), false);
	{
		auto gcr_table = sk_memo->GetGcrTable();
		auto N = sk.skin->parents.size();
		
		for(auto i = 0u; i < fins.size(); ++i)
		{
			auto offset = &gcr_table[N*fins[i].root];
			
			if(HasFlag(fins[i].type, SF::FIN))
				has_fins = true;
			else
				has_paddle_limbs = true;
			
			if(HasFlag(fins[i].type, SF::TAIL))
				is_tail_fin[i] = true;
				
			if(is_tail_fin[i])
			{
				auto fin_joints = sk_memo->GetAllChildrenOfRoot(fins[i].root);
				auto aabb = sk.aabb[fins[i].root];
				
				for(auto & j : fin_joints)
				{
					aabb.min = glm::min(aabb.min, sk.aabb[j].min);
					aabb.max = glm::min(aabb.max, sk.aabb[j].max);
				}
				
				
				is_tail_horizontal |= (aabb.max.x - aabb.min.x) > (aabb.max.y - aabb.min.y); 
				
				if(!is_tail_horizontal)
				{
					glm::vec3 center = (aabb.min + aabb.max) / 2.f;
					glm::vec3 centroid = sk.GetCentroid(fin_joints, in.behavior.scale);
					
					float offset = glm::dot(centroid - center, glm::vec3(0, 1, 0));
					offset = offset / (aabb.max.y - aabb.min.y);
					
	// therefore the tail generates lift and we don't have a swim bladder!				
					has_assymetric_tail |= (offset > 0.01);
				}
			}
		}
	};

	bool has_flexible_body = false;
	bool has_jet_system = false;

	AQUATIC_DBG("Checking for propulsion structures...");

	// Check semantic flags for aquatic structures
	for(auto i = 0u; i < semantic_flags.size(); ++i)
	{
		// Check for siphon (cephalopods)
		for(auto word : sk.skin->tags[i])
		{
			if(word == Word::siphon)
				has_jet_system = true;
		}
	}

	// Check body flexibility from tail or serpentine
	// Only consider tail propulsive if it has sufficient mass and length for momentum generation
	// Crocodiles: heavy tail (~25% body mass), good for undulation
	// Cats: light tail (~3% body mass), NOT for swimming
	// Dragonflies: thin abdomen operates at low Reynolds number (viscous regime)
	if(s.serpentine.has_value())
	{
		AQUATIC_DBG("Has serpentine locomotion -> flexible body");
		has_flexible_body = true;
	}
	else if(s.appendages.tails.size() > 0)
	{
		// Calculate tail momentum potential relative to body
		for(auto const& tail : s.appendages.tails)
		{
			float tail_mass_ratio = tail.mass_kg / s.physical.body_mass_kg;
			float tail_length_ratio = tail.stretched_length_m / s.physical.body_length_m;

			// Momentum factor: mass ratio × length ratio
			// Crocodile: ~0.25 × ~0.5 = 0.125
			// Cat: ~0.03 × ~0.5 = 0.015
			float momentum_factor = tail_mass_ratio * tail_length_ratio;

			// Threshold: need significant momentum for propulsion
			if(momentum_factor > 0.08f)
			{
				// REYNOLDS NUMBER CHECK for BCF swimming viability
				// Re = ρ * v * L / μ
				// Need Re > 1000 for effective undulation (inertial forces must dominate)

				// Estimate tail thickness from cross-section
				float tail_diameter_m = std::sqrt(4.0f * tail.max_cross_section_m2 / 3.14159f);

				// If cross-section data is missing, estimate from tail length
				if(tail_diameter_m < 0.001f)
				{
					// Estimate diameter as ~1/10 of tail length for typical taper
					tail_diameter_m = tail.stretched_length_m * 0.1f;
				}

				// Estimate swimming velocity (conservative: 1 body length/s)
				float swim_velocity_m_s = s.physical.body_length_m * 1.0f;

				float fluid_density = in.environment.fluidDensity_Kg_m3;
				float fluid_viscosity = in.environment.fluidViscosity_Pa_s;

				float reynolds = (fluid_density * swim_velocity_m_s * tail_diameter_m) / fluid_viscosity;

				// At Re < 1000, viscous forces dominate and undulation is ineffective
				// Dragonfly abdomen: Re ~ 100 (too viscous)
				// Crocodile tail: Re > 100,000 (inertial regime, effective)
				if(reynolds < 1000.0f)
				{
					continue; // Skip this tail, Reynolds number too low
				}

				has_flexible_body = true;
				break;
			}
		}
	}

	// Determine primary mode
	Output_Aquatic::PropulsionMode primary_mode;

	if(has_jet_system)
	{
		primary_mode = Output_Aquatic::PropulsionMode::JET_PROPULSION;
	}
	else if(is_tail_horizontal)
	{
		primary_mode = Output_Aquatic::PropulsionMode::DORSOVENTRAL_FLUKES;
	}
	else if(has_paddle_limbs)
	{
		primary_mode = Output_Aquatic::PropulsionMode::PADDLE_LIMBS;
	}
	else if(has_fins)
	{
		// Could be BCF (body-caudal fin) or MPF (median-paired fin)
		// Use BCF if has flexible body, otherwise MPF
		if(has_flexible_body)
			primary_mode = Output_Aquatic::PropulsionMode::BODY_CAUDAL_FIN;
		else
			primary_mode = Output_Aquatic::PropulsionMode::MEDIAN_PAIRED_FIN;
	}
	else if(has_flexible_body)
	{
		// AERIAL EXCLUSION: Don't classify flying animals as aquatic swimmers
		// Adult dragonflies have wings and fly - they don't swim
		// BUT waterfowl (ducks) swim with paddle limbs, which is already handled above
		if(s.aerial.has_value())
		{
			// If aerial mode exists and we ONLY have body flexibility (no fins/limbs/flukes),
			// this is probably a flying insect with a long abdomen, not a swimmer
			return {};
		}

		// Pure body undulation (eel-like, crocodile)
		primary_mode = Output_Aquatic::PropulsionMode::BODY_CAUDAL_FIN;
	}
	else
	{
		// No clear aquatic adaptations
		AQUATIC_DBG("No aquatic propulsion detected - rejecting");
		return {};
	}

	AQUATIC_DBG("Propulsion mode detected: " << (int)primary_mode);

	// 2. HYDRODYNAMIC PARAMETERS
	float body_mass_kg = s.physical.body_mass_kg;
	float body_length_m = s.physical.body_length_m;
	float body_volume_m3 = s.physical.body_volume_m3;
	float cross_section_m2 = s.physical.cross_sectional_area_m2;

	float fluid_density = in.environment.fluidDensity_Kg_m3;
//	float fluid_viscosity = in.environment.fluidViscosity_Pa_s;

	// 3. BUOYANCY
	float body_density_kg_m3 = in.body_density();
	float neutral_buoyancy_density = in.environment.fluidDensity_Kg_m3;

	// Most fish have swim bladders to achieve neutral buoyancy
  // Line 239: BEFORE calculating requires_constant_motion
	bool has_swim_bladder = HasFlag(s.physical.clade, CladeFlags::PISCES); // Default for fish
	
	// Asymmetric tail = generates lift = NO swim bladder (sharks, sturgeons)
	if (has_assymetric_tail) {
	  has_swim_bladder = false;
	}

	// Sharks and some fast swimmers lack swim bladders (must keep moving)
	bool requires_constant_motion = false;
	if(body_density_kg_m3 > fluid_density * 1.05f && !has_swim_bladder)
	{
		requires_constant_motion = true;
	}

	float swim_bladder_adjust_time_s = -1;
	if(has_swim_bladder)
	{
		// Time to adjust buoyancy (diffusion-limited)
		// Roughly: adjustment_time ∝ body_mass^(1/3)
		swim_bladder_adjust_time_s = 30.0f * std::pow(body_mass_kg, 0.33f);
	}

	// 4. CALCULATE SWIMMING SPEEDS
	// Using Strouhal number (St = f*A/U, optimal ≈ 0.25-0.35)

	float strouhal_optimal = 0.3f;

	// Beat frequency varies by propulsion mode:
	// - BCF (body-caudal fin): Low frequency, high amplitude oscillation
	// - MPF (median-paired fin): High frequency, low amplitude rowing
	// - Paddle limbs: Medium frequency, rowing motion
	// - Dorsoventral flukes (cetaceans): Medium frequency, up-down motion
	// Base scaling: f ∝ L^(-0.5) for cruising (from fish data)
	float beat_frequency_Hz;

	if(primary_mode == Output_Aquatic::PropulsionMode::BODY_CAUDAL_FIN)
	{
		beat_frequency_Hz = 2.0f * std::pow(body_length_m, -0.5f); // BCF: 1-3 Hz for most fish
	}
	else if(primary_mode == Output_Aquatic::PropulsionMode::MEDIAN_PAIRED_FIN)
	{
		beat_frequency_Hz = 4.0f * std::pow(body_length_m, -0.5f); // MPF: higher frequency
	}
	else if(primary_mode == Output_Aquatic::PropulsionMode::PADDLE_LIMBS)
	{
		beat_frequency_Hz = 1.5f * std::pow(body_length_m, -0.5f); // Rowing: medium frequency
	}
	else if(primary_mode == Output_Aquatic::PropulsionMode::DORSOVENTRAL_FLUKES)
	{
		beat_frequency_Hz = 2.5f * std::pow(body_length_m, -0.5f); // Cetacean fluking
	}
	else
	{
		beat_frequency_Hz = 2.0f * std::pow(body_length_m, -0.5f); // Default
	}

	// PHYSICS-BASED TAIL AMPLITUDE CALCULATION
	// The tail must oscillate through the fluid, which requires power to:
	// 1. Accelerate the added mass of fluid moving with the tail
	// 2. Overcome drag on the oscillating surface
	//
	// Power for oscillation: P ≈ 0.5 × ρ × A_tail × (v_peak)³ × C_d
	// where v_peak = 2π × f × amplitude
	//
	// Solving for amplitude: A = (P / (4π³ × ρ × A_tail × f³ × C_d))^(1/3)

	// Use actual tail fin area from computed fins
	float tail_area_m2 = 0.0f;
	for(size_t i = 0; i < fins.size(); ++i)
	{
		if(is_tail_fin[i])
		{
			tail_area_m2 += fins[i].area_m2;
		}
	}

	// Fallback: estimate from cross-section if no tail fins detected
	if(tail_area_m2 < 0.001f)
	{
		float estimated_cross_section = cross_section_m2;
		if(estimated_cross_section < 0.001f)
		{
			// Estimate as cylinder: π × (diameter/2)²
			// Diameter ≈ L / fineness_ratio, or roughly L/5 for typical fish
			float body_diameter = body_length_m / 5.0f;
			estimated_cross_section = M_PI * (body_diameter / 2.0f) * (body_diameter / 2.0f);
		}

		tail_area_m2 = estimated_cross_section;

		// Use tail cross-section if available
		if(s.appendages.tails.size() > 0)
		{
			float tail_cross_section = s.appendages.tails[0].max_cross_section_m2;
			if(tail_cross_section > 0.001f)
			{
				// Effective area for oscillation ≈ cross-section × amplitude range
				// Estimate as 3× cross-section for typical tail shape
				tail_area_m2 = std::max(tail_cross_section * 3.0f, estimated_cross_section * 0.5f);
			}
		}
	}

	AQUATIC_DBG("Using tail area: " << tail_area_m2 << "m2");

	// Available power for swimming - use metabolic rate, not peak muscle power
	// Aerobic scope gives sustainable power for cruising
	// Mechanical efficiency of swimming: ~20-25%
	float aerobic_power_W = s.metabolic.basal_rate_W * float(s.metabolic.aerobic_scope);
	float swim_power_W = s.metabolic.available_muscle_power_W * 0.3f; // ~30% for swimming
//; aerobic_power_W * 0.2f; // 20% mechanical efficiency

	AQUATIC_DBG("Metabolic: basal=" << s.metabolic.basal_rate_W << "W, aerobic_scope="
	            << s.metabolic.aerobic_scope << ", aerobic=" << aerobic_power_W
	            << "W, mechanical=" << swim_power_W << "W");

	// Drag coefficient for oscillating surfaces (higher than steady-state)
	float oscillation_drag_coef = 1.5f;

	// Calculate amplitude from power requirements
	// A³ = P / (4π³ × ρ × A_tail × f³ × C_d)
	float amplitude_cubed = swim_power_W /
		(4.0f * M_PI * M_PI * M_PI * fluid_density * tail_area_m2 *
		 std::pow(beat_frequency_Hz, 3.0f) * oscillation_drag_coef);

	float tail_amplitude_m = std::pow(amplitude_cubed, 1.0f / 3.0f);

	AQUATIC_DBG("Calculated amplitude: " << tail_amplitude_m << "m from power=" << swim_power_W << "W, tail_area=" << tail_area_m2 << "m2, freq=" << beat_frequency_Hz << "Hz");

	// PHYSICAL FEASIBILITY CHECK
	// Maximum amplitude is limited by spine/body flexibility
	float max_amplitude_ratio = 0.25f; // Typical fish: 0.2-0.3
	if(primary_mode == Output_Aquatic::PropulsionMode::BODY_CAUDAL_FIN)
	{
		max_amplitude_ratio = 0.35f; // Flexible swimmers (eels) can achieve higher
	}

	float max_amplitude_m = body_length_m * max_amplitude_ratio;

	AQUATIC_DBG("Max amplitude: " << max_amplitude_m << "m (" << max_amplitude_ratio << " × body_length)");
	AQUATIC_DBG("Rejection threshold: " << (max_amplitude_m * 2.5f) << "m");

	// If required amplitude exceeds physical limits, swimming is not viable in this medium
	if(tail_amplitude_m > max_amplitude_m * 2.5f) // Allow larger margin - reject only if impossibly large
	{
		// Required amplitude is impossibly large (e.g., trying to swim in air)
		// This primarily catches cases like trying to swim in air where amplitude would be 10x+ body length
		AQUATIC_DBG("REJECTING: amplitude (" << tail_amplitude_m << "m) > threshold (" << (max_amplitude_m * 2.5f) << "m)");

		// Add helpful diagnostics about what would make swimming viable
		// Since amplitude ∝ (1/ρ)^(1/3), to reduce amplitude we need to increase density
		float required_fluid_density = fluid_density * std::pow(tail_amplitude_m / max_amplitude_m, 3.0f);
		// Since amplitude ∝ (1/A_tail)^(1/3), to reduce amplitude we need to increase tail area
		float required_tail_area = tail_area_m2 * std::pow(tail_amplitude_m / max_amplitude_m, 3.0f);

		std::stringstream msg;
		msg << "Aquatic locomotion not viable: required tail amplitude ("
		    << tail_amplitude_m << "m = " << (tail_amplitude_m / body_length_m * 100) << "% body length) "
		    << "exceeds spine flexibility limit (" << max_amplitude_m << "m). "
		    << "Swimming would be possible if: (1) fluid density ≥ "
		    << required_fluid_density << " kg/m³ (currently " << fluid_density << "), or "
		    << "(2) tail/fin area ≥ " << required_tail_area << " m² (currently " << tail_area_m2 << ").";

		s.diagnostics.warnings.push_back({
			.level = Output_Diagnostics::Warning::Severity::INFO,
			.message = msg.str()
		});

		return {};
	}

	// Clamp to physically achievable amplitude
	tail_amplitude_m = std::min(tail_amplitude_m, max_amplitude_m);
	float tail_amplitude_ratio = tail_amplitude_m / body_length_m;

	AQUATIC_DBG("Final amplitude: " << tail_amplitude_m << "m (" << tail_amplitude_ratio << " × body_length)");

	// Populate per-fin animation parameters
	for(size_t i = 0; i < fins.size(); ++i)
	{
		if(is_tail_fin[i])
		{
			// Tail fins: primary propulsion, use main beat frequency
			fins[i].beat_frequency_Hz = beat_frequency_Hz;
			fins[i].beat_amplitude_rad = tail_amplitude_ratio * M_PI; // Convert linear amplitude to angular
			fins[i].wave_speed_ratio = 1.0f; // Wave travels at body speed for propulsion
		}
		else if(primary_mode == Output_Aquatic::PropulsionMode::MEDIAN_PAIRED_FIN)
		{
			// MPF mode: pectoral fins are primary propulsors
			if(HasFlag(fins[i].type, SF::FORELIMB) || HasFlag(fins[i].type, SF::FIN))
			{
				fins[i].beat_frequency_Hz = beat_frequency_Hz;
				fins[i].beat_amplitude_rad = 0.5f; // ~30 degrees typical for MPF rowing
				fins[i].wave_speed_ratio = 0.0f; // Rowing motion, not undulation
			}
		}
		else if(primary_mode == Output_Aquatic::PropulsionMode::PADDLE_LIMBS)
		{
			// Paddle limbs: rowing/flapping motion
			if(HasFlag(fins[i].type, SF::LIMB))
			{
				fins[i].beat_frequency_Hz = beat_frequency_Hz;
				fins[i].beat_amplitude_rad = 1.0f; // ~60 degrees for power stroke
				fins[i].wave_speed_ratio = 0.0f; // Rowing, not undulation
			}
		}
		// Stabilizing fins (dorsal, anal) remain at 0 frequency (passive)
	}

	// Cruise speed from Strouhal: U = f*A/St
	float cruise_speed_m_s = (beat_frequency_Hz * tail_amplitude_m) / strouhal_optimal;

	// Drag-based speed limit
	// Drag: D = 0.5 * ρ * v² * Cd * A
	float drag_coefficient = 0.04f; // Streamlined fish

	if(s.physical.fineness_ratio < 3.0f)
	{
		drag_coefficient = 0.08f; // Less streamlined
	}

	// Max speed when power = drag * velocity
	// P = D * v = 0.5 * ρ * v³ * Cd * A
	// v = (2*P / (ρ * Cd * A))^(1/3)
	float power_limited_speed = std::pow(
		(2.0f * swim_power_W) / (fluid_density * drag_coefficient * cross_section_m2),
		1.0f / 3.0f
	);

	// Cruise is the lower of Strouhal-optimal or power-limited
	cruise_speed_m_s = std::min(cruise_speed_m_s, power_limited_speed);

	// Burst speed (anaerobic, can use max metabolic rate for short duration)
	// Use max_rate for anaerobic bursts (5-10 seconds)
	float burst_power_W = s.metabolic.max_rate_W * 0.25f; // 25% mechanical efficiency for burst
	float burst_power_limited_speed = std::pow(
		(2.0f * burst_power_W) / (fluid_density * drag_coefficient * cross_section_m2),
		1.0f / 3.0f
	);

	// Burst can also be limited by Strouhal at higher amplitude
	float burst_strouhal_speed = (beat_frequency_Hz * tail_amplitude_m * 1.5f) / strouhal_optimal;

	float burst_speed_m_s = std::min(burst_power_limited_speed, burst_strouhal_speed);

	// Apply muscle quality
	cruise_speed_m_s *= glm::mix(0.8f, 1.2f, in.muscle_quality);
	burst_speed_m_s *= glm::mix(0.8f, 1.2f, in.muscle_quality);

	// Minimum swim speed (to maintain control/lift)
	float min_swim_speed_m_s = 0.0f;
	if(requires_constant_motion)
	{
		// Must swim fast enough to generate lift
		min_swim_speed_m_s = cruise_speed_m_s * 0.5f;
	}

	// 5. MANEUVERABILITY
	// Turning radius limited by centripetal force
	// F_centripetal = m*v²/r

	bool can_hover = false;
	if(primary_mode == Output_Aquatic::PropulsionMode::MEDIAN_PAIRED_FIN ||
	   primary_mode == Output_Aquatic::PropulsionMode::JET_PROPULSION)
	{
		// MPF swimmers and jet propulsion can hover
		can_hover = true;
	}

	// Turning radius based on body flexibility and control surfaces
	float agility_factor = 1.0f;

	if(has_flexible_body)
		agility_factor = 2.0f; // More agile

	if(primary_mode == Output_Aquatic::PropulsionMode::PADDLE_LIMBS)
		agility_factor = 1.5f; // Good maneuverability

	// r = v² / (a_max)
	// Max lateral acceleration ≈ 0.5*g for typical fish
	float max_lateral_accel = in.environment.gravity_m_s2 * 0.5f * agility_factor;
	float min_turning_radius_m = (cruise_speed_m_s * cruise_speed_m_s) / max_lateral_accel;

	// 6. C-START ESCAPE RESPONSE
	// Most fish can perform C-starts for rapid acceleration
	std::optional<Output_Aquatic::CStartResponse> c_start;

	if(has_flexible_body && primary_mode != Output_Aquatic::PropulsionMode::PADDLE_LIMBS)
	{
		// C-start: bend body into C-shape, then snap straight
		float c_start_duration_s = 0.05f; // ~50ms typical
		float max_curvature_rad = M_PI / 2.0f; // 90-degree bend

		// Acceleration = v / t, where v ≈ burst_speed
		float c_start_acceleration = burst_speed_m_s / c_start_duration_s;

		c_start = Output_Aquatic::CStartResponse{
			.duration_s = c_start_duration_s,
			.max_body_curvature_rad = max_curvature_rad,
			.acceleration_m_s2 = c_start_acceleration
		};
	}

	// 7. DEPTH CAPABILITIES
	// Based on body structure and compressibility

	float preferred_depth_min_m = 0.0f;
	float preferred_depth_max_m = 100.0f; // Default shallow water
	float crush_depth_m = -1.0f;

	// Estimate from body structure
	// Rigid bodies (shells, armor) can go deeper
	// Swim bladders limit depth range

	if(has_swim_bladder)
	{
		// Swim bladder limits depth changes
		preferred_depth_max_m = 50.0f;
		crush_depth_m = 200.0f; // Swim bladder compression
	}
	else if(primary_mode == Output_Aquatic::PropulsionMode::JET_PROPULSION)
	{
		// Cephalopods can handle deep water
		preferred_depth_max_m = 500.0f;
		crush_depth_m = 2000.0f;
	}
	else
	{
		// Other modes (paddle, constant motion)
		preferred_depth_max_m = 200.0f;
		crush_depth_m = 500.0f;
	}

	// Scale by structure_vs_weight
	crush_depth_m *= glm::mix(0.5f, 2.0f, in.structure_vs_weight);

	// 9. BODY WAVE (for BCF swimmers)
	std::optional<Output_BodyWave> body_wave;

	if(primary_mode == Output_Aquatic::PropulsionMode::BODY_CAUDAL_FIN && has_flexible_body)
	{
		Output_BodyWave wave;

		// Use tail as reference
		if(s.appendages.tails.size() > 0)
		{
			auto const& tail = s.appendages.tails[0];
			static_cast<Output_Chain&>(wave) = tail;
		}

		wave.wavelength_ratio = 0.7f; // Wavelength ≈ 0.7 * body_length
		wave.amplitude_ratio = tail_amplitude_ratio;

		// Body flexibility: 0=rigid (tuna), 1=eel
		if(s.serpentine.has_value())
		{
			wave.body_flexibility = 0.8f; // Very flexible
		}
		else
		{
			wave.body_flexibility = 0.4f; // Moderate (typical fish)
		}

		// Amplitude envelope (increases toward tail)
		wave.segment_amplitude.constant = 0.1f;
		wave.segment_amplitude.linear = 0.3f;
		wave.segment_amplitude.geometric = 1.0f;

		body_wave = wave;
	}

	// 10. JET PROPULSION (for cephalopods)
	std::optional<Output_Aquatic::JetPropulsion> jet_propulsion;

	if(has_jet_system)
	{
		Output_Aquatic::JetPropulsion jet;

		// Find siphon joint
		jet.siphon_joint = -1;
		for(auto i = 0u; i < sk.skin->tags.size(); ++i)
		{
			for(auto word : sk.skin->tags[i])
			{
				if(word == Word::siphon)
				{
					jet.siphon_joint = i;
					break;
				}
			}
		}

		// Jet parameters
		jet.mantle_contraction_frequency_Hz = beat_frequency_Hz * 1.5f;
		jet.jet_pulse_volume_m3 = body_volume_m3 * 0.1f; // ~10% of body volume per pulse

		// Jet velocity from momentum conservation
		// m_jet * v_jet = m_body * v_body
		float jet_mass_kg = jet.jet_pulse_volume_m3 * fluid_density;
		jet.jet_velocity_m_s = (body_mass_kg * cruise_speed_m_s) / jet_mass_kg;

		jet.siphon_articulation_range_rad = M_PI; // 180-degree steering

		jet_propulsion = jet;
	}

	// ========== CLADE-SPECIFIC REFINEMENTS ==========
	// Applied after generic physics calculations to adjust for biological reality

	using CF = CladeFlags;

	// PISCES: Fish-specific swimming characteristics
	if (HasFlag(s.physical.clade, CF::PISCES)) {
		// Fish have specialized muscle fiber types
		// Rome et al. (1988): Red muscle 200 W/kg sustained, white muscle 250-500 W/kg burst
		// Wardle (1975): White muscle can produce 400 W/kg for short bursts (5-10s)
		// Bone & Marshall (1982): Most fish have 30-70% white muscle by mass

		// Burst capability depends on white muscle fraction
		// Power-oriented fish (predators) have more white muscle
		float white_muscle_fraction = glm::mix(0.3f, 0.7f, 1.0f - in.behavior.endurance_vs_power);

		// White muscle gives 2x power output of red muscle for short bursts
		float burst_power_multiplier = 1.0f + white_muscle_fraction; // 1.3x to 1.7x
		burst_speed_m_s *= burst_power_multiplier;
		burst_speed_m_s = std::min(burst_speed_m_s, cruise_speed_m_s * 3.5f); // Cap at 3.5x

		// Caudal aspect ratio affects maximum speed
		// Sambilay (1990): V = 0.59 * exp(0.42 * AR) relationship across 63 species
		// High AR (tuna, AR~6-8) = fast cruiser, Low AR (pike, AR~1-2) = burst acceleration
		// This refinement is already captured in generic physics, no additional adjustment needed
	}

	// CETACEA: Air-breathing marine mammals (whales, dolphins, porpoises)
	if (HasFlag(s.physical.clade, CF::CETACEA)) {
		// Cetaceans don't have swim bladders - they use lungs for buoyancy control
		has_swim_bladder = false;
		swim_bladder_adjust_time_s = -1.0f;

		// Can control buoyancy via lung volume (not ballast like sharks)
		requires_constant_motion = false;
		neutral_buoyancy_density = body_density_kg_m3; // Can match water density

		// Cetaceans can dive MUCH deeper than fish (no swim bladder compression)
		// Sperm whales: 2000m+, beaked whales: 3000m+
		crush_depth_m *= 5.0f; // 5x deeper than comparable fish
		preferred_depth_max_m = std::min(preferred_depth_max_m * 3.0f, crush_depth_m * 0.5f);

		// Must surface to breathe (oxygen stores in muscle myoglobin + blood)
		// Dive duration depends on lung volume and metabolic rate
		float lung_volume_L = body_volume_m3 * 0.05f * 1000.0f; // ~5% body volume
		float O2_storage_L = lung_volume_L * 0.9f; // ~90% usable O2

		// O2 consumption rate (Kleiber's law for mammals)
		float O2_consumption_L_per_min = s.metabolic.max_rate_W * 0.05f; // ~0.05 L O2 per watt per minute

		// Maximum dive time before must surface

		// Cetaceans are endotherms - higher speed capability than similar-sized fish
		cruise_speed_m_s *= 1.2f; // ~20% faster due to sustained high power
		burst_speed_m_s *= 1.2f;
	}

	// MOLLUSCA/CEPHALOPODA: Jet propulsion swimmers (octopus, squid, nautilus)
	// Already handled by jet_propulsion mode detection
	// No additional refinements needed - mode captures the physics

	// AMPHIBIA: Tadpoles, aquatic frogs (if somehow has fins/tail)
	if (HasFlag(s.physical.clade, CF::AMPHIBIA)) {
		// Amphibians have lower metabolic rates (ectotherm) → lower sustained speeds
		// Already reflected in metabolic scaling
		// Most use paddle limbs or body undulation (already captured)

		// Amphibians must remain in specific depth ranges (skin respiration)
		preferred_depth_max_m = std::min(preferred_depth_max_m, 20.0f); // Shallow water
		crush_depth_m = std::min(crush_depth_m, 100.0f); // Can't handle deep pressure
	}

	// REPTILIA: Marine reptiles (sea turtles, marine iguanas, sea snakes)
	if (HasFlag(s.physical.clade, CF::REPTILIA | CF::CHELONIA)) {
		// Reptiles must surface to breathe but can hold breath longer than mammals
		// Lower metabolic rate (ectotherm) → lower O2 consumption

		// Turtles (CHELONIA) have excellent dive duration
		if (HasFlag(s.physical.clade, CF::CHELONIA)) {
			// Sea turtles can dive for hours (low metabolism + anaerobic tolerance)
			preferred_depth_max_m = std::min(preferred_depth_max_m * 2.0f, 300.0f);
		}

		// Reptiles are ectotherms - prefer warmer surface waters
		preferred_depth_min_m = std::max(preferred_depth_min_m, 0.0f); // Surface preference
	}

	// HYDRODYNAMIC CALCULATIONS
	// Calculate lift generation per meter swam (for negatively buoyant animals)
	float lift_per_meter_N = 0.0f;
	float sink_rate_m_s = 0.0f;

	if(body_density_kg_m3 > fluid_density)
	{
		// Negatively buoyant - sinks without swimming
		float weight_in_water_N = (body_density_kg_m3 - fluid_density) * body_volume_m3 * in.environment.gravity_m_s2;

		// Sink rate from terminal velocity: v = sqrt(2*mg / (ρ*Cd*A))
		sink_rate_m_s = std::sqrt((2.0f * weight_in_water_N) /
			(fluid_density * drag_coefficient * cross_section_m2));

		// Lift per meter: needs to generate enough lift to balance weight over distance
		// Simplified: lift_per_meter ≈ weight / glide_ratio
		// For fish without dedicated lifting surfaces, this is about supporting weight
		lift_per_meter_N = weight_in_water_N;
	}

	// Reynolds number at cruise speed
	float characteristic_length = body_length_m;
	float reynolds = (fluid_density * cruise_speed_m_s * characteristic_length) / in.environment.fluidViscosity_Pa_s;

	return Output_Aquatic{
		.propulsors = shared_array<Output_Aquatic::Fin>::FromArray(fins),
		.body_wave = body_wave,
		.cruise_speed_m_s = cruise_speed_m_s,
		.burst_speed_m_s = burst_speed_m_s,
		.min_swim_speed_m_s = min_swim_speed_m_s,
		.lift_per_meter_swam_N = lift_per_meter_N,
		.sink_rate_m_s = sink_rate_m_s,
		.reynolds_number = reynolds,
		.drag_coefficient = drag_coefficient,
		.tail_amplitude_m = tail_amplitude_m,
		.beat_frequency_Hz = beat_frequency_Hz,
		.neutral_buoyancy_density_kg_m3 = neutral_buoyancy_density,
		.swim_bladder_adjust_time_s = swim_bladder_adjust_time_s,
		.primary_mode = primary_mode,
		.has_swim_bladder = has_swim_bladder,
		.can_hover = can_hover,
		.requires_constant_motion = requires_constant_motion,
		.min_turning_radius_m = min_turning_radius_m,
		.preferred_depth_min_m = preferred_depth_min_m,
		.preferred_depth_max_m = preferred_depth_max_m,
		.crush_depth_m = crush_depth_m,
		.c_start = c_start,
		.jet_propulsion = jet_propulsion
	};
}

std::vector<TonTon::Output_Aquatic::Fin> TonTon::Compute_Fins(const Input &in, Scratch &s)
{
	auto & sk = *in.skinnedMesh;
	auto * sk_memo = sk.skin->memo();
	auto position = sk.skin->position.data();
	auto tails = sk->GetBoneTails();
	auto semantic_flags = sk_memo->GetSemanticFlags();
	auto relative_flags = sk_memo->GetRelativeFlags();
	
	// get limbs and fins!
	auto fin_chains = GetChainsFromRoot(in, SF::LIMB|SF::FIN);
	auto fin_appendages = GetAppendages(in, std::move(fin_chains));
	double area_scale = in.behavior.area_scale();
		
	std::vector<TonTon::Output_Aquatic::Fin>  fins;
	fins.reserve(fin_appendages.size());
	
	for(auto & appendage : fin_appendages)
	{
		appendage.rest_length_m += glm::length(tails[appendage.tip] - position[appendage.root]);
		appendage.stretched_length_m += glm::length(tails[appendage.tip] - position[appendage.tip]);
		
		Output_Aquatic::Fin fin;
		static_cast<Output_Appendage&>(fin) = appendage;

		SkinnedMesh::LimbMetrics limb_metrics;
		std::pair<glm::quat, glm::vec3> eigen_decomposition;
	
		// Get fin area from silhouette
		auto fin_joints = sk_memo->GetAllChildrenOfRoot(appendage.root);
		auto projection = in.skinnedMesh->memo()->GetProjectionMatrix(
			EigenValue::Small, 
			in.behavior.scale,
			fin_joints, 
			&limb_metrics, 
			&eigen_decomposition);
			
		auto& silhouette = in.skinnedMesh->memo()->GetSilhouettes(projection, in.behavior.scale, fin_joints);
		
		fin.area_m2 = silhouette.area;
		fin.chord_m = silhouette.MeasureWidth_Segment(position[fin.root], tails[fin.tip]).length;
		
		// aspect ratio wasn't really meaningful because a shark's body is so fat!

		fin.normal_vector = GetProjectionDirection(EigenValue::Small, eigen_decomposition.first);
		fin.type = relative_flags[appendage.root].child_flags|semantic_flags[appendage.root];
		auto surfaceArea = in.skinnedMesh->GetSurfaceArea(fin_joints, area_scale);

		// Add this analysis before your flipper check
		float percent_area = (silhouette.area*2.0) / surfaceArea; // volume-to-area ratio
	//	float thickness = surfaceArea / limb_metrics.volume;
		int joint_count = fin_joints.size();
		float aspect_ratio = fin.rest_length_m / fin.chord_m;
		
		// Flipper characteristics
		bool has_internal_structure = joint_count < 3; // flippers don't have digitgrade etc complexity. usually one bone. 
		bool is_thick_enough = percent_area > 0.75; // tune this value
		bool is_paddle_shaped = aspect_ratio < 3.0f; // flippers are relatively short and wide
		
		bool is_flipper = has_internal_structure && is_thick_enough && is_paddle_shaped;

		if(HasFlag(fin.type, SF::FIN) 
		|| is_flipper)
			fins.push_back(fin);
	}
	
	return fins;
}
		
