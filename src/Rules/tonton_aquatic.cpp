#include "tonton_aquatic.h"
#include "dodeedum.h"
#include "tonton_output.h"
#include "tonton_input.h"
#include "Rules/tonton_scratch.h"
#include "Memos/tonton_armaturememo.h"
#include "Memos/tonton_skinnedmeshmemo.h"
#include <cfloat>
#include <cmath>

using SF = TonTon::SemanticFlags;

std::optional<TonTon::Output_Aquatic>   TonTon::ComputeAquatic(Input const& in, Scratch & s)
{
	auto & sk = *in.skinnedMesh;
	auto * sk_memo = sk.skin->memo();

	auto semantic_flags = sk_memo->GetSemanticFlags();
	auto relative_flags = sk_memo->GetRelativeFlags();

	// 1. DETECT PROPULSION MODE
	// Check for fins, flukes, limbs, or body flexibility

	bool has_fins = false;
	bool has_flukes = false;
	bool has_paddle_limbs = false;
	bool has_flexible_body = false;
	bool has_jet_system = false;

	// Check semantic flags for aquatic structures
	for(auto i = 0u; i < semantic_flags.size(); ++i)
	{
		bool is_fin = HasFlag(semantic_flags[i], SF::FIN);
	
		if(is_fin)
			has_fins = true;

		// Flukes are typically horizontal tail fins (cetaceans)
		if(HasFlag(semantic_flags[i], SF::TAIL) && is_fin)
			has_flukes = true;

		// Paddle limbs: limbs with AQUATIC flag
		if(HasFlag(semantic_flags[i], SF::LIMB) && HasFlag(semantic_flags[i], SF::AQUATIC))
			has_paddle_limbs = true;

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
	else if(has_flukes)
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
		return {};
	}

	// 2. HYDRODYNAMIC PARAMETERS
	float body_mass_kg = s.physical.body_mass_kg;
	float body_length_m = s.physical.body_length_m;
	float body_volume_m3 = s.physical.body_volume_m3;
	float cross_section_m2 = s.physical.cross_sectional_area_m2;

	float fluid_density = in.environment.fluidDensity_Kg_m3;
//	float fluid_viscosity = in.environment.fluidViscosity_Pa_s;

	// 3. BUOYANCY
	float body_density_kg_m3 = body_mass_kg / body_volume_m3;
	float neutral_buoyancy_density = body_density_kg_m3;

	// Most fish have swim bladders to achieve neutral buoyancy
	bool has_swim_bladder = (primary_mode == Output_Aquatic::PropulsionMode::BODY_CAUDAL_FIN ||
	                         primary_mode == Output_Aquatic::PropulsionMode::MEDIAN_PAIRED_FIN);

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

	// Estimate tail beat frequency from body size
	// f ∝ L^(-0.5) for cruising (from fish data)
	float beat_frequency_Hz = 2.0f * std::pow(body_length_m, -0.5f);

	// Tail amplitude as fraction of body length
	float tail_amplitude_ratio = 0.2f; // Typical: 0.1-0.3

	if(primary_mode == Output_Aquatic::PropulsionMode::BODY_CAUDAL_FIN)
	{
		// Flexible swimmers have larger amplitudes
		tail_amplitude_ratio = 0.25f;
	}

	float tail_amplitude_m = body_length_m * tail_amplitude_ratio;

	// Cruise speed from Strouhal: U = f*A/St
	float cruise_speed_m_s = (beat_frequency_Hz * tail_amplitude_m) / strouhal_optimal;

	// Drag-based speed limit
	// Drag: D = 0.5 * ρ * v² * Cd * A
	float drag_coefficient = 0.04f; // Streamlined fish

	if(s.physical.fineness_ratio < 3.0f)
	{
		drag_coefficient = 0.08f; // Less streamlined
	}

	// Available power for swimming
	float swim_power_W = s.metabolic.available_muscle_power_W * 0.3f; // ~30% for swimming

	// Max speed when power = drag * velocity
	// P = D * v = 0.5 * ρ * v³ * Cd * A
	// v = (2*P / (ρ * Cd * A))^(1/3)
	float power_limited_speed = std::pow(
		(2.0f * swim_power_W) / (fluid_density * drag_coefficient * cross_section_m2),
		1.0f / 3.0f
	);

	// Cruise is the lower of Strouhal-optimal or power-limited
	cruise_speed_m_s = std::min(cruise_speed_m_s, power_limited_speed);

	// Burst speed (anaerobic, 2-3x cruise for short duration)
	float burst_speed_m_s = cruise_speed_m_s * 2.5f;

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

	// 8. BUILD PROPULSOR STRUCTURES
	std::vector<Output_Aquatic::Fin> fins;

	// Find fin appendages
	SF constexpr NOT_LIMB_FLAGS = SF(
		int64_t(SF::HEAD)|
		int64_t(SF::NECK)|
		int64_t(SF::SPINE)|
		int64_t(SF::ABDOMEN)
	);

	if(has_fins || has_flukes)
	{
		auto fin_chains = GetChainsFromRoot(in, SF::FIN);
		auto fin_appendages = GetAppendages(in, std::move(fin_chains));

		for(auto const& appendage : fin_appendages)
		{
			Output_Aquatic::Fin fin;
			static_cast<Output_Appendage&>(fin) = appendage;

			// Get fin area from silhouette
			auto fin_joints = sk_memo->GetAllChildrenOfRoot(appendage.root);
			auto& silhouette = in.skinnedMesh->memo()->GetSilhouettes(
				EigenValue::Small,
				in.behavior.scale,
				std::span(fin_joints.data(), fin_joints.size())
			);

			fin.area_m2 = silhouette.area;

			// Aspect ratio: span² / area
			float span = appendage.stretched_length_m;
			fin.aspect_ratio = (span * span) / std::max(fin.area_m2, 0.001f);

			// Animation parameters
			fin.beat_frequency_Hz = beat_frequency_Hz;
			fin.beat_amplitude_rad = tail_amplitude_ratio * M_PI;
			fin.wave_speed_ratio = 1.2f; // Wave travels faster than body

			fin.type = semantic_flags[appendage.root];

			fins.push_back(fin);
		}
	}
	else if(has_paddle_limbs)
	{
		// Use limbs as propulsors
		if(s.terrestrial.has_value())
		{
			for(auto const& leg : s.terrestrial.value().legs)
			{
				Output_Aquatic::Fin fin;
				static_cast<Output_Appendage&>(fin) = leg;

				fin.area_m2 = leg.surface_area_m2;
				fin.aspect_ratio = 2.0f; // Paddles are typically low aspect ratio
				fin.beat_frequency_Hz = beat_frequency_Hz * 0.7f; // Slower than tail
				fin.beat_amplitude_rad = M_PI / 4.0f; // 45-degree stroke
				fin.wave_speed_ratio = 0.0f; // No wave propagation
				fin.type = SF::LIMB;

				fins.push_back(fin);
			}
		}
	}

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
		float max_dive_time_min = O2_storage_L / O2_consumption_L_per_min;

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

	return Output_Aquatic{
		.propulsors = shared_array<Output_Aquatic::Fin>::FromArray(fins),
		.body_wave = body_wave,
		.cruise_speed_m_s = cruise_speed_m_s,
		.burst_speed_m_s = burst_speed_m_s,
		.min_swim_speed_m_s = min_swim_speed_m_s,
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
