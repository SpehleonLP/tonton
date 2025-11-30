#include "tonton_serpentine.h"
#include "tonton_builder.h"
#include "tonton_input.h"
#include "tonton_scratch.h"
#include "Memos/tonton_armaturememo.h"
#include "Memos/tonton_skinnedmeshmemo.h"
#include "../../include/tonton_skinnedmesh.h"
#include <cmath>
#include <algorithm>

namespace TonTon
{

using SF = SemanticFlags;

// Helper to determine if creature can use serpentine locomotion
static bool CanUseSerpentineLocomotion(Input const&, Scratch const& s)
{
	if(s.appendages.tails == nullptr)
	{
		return false;
	}

/*
	// Check for tail with propulsion capability
	bool has_propulsive_tail = false;
	for(auto const& tail : s.appendages.tails) {
		if(tail.used_for & Analysis_Tail::Propulsion) {
			has_propulsive_tail = true;
			break;
		}
	}

	if(!has_propulsive_tail)
		return false;
	*/

	// Serpentine locomotion requires either:
	// 1. No legs at all (true snakes)
	// 2. Very short/weak legs relative to body (skinks, lamias)
	// 3. Tail length >> leg length (can undulate even with arms/hands)

	bool has_terrestrial = s.terrestrial.has_value();
	if(!has_terrestrial) {
		// No terrestrial locomotion at all - serpentine is the way
		// (e.g., eels, sea snakes - they're aquatic but still serpentine)
		return true;
	}

	// CHECK FLEXIBILITY: For TERRESTRIAL serpentine locomotion, check body flexibility
	// Use tail cross-section to estimate flexibility (assuming cylindrical tail)
	// Flexible creatures (snakes, eels) have high length/diameter ratios
	// Rigid creatures (sharks, most fish) have low ratios
	// NOTE: This check only applies to terrestrial locomotion, not aquatic anguilliform swimming

	if(s.appendages.tails.size() > 0) {
		auto const& tail = s.appendages.tails[0];

		// Serpentine locomotion requires a minimum tail/body length to undulate
		// Penguins, chickens, etc. with vestigial tails (< 5% body length) cannot undulate
		auto min_tail_length = s.physical.body_length_m * 0.05f;
		if(tail.stretched_length_m < min_tail_length) {
			return false; // Tail too short for serpentine motion
		}

		// Calculate tail diameter from cross-section (assume circular)
		// A = πr² → r = sqrt(A/π) → diameter = 2r
		auto cross_section_m2 = tail.max_cross_section_m2;
		if(cross_section_m2 > 0.0001f) {
			auto tail_diameter_m = 2.0f * sqrt(cross_section_m2 / M_PI);
			auto tail_aspect_ratio = tail.stretched_length_m / tail_diameter_m;

			// Snakes: aspect ratio ~40-80 (very flexible)
			// Lizards: aspect ratio ~10-20 (moderately flexible)
			// Fish/Sharks: aspect ratio ~3-8 (rigid)
			// Threshold: need aspect ratio > 15 for TERRESTRIAL serpentine locomotion
			if(tail_aspect_ratio < 15.0f) {
				return false; // Too rigid for terrestrial serpentine motion
			}
		}
	}

	// If has legs, check if they're dominant or subordinate to tail
	auto const& terr = s.terrestrial.value();

	if(terr.legs.empty()) {
		return true; // Legless
	}

	// Check leg contribution vs tail length
	length_m total_leg_length = 0.0f;
	for(auto const& leg : terr.legs) {
		total_leg_length += leg.stretched_length_m;
	}
	auto avg_leg_length = total_leg_length / terr.legs.size();

	// Serpentine is viable if tail >> legs (lamia pattern)
	// or if body is very long relative to legs (lizard pattern)
	auto body_to_leg_ratio = s.physical.body_length_m / std::max(avg_leg_length, length_m(0.01f));

	// Serpentine locomotion is possible if:
	// - Body length > 5× leg length (skink-like)
	// - Or tail is marked for propulsion (explicit serpentine design)
	return (body_to_leg_ratio > 5.0f);
}

// Calculate body wave parameters for lateral undulation
// Based on Gray (1936), Garland (1994)
static Analysis_BodyWave CalculateLateralUndulation(Input const& in, Scratch const& s)
{
	Analysis_BodyWave wave{};
	
	in.builder->bodyWave->copy_into(wave, in.scale);

	auto body_length = wave.stretched_length_m;

	// WAVELENGTH: Typically 0.6-1.0 body lengths for terrestrial snakes
	// Shorter wavelength = more push points but more energy
	// Influenced by body flexibility and substrate
	auto wavelength_factor = 0.6f + 0.3f * in.muscle_quality; // 0.6-0.9 BL
	wave.wavelength_ratio = wavelength_factor;

	// AMPLITUDE: Typically 0.1-0.2 body lengths at the widest point
	// Larger amplitude = more lateral force but requires more flexibility
	auto amplitude_factor = 0.1f + 0.1f * in.muscle_quality; // 0.1-0.2 BL
	wave.amplitude_ratio = amplitude_factor;

	// BODY FLEXIBILITY: How "eel-like" vs "rigid" the motion is
	// Based on vertebral count and tail length relative to body
	auto tail_ratio = s.physical.tail_length_m / body_length;
	wave.body_flexibility = glm::clamp(tail_ratio * 1.5f, 0.0f, 1.0f);

	// Amplitude increases parabolically from head to tail
	// Gray (1936): amplitude = a₀ + a₁×x + a₂×x²
	// Most snakes have increasing amplitude toward tail
	wave.segment_amplitude.constant = 0.3f;
	wave.segment_amplitude.linear = 0.2f;
	wave.segment_amplitude.geometric = 0.5f; // Quadratic dominates

	return wave;
}

std::optional<Analysis_Serpentine> ComputeSerpentine(Input const& in, Scratch &s)
{
	if(!CanUseSerpentineLocomotion(in, s)
	|| !in.builder->bodyWave) {
		return {}; // Not capable of serpentine locomotion
	}

	Analysis_Serpentine result{};

	// ========================================================================
	// FRICTION PROPERTIES
	// ========================================================================
	// Snakes rely on friction anisotropy: high friction perpendicular to body,
	// low friction parallel to body (scales create directional grip)
	// Garland (1994), Gray (1936)

	// Forward friction (parallel to body): smooth scales
	result.forward_friction_coef = 0.3f; // Smooth sliding

	// Lateral friction (perpendicular): ventral scales catch
	// Quality affects scale development and grip
	auto lateral_base = 0.6f;
	auto lateral_bonus = 0.4f * in.muscle_quality; // Better development = better scales
	result.lateral_friction_coef = lateral_base + lateral_bonus;

	// Anisotropy ratio: critical for serpentine locomotion
	// Typical range: 1.5-3.0 for snakes
	result.friction_anisotropy_ratio = result.lateral_friction_coef / result.forward_friction_coef;

	// ========================================================================
	// LATERAL UNDULATION (primary terrestrial mode)
	// ========================================================================
	result.lateral_undulation = CalculateLateralUndulation(in, s);

	auto body_length_m = s.physical.body_length_m;
	auto body_mass_kg = s.physical.body_mass_kg;

	// Speed calculation based on wave mechanics
	// Speed ≈ wavelength × frequency
	// Frequency scales with size: f ∝ M^(-1/3) (similar to swimming)
	freq_Hz frequency_base_Hz = 2.0f / std::pow(float(body_mass_kg), 0.33f);

	// Muscle quality affects sustainable frequency
	auto frequency_Hz = frequency_base_Hz * glm::mix(0.7f, 1.3f, in.muscle_quality);

	auto wavelength_m = result.lateral_undulation.wavelength_ratio * body_length_m;
	auto lateral_speed_m_s = wavelength_m * frequency_Hz;

	// Efficiency depends on friction anisotropy and substrate
	// Better anisotropy = faster movement
	auto efficiency = glm::clamp(result.friction_anisotropy_ratio / 3.0f, 0.3f, 1.0f);
	lateral_speed_m_s *= efficiency;

	// ========================================================================
	// CAPABLE MODES
	// ========================================================================
	result.capable_modes = Analysis_Serpentine::Mode::LATERAL_UNDULATION;

	// RECTILINEAR: Slow, stealthy "caterpillar" crawl
	// Requires long body and good muscle control
	// Used by large constrictors (boas, pythons)
	if(body_mass_kg > 2.0f && body_length_m > 1.0f) {
		result.capable_modes = Analysis_Serpentine::Mode(
			int(result.capable_modes) | int(Analysis_Serpentine::Mode::RECTILINEAR)
		);

		Analysis_Serpentine::Rectilinear recti;
		// Rectilinear is very slow: ~0.02-0.05 m/s for most snakes
		recti.speed_m_s = freq_Hz(0.02f) * body_length_m;
		recti.frequency_Hz = recti.speed_m_s / body_length_m;

		result.rectilinear = recti;
	}

	// SIDEWINDING: Desert specialist mode
	// Requires very flexible body and works best on loose substrate
	// Reduces contact area on hot surfaces
	auto flexibility_threshold = 0.6f;
	if(result.lateral_undulation.body_flexibility > flexibility_threshold) {
		result.capable_modes = Analysis_Serpentine::Mode(
			int(result.capable_modes) | int(Analysis_Serpentine::Mode::SIDEWINDING)
		);

		Analysis_Serpentine::SideWinding side;
		// Sidewinding can be faster than lateral undulation on loose surfaces
		side.speed_m_s = lateral_speed_m_s * 1.2f;
		side.frequency_Hz = frequency_Hz;
		side.amplitude_ratio = result.lateral_undulation.amplitude_ratio * 1.5f;

		// Only 2 contact points touching ground at once (middle of S-curves)
		side.contact_points = 2;

		result.sidewinding = side;
	}

	// CONCERTINA: Climbing and tight spaces
	// Requires ability to form tight loops (high flexibility)
	// Used in burrows, pipes, climbing
	if(result.lateral_undulation.body_flexibility > 0.5f) {
		result.capable_modes = Analysis_Serpentine::Mode(
			int(result.capable_modes) | int(Analysis_Serpentine::Mode::CONCERTINA)
		);

		Analysis_Serpentine::Concertina conc;
		// Concertina is slow but powerful
		// Speed ≈ body_length / (2 × cycle_time)
		auto cycle_time_s = 2.0f / frequency_Hz; // Slower than undulation
		conc.speed_m_s = body_length_m / (2.0f * cycle_time_s);

		// Compression ratio: how much the body compresses during anchor phase
		// Typical range: 0.3-0.5 (compress to 30-50% of stretched length)
		conc.compression_ratio = 0.4f;

		result.concertina = conc;
	}

	// ========== CLADE-SPECIFIC REFINEMENTS ==========
	using CF = CladeFlags;
	auto clade = s.physical.clade;

	// REPTILIA: True snakes - specialized for terrestrial undulation
	if (HasFlag(clade, CF::REPTILIA)) {
		// Snakes have ventral scales optimized for friction anisotropy
		// Gray (1946): Ventral scales catch substrate during lateral push
		// Hu et al. (2009): Friction ratio 1.5-4.0 depending on scale morphology

		result.lateral_friction_coef = std::min(result.lateral_friction_coef * 1.3f, 1.2f);
		result.friction_anisotropy_ratio = result.lateral_friction_coef / result.forward_friction_coef;

		// Snakes excel at concertina and rectilinear (specialized ribs and muscles)
		// Jayne (1986): Specialized costal muscles for rectilinear crawling
		if (result.rectilinear.has_value()) {
			result.rectilinear->speed_m_s *= 1.4f; // More efficient than generic
		}

		if (result.concertina.has_value()) {
			result.concertina->speed_m_s *= 1.3f; // Better muscle coordination
			result.concertina->compression_ratio = 0.3f; // Can compress tighter
		}

		// Many snakes adapted for specific substrates
		// Desert snakes (sidewinding specialists)
		// Marvi & Hu (2012): Sidewinding optimal on granular media
		//  && in.behavior.habitat_type == HabitatType::DESERT
		if (result.sidewinding.has_value()) {
			result.sidewinding->speed_m_s *= 1.5f; // Extremely efficient on sand
		}

		// Aquatic snakes (sea snakes, water snakes) have compressed tails
		if (s.aquatic.has_value()) {
			// Lateral undulation works well in water (anguilliform swimming)
			result.lateral_undulation.wavelength_ratio *= 0.8f; // Shorter wavelength
			result.lateral_undulation.amplitude_ratio *= 1.2f; // Larger amplitude
		}
	}

	// PISCES: Eel-like fish - undulation in water
	if (HasFlag(clade, CF::PISCES) && s.aquatic.has_value()) {
		// Eels use anguilliform swimming (body undulation)
		// Gillis (1998): Eel undulation wave travels at 1.5-2x swimming speed
		// Very different mechanics than terrestrial snakes

		// Friction anisotropy less important in water (viscous drag dominates)
		result.friction_anisotropy_ratio = 1.0f; // Isotropic in water

		// Eel undulation parameters
		// Lauder & Tytell (2006): Eels use wavelength ~0.6-0.8 BL
		result.lateral_undulation.wavelength_ratio = 0.7f;
		result.lateral_undulation.amplitude_ratio = 0.15f; // Moderate amplitude

		// Speed calculation for aquatic undulation
		// Webb (1975): Speed = wavelength × frequency × slip_factor
		// Slip factor ~0.7-0.9 (some backward slipping in water)
		freq_Hz frequency_Hz = 2.0f / std::pow(float(s.physical.body_mass_kg), 0.33f);
		auto wavelength_m = result.lateral_undulation.wavelength_ratio * body_length_m;
		auto aquatic_speed = wavelength_m * frequency_Hz * 0.8f; // 80% slip efficiency

		// Eels are efficient swimmers
		// Van Ginneken & Van Den Thillart (2000): Eels migrate 6000km
		result.lateral_undulation_speed_m_s = aquatic_speed * 1.2f;

		// Concertina and sidewinding don't work underwater
		result.concertina = std::nullopt;
		result.sidewinding = std::nullopt;
		result.capable_modes = Analysis_Serpentine::Mode::LATERAL_UNDULATION;
	}

	// AMPHIBIA: Salamanders and caecilians
	if (HasFlag(clade, CF::AMPHIBIA)) {
		// Salamanders use lateral undulation with legs
		// Frolich & Biewener (1992): Salamander walking + undulation hybrid

		if (s.terrestrial.has_value() && !s.terrestrial->legs.empty()) {
			// Legs assist undulation (not pure serpentine)
			result.lateral_undulation_speed_m_s *= 0.7f; // Less efficient than snakes
			result.lateral_undulation.amplitude_ratio *= 0.7f; // Reduced amplitude
		} else {
			// Legless caecilians (fossorial)
			// Gans (1973): Concertina locomotion in burrows
			if (result.concertina.has_value()) {
				result.concertina->speed_m_s *= 1.2f; // Well-adapted for burrowing
			}
		}

		// Amphibians excel in aquatic undulation when swimming
		if (s.aquatic.has_value()) {
			result.lateral_undulation_speed_m_s *= 1.3f; // Good swimmers
		}
	} 

	// ANNELIDA: Worms (if we ever add this clade)
	// Earthworms use peristaltic waves (more like concertina)
	// Would need ANNELIDA added to CladeFlags first

	// MOLLUSCA: Gastropods with muscular feet
	// Some sea slugs use body undulation
	if (HasFlag(clade, CF::MOLLUSCA) && s.aquatic.has_value()) {
		// Nudibranchs swim via lateral flexion
		// Newcomb et al. (2012): Nudibranch swimming 0.01-0.05 m/s
		result.lateral_undulation_speed_m_s *= 0.3f; // Very slow (muscular hydrostats)
		result.lateral_undulation.wavelength_ratio = 0.5f; // Short wavelength
	}

	return result;
}

} // namespace TonTon
