#include "tonton_serpentine.h"
#include "tonton_scratch.h"
#include "Memos/tonton_armaturememo.h"
#include "Memos/tonton_skinnedmeshmemo.h"
#include "../../include/tonton_input.h"
#include <cmath>
#include <algorithm>

namespace TonTon
{

using SF = SemanticFlags;

// Helper to determine if creature can use serpentine locomotion
static bool CanUseSerpentineLocomotion(Input const&, Scratch const& s)
{
/*
	// Check for tail with propulsion capability
	bool has_propulsive_tail = false;
	for(auto const& tail : s.appendages.tails) {
		if(tail.used_for & Output_Tail::Propulsion) {
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
		return true;
	}

	// If has legs, check if they're dominant or subordinate to tail
	auto const& terr = s.terrestrial.value();

	if(terr.legs.empty()) {
		return true; // Legless
	}

	// Check leg contribution vs tail length
	float total_leg_length = 0.0f;
	for(auto const& leg : terr.legs) {
		total_leg_length += leg.stretched_length_m;
	}
	float avg_leg_length = total_leg_length / terr.legs.size();

	// Serpentine is viable if tail >> legs (lamia pattern)
	// or if body is very long relative to legs (lizard pattern)
	float body_to_leg_ratio = s.physical.body_length_m / std::max(avg_leg_length, 0.01f);

	// Serpentine locomotion is possible if:
	// - Body length > 5× leg length (skink-like)
	// - Or tail is marked for propulsion (explicit serpentine design)
	return (body_to_leg_ratio > 5.0f);
}

// Calculate body wave parameters for lateral undulation
// Based on Gray (1936), Garland (1994)
static Output_BodyWave CalculateLateralUndulation(Input const& in, Scratch const& s)
{
	Output_BodyWave wave{};

	// Find spine chain from tail to head
	auto & sk = *in.armature;
	auto * memo = sk.armature->memo();
	auto semantic_flags = memo->GetSemanticFlags();
	auto parents = sk.armature->parents.data();

	// Find tail root and spine chain
	int tail_tip = -1;
	int spine_root = s.physical.spine_root;

	for(auto const& tail : s.appendages.tails) {
		tail_tip = tail.tip;
		break; // Use first tail
	}

	if(tail_tip < 0) {
		// Fallback: use body length
		wave.root = spine_root;
		wave.tip = tail_tip;
		wave.noJoints = 0;
		wave.stretched_length_m = s.physical.body_length_m;
		wave.rest_length_m = s.physical.body_length_m * 0.95f;
	} else {
		// Build chain from tail to spine
		std::vector<int> chain;
		for(int j = tail_tip; j >= 0; j = parents[j]) {
			chain.push_back(j);
			if(j == spine_root || HasFlag(semantic_flags[j], SF::HEAD)) {
				break;
			}
		}

		wave.root = chain.empty() ? spine_root : chain.back();
		wave.tip = tail_tip;
		wave.noJoints = chain.size();

		// Calculate chain length
		float length = 0.0f;
		for(size_t i = 1; i < chain.size(); ++i) {
			length += glm::distance(in.position(chain[i]), in.position(chain[i-1]));
		}
		wave.stretched_length_m = length;
		wave.rest_length_m = length * 0.95f;
	}

	float body_length = wave.stretched_length_m;

	// WAVELENGTH: Typically 0.6-1.0 body lengths for terrestrial snakes
	// Shorter wavelength = more push points but more energy
	// Influenced by body flexibility and substrate
	float wavelength_factor = 0.6f + 0.3f * in.muscle_quality; // 0.6-0.9 BL
	wave.wavelength_ratio = wavelength_factor;

	// AMPLITUDE: Typically 0.1-0.2 body lengths at the widest point
	// Larger amplitude = more lateral force but requires more flexibility
	float amplitude_factor = 0.1f + 0.1f * in.muscle_quality; // 0.1-0.2 BL
	wave.amplitude_ratio = amplitude_factor;

	// BODY FLEXIBILITY: How "eel-like" vs "rigid" the motion is
	// Based on vertebral count and tail length relative to body
	float tail_ratio = s.physical.tail_length_m / body_length;
	wave.body_flexibility = glm::clamp(tail_ratio * 1.5f, 0.0f, 1.0f);

	// Amplitude increases parabolically from head to tail
	// Gray (1936): amplitude = a₀ + a₁×x + a₂×x²
	// Most snakes have increasing amplitude toward tail
	wave.segment_amplitude.constant = 0.3f;
	wave.segment_amplitude.linear = 0.2f;
	wave.segment_amplitude.geometric = 0.5f; // Quadratic dominates

	return wave;
}

std::optional<Output_Serpentine> ComputeSerpentine(Input const& in, Scratch &s)
{
	if(!CanUseSerpentineLocomotion(in, s)) {
		return {}; // Not capable of serpentine locomotion
	}

	Output_Serpentine result{};

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
	float lateral_base = 0.6f;
	float lateral_bonus = 0.4f * in.muscle_quality; // Better development = better scales
	result.lateral_friction_coef = lateral_base + lateral_bonus;

	// Anisotropy ratio: critical for serpentine locomotion
	// Typical range: 1.5-3.0 for snakes
	result.friction_anisotropy_ratio = result.lateral_friction_coef / result.forward_friction_coef;

	// ========================================================================
	// LATERAL UNDULATION (primary terrestrial mode)
	// ========================================================================
	result.lateral_undulation = CalculateLateralUndulation(in, s);

	float body_length_m = s.physical.body_length_m;
	float body_mass_kg = s.physical.body_mass_kg;

	// Speed calculation based on wave mechanics
	// Speed ≈ wavelength × frequency
	// Frequency scales with size: f ∝ M^(-1/3) (similar to swimming)
	float frequency_base_Hz = 2.0f / std::pow(body_mass_kg, 0.33f);

	// Muscle quality affects sustainable frequency
	float frequency_Hz = frequency_base_Hz * glm::mix(0.7f, 1.3f, in.muscle_quality);

	float wavelength_m = result.lateral_undulation.wavelength_ratio * body_length_m;
	float lateral_speed_m_s = wavelength_m * frequency_Hz;

	// Efficiency depends on friction anisotropy and substrate
	// Better anisotropy = faster movement
	float efficiency = glm::clamp(result.friction_anisotropy_ratio / 3.0f, 0.3f, 1.0f);
	lateral_speed_m_s *= efficiency;

	// ========================================================================
	// CAPABLE MODES
	// ========================================================================
	result.capable_modes = Output_Serpentine::Mode::LATERAL_UNDULATION;

	// RECTILINEAR: Slow, stealthy "caterpillar" crawl
	// Requires long body and good muscle control
	// Used by large constrictors (boas, pythons)
	if(body_mass_kg > 2.0f && body_length_m > 1.0f) {
		result.capable_modes = Output_Serpentine::Mode(
			int(result.capable_modes) | int(Output_Serpentine::Mode::RECTILINEAR)
		);

		Output_Serpentine::Rectilinear recti;
		// Rectilinear is very slow: ~0.02-0.05 m/s for most snakes
		recti.speed_m_s = 0.02f * body_length_m;
		recti.frequency_Hz = recti.speed_m_s / body_length_m;

		result.rectilinear = recti;
	}

	// SIDEWINDING: Desert specialist mode
	// Requires very flexible body and works best on loose substrate
	// Reduces contact area on hot surfaces
	float flexibility_threshold = 0.6f;
	if(result.lateral_undulation.body_flexibility > flexibility_threshold) {
		result.capable_modes = Output_Serpentine::Mode(
			int(result.capable_modes) | int(Output_Serpentine::Mode::SIDEWINDING)
		);

		Output_Serpentine::SideWinding side;
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
		result.capable_modes = Output_Serpentine::Mode(
			int(result.capable_modes) | int(Output_Serpentine::Mode::CONCERTINA)
		);

		Output_Serpentine::Concertina conc;
		// Concertina is slow but powerful
		// Speed ≈ body_length / (2 × cycle_time)
		float cycle_time_s = 2.0f / frequency_Hz; // Slower than undulation
		conc.speed_m_s = body_length_m / (2.0f * cycle_time_s);

		// Compression ratio: how much the body compresses during anchor phase
		// Typical range: 0.3-0.5 (compress to 30-50% of stretched length)
		conc.compression_ratio = 0.4f;

		result.concertina = conc;
	}

	return result;
}

} // namespace TonTon
