#include "tonton_metabolic.h"
#include "tonton_analysis.h"
#include "tonton_builder.h"
#include "tonton_input.h"
#include "tonton_skinnedmesh.h"
#include "Rules/tonton_scratch.h"
#include <cmath>
#include <vector>

using CF = TonTon::CladeFlags;

TonTon::Analysis_Metabolic TonTon::ComputeMetabolic(Input const& in, Scratch & s)
{
	auto body_mass_kg = s.physical.body_mass_kg;

	// Multi-clade blending structure for hybrid creatures (pegasus, griffin, merfolk, etc.)
	struct CladeContribution {
		float weight;           // Relative importance of this clade
		float rmr_coef;         // Coefficient in RMR = coef * M^exp
		float rmr_exp;          // Exponent in RMR = coef * M^exp
		cost_W_kg muscle_W_kg;      // Sustainable muscle power density
		temp_K body_temp_K;      // Body temperature (if endotherm)
		bool is_endotherm;      // Thermal strategy
	};
	std::vector<CladeContribution> contributions;

	auto const& clade = s.physical.clade;

	// ========== DETECT ENDOTHERMY REQUIREMENT ==========
	// Flight or high-speed running requires endothermy for sustained high power output

	bool needs_endothermy = false;

	// Check for functional wings (flight requires high metabolic rate)
	if (s.aerial.has_value() && HasFlag(clade, CF::CHORDATA)) {
		needs_endothermy = true;
	}

	// Birds and mammals are obligate endotherms
	if (HasFlag(clade, CF::AVES) || HasFlag(clade, CF::MAMMALIA)) {
		needs_endothermy = true;
	}

	// ========== GATHER CLADE CONTRIBUTIONS ==========

	// AVES: Highest metabolic rate
	// Lasiewski & Dawson (1967): RMR = 6.25 * M^0.72 for passerines
	// Aschoff & Pohl (1970): Similar scaling confirmed across 92 species
	// Ellington et al. (1990): Flight muscle: 200-400 W/kg sustained power
	if (HasFlag(clade, CF::AVES)) {
		contributions.push_back({
			.weight = 1.0f,
			.rmr_coef = 6.25f,
			.rmr_exp = 0.72f,
			.muscle_W_kg = (200.0f + 200.0f * in.muscle_quality),      // High-performance flight muscle
			.body_temp_K = 313.15f,     // 40°C (birds run hot)
			.is_endotherm = true
		});
	}

	// MAMMALIA: Standard endotherm
	// Kleiber (1932): RMR = 4.18 * M^0.75 ("Kleiber's Law", R² = 0.99)
	// Validated by Savage et al. (2004) across 619 species
	// Rome et al. (1988): Mammalian muscle 200 W/kg sustained
	if (HasFlag(clade, CF::MAMMALIA)) {
		contributions.push_back({
			.weight = 1.0f,
			.rmr_coef = 4.18f,
			.rmr_exp = 0.75f,
			.muscle_W_kg = 200.0f,
			.body_temp_K = 310.15f,     // 37°C
			.is_endotherm = true
		});
	}

	// EQUIDAE: Specialized ungulates (mammal subgroup)
	// Horses have exceptional running muscle performance
	if (HasFlag(clade, CF::EQUIDAE)) {
		contributions.push_back({
			.weight = 1.5f,             // Extra weight for specialized adaptation
			.rmr_coef = 4.18f,
			.rmr_exp = 0.75f,
			.muscle_W_kg = 220.0f,      // Excellent running endurance
			.body_temp_K = 310.15f,
			.is_endotherm = true
		});
	}

	// CETACEA: Marine mammals (whale, dolphin)
	// Similar to terrestrial mammals but adapted for swimming
	if (HasFlag(clade, CF::CETACEA)) {
		contributions.push_back({
			.weight = 1.0f,
			.rmr_coef = 4.18f,
			.rmr_exp = 0.75f,
			.muscle_W_kg = 200.0f,
			.body_temp_K = 310.15f,
			.is_endotherm = true
		});
	}

	// PISCES: Fish (ectotherm with good muscle performance)
	// Killen et al. (2016): RMR = 0.8 * M^0.80 across 131 teleost species
	// Clarke & Johnston (1999): Scaling exponent 0.79-0.82 typical
	// Rome et al. (1988): Red muscle 200 W/kg, white muscle 250-500 W/kg burst
	// Wardle (1975): White muscle peak power 400 W/kg validated
	if (HasFlag(clade, CF::PISCES)) {
		contributions.push_back({
			.weight = 1.0f,
			.rmr_coef = 0.8f,           // Low ectotherm basal rate
			.rmr_exp = 0.80f,
			.muscle_W_kg = (200.0f + 50.0f * in.muscle_quality),      // Red muscle sustained (white burst handled in aquatic rules)
			.body_temp_K = in.environment.temperature_K,
			.is_endotherm = false
		});
	}

	// REPTILIA: Reptiles (ectotherm)
	// Andrews & Pough (1985): RMR = 0.5 * M^0.80 for reptiles at 20°C
	// Bennett & Dawson (1976): Exponent 0.79-0.83 across 37 lizard species
	// Lower muscle performance than mammals (ectotherm constraint)
	if (HasFlag(clade, CF::REPTILIA) || HasFlag(clade, CF::CHELONIA)) {
		contributions.push_back({
			.weight = 1.0f,
			.rmr_coef = 0.5f,
			.rmr_exp = 0.80f,
			.muscle_W_kg = 100.0f + 100.0f * in.muscle_quality,
			.body_temp_K = in.environment.temperature_K,
			.is_endotherm = false
		});
	}

	// AMPHIBIA: Amphibians (ectotherm)
	// Similar to reptiles but generally lower performance
	if (HasFlag(clade, CF::AMPHIBIA)) {
		contributions.push_back({
			.weight = 1.0f,
			.rmr_coef = 0.5f,
			.rmr_exp = 0.80f,
			.muscle_W_kg = 100.0f + 40.0f * in.muscle_quality,
			.body_temp_K = in.environment.temperature_K,
			.is_endotherm = false
		});
	}

	// ARTHROPODA: Arthropods (ectotherm, different scaling exponent)
	// Addo-Bediako et al. (2002): RMR = 4.14 * M^0.66 (R² = 0.90, F₁,₅₉ = 544.9)
	// Chown et al. (2007): Confirmed 0.66 exponent across terrestrial arthropods
	// Niven & Scharlemann (2005): FMR = 35.08 * M^1.10 for flying insects (R² = 0.95)
	// Note: Different exponent from vertebrates due to tracheal respiratory system
	if (HasFlag(clade, CF::ARTHROPODA)) {
		// Flying insects have higher metabolic coefficients
		bool is_flying_insect = false;
		
		for(auto ap : in.builder->appendages)
		{
			if(HasFlag(ap.semantic_flags, SemanticFlags::WING))
			{
				is_flying_insect = true;
				break;
			}
		}

		// Flying insect muscle power: Odonata (dragonflies) achieve 200-400 W/kg
		// Ellington (1985): flight muscle power ~250 W/kg at 25°C
		// Higher than non-flying arthropods (100 W/kg)
		auto insect_muscle_power = is_flying_insect ?
			(200.0f + 100.0f * in.muscle_quality) :  // 200-300 W/kg for fliers
			100.0f;

		contributions.push_back({
			.weight = 1.0f,
			.rmr_coef = 4.14f,
			.rmr_exp = 0.66f,           // Different scaling exponent from vertebrates
			.muscle_W_kg = insect_muscle_power,
			.body_temp_K = in.environment.temperature_K,
			.is_endotherm = false
		});
	}

	// MOLLUSCA: Mollusks (ectotherm, very low metabolic rate)
	// Gastropods, bivalves, cephalopods
	if (HasFlag(clade, CF::MOLLUSCA)) {
		contributions.push_back({
			.weight = 1.0f,
			.rmr_coef = 0.3f,           // Very low metabolic rate
			.rmr_exp = 0.75f,
			.muscle_W_kg = 100.0f,
			.body_temp_K = in.environment.temperature_K,
			.is_endotherm = false
		});
	}

	// CEPHALOPODA: Octopus, squid (more active than other mollusks)
	if (HasFlag(clade, CF::CEPHALOPODA)) {
		contributions.push_back({
			.weight = 1.5f,             // More active than general mollusks
			.rmr_coef = 0.5f,
			.rmr_exp = 0.75f,
			.muscle_W_kg = 120.0f,
			.body_temp_K = in.environment.temperature_K,
			.is_endotherm = false
		});
	}

	// ========== FALLBACK: No clade specified ==========
	// Use morphological cues when clade detection from bone names fails
	if (contributions.empty()) {
		// Count wings to detect insect-like creatures
		int wing_count = 0;
		for (auto& ap : in.builder->appendages) {
			if (HasFlag(ap.semantic_flags, SemanticFlags::WING)) {
				wing_count++;
			}
		}

		// 4+ wings + small mass = likely insect (dragonfly, butterfly, etc.)
		// 2 wings + small mass = could be bird or bat or 2-winged insect
		bool likely_insect = (wing_count >= 4) ||
			(wing_count >= 2 && float(body_mass_kg) < 0.01f && !HasFlag(clade, CF::CHORDATA));

		if (likely_insect) {
			// Use insect-like metabolics
			// Flying insect muscle: 200-300 W/kg (Ellington 1985)
			auto insect_muscle_power = wing_count > 0 ?
				(200.0f + 100.0f * in.muscle_quality) :
				100.0f;

			contributions.push_back({
				.weight = 1.0f,
				.rmr_coef = 4.14f,       // Arthropod scaling (Addo-Bediako 2002)
				.rmr_exp = 0.66f,        // Different from vertebrates
				.muscle_W_kg = insect_muscle_power,
				.body_temp_K = in.environment.temperature_K,
				.is_endotherm = false
			});
		} else {
			// Generic ectotherm fallback
			contributions.push_back({
				.weight = 1.0f,
				.rmr_coef = 1.0f,
				.rmr_exp = 0.75f,
				.muscle_W_kg = 150.0f,
				.body_temp_K = in.environment.temperature_K,
				.is_endotherm = false
			});
		}
	}

	// ========== WEIGHTED BLEND FOR HYBRIDS ==========
	// Example: Pegasus (EQUIDAE + AVES) blends horse body with bird wings

	auto total_weight = 0.0f;
	for (auto const& c : contributions) {
		total_weight += c.weight;
	}

	auto rmr_coefficient = 0.0f;
	auto rmr_exponent = 0.0f;
	cost_W_kg muscle_power_W_kg = 0.0f;
	temp_K body_temperature_K = 0.0f;

	for (auto const& c : contributions) {
		auto w = c.weight / total_weight;
		rmr_coefficient += c.rmr_coef * w;
		rmr_exponent += c.rmr_exp * w;
		muscle_power_W_kg += c.muscle_W_kg * w;
		body_temperature_K += c.body_temp_K * w;
	}
	
	rmr_coefficient   *= std::exp2(in.mana.fire);
	muscle_power_W_kg *= std::exp2(in.mana.fire);

	// ========== UPGRADE TO ENDOTHERM IF REQUIRED ==========
	// If creature needs endothermy (flight/fast running) but all clades were ectotherms,
	// force endotherm status (e.g., dragon with reptile clade but functional wings)

	if (needs_endothermy && body_temperature_K < 300.0f) {
		body_temperature_K = 310.15f;   // Force endotherm body temp
		rmr_coefficient *= 5.0f;         // Boost to endotherm levels (~5x ectotherm)
		muscle_power_W_kg = std::max<cost_W_kg>(muscle_power_W_kg, 200.0f);
	}

	// ========== COMPUTE FINAL METABOLIC RATES ==========

	// Basal metabolic rate (RMR = coefficient * M^exponent)
	power_W basal_rate_W = rmr_coefficient * std::pow(float(body_mass_kg), rmr_exponent);

	// Aerobic scope (max/basal ratio)
	// Endotherms: 5-15x (typical 10x)
	// Ectotherms: 2-8x (typical 5x)
	auto aerobic_scope = needs_endothermy ? 10.0f : 5.0f;

	// Maximum metabolic rate
	auto max_rate_W = basal_rate_W * aerobic_scope;

	// Muscle mass (typically 35-45% of body mass)
	// Higher for specialized athletes (horses, cheetahs)
	auto muscle_fraction = 0.40f;
	if (HasFlag(clade, CF::EQUIDAE)) {
		muscle_fraction = 0.45f; // Horses are exceptionally muscular
	}
	if (HasFlag(clade, CF::AVES) && s.aerial.has_value()) {
		muscle_fraction = 0.35f; // Birds sacrifice muscle for lightness
	}

	auto muscle_mass_kg = body_mass_kg * muscle_fraction * (0.75f + 0.50f * in.stability_vs_speed);

	// Available muscle power (muscle_mass * power_density)
	auto available_muscle_power_W = muscle_mass_kg * muscle_power_W_kg;

	// Temperature regulation zone (for endotherms)
	temp_K thermal_neutral_zone_min_K = -1.0f;
	temp_K thermal_neutral_zone_max_K = -1.0f;

	if (needs_endothermy) {
		// Thermal neutral zone: ±5-15°C around body temperature
		// Larger animals have narrower zones (better thermal inertia)
		temp_K zone_width_K = 15.0f * std::pow(float(body_mass_kg), -0.2f);
		zone_width_K = std::clamp<temp_K>(zone_width_K, 5.0f, 15.0f);

		thermal_neutral_zone_min_K = body_temperature_K - zone_width_K;
		thermal_neutral_zone_max_K = body_temperature_K + zone_width_K;
	}

	// ========== ENVIRONMENTAL ADJUSTMENTS ==========

	// Temperature affects ectotherm metabolic rate (Q10 = 2-3)
	if (!needs_endothermy) {
		auto q10 = 2.5f; // Typical value
		auto temp_diff_K = in.environment.temperature_K - temp_K(298.15f); // Relative to 25°C
		float temp_factor = std::pow(q10, float(temp_diff_K) / 10.0f);

		basal_rate_W *= temp_factor;
		max_rate_W *= temp_factor;
		available_muscle_power_W *= temp_factor;
	}

	return Analysis_Metabolic{
		.basal_rate_W = basal_rate_W,
		.max_rate_W = max_rate_W,
		.muscle_mass_kg = muscle_mass_kg,
		.available_muscle_power_W = available_muscle_power_W,
		.body_temperature_K = needs_endothermy ? body_temperature_K : -1.0f,
		.thermal_neutral_zone_min_K = thermal_neutral_zone_min_K,
		.thermal_neutral_zone_max_K = thermal_neutral_zone_max_K
	};
}
