#include "tonton_metabolic.h"
#include "tonton_analysis.h"
#include "tonton_builder.h"
#include "tonton_input.h"
#include "tonton_skinnedmesh.h"
#include "Rules/tonton_scratch.h"
#include <cmath>
#include <vector>
#include <sstream>

using CF = TonTon::CladeFlags;
using Severity = TonTon::Analysis_Diagnostics::Warning::Severity;

namespace {

// Helper to build a string of all active clade flags
std::string CladeFlagsToString(TonTon::CladeFlags clade) {
	std::string result;

	// List of all clade flags to check
	static const TonTon::CladeFlags all_clades[] = {
		CF::CHORDATA, CF::AMPHIBIA, CF::REPTILIA, CF::CHELONIA,
		CF::AVES, CF::MAMMALIA, CF::UNGULATA, CF::EQUIDAE, CF::CETACEA,
		CF::PISCES, CF::ARTHROPODA, CF::INSECTA, CF::ARACHNIDA, CF::CRUSTACEA,
		CF::MOLLUSCA, CF::CEPHALOPODA
	};

	for (auto f : all_clades) {
		if (HasFlag(clade, f)) {
			if (!result.empty()) result += " | ";
			result += TonTon::WordToString(f);
		}
	}

	return result.empty() ? "NONE (fallback heuristics used)" : result;
}

} // anonymous namespace

TonTon::Analysis_Metabolic TonTon::ComputeMetabolic(Input const& in, Scratch & s,
                                                    MetabolicDemand const& demand)
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
	bool has_wings = false;
	
	for(auto ap : in.builder->appendages)
	{
		if(HasFlag(ap.semantic_flags, SemanticFlags::WING))
		{
			has_wings = true;
			break;
		}
	}
	
	// Sustained powered flight requires endothermy. Note this is a statement about
	// POWER, not about taxonomy: it is true of a bat, a bird and a wyvern alike,
	// and it is why a patagium should contribute a physiological trait rather than
	// a clade -- the membrane implies sustained flight, and implies nothing at all
	// about ancestry.
	//
	// `demand.sustained_flight_W` is zero on pass 1, so this reduces to the
	// anatomical shortcut ("it has wings, so presumably it flies") until locomotion
	// has actually reported a number. On pass 2 the real question gets asked.
	if (has_wings && HasFlag(clade, CF::CHORDATA)) {
		needs_endothermy = true;
	}

	// Birds and mammals are obligate endotherms
	if (HasFlag(clade, CF::AVES) || HasFlag(clade, CF::MAMMALIA)) {
		needs_endothermy = true;
	}

	// ========== GATHER CLADE CONTRIBUTIONS ==========

	// AVES: Highest metabolic rate
	// McKechnie & Wolf (2004) / Lasiewski & Dawson (1967): avian BMR allometry.
	// Coefficient expressed in WATTS: a ~1 kg bird BMR is ~5-7 W (non-passerines
	// near mammalian level, passerines somewhat higher). 4.8*1^0.72 = 4.8 W sits in
	// that band; the prior 6.25 reads as a kcal/day-based regression figure (6.25 W
	// at 1 kg, and with exp 0.72 < mammal 0.75 it diverges further above Kleiber's
	// 4.18*M^0.75 only at sub-kg masses) which is ~too high when interpreted as Watts.
	// Ellington et al. (1990): Flight muscle: 200-400 W/kg sustained power
	if (HasFlag(clade, CF::AVES)) {
		contributions.push_back({
			.weight = 1.0f,
			.rmr_coef = 4.8f,    // avian BMR in W (was 6.25 — that figure reads as kcal/day, ~too high in W)
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
			.rmr_coef = 0.8f,           // Low ectotherm basal rate, in WATTS (0.8 W at 1 kg — consistent with Watts)
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
			.rmr_coef = 0.5f,           // Reptile ectotherm basal rate, in WATTS (0.5 W at 1 kg — consistent with Watts)
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
		// Flying insect muscle power: Odonata (dragonflies) achieve 200-400 W/kg
		// Ellington (1985): flight muscle power ~250 W/kg at 25°C
		// Higher than non-flying arthropods (100 W/kg)
		auto insect_muscle_power = has_wings ?
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

	// ========== DIAGNOSTICS: CLADE DETECTION ==========
	{
		char buf[256];
		snprintf(buf, sizeof(buf), "Metabolic: detected clades = %s", CladeFlagsToString(clade).c_str());
		s.diagnostics.warnings.push_back({Severity::INFO, buf});

		if (contributions.size() > 1) {
			snprintf(buf, sizeof(buf), "Metabolic: hybrid creature - blending %zu clade contributions", contributions.size());
			s.diagnostics.warnings.push_back({Severity::INFO, buf});
		}

		snprintf(buf, sizeof(buf), "Metabolic: thermal strategy = %s%s",
			needs_endothermy ? "ENDOTHERM" : "ECTOTHERM",
			(needs_endothermy && has_wings) ? " (required for flight)" : "");
		s.diagnostics.warnings.push_back({Severity::INFO, buf});
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
		// Target a floor at the mammalian endotherm level rather than blindly stacking ×5
		// (which can push an already-warm clade past mammalian RMR).
		rmr_coefficient = std::max(rmr_coefficient, 4.18f);
		muscle_power_W_kg = std::max<cost_W_kg>(muscle_power_W_kg, 200.0f);

		s.diagnostics.warnings.push_back({Severity::INFO,
			"Metabolic: upgraded ectotherm clade to endotherm (flight requires sustained high power output)"});
	}

	// ========== COMPUTE FINAL METABOLIC RATES ==========

	// Basal metabolic rate (RMR = coefficient * M^exponent)
	power_W basal_rate_W = rmr_coefficient * std::pow(float(body_mass_kg), rmr_exponent);

	// Aerobic scope (max/basal ratio)
	// Ectotherms:            2-8x  (typical 5x)
	// Endotherms, running:  7-15x  (typical 10x)
	// Endotherms, FLYING:  15-30x  -- flapping flight is the most aerobically
	//   demanding sustained activity in vertebrates. Bats and birds hold 15-25x
	//   BMR in level flight; hummingbirds exceed 30x. A flat 10x is a running
	//   mammal's ceiling and cannot fund flight at all.
	auto aerobic_scope = needs_endothermy ? 10.0f : 5.0f;

	// Locomotion interrogates physiology, rather than physiology being asserted
	// and locomotion having to live inside it. If level flight demanded more than
	// a running endotherm's budget funds, ask whether a FLYER's budget would cover
	// it -- and grant it only if the answer is yes.
	//
	// The cap is what keeps this honest. Granting exactly whatever was demanded
	// would make every creature able to afford whatever it happens to need, which
	// is unfalsifiable. Capping at the empirical flyer ceiling means "cannot fly"
	// stays a reachable verdict: a creature whose demand exceeds even 22x BMR is
	// simply not a flyer, and gets no physiology it cannot justify.
	// Mechanical-to-metabolic conversion, shared with sustained_muscle_power_W below
	// so the two cannot drift apart.
	const float muscle_efficiency = 0.20f + 0.05f * in.metabolic_efficiency;

	constexpr float kFlyerAerobicScope = 22.0f;   // Thomas (1975) bats; Ward et al. birds
	if (demand.sustained_flight_W > 0.0f && aerobic_scope < kFlyerAerobicScope) {
		const float funded_at_flyer_scope =
			float(basal_rate_W) * kFlyerAerobicScope * muscle_efficiency;

		if (demand.sustained_flight_W <= funded_at_flyer_scope) {
			aerobic_scope = kFlyerAerobicScope;
			char buf[256];
			snprintf(buf, sizeof(buf),
				"Metabolic: sustained flight demands %.3g W mechanical; raised aerobic scope "
				"%.0fx -> %.0fx (flyer range 15-30x BMR)",
				demand.sustained_flight_W, 10.0f, kFlyerAerobicScope);
			s.diagnostics.warnings.push_back({Severity::INFO, buf});
		}
	}

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
	auto burst_muscle_power_W = muscle_mass_kg * muscle_power_W_kg;

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

	// Diagnostic: report environment temperature
	{
		char buf[256];
		float temp_C = float(in.environment.temperature_K) - 273.15f;
		snprintf(buf, sizeof(buf), "Metabolic: environment temperature = %.1f K (%.1f °C)",
			float(in.environment.temperature_K), temp_C);
		s.diagnostics.warnings.push_back({Severity::INFO, buf});
	}

	// Temperature affects ectotherm metabolic rate (Q10 = 2-3)
	if (!needs_endothermy) {
		auto q10 = 2.5f; // Typical value for metabolic RATE
		// Muscle MECHANICAL power is less temperature-sensitive than metabolic rate.
		auto power_q10 = 1.8f; // Muscle mechanical power is less temperature-sensitive than metabolic rate
		auto temp_diff_K = in.environment.temperature_K - temp_K(298.15f); // Relative to 25°C
		float temp_factor = std::pow(q10, float(temp_diff_K) / 10.0f);
		float power_temp_factor = std::pow(power_q10, float(temp_diff_K) / 10.0f);

		// Diagnostic: report Q10 temperature adjustment
		{
			char buf[256];
			float temp_C = float(in.environment.temperature_K) - 273.15f;
			snprintf(buf, sizeof(buf),
				"Metabolic: Q10 adjustment = %.3fx (env %.1f°C vs reference 25°C)",
				temp_factor, temp_C);
			s.diagnostics.warnings.push_back({Severity::INFO, buf});

			// Temperature warnings for ectotherms
			if (temp_factor < 0.1f) {
				snprintf(buf, sizeof(buf),
					"Metabolic: CRITICAL - temperature %.1f°C severely impairs ectotherm function (Q10=%.3f)",
					temp_C, temp_factor);
				s.diagnostics.warnings.push_back({Severity::ERROR, buf});
			} else if (temp_factor < 0.3f) {
				snprintf(buf, sizeof(buf),
					"Metabolic: temperature %.1f°C significantly reduces ectotherm performance (Q10=%.3f)",
					temp_C, temp_factor);
				s.diagnostics.warnings.push_back({Severity::CAUTION, buf});
			} else if (temp_factor < 0.6f) {
				snprintf(buf, sizeof(buf),
					"Metabolic: cool temperature %.1f°C reduces ectotherm metabolic rate (Q10=%.3f)",
					temp_C, temp_factor);
				s.diagnostics.warnings.push_back({Severity::INFO, buf});
			} else if (temp_factor > 4.0f) {
				snprintf(buf, sizeof(buf),
					"Metabolic: WARNING - high temperature %.1f°C may cause heat stress (Q10=%.3f)",
					temp_C, temp_factor);
				s.diagnostics.warnings.push_back({Severity::CAUTION, buf});
			}
		}

		basal_rate_W *= temp_factor;
		max_rate_W *= temp_factor;
		burst_muscle_power_W *= power_temp_factor;
	} else {
		// Endotherm thermal stress warnings
		float temp_C = float(in.environment.temperature_K) - 273.15f;
		if (float(in.environment.temperature_K) < float(thermal_neutral_zone_min_K)) {
			char buf[256];
			float below_tnz = float(thermal_neutral_zone_min_K) - float(in.environment.temperature_K);
			snprintf(buf, sizeof(buf),
				"Metabolic: environment %.1f°C is %.1f K below thermal neutral zone (%.1f-%.1f K) - increased heat production required",
				temp_C, below_tnz, float(thermal_neutral_zone_min_K), float(thermal_neutral_zone_max_K));
			s.diagnostics.warnings.push_back({Severity::INFO, buf});
		} else if (float(in.environment.temperature_K) > float(thermal_neutral_zone_max_K)) {
			char buf[256];
			float above_tnz = float(in.environment.temperature_K) - float(thermal_neutral_zone_max_K);
			snprintf(buf, sizeof(buf),
				"Metabolic: environment %.1f°C is %.1f K above thermal neutral zone - heat dissipation required",
				temp_C, above_tnz);
			s.diagnostics.warnings.push_back({Severity::CAUTION, buf});
		}
	}

	// ========== SUSTAINED (AEROBIC-LIMITED) MECHANICAL POWER ==========
	// burst_muscle_power_W above is what the muscle tissue can produce; it says
	// nothing about whether the animal can pay for it. Sustained output is capped
	// by the aerobic system: P_mech <= efficiency * P_metabolic.
	//
	// Muscle chemical->mechanical efficiency is 20-25% across vertebrates
	// (Smith et al. 2005; Barclay 2015).
	//
	// This cap is not new: tonton_aerial.cpp applied exactly this rule privately
	// so that flight power checks were honest, while every other locomotion mode
	// read the raw burst figure. Hoisting it here makes one rule serve all modes.
	// Computed AFTER the Q10 block so both terms are temperature-adjusted.
	power_W sustained_muscle_power_W =
		std::min<power_W>(burst_muscle_power_W, max_rate_W * muscle_efficiency);

	return Analysis_Metabolic{
		.basal_rate_W = basal_rate_W,
		.max_rate_W = max_rate_W,
		.muscle_mass_kg = muscle_mass_kg,
		.burst_muscle_power_W = burst_muscle_power_W,
		.sustained_muscle_power_W = sustained_muscle_power_W,
		.body_temperature_K = needs_endothermy ? body_temperature_K : -1.0f,
		.thermal_neutral_zone_min_K = thermal_neutral_zone_min_K,
		.thermal_neutral_zone_max_K = thermal_neutral_zone_max_K
	};
}
