// tonton_inference.cpp
// Metabolic and Behavioral Inference System
// Infers metabolic rates and behavioral traits from morphology and performance capabilities
#include "tonton_behavior.h"
#include "../../include/tonton_skinnedmesh.h"
#include "../../include/tonton_analysis.h"
#include "Memos/tonton_armaturememo.h"
#include "Rules/tonton_scratch.h"
#include "tonton_builder.h"
#include "tonton_input.h"
#include <cmath>

namespace TonTon {
// ============================================================================
// HELPER FUNCTIONS - Semantic Analysis
// ============================================================================

using SemanticAnalysis = Builder::SemanticAnalysis;

// ============================================================================
// TEMPERATURE SCALING
// ============================================================================

static auto TemperatureScaling_Q10(temp_K temperature, temp_K ref_temperature = 303.15f) {
    // Q10 temperature coefficient (Makarieva et al. 2008)
    // Amphibians: 2.21, Reptiles: 2.44
    const auto Q10 = 2.3f;
    return std::pow(Q10, float(temperature - ref_temperature) / 10.0f);
}

// ============================================================================
// ENDOTHERMY DETECTION
// ============================================================================

static bool InferEndothermy(Scratch const& scratch, SemanticAnalysis const& sem) {
    // Birds: powered flight with feathers
    if (scratch.aerial && scratch.aerial->wingbeat_frequency_Hz > 0) {
        // Check for avian characteristics
        if (sem.has_talons) {
            return true; // Birds
        }
        // Could be bat - check for membrane wings vs feathered
        // For now, assume powered flyers are endothermic
        return true;
    }
    
    // Mammals: fur, specialized ears, high activity
    // Would need fur/hair detection from semantic flags
    // For now, use metabolic indicators
    
    // High sustained activity suggests endothermy
    if (scratch.terrestrial) {
        auto sustainable_fraction = scratch.terrestrial->max_sustainable_speed_m_s / 
                                    scratch.terrestrial->max_sprint_speed_m_s;
        if (sustainable_fraction > 0.6f) {
            return true; // Endurance runners (mammals)
        }
    }
    
    // Default: ectothermic
    return false;
}

}

// ============================================================================
// METABOLIC INFERENCE
// ============================================================================

TonTon::Analysis_Metabolic TonTon::ComputeMetabolic(Input const& in, Scratch & scratch) {
    Analysis_Metabolic result;
    
    Builder::SemanticAnalysis const& sem = in.builder->semanticAnalyisis; 
    
    auto mass_kg = scratch.physical.body_mass_kg;
    bool is_endotherm = InferEndothermy(scratch, sem);
    
    // ========================================================================
    // BASAL METABOLIC RATE
    // ========================================================================
    
    if (is_endotherm) {
        // Kleiber's Law for endotherms: BMR ∝ M^0.75
        // Birds: ~4.1 W/kg at 1kg (Alexander 1992)
        // Mammals: ~3.5 W/kg at 1kg (White & Seymour 2003)
        auto base_coefficient = scratch.aerial ? 4.1f : 3.5f;
        result.basal_rate_W = base_coefficient * std::pow(float(mass_kg), 0.75f);
        
        // Adjust for metabolic intensity from user input
        auto intensity = glm::mix(0.8f, 1.2f, in.metabolic_efficiency);
        result.basal_rate_W *= intensity;
        
    } else {
        // Ectotherms (Bennett & Dawson 1976, Pough 1980)
        // At 30°C: ~0.5 W/kg for reptiles
        // ~10-15% of mammal BMR
        auto base_rate = 0.5f * std::pow(float(mass_kg), 0.75f);
        
        // Temperature scaling
        auto temp_factor = TemperatureScaling_Q10(in.environment.temperature_K);
        result.basal_rate_W = base_rate * temp_factor;
        
        // Adjust for metabolic efficiency
        auto efficiency = glm::mix(0.7f, 1.3f, in.metabolic_efficiency);
        result.basal_rate_W *= efficiency;
    }
    
    // ========================================================================
    // AEROBIC SCOPE & MAXIMUM METABOLIC RATE
    // ========================================================================
    
    auto aerobic_scope = 10.0f; // Default
    
    // Adjust based on locomotion mode and lifestyle
    if (scratch.aerial) {
        // Powered flight demands high scope (Butler & Woakes 2001)
        aerobic_scope = 15.0f;
        
        // Hovering specialists even higher
        if (scratch.aerial->hovering_efficiency > 0.7f) {
            aerobic_scope = 18.0f;
        }
    } else if (scratch.aquatic) {
        if (scratch.aquatic->requires_constant_motion) {
            // Obligate ram ventilators (tuna, sharks)
            aerobic_scope = 12.0f;
        } else {
            aerobic_scope = 10.0f;
        }
    } else if (scratch.terrestrial) {
        // Cursorial mammals (endurance runners)
        if (is_endotherm && scratch.terrestrial->max_sustainable_speed_m_s > 10.0f) {
            aerobic_scope = 12.0f;
        } else {
            aerobic_scope = 9.0f;
        }
    }
    
    // Reduce scope for ambush predators (low sustained activity)
    if (scratch.behavior.ambush_vs_pursuit > 0.7f) {
        aerobic_scope *= 0.6f; // Sit-and-wait: 6x scope typical
    }
    
    // Ectotherms generally have lower scope
    if (!is_endotherm) {
        aerobic_scope *= 0.7f;
    }
    
//    result.aerobic_scope = aerobic_scope;
    result.max_rate_W = result.basal_rate_W * aerobic_scope;
    
    // ========================================================================
    // MUSCLE MASS
    // ========================================================================
    
    auto muscle_fraction = 0.40f; // Default: 40% of body mass
    
    if (scratch.aerial) {
        // Birds: pectoralis 15-35% alone (Tobalske et al. 2003, Hartman 1961)
        // Total flight muscle ~25-45%
        if (scratch.aerial->hovering_efficiency > 0.7f) {
            muscle_fraction = 0.50f; // Hummingbirds
        } else if (scratch.aerial->flapping_efficiency > 0.7f) {
            muscle_fraction = 0.45f; // Active flapping birds
        } else {
            muscle_fraction = 0.35f; // Soaring/gliding birds
        }
    } else if (scratch.terrestrial) {
        // Cursorial adaptations increase leg muscle
        if (scratch.terrestrial->max_sprint_speed_m_s > 15.0f) {
            muscle_fraction = 0.48f; // Sprinters (cheetahs, greyhounds)
        } else if (scratch.terrestrial->max_sustainable_speed_m_s > 8.0f) {
            muscle_fraction = 0.45f; // Endurance runners
        }
    } else if (scratch.aquatic) {
        // Tuna: 60-70% muscle (Altringham & Block 1997)
        if (scratch.aquatic->requires_constant_motion) {
            muscle_fraction = 0.55f;
        } else {
            muscle_fraction = 0.42f;
        }
    } else if (scratch.serpentine) {
        // Snakes are mostly muscle for locomotion
        muscle_fraction = 0.50f;
    }
    
    // Adjust for user-specified muscle quality
    muscle_fraction *= glm::mix(0.8f, 1.2f, in.muscle_quality);
    
    result.muscle_mass_kg = mass_kg * muscle_fraction;
    
    // ========================================================================
    // AVAILABLE MUSCLE POWER
    // ========================================================================
    
    // Theoretical maximum: ~400 W/kg (Rome et al. 1988)
    // Practical sustained: ~200-250 W/kg
    cost_W_kg power_density_W_kg = 400.0f;
    
    // Adjust for muscle quality
    power_density_W_kg *= in.muscle_quality;
    
    // Adjust for fiber type specialization
    // Sprint muscle (fast-twitch): higher peak power, lower endurance
    // Endurance muscle (slow-twitch): lower peak power, sustainable
    auto endurance_vs_power = in.behavior.endurance_vs_power;
    auto fiber_modifier = glm::mix(1.2f, 0.8f, endurance_vs_power);
    power_density_W_kg *= fiber_modifier;
    
    result.available_muscle_power_W = result.muscle_mass_kg * power_density_W_kg;
    
    // ========================================================================
    // THERMOREGULATION
    // ========================================================================
    
    if (is_endotherm) {
        // Birds: 40-42°C, Mammals: 36-38°C
        result.body_temperature_K = scratch.aerial ? 314.15f : 310.15f;
        
        // Thermal Neutral Zone (Scholander et al. 1950)
        // Width inversely proportional to surface area / volume ratio
        auto sa_to_vol = scratch.physical.surface_area_m2 / 
                         scratch.physical.body_volume_m3;
        
        // Small animals have narrower TNZ (higher heat loss rate)
        float tnz_width = 10.0f / (1.0f + float(sa_to_vol) * 0.1f);
        
        // TNZ center typically 3-5°C below body temperature
        auto tnz_center = result.body_temperature_K - 3.0f;
        result.thermal_neutral_zone_min_K = tnz_center - tnz_width * 0.5f;
        result.thermal_neutral_zone_max_K = tnz_center + tnz_width * 0.5f;
        
    } else {
        // Ectotherms: no fixed body temperature
        result.body_temperature_K = -1.0f;
        result.thermal_neutral_zone_min_K = -1.0f;
        result.thermal_neutral_zone_max_K = -1.0f;
    }
    
    return result;
}

// ============================================================================
// BEHAVIORAL INFERENCE
// ============================================================================

TonTon::Analysis_Behavior TonTon::ComputeBehavior(Input const& in, Scratch & scratch) {
    Analysis_Behavior result;

    Builder::SemanticAnalysis const& sem = in.builder->semanticAnalyisis; 
    
    bool is_endotherm = scratch.metabolic.is_endotherm();
    auto body_mass_kg = scratch.physical.body_mass_kg;

    // Get niche flags from armature (ecological roles)
    auto niche_flags = in.builder->physical.niche;

    // Enhanced predator detection: morphology OR niche flags
    bool is_predator = sem.is_predator ||
                      HasFlag(niche_flags, NicheFlags::PREDATOR) ||
                      HasFlag(niche_flags, NicheFlags::CARNIVORE);

    bool is_herbivore = HasFlag(niche_flags, NicheFlags::HERBIVORE);

    // ========================================================================
    // AGGRESSION
    // ========================================================================

    result.aggression = in.behavior.aggression_adjustment;

    // Predatory morphology increases aggression
    if (sem.has_sharp_teeth || sem.has_claws) {
        result.aggression += 0.2f;
    }

    // Explicit predator tagging increases aggression
    if (is_predator) {
        result.aggression += 0.1f;
    }
    
    // Venomous creatures are defensively aggressive
    if (sem.has_venom) {
        result.aggression += 0.15f;
    }
    
    // Horned creatures (territorial fighting)
    if (sem.has_horns) {
        result.aggression += 0.15f;
    }
    
    // Large body size correlates with territoriality/aggression
    auto size_factor = glm::clamp<float>(float(body_mass_kg) / 10.0f, 0.0f, 0.2f);
    result.aggression += size_factor;

    // Apex predators are more aggressive
    if (is_predator && body_mass_kg > 50.0f) {
        result.aggression += 0.15f;
    }
    
    result.aggression = glm::clamp(result.aggression, 0.0f, 1.0f);
    
    // ========================================================================
    // SOCIAL TENDENCY
    // ========================================================================
    
    result.social_tendency = in.behavior.social_tendency;
    
    // Eye position indicates social strategy (Kotrschal et al. 1998)
    if (sem.has_lateral_eyes && !sem.has_forward_eyes) {
        // Prey fish with lateral vision -> schooling
        result.social_tendency += 0.3f;
    }
    
    // Good hearing suggests vocal communication (Heffner & Heffner 2008)
    if (sem.has_hearing_organs && scratch.sensory.hearing) {
        if (scratch.sensory.hearing->sensitivity > 0.7f) {
            result.social_tendency += 0.2f;
        }
    }
    
    // Small, defenseless creatures seek safety in numbers
    if (body_mass_kg < 1.0f && !sem.has_weapons) {
        result.social_tendency += 0.25f;
    }
    
    // Ambush predators are solitary
    if (scratch.behavior.ambush_vs_pursuit > 0.7f) {
        result.social_tendency -= 0.3f;
    }

    // Aerial predators hunt alone (except cooperative hunters)
    if (scratch.aerial && is_predator && body_mass_kg > 5.0f) {
        result.social_tendency -= 0.2f;
    }
    
    result.social_tendency = glm::clamp(result.social_tendency, 0.0f, 1.0f);
    
    // ========================================================================
    // ACTIVITY LEVEL
    // ========================================================================
    
    result.activity_level = in.behavior.activity_adjustment;
    
    // High aerobic scope enables sustained activity
    if (scratch.metabolic.aerobic_scope() > 12.0f) {
        result.activity_level += 0.2f;
    }
    
    // Obligate ram ventilators must keep moving
    if (scratch.aquatic && scratch.aquatic->requires_constant_motion) {
        result.activity_level = 0.9f; // Override
    }
    
    // Sit-and-wait predators are less active
    if (scratch.behavior.ambush_vs_pursuit > 0.7f) {
        result.activity_level = glm::min(result.activity_level, 0.3f);
    }
    
    // Ectotherms have lower baseline activity
    if (!is_endotherm) {
        result.activity_level *= 0.7f;
    }
    
    // Thermal soaring birds conserve energy (low activity)
    if (scratch.aerial && scratch.aerial->flapping_efficiency < 0.3f) {
        result.activity_level *= 0.6f; // Soaring/gliding
    }
    
    result.activity_level = glm::clamp(result.activity_level, 0.0f, 1.0f);
    
    // ========================================================================
    // CURIOSITY
    // ========================================================================
    
    // Brain size relative to body suggests intelligence/curiosity
    // Using activity level and sensory investment as proxies
    result.curiosity = 0.5f;
    
    if (scratch.sensory.vision && scratch.sensory.vision->acuity > 0.7f) {
        result.curiosity += 0.2f; // Good vision = investigative
    }
    
    // Manipulators suggest exploratory behavior
    if (!scratch.appendages.manipulation.empty()) {
        result.curiosity += 0.3f;
    }

    // Predators are curious (hunting requires investigation)
    if (is_predator) {
        result.curiosity += 0.15f;
    }
    
    result.curiosity = glm::clamp(result.curiosity, 0.0f, 1.0f);
    
    // ========================================================================
    // TERRITORIALITY
    // ========================================================================
    
    result.territoriality = 0.5f;

    // Large predators defend territories
    if (body_mass_kg > 5.0f && is_predator) {
        result.territoriality = 0.8f;
    }
    
    // Ambush hunters defend feeding sites
    if (scratch.behavior.ambush_vs_pursuit > 0.7f) {
        result.territoriality += 0.2f;
    }
    
    // Horned animals fight for territory
    if (sem.has_horns) {
        result.territoriality += 0.2f;
    }
    
    // Schooling species are less territorial
    if (result.social_tendency > 0.7f) {
        result.territoriality -= 0.3f;
    }
    
    // Migratory animals don't defend territories
    if (result.is_migratory) {
        result.territoriality = 0.1f;
    }
    
    result.territoriality = glm::clamp(result.territoriality, 0.0f, 1.0f);
    
    // ========================================================================
    // DIURNAL PREFERENCE
    // ========================================================================
    
    if (scratch.sensory.vision) {
        // Large eyes relative to body -> nocturnal (Kirk 2006)
        auto eye_ratio = sem.eye_body_ratio;
        
        if (eye_ratio > 0.15f) {
            result.diurnal_preference = 0.2f; // Nocturnal
        } else if (eye_ratio < 0.08f) {
            result.diurnal_preference = 0.8f; // Diurnal
        } else {
            result.diurnal_preference = 0.5f; // Crepuscular
        }
    } else {
        result.diurnal_preference = 0.5f; // Unknown
    }
    
    // ========================================================================
    // MIGRATORY BEHAVIOR
    // ========================================================================
    
    result.is_migratory = false;
    
    // Long-distance fliers may migrate (high aspect ratio wings)
    if (scratch.aerial) {
        auto avg_aspect_ratio = 0.0f;
        for (auto& wing : scratch.aerial->wings) {
            avg_aspect_ratio += wing.aspect_ratio();
        }
        avg_aspect_ratio /= scratch.aerial->wings.size();
        
        // High aspect ratio (>7) suggests soaring/long-distance flight
        if (avg_aspect_ratio > 7.0f && body_mass_kg > 0.5f) {
            result.is_migratory = true;
        }
    }
    
    // User override from seasonal_behavior input
    if (in.behavior.seasonal_behavior > 0.7f) {
        result.is_migratory = true;
    }
    
    // ========================================================================
    // AMBUSH VS PURSUIT
    // ========================================================================
    
    result.ambush_vs_pursuit = 0.5f;
    
    if (scratch.terrestrial) {
        // High acceleration -> ambush (Webb 1984)
        if (scratch.terrestrial->max_acceleration_m_s2 > 30.0f) {
            result.ambush_vs_pursuit += 0.3f;
        }
        
        // High endurance -> pursuit
        auto endurance_ratio = scratch.terrestrial->max_sustainable_speed_m_s /
                               scratch.terrestrial->max_sprint_speed_m_s;
        if (endurance_ratio > 0.7f) {
            result.ambush_vs_pursuit -= 0.3f; // Pursuit hunter
        }
    }
    
    if (scratch.aquatic) {
        // Maneuverability -> ambush
        if (scratch.aquatic->can_hover) {
            result.ambush_vs_pursuit += 0.2f;
        }
        
        // Requires constant motion -> pursuit
        if (scratch.aquatic->requires_constant_motion) {
            result.ambush_vs_pursuit = 0.1f; // Must be pursuit
        }
    }
    
    // Body plan: stocky -> ambush, slender -> pursuit (Fulton et al. 2001)
    float fineness = scratch.physical.fineness_ratio();
    if (fineness < 3.0f) {
        result.ambush_vs_pursuit += 0.2f; // Stocky
    } else if (fineness > 5.0f) {
        result.ambush_vs_pursuit -= 0.3f; // Streamlined
    }
    
    result.ambush_vs_pursuit = glm::clamp(result.ambush_vs_pursuit, 0.0f, 1.0f);
    
    // ========================================================================
    // PREY SIZE PREFERENCE
    // ========================================================================
    
    // Relative to own body mass
    result.prey_size_preference = 0.3f; // Default: small prey

    if (is_predator) {
        // Large predators take larger prey
        if (body_mass_kg > 50.0f) {
            result.prey_size_preference = 0.6f;
        }
        
        // Constriction ability enables large prey
        if (scratch.specialized.constriction) {
            result.prey_size_preference += 0.3f;
        }
        
        // Pack hunters take larger prey
        if (result.social_tendency > 0.7f) {
            result.prey_size_preference += 0.2f;
        }
    }
    
    result.prey_size_preference = glm::clamp(result.prey_size_preference, 0.0f, 1.0f);
    
    // ========================================================================
    // DIETARY SPECIALIZATION
    // ========================================================================
    
    result.dietary_specialization = 0.5f; // Default: generalist
    
    // Specialized feeding apparatus -> specialist
    if (!scratch.appendages.manipulation.empty()) {
        // Complex manipulators suggest specialized feeding
        result.dietary_specialization += 0.2f;
    }
    
    // Extreme morphology -> specialist
    if (fineness > 8.0f || fineness < 2.0f) {
        result.dietary_specialization += 0.2f;
    }
    
    // Venom -> specialist (specific prey types)
    if (sem.has_venom) {
        result.dietary_specialization += 0.2f;
    }
    
    result.dietary_specialization = glm::clamp(result.dietary_specialization, 0.0f, 1.0f);
    
    // ========================================================================
    // SOCIAL STRUCTURE
    // ========================================================================
    
    if (result.social_tendency < 0.5f) {
        result.optimal_group_size = -1.0f;
        result.personal_space_radius_m = -1.0f;
    } else {
        // Group size scales with predator/prey and body mass
        if (is_predator) {
            result.optimal_group_size = 5.0f; // Small packs (2-10)
        } else {
            // Prey form larger schools, inversely proportional to size
            result.optimal_group_size = 50.0f * std::pow(float(body_mass_kg), -0.25f);
        }
        
        // Personal space: 0.5-1.0 body lengths (Partridge 1982)
        result.personal_space_radius_m = scratch.physical.body_length_m * 0.75f;
    }
    
    // ========================================================================
    // TERRITORY SIZE
    // ========================================================================
    
    if (result.territoriality < 0.5f) {
        result.territory_radius_m = -1.0f;
    } else {
        // Territory size ∝ M^1.0 for vertebrates (Davies & Houston 1984)
        auto base_radius = 10.0f * std::pow(float(body_mass_kg), 1.0f);

        // Predators need larger territories
        if (is_predator) {
            base_radius *= 2.0f;
        }
        
        // Three-dimensional space users have larger territories
        if (scratch.aerial || scratch.climbing) {
            base_radius *= 1.5f;
        }
        
        result.territory_radius_m = base_radius;
    }
    
    // ========================================================================
    // THREAT RESPONSE
    // ========================================================================
    
    // Alarm calls (vocal communication + social)
    result.uses_alarm_calls = result.social_tendency > 0.6f && 
                              sem.has_hearing_organs;
    
    // Mobbing behavior (small birds harassing predators)
    result.uses_mobbing_behavior = result.social_tendency > 0.7f && 
                                   body_mass_kg < 0.5f &&
                                   scratch.aerial.has_value();
    
    // Fight vs flight depends on size, weapons, social support
    result.fight_vs_flight = 0.3f; // Default: flee
    
    if (sem.has_weapons) {
        result.fight_vs_flight += 0.3f;
    }
    
    if (body_mass_kg > 10.0f) {
        result.fight_vs_flight += 0.2f; // Size gives confidence
    }
    
    if (result.social_tendency > 0.7f) {
        result.fight_vs_flight += 0.2f; // Group defense
    }
    
    result.fight_vs_flight = glm::clamp(result.fight_vs_flight, 0.0f, 1.0f);
    
    // ========================================================================
    // HABITAT PREFERENCES
    // ========================================================================
    
    // Temperature range depends on thermoregulation
    if (is_endotherm) {
        // Endotherms tolerate wide temperature ranges
        result.habitat.temperature_min_K = 253.15f; // -20°C
        result.habitat.temperature_max_K = 323.15f; // 50°C
    } else {
        // Ectotherms have narrower optimal range (Huey & Stevenson 1979)
        auto optimal_temp = in.environment.temperature_K;
        result.habitat.temperature_min_K = optimal_temp - 10.0f;
        result.habitat.temperature_max_K = optimal_temp + 15.0f;
    }
    
    // Humidity preference (mainly for amphibians/arthropods)
    if (scratch.aquatic) {
        result.habitat.humidity_preference = 1.0f; // High
    } else if (scratch.aerial) {
        result.habitat.humidity_preference = 0.5f; // Moderate
    } else {
        result.habitat.humidity_preference = 0.5f; // Default
    }
    
    // Open vs dense habitat from mobility
    if (scratch.aerial) {
        result.habitat.open_vs_dense = 0.2f; // Open terrain
    } else if (scratch.climbing) {
        result.habitat.open_vs_dense = 0.8f; // Dense forest
    } else if (scratch.terrestrial && scratch.terrestrial->max_sprint_speed_m_s > 15.0f) {
        result.habitat.open_vs_dense = 0.3f; // Open terrain for speed
    } else {
        result.habitat.open_vs_dense = 0.5f; // Mixed
    }
    
    // Canopy height preference
    if (scratch.aerial) {
        result.habitat.canopy_height_preference_m = 20.0f; // Above canopy
    } else if (scratch.climbing) {
        result.habitat.canopy_height_preference_m = 15.0f; // Mid-canopy
    } else {
        result.habitat.canopy_height_preference_m = 0.0f; // Ground level
    }
    
    // ========================================================================
    // AI ARCHETYPE SUGGESTION
    // ========================================================================

    // Decision tree based on key discriminators
    if (result.social_tendency > 0.7f) {
        if (is_predator) {
            result.suggested_archetype = Analysis_Behavior::AIArchetype::PACK_COORDINATOR;
        } else {
            result.suggested_archetype = scratch.aquatic ?
                Analysis_Behavior::AIArchetype::SCHOOLING_PREY :
                Analysis_Behavior::AIArchetype::SOCIAL_FORAGER;
        }
    } else if (result.territoriality > 0.7f) {
        result.suggested_archetype = Analysis_Behavior::AIArchetype::TERRITORIAL_DEFENDER;
    } else if (result.ambush_vs_pursuit > 0.7f) {
        result.suggested_archetype = Analysis_Behavior::AIArchetype::SOLITARY_AMBUSH_HUNTER;
    } else if (scratch.aerial && is_predator) {
        // Aerial predators: birds of prey, dragonflies, bats
        // No mass requirement - dragonflies are small but still aerial predators
        result.suggested_archetype = Analysis_Behavior::AIArchetype::AERIAL_PREDATOR;
    } else if (is_predator && body_mass_kg > 100.0f) {
        result.suggested_archetype = Analysis_Behavior::AIArchetype::APEX_PREDATOR;
    } else if (is_herbivore) {
        // Herbivores are social foragers by default
        result.suggested_archetype = Analysis_Behavior::AIArchetype::SOCIAL_FORAGER;
    } else if (is_predator) {
        // Small/medium predators that don't fit other categories
        result.suggested_archetype = Analysis_Behavior::AIArchetype::SOLITARY_AMBUSH_HUNTER;
    } else {
        result.suggested_archetype = Analysis_Behavior::AIArchetype::OPPORTUNISTIC_SCAVENGER;
    }
    
    return result;
}
