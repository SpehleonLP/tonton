#include "metrics_mammalia.h"
#include "gltfmeshfile.h"
#include <cmath>
#include <algorithm>
#include <numeric>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace Mammalia;

namespace {
    // Universal mammalian constants from literature
    
    // Tissue properties (Love, 1970; Schmidt-Nielsen, 1984)
    constexpr float MAMMAL_TISSUE_DENSITY = 1050.0f; // kg/m³
    constexpr float BONE_DENSITY = 1800.0f;          // kg/m³
    
    // Muscle mechanics (Rome et al., 1988; Biewener, 1990)
    constexpr float MUSCLE_STRESS_MAX = 300000.0f;   // Pa (300 kPa)
    constexpr float MUSCLE_POWER_DENSITY = 400.0f;   // W/kg peak
    constexpr float MUSCLE_EFFICIENCY = 0.25f;       // 25% mechanical efficiency
    
    // Skeletal safety factors (Biewener, 1989, 1990)
    constexpr float BONE_STRESS_SAFE = 200.0e6f;     // Pa (200 MPa with SF 2-4)
    constexpr float SAFETY_FACTOR_MIN = 2.0f;
    constexpr float SAFETY_FACTOR_MAX = 4.0f;
    
    // Metabolic scaling (Kleiber, 1947; White & Seymour, 2003)
    constexpr float KLEIBER_COEFFICIENT = 3.5f;      // W for 1 kg mammal
    constexpr float KLEIBER_EXPONENT = 0.75f;        // BMR ∝ M^0.75
    
    // Gait transition Froude numbers (Alexander & Jayes, 1983)
    constexpr float FROUDE_WALK_TROT = 0.35f;
    constexpr float FROUDE_TROT_GALLOP = 2.5f;       // Typical value 2-3
    
    // Thermal properties (Scholander et al., 1950)
    constexpr float MAMMAL_CORE_TEMP = 37.0f;        // °C typical
    constexpr float THERMAL_CONDUCTIVITY_FUR = 0.04f; // W/(m·K)
    constexpr float THERMAL_CONDUCTIVITY_BLUBBER = 0.2f; // W/(m·K)
}

float MorphologicalAnalysis::Prediction::final(float manual_adjustment, float weight) const {
    if (confidence > 0.9f) return value; // High confidence geometry wins
    
    float uncertainty = 1.0f - confidence;
    float manual_influence = uncertainty * weight;
    
    return value * (1.0f - manual_influence) + 
           manual_adjustment * manual_influence;
}

MorphologicalAnalysis MorphologicalAnalysis::AnalyzeMorphology(const MammalTonTonInputs& inputs) {
    gltf::MeshFile_SkinnedMesh const& skinnedMesh = inputs.skinnedMesh();
    gltf::MeshFile_Skin const& skin = inputs.skin();
    
    MorphologicalAnalysis analysis = {};
    
    // ========== BASIC BODY MEASUREMENTS ==========
    
    auto bounds = inputs.GetBoundingBox();
    float total_volume = skinnedMesh.volume * inputs.volume_scale();
    float total_surface_area = skinnedMesh.surfaceArea * inputs.area_scale();
    
    // Body dimensions (Y=up, Z=forward for quadrupeds)
    analysis.body_length_m = bounds.max.z - bounds.min.z;
    float body_height = bounds.max.y - bounds.min.y;
    float body_width = bounds.max.x - bounds.min.x;
    float aabb_volume = analysis.body_length_m * body_height * body_width;
    
    // Mass estimation: tissue density ~1050 kg/m³ (Schmidt-Nielsen, 1984)
    analysis.body_mass_kg = total_volume * MAMMAL_TISSUE_DENSITY;
    
    // Core geometric ratios
    analysis.surface_area_to_volume = total_surface_area / total_volume;
    analysis.body_compactness = total_volume / aabb_volume;
    
    // ========== LIMB ANALYSIS ==========
    
    struct LimbData {
        float length = 0.0f;
        float cross_section = 0.0f;
        float digit_length = 0.0f;
        float claw_length = 0.0f;
        float claw_curvature = 0.0f;
        float pad_area = 0.0f;
        glm::vec3 centroid = glm::vec3(0);
        int bone_count = 0;
    };
    
    LimbData forelimbs, hindlimbs;
    float tail_length = 0.0f;
    int tail_bones = 0;
    
    inputs.for_each_tag([&](int i, immutable_array<counted_string> tags) {
        bool is_forelimb = inputs.HasTag(tags, "forelimb") || 
                          inputs.HasTag(tags, "arm") || 
                          inputs.HasTag(tags, "foreleg");
        bool is_hindlimb = inputs.HasTag(tags, "hindlimb") || 
                          inputs.HasTag(tags, "leg") || 
                          inputs.HasTag(tags, "hind");
        bool is_tail = inputs.HasTag(tags, "tail");
        bool is_digit = inputs.HasTag(tags, "digit") || 
                       inputs.HasTag(tags, "finger") || 
                       inputs.HasTag(tags, "toe");
        bool is_claw = inputs.HasTag(tags, "claw") || inputs.HasTag(tags, "nail");
        bool is_pad = inputs.HasTag(tags, "pad") || inputs.HasTag(tags, "palm");
        
        // Position-based detection if no tags
        if (tags.empty()) {
            float z_pos = skinnedMesh.perBone.centroid[i].z;
            float x_pos = std::abs(skinnedMesh.perBone.centroid[i].x);
            float y_pos = skinnedMesh.perBone.centroid[i].y;
            
            // Limbs are lateral structures with elongated geometry
            float aspect_ratio = 0.0f;
            // TODO: Calculate actual bone aspect ratio from geometry
            
            if (x_pos > 0.2f * body_width && y_pos < body_height * 0.6f) {
                if (z_pos < bounds.min.z + 0.4f * analysis.body_length_m) {
                    is_forelimb = true;
                } else if (z_pos > bounds.min.z + 0.4f * analysis.body_length_m) {
                    is_hindlimb = true;
                }
            }
            
            // Tail: posterior, medial, elongated
            if (z_pos > bounds.max.z - 0.1f * analysis.body_length_m && 
                x_pos < 0.15f * body_width) {
                is_tail = true;
            }
        }
        
        if (is_forelimb) {
            forelimbs.length = std::max(forelimbs.length, 
                skinnedMesh.perBone.centroid[i].y - bounds.min.y);
            forelimbs.cross_section += skinnedMesh.perBone.surfaceArea[i] / 
                (forelimbs.length + 0.001f);
            forelimbs.bone_count++;
        }
        
        if (is_hindlimb) {
            hindlimbs.length = std::max(hindlimbs.length,
                skinnedMesh.perBone.centroid[i].y - bounds.min.y);
            hindlimbs.cross_section += skinnedMesh.perBone.surfaceArea[i] / 
                (hindlimbs.length + 0.001f);
            hindlimbs.bone_count++;
        }
        
        if (is_digit) {
            float digit_len = std::pow(skinnedMesh.perBone.volume[i], 1.0f/3.0f);
            forelimbs.digit_length = std::max(forelimbs.digit_length, digit_len);
            hindlimbs.digit_length = std::max(hindlimbs.digit_length, digit_len);
        }
        
        if (is_claw) {
            // Claw curvature from bounding box aspect (Cartmill, 1985)
            float claw_len = std::pow(skinnedMesh.perBone.volume[i], 1.0f/3.0f);
            forelimbs.claw_length = std::max(forelimbs.claw_length, claw_len);
            // TODO: Extract actual curvature from geometry
            forelimbs.claw_curvature = 0.5f; // Placeholder
        }
        
        if (is_pad) {
            forelimbs.pad_area += skinnedMesh.perBone.surfaceArea[i];
            hindlimbs.pad_area += skinnedMesh.perBone.surfaceArea[i];
        }
        
        if (is_tail) {
            tail_length += std::pow(skinnedMesh.perBone.volume[i], 1.0f/3.0f);
            tail_bones++;
        }
    });
    
    // Calculate limb ratios
    analysis.limb_length_ratio = (forelimbs.length + hindlimbs.length) / 
        (2.0f * analysis.body_length_m);
    
    // Digit length ratio for arboreal detection (Kirk et al., 2008)
    float palm_length = forelimbs.length * 0.15f; // Approximate
    analysis.digit_length_ratio = forelimbs.digit_length / (palm_length + 0.001f);
    
    // Hindlimb dominance for saltatorial species (Alexander, 1974)
    analysis.hindlimb_length_ratio = hindlimbs.length / (forelimbs.length + 0.001f);
    
    // Tail ratio for balance and arboreal behavior
    analysis.tail_length_ratio = tail_length / analysis.body_length_m;
    
    // Forelimb robustness for digging (Hopkins, 2007)
    analysis.forelimb_robustness = forelimbs.cross_section / (forelimbs.length + 0.001f);
    
    // Claw metrics for climbing (Zani, 2000; Birn-Jeffrey et al., 2012)
    analysis.claw_curvature = forelimbs.claw_curvature;
    analysis.claw_length_ratio = forelimbs.claw_length / (forelimbs.digit_length + 0.001f);
    
    // Foot pad area for traction (Maiolino et al., 2016)
    float total_foot_area = forelimbs.length * body_width * 0.1f; // Approximate
    analysis.foot_pad_area_ratio = forelimbs.pad_area / (total_foot_area + 0.001f);
    
    // ========== HEAD AND SENSORY ANALYSIS ==========
    
    float eye_diameter = 0.0f;
    float ear_area = 0.0f;
    float snout_length = 0.0f;
    float skull_length = 0.0f;
    float jaw_depth = 0.0f;
    bool found_eyes = false;
    
    inputs.for_each_tag([&](int i, immutable_array<counted_string> tags) {
        bool is_eye = inputs.HasTag(tags, "eye");
        bool is_ear = inputs.HasTag(tags, "ear");
        bool is_snout = inputs.HasTag(tags, "snout") || inputs.HasTag(tags, "nose");
        bool is_jaw = inputs.HasTag(tags, "jaw") || inputs.HasTag(tags, "mandible");
        bool is_head = inputs.HasTag(tags, "head") || is_eye || is_ear || is_snout;
        
        // Position-based head detection: anterior 15% of body
        if (tags.empty() && skinnedMesh.perBone.centroid[i].z < 
            bounds.min.z + 0.15f * analysis.body_length_m) {
            is_head = true;
        }
        
        if (is_eye) {
            // Spherical approximation (Heesy, 2008)
            eye_diameter = std::max(eye_diameter, 
                std::pow(skinnedMesh.perBone.volume[i] * 6.0f / M_PI, 1.0f/3.0f));
            found_eyes = true;
        }
        
        if (is_ear) {
            ear_area += skinnedMesh.perBone.surfaceArea[i];
        }
        
        if (is_snout) {
            snout_length = std::max(snout_length,
                skinnedMesh.perBone.centroid[i].z - bounds.min.z);
        }
        
        if (is_head) {
            skull_length = std::max(skull_length,
                skinnedMesh.perBone.centroid[i].z - bounds.min.z);
        }
        
        if (is_jaw) {
            jaw_depth = std::max(jaw_depth,
                bounds.max.y - skinnedMesh.perBone.centroid[i].y);
        }
    });
    
    // Eye size correlates with activity pattern (Heesy, 2008; Kirk, 2006)
    analysis.eye_diameter_ratio = found_eyes ? 
        (eye_diameter / skull_length) : 0.05f;
    
    // Ear size indicates auditory acuity (Heffner & Heffner, 2008)
    float head_area = skull_length * body_height * 0.3f;
    analysis.ear_size_ratio = ear_area / (head_area + 0.001f);
    
    // Snout length indicates olfactory emphasis (Muchlinski, 2010)
    analysis.snout_length_ratio = snout_length / (skull_length + 0.001f);
    
    // Jaw robustness indicates bite force (Wroe et al., 2005)
    analysis.jaw_robustness = jaw_depth / (skull_length + 0.001f);
    
    // ========== SKELETAL ANALYSIS ==========
    
    // Bone gracility: cursorial mammals have slender bones (Carrano, 1999)
    float avg_bone_diameter = std::pow(total_volume / 
        (analysis.body_length_m * 10.0f), 0.5f); // Approximate
    analysis.bone_gracility = avg_bone_diameter / analysis.body_length_m;
    
    // Vertebral flexibility from body proportions (Slijper, 1946)
    analysis.vertebral_flexibility = std::clamp(
        body_height / (analysis.body_length_m * 0.3f), 0.0f, 1.0f);
    
    // Limb posture from limb angle (Biewener, 1989)
    // Sprawling (salamanders) = ~30°, Erect (horses) = ~90°
    analysis.limb_posture_angle = 45.0f + analysis.limb_length_ratio * 45.0f;
    
    // ========== MOMENT OF INERTIA ==========
    
    glm::vec3 total_inertia = glm::vec3(0);
    float total_mass = total_volume * MAMMAL_TISSUE_DENSITY;
    
    for (auto i = 0u; i < skin.joints.size(); i++) {
        glm::mat3 global_inertia = transform_inertia(
            inputs.file.nodes._referencePose[i],
            skinnedMesh.perBone.inertia[i],
            skinnedMesh.perBone.volume[i]);
        
        total_inertia.x += global_inertia[0][0] * skinnedMesh.perBone.volume[i] * 
            MAMMAL_TISSUE_DENSITY;
        total_inertia.y += global_inertia[1][1] * skinnedMesh.perBone.volume[i] * 
            MAMMAL_TISSUE_DENSITY;
        total_inertia.z += global_inertia[2][2] * skinnedMesh.perBone.volume[i] * 
            MAMMAL_TISSUE_DENSITY;
    }
    
    analysis.moment_of_inertia = total_inertia;
    
    // ========== LOCOMOTOR MODE PREDICTIONS ==========
    
    // Cursorial capability (Garland, 1983; Carrano, 1999)
    {
        float cursorial_score = 0.0f;
        
        // Long, gracile limbs (Carrano, 1999)
        cursorial_score += std::clamp(analysis.limb_length_ratio * 1.5f, 0.0f, 0.3f);
        
        // Erect posture (Biewener, 1989)
        cursorial_score += std::clamp((analysis.limb_posture_angle - 45.0f) / 45.0f, 
            0.0f, 0.3f);
        
        // Digitigrade/unguligrade stance: small pad ratio (Garland & Janis, 1993)
        cursorial_score += std::clamp((0.3f - analysis.foot_pad_area_ratio) * 2.0f, 
            0.0f, 0.2f);
        
        // Reduced digit number: inferred from low digit length (Carrano, 1999)
        cursorial_score += std::clamp((0.5f - analysis.digit_length_ratio), 0.0f, 0.2f);
        
        analysis.cursorial_capability = {
            .value = std::clamp(cursorial_score, 0.0f, 1.0f),
            .confidence = 0.78f // Supported by Garland (1983), Carrano (1999)
        };
    }
    
    // Saltatorial capability (Alexander, 1974, 1975; Biewener & Blickhan, 1988)
    {
        float saltatorial_score = 0.0f;
        
        // Hindlimb >> forelimb (Alexander, 1974)
        if (analysis.hindlimb_length_ratio > 1.2f) {
            saltatorial_score += 0.4f * std::min(
                (analysis.hindlimb_length_ratio - 1.0f), 1.0f);
        }
        
        // Long tail for balance (Webster & Dawson, 2003)
        saltatorial_score += std::clamp(analysis.tail_length_ratio * 0.5f, 0.0f, 0.3f);
        
        // Robust hindlimbs for power (Biewener & Blickhan, 1988)
        saltatorial_score += std::clamp(hindlimbs.cross_section * 0.1f, 0.0f, 0.3f);
        
        analysis.saltatorial_capability = {
            .value = std::clamp(saltatorial_score, 0.0f, 1.0f),
            .confidence = 0.82f // Strong correlation (Alexander, 1974)
        };
    }
    
    // Arboreal capability (Cartmill, 1985; Kirk et al., 2008)
    {
        float arboreal_score = 0.0f;
        
        // Long digits for grasping (Kirk et al., 2008)
        arboreal_score += std::clamp(analysis.digit_length_ratio * 0.5f, 0.0f, 0.3f);
        
        // Curved claws for climbing (Zani, 2000)
        arboreal_score += std::clamp(analysis.claw_curvature * 0.6f, 0.0f, 0.3f);
        
        // Prehensile tail (if present)
        if (analysis.tail_length_ratio > 0.8f) {
            arboreal_score += 0.2f;
        }
        
        // Large foot pads for grip (Maiolino et al., 2016)
        arboreal_score += std::clamp(analysis.foot_pad_area_ratio * 0.5f, 0.0f, 0.2f);
        
        analysis.arboreal_capability = {
            .value = std::clamp(arboreal_score, 0.0f, 1.0f),
            .confidence = 0.75f // Cartmill (1985), Kirk et al. (2008)
        };
    }
    
    // Fossorial capability (Hopkins, 2007; Casinos et al., 1993)
    {
        float fossorial_score = 0.0f;
        
        // Robust forelimbs (Hopkins, 2007)
        fossorial_score += std::clamp(analysis.forelimb_robustness * 5.0f, 0.0f, 0.4f);
        
        // Short limbs (Casinos et al., 1993)
        fossorial_score += std::clamp((0.5f - analysis.limb_length_ratio), 0.0f, 0.3f);
        
        // Long claws (Hildebrand, 1985)
        fossorial_score += std::clamp(analysis.claw_length_ratio * 0.5f, 0.0f, 0.2f);
        
        // Long snout for head-lift digging (Hopkins, 2007)
        fossorial_score += std::clamp(analysis.snout_length_ratio * 0.3f, 0.0f, 0.1f);
        
        analysis.fossorial_capability = {
            .value = std::clamp(fossorial_score, 0.0f, 1.0f),
            .confidence = 0.72f // Hopkins (2007)
        };
    }
    
    // Aquatic capability (Fish & Baudinette, 1999; Williams, 1999)
    {
        float aquatic_score = 0.0f;
        
        // Streamlined body (Fish, 1993)
        float fineness = analysis.body_length_m / 
            (std::sqrt(body_height * body_width) + 0.001f);
        aquatic_score += std::clamp((fineness - 3.0f) / 4.0f, 0.0f, 0.3f);
        
        // Reduced limb length for hydrodynamics (Fish & Baudinette, 1999)
        aquatic_score += std::clamp((0.4f - analysis.limb_length_ratio), 0.0f, 0.3f);
        
        // Large foot pads (webbing) (Fish, 1993)
        aquatic_score += std::clamp(analysis.foot_pad_area_ratio * 0.5f, 0.0f, 0.2f);
        
        // Reduced external ears (Williams, 1999)
        aquatic_score += std::clamp((0.05f - analysis.ear_size_ratio) * 4.0f, 
            0.0f, 0.2f);
        
        analysis.aquatic_capability = {
            .value = std::clamp(aquatic_score, 0.0f, 1.0f),
            .confidence = 0.68f // Fish & Baudinette (1999)
        };
    }
    
    // ========== METABOLIC PREDICTIONS ==========
    
    // Metabolic intensity from SA/V ratio (Lovegrove, 2000)
    {
        // Small mammals have higher mass-specific metabolism (Schmidt-Nielsen, 1984)
        float predicted_bmr = KLEIBER_COEFFICIENT * 
            std::pow(analysis.body_mass_kg, KLEIBER_EXPONENT);
        float normalized_bmr = predicted_bmr / analysis.body_mass_kg; // W/kg
        
        // SA/V ratio affects heat loss (Lovegrove, 2000)
        float sa_v_effect = std::clamp(
            (analysis.surface_area_to_volume - 200.0f) / 400.0f, 0.0f, 1.0f);
        
        analysis.metabolic_intensity = {
            .value = sa_v_effect,
            .confidence = 0.73f // Lovegrove (2000), White & Seymour (2003)
        };
    }
    
    // Social complexity from brain size proxy (Dunbar & Shultz, 2007)
    {
        // Eye size correlates with brain size (Heesy, 2008)
        // Large ears suggest group coordination (Heffner & Heffner, 2008)
        float social_score = analysis.eye_diameter_ratio * 5.0f * 0.5f +
                           analysis.ear_size_ratio * 10.0f * 0.5f;
        
        analysis.social_complexity = {
            .value = std::clamp(social_score, 0.0f, 1.0f),
            .confidence = 0.65f // Lower confidence, indirect measures
        };
    }
    
    // Sensory strategy (Kirk, 2006; Muchlinski, 2010)
    {
        // Large eyes = vision-dominant (Kirk, 2006)
        // Long snout = olfaction-dominant (Muchlinski, 2010)
        float vision_score = analysis.eye_diameter_ratio * 10.0f;
        float olfaction_score = analysis.snout_length_ratio * 2.0f;
        
        // Normalize: 0 = vision, 1 = olfaction
        float total = vision_score + olfaction_score + 0.001f;
        
        analysis.sensory_strategy = {
            .value = olfaction_score / total,
            .confidence = 0.70f // Kirk (2006), Muchlinski (2010)
        };
    }
    
    // ========== TAG-DERIVED SPECIALIZATIONS ==========
    
    inputs.for_each_tag([&](int i, immutable_array<counted_string> tags) {
        if (inputs.HasTag(tags, "flipper")) analysis.has_flippers = true;
        if (inputs.HasTag(tags, "wing")) analysis.has_wings = true;
        if (inputs.HasTag(tags, "hoof")) analysis.has_hooves = true;
        if (inputs.HasTag(tags, "retractile") && inputs.HasTag(tags, "claw")) 
            analysis.has_claws_retractile = true;
        if (inputs.HasTag(tags, "prehensile") && inputs.HasTag(tags, "tail"))
            analysis.has_prehensile_tail = true;
        if (inputs.HasTag(tags, "pouch") && inputs.HasTag(tags, "cheek"))
            analysis.has_cheek_pouches = true;
        if (inputs.HasTag(tags, "horn") || inputs.HasTag(tags, "antler"))
            analysis.has_horns_antlers = true;
        if (inputs.HasTag(tags, "blubber")) analysis.has_blubber = true;
        if (inputs.HasTag(tags, "dense") && inputs.HasTag(tags, "fur"))
            analysis.has_fur_dense = true;
    });
    
    // ========== INFERRED SPECIALIZATIONS ==========
    
    // Graviportal: >1000 kg with columnar limbs (Alexander et al., 1979)
    analysis.is_graviportal = (analysis.body_mass_kg > 1000.0f) &&
        (analysis.limb_posture_angle > 75.0f);
    
    // Glider: large lateral skin surface (patagium detection)
    // TODO: Detect skin membrane from surface area anomalies
    analysis.is_glider = false; // Placeholder
    
    // Bipedal: hindlimb dominance + tail balance (Alexander, 1974)
    analysis.is_bipedal = (analysis.hindlimb_length_ratio > 1.5f) &&
        (analysis.tail_length_ratio > 0.8f);
    
    // ========== OVERALL CONFIDENCE ==========
    
    std::vector<float> confidences = {
        analysis.cursorial_capability.confidence,
        analysis.saltatorial_capability.confidence,
        analysis.arboreal_capability.confidence,
        analysis.fossorial_capability.confidence,
        analysis.aquatic_capability.confidence,
        analysis.metabolic_intensity.confidence,
        analysis.social_complexity.confidence,
        analysis.sensory_strategy.confidence
    };
    
    analysis.overall_confidence = std::accumulate(
        confidences.begin(), confidences.end(), 0.0f) / confidences.size();
    
    return analysis;
}

// ========== ENVIRONMENTAL LIMITS ==========

EnvironmentalLimits EnvironmentalLimits::Factory(
    const MammalTonTonInputs& inputs,
    const MorphologicalAnalysis& analysis) {
    
    EnvironmentalLimits limits = {};
    const auto& env = inputs.environment;
    
    float body_mass = analysis.body_mass_kg;
    float body_length = analysis.body_length_m;
    
    // ========== LOCOMOTOR CONSTRAINTS ==========
    
    // Minimum friction for traction (Alexander, 1983)
    {
        // Digitigrade/unguligrade need more friction (Garland & Janis, 1993)
        float base_friction = 0.3f; // Minimum for plantigrade
        if (analysis.cursorial_capability.value > 0.7f) {
            base_friction = 0.5f; // Cursorial specialists need more
        }
        if (analysis.foot_pad_area_ratio < 0.15f) {
            base_friction = 0.6f; // Small contact area = higher friction needed
        }
        limits.min_substrate_friction = base_friction;
    }
    
    // Maximum climbable slope (Taylor et al., 1972)
    {
        float base_angle = 30.0f; // Degrees, typical mammal
        
        // Arboreal specialists (Cartmill, 1985)
        if (analysis.arboreal_capability.value > 0.7f) {
            base_angle = 85.0f; // Nearly vertical
        } else if (analysis.claw_curvature > 0.5f) {
            base_angle = 60.0f; // Good climbers
        }
        
        // Mass penalty for large animals (Preuschoft & Witte, 1991)
        if (body_mass > 50.0f) {
            base_angle *= std::pow(50.0f / body_mass, 0.2f);
        }
        
        limits.max_slope_angle_deg = std::clamp(base_angle, 0.0f, 90.0f);
    }
    
    // Jumping capability (Alexander, 1995)
    {
        // Muscle power available (Rome et al, 1988)
        // Peak power: 400 W/kg (Rome et al., 1988)
        float muscle_mass = body_mass * 0.4f; // ~40% muscle mass (Grand, 1977)
        float peak_power = muscle_mass * MUSCLE_POWER_DENSITY;
        
        // Vertical jump: convert power to height (Alexander, 1995)
        // E = mgh, work done by muscle over leg extension distance
        float leg_extension = hindlimbs.length * 0.6f; // ~60% extension
        float work_capacity = peak_power * 0.1f; // 100ms contraction
        
        // With elastic storage boost (Alexander, 1974)
        if (analysis.saltatorial_capability.value > 0.7f) {
            work_capacity *= 1.7f; // Tendon spring contribution (Biewener & Blickhan, 1988)
        }
        
        limits.max_jump_height_m = work_capacity / (body_mass * env.gravity_m_s2);
        limits.max_jump_height_m = std::clamp(limits.max_jump_height_m, 0.0f, 3.0f);
        
        // Horizontal jump: typically 2-4× vertical (Alexander, 1995)
        limits.max_jump_distance_m = limits.max_jump_height_m * 
            (2.5f + analysis.saltatorial_capability.value * 1.5f);
    }
    
    // Turning radius (Carrier et al., 2001; Walter, 2003)
    {
        // Minimum radius from moment of inertia and speed
        float typical_speed = body_length * 2.0f; // 2 BL/s
        float normalized_inertia = analysis.moment_of_inertia.y / 
            (body_mass * body_length * body_length);
        
        limits.min_turning_radius_m = typical_speed * typical_speed * 
            normalized_inertia / (env.gravity_m_s2 * inputs.environment.substrate_friction);
        limits.min_turning_radius_m = std::clamp(
            limits.min_turning_radius_m, body_length * 0.3f, body_length * 5.0f);
    }
    
    // Swimming ability (Fish & Baudinette, 1999)
    {
        if (analysis.aquatic_capability.value > 0.5f) {
            limits.water_crossing_ability = analysis.aquatic_capability.value;
        } else if (analysis.has_blubber || analysis.foot_pad_area_ratio > 0.3f) {
            limits.water_crossing_ability = 0.5f; // Semi-aquatic capable
        } else {
            limits.water_crossing_ability = 0.2f; // Poor swimmer
        }
    }
    
    // ========== THERMAL CONSTRAINTS ==========
    
    // Thermal limits (Scholander et al., 1950; Bradley & Deavers, 1980)
    {
        float insulation_quality = 0.5f;
        
        if (analysis.has_blubber) {
            insulation_quality = 0.9f; // Excellent (marine mammals)
        } else if (analysis.has_fur_dense) {
            insulation_quality = 0.8f; // Very good (arctic mammals)
        } else if (body_mass > 100.0f) {
            insulation_quality = 0.7f; // Mass provides thermal inertia
        }
        
        // Lower critical temperature (Scholander et al., 1950)
        // Larger animals and better insulation = lower LCT
        float base_lct = 15.0f; // °C for moderate-sized mammal
        base_lct -= (insulation_quality - 0.5f) * 20.0f;
        base_lct -= std::log10(body_mass + 1.0f) * 5.0f; // Size effect
        
        limits.min_ambient_temp_C = base_lct - 15.0f; // Lethal ~15°C below LCT
        
        // Upper critical temperature (Dawson & Schmidt-Nielsen, 1966)
        float base_uct = 30.0f; // °C typical
        
        // Large animals overheat easier (Schmidt-Nielsen, 1984)
        if (body_mass > 500.0f) {
            base_uct -= 5.0f;
        }
        
        // Dense fur reduces heat tolerance (Chappell & Bartholomew, 1981)
        if (analysis.has_fur_dense) {
            base_uct -= 10.0f;
        }
        
        limits.max_ambient_temp_C = base_uct + 15.0f; // Lethal ~15°C above UCT
        
        // Maximum activity temperature (Taylor, 1969)
        limits.max_sustained_activity_temp_C = base_uct;
        
        // Metabolic water production (Schmidt-Nielsen, 1964)
        // Desert specialists produce ~0.5 ml/MJ from fat oxidation
        float water_efficiency = 0.3f; // ml/MJ baseline
        if (analysis.fossorial_capability.value > 0.5f) {
            water_efficiency = 0.5f; // Burrowers are water-efficient
        }
        limits.metabolic_water_production = water_efficiency;
    }
    
    // ========== RESPIRATORY CONSTRAINTS ==========
    
    // Altitude tolerance (West, 1984; Monge & Leon-Velarde, 1991)
    {
        // Sea level O₂: 21 kPa
        // Mammals need ~15 kPa minimum (Hochachka et al., 1983)
        float min_o2_pressure = 15000.0f; // Pa
        
        // Altitude adaptation increases tolerance (Monge & Leon-Velarde, 1991)
        if (env.altitude_m > 2000.0f) {
            min_o2_pressure *= 0.8f; // Adapted populations
        }
        
        limits.min_oxygen_partial_pressure = min_o2_pressure;
    }
    
    // Maximum sustained speed (Taylor et al., 1982; Garland, 1983)
    {
        // Aerobic capacity limits sustained speed
        // VO₂max scales as M^0.8 (Taylor et al., 1981)
        float vo2_max = 150.0f * std::pow(body_mass, 0.8f); // ml O₂/min
        
        // Cost of transport: ~2-4 J/(kg·m) (Taylor et al., 1982)
        float cot = 3.0f * std::pow(body_mass, -0.32f); // J/(kg·m)
        
        // Metabolic scope: endotherms ~10-15× BMR (Weibel et al., 2004)
        float aerobic_power = vo2_max * 20.0f; // J/min (20 J/ml O₂)
        
        // Sustainable speed from aerobic power
        limits.max_sustained_speed_mps = (aerobic_power * 60.0f) / 
            (body_mass * cot);
        
        // Apply locomotor mode corrections
        if (analysis.cursorial_capability.value > 0.8f) {
            limits.max_sustained_speed_mps *= 1.5f; // Sprint specialists
        } else if (analysis.fossorial_capability.value > 0.7f) {
            limits.max_sustained_speed_mps *= 0.3f; // Slow movers
        }
        
        limits.max_sustained_speed_mps = std::clamp(
            limits.max_sustained_speed_mps, 0.5f, 30.0f);
    }
    
    // Burst duration (Hochachka, 1985)
    {
        // Anaerobic capacity: ~200-300 J/kg (Hochachka, 1985)
        float anaerobic_capacity = 250.0f * body_mass; // J
        
        // Burst power: 2-3× aerobic (Rome et al., 1988)
        float burst_power = 3.0f * body_mass * 10.0f; // W (rough estimate)
        
        limits.max_burst_duration_s = anaerobic_capacity / burst_power;
        limits.max_burst_duration_s = std::clamp(
            limits.max_burst_duration_s, 2.0f, 30.0f);
    }
    
    // ========== STRUCTURAL CONSTRAINTS ==========
    
    // Bone stress safety factor (Biewener, 1989, 1990)
    {
        // Peak stresses are 25-50% of failure strength
        limits.bone_stress_safety_factor = 3.0f; // Typical
        
        if (body_mass > 1000.0f) {
            limits.bone_stress_safety_factor = 2.0f; // Giants run closer to limit
        } else if (body_mass < 1.0f) {
            limits.bone_stress_safety_factor = 4.0f; // Small mammals overbuilt
        }
    }
    
    // Maximum impact force (Biewener & Taylor, 1986)
    {
        // Ground reaction forces: 1-3× body weight typical (Alexander, 1977)
        // Peak impacts: up to 10× during jumping (Alexander, 1974)
        float max_grf_multiple = 3.0f;
        
        if (analysis.saltatorial_capability.value > 0.7f) {
            max_grf_multiple = 10.0f; // Jumpers experience high impacts
        } else if (analysis.cursorial_capability.value > 0.8f) {
            max_grf_multiple = 4.0f; // Runners
        }
        
        limits.max_impact_force_N = body_mass * env.gravity_m_s2 * max_grf_multiple;
    }
    
    // Burrow soil hardness limit (Carotenuto et al., 2020)
    {
        if (analysis.fossorial_capability.value > 0.5f) {
            // Unconfined compressive strength tolerance
            // Muscle force / claw area (Hopkins, 2007)
            float muscle_force = analysis.forelimb_robustness * 1000.0f; // N
            float claw_area = 0.0001f; // m² (approximate)
            
            limits.burrow_soil_hardness_limit = muscle_force / claw_area;
            limits.burrow_soil_hardness_limit = std::clamp(
                limits.burrow_soil_hardness_limit, 50000.0f, 500000.0f); // 50-500 kPa
        } else {
            limits.burrow_soil_hardness_limit = 0.0f;
        }
    }
    
    // Climbing grip force (Cartmill, 1985; Autumn et al., 2000)
    {
        if (analysis.arboreal_capability.value > 0.5f) {
            // Claw penetration or frictional grip
            float grip_coefficient = 0.5f; // Base friction
            
            if (analysis.claw_curvature > 0.5f) {
                grip_coefficient = 2.0f; // Claws interlock with substrate
            }
            
            limits.climbing_grip_force_N = body_mass * env.gravity_m_s2 * 
                grip_coefficient * 2.0f; // Safety factor
        } else {
            limits.climbing_grip_force_N = 0.0f;
        }
    }
    
    // ========== ENERGETIC CONSTRAINTS ==========
    
    // Starvation time (Lindstedt & Boyce, 1985)
    {
        // Fat reserves: 10-30% body mass (Pond, 1998)
        float fat_fraction = 0.15f;
        if (analysis.has_blubber) {
            fat_fraction = 0.35f; // Marine mammals store more
        }
        
        float fat_mass = body_mass * fat_fraction;
        float fat_energy = fat_mass * 39.0e6f; // J (39 MJ/kg fat)
        
        // Daily energy: 2-4× BMR (Nagy et al., 1999)
        float bmr = KLEIBER_COEFFICIENT * std::pow(body_mass, KLEIBER_EXPONENT);
        float daily_energy = bmr * 86400.0f * 2.5f; // J/day
        
        limits.starvation_time_days = fat_energy / daily_energy;
        limits.starvation_time_days = std::clamp(
            limits.starvation_time_days, 1.0f, 90.0f);
    }
    
    // Dehydration time (Schmidt-Nielsen, 1964)
    {
        // Water loss: evaporation + excretion
        // Baseline: 50 ml/(kg·day) (Schmidt-Nielsen, 1964)
        float water_loss_rate = 50.0f; // ml/(kg·day)
        
        // Arid adaptations reduce loss (Schmidt-Nielsen, 1964)
        if (analysis.fossorial_capability.value > 0.5f) {
            water_loss_rate *= 0.3f; // Burrowers conserve water
        }
        
        // Body water: ~60% body mass (Schmidt-Nielsen, 1984)
        float body_water = body_mass * 0.6f * 1000.0f; // ml
        
        // Can lose ~15% before lethal dehydration (Schmidt-Nielsen, 1964)
        float available_water = body_water * 0.15f;
        
        limits.dehydration_time_days = available_water / 
            (water_loss_rate * body_mass);
        limits.dehydration_time_days = std::clamp(
            limits.dehydration_time_days, 0.5f, 30.0f);
    }
    
    // Daily energy budget (Nagy et al., 1999)
    {
        // Field metabolic rate: 2-4× BMR (Nagy et al., 1999)
        float bmr = KLEIBER_COEFFICIENT * std::pow(body_mass, KLEIBER_EXPONENT);
        float fmr_multiplier = 2.5f;
        
        if (analysis.metabolic_intensity.value > 0.7f) {
            fmr_multiplier = 4.0f; // Active species
        } else if (analysis.fossorial_capability.value > 0.7f) {
            fmr_multiplier = 2.0f; // Sedentary burrowers
        }
        
        limits.max_daily_energy_budget_MJ = (bmr * 86400.0f * fmr_multiplier) / 1.0e6f;
    }
    
    // Minimum food density (Harestad & Bunnel, 1979)
    {
        // Home range scales as M^1.0-1.3 (Harestad & Bunnel, 1979)
        float home_range_km2 = 0.01f * std::pow(body_mass, 1.1f);
        
        // Predators need larger ranges (Gittleman & Harvey, 1982)
        if (analysis.jaw_robustness > 0.5f) {
            home_range_km2 *= 10.0f;
        }
        
        // Food needed = energy budget / food energy density
        float food_needed_kg = limits.max_daily_energy_budget_MJ / 5.0f; // ~5 MJ/kg food
        
        limits.min_food_density_kg_km2 = food_needed_kg / home_range_km2;
    }
    
    return limits;
}

// ========== BEHAVIORAL PROFILE CONSTRUCTION ==========

MammalBehavioralProfile ConstructBehavioralProfile(
    const MammalTonTonInputs& inputs,
    const MorphologicalAnalysis& analysis) {
    
    MammalBehavioralProfile profile = {};
    
    // Helper for applying manual adjustments
    auto ApplyAdjustment = [](const MorphologicalAnalysis::Prediction& pred, 
        float manual_adj, float weight = 0.2f) -> float {
        if (pred.confidence > 0.9f) return pred.value;
        float uncertainty = 1.0f - pred.confidence;
        float manual_influence = uncertainty * weight;
        return pred.value * (1.0f - manual_influence) + manual_adj * manual_influence;
    };
    
    float body_mass = analysis.body_mass_kg;
    float body_length = analysis.body_length_m;
    
    // ========== LOCOMOTOR MODE DETERMINATION ==========
    
    // Determine primary locomotion from morphology
    float max_capability = 0.0f;
    LocomotorMode primary = Cursorial; // Default
    
    struct ModeScore {
        LocomotorMode mode;
        float score;
    };
    
    std::vector<ModeScore> modes = {
        {Cursorial, analysis.cursorial_capability.value},
        {Saltatorial, analysis.saltatorial_capability.value},
        {Arboreal, analysis.arboreal_capability.value},
        {Fossorial, analysis.fossorial_capability.value},
        {Aquatic, analysis.aquatic_capability.value}
    };
    
    // Sort by score
    std::sort(modes.begin(), modes.end(), 
        [](const ModeScore& a, const ModeScore& b) { return a.score > b.score; });
    
    profile.primary_locomotion = modes[0].mode;
    
    // Secondary locomotion if close second
    if (modes[1].score > 0.5f && modes[1].score > modes[0].score * 0.7f) {
        profile.secondary_locomotion = modes[1].mode;
    }
    
    // Override if forced
    if (inputs.force_locomotor_mode.has_value()) {
        profile.primary_locomotion = inputs.force_locomotor_mode.value();
    }
    
    // ========== SPEED AND GAIT CHARACTERISTICS ==========
    
    // Maximum speeds (Garland, 1983; Alexander, 1977)
    {
        // Sprint speed scaling (Garland, 1983)
        // V_max ∝ M^0.17 for terrestrial mammals
        float base_sprint_BLs = 3.0f * std::pow(body_mass, 0.17f / 0.33f); // Convert to BL/s
        
        // Cursorial specialists (Garland, 1983)
        if (analysis.cursorial_capability.value > 0.8f) {
            base_sprint_BLs *= 2.0f; // Cheetahs, gazelles
        }
        
        // Size penalties for giants (Alexander, 1985)
        if (body_mass > 1000.0f) {
            base_sprint_BLs *= std::pow(1000.0f / body_mass, 0.2f);
        }
        
        profile.max_sprint_speed_BLs = std::clamp(base_sprint_BLs, 1.0f, 30.0f);
        
        // Sustained speed: typically 40-60% of sprint (Taylor et al., 1982)
        profile.max_sustainable_speed_BLs = profile.max_sprint_speed_BLs * 0.5f;
        
        // Preferred speed: ~30-40% of max (Hoyt & Taylor, 1981)
        profile.preferred_speed_BLs = profile.max_sprint_speed_BLs * 0.35f;
    }
    
    // Acceleration (Walter & Carrier, 2007)
    {
        // Peak acceleration: limited by muscle power and grip
        float muscle_accel = (MUSCLE_POWER_DENSITY * 0.4f * body_mass) / 
            (body_mass * body_length);
        
        profile.acceleration_capability = std::clamp(muscle_accel, 5.0f, 25.0f);
    }
    
    // Maneuverability (Carrier et al., 2001)
    {
        float maneuver_score = ApplyAdjustment(
            MorphologicalAnalysis::Prediction{
                .value = 1.0f - std::clamp(analysis.moment_of_inertia.y / 
                    (body_mass * body_length * body_length * 0.1f), 0.0f, 1.0f),
                .confidence = 0.70f
            },
            inputs.activity_adjustment * 0.3f
        );
        
        profile.maneuverability = maneuver_score;
    }
    
    // Specialized locomotor abilities
    profile.climbing_ability = analysis.arboreal_capability.value;
    profile.jumping_ability = analysis.saltatorial_capability.value;
    profile.swimming_ability = analysis.aquatic_capability.value;
    profile.digging_capability = analysis.fossorial_capability.value;
    
    // ========== GAIT PARAMETERS ==========
    
    // Preferred gait (Hildebrand, 1977)
    {
        if (profile.primary_locomotion == Saltatorial) {
            profile.preferred_gait = Hop;
        } else if (analysis.cursorial_capability.value > 0.7f) {
            profile.preferred_gait = Trot; // Cursors prefer trot
        } else if (analysis.arboreal_capability.value > 0.7f) {
            profile.preferred_gait = Walk; // Climbers walk carefully
        } else {
            profile.preferred_gait = Walk; // Default
        }
    }
    
    // Gait transition Froude numbers (Alexander & Jayes, 1983)
    profile.walk_trot_froude = FROUDE_WALK_TROT;
    profile.trot_gallop_froude = FROUDE_TROT_GALLOP;
    
    // Duty factors (Hildebrand, 1968)
    profile.duty_factor_walk = 0.6f + (body_mass > 100.0f ? 0.05f : 0.0f);
    profile.duty_factor_trot = 0.45f - (analysis.cursorial_capability.value * 0.05f);
    
    // Stride parameters (Heglund et al., 1974)
    {
        // Stride frequency: f ∝ M^-0.14 (Heglund et al., 1974)
        profile.stride_frequency_hz = 2.0f * std::pow(body_mass, -0.14f);
        
        // Stride length: ~1.5-2.0 BL typical (Alexander, 1977)
        profile.stride_length_BL = 1.7f + analysis.cursorial_capability.value * 0.3f;
    }
    
    // Gallop type (Hildebrand, 1977)
    profile.uses_gallop_rotary = (body_mass < 50.0f) && 
        (analysis.vertebral_flexibility > 0.7f); // Cheetah-style
    
    // Ricochetal locomotion (Alexander, 1974)
    profile.uses_ricochetal_locomotion = (profile.primary_locomotion == Saltatorial);
    
    // ========== POSTURAL CHARACTERISTICS ==========
    
    profile.limb_angle_degrees = analysis.limb_posture_angle;
    profile.spinal_flexion_range = analysis.vertebral_flexibility * M_PI * 0.4f; // Radians
    
    // Elastic energy storage (Alexander, 1988; Biewener et al., 1981)
    profile.uses_elastic_storage = (body_mass > 5.0f) && 
        (analysis.cursorial_capability.value > 0.5f || 
         analysis.saltatorial_capability.value > 0.7f);
    
    // ========== THERMOREGULATION ==========
    
    // Thermal strategy
    if (inputs.thermal_strategy.has_value()) {
        profile.thermal_strategy = inputs.thermal_strategy.value();
    } else {
        if (body_mass > 100.0f) {
            profile.thermal_strategy = PreciseRegulator;
        } else if (analysis.has_fur_dense || analysis.has_blubber) {
            profile.thermal_strategy = ModerateRegulator;
        } else {
            profile.thermal_strategy = Thermoconformer;
        }
    }
    
    // Thermal neutral zone width (Scholander et al., 1950)
    {
        float base_tnz = 10.0f; // °C
        
        if (analysis.has_blubber || analysis.has_fur_dense) {
            base_tnz = 20.0f; // Excellent insulation
        }
        if (body_mass > 500.0f) {
            base_tnz += 10.0f; // Thermal inertia
        }
        
        profile.thermal_neutral_zone_width = base_tnz;
    }
    
    // Critical temperatures
    profile.lower_critical_temp_C = MAMMAL_CORE_TEMP - 
        profile.thermal_neutral_zone_width - 5.0f;
    profile.upper_critical_temp_C = MAMMAL_CORE_TEMP + 5.0f;
    
    // Insulation quality
    profile.insulation_quality = 0.5f;
    if (analysis.has_blubber) profile.insulation_quality = 0.9f;
    else if (analysis.has_fur_dense) profile.insulation_quality = 0.8f;
    else if (body_mass > 100.0f) profile.insulation_quality = 0.7f;
    
    // Heat loss rate (Scholander et al., 1950)
    profile.surface_heat_loss_rate = 100.0f * (1.0f - profile.insulation_quality);
    
    // ========== METABOLIC CHARACTERISTICS ==========
    
    // Basal metabolic rate (Kleiber, 1947)
    profile.basal_metabolic_rate_W = KLEIBER_COEFFICIENT * 
        std::pow(body_mass, KLEIBER_EXPONENT);
    
    // Field metabolic rate (Nagy et al., 1999)
    float fmr_multiplier = 2.5f;
    if (analysis.metabolic_intensity.value > 0.7f) fmr_multiplier = 4.0f;
    else if (analysis.fossorial_capability.value > 0.7f) fmr_multiplier = 2.0f;
    
    profile.field_metabolic_rate_W = profile.basal_metabolic_rate_W * fmr_multiplier;
    
    // Aerobic scope (Weibel et al., 2004)
    profile.aerobic_scope = fmr_multiplier * 3.0f; // FMR is submaximal
    
    // Cost of transport (Taylor et al., 1982)
    profile.cost_of_transport = 3.0f * std::pow(body_mass, -0.32f); // J/(kg·m)
    
    // Endurance factor
    profile.endurance_factor = 0.5f + analysis.cursorial_capability.value * 0.3f;
    
    // ========== SOCIAL BEHAVIOR ==========
    
    profile.gregariousness = ApplyAdjustment(
        analysis.social_complexity,
        inputs.social_adjustment
    );
    
    // Group size (Jarman, 1974; Dunbar, 1992)
    {
        if (profile.gregariousness > 0.7f) {
            profile.optimal_group_size = 10.0f + body_mass * 0.05f; // Large herds
        } else if (profile.gregariousness > 0.4f) {
            profile.optimal_group_size = 3.0f + profile.gregariousness * 5.0f;
        } else {
            profile.optimal_group_size = 1.0f; // Solitary
        }
    }
    
    // Territoriality (Grant, 1997; Harestad & Bunnel, 1979)
    {
        profile.territoriality = 0.5f;
        
        if (analysis.jaw_robustness > 0.5f) {
            profile.territoriality = 0.8f; // Predators defend territories
        } else if (profile.gregariousness > 0.7f) {
            profile.territoriality = 0.2f; // Herds less territorial
        }
        
        // Territory size: home range scaling (Harestad & Bunnel, 1979)
        profile.territorial_range_m = 100.0f * std::pow(body_mass, 0.55f);
    }
    
    // Dominance hierarchy (Drews, 1993)
    {
        if (profile.optimal_group_size > 5.0f) {
            profile.dominance_hierarchy_strength = 0.7f; // Large groups need structure
        } else {
            profile.dominance_hierarchy_strength = 0.3f;
        }
    }
    
    // ========== ACTIVITY PATTERNS ==========
    
    // Diurnal preference (Roll et al., 2006; Bennie et al., 2014)
    {
        if (inputs.nocturnal_override) {
            profile.diurnal_preference = 0.0f;
        } else {
            // Large eyes = nocturnal (Kirk, 2006)
            float eye_score = analysis.eye_diameter_ratio * 10.0f;
            profile.diurnal_preference = std::clamp(1.0f - eye_score, 0.0f, 1.0f);
            
            // Predators often crepuscular (Zielinski, 1988)
            if (analysis.jaw_robustness > 0.6f) {
                profile.diurnal_preference = 0.5f;
            }
        }
    }
    
    profile.activity_level = ApplyAdjustment(
        analysis.metabolic_intensity,
        inputs.activity_adjustment
    );
    
    // Hibernation/estivation (Geiser, 2004)
    {
        profile.hibernates = (body_mass < 10.0f) && 
            (profile.metabolic_intensity.value < 0.5f);
        profile.estivates = false; // Rare in mammals
    }
    
    // Daily active hours
    {
        if (profile.diurnal_preference > 0.7f) {
            profile.daily_active_hours = 12.0f; // Diurnal
        } else if (profile.diurnal_preference < 0.3f) {
            profile.daily_active_hours = 10.0f; // Nocturnal
        } else {
            profile.daily_active_hours = 4.0f; // Crepuscular
        }
    }
    
    // ========== FEEDING ECOLOGY ==========
    
    profile.predatory_capability = ApplyAdjustment(
        MorphologicalAnalysis::Prediction{
            .value = analysis.jaw_robustness * 1.5f,
            .confidence = 0.72f
        },
        inputs.aggression_adjustment * 0.4f
    );
    profile.predatory_capability = std::clamp(profile.predatory_capability, 0.0f, 1.0f);
    
    // Diet specialization (Clutton-Brock & Harvey, 1977)
    {
        // Specialists have distinctive morphology
        if (analysis.has_hooves) {profile.diet_specialization = 0.8f; // Herbivore specialists
        } else if (analysis.has_claws_retractile) {
            profile.diet_specialization = 0.9f; // Carnivore specialists
        } else if (analysis.has_cheek_pouches) {
            profile.diet_specialization = 0.7f; // Granivores
        } else {
            profile.diet_specialization = 0.3f; // Generalists
        }
    }
    
    // Foraging range (Harestad & Bunnel, 1979; McNab, 1963)
    {
        // Home range scales as M^1.0-1.3 (Harestad & Bunnel, 1979)
        float range_km = std::pow(body_mass, 0.55f) * 0.1f;
        
        if (profile.predatory_capability > 0.7f) {
            range_km *= 10.0f; // Carnivores range widely
        }
        
        profile.foraging_range_km = std::clamp(range_km, 0.01f, 100.0f);
    }
    
    // Food caching (Vander Wall, 1990)
    {
        profile.caches_food = (body_mass < 20.0f) && 
            (profile.hibernates || analysis.has_cheek_pouches);
    }
    
    // ========== HABITAT PREFERENCES ==========
    
    // Canopy preference
    profile.canopy_preference = analysis.arboreal_capability.value;
    
    // Water dependency (Brashares & Arcese, 2002)
    {
        if (analysis.aquatic_capability.value > 0.7f) {
            profile.water_dependency = 1.0f;
        } else if (analysis.fossorial_capability.value > 0.5f) {
            profile.water_dependency = 0.2f; // Desert burrowers
        } else {
            profile.water_dependency = 0.5f; // Moderate
        }
    }
    
    // Shelter requirement
    {
        if (analysis.fossorial_capability.value > 0.7f) {
            profile.shelter_requirement = 1.0f; // Must have burrows
        } else if (body_mass > 500.0f) {
            profile.shelter_requirement = 0.2f; // Large animals less vulnerable
        } else {
            profile.shelter_requirement = 0.6f;
        }
    }
    
    // Altitude tolerance (West, 1984)
    {
        // High-altitude adaptations (Monge & Leon-Velarde, 1991)
        float base_altitude = 3000.0f; // Meters
        
        if (body_mass < 50.0f) {
            base_altitude = 5000.0f; // Small mammals tolerate altitude better
        }
        
        profile.altitude_tolerance_m = base_altitude;
    }
    
    // ========== REPRODUCTIVE CONSTRAINTS ==========
    
    // Breeding seasonality (Bronson, 1989)
    {
        if (body_mass > 100.0f) {
            profile.breeding_seasonality = 0.8f; // Large mammals seasonal
        } else {
            profile.breeding_seasonality = 0.3f; // Small mammals less seasonal
        }
    }
    
    // Gestation period (Sacher & Staffeldt, 1974)
    {
        // Gestation ∝ M^0.19 (Sacher & Staffeldt, 1974)
        profile.gestation_period_days = 30.0f * std::pow(body_mass, 0.19f);
    }
    
    // Offspring precocity (Derrickson, 1992)
    {
        if (analysis.cursorial_capability.value > 0.7f) {
            profile.offspring_precocity = 0.9f; // Precocial (ungulates)
        } else if (analysis.arboreal_capability.value > 0.7f) {
            profile.offspring_precocity = 0.3f; // Altricial (primates)
        } else {
            profile.offspring_precocity = 0.5f;
        }
    }
    
    // ========== COMPONENT RECOMMENDATIONS ==========
    
    // AI component selection
    {
        if (profile.gregariousness > 0.7f && profile.predatory_capability < 0.3f) {
            profile.recommended_ai_component = HerdBehavior;
        } else if (profile.gregariousness > 0.6f && profile.predatory_capability > 0.7f) {
            profile.recommended_ai_component = PackHunter;
        } else if (profile.predatory_capability > 0.7f) {
            profile.recommended_ai_component = SolitaryPredator;
        } else if (profile.territoriality > 0.7f) {
            profile.recommended_ai_component = TerritorialDefender;
        } else if (analysis.fossorial_capability.value > 0.7f) {
            profile.recommended_ai_component = ColonialBurrower;
        } else {
            profile.recommended_ai_component = OpportunisticForager;
        }
    }
    
    // Animation component selection
    {
        if (profile.primary_locomotion == Saltatorial) {
            profile.recommended_animation = BipedalHopper;
        } else if (profile.primary_locomotion == Arboreal) {
            profile.recommended_animation = ArborealClimber;
        } else if (profile.primary_locomotion == Aquatic) {
            profile.recommended_animation = AquaticSwimmer;
        } else if (analysis.cursorial_capability.value > 0.8f) {
            profile.recommended_animation = CursorsSprinter;
        } else {
            profile.recommended_animation = QuadrupedalWalker;
        }
    }
    
    // Simulation priority
    {
        profile.simulation_priority = 
            profile.predatory_capability * 0.3f +
            profile.gregariousness * 0.2f +
            std::clamp(body_mass / 100.0f, 0.0f, 1.0f) * 0.3f +
            profile.activity_level * 0.2f;
    }
    
    return profile;
}

// ========== ANIMATION PARAMETERS ==========

LocomotionAnimationParameters LocomotionAnimationParameters::Factory(
    const MammalTonTonInputs& inputs,
    const MorphologicalAnalysis& analysis,
    const MammalBehavioralProfile& behavior) {
    
    LocomotionAnimationParameters params = {};
    
    float body_mass = analysis.body_mass_kg;
    float body_length = analysis.body_length_m;
    
    // ========== WALKING GAIT ==========
    
    // Walk frequency scaling (Heglund et al., 1974)
    {
        // f ∝ M^-0.14 (Heglund et al., 1974)
        params.walk_frequency_hz = 2.0f * std::pow(body_mass, -0.14f);
        params.walk_frequency_hz = std::clamp(params.walk_frequency_hz, 0.5f, 4.0f);
    }
    
    // Walk stride length (Alexander, 1977)
    {
        // Typically 1.5-2.0 BL (Alexander, 1977)
        params.walk_stride_length_m = body_length * 1.7f;
        
        if (behavior.limb_angle_degrees > 75.0f) {
            params.walk_stride_length_m *= 1.2f; // Erect posture = longer stride
        }
    }
    
    // Walk duty factor (Hildebrand, 1968)
    {
        params.walk_duty_factor = 0.6f;
        
        // Giants have higher duty factor (Alexander & Jayes, 1983)
        if (body_mass > 1000.0f) {
            params.walk_duty_factor = 0.7f;
        }
    }
    
    // Lateral sequence (Hildebrand, 1968)
    {
        // Most mammals use lateral sequence walks
        params.walk_lateral_sequence = 0.75f; // 0.5=diagonal, 1.0=lateral
        
        if (analysis.arboreal_capability.value > 0.7f) {
            params.walk_lateral_sequence = 0.5f; // Diagonal for climbing
        }
    }
    
    // ========== TROTTING GAIT ==========
    
    // Trot frequency (Heglund et al., 1974)
    {
        params.trot_frequency_hz = params.walk_frequency_hz * 1.5f;
        params.trot_frequency_hz = std::clamp(params.trot_frequency_hz, 1.0f, 6.0f);
    }
    
    // Trot stride length
    {
        params.trot_stride_length_m = params.walk_stride_length_m * 1.3f;
    }
    
    // Trot duty factor (Hildebrand, 1968)
    {
        params.trot_duty_factor = 0.45f;
        
        if (analysis.cursorial_capability.value > 0.8f) {
            params.trot_duty_factor = 0.40f; // Lower DF = aerial phase
        }
    }
    
    // Suspension phase (Hildebrand, 1977)
    {
        float stride_time = 1.0f / params.trot_frequency_hz;
        params.trot_suspension_phase = stride_time * (1.0f - params.trot_duty_factor * 2.0f);
        params.trot_suspension_phase = std::max(params.trot_suspension_phase, 0.0f);
    }
    
    // ========== GALLOPING GAIT ==========
    
    // Gallop frequency (Heglund et al., 1974)
    {
        params.gallop_frequency_hz = params.trot_frequency_hz * 1.2f;
        params.gallop_frequency_hz = std::clamp(params.gallop_frequency_hz, 1.5f, 8.0f);
    }
    
    // Gallop stride length (Alexander, 1977)
    {
        params.gallop_stride_length_m = params.trot_stride_length_m * 1.8f;
        
        // Flexible spine extends stride (Hildebrand, 1977)
        params.gallop_stride_length_m *= (1.0f + behavior.spinal_flexion_range / M_PI);
    }
    
    // Gallop duty factor
    {
        params.gallop_duty_factor = 0.35f;
        
        if (analysis.cursorial_capability.value > 0.8f) {
            params.gallop_duty_factor = 0.25f; // Extended flight phase
        }
    }
    
    // Spinal flexion contribution (Hildebrand, 1977)
    {
        params.gallop_flexion_extension = behavior.spinal_flexion_range;
    }
    
    // Gallop type (Hildebrand, 1977)
    {
        params.gallop_rotary_vs_transverse = behavior.uses_gallop_rotary;
    }
    
    // ========== HOPPING/SALTATION ==========
    
    if (behavior.primary_locomotion == Saltatorial) {
        // Hop frequency (Dawson & Taylor, 1973)
        params.hop_frequency_hz = 1.5f * std::pow(body_mass, -0.14f);
        
        // Ground contact time (Alexander, 1974)
        params.hop_ground_contact_time = 0.15f / params.hop_frequency_hz.value();
        
        // Aerial time
        float hop_period = 1.0f / params.hop_frequency_hz.value();
        params.hop_aerial_time = hop_period - params.hop_ground_contact_time.value();
        
        // Elastic energy recovery (Biewener & Blickhan, 1988)
        params.hop_energy_recovery = 0.7f; // Up to 70% return from tendons
    }
    
    // ========== CLIMBING KINEMATICS ==========
    
    if (analysis.arboreal_capability.value > 0.5f) {
        // Climb speed (Cartmill, 1985)
        params.climb_speed_BLs = 0.5f * analysis.arboreal_capability.value;
        
        // Climb frequency (slower than walking)
        params.climb_stride_frequency = params.walk_frequency_hz * 0.6f;
        
        // Descent speed (typically faster) (Jenkins & McClearn, 1984)
        params.descent_speed_ratio = 1.3f;
        
        // Head-first descent capability (Jenkins & McClearn, 1984)
        params.can_descend_head_first = analysis.claw_curvature > 0.6f;
    }
    
    // ========== SWIMMING KINEMATICS ==========
    
    if (analysis.aquatic_capability.value > 0.5f) {
        // Swim speed (Fish & Baudinette, 1999)
        params.swim_speed_mps = body_length * analysis.aquatic_capability.value * 2.0f;
        
        // Paddle frequency
        params.paddle_frequency_hz = params.walk_frequency_hz * 0.8f;
        
        // Dive duration (Kooyman et al., 1981)
        float lung_volume = body_mass * 0.06f; // L (6% body mass)
        float o2_storage = lung_volume * 0.2f; // L O₂
        float metabolic_rate = behavior.basal_metabolic_rate_W / 20.0f; // L O₂/min
        params.dive_duration_s = (o2_storage / metabolic_rate) * 60.0f * 0.7f; // 70% usable
        
        // Dive depth (Schreer & Kovacs, 1997)
        params.dive_depth_m = 10.0f * analysis.aquatic_capability.value * 
            std::log10(body_mass + 1.0f);
    }
    
    // ========== BODY ARTICULATION ==========
    
    // Neck flexibility (typical mammalian range)
    params.neck_flexibility_deg = 120.0f; // Left-right
    if (analysis.is_graviportal) {
        params.neck_flexibility_deg = 60.0f; // Giants less flexible
    }
    
    // Torso flexibility
    params.torso_lateral_flex_deg = 30.0f + analysis.vertebral_flexibility * 30.0f;
    params.torso_dorsoventral_flex_deg = behavior.spinal_flexion_range * 180.0f / M_PI;
    
    // Tail articulation
    params.tail_articulation_segments = analysis.tail_length_ratio * 20.0f;
    if (analysis.has_prehensile_tail) {
        params.tail_articulation_segments = std::max(
            params.tail_articulation_segments, 15.0f);
    }
    
    // ========== LIMB KINEMATICS ==========
    
    // Protraction/retraction angles (typical mammalian ROM)
    {
        float base_protraction = 45.0f;
        float base_retraction = 60.0f;
        
        // Cursorial adaptations (Garland & Janis, 1993)
        if (analysis.cursorial_capability.value > 0.7f) {
            base_protraction = 60.0f;
            base_retraction = 80.0f;
        }
        
        params.forelimb_protraction_deg = base_protraction;
        params.forelimb_retraction_deg = base_retraction;
        params.hindlimb_protraction_deg = base_protraction * 1.2f;
        params.hindlimb_retraction_deg = base_retraction * 1.3f;
    }
    
    // Joint flexion ranges
    params.elbow_flexion_range_deg = 120.0f;
    params.knee_flexion_range_deg = 140.0f;
    
    // ========== FOOT/PAW ARTICULATION ==========
    
    // Digit splay (Maiolino et al., 2016)
    {
        params.digit_splay_deg = 20.0f;
        
        if (analysis.arboreal_capability.value > 0.7f) {
            params.digit_splay_deg = 45.0f; // Wide splay for grip
        } else if (analysis.has_hooves) {
            params.digit_splay_deg = 0.0f; // No splay
        }
    }
    
    // Wrist/ankle flexibility
    {
        params.wrist_ankle_flexibility_deg = 90.0f;
        
        if (analysis.arboreal_capability.value > 0.7f) {
            params.wrist_ankle_flexibility_deg = 180.0f; // High mobility for climbing
        }
    }
    
    // Retractile claws
    if (analysis.has_claws_retractile) {
        params.claw_extension_mm = analysis.claw_length_ratio * body_length * 1000.0f * 0.02f;
    }
    
    // ========== TURNING DYNAMICS ==========
    
    // Minimum turn radius (Walter, 2003)
    {
        params.min_turn_radius_BL = 0.5f + 
            analysis.moment_of_inertia.y / (body_mass * body_length * body_length) * 3.0f;
        params.min_turn_radius_BL = std::clamp(params.min_turn_radius_BL, 0.3f, 5.0f);
    }
    
    // Turn initiation time (Jindrich & Full, 1999)
    {
        params.turn_initiation_time_s = 0.1f + body_mass * 0.01f;
        params.turn_initiation_time_s = std::clamp(
            params.turn_initiation_time_s, 0.05f, 0.5f);
    }
    
    // Pivot turn capability (small agile mammals)
    {
        params.pivot_turn_capability = std::clamp(
            (5.0f - body_mass) / 5.0f * behavior.maneuverability, 0.0f, 1.0f);
    }
    
    // ========== GAIT TRANSITION PARAMETERS ==========
    
    params.froude_walk_trot = behavior.walk_trot_froude;
    params.froude_trot_gallop = behavior.trot_gallop_froude;
    
    // Hysteresis (Alexander & Jayes, 1983)
    {
        params.hysteresis_bandwidth = 0.15f; // ±15% overlap
    }
    
    // ========== ENVIRONMENTAL SCALING ==========
    
    // Gravity scaling (Alexander, 1985)
    {
        params.gravity_scaling_exponent = 0.5f; // Speeds scale as √g
    }
    
    // Friction requirement
    {
        params.friction_requirement = 0.4f;
        
        if (analysis.cursorial_capability.value > 0.7f) {
            params.friction_requirement = 0.6f; // High-speed needs traction
        }
    }
    
    // Substrate stiffness
    {
        params.substrate_stiffness_optimal = 0.8f; // 0=soft, 1=rigid
        
        if (body_mass > 500.0f) {
            params.substrate_stiffness_optimal = 0.95f; // Giants need firm ground
        } else if (analysis.saltatorial_capability.value > 0.7f) {
            params.substrate_stiffness_optimal = 0.85f; // Hoppers need bounce
        }
    }
    
    return params;
}

} // namespace Mammalia

/*
# Mammalian Locomotion and Physiology Bibliography

## Allometry and Scaling
Alexander, R. McN. (1977). Allometry of the limbs of antelopes (Bovidae). Journal of Zoology, 183(1), 125-146.

Alexander, R. McN. (1985). Mechanics of posture and gait of some large dinosaurs. Zoological Journal of the Linnean Society, 83(1), 1-25.

Alexander, R. McN., Jayes, A. S., Maloiy, G. M. O., & Wathuta, E. M. (1979). Allometry of the limb bones of mammals from shrews (Sorex) to elephant (Loxodonta). Journal of Zoology, 189(3), 305-314.

Biewener, A. A. (1989). Scaling body support in mammals: limb posture and muscle mechanics. Science, 245(4913), 45-48.

Biewener, A. A. (1990). Biomechanics of mammalian terrestrial locomotion. Science, 250(4984), 1097-1103.

Carrano, M. T. (1999). What, if anything, is a cursor? Categories versus continua for determining locomotor habit in mammals and dinosaurs. Journal of Zoology, 247(1), 29-42.

Schmidt-Nielsen, K. (1984). Scaling: Why is Animal Size So Important? Cambridge University Press.

## Gait Mechanics and Transitions
Alexander, R. McN., & Jayes, A. S. (1983). A dynamic similarity hypothesis for the gaits of quadrupedal mammals. Journal of Zoology, 201(1), 135-152.

Heglund, N. C., Taylor, C. R., & McMahon, T. A. (1974). Scaling stride frequency and gait to animal size: mice to horses. Science, 186(4169), 1112-1113.

Hildebrand, M. (1968). Symmetrical gaits of dogs in relation to body build. Journal of Morphology, 124(3), 353-359.

Hildebrand, M. (1977). Analysis of asymmetrical gaits. Journal of Mammalogy, 58(2), 131-156.

Hoyt, D. F., & Taylor, C. R. (1981). Gait and the energetics of locomotion in horses. Nature, 292(5820), 239-240.

## Jumping and Elastic Energy Storage
Alexander, R. McN. (1974). The mechanics of jumping by a dog (Canis familiaris). Journal of Zoology, 173(4), 549-573.

Alexander, R. McN. (1975). Mechanics of hopping by kangaroos (Macropodidae). Journal of Zoology, 177(2), 265-303.

Alexander, R. McN. (1988). Elastic Mechanisms in Animal Movement. Cambridge University Press.

Alexander, R. McN. (1995). Leg design and jumping technique for humans, other vertebrates and insects. Philosophical Transactions of the Royal Society B, 347(1321), 235-248.

Biewener, A. A., & Blickhan, R. (1988). Kangaroo rat locomotion: design for elastic energy storage or acceleration? Journal of Experimental Biology, 140(1), 243-255.

Biewener, A. A., Alexander, R. McN., & Heglund, N. C. (1981). Elastic energy storage in the hopping of kangaroo rats (Dipodomys spectabilis). Journal of Zoology, 195(3), 369-383.

Dawson, T. J., & Taylor, C. R. (1973). Energetic cost of locomotion in kangaroos. Nature, 246(5431), 313-314.

## Arboreal Locomotion and Climbing
Autumn, K., Liang, Y. A., Hsieh, S. T., Zesch, W., Chan, W. P., Kenny, T. W., ... & Full, R. J. (2000). Adhesive force of a single gecko foot-hair. Nature, 405(6787), 681-685.

Birn-Jeffery, A. V., Miller, C. E., Naish, D., Rayfield, E. J., & Hone, D. W. E. (2012). Pedal claw curvature in birds, lizards and Mesozoic dinosaurs–complicated categories and compensating for mass-specific and phylogenetic control. PLoS ONE, 7(12), e50555.

Cartmill, M. (1985). Climbing. In M. Hildebrand, D. M. Bramble, K. F. Liem, & D. B. Wake (Eds.), Functional Vertebrate Morphology (pp. 73-88). Harvard University Press.

Jenkins, F. A., & McClearn, D. (1984). Mechanisms of hind foot reversal in climbing mammals. Journal of Morphology, 182(2), 197-219.

Kirk, E. C., Lemelin, P., Hamrick, M. W., Boyer, D. M., & Bloch, J. I. (2008). Intrinsic hand proportions of euarchontans and other mammals: implications for the locomotor behavior of plesiadapiforms. Journal of Human Evolution, 55(2), 278-299.

Maiolino, S., Boyer, D. M., Bloch, J. I., Gilbert, C. C., & Groenke, J. (2016). Evidence for a grooming claw in a North American adapiform primate: implications for anthropoid origins. PLoS ONE, 11(10), e0163e0163719.

Preuschoft, H., & Witte, H. (1991). Biomechanical reasons for the evolution of hominid body shape. In Y. Coppens & B. Senut (Eds.), Origine(s) de la Bipédie chez les Hominidés (pp. 59-77). CNRS.

Zani, P. A. (2000). The comparative evolution of lizard claw and toe morphology and clinging performance. Journal of Evolutionary Biology, 13(2), 316-325.

## Fossorial Locomotion
Casinos, A., Quintana, C., & Viladiu, C. (1993). Allometry and adaptation in the long bones of a digging group of rodents (Ctenomyinae). Zoological Journal of the Linnean Society, 107(2), 107-115.

Carotenuto, F., Barbera, C., & Raia, P. (2020). Burrowing below ground: interaction between soil mechanics and evolution of subterranean mammals. Journal of the Royal Society Interface, 17(164), 20190521.

Hopkins, S. S. B. (2007). Causes of lineage decline in the Aplodontidae: testing for the influence of physical and biological change. Palaeogeography, Palaeoclimatology, Palaeoecology, 246(2-4), 331-353.

## Aquatic and Semi-Aquatic Locomotion
Fish, F. E. (1993). Influence of hydrodynamic design and propulsive mode on mammalian swimming energetics. Australian Journal of Zoology, 42(1), 79-101.

Fish, F. E., & Baudinette, R. V. (1999). Energetics of locomotion by the Australian water rat (Hydromys chrysogaster): a comparison of swimming and running in a semi-aquatic mammal. Journal of Experimental Biology, 202(4), 353-363.

Kooyman, G. L., Wahrenbrock, E. A., Castellini, M. A., Davis, R. W., & Sinnett, E. E. (1981). Aerobic and anaerobic metabolism during voluntary diving in Weddell seals: evidence of preferred pathways from blood chemistry and behavior. Journal of Comparative Physiology B, 138(4), 335-346.

Schreer, J. F., & Kovacs, K. M. (1997). Allometry of diving capacity in air-breathing vertebrates. Canadian Journal of Zoology, 75(3), 339-358.

Williams, T. M. (1999). The evolution of cost efficient swimming in marine mammals: limits to energetic optimization. Philosophical Transactions of the Royal Society B, 354(1380), 193-201.

## Metabolic Scaling
Kleiber, M. (1947). Body size and metabolic rate. Physiological Reviews, 27(4), 511-541.

Lovegrove, B. G. (2000). The zoogeography of mammalian basal metabolic rate. American Naturalist, 156(2), 201-219.

Nagy, K. A., Girard, I. A., & Brown, T. K. (1999). Energetics of free-ranging mammals, reptiles, and birds. Annual Review of Nutrition, 19(1), 247-277.

Rome, L. C., Funke, R. P., Alexander, R. M., Lutz, G., Aldridge, H., Scott, F., & Freadman, M. (1988). Why animals have different muscle fibre types. Nature, 335(6193), 824-827.

Taylor, C. R., Maloiy, G. M. O., Weibel, E. R., Langman, V. A., Kamau, J. M. Z., Seeherman, H. J., & Heglund, N. C. (1981). Design of the mammalian respiratory system. III. Scaling maximum aerobic capacity to body mass: wild and domestic mammals. Respiration Physiology, 44(1), 25-37.

Taylor, C. R., Heglund, N. C., & Maloiy, G. M. O. (1982). Energetics and mechanics of terrestrial locomotion. I. Metabolic energy consumption as a function of speed and body size in birds and mammals. Journal of Experimental Biology, 97(1), 1-21.

Weibel, E. R., Bacigalupe, L. D., Schmitt, B., & Hoppeler, H. (2004). Allometric scaling of maximal metabolic rate in mammals: muscle aerobic capacity as determinant factor. Respiratory Physiology & Neurobiology, 140(2), 115-132.

White, C. R., & Seymour, R. S. (2003). Mammalian basal metabolic rate is proportional to body mass^2/3. Proceedings of the National Academy of Sciences, 100(7), 4046-4049.

## Thermoregulation
Bradley, S. R., & Deavers, D. R. (1980). A re-examination of the relationship between thermal conductance and body weight in mammals. Comparative Biochemistry and Physiology Part A, 65(4), 465-476.

Chappell, M. A., & Bartholomew, G. A. (1981). Activity and thermoregulation of the antelope ground squirrel Ammospermophilus leucurus in winter and summer. Physiological Zoology, 54(2), 215-223.

Dawson, T. J., & Schmidt-Nielsen, K. (1966). Effect of thermal conductance on water economy in the antelope jack rabbit, Lepus alleni. Journal of Cellular Physiology, 67(3), 463-471.

Geiser, F. (2004). Metabolic rate and body temperature reduction during hibernation and daily torpor. Annual Review of Physiology, 66, 239-274.

Scholander, P. F., Hock, R., Walters, V., Johnson, F., & Irving, L. (1950). Heat regulation in some arctic and tropical mammals and birds. Biological Bulletin, 99(2), 237-258.

Schmidt-Nielsen, K. (1964). Desert Animals: Physiological Problems of Heat and Water. Oxford University Press.

Taylor, C. R. (1969). The eland and the oryx. Scientific American, 220(1), 88-95.

## Sensory Ecology
Heesy, C. P. (2008). Ecomorphology of orbit orientation and the adaptive significance of binocular vision in primates and other mammals. Brain, Behavior and Evolution, 71(1), 54-67.

Heffner, R. S., & Heffner, H. E. (2008). High-frequency hearing. In A. I. Basbaum, A. Kaneko, G. M. Shepherd, & G. Westheimer (Eds.), The Senses: A Comprehensive Reference (Vol. 3, pp. 55-60). Academic Press.

Kirk, E. C. (2006). Eye morphology in cathemeral lemurids and other mammals. Folia Primatologica, 77(1-2), 27-49.

Muchlinski, M. N. (2010). Ecological correlates of primate olfactory bulb size. American Journal of Primatology, 72(4), 287-293.

## Social Behavior and Ecology
Brashares, J. S., & Arcese, P. (2002). Role of forage, habitat and predation in the behavioural plasticity of a small African antelope. Journal of Animal Ecology, 71(4), 626-638.

Bronson, F. H. (1989). Mammalian Reproductive Biology. University of Chicago Press.

Clutton-Brock, T. H., & Harvey, P. H. (1977). Primate ecology and social organization. Journal of Zoology, 183(1), 1-39.

Derrickson, E. M. (1992). Comparative reproductive strategies of altricial and precocial eutherian mammals. Functional Ecology, 6(1), 57-65.

Drews, C. (1993). The concept and definition of dominance in animal behaviour. Behaviour, 125(3-4), 283-313.

Dunbar, R. I. M. (1992). Neocortex size as a constraint on group size in primates. Journal of Human Evolution, 22(6), 469-493.

Dunbar, R. I. M., & Shultz, S. (2007). Evolution in the social brain. Science, 317(5843), 1344-1347.

Gittleman, J. L., & Harvey, P. H. (1982). Carnivore home-range size, metabolic needs and ecology. Behavioral Ecology and Sociobiology, 10(1), 57-63.

Grant, J. W. A. (1997). Territoriality. In J. J. Godin (Ed.), Behavioural Ecology of Teleost Fishes (pp. 81-103). Oxford University Press.

Harestad, A. S., & Bunnel, F. L. (1979). Home range and body weight—a reevaluation. Ecology, 60(2), 389-402.

Jarman, P. J. (1974). The social organisation of antelope in relation to their ecology. Behaviour, 48(3-4), 215-267.

McNab, B. K. (1963). Bioenergetics and the determination of home range size. American Naturalist, 97(894), 133-140.

Vander Wall, S. B. (1990). Food Hoarding in Animals. University of Chicago Press.

## Locomotor Performance
Carrier, D. R., Heglund, N. C., & Earls, K. D. (2001). Variable gearing during locomotion in the human musculoskeletal system. Science, 265(5172), 651-653.

Garland, T., Jr. (1983). The relation between maximal running speed and body mass in terrestrial mammals. Journal of Zoology, 199(2), 157-170.

Garland, T., Jr., & Janis, C. M. (1993). Does metatarsal/femur ratio predict maximal running speed in cursorial mammals? Journal of Zoology, 229(1), 133-151.

Grand, T. I. (1977). Body weight: its relation to tissue composition, segment distribution, and motor function. I. Interspecific comparisons. American Journal of Physical Anthropology, 47(2), 211-240.

Hochachka, P. W. (1985). Exercise limitations at high altitude: the metabolic problem and search for its solution. In K. B. Pandolf (Ed.), Exercise and Sport Sciences Reviews (Vol. 13, pp. 13-64). Macmillan.

Hochachka, P. W., Stanley, C., Merkt, J., & Sumar-Kalinowski, J. (1983). Metabolic meaning of elevated levels of oxidative enzymes in high altitude adapted animals: an interpretive hypothesis. Respiration Physiology, 52(3), 303-313.

Jindrich, D. L., & Full, R. J. (1999). Many-legged maneuverability: dynamics of turning in hexapods. Journal of Experimental Biology, 202(12), 1603-1623.

Lindstedt, S. L., & Boyce, M. S. (1985). Seasonality, fasting endurance, and body size in mammals. American Naturalist, 125(6), 873-878.

Monge, C., & Leon-Velarde, F. (1991). Physiological adaptation to high altitude: oxygen transport in mammals and birds. Physiological Reviews, 71(4), 1135-1172.

Pond, C. M. (1998). The Fats of Life. Cambridge University Press.

Sacher, G. A., & Staffeldt, E. F. (1974). Relation of gestation time to brain weight for placental mammals: implications for the theory of vertebrate growth. American Naturalist, 108(963), 593-615.

Walter, R. M. (2003). Kinematics of 90° running turns in wild mice. Journal of Experimental Biology, 206(10), 1739-1749.

Walter, R. M., & Carrier, D. R. (2007). Ground forces applied by galloping dogs. Journal of Experimental Biology, 210(2), 208-216.

Webster, K. N., & Dawson, T. J. (2003). Locomotion energetics and gait characteristics of a rat-kangaroo, Bettongia penicillata, have some kangaroo-like features. Journal of Comparative Physiology B, 173(7), 549-557.

West, J. B. (1984). Human physiology at extreme altitudes on Mount Everest. Science, 223(4638), 784-788.

Wroe, S., McHenry, C., & Thomason, J. (2005). Bite club: comparative bite force in big biting mammals and the prediction of predatory behaviour in fossil taxa. Proceedings of the Royal Society B, 272(1563), 619-625.

Zielinski, W. J. (1988). The influence of daily variation in foraging cost on the activity of small carnivores. Animal Behaviour, 36(1), 239-249.

## Additional References
Bennie, J. J., Duffy, J. P., Inger, R., & Gaston, K. J. (2014). Biogeography of time partitioning in mammals. Proceedings of the National Academy of Sciences, 111(38), 13727-13732.

Biewener, A. A., & Taylor, C. R. (1986). Bone strain: a determinant of gait and speed? Journal of Experimental Biology, 123(1), 383-400.

Roll, U., Dayan, T., & Kronfeld-Schor, N. (2006). On the role of phylogeny in determining activity patterns of rodents. Evolutionary Ecology, 20(5), 479-490.

Slijper, E. J. (1946). Comparative biologic-anatomical investigations on the vertebral column and spinal musculature of mammals. Verhandelingen der Koninklijke Nederlandse Akademie van Wetenschappen, Afdeling Natuurkunde, 42, 1-128.

Taylor, C. R., Caldwell, S. L., & Rowntree, V. J. (1972). Running up and down hills: some consequences of size. Science, 178(4065), 1096-1097.
*/
