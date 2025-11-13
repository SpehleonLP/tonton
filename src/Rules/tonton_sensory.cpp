#include "tonton_sensory.h"
#include "../include/tonton_input.h"
#include "../include/tonton_output.h"
#include "Memos/tonton_armaturememo.h"
#include "Rules/tonton_scratch.h"

std::pair<glm::quat, glm::vec3> EigenDecomposition(glm::mat3 const& m);
std::pair<glm::quat, glm::vec3> EigenDecomposition(glm::dmat3 const& m);

namespace TonTon
{
struct EyeInfo {
    uint16_t joint_index;
    glm::vec3 position;           // In rest pose
    glm::vec3 base_position;      // Where eyestalk attaches (if applicable)
    glm::vec3 pointing_direction; // Forward vector of this eye
    bool is_on_stalk;
    float stalk_length_m;
    float eye_diameter_m;
    float mobility_rad;           // How much can it rotate? (stalk vs fixed)
};
static std::vector<EyeInfo> FindEyes(Input const& in, Scratch const&) {
	using SF = SemanticFlags;
	
    auto & sk = *in.skinnedMesh;
    auto parents = sk.skin->parents.data();
    auto semantic_flags = sk.skin->memo()->GetSemanticFlags();
    
    std::vector<EyeInfo> eyes;
    
    for(uint32_t i = 0; i < sk.skin->names.size(); ++i) 
    {      
        if(!HasFlag(semantic_flags[i], SF::VISION)) continue;
        
        EyeInfo eye;
        eye.joint_index = i;
        eye.position = in.position(i);
        
        // Estimate eye size
        double eye_volume = sk.volume[i] * in.behavior.volume_scale();
        eye.eye_diameter_m = std::cbrt(eye_volume * 6.0 / 3.14159);
        
        // ===================================================================
        // DETECT EYESTALK - find nearest body attachment
        // ===================================================================
        
        // Walk up to find head/body (not tagged as appendage)
        int body_attachment = -1;
        for(int j = parents[i]; j >= 0; j = parents[j]) {
        
            if(HasFlag(semantic_flags[i], SF::HEAD|SF::SPINE|SF::ABDOMEN|SF::LIMB|SF::TAIL))
            {
                body_attachment = j;
                break;
            }
        }
        
        if(body_attachment < 0) {
            body_attachment = parents[i]; // Fallback to parent
        }
        
        // Check if there's a stalk between body and eye
        SkinnedMesh::StalkData stalk;
        eye.is_on_stalk = sk.GetStalkData(stalk, body_attachment, i, in.behavior.scale);
        
        if(eye.is_on_stalk) {
            eye.base_position = in.position(stalk.root);
            eye.stalk_length_m = stalk.length_m;
            
            // Eye points away from base
            eye.pointing_direction = glm::normalize(eye.position - eye.base_position);
            
            // Stalks are mobile
            eye.mobility_rad = glm::radians(90.0f);
            
        } else {
            // Fixed eye
            eye.base_position = in.position(body_attachment);
            
            // Eye points radially + respects local rotation
            glm::vec3 radial = glm::normalize(eye.position - eye.base_position);
            glm::quat local_rot = sk.skin->rotation[i];
            glm::vec3 local_forward = local_rot * glm::vec3(0, 0, 1);
            
            eye.pointing_direction = glm::normalize(radial * 0.3f + local_forward * 0.7f);
            eye.mobility_rad = glm::radians(15.0f);
            eye.stalk_length_m = 0.0f;
        }
        
        eyes.push_back(eye);
    }
    
    return eyes;
}

static Output_Vision ComputeVision(Input const& in, Scratch const& s) {
    Output_Vision vision{};
    
    auto gcr_table = in.skinnedMesh->skin->memo()->GetGcrTable();
    auto N = in.skinnedMesh->skin->names.size();
    auto eyes = FindEyes(in, s);
    
    if(eyes.empty()) {
        // No eyes = blind (return nullopt in calling code if you want)
        return vision;
    }
    
    // ===================================================================
    // ACUITY - based on eye size
    // ===================================================================
    
    float max_eye_diameter = 0.0f;
    for(auto const& eye : eyes) {
        max_eye_diameter = std::max(max_eye_diameter, eye.eye_diameter_m);
    }
    
    // Scale: 0.002m (ant) -> 0.05m (human) -> 0.1m (horse) -> 0.3m (giant squid)
    float geometric_acuity = std::clamp(max_eye_diameter / 0.05f, 0.0f, 1.0f);
    
    // Behavioral modifier: diurnal animals need better vision
    float activity_bonus = in.behavior.activity_pattern;
    vision.acuity = geometric_acuity * glm::mix(0.7f, 1.3f, activity_bonus);
    vision.acuity = std::clamp(vision.acuity, 0.0f, 1.0f);
    
    // ===================================================================
    // BINOCULAR OVERLAP - field of view intersection
    // ===================================================================
    
    if(eyes.size() >= 2) {
        // Find the two most "forward-facing" eyes for binocular vision
        // (The ones with the most similar pointing directions)
        
        int best_pair[2] = {0, 1};
        float best_alignment = -1.0f;
        
        for(size_t i = 0; i < eyes.size(); ++i) {
            for(size_t j = i + 1; j < eyes.size(); ++j) {
                float alignment = glm::dot(eyes[i].pointing_direction, 
                                          eyes[j].pointing_direction);
                if(alignment > best_alignment) {
                    best_alignment = alignment;
                    best_pair[0] = i;
                    best_pair[1] = j;
                }
            }
        }
        
        auto & eye_A = eyes[best_pair[0]];
        auto & eye_B = eyes[best_pair[1]];
        
        // Get their common ancestor
        int common_root = gcr_table[eye_A.joint_index * N + eye_B.joint_index];
        
        glm::vec3 common_pos = in.position(common_root);
        
        // Vectors from common root to each eye
        glm::vec3 to_A = eye_A.position - common_pos;
        glm::vec3 to_B = eye_B.position - common_pos;
        
        // Plane normal (perpendicular to both eye vectors)
        glm::vec3 plane_normal = glm::normalize(glm::cross(to_A, to_B));
        
        // Forward direction (average of eye pointing directions)
        glm::vec3 forward = glm::normalize(eye_A.pointing_direction + 
                                          eye_B.pointing_direction);
        
        // Project eye positions onto the plane perpendicular to forward
        glm::vec3 right = glm::normalize(glm::cross(forward, plane_normal));
        
        // Angular separation between eyes (in the horizontal plane)
        float angle_A = std::atan2(glm::dot(to_A, right), glm::dot(to_A, forward));
        float angle_B = std::atan2(glm::dot(to_B, right), glm::dot(to_B, forward));
        
        float angular_separation_rad = std::abs(angle_B - angle_A);
        
        // Typical FOV per eye: ~90-180 degrees depending on eye type
        // Predators: narrow FOV (~60°), Prey: wide FOV (~180°)
        float fov_per_eye_rad = glm::mix(glm::radians(60.0f), 
                                     glm::radians(150.0f),
                                     1.0f - in.behavior.aggression_adjustment);
        
        // Binocular overlap = (2 × FOV - separation) / FOV
        // If eyes are close together and pointing same direction: high overlap
        // If eyes are on sides of head: low/no overlap
        float overlap_angle = 2.0f * fov_per_eye_rad - angular_separation_rad;
        vision.binocular_overlap = std::clamp(overlap_angle / fov_per_eye_rad, 0.0f, 1.0f);
        
        // ===============================================================
        // CENTERING - for asymmetric creatures (flatfish!)
        // ===============================================================
        
        // Check if eyes are on opposite sides (normal) or same side (flatfish)
        float left_right_balance = (angle_A + angle_B) / 2.0f;
        
        // If both eyes are on left (negative) or both on right (positive): asymmetric
        vision.centering = std::clamp(left_right_balance / glm::radians(90.0f), -1.0f, 1.0f);
        
    } else if(eyes.size() == 1) {
        // Monocular vision
        vision.binocular_overlap = 0.0f;
        vision.centering = 0.0f; // Single eye is "centered" by default
        
    } else {
        // Multiple eyes but strange configuration - use heuristics
        vision.binocular_overlap = 0.3f; // Assume some overlap
        vision.centering = 0.0f;
    }
    
    // ===================================================================
    // COLOR & NIGHT VISION
    // ===================================================================
    
    vision.has_color_vision = (in.behavior.activity_pattern > 0.4f); // Diurnal
    vision.has_night_vision = (in.behavior.activity_pattern < 0.6f); // Nocturnal
    
    // ===================================================================
    // DETECTION RANGE
    // ===================================================================
    
    float scale_factor = std::cbrt(s.physical.body_volume_m3);
    vision.detection_range_m = scale_factor * 100.0f * vision.acuity;
    
    // Eyestalks give better view distance (periscope effect)
    bool has_eyestalks = false;
    float max_stalk_height = 0.0f;
    for(auto const& eye : eyes) {
        if(eye.is_on_stalk) {
            has_eyestalks = true;
            max_stalk_height = std::max(max_stalk_height, eye.stalk_length_m);
        }
    }
    
    if(has_eyestalks) {
        // Higher eyes see further (horizon distance)
        float horizon_bonus = 1.0f + std::sqrt(max_stalk_height * 2.0f); // Rough approximation
        vision.detection_range_m *= horizon_bonus;
    }
    
    vision.detection_range_m = std::clamp(vision.detection_range_m, 1.0f, 10000.0f);
    
    return vision;
}

static std::optional<Output_Hearing> ComputeHearing(Input const& in, Scratch const& s) {
    Output_Hearing hearing{};
    auto & physical = s.physical;
    
    // BASE VALUES from geometry
    float ear_surface_area = 0.0f;
    bool has_external_ears = false;
    
    for(uint32_t i = 0; i < in.skinnedMesh->skin->names.size(); ++i) {
        for(auto word : in.skinnedMesh->skin->tags[i]) {
            if(word == Word::ear || word == Word::pinna) {
                has_external_ears = true;
                ear_surface_area += in.skinnedMesh->surfaceArea[i] * in.behavior.area_scale();
                break;
            }
        }
    }
        
    // SENSITIVITY: Blend geometry + social tendency + activity pattern
    float geometric_sensitivity = 0.5f; // Default
    if(has_external_ears) {
        // Larger ears = better hearing (elephant ears = 2 m², mouse ears = 0.0001 m²)
        geometric_sensitivity = std::clamp(ear_surface_area / 0.01f, 0.3f, 1.0f);
    }
    
    // Social animals need good hearing (communication)
    // Nocturnal animals need good hearing (can't see)
    float social_bonus = in.behavior.social_tendency;
    float nocturnal_bonus = 1.0f - in.behavior.activity_pattern;
    float behavioral_sensitivity = std::max(social_bonus, nocturnal_bonus);
    
    hearing.sensitivity = (geometric_sensitivity + behavioral_sensitivity) * 0.5f;
    hearing.sensitivity = std::clamp(hearing.sensitivity, 0.0f, 1.0f);
    
    // FREQUENCY RANGE: Allometric scaling from body mass
    // Small animals hear high frequencies (mice: 1-100 kHz)
    // Large animals hear low frequencies (elephants: 16-12000 Hz)
    float body_mass_kg = physical.body_mass_kg;
    
    hearing.frequency_range_Hz_min = 20.0f * std::pow(body_mass_kg, -0.15f);
    hearing.frequency_range_Hz_max = 20000.0f * std::pow(body_mass_kg, -0.25f);
    
    // Clamp to biological reality
    hearing.frequency_range_Hz_min = std::clamp(hearing.frequency_range_Hz_min, 10.0f, 100.0f);
    hearing.frequency_range_Hz_max = std::clamp(hearing.frequency_range_Hz_max, 1000.0f, 120000.0f);
    
    // DETECTION RANGE: Larger animals + better sensitivity = longer range
    float size_factor = std::cbrt(body_mass_kg); // Cube root for volume->length
    hearing.detection_range_m = size_factor * 50.0f * hearing.sensitivity;
    hearing.detection_range_m = std::clamp(hearing.detection_range_m, 5.0f, 5000.0f);
    
    return hearing;
}
static std::optional<Output_Olfaction> ComputeOlfaction(
    Input const& in, 
    Scratch const& s,
    immutable_array<Output_Chain> antennae)
{
    Output_Olfaction olfaction{};
    
    // BASE VALUES from geometry
    float nasal_surface_area = 0.0f;
    bool has_snout = false;
    
    auto & sk = *in.skinnedMesh;
    
    // Scan for vertebrate olfactory structures
    for(uint32_t i = 0; i < sk.skin->names.size(); ++i) {
        for(auto word : sk.skin->tags[i]) {
            if(word == Word::snout || word == Word::nose || word == Word::nostril) {
                has_snout = true;
                nasal_surface_area += sk.surfaceArea[i] * in.behavior.area_scale();
                break;
            }
        }
    }
    
    // ANTENNAE contribution (invertebrate chemoreception)
    float antennal_surface_area = 0.0f;
    bool has_sensory_antennae = false;
    
    if(!antennae.empty()) {
        for(auto const& antenna : antennae) {
            // Get all joints in the antenna chain
            auto relevant_joints = sk.skin->memo()->GetAllChildrenOfRoot(antenna.root);
            
            // Check if antenna is tagged as sensory
            bool is_sensory = false;
            for(auto joint : relevant_joints) {
                for(auto word : sk.skin->tags[joint]) {
                    if(word == Word::sensory || word == Word::chemoreceptor) {
                        is_sensory = true;
                        break;
                    }
                }
                if(is_sensory) break;
            }
            
            if(is_sensory || antenna.stretched_length_m > 0) {
                has_sensory_antennae = true;
                
                // Estimate surface area of antenna
                for(auto joint : relevant_joints) {
                    antennal_surface_area += sk.surfaceArea[joint] * in.behavior.area_scale();
                }
            }
        }
    }
    
    // If no olfactory organs at all, return nullopt
    if(!has_snout && !has_sensory_antennae) {
        return std::nullopt;
    }
    
    // SENSITIVITY CALCULATION
    float geometric_sensitivity = 0.0f;
    
    if(has_snout) {
        // VERTEBRATE OLFACTION
        // Dog snout: ~150 cm² epithelium, Human: ~5 cm²
        float surface_cm2 = nasal_surface_area * 10000.0f;
        geometric_sensitivity = std::clamp(surface_cm2 / 150.0f, 0.2f, 1.0f);
    }
    
    if(has_sensory_antennae) {
        // ARTHROPOD CHEMORECEPTION
        // Antennae are extremely sensitive - moths can detect pheromones at ppb
        // Surface area is key: more sensilla = better detection
        
        float antennal_cm2 = antennal_surface_area * 10000.0f;
        
        // Insects have amazing olfaction despite small antennae
        // Scale differently than vertebrate noses
        // Ant antenna: ~0.01 cm², Moth antenna: ~1 cm²
        float antennal_sensitivity = std::clamp(antennal_cm2 / 1.0f, 0.4f, 1.0f);
        
        // Antennae are often MORE sensitive than noses (for specific compounds)
        geometric_sensitivity = std::max(geometric_sensitivity, antennal_sensitivity);
    }
    
    // BEHAVIORAL MODIFIERS
    
    // Nocturnal animals rely more on smell
    float nocturnal_bonus = 1.0f - in.behavior.activity_pattern;
    
    // Predators (high aggression) often have good smell for tracking prey
    float predator_bonus = in.behavior.aggression_adjustment;
    
    // Social animals may use pheromones (especially with antennae)
    float social_bonus = has_sensory_antennae ? in.behavior.social_tendency * 0.5f : 0.0f;
    
    float behavioral_sensitivity = std::max({
        nocturnal_bonus * 0.5f,
        predator_bonus * 0.3f,
        social_bonus
    });
    
    // Blend geometric and behavioral
    olfaction.sensitivity = (geometric_sensitivity * 0.7f + behavioral_sensitivity * 0.3f);
    olfaction.sensitivity = std::clamp(olfaction.sensitivity, 0.0f, 1.0f);
    
    // DETECTION RANGE
    
    if(has_sensory_antennae) {
        // Arthropod chemoreception: extremely sensitive but shorter range
        // Ants: 1-10m, Moths (pheromone): up to 10km for specific compounds
        
        // Base range on sensitivity and antenna length
        float max_antenna_length = 0.0f;
        for(auto const& antenna : antennae) {
            max_antenna_length = std::max(max_antenna_length, antenna.stretched_length_m);
        }
        
        // Longer antennae = better directionality and sampling volume
        float length_factor = std::clamp(max_antenna_length / 0.1f, 0.5f, 2.0f);
        
        olfaction.detection_range_m = glm::mix(5.0f, 1000.0f, olfaction.sensitivity) * length_factor;
        
    } else {
        // Vertebrate olfaction: moderate sensitivity, long range possible
        // Poor: 1m (most birds), Moderate: 50m (humans), Excellent: 20km (polar bears)
        olfaction.detection_range_m = glm::mix(1.0f, 5000.0f, olfaction.sensitivity);
        
        // Size scaling: larger animals can detect scents from further away
        float size_bonus = std::pow(s.physical.body_mass_kg, 0.2f);
        olfaction.detection_range_m *= size_bonus;
    }
    
    olfaction.detection_range_m = std::clamp(olfaction.detection_range_m, 0.5f, 20000.0f);
    
    return olfaction;
}

Output_Sensory<std::optional>  ComputeSensory(Input const& in, Scratch & s)
{
	Output_Sensory<std::optional> out;
	
	std::array<Word, 1> words = {Word::antenna}; 
	auto chains = GetChainsFromRoot(in, std::span<Word>{words});
	
	out.antennae = shared_array<Output_Chain>::FromArray(chains);
	
	out.vision = ComputeVision(in, s);
	out.hearing = ComputeHearing(in, s);
	out.olfaction = ComputeOlfaction(in, s, out.antennae);
	
	return out;
}


}
