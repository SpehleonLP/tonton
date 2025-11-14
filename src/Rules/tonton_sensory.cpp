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

    // ========== CLADE-SPECIFIC REFINEMENTS ==========
    using CF = CladeFlags;
    auto clade = s.physical.clade;

    // ARTHROPODA: Compound eyes with fundamentally different optics
    if (HasFlag(clade, CF::ARTHROPODA)) {
        // Compound eyes (Land & Nilsson 2012)
        // Resolution limited by ommatidial count and inter-ommatidial angle
        // Dragonflies: ~30,000 ommatidia per eye
        // Ants: ~100 ommatidia
        // Acuity ∝ sqrt(ommatidium_count)

        // Estimate ommatidium count from eye surface area
        // Assuming ~20 μm diameter ommatidia (typical for insects)
        float eye_area_m2 = 3.14159f * max_eye_diameter * max_eye_diameter;
        float ommatidium_diameter_m = 0.00002f; // 20 microns
        float ommatidium_area = 3.14159f * ommatidium_diameter_m * ommatidium_diameter_m;
        float ommatidium_count = eye_area_m2 / ommatidium_area;

        // Resolution scales with sqrt(N) (Land 1997)
        float compound_acuity = std::sqrt(ommatidium_count / 10000.0f);
        vision.acuity = std::clamp(compound_acuity, 0.1f, 0.7f);

        // Compound eyes excel at motion detection but poor at detail
        // Warrant & McIntyre (1993): ~100x better temporal resolution than vertebrates
        vision.motion_sensitivity_bonus = 2.0f; // Could add this field to Output_Vision

        // Many arthropods have UV vision (300-400nm)
        // Beneficial for flower navigation, mate selection
        vision.has_color_vision = true; // Usually trichromatic or tetrachromatic
        vision.has_uv_vision = true; // Most insects (Briscoe & Chittka 2001)

        // Smaller insects have wider field of view (nearly 360°)
        if (s.physical.body_mass_kg < 0.001f) {
            vision.binocular_overlap *= 0.5f; // Trade stereo for panoramic view
        }
    }

    // REPTILIA (Squamata): Pit organs for thermal vision
    if (HasFlag(clade, CF::REPTILIA)) {
        // Check for snake characteristics (no legs, long body)
        bool is_serpentine = !s.terrestrial.has_value() ||
                            (s.terrestrial.has_value() && s.terrestrial->legs.empty());

        if (is_serpentine) {
            // Crotalinae (pit vipers) & Boidae (pythons/boas) have infrared vision
            // Grace et al. (2001): Detection of 0.003°C temperature differences
            // Effective range: 1m for small prey at 30°C ambient

            // Add thermal vision capability
            vision.has_thermal_vision = true;
            vision.thermal_detection_range_m = s.physical.body_length_m * 1.5f;
            vision.thermal_sensitivity_K = 0.01f; // 0.01°C threshold

            // Snakes often have reduced visual acuity (except diurnal species)
            if (in.behavior.activity_pattern < 0.5f) { // Nocturnal
                vision.acuity *= 0.7f; // Reduced visual acuity
            }
        }
    }

    // AVES: Tetrachromatic color vision and exceptional acuity
    if (HasFlag(clade, CF::AVES)) {
        // Birds have 4 color receptors vs 3 in mammals
        // Cuthill et al. (2000): UV, blue, green, red cones
        vision.has_color_vision = true;
        vision.has_uv_vision = true; // Most birds (Bennett & Cuthill 1994)

        // Raptors have exceptional acuity (2-3x human)
        // Reymond (1985): Eagles resolve 140 cycles/degree vs 50 for humans
        if (s.aerial.has_value() && s.physical.body_mass_kg > 0.5f) {
            vision.acuity = std::min(vision.acuity * 1.5f, 1.0f);
            vision.detection_range_m *= 2.0f; // Raptors spot prey from km away
        }

        // Forward-facing eyes in raptors for binocular vision
        // Martin (2007): Owls have 50-70% binocular overlap
        if (s.aerial.has_value() && HasFlag(s.physical.clade, CF::MAMMALIA) == false) {
            vision.binocular_overlap = std::max(vision.binocular_overlap, 0.5f);
        }
    }

    // CHIROPTERA (Bats): Echolocation reduces reliance on vision
    // Note: Would need to check for bat-specific traits since CHIROPTERA isn't in CladeFlags yet
    // Handled in hearing refinements below

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

    // ========== CLADE-SPECIFIC REFINEMENTS ==========
    using CF = CladeFlags;
    auto clade = s.physical.clade;

    // CHIROPTERA (Bats): Echolocation for navigation and hunting
    // Note: CHIROPTERA not in current CladeFlags, use MAMMALIA + flight as proxy
    bool is_bat = HasFlag(clade, CF::MAMMALIA) && s.aerial.has_value() &&
                  s.aerial->wings.size() >= 2 && !HasFlag(clade, CF::AVES);

    if (is_bat) {
        // Neuweiler (2000): Bats use 20-200 kHz ultrasonic calls
        // FM bats (gleaning): 20-80 kHz, CF bats (hawking): 50-120 kHz
        hearing.frequency_range_Hz_min = 1000.0f; // Can hear low frequencies too
        hearing.frequency_range_Hz_max = 120000.0f; // Ultrasonic

        // Echolocation provides exceptional sensitivity and resolution
        // Simmons et al. (1974): 0.4 mm range discrimination
        hearing.sensitivity = 1.0f; // Maximum
        hearing.has_echolocation = true;
        hearing.echolocation_range_m = s.physical.body_length_m * 50.0f; // ~5-10m typical
        hearing.echolocation_range_m = std::clamp(hearing.echolocation_range_m, 5.0f, 20.0f);

        // Detection range for echolocation is excellent
        hearing.detection_range_m = hearing.echolocation_range_m;
    }

    // CETACEA: Underwater echolocation (dolphins, toothed whales)
    if (HasFlag(clade, CF::CETACEA)) {
        // Au (1993): Dolphins use 20-150 kHz clicks
        // Sperm whales: 2-30 kHz (lower frequencies penetrate deeper)
        hearing.frequency_range_Hz_min = 100.0f;
        hearing.frequency_range_Hz_max = 150000.0f; // Dolphins up to 150 kHz

        // Echolocation in water
        hearing.has_echolocation = true;
        hearing.sensitivity = 1.0f;

        // Water acoustics: sound travels 4.3x faster than air
        // Detection range much greater in water
        // Sperm whales can detect giant squid from 500m+ (Madsen et al. 2005)
        float echolocation_range = s.physical.body_length_m * 100.0f;
        hearing.echolocation_range_m = std::clamp(echolocation_range, 50.0f, 1000.0f);
        hearing.detection_range_m = hearing.echolocation_range_m;

        // Cetaceans have directional hearing (phased array)
        hearing.directional_accuracy_deg = 1.0f; // ±1° resolution
    }

    // ARTHROPODA: Tympanal organs and Johnston's organ
    if (HasFlag(clade, CF::ARTHROPODA)) {
        // Insects have diverse hearing mechanisms
        // Moths: tympanal organs detect 20-100 kHz (bat avoidance)
        // Crickets: tympanal hearing 2-50 kHz (mating calls)
        // Mosquitoes: Johnston's organ in antennae 200-800 Hz

        if (s.physical.body_mass_kg < 0.001f) {
            // Small insects: limited by size (wavelength >> body)
            // Best tuned to frequencies matching body size
            hearing.frequency_range_Hz_min = 500.0f;
            hearing.frequency_range_Hz_max = 20000.0f;

            // Sensitive to vibrations more than airborne sound
            hearing.substrate_vibration_sensitivity = 0.9f; // Could add to struct
        } else {
            // Larger arthropods can detect lower frequencies
            hearing.frequency_range_Hz_min = 100.0f;
            hearing.frequency_range_Hz_max = 50000.0f;
        }

        // Insects have poor sound localization (small baseline)
        // Michelsen (1998): 15-30° accuracy typical
        hearing.directional_accuracy_deg = 20.0f;

        // Many moths can detect ultrasonic bat calls
        // Roeder (1967): Detection at 30-40m range
        if (s.aerial.has_value() && HasFlag(clade, CF::INSECTA)) {
            hearing.frequency_range_Hz_max = 100000.0f; // Ultrasonic detection
            hearing.detection_range_m = std::max(hearing.detection_range_m, 30.0f);
        }
    }

    // AVES: Excellent hearing but limited frequency range
    if (HasFlag(clade, CF::AVES)) {
        // Birds hear 100 Hz - 10 kHz (narrower than mammals)
        // Dooling (2002): Most sensitive 1-5 kHz
        hearing.frequency_range_Hz_min = 100.0f;
        hearing.frequency_range_Hz_max = 10000.0f;

        // Owls have exceptional hearing for nocturnal hunting
        // Konishi (1973): Barn owls localize to ±1° in azimuth
        if (s.aerial.has_value() && in.behavior.activity_pattern < 0.4f) { // Nocturnal
            hearing.sensitivity = std::min(hearing.sensitivity * 1.5f, 1.0f);
            hearing.directional_accuracy_deg = 1.0f;

            // Facial disc acts as parabolic reflector
            hearing.detection_range_m *= 2.0f;
        }

        // Some cave-dwelling birds use echolocation (oilbirds, swiftlets)
        // Snow (1961): Simple click-based echolocation, not as refined as bats
        // Would need cave/dark habitat detection to enable this
    }

    // MAMMALIA: Generally excellent hearing range
    if (HasFlag(clade, CF::MAMMALIA) && !is_bat) {
        // Mammals have broad frequency range
        // Heffner & Heffner (2007): Most mammals 60 Hz - 60 kHz

        // Large mammals hear lower frequencies
        // Elephants can hear infrasound <20 Hz (Payne et al. 1986)
        if (body_mass_kg > 1000.0f) {
            hearing.frequency_range_Hz_min = 10.0f; // Infrasound
        }

        // Small mammals hear higher frequencies
        // Mice: up to 100 kHz (Heffner & Masterton 1980)
        if (body_mass_kg < 0.1f) {
            hearing.frequency_range_Hz_max = 90000.0f;
        }

        // Predatory mammals have better directional hearing
        // Heffner & Heffner (1992): Cats ±5°, humans ±10-15°
        if (in.behavior.aggression_adjustment > 0.6f) {
            hearing.directional_accuracy_deg = 5.0f;
        }
    }

    // AMPHIBIA: Hearing optimized for mate calls
    if (HasFlag(clade, CF::AMPHIBIA)) {
        // Frogs hear narrow range centered on their call frequency
        // Narins & Capranica (1980): Most sensitive 1-3 kHz
        hearing.frequency_range_Hz_min = 500.0f;
        hearing.frequency_range_Hz_max = 5000.0f;

        // High social tendency = vocal communication
        if (in.behavior.social_tendency > 0.5f) {
            hearing.sensitivity = std::min(hearing.sensitivity * 1.3f, 1.0f);
        }

        // Many frogs also detect substrate vibrations (seismic sensitivity)
        hearing.substrate_vibration_sensitivity = 0.8f;
    }

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

    // ========== CLADE-SPECIFIC REFINEMENTS ==========
    using CF = CladeFlags;
    auto clade = s.physical.clade;

    // ARTHROPODA: Antenna-based chemoreception (extremely sensitive)
    if (HasFlag(clade, CF::ARTHROPODA) && has_sensory_antennae) {
        // Insects have exceptionally sensitive olfaction
        // Cardé & Willis (2008): Moths detect pheromones at 1 molecule per 10^17
        // Ants follow pheromone trails at ppb concentrations

        // Sensilla density on antennae
        // Honeybees: ~3000 sensilla per antenna (Esslen & Kaissling 1976)
        // Moths: up to 60,000 sensilla on male antennae (pheromone detection)

        olfaction.sensitivity = std::min(olfaction.sensitivity * 1.5f, 1.0f);

        // Social insects use pheromones for communication
        if (in.behavior.social_tendency > 0.7f) {
            olfaction.sensitivity = 1.0f; // Maximum (ants, bees, termites)
            olfaction.detection_range_m *= 2.0f; // Trail-following ability

            // Ants can discriminate colony-specific odors
            // Vander Meer & Morel (1998): Hydrocarbon profiles
            olfaction.odor_discrimination_count = 50; // Can distinguish many odors
        }

        // Male moths searching for females (pheromone plumes)
        // Cardé & Willis (2008): Detection at km distances downwind
        if (!has_snout && olfaction.sensitivity > 0.8f) {
            olfaction.detection_range_m = std::max(olfaction.detection_range_m, 1000.0f);
        }

        // Arthropods have poor directionality (need to zigzag in plume)
        olfaction.directional_acuity = 0.3f; // Requires bilateral comparison
    }

    // MAMMALIA: Highly variable olfaction
    if (HasFlag(clade, CF::MAMMALIA)) {
        // Mammals have wide range of olfactory ability
        // Dogs: ~220 million olfactory receptors
        // Humans: ~5-6 million olfactory receptors
        // Whales/dolphins: reduced or absent olfaction

        // Cetaceans have reduced olfaction (favor echolocation)
        if (HasFlag(clade, CF::CETACEA)) {
            olfaction.sensitivity *= 0.2f; // Vestigial
            olfaction.detection_range_m *= 0.2f;
        } else {
            // Predatory mammals have excellent olfaction
            // Polar bears can smell seals through 1m of ice at 30km (Stirling 1974)
            if (in.behavior.aggression_adjustment > 0.6f) {
                olfaction.sensitivity = std::min(olfaction.sensitivity * 1.4f, 1.0f);
                olfaction.detection_range_m *= 3.0f;

                // Large predators track prey over long distances
                if (s.physical.body_mass_kg > 50.0f) {
                    olfaction.detection_range_m = std::max(olfaction.detection_range_m, 10000.0f);
                }
            }

            // Ungulates have good olfaction for predator detection
            if (HasFlag(clade, CF::EQUIDAE)) {
                olfaction.sensitivity = std::min(olfaction.sensitivity * 1.2f, 1.0f);
            }

            // Nocturnal mammals rely more on smell
            if (in.behavior.activity_pattern < 0.4f) {
                olfaction.sensitivity = std::min(olfaction.sensitivity * 1.3f, 1.0f);
            }
        }

        // Mammals have good directional olfaction (stereo sniffing)
        // Rajan et al. (2006): Rats localize odors to ±20°
        olfaction.directional_acuity = 0.7f;
    }

    // AVES: Generally poor olfaction (except specialists)
    if (HasFlag(clade, CF::AVES)) {
        // Most birds have reduced olfactory bulbs
        // Bang & Cobb (1968): Olfactory bulb 0.1-0.8% of brain mass

        // Exception: Vultures, albatrosses, kiwis have good olfaction
        // Nevitt (2008): Procellariiform seabirds track dimethyl sulfide

        if (s.aerial.has_value()) {
            // Soaring birds may use olfaction to find carrion
            if (s.aerial->flapping_efficiency < 0.3f && in.behavior.aggression_adjustment > 0.4f) {
                // Vultures detect ethyl mercaptan from carcasses
                olfaction.sensitivity = 0.7f; // Better than most birds
                olfaction.detection_range_m = 1000.0f; // Downwind detection
            } else {
                // Most flying birds have poor olfaction
                olfaction.sensitivity *= 0.5f;
                olfaction.detection_range_m *= 0.5f;
            }
        } else if (in.behavior.activity_pattern < 0.3f) {
            // Nocturnal flightless birds (kiwi) have excellent olfaction
            // Corfield et al. (2015): Kiwi olfactory bulb 30% of brain
            olfaction.sensitivity = 0.9f;
            olfaction.detection_range_m *= 2.0f;
        } else {
            // Default birds: poor olfaction
            olfaction.sensitivity *= 0.5f;
            olfaction.detection_range_m *= 0.5f;
        }

        // Birds have poor directional olfaction
        olfaction.directional_acuity = 0.2f;
    }

    // REPTILIA: Good olfaction, especially snakes
    if (HasFlag(clade, CF::REPTILIA)) {
        // Lizards and snakes use vomeronasal organ (Jacobson's organ)
        // Schwenk (1995): Tongue-flicking delivers scent to vomeronasal

        bool is_serpentine = !s.terrestrial.has_value() ||
                            (s.terrestrial.has_value() && s.terrestrial->legs.empty());

        if (is_serpentine) {
            // Snakes have exceptional chemoreception
            // Mason & Parker (2010): Rattlesnakes track prey trails days old
            olfaction.sensitivity = std::min(olfaction.sensitivity * 1.5f, 1.0f);
            olfaction.detection_range_m *= 2.0f;

            // Snakes use tongue to sample air (stereo chemoreception)
            // Schwenk (1994): Forked tongue provides directional information
            olfaction.directional_acuity = 0.8f; // Excellent (tongue sampling)
        } else {
            // Lizards also have good olfaction but less specialized
            olfaction.sensitivity = std::min(olfaction.sensitivity * 1.2f, 1.0f);
            olfaction.directional_acuity = 0.6f;
        }
    }

    // AMPHIBIA: Moderate olfaction, aquatic chemoreception
    if (HasFlag(clade, CF::AMPHIBIA)) {
        // Amphibians use olfaction for mate finding and homing
        // Kiseleva (1996): Salamanders return to breeding ponds using olfaction

        // Aquatic/semi-aquatic amphibians detect waterborne chemicals
        if (s.aquatic.has_value()) {
            olfaction.sensitivity = std::min(olfaction.sensitivity * 1.2f, 1.0f);
            // Chemical diffusion slower in water but more persistent
            olfaction.detection_range_m *= 0.7f; // Shorter but reliable
        }

        // Terrestrial phase: airborne olfaction
        olfaction.directional_acuity = 0.5f;
    }

    // PISCES: Aquatic chemoreception (highly variable)
    if (HasFlag(clade, CF::PISCES)) {
        // Fish have excellent underwater olfaction
        // Hara (1975): Salmon detect home stream at 1 ppb (parts per billion)

        // Predatory fish track blood in water
        // Sharks can detect 1 part per 10 billion (Gardiner & Atema 2010)
        if (in.behavior.aggression_adjustment > 0.6f) {
            olfaction.sensitivity = 1.0f; // Maximum (sharks)
            olfaction.detection_range_m = std::max(olfaction.detection_range_m, 500.0f);
        } else {
            olfaction.sensitivity = std::min(olfaction.sensitivity * 1.3f, 1.0f);
        }

        // Fish have good directional olfaction (bilateral nares)
        // Atema (1971): Fish localize odor sources via nare comparison
        olfaction.directional_acuity = 0.7f;

        // Migratory fish use olfaction for homing
        // Scholz et al. (1976): Salmon imprint on natal stream odor
        if (s.physical.body_mass_kg > 1.0f && s.aquatic.has_value()) {
            olfaction.detection_range_m *= 2.0f; // Long-distance tracking
        }
    }

    // CEPHALOPODA: Chemoreception without dedicated olfactory organs
    if (HasFlag(clade, CF::CEPHALOPODA)) {
        // Octopuses taste with suckers, detect chemicals via skin
        // Graziadei & Gagne (1976): Suckers contain chemoreceptors

        olfaction.sensitivity = 0.6f; // Moderate (taste + chemoreception blurred)
        olfaction.detection_range_m *= 0.5f; // Close-range detection
        olfaction.directional_acuity = 0.4f; // Multi-arm sampling
    }

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
