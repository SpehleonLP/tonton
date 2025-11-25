#include "tonton_sensory.h"
#include "../include/tonton_input.h"
#include "../include/tonton_analysis.h"
#include "Memos/tonton_armaturememo.h"
#include "Rules/tonton_scratch.h"
#include "tonton_builder.h"
#include "tonton_skinnedmesh.h"

namespace TonTon
{
static Analysis_Vision ComputeVision(Input const& in, Scratch const& s) {
    Analysis_Vision vision{};
    
    auto & eyes = in.builder->sensory.vision.eyes;
    
    vision.binocular_overlap = in.builder->sensory.vision.binocular_overlap;
    vision.centering = in.builder->sensory.vision.centering; // Single eye is "centered" by default
        
    if(eyes.empty()) {
        // No eyes = blind (return nullopt in calling code if you want)
        return vision;
    }
    
    // ===================================================================
    // ACUITY - based on eye size
    // ===================================================================
    
    length_m max_eye_diameter = 0.0f;
    for(auto const& eye : eyes) {
        max_eye_diameter = std::max(max_eye_diameter, eye.eye_diameter_m);
    }
    
    // Scale: 0.002m (ant) -> 0.05m (human) -> 0.1m (horse) -> 0.3m (giant squid)
    auto geometric_acuity = std::clamp(float(max_eye_diameter) / 0.05f, 0.0f, 1.0f);
    
    // Behavioral modifier: diurnal animals need better vision
    auto activity_bonus = in.behavior.activity_pattern;
    vision.acuity = geometric_acuity * glm::mix(0.7f, 1.3f, activity_bonus);
    vision.acuity = std::clamp(vision.acuity, 0.0f, 1.0f);
    
    // ===================================================================
    // COLOR & NIGHT VISION
    // ===================================================================
    
    vision.has_color_vision = (in.behavior.activity_pattern > 0.4f); // Diurnal
    vision.has_night_vision = (in.behavior.activity_pattern < 0.6f); // Nocturnal
    
    // ===================================================================
    // DETECTION RANGE
    // ===================================================================
    
    auto scale_factor = cbrt(s.physical.body_volume_m3);
    vision.detection_range_m = scale_factor * 100.0f * vision.acuity;
    
    // Eyestalks give better view distance (periscope effect)
    bool has_eyestalks = false;
    length_m max_stalk_height = 0.0f;
    for(auto const& eye : eyes) {
        if(eye.is_on_stalk) {
            has_eyestalks = true;
            max_stalk_height = std::max(max_stalk_height, eye.stalk_length_m);
        }
    }
    
    if(has_eyestalks) {
        // Higher eyes see further (horizon distance)
        auto horizon_bonus = 1.0f + std::sqrt(float(max_stalk_height) * 2.0f); // Rough approximation
        vision.detection_range_m *= horizon_bonus;
    }
    
    vision.detection_range_m = std::clamp<length_m>(vision.detection_range_m, 1.0f, 10000.0f);

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
        area_m2 eye_area_m2 = 3.14159f * max_eye_diameter * max_eye_diameter;
        length_m ommatidium_diameter_m = 0.00002f; // 20 microns
        area_m2 ommatidium_area = 3.14159f * ommatidium_diameter_m * ommatidium_diameter_m;
        auto ommatidium_count = eye_area_m2 / ommatidium_area;

        // Resolution scales with sqrt(N) (Land 1997)
        auto compound_acuity = std::sqrt(ommatidium_count / 10000.0f);
        vision.acuity = std::clamp(compound_acuity, 0.1f, 0.7f);

        // Compound eyes excel at motion detection but poor at detail
        // Warrant & McIntyre (1993): ~100x better temporal resolution than vertebrates
        vision.motion_sensitivity_bonus = 2.0f; // Could add this field to Analysis_Vision

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

static std::optional<Analysis_Hearing> ComputeHearing(Input const& in, Scratch const& s) {
    Analysis_Hearing hearing{};
    auto & physical = s.physical;
    
    // BASE VALUES from geometry
    auto ear_surface_area = scale_to<0>(in.builder->sensory.hearing.ear_surface_area, in.area_scale());
    bool has_external_ears = in.builder->sensory.hearing.has_external_ears;
    
    // SENSITIVITY: Blend geometry + social tendency + activity pattern
    auto geometric_sensitivity = 0.5f; // Default
    if(has_external_ears) {
        // Larger ears = better hearing (elephant ears = 2 m², mouse ears = 0.0001 m²)
        geometric_sensitivity = std::clamp(float(ear_surface_area) / 0.01f, 0.3f, 1.0f);
    }
    
    // Social animals need good hearing (communication)
    // Nocturnal animals need good hearing (can't see)
    auto social_bonus = in.behavior.social_tendency;
    auto nocturnal_bonus = 1.0f - in.behavior.activity_pattern;
    auto behavioral_sensitivity = std::max(social_bonus, nocturnal_bonus);
    
    hearing.sensitivity = (geometric_sensitivity + behavioral_sensitivity) * 0.5f;
    hearing.sensitivity = std::clamp(hearing.sensitivity, 0.0f, 1.0f);
    
    // FREQUENCY RANGE: Allometric scaling from body mass
    // Small animals hear high frequencies (mice: 1-100 kHz)
    // Large animals hear low frequencies (elephants: 16-12000 Hz)
    auto body_mass_kg = physical.body_mass_kg;
    
    hearing.frequency_range_Hz_min = 20.0f * std::pow(float(body_mass_kg), -0.15f);
    hearing.frequency_range_Hz_max = 20000.0f * std::pow(float(body_mass_kg), -0.25f);
    
    // Clamp to biological reality
    hearing.frequency_range_Hz_min = std::clamp<freq_Hz>(hearing.frequency_range_Hz_min, 10.0f, 100.0f);
    hearing.frequency_range_Hz_max = std::clamp<freq_Hz>(hearing.frequency_range_Hz_max, 1000.0f, 120000.0f);
    
    // DETECTION RANGE: Larger animals + better sensitivity = longer range
    auto size_factor = cbrt(physical.body_volume_m3); // Cube root for volume->length
    hearing.detection_range_m = size_factor * 50.0f * hearing.sensitivity;
    hearing.detection_range_m = std::clamp<length_m>(hearing.detection_range_m, 5.0f, 5000.0f);

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
        hearing.echolocation_range_m = std::clamp<length_m>(hearing.echolocation_range_m, 5.0f, 20.0f);

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
        auto echolocation_range = s.physical.body_length_m * 100.0f;
        hearing.echolocation_range_m = std::clamp<length_m>(echolocation_range, 50.0f, 1000.0f);
        hearing.detection_range_m = hearing.echolocation_range_m;

        // Cetaceans have directional hearing (phased array)
        hearing.directional_accuracy_rad = 1.0f * (M_PI / 180); // ±1° resolution
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
        hearing.directional_accuracy_rad = 20.0f * (M_PI / 180);

        // Many moths can detect ultrasonic bat calls
        // Roeder (1967): Detection at 30-40m range
        if (s.aerial.has_value() && HasFlag(clade, CF::INSECTA)) {
            hearing.frequency_range_Hz_max = 100000.0f; // Ultrasonic detection
            hearing.detection_range_m = std::max<length_m>(hearing.detection_range_m, 30.0f);
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
            hearing.directional_accuracy_rad = 1.0f * (M_PI / 180);

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
            hearing.directional_accuracy_rad = 5.0f * (M_PI / 180);
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
static std::optional<Analysis_Olfaction> ComputeOlfaction(
    Input const& in, 
    Scratch const& s)
{
    Analysis_Olfaction olfaction{};
    
    // BASE VALUES from geometry
    bool has_snout = in.builder->sensory.has_snout;
    area_m2 nasal_surface_area = scale_to<0>(in.builder->sensory.nasal_surface_area, in.area_scale());
    
    auto antennae = s.sensory.antennae;
    auto has_sensory_antennae = in.builder->sensory.antennae.is_sensory;
    auto antennal_surface_area = scale_to<0>(in.builder->sensory.antennae.surface_area, in.area_scale());
    
    // If no olfactory organs at all, return nullopt
    if(!has_snout && !has_sensory_antennae) {
        return std::nullopt;
    }
    
    // SENSITIVITY CALCULATION
    auto geometric_sensitivity = 0.0f;
    
    if(has_snout) {
        // VERTEBRATE OLFACTION
        // Dog snout: ~150 cm² epithelium, Human: ~5 cm²
        float surface_cm2 = float(nasal_surface_area) * 10000.0f;
        geometric_sensitivity = std::clamp(surface_cm2 / 150.0f, 0.2f, 1.0f);
    }
    
    if(has_sensory_antennae) {
        // ARTHROPOD CHEMORECEPTION
        // Antennae are extremely sensitive - moths can detect pheromones at ppb
        // Surface area is key: more sensilla = better detection
        
        auto antennal_cm2 = float(antennal_surface_area) * 10000.0f;
        
        // Insects have amazing olfaction despite small antennae
        // Scale differently than vertebrate noses
        // Ant antenna: ~0.01 cm², Moth antenna: ~1 cm²
        auto antennal_sensitivity = std::clamp(antennal_cm2 / 1.0f, 0.4f, 1.0f);
        
        // Antennae are often MORE sensitive than noses (for specific compounds)
        geometric_sensitivity = std::max(geometric_sensitivity, antennal_sensitivity);
    }
    
    // BEHAVIORAL MODIFIERS
    
    // Nocturnal animals rely more on smell
    auto nocturnal_bonus = 1.0f - in.behavior.activity_pattern;
    
    // Predators (high aggression) often have good smell for tracking prey
    auto predator_bonus = in.behavior.aggression_adjustment;
    
    // Social animals may use pheromones (especially with antennae)
    auto social_bonus = has_sensory_antennae ? in.behavior.social_tendency * 0.5f : 0.0f;
    
    auto behavioral_sensitivity = std::max({
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
        length_m max_antenna_length = 0.0f;
        for(auto const& antenna : antennae) {
            max_antenna_length = std::max(max_antenna_length, antenna.stretched_length_m);
        }
        
        // Longer antennae = better directionality and sampling volume
        auto length_factor = std::clamp(float(max_antenna_length) / 0.1f, 0.5f, 2.0f);
        
        olfaction.detection_range_m = glm::mix(5.0f, 1000.0f, olfaction.sensitivity) * length_factor;
        
    } else {
        // Vertebrate olfaction: moderate sensitivity, long range possible
        // Poor: 1m (most birds), Moderate: 50m (humans), Excellent: 20km (polar bears)
        olfaction.detection_range_m = glm::mix(1.0f, 5000.0f, olfaction.sensitivity);
        
        // Size scaling: larger animals can detect scents from further away
        auto size_bonus = std::pow(float(s.physical.body_mass_kg), 0.2f);
        olfaction.detection_range_m *= size_bonus;
    }
    
    olfaction.detection_range_m = std::clamp<length_m>(olfaction.detection_range_m, 0.5f, 20000.0f);

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
            olfaction.detection_range_m = std::max<length_m>(olfaction.detection_range_m, 1000.0f);
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
                    olfaction.detection_range_m = std::max<length_m>(olfaction.detection_range_m, 10000.0f);
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
            olfaction.detection_range_m = std::max<length_m>(olfaction.detection_range_m, 500.0f);
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

Analysis_Sensory<std::optional>  ComputeSensory(Input const& in, Scratch & s)
{
	Analysis_Sensory<std::optional> out;
	
//	std::array<Word, 1> words = {Word::antenna}; 
//	auto chains = GetChainsFromRoot(in, std::span<Word>{words});
	auto & chains = in.builder->sensory.antennae.chains;
	out.antennae = shared_array<Analysis_Chain>::Build(chains.size(), [&](int i ){
		Analysis_Chain r;
		chains[i].copy_into(r, in.scale);
		return r;
	});
	
	out.vision = ComputeVision(in, s);
	out.hearing = ComputeHearing(in, s);
	out.olfaction = ComputeOlfaction(in, s);
	
	return out;
}


}
