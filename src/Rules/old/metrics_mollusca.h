#ifndef METRICS_MOLLUSCA_H
#define METRICS_MOLLUSCA_H
#include "metrics_inputsbase.h"
#include <glm/vec3.hpp>
#include <vector>

/* Tonton: Trait Oriented Normalization for Traits of Organismal Nature
 * 
 * MOLLUSCA MODULE
 * Covers non-cephalopod mollusks with emphasis on performance prediction
 * rather than detailed animation. Focus: speeds, forces, energetics.
 */

namespace Mollusca
{

enum MolluskClass
{
    Cephalopoda,      // Covered by separate Cephalopod BSDF
    Gastropoda,       // Snails, slugs, sea butterflies
    Bivalvia,         // Clams, mussels, scallops
    Polyplacophora,   // Chitons (mostly sessile, low priority)
    Scaphopoda        // Tusk shells (burrowing, low priority)
};

enum LocomotionMode
{
    PedalWaveCrawling,     // Gastropod mucus-based crawling
    JetEscape,             // Scallop clap-and-fling
    PteropodFlight,        // Sea butterfly wing swimming
    Sessile,               // Attached/stationary (oysters, mussels)
    Burrowing              // Digging into substrate
};

struct FluidEnvironment {
    float density_Kg_m3{1000};        // kg/m³ (water=1000, air=1.2)
    float viscosity_Pas{0.00096};     // Pa·s (affects Reynolds numbers)
    float gravity_m_s2{9.81};         // m/s²
    float temperature_K{298.15};      // K (affects mucus viscosity)
};

struct MolluskTonTonInputs : public Metrics::InputsBase {
    FluidEnvironment environment;
    
    // BEHAVIORAL FINE-TUNING (0-1 sliders)
    float activity_adjustment = 0.5;      // ±20% from morphology prediction
    float predation_urgency = 0.5;        // Escape vs normal locomotion
    
    // ENVIRONMENTAL CONTEXT
    float substrate_roughness = 0.5;      // 0=smooth glass, 1=coarse sandpaper
    float substrate_inclination_deg = 0;  // Degrees from horizontal
    bool in_current = false;              // Water flow present
};

struct MorphologicalAnalysis {
    static MorphologicalAnalysis AnalyzeMorphology(const MolluskTonTonInputs& inputs);
    
    // CORE GEOMETRIC MEASUREMENTS
    float body_length;                    // Maximum dimension (m)
    float body_volume;                    // Total volume (m³)
    float body_mass;                      // Estimated mass (kg, ρ≈1050-1100)
    float shell_mass_fraction;            // Shell mass / total mass (0-0.95)
    
    // GASTROPOD-SPECIFIC
    float foot_area;                      // Ventral foot surface (m²)
    float foot_aspect_ratio;              // Length / width
    glm::vec3 shell_centroid;             // Center of mass offset
    
    // BIVALVE-SPECIFIC
    float shell_height;                   // Dorso-ventral dimension (m)
    float shell_gape_area;                // Aperture area when open (m²)
    float hinge_length;                   // Hinge line length (m)
    float adductor_attachment_area;       // Muscle scar area (m²)
    
    // PTEROPOD-SPECIFIC
    float parapodial_area;                // Wing area (m²)
    float wing_aspect_ratio;              // Span² / area
    float shell_drag_coefficient;         // Estimated Cd
    
    // UNIVERSAL MEASUREMENTS
    float fineness_ratio;                 // Length / mean diameter
    float moment_of_inertia_normalized;   // I / (m·L²)
    float surface_area_to_volume;         // m⁻¹
    
    // CLASSIFICATION CONFIDENCE
    struct Prediction {
        float value;
        float confidence;
        float final(float manual_adjustment, float weight = 0.2) const;
    };
    
    Prediction locomotion_mode_score;     // Which mode dominates
    Prediction swimming_capability;       // 0=crawling only, 1=excellent swimmer
    Prediction escape_performance;        // Burst capability (scallops)
    
    // DETECTED FEATURES
    bool has_operculum;                   // Gastropod protective plate
    bool has_siphon;                      // Bivalve water intake structure
    bool has_byssal_threads;              // Mussel attachment fibers
    MolluskClass detected_class;
    LocomotionMode primary_locomotion;
    
    float overall_confidence;
};

struct LocomotionPerformance {
    // SPEEDS (body lengths per second or m/s where appropriate)
    float max_crawling_speed_BLs;        // Gastropod pedal wave speed
    float max_crawling_speed_ms;         // Absolute speed (m/s)
    float escape_velocity_ms;            // Scallop/pteropod burst speed
    float sustained_swimming_speed_ms;   // Pteropod cruise speed
    
    // FREQUENCIES
    float pedal_wave_frequency_Hz;       // Gastropod wave cycles/second
    float shell_clap_frequency_Hz;       // Scallop adductor beats/second
    float wing_beat_frequency_Hz;        // Pteropod parapodial beats/second
    
    // FORCES AND POWER
    float thrust_force_N;                // Propulsive force generated
    float adhesion_force_N;              // Gastropod substrate grip
    float muscle_power_output_W;         // Mechanical power available
    float adductor_stress_Pa;            // Scallop muscle stress (~1.06×10⁵)
    
    // ENERGETICS
    float metabolic_cost_J_per_m;        // Cost of transport
    float mucus_production_rate_kg_s;    // Gastropod secretion rate
    float anaerobic_capacity_J;          // Burst energy reserve (scallops)
    
    // KINEMATICS
    float pedal_wavelength_m;            // Gastropod wave spacing
    float wing_stroke_amplitude_rad;     // Pteropod wing angle
    float body_pitch_angle_rad;          // Pteropod hyper-pitching
    
    // PERFORMANCE ENVELOPE
    float max_sustainable_distance_m;    // Before exhaustion
    float turning_radius_BL;             // Minimum turn radius
    float max_inclination_deg;           // Maximum climbable angle
    float minimum_substrate_friction;    // μ_min for gastropod locomotion
};

struct MucusProperties {
    // Gastropod-specific secretion characteristics
    // Denny (1980, 1981), Denny & Gosline (1980)
    
    float water_content_fraction;        // 0.91-0.98 typical
    float glycoprotein_concentration;    // g/L (affects viscosity)
    float shear_modulus_Pa;              // Viscoelastic property
    float yield_stress_Pa;               // Minimum stress for flow
    float kinetic_friction_coeff;        // Against substrate
    float static_friction_coeff;         // Adhesion strength
    
    // Temperature-dependent behavior (Denny 1984)
    float viscosity_at_temperature_Pas;  // Function of T
    float adhesion_at_temperature_N_m2;  // Function of T
};

struct ScallopDynamics {
    // Bivalve jet escape mechanics
    // Cheng & DeMont (1996), Denny & Miller (2006)
    
    float shell_mass_kg;                 // Including both valves
    float adductor_mass_kg;              // Fast-twitch muscle mass
    float hinge_stiffness_N_m;           // Abductin elastic modulus
    float hinge_resilience;              // Energy recovery (0.3-0.75)
    
    // Jet parameters
    float jet_volume_m3;                 // Water expelled per clap
    float jet_velocity_ms;               // Exit velocity
    float jet_duration_s;                // Pulse time
    float aperture_ratio;                // Jet area / shell area
    
    // Hydrodynamic efficiency (Cheng & DeMont 1996)
    float froude_efficiency;             // 0.3-0.5 typical
    float added_mass_coefficient;        // ~10× valve mass
    float optimal_clap_angle_deg;        // 6-12° from horizontal
    
    // Power output (Denny & Miller 2006)
    float peak_power_W_kg;               // 185 W/kg muscle
    float cyclic_work_J_kg;              // 9.0 J/kg muscle
};

struct PteropodKinematics {
    // Sea butterfly wing mechanics
    // Murphy et al. (2016), Limacina helicina studies
    
    float wing_span_m;                   // Parapodial span
    float wing_chord_m;                  // Mean chord length
    float wing_area_m2;                  // Total wing area (both sides)
    
    // Stroke pattern (clap-and-fling mechanism)
    float beat_frequency_Hz;             // 4-10 Hz typical
    float stroke_amplitude_deg;          // Wing angle range
    float angle_of_attack_deg;           // 45-50° for lift generation
    float body_pitch_range_deg;          // ±60° hyper-pitching
    
    // Convergent evolution with insects (Murphy et al. 2016)
    float figure_eight_pattern_ratio;    // Wingtip trajectory aspect
    float clap_fling_efficiency;         // Lift enhancement
    
    // Performance
    float lift_coefficient;              // CL from wing geometry
    float drag_coefficient;              // CD including shell
    float swimming_reynolds_number;      // Re = 10-100 range
    float sinking_rate_ms;               // When not swimming (negative buoyancy)
};

struct EnvironmentalLimits {
    static EnvironmentalLimits Factory(
        const MolluskTonTonInputs& inputs,
        const MorphologicalAnalysis& analysis);
    
    // PHYSICAL CONSTRAINTS
    float min_substrate_friction;        // Below this, gastropod slides
    float max_viscosity_for_mucus_Pas;   // Above this, mucus doesn't work
    float max_flow_velocity_ms;          // Current limit before detachment
    float pressure_tolerance_Pa;         // Shell crush depth
    
    // THERMAL LIMITS
    float min_temperature_K;             // Mucus freezing / metabolic minimum
    float max_temperature_K;             // Protein denaturation / desiccation
    float optimal_temperature_K;         // Peak performance
    
    // ENERGETIC CONSTRAINTS  
    float max_continuous_distance_m;     // Before mucus depletion / exhaustion
    float recovery_time_s;               // Between escape responses
    float starvation_time_s;             // Energy reserve duration
};

struct AnimationParameters {
    // Simplified compared to fish - most mollusks have simple motions
    
    // GASTROPOD WAVES
    float wave_speed_ratio;              // Wave speed / body speed
    float wave_amplitude_m;              // Wave height
    float wave_count;                    // Simultaneous waves on foot
    bool use_direct_waves;               // vs retrograde waves
    
    // SCALLOP CLAPPING
    float valve_opening_angle_deg;       // Maximum gape
    float clap_duration_s;               // Adduction phase
    float refill_duration_s;             // Abduction phase
    
    // PTEROPOD WINGS
    float wing_stroke_period_s;          // 1/frequency
    float wing_upstroke_ratio;           // Fraction of cycle in upstroke
    float body_pitch_amplitude_deg;      // Rotation per stroke
    
    // GENERAL
    float rest_pose_stability;           // 0=active, 1=can remain still
    bool requires_continuous_motion;     // Pteropods sink if stopped
};

} // namespace Mollusca

#endif // METRICS_MOLLUSCA_H

/*
 * BIBLIOGRAPHY - MOLLUSK BIOMECHANICS
 * 
 * === GASTROPOD LOCOMOTION ===
 * 
 * Denny, M. W. (1980a). Locomotion: the cost of gastropod crawling. Science, 
 *   208(4449), 1288-1290.
 *   - Established energetic cost of pedal wave locomotion
 * 
 * Denny, M. W. (1980b). The role of gastropod pedal mucus in locomotion. 
 *   Nature, 285(5760), 160-161.
 *   - Demonstrated dual adhesive/lubricant function
 * 
 * Denny, M. W. (1981). A quantitative model for the adhesive locomotion of 
 *   the terrestrial slug, Ariolimax columbianus. Journal of Experimental 
 *   Biology, 91(1), 195-217.
 *   - Mathematical model of mucus-based crawling mechanics
 * 
 * Denny, M. W. (1984). Mechanical properties of pedal mucus and their 
 *   consequences for gastropod structure and performance. American Zoologist, 
 *   24(1), 23-36.
 *   - Comprehensive review of mucus rheology
 * 
 * Denny, M. W., & Gosline, J. M. (1980). The physical properties of the pedal 
 *   mucus of the terrestrial slug, Ariolimax columbianus. Journal of 
 *   Experimental Biology, 88(1), 375-393.
 *   - Measured shear modulus, yield stress, hysteresis
 * 
 * Lai, Y. C., Cameron, A., & Fang, Y. C. (2010). The advantage of mucus for 
 *   adhesive locomotion in gastropods. Journal of Experimental Biology, 
 *   213(Pt 19), 3398-3404.
 *   - Demonstrated hysteresis property essential for crawling
 * 
 * Chan, B., Balmforth, N. J., & Hosoi, A. E. (2005). Building a better snail: 
 *   lubrication and adhesive locomotion. Physics of Fluids, 17(11), 113101.
 *   - Theoretical model of mucus flow during pedal waves
 * 
 * Crozier, W. J., & Pilz, G. F. (1924). The locomotion of Limax. Journal of 
 *   General Physiology, 6(3), 711-740.
 *   - Early kinematic measurements of wave frequency and wavelength
 * 
 * Lissmann, H. W. (1945). The mechanism of locomotion in gastropod molluscs. 
 *   I. Kinematics. Journal of Experimental Biology, 21(3-4), 58-69.
 *   - Foundational work on direct vs retrograde waves
 * 
 * Jones, H. D. (1973). The mechanism of locomotion of Agriolimax reticulatus 
 *   (Mollusca: Gastropoda). Journal of Zoology, 171(4), 489-498.
 *   - Detailed analysis of pedal wave dynamics
 * 
 * Pembury Smith, M. Q. R., & Ruxton, G. D. (2021). How fast is a snail's pace? 
 *   The influences of size and substrate on gastropod speed of locomotion. 
 *   Journal of Zoology, 313(1), 57-64.
 *   - Demonstrated substrate texture effects on crawling speed
 * 
 * === SCALLOP JET PROPULSION ===
 * 
 * Cheng, J. Y., & DeMont, M. E. (1996). Jet-propelled swimming in scallops: 
 *   swimming mechanics and ontogenic scaling. Canadian Journal of Zoology, 
 *   74(9), 1734-1748.
 *   - Established Froude efficiency 0.3-0.5, optimal angle 6-12°
 * 
 * Denny, M., & Miller, L. (2006). Jet propulsion in the cold: mechanics of 
 *   swimming in the Antarctic scallop Adamussium colbecki. Journal of 
 *   Experimental Biology, 209(22), 4503-4514.
 *   - Demonstrated abductin resilience compensates for reduced muscle mass
 * 
 * Cheng, J. Y., Davison, I. G., & DeMont, M. E. (1996). Dynamics and energetics 
 *   of scallop locomotion. Journal of Experimental Biology, 199(9), 1931-1946.
 *   - Mathematical model integrating hinge mechanics, valve inertia, fluid forces
 *   - Added mass ~10× valve mass, peak power 185 W/kg muscle
 * 
 * Guderley, H., & Tremblay, I. (2013). Escape responses by jet propulsion in 
 *   scallops. Canadian Journal of Zoology, 91(6), 420-430.
 *   - Review of escape response variation across species and ontogeny
 * 
 * Ansell, A. D., Cattaneo-Vietti, R., & Chiantore, M. (1998). Swimming in the 
 *   Antarctic scallop Adamussium colbecki: analysis of in situ video recordings. 
 *   Antarctic Science, 10(4), 369-375.
 *   - Antarctic scallops swim 25-50% speed of temperate species
 * 
 * === PTEROPOD FLIGHT ===
 * 
 * Murphy, D. W., Webster, D. R., Kawaguchi, S., King, R., & Yen, J. (2011). 
 *   Deep-sea fishing illuminates the roles of hydrodynamic flow and bioluminescent 
 *   light in sea butterfly escape. Biology Letters, 7(6), 906-909.
 *   - Identified escape behaviors using figure-eight wing patterns
 * 
 * Murphy, D. W., Adhikari, D., Webster, D. R., & Yen, J. (2016). Underwater 
 *   flight by the planktonic sea butterfly. Journal of Experimental Biology, 
 *   219(4), 535-543.
 *   - Demonstrated clap-and-fling mechanism, 45-50° angle of attack
 *   - Wingbeat 4-10 Hz, hyper-pitching up to 60°, Re=10-100
 * 
 * Chang, Y. S., & Yen, J. (2012). Swimming in the intermediate Reynolds range: 
 *   kinematics of the pteropod Limacina helicina. Integrative and Comparative 
 *   Biology, 52(5), 597-615.
 *   - Temperate L. helicina: 10-50 mm/s, sub-Arctic: up to 120 mm/s
 * 
 * Fields, D. M., & Yen, J. (1997). The escape behavior of marine copepods in 
 *   response to a quantifiable fluid mechanical disturbance. Journal of 
 *   Plankton Research, 19(9), 1289-1304.
 *   - Context for pteropod predator-prey interactions
 * 
 * === GENERAL MOLLUSK BIOLOGY ===
 * 
 * Barr, L. (1926). The mucous glands of the foot in the Prosobranchia. 
 *   Quarterly Journal of Microscopical Science, 70(280), 647-682.
 *   - Anatomical basis for mucus production
 * 
 * Barr, L. (1927). Further notes on the mucous glands in the Prosobranchia 
 *   and Pulmonata. Quarterly Journal of Microscopical Science, 71(281), 1-31.
 *   - Dual mucus system: thick pedal + thin sole mucus
 * 
 * Alexander, R. M. (1992). Exploring Biomechanics: Animals in Motion. 
 *   Scientific American Library, New York.
 *   - General principles applicable to mollusk locomotion
 * 
 * === FLUID DYNAMICS ===
 * 
 * Vogel, S. (1994). Life in Moving Fluids: The Physical Biology of Flow 
 *   (2nd ed.). Princeton University Press.
 *   - Theoretical foundation for lift generation, Reynolds numbers
 * 
 * Weis-Fogh, T. (1973). Quick estimates of flight fitness in hovering animals, 
 *   including novel mechanisms for lift production. Journal of Experimental 
 *   Biology, 59(1), 169-230.
 *   - Clap-and-fling mechanism convergently evolved in pteropods
 * 
 * === METABOLIC SCALING ===
 * 
 * Love, R. M. (1970). The Chemical Biology of Fishes. Academic Press, London.
 *   - Tissue density values for marine organisms
 * 
 * === BIOMECHANICAL MODELING ===
 * 
 * Lauga, E., & Hosoi, A. E. (2006). Tuning gastropod locomotion: modeling 
 *   the influence of mucus rheology on the cost of crawling. Physics of Fluids, 
 *   18(11), 113102.
 *   - Optimization theory for pedal wave locomotion
 * 
 * Ewoldt, R. H., Clasen, C., Hosoi, A. E., & McKinley, G. H. (2007). 
 *   Rheological fingerprinting of gastropod pedal mucus and synthetic complex 
 *   fluids for biomimicking adhesive locomotion. Soft Matter, 3(5), 634-643.
 *   - Quantitative rheology for bio-inspired robotics
 */
