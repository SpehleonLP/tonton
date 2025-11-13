#ifndef METRICS_ARTHROPOD_H
#define METRICS_ARTHROPOD_H
#include "metrics_inputsbase.h"
#include <glm/vec3.hpp>
#include <vector>

/* Arthropod Behavioral Trait Analysis System
 * 
 * Physics-based behavior prediction from exoskeleton morphology
 * Research-validated scaling relationships for locomotion and cognition
 */

namespace Arthropod
{

enum AIArchetype {
    SOLITARY_AMBUSH_HUNTER,
    PACK_COORDINATOR,
    SOCIAL_FORAGER,
    TERRITORIAL_DEFENDER,
    OPPORTUNISTIC_SCAVENGER,
    AGGRESSIVE_COMPETITOR,
    EUSOCIAL_WORKER,
    NOMADIC_WANDERER
};

enum AnimationArchetype {
    TRIPOD_WALKER,
    WAVE_CRAWLER,
    CLIMBING_SPECIALIST,
    JUMPING_SPECIALIST,
    FLYING_INSECT,
    BURROWING_DIGGER
};

struct Environment {
    float oxygen_percent{20};           // % O₂ (affects respiratory limits)
    float density_Kg_m3{1.2};          // kg/m³ (air=1.2, affects flight)
    float viscosity_Pas{0.000018};     // Pa·s (affects drag at small scales)
    float gravity_m_s2{9.81};          // m/s² (affects locomotion energetics)
    float pressure_Pa{101325};         // Pa (atmospheric baseline)
    float temperature_K{298.15};       // K (affects metabolic rates via Q10)
};

struct BehavioralInputs {
    // COMBAT CAPABILITY (0-1 normalized, context-dependent)
    float biteForce;                   // Jaw/mandible effectiveness
    float weaponDamage;                // Claws/stingers/horns effectiveness
    
    // DEFENSIVE CAPABILITY (0-1 normalized)
    float exoskeletonThickness;        // Armor vs mobility trade-off
    
    // SOCIAL TRAITS (0-1 normalized)
    float socialTolerance;             // Conspecific tolerance
    float cooperationLevel;            // Task coordination capability
    float territorialness;             // Resource defense tendency
    float communicationComplexity;     // Chemical/tactile/visual signaling
    float groupCoordination;           // Follows/leads group behaviors
};

struct ArthropodInputs : public Metrics::InputsBase {
    Environment environment;
    BehavioralInputs behavior;
    
    // BEHAVIORAL ADJUSTMENTS (optional artist overrides)
    float activity_adjustment = 0.5f;      // ±20% from metabolic predictions
    float aggression_adjustment = 0.5f;    // ±20% from weapon morphology
    float exploration_adjustment = 0.5f;   // ±20% from sensory analysis
};

struct MorphologicalAnalysis {
    static MorphologicalAnalysis Analyze(const ArthropodInputs& inputs);
    
    // ========== CORE GEOMETRIC MEASUREMENTS ==========
    
    struct BasicMorphometrics {
        float bodyLength_m;                // Anterior-posterior axis
        float bodyMass_kg;                 // From volume × density
        float legSpanRatio;                // Leg reach / body width
        float surfaceAreaToVolume;         // m⁻¹ (affects heat/water loss)
        float centerOfMassHeight;          // Stability indicator
        float finessRatio;                 // Length / mean diameter
    };
    BasicMorphometrics morphometrics;
    
    // ========== HIGH-CONFIDENCE PHYSIOLOGY (R² > 0.90) ==========
    // Citation: Shik et al. (2010), Lighton et al. (1993)
    
    struct Physiology {
        // Metabolic scaling: RMR = 4.14M^0.66 (R² = 0.90)
        // Citation: Addo-Bediako et al. (2002); Chown et al. (2007)
        float restingMetabolicRate_W;      // Baseline energy demand
        
        // Flight metabolic rate: FMR = 35.08M^1.10 (R² = 0.95)
        // Citation: Niven & Scharlemann (2005)
        float maxMetabolicRate_W;          // Peak aerobic capacity
        
        // Cost of transport: scales M^-0.32
        // Citation: Full & Tu (1991); Herreid et al. (1981)
        float costOfTransport_J_per_kg_m;
        
        // Respiratory constraints: critical PO₂ = 5-10 kPa
        // Citation: Greenlee & Harrison (2004); Harrison et al. (2006)
        float criticalO2Pressure_Pa;
        
        // Temperature effects: Q10 = 2-3 typical
        // Citation: Angilletta et al. (2004); Dell et al. (2011)
        float thermalQ10Factor;
        float optimalTemperature_K;
        float thermalToleranceRange_K;
        
        // Circulatory limits
        // Citation: Greenlee et al. (2013)
        float hemolymphVolume_L;
        
        // Size-dependent respiratory limits
        // Citation: Kaiser et al. (2007); Verberk & Bilton (2011)
        float trachelealDiffusionLimit_mg_O2_per_s;
    };
    Physiology physiology;
    
    // ========== HIGH-CONFIDENCE STRUCTURAL MECHANICS (R² > 0.85) ==========
    // Citation: Vincent & Wegst (2004); Klocke & Schmitz (2011)
    
    struct Structure {
        // Cuticle properties: 1-20 GPa Young's modulus
        // Citation: Vincent & Wegst (2004)
        float exoskeletonYoungsModulus_Pa;
        float exoskeletonDensity_kg_m3;    // 1.0-1.3 g/cm³
        
        // Calcified structures: ~124 MPa fracture strength
        // Citation: Raabe et al. (2005)
        float fractureStrength_Pa;
        
        // Square-cube law constraints
        // Citation: Haldane (1926); Kram et al. (1997)
        float optimalThickness_m;
        float maxSupportableSize_m;        // Before structural failure
        float safetyFactor;                // 2-6 range typical
        float strengthToWeightRatio;       // Decreases as L^-1
        
        // Joint mechanics
        // Citation: Gorb et al. (2000)
        float jointFlexionLimit_rad;       // ~90-120° typical
        float mechanicalAdvantage;         // 0.1-0.3 for internal muscles
    };
    Structure structure;
    
    // ========== MEDIUM-CONFIDENCE LOCOMOTION (R² 0.70-0.85) ==========
    
    struct Locomotion {
        struct Terrestrial {
            // Speed predictions from leg geometry + metabolic limits
            // Citation: Full & Tu (1991); Herreid et al. (1981)
            float maxWalkingSpeed_m_s;
            float maxRunningSpeed_m_s;
            
            // Gait mechanics
            // Citation: Full et al. (1991); Ting et al. (1994)
            float strideLength_m;
            float stepFrequency_Hz;
            float dutyFactor;              // 0.5-0.9 range
            
            // Ground reaction forces: F ∝ M^0.73-1.0
            // Citation: Full et al. (1991)
            float maxVerticalForce_N;
            float maxLateralForce_N;       // Up to M^1.3 scaling
            
            // Gait transitions
            // Citation: Ting et al. (1994)
            float criticalFroudeNumber;    // NOTE: Limited applicability
            float stabilityMargin_m;
            
            // Climbing capability
            // Citation: Autumn et al. (2000); Federle et al. (2006)
            float maxClimbAngle_rad;
            float adhesionCapacity_N;      // If adhesive pads present
            
            // Jumping performance
            // Citation: Bennet-Clark & Lucey (1967); Burrows (2006)
            float jumpHeight_m;
            float jumpDistance_m;
        } terrestrial;
        
        struct Aquatic {
            // Drag coefficients: 0.059-1.22 range
            // Citation: Vogel (1994)
            float dragCoefficient;
            float maxSwimmingSpeed_m_s;
            float buoyancyControl_kg_m3;
            
            // Surface tension for water-walking
            // Citation: Hu et al. (2003)
            float surfaceTensionLimit_N;
        } aquatic;
        
        struct Flight {
            // Wing beat frequency: 295.5*Mb^0.45*S^-0.87
            // Citation: Ellington (1984); Dudley (2000)
            float wingBeatFrequency_Hz;
            
            // Aerodynamic coefficients
            // Citation: Ellington et al. (1996)
            float liftCoefficient;
            float thrustCoefficient;
            
            // Performance metrics
            // Citation: Dudley (2000); Ellington (1984)
            float powerLoading_W_per_N;
            float diskLoading_N_per_m2;
            float reynoldsNumber;
        } flight;
    };
    Locomotion locomotion;
    
    // ========== MEDIUM-CONFIDENCE NEURAL SYSTEMS (R² 0.60-0.80) ==========
    
    struct Neural {
        // Reaction time: scales L^0.5-1.0
        // Citation: Jindrich & Full (2002)
        float reactionTime_s;
        
        // Neural conduction: 0.5-10 m/s in arthropods
        // Citation: Burrows (1996)
        float conductionVelocity_m_per_s;
        
        // Processing capacity: Haller's rule L²
        // Citation: Chittka & Niven (2009)
        float processingCapacity_ops_per_s;
        
        // Sensory capabilities
        // Citation: Land & Nilsson (2002)
        float sensoryRange_m;
        float coordinationComplexity;
        float learningCapacity;
    };
    Neural neural;
    
    // ========== CAPABILITY-SPECIFIC ANALYSIS ==========
    
    struct CapabilityAnalysis {
        std::string name;                  // "leg", "mandible", "antenna"
        float functionalEfficiency;        // 0-1 performance rating
        float powerRequirement_W;
        float rangeOfMotion_rad;
        float maxForceOutput_N;
        glm::vec3 workspaceCenter;
        float workspaceVolume_m3;
        float adhesionCapacity_N;          // If applicable
    };
    std::vector<CapabilityAnalysis> capabilities;
    
    // ========== CONFIDENCE TRACKING ==========
    
    struct Confidence {
        float overallReliability;          // Weighted average R²
        float morphometricConfidence;      // Geometry-based predictions
        float allometricConfidence;        // Scaling law accuracy
        float environmentalSensitivity;    // Parameter variation range
    };
    Confidence confidence;
};

// ========== BEHAVIORAL PREDICTION OUTPUT ==========

struct BehavioralAnalysis {
    
    // ========== HIGH-CONFIDENCE PREDICTIONS (R² > 0.8) ==========
    
    struct HighConfidence {
        // TERRITORIAL (territory size ∝ M^1.37, R² = 0.71-0.88)
        // Citation: Schoener (1968); Davies & Houston (1984)
        float territorySize_m2;
        float territoryDefenseRadius_m;
        float contestWinProbability;       // From weapon + mass scaling
        
        // FORAGING (prey size ∝ M^0.8-1.2)
        // Citation: Cohen et al. (1993); Brose et al. (2006)
        float maxPreySize_kg;
        float foragingRadius_m;
        float biteForcePrediction_N;       // From jaw scaling M^0.67
        float handlingTimePerPrey_s;
        
        // LOCOMOTION (from CCD + biomechanics)
        // Citation: Full & Tu (1991)
        float preferredGaitSpeed_m_s;
        float maxSustainableSpeed_m_s;
        float climbingCapability_rad;
        float jumpingRange_m;
    } highConfidence;
    
    // ========== MEDIUM-CONFIDENCE PREDICTIONS (R² 0.4-0.8) ==========
    
    struct MediumConfidence {
        // COGNITIVE (brain size ∝ M^0.75, learning R² = 0.44-0.67)
        // Citation: Chittka & Niven (2009); Gronenberg & Couvillon (2010)
        float learningCapacity;            // Head volume ratio
        float memoryRetention_days;
        float behavioralFlexibility;
        float decisionSpeed_s;             // T = 2.3 × Brain^-0.45
        
        // DEFENSIVE (escape capability correlations)
        // Citation: Domenici & Blake (1997) - adapted for arthropods
        float escapeResponseTime_ms;       // 4-200ms range
        float maxEscapeSpeed_m_s;
        float escapeAcceleration_m_s2;
        float standfightThreshold;         // Armor vs escape trade-off
        
        // ENVIRONMENTAL (thermal tolerance CTmax = 45.2 - 0.008×elevation)
        // Citation: Deutsch et al. (2008); Sunday et al. (2011)
        float thermalToleranceMin_K;
        float thermalToleranceMax_K;
        float desiccationResistance;       // Cuticle correlation r = 0.82
        float oxygenDemandThreshold_Pa;
    } mediumConfidence;
    
    // ========== BEHAVIORAL TENDENCIES (Lower confidence, heuristic) ==========
    
    struct Tendencies {
        // ACTIVITY PATTERNS
        float dailyActivityLevel;          // Metabolic + thermal constraints
        float foragingTimeAllocation;      // Energy budget allocation
        float socialTolerance;             // Weapon inverse correlation
        float explorationPropensity;       // Brain size + sensory
        
        // RISK ASSESSMENT
        float predatorAvoidance;
        float aggressionLevel;
        float resourceGuarding;
        float groupingTendency;
        
        // HABITAT PREFERENCES
        float surfacePreference;           // From adhesion capability
        float verticalActivityRange;
        float substrateCompliance;
        float coverDependency;
    } tendencies;
    
    // ========== ACTIVITY BUDGET (Metabolically constrained) ==========
    
    struct ActivityBudget {
        float restingProportion;
        float foragingProportion;
        float locomotionProportion;
        float maintenanceProportion;
        float socialProportion;
        
        // ENERGETIC CONSTRAINTS
        float dailyEnergyBudget_J;
        float locomotionCostPerMeter_J;
        float foragingEnergyReturn_J;
        float maintenanceEnergyPerDay_J;
    } activityBudget;
    
    // ========== COMBAT EFFECTIVENESS ==========
    
    struct CombatAnalysis {
        float weaponReach_m;               // From leaf bone positions
        float weaponEffectiveness;         // Input × reach × mass scaling
        float defensivePosturing;
        float combatEndurance_s;
        float combatInitiationThreshold;
        float retreatThreshold;
        float threateningDisplayPower;
    } combat;
    
    // ========== KINEMATIC CAPABILITIES (From CCD solver) ==========
    
    struct KinematicBehaviors {
        float manipulationPrecision;       // Fine motor control
        float movementEconomy;             // Energy-optimal patterns
        float balanceRecovery;
        float obstacleManeuvering;
        float multiTaskingCapability;
        float adaptiveLocomotion;
        float precisePositioning;
    } kinematics;
    
    // ========== SOCIALITY ASSESSMENT ==========
    // NOTE: Limited predictive power from morphology alone
    // Citation: Bourke (2011); Nowak et al. (2010)
    
    struct SocialityAssessment {
        // RELIABLE ANTI-SOCIAL INDICATORS
        float solitaryConfidence;          // High weapons → likely solitary
        float aggressiveExclusivity;
        
        // WEAK SOCIAL POTENTIAL
        float groupTolerancePotential;
        float communicationCapability;
        float cooperativeWorkPotential;
        
        float socialityUncertainty;        // Typically 0.7-0.9
        std::string socialityNote;         // Honest uncertainty statement
    } sociality;
    
    // ========== PREDICTION CONFIDENCE ==========
    
    struct PredictionMetrics {
        float morphologyDependence;        // Pure geometry predictions
        float inputDependence;             // Requires behavioral inputs
        float kinematicDependence;         // CCD-derived predictions
        float scalingLawConfidence;        // Research equation applicability
        float individualVariationRange;    // Expected biological variation
        
        std::vector<std::string> missingWeaponGeometry;
        float geometricCompleteness;
        float tagCompleteness;
    } predictionMetrics;
    
    // ========== COMPONENT SELECTION ==========
    
    AIArchetype recommendedAI;
    AnimationArchetype recommendedAnimation;
    float simulationPriority;              // 0=background, 1=hero
};

// ========== ENVIRONMENTAL LIMITS ==========

struct EnvironmentalLimits {
    static EnvironmentalLimits Calculate(
        const ArthropodInputs& inputs,
        const MorphologicalAnalysis& analysis);
    
    // Respiratory constraints
    // Citation: Harrison et al. (2010); Verberk & Bilton (2011)
    float minO2Concentration_percent;
    float maxCO2Tolerance_percent;
    
    // Temperature limits
    // Citation: Deutsch et al. (2008)
    float criticalThermalMin_K;
    float criticalThermalMax_K;
    
    // Humidity constraints
    // Citation: Edney (1977)
    float minRelativeHumidity_percent;
    float maxDesiccationTime_hours;
    
    // Pressure tolerance
    // Citation: Chapman (1998)
    float maxAltitude_m;
    float minPressure_Pa;
    
    // Locomotion boundaries
    float minViableGravity_m_s2;
    float maxViableGravity_m_s2;
};

// ========== ANIMATION PARAMETERS ==========

struct AnimationParameters {
    
    // GAIT KINEMATICS
    // Citation: Full et al. (1991); Ting et al. (1994)
    float baseStepFrequency_Hz;
    float strideLength_m;
    float dutyFactor;
    float phaseRelationships[8];           // Inter-leg coordination
    
    // BODY DYNAMICS
    // Citation: Full & Koditschek (1999)
    float bodyHeightOscillation_m;
    float lateralUndulation_m;
    float pitchVariation_rad;
    float rollVariation_rad;
    
    // TRANSITION THRESHOLDS
    float walkToTrotSpeed_m_s;
    float trotToGallopSpeed_m_s;          // If applicable
    
    // SPECIALIZED BEHAVIORS
    // Citation: Burrows (2006) - jumping; Autumn et al. (2000) - climbing
    float jumpPreparationTime_s;
    float climbingGaitModification;
    float bumpyTerrainAdaptation;
};

}; // namespace Arthropod

#endif // METRICS_ARTHROPOD_H
