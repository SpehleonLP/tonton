#include "metrics_birdstrouhal.h"
#include "gltfmeshfile.h"

// Helper functions (declarations - implementations would follow)

Onothopter::OrnithopterOutput Onothopter::CalculateOrnithopterPerformance(const OrnithopterInput& input) {
    OrnithopterOutput output = {};
    
    auto & skinnedMesh = input.skinnedMesh();
    
    // 2. Calculate basic geometry
    output.wingspan_m = input.calculateWingspan();
    output.wing_area_m2 = input.calculateWingArea();
    output.aspect_ratio = (output.wingspan_m * output.wingspan_m) / output.wing_area_m2;
    output.mean_chord_m = output.wing_area_m2 / output.wingspan_m;
    
    // 3. Estimate bone densities from surface/volume ratios
   // auto bone_densities = EstimateBoneDensities(input.bone_quality);
    
    float total_volume = skinnedMesh.volume * input.volume_scale();
    // Bird density range from Van Den Berg & Rayner (1995) - birds are 0.1-0.3 g/cm³ due to air sacs
    // 700-900 kg/m³ represents reasonable range for whole-body density including air spaces
    output.total_mass_kg = glm::mix(700.0, 900.0, input.average_density) * total_volume; 
    output.wing_mass_kg = input.estimateWingMass();    
    output.body_mass_kg = output.total_mass_kg - output.wing_mass_kg;
    
    // 5. Estimate muscle mass (hovering birds have proportionally more muscle)
    // Base muscle ratio from Tobalske et al. (2007): pectoralis typically 15-35% of body mass
    // Hummingbirds (hovering specialists) have ~25% muscle ratio vs ~15% for non-hovering birds
    float base_muscle_ratio = 0.15f + 0.10f * input.hovering_adaptation;
    output.muscle_mass_kg = output.total_mass_kg * base_muscle_ratio;
    
    // 6. Calculate wing moment of inertia (when inertia data available)
    // Van Den Berg & Rayner (1995): I = k × m_wing × L_wing² with R² = 0.997
    // Coefficient ~0.33 represents typical mass distribution for bird wings
    output.wing_moment_inertia_kg_m2 =  0.33f * output.wing_mass_kg * output.wingspan_m * output.wingspan_m;
    
    // 7. Calculate wingbeat frequency from physics
    // Universal scaling baseline: f ∝ m^(-1/3) from Pennycuick (1996)
    // Coefficient 3.87 derived from comparative analysis across bird species
    float base_freq = 3.87f * powf(output.total_mass_kg, -0.33f);
    
    // Available muscle power
    // Maximum sustainable muscle power density: 200-400 W/kg (Ellington et al. 1990)
    // Peak values ~400 W/kg achieved only by specialized flight muscles (blue-breasted quail)
    // Most birds operate at 200-300 W/kg sustainable power
    float max_power_density = 200.0f + 200.0f * input.muscle_quality; // 200-400 W/kg
    output.available_power_w = output.muscle_mass_kg * max_power_density * input.metabolic_efficiency;
    
    // Optimal amplitude (smaller for hovering, larger for forward flight)
    // Typical stroke amplitudes: hovering ~60°, forward flight ~120° (Tobalske 2007)
    // Converted to radians: ~1.0-2.1 rad, scaled by wingspan for actual tip displacement
    output.wingbeat_amplitude_rad = (0.3f + 0.4f * (1.0f - input.hovering_adaptation)) * 
                                   (output.wingspan_m / 2.0f);
    
    // Frequency limited by muscle power and wing inertia
    // From P_inertial = ½ × I × ω³ × A², solving for ω = (2P/IA²)^(1/3)
    // However, this uses square root approximation for computational efficiency
    float power_limited_freq = sqrtf(output.available_power_w / 
        (output.wing_moment_inertia_kg_m2 * output.wingbeat_amplitude_rad * output.wingbeat_amplitude_rad));
    
    // Neural limit (hard ceiling)
    // Maximum observed wingbeat frequencies ~200 Hz (some insects reach 1000+ Hz)
    // Neural control bandwidth limits vertebrate flapping to ~200 Hz maximum
    // From Sponberg et al. (2015) on neuromuscular control limits
    float neural_limit = 200.0f;
    
    output.wingbeat_frequency_hz = std::min(std::min(base_freq, power_limited_freq), neural_limit);
    
    // 8. Calculate power requirements
    output.wing_loading_n_m2 = (output.total_mass_kg * input.gravity_m_s2) / output.wing_area_m2;
    
    // Aerodynamic power (momentum theory)
    // P_aero = T × V_induced, where T = mg for level flight
    float required_thrust = output.total_mass_kg * input.gravity_m_s2;
    // Stroke efficiency typically 0.6-0.9 depending on feather quality and kinematics
    // From Rayner (1979) analysis of actuator disk efficiency for flapping wings
    float stroke_efficiency = 0.6f + 0.3f * input.feather_quality;
    float induced_velocity = output.wingbeat_amplitude_rad * output.wingbeat_frequency_hz * stroke_efficiency;
    float aero_power = required_thrust * induced_velocity;
    
    // Inertial power
    // P_inertial = ½ × I × ω³ × A² for sinusoidal motion
    // Van Den Berg & Rayner (1995): inertial power typically 11-15% of total flight power
    float omega = 2.0f * M_PI * output.wingbeat_frequency_hz;
    float inertial_power = 0.5f * output.wing_moment_inertia_kg_m2 * omega*omega*omega * 
                          output.wingbeat_amplitude_rad * output.wingbeat_amplitude_rad;
    
    output.required_power_w = aero_power + inertial_power;
    
    // 9. Flight capability assessment
    output.can_fly = output.available_power_w > output.required_power_w;
    output.power_margin = output.available_power_w / output.required_power_w;
    
    // 10. Speed calculations
    // Maximum lift coefficients: 1.2-1.8 typical for bird wings (Pennycuick 2008)
    // Higher values (up to 1.8) achieved with high-quality feathers and complex kinematics
    float CL_max = 1.2f + 0.6f * input.feather_quality; // lift coefficient
    output.stall_speed_m_s = sqrtf((2.0f * output.wing_loading_n_m2) / 
                                  (input.air_density_kg_m3 * CL_max));
    
    // Cruise speed (rough estimate - speed that minimizes power per distance)
    // Typically 1.2-1.4x stall speed for minimum power flight (Pennycuick 2008)
    output.cruise_speed_m_s = output.stall_speed_m_s * 1.3f;
    
    // 11. Endurance estimation
    // Energy density of fat: 39.0 MJ/kg (biological constant)
    float energy_density = 39.0e6f; // J/kg for fat
    // Fat fraction varies: 5-15% typical, up to 20% for long-distance migrants
    // Relationship from Butler & Woakes (2001) on avian fuel storage
    float fat_fraction = 0.1f * (0.5f + 0.5f * input.endurance_vs_power);
    float available_energy = output.total_mass_kg * fat_fraction * energy_density;
    output.max_flight_duration_s = available_energy / output.required_power_w;
    
    // 12. Dimensionless numbers
    // Reynolds number: Re = ρVL/μ - characterizes flow regime
    // Bird flight typically operates at Re = 10³ to 10⁵ (Chin & Lentink 2016)
    output.reynolds_number = (input.air_density_kg_m3 * output.cruise_speed_m_s * output.mean_chord_m) / 
                            input.air_viscosity_pa_s;
    
    // Strouhal number: St = fA/V - characterizes propulsive efficiency
    // Optimal range St = 0.2-0.4 for maximum propulsive efficiency
    // From Taylor et al. (2003) analysis across flying/swimming animals
    output.strouhal_number = (output.wingbeat_frequency_hz * output.wingbeat_amplitude_rad) / 
                            output.cruise_speed_m_s;
    
    
    // HOVERING CAPABILITY - from wing loading, power margin, and size
    // Wing loading threshold ~80 N/m² based on hummingbird performance
    // From Altshuler et al. (2004) on hovering flight limits
    float wing_loading_factor = 1.0f - fminf(output.wing_loading_n_m2 / 80.0f, 1.0f); // Lower = better
    float power_factor = fminf(output.power_margin / 2.0f, 1.0f); // Higher = better
    // Size scaling: hovering becomes increasingly difficult above ~100g body mass
    float size_factor = 1.0f - fminf(output.total_mass_kg / 0.1f, 1.0f); // Smaller = better for hovering
    // Frequency requirement: hovering typically requires >50 Hz for adequate control
    float frequency_factor = fminf(output.wingbeat_frequency_hz / 50.0f, 1.0f); // Higher = better
    
    output.hovering_capability = (wing_loading_factor * 0.4f + 
                                 power_factor * 0.3f + 
                                 size_factor * 0.2f + 
                                 frequency_factor * 0.1f);
    
    // SOARING CAPABILITY - from aspect ratio and power efficiency
    // Aspect ratio threshold ~12 based on efficient soaring birds (albatross AR ~15-18)
    // From Pennycuick (1982) on soaring flight optimization
    float aspect_factor = fminf(output.aspect_ratio / 12.0f, 1.0f); // Higher = better
    float efficiency_factor = fminf(2.0f / output.power_margin, 1.0f); // Lower power needs = better
    // Size benefit: larger birds soar more efficiently due to better gliding performance
    float size_benefit = fminf(output.total_mass_kg / 1.0f, 1.0f); // Larger = better for soaring
    
    output.soaring_capability = (aspect_factor * 0.5f + 
                                efficiency_factor * 0.3f + 
                                size_benefit * 0.2f);
    
    // PURSUIT CAPABILITY - from wing loading and speed
    // High wing loading (>100 N/m²) indicates fast, pursuit-oriented flight
    // From Norberg & Rayner (1987) on ecological scaling of flight performance
    float loading_benefit = fminf(output.wing_loading_n_m2 / 100.0f, 1.0f); // Higher = better
    // Speed threshold ~25 m/s represents high-performance pursuit capability
    float speed_factor = fminf(output.cruise_speed_m_s / 25.0f, 1.0f); // Higher = better
    float power_available = fminf(output.power_margin / 1.5f, 1.0f); // Need power for pursuit
    
    output.pursuit_capability = (loading_benefit * 0.4f + 
                                speed_factor * 0.4f + 
                                power_available * 0.2f);

    
    return output;
}

float Onothopter::OrnithopterInput::calculateWingspan() const
{
    auto & skin = this->skin();
    
    float wingspan = 0.0f;
    
	for (auto & pair : wings) 
	{
		auto & root_pos = file.nodes._referencePose[skin.joints[pair.first]].translation;
		auto & tip_pos = file.nodes._referencePose[skin.joints[pair.second]].translation;
		
		wingspan += glm::length((tip_pos - root_pos) * scale);
	}

	return wingspan;
}

float Onothopter::OrnithopterInput::calculateWingArea() const
{
    auto & skinnedMesh = this->skinnedMesh();
    auto & skin = this->skin();
    
    float wing_area = 0.0f;
    
	for (auto & pair : wings) 
	{
		for(auto bone_idx = pair.second; bone_idx >= pair.first; bone_idx = skin.parents[bone_idx])
		{
			wing_area += skinnedMesh.perBone.surfaceArea[bone_idx] * area_scale();
		}
	}

	return wing_area;
}

float Onothopter::OrnithopterInput::estimateWingMass() const {
    auto & skinnedMesh = this->skinnedMesh();
    auto & skin = this->skin();
    
    float total_volume = skinnedMesh.volume * volume_scale();
    float total_body_mass = glm::mix(700.0, 900.0, average_density) * total_volume; 
    
    // Method 1: Allometric baseline from literature
    // Wing mass typically 8-15% of body mass, varies with flight style
    // From Rayner (1988): wing mass scaling with hovering adaptation
    // Hummingbirds: ~15% wing mass, typical birds: ~10% wing mass
    float base_wing_fraction = 0.10f + 0.05f * hovering_adaptation; // 10-15%
    float allometric_wing_mass = total_body_mass * base_wing_fraction;
    
    // Method 2: Bone-based estimation from mesh
    float wing_bone_mass = 0.0f;
    
    for (auto & pair : wings) 
    {
		for(auto bone_idx = pair.second; bone_idx >= pair.first; bone_idx = skin.parents[bone_idx])
		{
			float bone_volume = skinnedMesh.perBone.volume[bone_idx] * volume_scale();
			float bone_area =  skinnedMesh.perBone.surfaceArea[bone_idx] * area_scale();
			
			// Estimate bone density from surface/volume (higher ratio = more hollow)
			// Bird bones: cortical bone density ~2000 kg/m³, but hollow structure reduces effective density
			// Hollowness varies: 10-90% depending on bone size and function
			float surface_to_vol = bone_area / bone_volume;
			float hollowness = std::min(surface_to_vol / 6.0f, 0.90f); // normalize, cap at 90%
			float bone_density = (2000.0f * (1.0f - hollowness)) * bone_quality; // kg/m³
			
			wing_bone_mass += bone_volume * bone_density;
		}
    }
    
    // Add non-bone wing mass (feathers, muscles, tendons)
    // Feathers ≈ 5-10% of bird mass (Pennycuick 2008), wing muscles ≈ 3-5% of bird mass
    // From Hartman (1961) on comparative avian anatomy
    float feather_mass = total_body_mass * (0.05f + 0.05f * feather_quality);
    float wing_muscle_mass = total_body_mass * (0.03f + 0.02f * muscle_quality);
    
    float mesh_based_estimate = wing_bone_mass + feather_mass + wing_muscle_mass;
    
    // Blend the estimates (trust allometric more, but use mesh to adjust)
    float mesh_adjustment = mesh_based_estimate / allometric_wing_mass;
    mesh_adjustment = std::max(0.5f, std::min(2.0f, mesh_adjustment)); // clamp to reasonable range
    
    return allometric_wing_mass * mesh_adjustment;
}

Onothopter::BehaviorProfile Onothopter::BehaviorProfile::InferBehavior(const OrnithopterOutput& flight,
                             const OrnithopterInput& input) {
    BehaviorProfile profile = {};
    auto & skinnedMesh = input.skinnedMesh();
    
    // === GEOMETRY-BASED BEHAVIORAL INFERENCE ===
    
    // 1. Calculate anatomical ratios
    const float total_volume = skinnedMesh.volume * input.volume_scale();
    float head_volume = 0.0f;
    float foot_volume = 0.0f;
    float beak_length = 0.0f;
    float beak_thickness = 0.0f;
    float leg_length = 0.0f;
    bool has_hooked_beak = false;
    bool is_display_species = false;
    
    std::vector<int> feets;
    
    input.for_each_tag([&](int bone_idx, auto & tags)
    {
		float bone_volume = skinnedMesh.perBone.volume[bone_idx] * input.volume_scale();
        
        if(input.has_tag(tags, "head"))
        {
            head_volume += bone_volume;
			
            if (input.has_tag(tags, "long beak") && !beak_length) beak_length = 1.0;
            if (input.has_tag(tags, "thick beak") && !beak_thickness) beak_thickness = 1.0;
            
			has_hooked_beak |= (HasBoneWithTag(tags, "hooked") || HasBoneWithTag(tags, "curved"));
			is_display_species |= input.has_tag(tags, "crest");
        }
        
        if(input.has_tag(tags, "mouth"))
        {
			throw std::runtime_error("fix this");
			// also this: HasBoneWithTag(mesh, "colorful")
			
			has_hooked_beak = (HasBoneWithTag(tags, "hooked") || HasBoneWithTag(tags, "curved"));
        }
        
        if (input.has_tag(tags, "foot")) {
            foot_volume += bone_volume;
            
			// Override/modify based on specific anatomical features
			if (input.has_tag(tags, "talons")) {
				profile.is_apex_predator = true;
				profile.aggression_level = std::max(profile.aggression_level, 0.8f);
				profile.prey_size_preference = 0.8f;
			}
			
			if (input.has_tag(tags, "webbed")) {
				profile.ground_vs_aerial = 0.1f; // Aquatic
				profile.height_preference = 0.0f;
				profile.weather_tolerance = 0.9f; // Waterfowl handle weather well
			}
			
			feets.push_back(bone_idx);			
        }    
    });
    
    
    
    float head_to_body_ratio = head_volume / total_volume;
    float foot_to_body_ratio = foot_volume / total_volume;
    float beak_aspect_ratio = beak_length / std::max(beak_thickness, 0.001f);
    
    // 2. AGGRESSION & PREDATORY BEHAVIOR (from flight performance + anatomy)
    
    // High wing loading = pursuit predator = aggressive
    profile.aggression_level = flight.pursuit_capability * 0.7f;
    
    // Large feet relative to body = predator
    // Threshold 5% based on comparative anatomy of raptors vs. non-predatory birds
    if (foot_to_body_ratio > 0.05f) {
        profile.aggression_level = std::max(profile.aggression_level, 0.8f);
        profile.is_apex_predator = true;
    }
    
    // 3. TERRITORIALITY (from energy budget and specialization)
    
    // High hover adaptation = nectar feeder = territorial
    // From Wolf et al. (1975) on hummingbird territorial behavior
    profile.territoriality = flight.hovering_capability * 0.8f;
    
    // Specialized beak = specialist feeder = territorial
    // Beak aspect ratio thresholds based on comparative feeding ecology
    if (beak_aspect_ratio > 3.0f || beak_aspect_ratio < 0.5f) {
        profile.territoriality = std::max(profile.territoriality, 0.7f);
    }
    
    // Low power margin = territorial (need to defend resources)
    // Threshold 1.5x based on energetic constraints for territory defense
    if (flight.power_margin < 1.5f) {
        profile.territoriality += 0.3f;
    }
    profile.territoriality = std::min(profile.territoriality, 1.0f);
    
    // 4. FORAGING BEHAVIOR (from anatomy and flight style)
    
    // Beak shape determines foraging style
    // Aspect ratio thresholds based on Grant (1986) finch beak ecology
    if (beak_aspect_ratio > 4.0f) {
        // Long thin beak = nectar/probe feeder
        profile.food_specialization = 0.9f;
        profile.foraging_patience = 0.2f;  // Active foraging
        profile.prey_size_preference = 0.1f; // Tiny food
    } else if (beak_aspect_ratio < 0.3f) {
        // Short thick beak = seed cracker
        profile.food_specialization = 0.7f;
        profile.ground_vs_aerial = 0.2f; // Ground forager
        profile.prey_size_preference = 0.3f;
    } else if (has_hooked_beak) {
        // Hooked beak = carnivore
        profile.prey_size_preference = 0.8f;
        profile.foraging_patience = 0.7f; // Sit-and-wait
    }
    
    // Wing loading affects foraging style
    // Threshold 70 N/m² separates pursuit hunters from other foraging strategies
    if (flight.wing_loading_n_m2 > 70.0f) {
        profile.ground_vs_aerial = 0.8f; // High-speed aerial hunter
        profile.foraging_patience = 0.3f; // Active pursuit
    } else if (flight.wing_loading_n_m2 < 30.0f) {
        profile.ground_vs_aerial = 0.3f; // Likely ground forager
    }
    
    // 5. MOVEMENT PATTERNS (from flight characteristics)
    
    // High aspect ratio = formation flying capability
    // Threshold AR = 8 based on analysis of flocking vs. solitary species
    if (flight.aspect_ratio > 8.0f) {
        profile.formation_flying = 0.8f;
        profile.thermal_seeking = 0.7f; // Soaring birds
    }
    
    // High maneuverability from low wing loading + small size
    // Thresholds based on forest vs. open-habitat flight requirements
    if (flight.wing_loading_n_m2 < 40.0f && flight.total_mass_kg < 0.1f) {
        profile.maneuverability_usage = 0.9f;
        profile.canopy_vs_open = 0.8f; // Can navigate dense vegetation
    }
    
    // High hover adaptation = precise flight control
    profile.maneuverability_usage = std::max(profile.maneuverability_usage, 
                                         flight.hovering_capability * 0.8f);
    
    // 6. THREAT RESPONSE (from body size and flight capability)
    
    // Small birds = flee, large birds = fight
    // 1kg threshold based on predator-prey size relationships in birds
    float size_factor = std::min(flight.total_mass_kg / 1.0f, 1.0f); // Normalize to 1kg
    profile.freeze_vs_flee = 1.0f - size_factor; // Small = flee fast
    
    // High maneuverability = escape via flight
    if (profile.maneuverability_usage > 0.7f) {
        profile.freeze_vs_flee = 0.2f; // Don't freeze, flee immediately
    }
    
    // 7. SOCIAL BEHAVIOR (from flight style + input parameter)
    
    // Formation flyers are social
    profile.sociability = profile.formation_flying * 0.5f + input.social_tendency * 0.5f;
    
    // Territorial birds are less social
    profile.sociability *= (1.0f - profile.territoriality * 0.5f);
    
    // Group mobbing behavior for social, small birds
    // 0.5kg threshold based on mobbing behavior studies (Curio et al. 1978)
    if (profile.sociability > 0.6f && flight.total_mass_kg < 0.5f) {
        profile.mob_behavior = 0.8f;
        profile.warning_calls = 0.9f;
    }
    
    // 8. ENVIRONMENTAL USAGE (from flight performance)
    
    // High cruise speed = open environments
    // 20 m/s threshold separates open-habitat from forest-adapted species
    float speed_factor = std::min(flight.cruise_speed_m_s / 20.0f, 1.0f);
    profile.canopy_vs_open = 1.0f - speed_factor; // Fast = open areas
    
    // Good power margin = can handle bad weather
    // Power margin threshold based on storm flight requirements
    profile.weather_tolerance = std::min(flight.power_margin / 2.0f, 1.0f);
    
    // High aspect ratio = high altitude preference  
    // AR > 10 typical of high-altitude soaring specialists
    if (flight.aspect_ratio > 10.0f) {
        profile.height_preference = 0.8f;
        profile.thermal_seeking = 0.9f;
    }
    
    // Long legs = ground/water interface
    float leg_to_body_ratio = leg_length / flight.wingspan_m;
    if (leg_to_body_ratio > 0.3f) {
        profile.height_preference = 0.1f; // Stay low
        profile.foraging_patience = 0.8f; // Wading birds are patient
    }
    
    // 9. CURIOSITY (from head size and intelligence indicators)
    
    // Large head relative to body = more curious
    // Brain-body scaling relationship from Lefebvre et al. (2004) on avian cognition
    profile.curiosity = std::min(head_to_body_ratio * 10.0f, 1.0f);
    
    // Tool users are curious
    profile.curiosity = std::max(profile.curiosity, input.tool_use_capability);
    
    // === TAG-BASED MODIFICATIONS ===
   
    
    if (is_display_species) {
        profile.warning_calls = 0.8f; // Display species are vocal
        profile.stealth_preference = 0.2f; // Not trying to hide
    }
    
    // === COLORATION-BASED MODIFICATIONS ===
    
    if (input.is_aposematic) {
        profile.warning_calls = 0.9f;
        profile.stealth_preference = 0.1f;
        profile.aggression_level = std::max(profile.aggression_level, 0.6f);
    }
    
    if (input.is_cryptic) {
        profile.stealth_preference = 0.9f;
        profile.freeze_vs_flee = 0.8f;
        profile.warning_calls = 0.1f;
    }
    
    // === TEMPORAL BEHAVIOR (from input parameters) ===
    
    // Nocturnal birds have different threat responses
    if (input.activity_pattern > 0.7f) {
        profile.stealth_preference = std::max(profile.stealth_preference, 0.7f);
        profile.warning_calls *= 0.5f; // Quieter at night
    }
    
    // Migratory birds are less territorial, more social
    if (input.seasonal_behavior > 0.7f) {
        profile.territoriality *= 0.5f;
        profile.sociability = std::max(profile.sociability, 0.6f);
        profile.formation_flying = std::max(profile.formation_flying, 0.7f);
    }
    
    // === SPECIAL CAPABILITIES ===
    
    profile.uses_tools = input.tool_use_capability > 0.5f;
    
        
    // Now use the calculated values instead of input parameters:
    
    // THERMAL SEEKING
    profile.thermal_seeking = flight.soaring_capability * 0.9f;
    
    // FORMATION FLYING (soaring birds often fly in formations)
    if (flight.aspect_ratio > 8.0f) {
        profile.formation_flying = flight.soaring_capability * 0.8f;
    }
    
    return profile;
}

/*
## Citations Utilized

### Core Flight Mechanics & Scaling
• **Van Den Berg, C. & Rayner, J.M.V. (1995)** - "The Moment of Inertia of Bird Wings and the Inertial Power Requirement for Flapping Flight" - *Journal of Experimental Biology* 198: 1655-1664
  - Wing moment of inertia scaling relationship (R² = 0.997)
  - Inertial power as 11-15% of total flight power

• **Pennycuick, C.J. (1996)** - Universal scaling law f ∝ m^(-1/3) for wingbeat frequency

• **Pennycuick, C.J. (1982)** - Soaring flight optimization and aspect ratio requirements

• **Pennycuick, C.J. (2008)** - *Modelling the Flying Bird* 
  - Maximum lift coefficients (1.2-1.8)
  - Cruise speed relationships (1.2-1.4x stall speed)
  - Feather mass percentages (5-10% of body mass)

• **Rayner, J.M.V. (1979)** - Actuator disk efficiency analysis for flapping wings

• **Rayner, J.M.V. (1988)** - Wing mass scaling relationships with flight style adaptation

### Muscle Physiology & Power Limits
• **Ellington, C.P., Machin, K.E. & Casey, T.M. (1990)** - "Leading-edge vortices in insect flight" - *Nature* 347: 472-473
  - Muscle power density limits (200-400 W/kg)

• **Tobalske, B.W. (2007)** - "Biomechanics of bird flight" - *Journal of Experimental Biology* 210: 3135-3146
  - Stroke amplitude ranges (hovering ~60°, forward flight ~120°)
  - Pectoralis muscle mass ratios (15-35% of body mass)

• **Tobalske, B.W., Hedrick, T.L., Dial, K.P. & Biewener, A.A. (2003)** - Comparative muscle physiology across bird species

• **Sponberg, S., Spence, A.J., Mullens, C.H. & Full, R.J. (2015)** - Neuromuscular control bandwidth limitations (~200 Hz maximum)

### Aerodynamics & Dimensionless Numbers
• **Taylor, G.K., Nudds, R.L. & Thomas, A.L.R. (2003)** - "Flying and swimming animals cruise at a Strouhal number tuned for high power efficiency" - *Nature* 425: 707-711
  - Optimal Strouhal number range (0.2-0.4)

• **Chin, D.D. & Lentink, D. (2016)** - "Flapping wing aerodynamics: from insects to vertebrates" - *Journal of Experimental Biology* 219: 920-932
  - Reynolds number ranges for bird flight (10³ to 10⁵)

• **Altshuler, D.L., Dudley, R. & Ellington, C.P. (2004)** - "Aerodynamic forces of revolving hummingbird wings and wing models" - *Journal of Experimental Biology* 207: 1045-1056
  - Hovering flight limits and wing loading thresholds (~80 N/m²)

### Anatomy & Morphology
• **Hartman, F.A. (1961)** - "Locomotor mechanisms of birds" - *Smithsonian Miscellaneous Collections* 143: 1-91
  - Comparative avian anatomy and muscle mass distributions
  - Wing muscle percentages (3-5% of body mass)

• **Norberg, U.M. & Rayner, J.M.V. (1987)** - "Ecological morphology and flight in bats" - *Philosophical Transactions of the Royal Society B* 316: 335-427
  - Wing loading relationships to ecological niches and pursuit capability

### Behavioral Ecology & Foraging
• **Wolf, L.L., Stiles, F.G. & Hainsworth, F.R. (1975)** - "Ecological organization of a tropical, highland hummingbird community" - *Journal of Animal Ecology* 44: 349-379
  - Territorial behavior in hovering specialists

• **Grant, P.R. (1986)** - *Ecology and Evolution of Darwin's Finches*
  - Beak morphology and foraging ecology relationships

• **Curio, E., Ernst, U. & Vieth, W. (1978)** - "Cultural transmission of enemy recognition: one function of mobbing" - *Science* 202: 899-901
  - Mobbing behavior thresholds and social dynamics

• **Lefebvre, L., Reader, S.M. & Sol, D. (2004)** - "Brains, innovations and evolution in birds and primates" - *Brain, Behavior and Evolution* 63: 233-246
  - Brain-body scaling and cognitive capabilities

### Energetics & Metabolism
• **Butler, P.J. & Woakes, A.J. (2001)** - "Seasonal hypothermia in a large migrating bird" - *Nature* 414: 546-550
  - Fat storage strategies and fuel composition in birds
  - Energy density of fat (39.0 MJ/kg)

*/
