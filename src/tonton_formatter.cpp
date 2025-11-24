#include "../include/tonton_formatter.h"

namespace TonTon {

// Helper for optional values
template<typename T>
static void print_optional(std::ostream& os, const char* name, const std::optional<T>& opt) {
    if (opt) {
        os << "  " << name << ": " << *opt << "\n";
    }
}

// Helper for optional values
static void print_optional(std::ostream& os, const char* name, const float& opt) {
    if (opt > 0) {
        os << "  " << name << ": " << opt << "\n";
    }
}

// Physical
std::ostream& operator<<(std::ostream& os, const Analysis_Physical& p) {
    os << "Physical:\n"
       << "  body_mass_kg: " << p.body_mass_kg << "\n"
       << "  svl_m: " << p.svl_m() << "\n"
       << "  total_length_m: " << p.body_length_m  << "\n"
       << "  body_volume_m3: " << p.body_volume_m3 << "\n"
      // << "  center_of_mass: (" << p.center_of_mass.x << ", " << p.center_of_mass.y << ", " << p.center_of_mass.z << ")\n"
       << "  surface_area_m2: " << p.surface_area_m2 << "\n"
       << "  cross_sectional_area_m2: " << p.cross_sectional_area_m2 << "\n"
       << "  cross_sectional_diameter: " << std::sqrt(p.cross_sectional_area_m2 / M_PI)*2.0 << "\n"
       << "  fineness_ratio: " << p.fineness_ratio << "\n";
    return os;
}

// Metabolic
std::ostream& operator<<(std::ostream& os, const Analysis_Metabolic& m) {
    os << "Metabolic:\n"
       << "  basal_rate_W: " << m.basal_rate_W << "\n"
       << "  max_rate_W: " << m.max_rate_W << "\n"
       << "  aerobic_scope: " << m.aerobic_scope << "\n"
       << "  muscle_mass_kg: " << m.muscle_mass_kg << "\n"
       << "  available_muscle_power_W: " << m.available_muscle_power_W << "\n"
       << "  is_endotherm: " << (m.is_endotherm() ? "true" : "false") << "\n";
    print_optional(os, "body_temperature_K", m.body_temperature_K);
    print_optional(os, "thermal_neutral_zone_min_K", m.thermal_neutral_zone_min_K);
    print_optional(os, "thermal_neutral_zone_max_K", m.thermal_neutral_zone_max_K);
    return os;
}

// Behavior
std::ostream& operator<<(std::ostream& os, const Analysis_Behavior& b) {
    os << "Behavior:\n"
       << "  aggression: " << b.aggression << "\n"
       << "  social_tendency: " << b.social_tendency << "\n"
       << "  activity_level: " << b.activity_level << "\n"
       << "  curiosity: " << b.curiosity << "\n"
       << "  territoriality: " << b.territoriality << "\n"
       << "  diurnal_preference: " << b.diurnal_preference << "\n"
       << "  is_migratory: " << (b.is_migratory ? "true" : "false") << "\n"
       << "  suggested_archetype: ";
    switch(b.suggested_archetype) {
        case Analysis_Behavior::AIArchetype::SOLITARY_AMBUSH_HUNTER: os << "SOLITARY_AMBUSH_HUNTER"; break;
        case Analysis_Behavior::AIArchetype::PACK_COORDINATOR: os << "PACK_COORDINATOR"; break;
        case Analysis_Behavior::AIArchetype::SOCIAL_FORAGER: os << "SOCIAL_FORAGER"; break;
        case Analysis_Behavior::AIArchetype::TERRITORIAL_DEFENDER: os << "TERRITORIAL_DEFENDER"; break;
        case Analysis_Behavior::AIArchetype::OPPORTUNISTIC_SCAVENGER: os << "OPPORTUNISTIC_SCAVENGER"; break;
        case Analysis_Behavior::AIArchetype::AERIAL_PREDATOR: os << "AERIAL_PREDATOR"; break;
        case Analysis_Behavior::AIArchetype::SCHOOLING_PREY: os << "SCHOOLING_PREY"; break;
        case Analysis_Behavior::AIArchetype::APEX_PREDATOR: os << "APEX_PREDATOR"; break;
    }
    os << "\n";
    return os;
}

// Sensory
std::ostream& operator<<(std::ostream& os, const Output::Sensory& s) {
	if(s.vision)
	{
		os << "Sensory:\n"
		   << "  vision.acuity: " << s.vision->acuity << "\n"
		   << "  vision.binocular_overlap: " << s.vision->binocular_overlap << "\n"
		   << "  vision.has_color_vision: " << (s.vision->has_color_vision ? "true" : "false") << "\n"
		   << "  vision.has_night_vision: " << (s.vision->has_night_vision ? "true" : "false") << "\n"
		   << "  vision.detection_range_m: " << s.vision->detection_range_m << "\n";
    }

    if (s.hearing) {
        os << "  hearing.sensitivity: " << s.hearing->sensitivity << "\n"
           << "  hearing.frequency_range_Hz: " << s.hearing->frequency_range_Hz_min
           << " - " << s.hearing->frequency_range_Hz_max << "\n"
           << "  hearing.detection_range_m: " << s.hearing->detection_range_m << "\n";
    }

    if (s.olfaction) {
        os << "  olfaction.sensitivity: " << s.olfaction->sensitivity << "\n"
           << "  olfaction.detection_range_m: " << s.olfaction->detection_range_m << "\n";
    }

    return os;
}

// Diagnostics
std::ostream& operator<<(std::ostream& os, const Analysis_Diagnostics& d) {
    os << "Diagnostics:\n"
       << "  overall_confidence: " << d.overall_confidence << "\n"
       << "  passes_power_budget_check: " << (d.passes_power_budget_check ? "true" : "false") << "\n"
       << "  passes_mass_budget_check: " << (d.passes_mass_budget_check ? "true" : "false") << "\n"
       << "  is_physically_plausible: " << (d.is_physically_plausible ? "true" : "false") << "\n";

    if (!d.warnings.empty()) {
        os << "  warnings:\n";
        for (const auto& w : d.warnings) {
            os << "    [";
            switch(w.level) {
                case Analysis_Diagnostics::Warning::Severity::INFO: os << "INFO"; break;
                case Analysis_Diagnostics::Warning::Severity::CAUTION: os << "CAUTION"; break;
                case Analysis_Diagnostics::Warning::Severity::ERROR: os << "ERROR"; break;
            }
            os << "] " << w.message << "\n";
        }
    }
    return os;
}

// BodyWave
std::ostream& operator<<(std::ostream& os, const Analysis_BodyWave& bw) {
    os << "    BodyWave(root:" << static_cast<int>(bw.root) << " tip:" << static_cast<int>(bw.tip)
       << " λ/L:" << bw.wavelength_ratio << " A/L:" << bw.amplitude_ratio
       << " flex:" << bw.body_flexibility << ")";
    return os;
}


// Terrestrial::SerpentineLocomotion
std::ostream& operator<<(std::ostream& os, const Analysis_Serpentine& serp) {
    os << "  SerpentineLocomotion:\n";

    os << "    capable_modes: ";
    {
        if((int)serp.capable_modes & (int)Analysis_Serpentine::Mode::LATERAL_UNDULATION) 
                os << "LATERAL_UNDULATION";
        if((int)serp.capable_modes & (int)Analysis_Serpentine::Mode::RECTILINEAR) 
                os << " RECTILINEAR"; 
        if((int)serp.capable_modes & (int)Analysis_Serpentine::Mode::SIDEWINDING) 
                os << " SIDEWINDING"; 
        if((int)serp.capable_modes & (int)Analysis_Serpentine::Mode::CONCERTINA) 
                os << " CONCERTINA"; 
    }
    os << "\n";

    os << "    friction: forward=" << serp.forward_friction_coef
       << " lateral=" << serp.lateral_friction_coef
       << " anisotropy=" << serp.friction_anisotropy_ratio << "\n";

    //if (serp.lateral_undulation) 
    {
        os << "    lateral_undulation: " << serp.lateral_undulation << "\n";
    }

    if (serp.rectilinear) {
        os << "    rectilinear: speed=" << serp.rectilinear->speed_m_s << "m/s";
        os << " freq=" << serp.rectilinear->frequency_Hz << "Hz";
        os << "\n";
    }

    if (serp.sidewinding) {
        os << "    sidewinding: speed=" << serp.sidewinding->speed_m_s << "m/s";
        os << " freq=" << serp.sidewinding->frequency_Hz << "Hz";
        os << " A/L=" << serp.sidewinding->amplitude_ratio;
        os << " contacts=" << serp.sidewinding->contact_points;
        os << "\n";
    }

    if (serp.concertina) {
        os << "    concertina: speed=" << serp.concertina->speed_m_s << "m/s";
        os << " compression=" << serp.concertina->compression_ratio;
        os << "\n";
    }

    return os;
}

// Terrestrial
std::ostream& operator<<(std::ostream& os, const Analysis_Terrestrial& t) {
    os << "Terrestrial:\n"
       << "  posture: " << t.posture;
    os << "\n  can_breathe_while_running: " << (t.max_sprint_duration_s < 0? "true" : "false") << "\n";

    os << "  legs:\n";
    for (const auto& leg : t.legs) {
        os << leg << "\n";
    }

    os << "  max_sprint_speed_m_s: " << t.max_sprint_speed_m_s << "\n"
       << "  max_sustainable_speed_m_s: " << t.max_sustainable_speed_m_s << "\n"
       << "  optimal_speed_m_s: " << t.optimal_speed_m_s << "\n";

    print_optional(os, "max_sprint_duration_s", t.max_sprint_duration_s);
    print_optional(os, "recovery_time_s", t.recovery_time_s);

    return os;
}

// Aerial::Wing
std::ostream& operator<<(std::ostream& os, const Analysis_Aerial::Wing& wing) {
    os << "    Wing(root:" << static_cast<int>(wing.root) << " tip:" << static_cast<int>(wing.tip)
       << " span:" << wing.span_m << "m area:" << wing.area_m2 << "m²"
       << " chord length:" << wing.chord_m << "m " 
       << " AR:" << wing.aspect_ratio() << ")";
    return os;
}

// Aerial
std::ostream& operator<<(std::ostream& os, const Analysis_Aerial& a) {
    os << "Aerial:\n"
       << "  wings:\n";
    for (const auto& wing : a.wings) {
        os << wing << "\n";
    }

    os << "  wingbeat_frequency_Hz: " << a.wingbeat_frequency_Hz << "\n"
       << "  can_sustain_level_flight: " << (a.can_sustain_level_flight ? "yes" : "no") << "\n"
       << "  can_slow_descent: " << (a.can_slow_descent ? "yes" : "no") << "\n"
       << "  can_hover: " << (a.can_hover ? "yes" : "no") << "\n"
       << "  speeds (min/cruise/max): " << a.min_flight_speed_m_s << "/"
       << a.cruise_speed_m_s << "/" << a.max_flight_speed_m_s << " m/s\n"
       
       << "  turning radius: " << a.min_turning_radius_m << " m\n"
       << "  manuverability (roll/pitch/yaw): " 
			<< a.max_roll_rate_rad_s << "/"
			<< a.max_pitch_rate_rad_s << "/"
			<< a.max_yaw_rate_rad_s << " rad/s\n"
       
       << "  cost (flap/hover): " << a.flapping_cost_W_per_N << "/" <<  a.hovering_cost_W_per_N << " W/N\n";
       
    return os;
}

// Aquatic::Fin
std::ostream& operator<<(std::ostream& os, const Analysis_Aquatic::Fin& fin) {
    os << "    Fin(root:" << static_cast<int>(+fin.root) << " tip:" << static_cast<int>(+fin.tip)
       << " area:" << fin.area_m2 << "m²)";
    return os;
}

// Aquatic::CStartResponse
std::ostream& operator<<(std::ostream& os, const Analysis_Aquatic::CStartResponse& cstart) {
    os << "  CStartResponse:\n"
       << "    duration_s: " << cstart.duration_s << "\n"
       << "    max_body_curvature_rad: " << cstart.max_body_curvature_rad << "\n"
       << "    acceleration_m_s2: " << cstart.acceleration_m_s2 << "\n";
    return os;
}

// Aquatic::JetPropulsion
std::ostream& operator<<(std::ostream& os, const Analysis_Aquatic::JetPropulsion& jet) {
    os << "  JetPropulsion:\n"
       << "    mantle_contraction_frequency_Hz: " << jet.mantle_contraction_frequency_Hz << "\n"
       << "    jet_pulse_volume_m3: " << jet.jet_pulse_volume_m3 << "\n"
       << "    jet_velocity_m_s: " << jet.jet_velocity_m_s << "\n"
       << "    siphon_joint: " << static_cast<int>(jet.siphon_joint) << "\n"
       << "    siphon_articulation_range_rad: " << jet.siphon_articulation_range_rad << "\n";
    return os;
}

// Aquatic
std::ostream& operator<<(std::ostream& os, const Analysis_Aquatic& aq) {
    os << "Aquatic:\n"
       << "  propulsion_mode: ";
    switch(aq.primary_mode) {
        case Analysis_Aquatic::PropulsionMode::BODY_CAUDAL_FIN: os << "BODY_CAUDAL_FIN"; break;
        case Analysis_Aquatic::PropulsionMode::MEDIAN_PAIRED_FIN: os << "MEDIAN_PAIRED_FIN"; break;
        case Analysis_Aquatic::PropulsionMode::JET_PROPULSION: os << "JET_PROPULSION"; break;
        case Analysis_Aquatic::PropulsionMode::PADDLE_LIMBS: os << "PADDLE_LIMBS"; break;
        case Analysis_Aquatic::PropulsionMode::DORSOVENTRAL_FLUKES: os << "DORSOVENTRAL_FLUKES"; break;
    }
    os << "\n";

    // Kinematics
    os << "\n  === KINEMATICS ===\n"
       << "  tail_beat_frequency: " << aq.beat_frequency_Hz << " Hz\n"
       << "  tail_amplitude: " << aq.tail_amplitude_m << " m ("
       << (aq.tail_amplitude_m > 0 ? aq.tail_amplitude_m / 2.3 * 100 : 0) << "% body length)\n"  // Approximate
       << "  speeds (min/cruise/burst): " << aq.min_swim_speed_m_s << "/"
       << aq.cruise_speed_m_s << "/" << aq.burst_speed_m_s << " m/s\n";

    // Hydrodynamics
    os << "\n  === HYDRODYNAMICS ===\n"
       << "  Reynolds_number: " << aq.reynolds_number;
    if (aq.reynolds_number < 1000) os << " (viscous regime)";
    else if (aq.reynolds_number < 100000) os << " (transitional)";
    else os << " (turbulent/inertial)";
    os << "\n"
       << "  drag_coefficient: " << aq.drag_coefficient;
    if (aq.drag_coefficient < 0.05) os << " (highly streamlined)";
    else if (aq.drag_coefficient < 0.1) os << " (streamlined)";
    else os << " (moderate streamlining)";
    os << "\n";

    // Buoyancy
    os << "\n  === BUOYANCY ===\n"
       << "  neutral_buoyancy_density: " << aq.neutral_buoyancy_density_kg_m3 << " kg/m³\n"
       << "  has_swim_bladder: " << (aq.has_swim_bladder ? "yes" : "no") << "\n";
    if (aq.has_swim_bladder) {
        os << "  swim_bladder_adjust_time: " << aq.swim_bladder_adjust_time_s << " s\n";
    }
    if (aq.sink_rate_m_s > 0.001f) {
        os << "  sink_rate_when_stationary: " << aq.sink_rate_m_s << " m/s (negatively buoyant)\n"
           << "  lift_required_per_meter: " << aq.lift_per_meter_swam_N << " N/m\n"
           << "  requires_constant_motion: " << (aq.requires_constant_motion ? "YES" : "no") << "\n";
    } else {
        os << "  buoyancy: neutral or positive\n";
    }

    // Maneuverability
    os << "\n  === MANEUVERABILITY ===\n"
       << "  min_turning_radius: " << aq.min_turning_radius_m << " m\n"
       << "  can_hover: " << (aq.can_hover ? "yes" : "no") << "\n";

    // Propulsors
    if (!aq.propulsors.empty()) {
        os << "\n  === PROPULSORS ===\n";
        for (const auto& fin : aq.propulsors) {
            os << "  " << fin << "\n";
        }
    }

    // Body wave
    if (aq.body_wave) {
        os << "\n  === BODY UNDULATION ===\n  " << *aq.body_wave;
    }

    // C-start response
    if (aq.c_start) {
        os << "\n  === ESCAPE RESPONSE ===\n  " << *aq.c_start;
    }

    // Jet propulsion
    if (aq.jet_propulsion) {
        os << "\n  === JET PROPULSION ===\n  " << *aq.jet_propulsion;
    }

    return os;
}

// Climbing
std::ostream& operator<<(std::ostream& os, const Analysis_Climbing& c) {
    os << "Climbing:\n"
       << "  limbs:\n";
    for (const auto& limb : c.limbs) {
        os << limb << "\n";
    }

    os << "  can_descend_head_first: " << (c.can_descend_head_first ? "true" : "false") << "\n"
       << "  max_climb_speed_m_s: " << c.max_climb_speed_m_s << "\n"
       << "  max_climb_angle_rad: " << c.max_climb_angle_rad << "\n";
    return os;
}

// Jumping
std::ostream& operator<<(std::ostream& os, const Analysis_Jumping& j) {
    os << "Jumping:\n"
       << "  mechanism: ";
    switch(j.mechanism) {
        case Analysis_Jumping::MechanismType::MUSCLE_DIRECT: os << "MUSCLE_DIRECT"; break;
        case Analysis_Jumping::MechanismType::ELASTIC_CATAPULT: os << "ELASTIC_CATAPULT"; break;
        case Analysis_Jumping::MechanismType::HYDRAULIC: os << "HYDRAULIC"; break;
    }
    os << "\n"
       << "  max_jump_height_m: " << j.max_jump_height_m << "\n"
       << "  max_jump_distance_m: " << j.max_jump_distance_m << "\n"
       << "  takeoff_velocity_m_s: " << j.takeoff_velocity_m_s << "\n";
    return os;
}

// Manipulation::Manipulator
std::ostream& operator<<(std::ostream& os, const Analysis_Manipulator& manip) {
    os << "    Manipulator(root:" << static_cast<int>(manip.root) << " tip:" << static_cast<int>(manip.tip)
       << " reach:" << manip.stretched_length_m << "m"
       << " grip:" << manip.max_grip_force_N << "N"
       //<< " dof:" << manip.degrees_of_freedom << 
       ")";
    return os;
}

// Manipulation
std::ostream& operator<<(std::ostream& os, const std::vector<Analysis_Manipulator> & m) {
    os << "Manipulation:\n"
       << "  manipulators:\n";
    for (const auto& manip : m) {
        os << manip << "\n";
    }
    return os;
}

// Brachiation::Arm
std::ostream& operator<<(std::ostream& os, const Analysis_Brachiation::Arm& arm) {
    os << "    Arm(root:" << static_cast<int>(arm.root) << " hand:" << static_cast<int>(arm.tip)
       << " reach:" << arm.reach_m << "m"
       << " grip:" << arm.grip_strength_N << "N)";
    return os;
}

// Brachiation
std::ostream& operator<<(std::ostream& os, const Analysis_Brachiation& br) {
    os << "Brachiation:\n"
       << "  arms:\n";
    for (const auto& arm : br.arms) {
        os << arm << "\n";
    }

    os << "  swing_frequency_Hz: " << br.swing_frequency_Hz << "\n"
       << "  arm_phase_offset: " << br.arm_phase_offset << "\n"
       << "  max_swing_speed_m_s: " << br.max_swing_speed_m_s << "\n";
    return os;
}

// Appendages::Tail
std::ostream& operator<<(std::ostream& os, const Output::Appendages::Tail& tail) {
    os << "    Tail(root:" << static_cast<int>(tail.root)
       << " tip:" << static_cast<int>(tail.tip)
       << " len:" << tail.stretched_length_m << "m";
   // if (tail.used_for & Output::Appendages::Tail::Balance) os << " BALANCE";
  //  if (tail.used_for & Output::Appendages::Tail::Propulsion) os << " PROPULSION";
    if (tail.used_for & Output::Appendages::Tail::Grasping) os << " GRASPING";
    os << ")\n";

    if (!tail.branches.empty()) {
        os << "      branches:\n";
        for (const auto& branch : tail.branches) {
            os << branch << "\n";
        }
    }
    return os;
}

// Appendages
std::ostream& operator<<(std::ostream& os, const Output::Appendages& app) {
    if (app.tails.empty()) {
        return os;
    }

    os << "Appendages:\n";

    if (!app.tails.empty()) {
        os << "  tails:\n";
        for (const auto& tail : app.tails) {
            os << tail;
        }
    }

    return os;
}

// Main Output
std::ostream& operator<<(std::ostream& os, const Output& output) {
    os << "=== TonTon Output ===\n\n";

    os << output.physical << "\n";
    os << output.metabolic << "\n";
    os << output.behavior << "\n";
    os << output.sensory << "\n";
    os << output.diagnostics << "\n";

    if (output.terrestrial) {
        os << *output.terrestrial << "\n";
    }

    if (output.serpentine) {
        os << *output.serpentine;
    }

    if (output.aerial) {
        os << *output.aerial << "\n";
    }

    if (output.aquatic) {
        os << *output.aquatic << "\n";
    }

    if (output.climbing) {
        os << *output.climbing << "\n";
    }

    if (output.jumping) {
        os << *output.jumping << "\n";
    }

    if (output.appendages.manipulation.size()) {
        os << output.appendages.manipulation << "\n";
    }

    if (output.brachiation) {
        os << *output.brachiation << "\n";
    }

    os << output.appendages;

    os << "===================\n";
    return os;
}

std::ostream& operator<<(std::ostream& os, const Word& output)
{
	return os << WordToString(output);
}

std::ostream& operator<<(std::ostream& os, const SemanticFlags& output)
{
	bool outputted = false;
	
	for(auto i = 0u; i < 64; ++i)
	{
		auto flag = SemanticFlags(uint64_t(1) << uint64_t(i));
		
		
		if(HasFlag(output, flag))
		{
			if(outputted)	os << "|";	
			os << WordToString(flag);				
			outputted = true;
		}		
	}
	
	return os;
}

std::ostream& operator<<(std::ostream& os, const CladeFlags& output)
{	bool outputted = false;
	
	for(auto i = 0u; i < 64; ++i)
	{
		auto flag = CladeFlags(uint64_t(1) << uint64_t(i));
		
		
		if(HasFlag(output, flag))
		{
			if(outputted)	os << "|";	
			os << WordToString(flag);				
			outputted = true;
		}		
	}
	
	return os;
}

} // namespace TonTon
