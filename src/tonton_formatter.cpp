#include "../include/tonton_formatter.h"

namespace TonTon {

// Helper for optional values
template<typename T>
static std::string format_optional(const char* name, const std::optional<T>& opt) {
	if (opt) {
	// not leaf so don't grow stack with snprintf
		auto str = format(*opt);
		std::string buffer;
		buffer.reserve(str.size() + strlen(name) + 8);
		
		buffer += "   ";
		buffer += name;
		buffer += ": ";
		buffer += str;
		buffer += "\n";
		
		return buffer;
	}
	return "";
}

// Helper for optional values (raw types with negative sentinel)
template<typename T>
static std::string format_optional(const char* name, const T& value) {
	if (static_cast<float>(value) > 0) {
		auto str = std::to_string(float(value));
		std::string buffer;
		buffer.reserve(str.size() + strlen(name) + 8);
		
		buffer += "   ";
		buffer += name;
		buffer += ": ";
		buffer += str;
		buffer += "\n";
		
		return buffer;
	}
	return "";
}

// ============================================================================
// ANALYSIS FORMATTERS
// ============================================================================

// Physical
std::string format(const Analysis_Physical& p) {
	char buffer[1024];
	auto chars_printed = snprintf(buffer, sizeof(buffer),
				"Physical:\n"
				"  body_mass_kg: %f\n"
				"  svl_m: %f\n"
				"  total_length_m: %f\n"
				"  body_volume_m3: %f\n"
				"  surface_area_m2: %f\n"
				"  cross_sectional_area_m2: %f\n"
				"  cross_sectional_diameter: %f\n"
				"  fineness_ratio: %f\n",
				float(p.body_mass_kg),
				float(p.svl_m()),
				float(p.body_length_m),
				float(p.body_volume_m3),
				float(p.surface_area_m2),
				float(p.cross_sectional_area_m2),
				float(p.cross_sectional_diameter_m()),
				p.fineness_ratio()
				);
	return std::string(buffer, chars_printed);
}

// Metabolic
std::string format(const Analysis_Metabolic& m) {
	char buffer[1024];
	auto chars_printed = snprintf(buffer, sizeof(buffer),
				"Metabolic:\n"
				"  basal_rate_W: %f\n"
				"  max_rate_W: %f\n"
				"  aerobic_scope: %f\n"
				"  muscle_mass_kg: %f\n"
				"  available_muscle_power_W: %f\n"
				"  is_endotherm: %s\n",
				float(m.basal_rate_W),
				float(m.max_rate_W),
				m.aerobic_scope(),
				float(m.muscle_mass_kg),
				float(m.available_muscle_power_W),
				m.is_endotherm() ? "true" : "false"
		);
		
	std::string result = std::string(buffer, chars_printed);
	result += format_optional("body_temperature_K", m.body_temperature_K);
	result += format_optional("thermal_neutral_zone_min_K", m.thermal_neutral_zone_min_K);
	result += format_optional("thermal_neutral_zone_max_K", m.thermal_neutral_zone_max_K);
	return result;
}

// Behavior
std::string format(const Analysis_Behavior& b) {
	const char* archetype_str = "UNKNOWN";
	switch(b.suggested_archetype) {
	case Analysis_Behavior::AIArchetype::SOLITARY_AMBUSH_HUNTER:
		archetype_str = "SOLITARY_AMBUSH_HUNTER"; break;
	case Analysis_Behavior::AIArchetype::PACK_COORDINATOR:
		archetype_str = "PACK_COORDINATOR"; break;
	case Analysis_Behavior::AIArchetype::SOCIAL_FORAGER:
		archetype_str = "SOCIAL_FORAGER"; break;
	case Analysis_Behavior::AIArchetype::TERRITORIAL_DEFENDER:
		archetype_str = "TERRITORIAL_DEFENDER"; break;
	case Analysis_Behavior::AIArchetype::OPPORTUNISTIC_SCAVENGER:
		archetype_str = "OPPORTUNISTIC_SCAVENGER"; break;
	case Analysis_Behavior::AIArchetype::AERIAL_PREDATOR:
		archetype_str = "AERIAL_PREDATOR"; break;
	case Analysis_Behavior::AIArchetype::SCHOOLING_PREY:
		archetype_str = "SCHOOLING_PREY"; break;
	case Analysis_Behavior::AIArchetype::APEX_PREDATOR:
		archetype_str = "APEX_PREDATOR"; break;
	default: break;
	}

	char buffer[1024];
	auto chars_printed = snprintf(buffer, sizeof(buffer),
				"Behavior:\n"
				"  aggression: %f\n"
				"  social_tendency: %f\n"
				"  activity_level: %f\n"
				"  curiosity: %f\n"
				"  territoriality: %f\n"
				"  diurnal_preference: %f\n"
				"  is_migratory: %s\n"
				"  suggested_archetype: %s\n",
				b.aggression,
				b.social_tendency,
				b.activity_level,
				b.curiosity,
				b.territoriality,
				b.diurnal_preference,
				b.is_migratory ? "true" : "false",
				archetype_str
				);
	return std::string(buffer, chars_printed);
}

// Sensory
std::string format(const Analysis_Sensory<optional>& s) {
	std::string result;
	char buffer[512];

	if (s.vision) {
		auto chars_printed = snprintf(buffer, sizeof(buffer),
					"Sensory:\n"
					"  vision.acuity: %f\n"
					"  vision.binocular_overlap: %f\n"
					"  vision.has_color_vision: %s\n"
					"  vision.has_night_vision: %s\n"
					"  vision.detection_range_m: %f\n",
					s.vision->acuity,
					s.vision->binocular_overlap,
					s.vision->has_color_vision ? "true" : "false",
					s.vision->has_night_vision ? "true" : "false",
					float(s.vision->detection_range_m)
					);
		result += std::string(buffer, chars_printed);
	}

	if (s.hearing) {
		auto chars_printed = snprintf(buffer, sizeof(buffer),
					"  hearing.sensitivity: %f\n"
					"  hearing.frequency_range_Hz: %f - %f\n"
					"  hearing.detection_range_m: %f\n",
					s.hearing->sensitivity,
					float(s.hearing->frequency_range_Hz_min),
					float(s.hearing->frequency_range_Hz_max),
					float(s.hearing->detection_range_m)
					);
		result += std::string(buffer, chars_printed);
	}

	if (s.olfaction) {
		auto chars_printed = snprintf(buffer, sizeof(buffer),
					"  olfaction.sensitivity: %f\n"
					"  olfaction.detection_range_m: %f\n",
					s.olfaction->sensitivity,
					float(s.olfaction->detection_range_m)
					);
		result += std::string(buffer, chars_printed);
	}

	return result;
}

// Diagnostics
std::string format(const Analysis_Diagnostics& d) {
	char buffer[1024];
	auto chars_printed = snprintf(buffer, sizeof(buffer),
				"Diagnostics:\n"
				"  overall_confidence: %f\n"
				"  passes_power_budget_check: %s\n"
				"  passes_mass_budget_check: %s\n"
				"  is_physically_plausible: %s\n",
				d.overall_confidence,
				d.passes_power_budget_check ? "true" : "false",
				d.passes_mass_budget_check ? "true" : "false",
				d.is_physically_plausible ? "true" : "false"
											);
	std::string result(buffer, chars_printed);

	if (!d.warnings.empty()) {
		result += "  warnings:\n";
		for (const auto& w : d.warnings) {
			const char* severity_str = "UNKNOWN";
			switch(w.level) {
			case Analysis_Diagnostics::Warning::Severity::INFO: severity_str = "INFO"; break;
			case Analysis_Diagnostics::Warning::Severity::CAUTION: severity_str = "CAUTION"; break;
			case Analysis_Diagnostics::Warning::Severity::ERROR: severity_str = "ERROR"; break;
			default: break;
			}
			chars_printed = snprintf(buffer, sizeof(buffer), "    [%s] %s\n", severity_str, w.message.c_str());
			result += std::string(buffer, chars_printed);
		}
	}
	return result;
}

// BodyWave
std::string format(const Analysis_BodyWave& bw) {
	char buffer[512];
	auto chars_printed = snprintf(buffer, sizeof(buffer),
				"    BodyWave(root:%i tip:%i λ/L:%f A/L:%f flex:%f)",
				static_cast<int>(bw.root),
				static_cast<int>(bw.tip),
				float(bw.wavelength_ratio),
				float(bw.amplitude_ratio),
				bw.body_flexibility
				);
	return std::string(buffer, chars_printed);
}

// Serpentine
std::string format(const Analysis_Serpentine& serp) {
	std::string result = "  SerpentineLocomotion:\n    capable_modes: ";
	
	bool first = true;
	if ((int)serp.capable_modes & (int)Analysis_Serpentine::Mode::LATERAL_UNDULATION) {
		if (!first) result += " ";
		result += "LATERAL_UNDULATION";
		first = false;
	}
	if ((int)serp.capable_modes & (int)Analysis_Serpentine::Mode::RECTILINEAR) {
		if (!first) result += " ";
		result += "RECTILINEAR";
		first = false;
	}
	if ((int)serp.capable_modes & (int)Analysis_Serpentine::Mode::SIDEWINDING) {
		if (!first) result += " ";
		result += "SIDEWINDING";
		first = false;
	}
	if ((int)serp.capable_modes & (int)Analysis_Serpentine::Mode::CONCERTINA) {
		if (!first) result += " ";
		result += "CONCERTINA";
		first = false;
	}
	result += "\n";
	
	result += std::format(
				"    friction: forward={} lateral={} anisotropy={}\n",
				serp.forward_friction_coef,
				serp.lateral_friction_coef,
				serp.friction_anisotropy_ratio
				);
	
	result += std::format("    lateral_undulation: {}\n", format(serp.lateral_undulation));
	
	if (serp.rectilinear) {
		result += std::format(
					"    rectilinear: speed={}m/s freq={}Hz\n",
					serp.rectilinear->speed_m_s,
					serp.rectilinear->frequency_Hz
					);
	}
	
	if (serp.sidewinding) {
		result += std::format(
					"    sidewinding: speed={}m/s freq={}Hz A/L={} contacts={}\n",
					serp.sidewinding->speed_m_s,
					serp.sidewinding->frequency_Hz,
					serp.sidewinding->amplitude_ratio,
					serp.sidewinding->contact_points
					);
	}
	
	if (serp.concertina) {
		result += std::format(
					"    concertina: speed={}m/s compression={}\n",
					serp.concertina->speed_m_s,
					serp.concertina->compression_ratio
					);
	}
	
	return result;
}

// Terrestrial
std::string format(const Analysis_Terrestrial& t) {
	std::string result = std::format(
				"Terrestrial:\n"
				"  posture: {}\n"
				"  can_breathe_while_running: {}\n"
				"  legs:\n",
				t.posture,
				(static_cast<float>(t.max_sprint_duration_s) < 0) ? "true" : "false"
																	);
	
	for (const auto& leg : t.legs) {
		result += format(leg) + "\n";
	}
	
	result += std::format(
				"  max_sprint_speed_m_s: {}\n"
				"  max_sustainable_speed_m_s: {}\n"
				"  optimal_speed_m_s: {}\n",
				t.max_sprint_speed_m_s,
				t.max_sustainable_speed_m_s,
				t.optimal_speed_m_s
				);
	
	result += format_optional("max_sprint_duration_s", t.max_sprint_duration_s);
	result += format_optional("recovery_time_s", t.recovery_time_s);
	
	return result;
}

// Aerial::Wing
std::string format(const Analysis_Aerial::Wing& wing) {
	char buffer[1024];
	auto chars_printed = snprintf(buffer, sizeof(buffer),  "Wing(root: %i tip: %i span: %f m area: %f m² chord length: %f m AR: %f)",
				static_cast<int>(wing.root),
				static_cast<int>(wing.tip),
				float(wing.span_m),
				float(wing.wing_area_m2),
				float(wing.chord_m),
				wing.aspect_ratio());
			
	return std::string(buffer, chars_printed);	
}

// Aerial
std::string format(const Analysis_Aerial& a) {
	std::string result = "Aerial:\n  wings:\n";
	
	for (const auto& wing : a.wings) {
		result += format(wing) + "\n";
	}
	
	result += std::format(
				"  wingbeat_frequency_Hz: {}\n"
				"  can_sustain_level_flight: {}\n"
				"  can_slow_descent: {}\n"
				"  can_hover: {}\n"
				"  speeds (min/cruise/max): {}/{}/{} m/s\n"
				"  turning radius: {} m\n"
				"  manuverability (roll/pitch/yaw): {}/{}/{} rad/s\n"
				"  cost (flap/hover): {}/{} W/N\n",
				a.wingbeat_frequency_Hz,
				a.can_sustain_level_flight ? "yes" : "no",
				a.can_slow_descent ? "yes" : "no",
				a.can_hover ? "yes" : "no",
				a.min_flight_speed_m_s,
				a.cruise_speed_m_s,
				a.max_flight_speed_m_s,
				a.min_turning_radius_m,
				a.max_roll_rate_rad_s,
				a.max_pitch_rate_rad_s,
				a.max_yaw_rate_rad_s,
				a.flapping_cost_W_per_N,
				a.hovering_cost_W_per_N
				);
	
	return result;
}

// Aquatic::Fin
std::string format(const Analysis_Aquatic::Fin& fin) {
	char buffer[256];
	auto chars_printed = snprintf(buffer, sizeof(buffer),
				"    Fin(root:%i tip:%i area:%fm²)",
				static_cast<int>(fin.root),
				static_cast<int>(fin.tip),
				float(fin.fin_area_m2)
				);
	return std::string(buffer, chars_printed);
}

// Aquatic::CStartResponse
std::string format(const Analysis_Aquatic::CStartResponse& cstart) {
	char buffer[512];
	auto chars_printed = snprintf(buffer, sizeof(buffer),
				"  CStartResponse:\n"
				"    duration_s: %f\n"
				"    max_body_curvature_rad: %f\n"
				"    acceleration_m_s2: %f\n",
				float(cstart.duration_s),
				float(cstart.max_body_curvature_rad),
				float(cstart.c_acceleration_m_s2)
				);
	return std::string(buffer, chars_printed);
}

// Aquatic::JetPropulsion
std::string format(const Analysis_Aquatic::JetPropulsion& jet) {
	char buffer[512];
	auto chars_printed = snprintf(buffer, sizeof(buffer),
				"  JetPropulsion:\n"
				"    mantle_contraction_frequency_Hz: %f\n"
				"    jet_pulse_volume_m3: %f\n"
				"    jet_velocity_m_s: %f\n"
				"    siphon_joint: %i\n"
				"    siphon_articulation_range_rad: %f\n",
				float(jet.mantle_contraction_frequency_Hz),
				float(jet.jet_pulse_volume_m3),
				float(jet.jet_velocity_m_s),
				static_cast<int>(jet.siphon_joint),
				float(jet.siphon_articulation_range_rad)
				);
	return std::string(buffer, chars_printed);
}

// Aquatic
std::string format(const Analysis_Aquatic& aq) {
	std::string result = "Aquatic:\n  propulsion_mode: ";
	
	switch(aq.primary_mode) {
	case Analysis_Aquatic::PropulsionMode::BODY_CAUDAL_FIN: result += "BODY_CAUDAL_FIN"; break;
	case Analysis_Aquatic::PropulsionMode::MEDIAN_PAIRED_FIN: result += "MEDIAN_PAIRED_FIN"; break;
	case Analysis_Aquatic::PropulsionMode::JET_PROPULSION: result += "JET_PROPULSION"; break;
	case Analysis_Aquatic::PropulsionMode::PADDLE_LIMBS: result += "PADDLE_LIMBS"; break;
	case Analysis_Aquatic::PropulsionMode::DORSOVENTRAL_FLUKES: result += "DORSOVENTRAL_FLUKES"; break;
	}
	result += "\n";
	
	// Kinematics
	result += std::format(
				"\n  === KINEMATICS ===\n"
				"  tail_beat_frequency: {} Hz\n"
				"  tail_amplitude: {} m ({}% body length)\n"
				"  speeds (min/cruise/burst): {}/{}/{} m/s\n",
				aq.beat_frequency_Hz,
				aq.tail_amplitude_m,
				(static_cast<float>(aq.tail_amplitude_m) > 0 ? static_cast<float>(aq.tail_amplitude_m) / 2.3f * 100 : 0),
				aq.min_swim_speed_m_s,
				aq.cruise_speed_m_s,
				aq.burst_speed_m_s
				);
	
	// Hydrodynamics
	std::string regime;
	if (aq.reynolds_number < 1000) regime = " (viscous regime)";
	else if (aq.reynolds_number < 100000) regime = " (transitional)";
	else regime = " (turbulent/inertial)";
	
	std::string streamlining;
	if (aq.drag_coefficient < 0.05f) streamlining = " (highly streamlined)";
	else if (aq.drag_coefficient < 0.1f) streamlining = " (streamlined)";
	else streamlining = " (moderate streamlining)";
	
	result += std::format(
				"\n  === HYDRODYNAMICS ===\n"
				"  Reynolds_number: {}{}\n"
				"  drag_coefficient: {}{}\n",
				aq.reynolds_number, regime,
				aq.drag_coefficient, streamlining
				);
	
	// Buoyancy
	result += std::format(
				"\n  === BUOYANCY ===\n"
				"  neutral_buoyancy_density: {} kg/m³\n"
				"  has_swim_bladder: {}\n",
				aq.neutral_buoyancy_density_kg_m3,
				aq.has_swim_bladder ? "yes" : "no"
									  );
	
	if (aq.has_swim_bladder) {
		result += std::format("  swim_bladder_adjust_time: {} s\n", aq.swim_bladder_adjust_time_s);
	}
	
	if (static_cast<float>(aq.sink_rate_m_s) > 0.001f) {
		result += std::format(
					"  sink_rate_when_stationary: {} m/s (negatively buoyant)\n"
					"  lift_required_per_meter: {} N/m\n"
					"  requires_constant_motion: {}\n",
					aq.sink_rate_m_s,
					aq.lift_per_meter_swam_N,
					aq.requires_constant_motion ? "YES" : "no"
												  );
	} else {
		result += "  buoyancy: neutral or positive\n";
	}
	
	// Maneuverability
	result += std::format(
				"\n  === MANEUVERABILITY ===\n"
				"  min_turning_radius: {} m\n"
				"  can_hover: {}\n",
				aq.min_turning_radius_m,
				aq.can_hover ? "yes" : "no"
							   );
	
	// Propulsors
	if (!aq.propulsors.empty()) {
		result += "\n  === PROPULSORS ===\n";
		for (const auto& fin : aq.propulsors) {
			result += "  " + format(fin) + "\n";
		}
	}
	
	// Body wave
	if (aq.body_wave) {
		result += "\n  === BODY UNDULATION ===\n  " + format(*aq.body_wave);
	}
	
	// C-start response
	if (aq.c_start) {
		result += "\n  === ESCAPE RESPONSE ===\n  " + format(*aq.c_start);
	}
	
	// Jet propulsion
	if (aq.jet_propulsion) {
		result += "\n  === JET PROPULSION ===\n  " + format(*aq.jet_propulsion);
	}
	
	return result;
}

// Climbing
std::string format(const Analysis_Climbing& c) {
	std::string result = "Climbing:\n  limbs:\n";
	
	for (const auto& limb : c.limbs) {
		result += format(limb) + "\n";
	}
	
	result += std::format(
				"  can_descend_head_first: {}\n"
				"  max_climb_speed_m_s: {}\n"
				"  max_climb_angle_rad: {}\n",
				c.can_descend_head_first ? "true" : "false",
				c.max_climb_speed_m_s,
				c.max_climb_angle_rad
				);
	
	return result;
}

// Jumping
std::string format(const Analysis_Jumping& j) {
	const char* mechanism_str = "UNKNOWN";
	switch(j.mechanism) {
	case Analysis_Jumping::MechanismType::MUSCLE_DIRECT: mechanism_str = "MUSCLE_DIRECT"; break;
	case Analysis_Jumping::MechanismType::ELASTIC_CATAPULT: mechanism_str = "ELASTIC_CATAPULT"; break;
	case Analysis_Jumping::MechanismType::HYDRAULIC: mechanism_str = "HYDRAULIC"; break;
	default: break;
	}

	char buffer[512];
	auto chars_printed = snprintf(buffer, sizeof(buffer),
				"Jumping:\n"
				"  mechanism: %s\n"
				"  max_jump_height_m: %f\n"
				"  max_jump_distance_m: %f\n"
				"  takeoff_velocity_m_s: %f\n",
				mechanism_str,
				float(j.max_jump_height_m),
				float(j.max_jump_distance_m),
				float(j.takeoff_velocity_m_s)
				);
	return std::string(buffer, chars_printed);
}

// Manipulator
std::string format(const Analysis_Manipulator& manip) {
	char buffer[256];
	auto chars_printed = snprintf(buffer, sizeof(buffer),
				"    Manipulator(root:%i tip:%i reach:%fm grip:%fN)",
				static_cast<int>(manip.root),
				static_cast<int>(manip.tip),
				float(manip.stretched_length_m),
				float(manip.max_grip_force_N)
				);
	return std::string(buffer, chars_printed);
}

// Brachiation::Arm
std::string format(const Analysis_Brachiation::Arm& arm) {
	char buffer[256];
	auto chars_printed = snprintf(buffer, sizeof(buffer),
				"    Arm(root:%i hand:%i reach:%fm grip:%fN)",
				static_cast<int>(arm.root),
				static_cast<int>(arm.tip),
				float(arm.reach_m),
				float(arm.grip_strength_N)
				);
	return std::string(buffer, chars_printed);
}

// Brachiation
std::string format(const Analysis_Brachiation& br) {
	std::string result = "Brachiation:\n  arms:\n";
	
	for (const auto& arm : br.arms) {
		result += format(arm) + "\n";
	}
	
	result += std::format(
				"  swing_frequency_Hz: {}\n"
				"  arm_phase_offset: {}\n"
				"  max_swing_speed_m_s: {}\n",
				br.swing_frequency_Hz,
				br.arm_phase_offset,
				br.max_swing_speed_m_s
				);
	
	return result;
}

// Tail
std::string format(const Analysis_Tail& tail) {
	std::string result = std::format(
				"    Tail(root:{} tip:{} len:{}m",
				static_cast<int>(tail.root),
				static_cast<int>(tail.tip),
				tail.stretched_length_m
				);
	
	if (tail.used_for & Analysis_Tail::Grasping) {
		result += " GRASPING";
	}
	result += ")\n";
	
	if (!tail.branches.empty()) {
		result += "      branches:\n";
		for (const auto& branch : tail.branches) {
			result += format(branch) + "\n";
		}
	}
	
	return result;
}

// Appendages
std::string format(const Output::Appendages& app) {
	if (app.tails.empty()) {
		return "";
	}
	
	std::string result = "Appendages:\n";
	
	if (!app.tails.empty()) {
		result += "  tails:\n";
		for (const auto& tail : app.tails) {
			result += format(tail);
		}
	}
	
	return result;
}

// Main Output
std::string format(const Output& output) {
	std::string result = "=== TonTon Output ===\n\n";
	
	result += format(output.physical) + "\n";
	result += format(output.metabolic) + "\n";
	result += format(output.behavior) + "\n";
	result += format(output.sensory) + "\n";
	result += format(output.diagnostics) + "\n";
	
	if (output.terrestrial) {
		result += format(*output.terrestrial) + "\n";
	}
	
	if (output.serpentine) {
		result += format(*output.serpentine);
	}
	
	if (output.aerial) {
		result += format(*output.aerial) + "\n";
	}
	
	if (output.aquatic) {
		result += format(*output.aquatic) + "\n";
	}
	
	if (output.climbing) {
		result += format(*output.climbing) + "\n";
	}
	
	if (output.jumping) {
		result += format(*output.jumping) + "\n";
	}
	
	if (output.appendages.manipulation.size()) {
		result += "Manipulation:\n  manipulators:\n";
		for (const auto& manip : output.appendages.manipulation) {
			result += format(manip) + "\n";
		}
		result += "\n";
	}
	
	if (output.brachiation) {
		result += format(*output.brachiation) + "\n";
	}
	
	result += format(output.appendages);
	
	result += "===================\n";
	return result;
}

// ============================================================================
// BUILDER FORMATTERS
// ============================================================================

// Builder_Chain
std::string format(const Builder_Chain& chain) {
	char buffer[256];
	auto chars_printed = snprintf(buffer, sizeof(buffer),
				"Chain(root:%u tip:%u joints:%u stretched:%f rest:%f)",
				chain.root,
				chain.tip,
				chain.noJoints,
				float(chain.stretched_length),
				float(chain.rest_length)
				);
	return std::string(buffer, chars_printed);
}

// Builder::SemanticAnalysis
std::string format(const Builder::SemanticAnalysis& sa) {
	std::string result = "SemanticAnalysis:\n";

	if (sa.has_sharp_teeth) result += "  has_sharp_teeth: true\n";
	if (sa.has_claws) result += "  has_claws: true\n";
	if (sa.has_talons) result += "  has_talons: true\n";
	if (sa.has_venom) result += "  has_venom: true\n";
	if (sa.has_horns) result += "  has_horns: true\n";
	if (sa.has_weapons) result += "  has_weapons: true\n";
	if (sa.is_predator) result += "  is_predator: true\n";
	if (sa.has_hearing_organs) result += "  has_hearing_organs: true\n";
	if (sa.has_good_vision) result += "  has_good_vision: true\n";

	char buffer[128];
	auto chars_printed = snprintf(buffer, sizeof(buffer), "  eye_body_ratio: %f\n", sa.eye_body_ratio);
	result += std::string(buffer, chars_printed);

	if (sa.has_lateral_eyes) result += "  has_lateral_eyes: true\n";
	if (sa.has_forward_eyes) result += "  has_forward_eyes: true\n";
	if (sa.has_incisor_teeth) result += "  has_incisor_teeth: true\n";

	return result;
}

// Builder::Physical
std::string format(const Builder::Physical& p) {
	return std::format(
				"Physical:\n"
				"  body_length: {}\n"
				"  body_volume: {}\n"
				"  tail_length: {}\n"
				"  surface_area: {}\n"
				"  cross_section_area: {}\n"
				"  spine_root: {}\n"
				"  upright: {}\n"
				"  clade: {}\n"
				"  niche: {}\n",
				p.body_length,
				p.body_volume,
				p.tail_length,
				p.surface_area,
				p.cross_section_area,
				p.spine_root,
				p.upright ? "true" : "false",
				format(p.clade),
				format(p.niche)
				);
}

// Builder::Sensory::Vision::EyeInfo
std::string format(const Builder::Sensory::Vision::EyeInfo& eye) {
	char buffer[512];
	auto chars_printed = snprintf(buffer, sizeof(buffer),
				"    Eye(joint:%u pos:(%f,%f,%f) dir:(%f,%f,%f) on_stalk:%s diameter:%f)",
				eye.joint_index,
				float(eye.position.x), float(eye.position.y), float(eye.position.z),
				eye.pointing_direction.x, eye.pointing_direction.y, eye.pointing_direction.z,
				eye.is_on_stalk ? "yes" : "no",
				float(eye.eye_diameter_m)
				);
	return std::string(buffer, chars_printed);
}

// Builder::Sensory::Vision
std::string format(const Builder::Sensory::Vision& vision) {
	std::string result = "  Vision:\n";
	
	for (const auto& eye : vision.eyes) {
		result += format(eye) + "\n";
	}
	
	result += std::format(
				"    binocular_overlap: {}\n"
				"    centering: {}\n",
				vision.binocular_overlap,
				vision.centering
				);
	
	return result;
}

// Builder::Sensory::Hearing
std::string format(const Builder::Sensory::Hearing& hearing) {
	char buffer[256];
	auto chars_printed = snprintf(buffer, sizeof(buffer),
				"  Hearing:\n"
				"    ear_surface_area: %f\n"
				"    has_external_ears: %s\n",
				float(hearing.ear_surface_area),
				hearing.has_external_ears ? "true" : "false"
											);
	return std::string(buffer, chars_printed);
}

// Builder::Sensory::Antennae
std::string format(const Builder::Sensory::Antennae& antennae) {
	std::string result = "  Antennae:\n";
	
	for (const auto& chain : antennae.chains) {
		result += "    " + format(chain) + "\n";
	}
	
	result += std::format(
				"    is_sensory: {}\n"
				"    surface_area: {}\n",
				antennae.is_sensory ? "true" : "false",
				antennae.surface_area
				);
	
	return result;
}

// Builder::Sensory
std::string format(const Builder::Sensory& sensory) {
	std::string result = "Sensory:\n";
	
	if (sensory.has_snout) {
		result += std::format(
					"  has_snout: true\n"
					"  nasal_surface_area: {}\n",
					sensory.nasal_surface_area
					);
	}
	
	if (!sensory.antennae.chains.empty()) {
		result += format(sensory.antennae);
	}
	
	if (sensory.hearing.has_external_ears || static_cast<float>(sensory.hearing.ear_surface_area) > 0) {
		result += format(sensory.hearing);
	}
	
	if (!sensory.vision.eyes.empty()) {
		result += format(sensory.vision);
	}
	
	return result;
}

// Builder::Specialized::Digging
std::string format(const Builder::Specialized::Digging& digging) {
	std::string result = "  Digging:\n";
	
	if (digging.has_incisor_teeth) result += "    has_incisor_teeth: true\n";
	if (digging.has_digging_claws) result += "    has_digging_claws: true\n";
	if (digging.has_strong_forelimbs) result += "    has_strong_forelimbs: true\n";
	
	return result;
}

// Builder::Specialized
std::string format(const Builder::Specialized& specialized) {
	std::string result = "Specialized:\n";
	
	if (specialized.digging.has_incisor_teeth ||
			specialized.digging.has_digging_claws ||
			specialized.digging.has_strong_forelimbs) {
		result += format(specialized.digging);
	}
	
	return result;
}

// Builder_Appendage
std::string format(const Builder_Appendage& appendage) {
	std::string result = std::format(
				"  Appendage(id:{} root:{} tip:{} gait_group:{})\n",
				appendage.id,
				appendage.root,
				appendage.tip,
				appendage.gait_group
				);
	
	result += std::format(
				"    length: {} (stretched: {})\n"
				"    surface_area: {}\n"
				"    volume: {}\n"
				"    centroid: ({}, {}, {})\n",
				appendage.rest_length,
				appendage.stretched_length,
				appendage.surface_area,
				appendage.volume,
				appendage.centroid.x, appendage.centroid.y, appendage.centroid.z
				);
	
	result += std::format(
				"    cross_section (min/avg/max): {} / {} / {}\n"
				"    moment (min/avg/max): {} / {} / {}\n",
				appendage.minCrossSection,
				appendage.avgCrossSection,
				appendage.maxCrossSection,
				appendage.minMoment,
				appendage.avgMoment,
				appendage.maxMoment
				);
	
	result += std::format(
				"    semantic_flags: {}\n"
				"    clade_flags: {}\n"
				"    niche_flags: {}\n",
				format(appendage.semantic_flags),
				format(appendage.clade_flags),
				format(appendage.niche_flags)
				);
	
	if (appendage.contact.has_claws || appendage.contact.has_suckers ||
			appendage.contact.has_setae || appendage.contact.has_thumb ||
			appendage.contact.has_wet_grip) {
		result += "    contact:\n";
		if (appendage.contact.has_suckers) result += "      has_suckers: true\n";
		if (appendage.contact.has_setae) result += "      has_setae: true\n";
		if (appendage.contact.has_claws) result += "      has_claws: true\n";
		if (appendage.contact.has_thumb) result += "      has_thumb: true\n";
		if (appendage.contact.has_wet_grip) result += "      has_wet_grip: true\n";
		result += std::format("      area: {}\n", appendage.contact.area);
	}
	
	return result;
}

// Builder
std::string format(const Builder& builder) {
	std::string result = "=== TonTon Builder ===\n\n";
	
	result += format(builder.physical) + "\n";
	
	if (!builder.appendages.empty()) {
		result += "Appendages:\n";
		for (const auto& app : builder.appendages) {
			result += format(app);
		}
		result += "\n";
	}
	
	result += format(builder.sensory) + "\n";
	result += format(builder.semanticAnalyisis) + "\n";
	
	if (builder.specialized.digging.has_digging_claws ||
			builder.specialized.digging.has_incisor_teeth ||
			builder.specialized.digging.has_strong_forelimbs) {
		result += format(builder.specialized) + "\n";
	}
	
	if (builder.serpentine.noJoints > 0) {
		result += "Serpentine: " + format(builder.serpentine) + "\n";
	}
	
	result += "===================\n";
	return result;
}

// ============================================================================
// ENUM/FLAGS FORMATTERS
// ============================================================================

// Word
std::string format(const Word& w) {
	return std::string(WordToString(w));
}

// SemanticFlags
std::string format(const SemanticFlags& flags) {
	std::string result;
	bool first = true;
	
	for (uint32_t i = 0; i < 64; ++i) {
		auto flag = SemanticFlags(uint64_t(1) << uint64_t(i));
		
		if (HasFlag(flags, flag)) {
			if (!first) result += "|";
			result += WordToString(flag);
			first = false;
		}
	}
	
	return result.empty() ? "NONE" : result;
}

// CladeFlags
std::string format(const CladeFlags& flags) {
	std::string result;
	bool first = true;
	
	for (uint32_t i = 0; i < 64; ++i) {
		auto flag = CladeFlags(uint64_t(1) << uint64_t(i));
		
		if (HasFlag(flags, flag)) {
			if (!first) result += "|";
			result += WordToString(flag);
			first = false;
		}
	}
	
	return result.empty() ? "NONE" : result;
}

// NicheFlags
std::string format(const NicheFlags& flags) {
	std::string result;
	bool first = true;
	
	for (uint32_t i = 0; i < 64; ++i) {
		auto flag = NicheFlags(uint64_t(1) << uint64_t(i));
		
		if (HasFlag(flags, flag)) {
			if (!first) result += "|";
			result += WordToString(flag);
			first = false;
		}
	}
	
	return result.empty() ? "NONE" : result;
}

} // namespace TonTon
