#include "../include/tonton_output.h"


#include "Rules/tonton_scratch.h"
#include "../include/tonton_output.h"
#include "../include/tonton_input.h"

	
counted_ptr<const TonTon::Output> TonTon::Output::Factory(Input const& in)
{
	TonTon::Scratch s(in);
	
#define OPT_SIZE(x) (s.x.has_value()? sizeof(s.x.value()) : 0)
	size_t no_bytes = sizeof(TonTon::Output)
		+ OPT_SIZE(terrestrial)
		+ OPT_SIZE(serpentine)
		+ OPT_SIZE(aerial)
		+ OPT_SIZE(aquatic)
		+ OPT_SIZE(climbing)
		+ OPT_SIZE(brachiation)
		+ OPT_SIZE(jumping)
		+ OPT_SIZE(specialized.digging)
		+ OPT_SIZE(specialized.constriction)
		+ OPT_SIZE(sensory.hearing)
		+ OPT_SIZE(sensory.olfaction)
		+ OPT_SIZE(sensory.vision);
		
	std::span<uint8_t> heap(new uint8_t[no_bytes], no_bytes);
	counted_ptr<TonTon::Output> r = UncountedWrap(new(heap.data()) TonTon::Output());
	heap = heap.subspan(sizeof(TonTon::Output));
	
#undef OPT_SIZE
#define COPY_OPT(x) r->x.load(heap, s.x);
	COPY_OPT(terrestrial)
	COPY_OPT(serpentine)
	COPY_OPT(aerial)
	COPY_OPT(aquatic)
	COPY_OPT(climbing)
	COPY_OPT(brachiation)
	COPY_OPT(jumping)
	COPY_OPT(specialized.digging)
	COPY_OPT(specialized.constriction)
	COPY_OPT(sensory.hearing)
	COPY_OPT(sensory.olfaction)
	COPY_OPT(sensory.vision);
#undef COPY_OPT

	r->physical = s.physical;
	r->metabolic = s.metabolic;
	r->behavior = s.behavior;
	
	r->appendages.manipulation = s.appendages.manipulation;
	r->appendages.tails = s.appendages.tails;
	r->diagnostics = std::move(s.diagnostics);
	
	return r;
}
	

// Steady glide: Lift = Weight
float TonTon::Output_Aerial::gliding_CL(float weight_N, float speed_m_s, float air_density) const {
	float dynamic_pressure = 0.5f * air_density * speed_m_s * speed_m_s;
	return weight_N / (dynamic_pressure * wing_area_m2);
}

// Simplified flapping CL estimation
float TonTon::Output_Aerial::flapping_CL_effective(float weight_N,
							float forward_speed_m_s, 
							float wingbeat_freq_Hz,
							float beat_amplitude_rad,
							float air_density) const {
	// Wing tip velocity from flapping
	float tip_velocity = wing_span_m * beat_amplitude_rad * wingbeat_freq_Hz * 2.0f * M_PI;
	
	// Effective velocity combines forward and flapping
	float effective_velocity = sqrt(forward_speed_m_s * forward_speed_m_s + 
								   tip_velocity * tip_velocity);
	
	// During power stroke, CL can be much higher (1.5-2.5)
	// averaged over full cycle, use similar to gliding
	float dynamic_pressure = 0.5f * air_density * effective_velocity * effective_velocity;
	return weight_N / (dynamic_pressure * wing_area_m2);
}

float TonTon::Output_Aerial::hovering_disk_loading_N_m2(float weight_N) const {
	// Total disk area (both wings sweep a circle)
	float disk_area = M_PI * wing_span_m * wing_span_m;
	return weight_N / disk_area;  // N/m²
}

// Momentum theory for hovering
float TonTon::Output_Aerial::hovering_power_ideal_W(float weight_N, float air_density_kg_m3) const {	
	// Induced velocity needed to hover
	float v_induced = sqrt(hovering_disk_loading_N_m2(weight_N) / (2.0f * air_density_kg_m3));
	
	// Ideal power (real power is ~1.2-1.5x this due to profile drag)
	// N * sqrt(N/m2 / Kg_m3)
	// N * sqrt(N/m2 * m3 / kg)
	// N * sqrt(N/1 * m / kg)
	// N * sqrt(kg * m / s ^2 * m / kg)
	// N * sqrt(m^2 / s ^2)
	return weight_N * v_induced;
}
	
