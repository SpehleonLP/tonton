#include "../include/tonton_input.h"
#include "tonton_builder.h"

namespace TonTon
{

mass_kg TonTon::Input::body_mass_kg() const
{
	volume_m3 volume = builder?
		  scale_to<0>(builder->physical.body_volume, volume_scale())
		: volume_m3(0);
	
	return volume * body_density();
}

force_N TonTon::Input::body_weight_N() const
{
	volume_m3 volume = builder?
		  scale_to<0>(builder->physical.body_volume, volume_scale())
		: volume_m3(0);

	return volume
		  * (body_density() - environment.fluidDensity_Kg_m3)
		  * environment.gravity_m_s2;
}

area_m2 TonTon::Input::cross_sectional_area_m2() const
{
	return builder? scale_to<0>(builder->physical.cross_section_area, scale*scale) : 0.f; 
}

glm::mat3 TonTon::Input::inertia_restPose() const
{
	std::array<float, 6>  I{0.5f, 0.5f, 0.5f, 0.f, 0.f, 0.f};
	if(builder)
		I = builder->physical.covariance_restPose;
		
	float dens = float(body_density()) * float((scale*scale*scale*(scale*scale)));
	
	I = {
		(I[1]+I[2]) * dens,
		(I[0]+I[2]) * dens,
		(I[0]+I[1]) * dens,
		(I[3]) * dens,
		(I[4]) * dens,
		(I[5]) * dens,	
	};
	
	return glm::mat3{ 
		 I[1] +I[2],-I[3],-I[4],
		-I[3], I[0] +I[2],-I[5],
		-I[4],-I[5], I[0] +I[1] 
	};
}

}

