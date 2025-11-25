#include "../include/tonton_input.h"
#include "tonton_builder.h"

namespace TonTon
{

mass_kg TonTon::Input::body_mass_kg() const
{
	if(builder)
	{
		return volume_scale()
		     * builder->physical.body_volume * body_density();
	}
	
	return volume_t(1) * volume_scale() * body_density();
}

force_N TonTon::Input::body_weight_N() const
{
	if(builder)
	{
		return volume_scale()
			  * builder->physical.body_volume 
			  * (body_density() - environment.fluidDensity_Kg_m3)
			  * environment.gravity_m_s2;
	}
	
	return volume_t(1) * volume_scale() * 
		   (body_density() - environment.fluidDensity_Kg_m3)
		  * environment.gravity_m_s2;
}

area_m2 TonTon::Input::cross_sectional_area_m2() const
{
	return (builder? builder->physical.cross_section_area : area_t(1.f)) * (scale*scale); 
}

glm::mat3 TonTon::Input::inertia_restPose() const
{
	std::array<float, 6>  covariance{1.f, 1.f, 1.f, 0.f, 0.f, 0.f};
	if(builder)
		covariance = builder->physical.covariance_restPose;
		
	auto inertia = builder?
		builder->physical.inertia_restPose()
		: glm::mat3(1);
	
	auto conv =  (scale*scale*scale*(scale*scale)) * body_density();
	
	return (inertia * (scale*scale*scale*(scale*scale))) * body_density();	
}

}

