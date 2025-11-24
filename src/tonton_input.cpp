#include "../include/tonton_input.h"
#include "tonton_builder.h"

namespace TonTon
{

float TonTon::Input::body_mass_kg() const
{
	if(builder)
	{
		return volume_scale()
		     * builder->physical.body_volume_m3 * body_density();
	}
	
	return volume_scale() * body_density();
}

float TonTon::Input::body_weight_N() const
{
	if(builder)
	{
		return volume_scale()
			  * builder->physical.body_volume_m3 
			  * (body_density() - environment.fluidDensity_Kg_m3)
			  * environment.gravity_m_s2;
	}
	
	return volume_scale() * 
		   (body_density() - environment.fluidDensity_Kg_m3)
		  * environment.gravity_m_s2;
}

float TonTon::Input::cross_sectional_area_m2() const
{
	return (builder? builder->physical.cross_sectional_area_m2 : 1.f) * (scale*scale); 
}

glm::mat3 TonTon::Input::inertia_restPose() const
{
	auto inertia = builder?
		builder->physical.inertia_restPose()
		: glm::mat3(1);
	
	return inertia * (scale*scale*scale*scale*scale) * body_density();	
}

}
