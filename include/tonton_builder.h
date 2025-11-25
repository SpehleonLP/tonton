#ifndef TONTON_BUILDER_H
#define TONTON_BUILDER_H
#include "tonton_counted_ptr.hpp"
#include "tonton_analysis.h"
#include "tonton_shared_array.hpp"
#include "tonton_skinnedmesh.h"
#include "tonton_units.hpp"
#include <glm/vec3.hpp>

namespace TonTon
{

struct SkinnedMesh;
struct Analysis_Appendage;

// input is really heavy so we try to construct something faster.

struct BuilderCommand
{
	counted_ptr<const SkinnedMesh> skinnedMesh;
	glm::vec3 body_scale{1};
	immutable_array<glm::vec3> bone_scales;
};

struct Builder_Appendage : public Analysis_Appendage
{
using position_t = glm::vec<3, length_t>;
	SemanticFlags semantic_flags{};
	CladeFlags clade_flags{};
	NicheFlags niche_flags{};
	
	area_t    area{};	
	volume_t  volume{};
	position_t centroid;
	
	area_t minCrossSection_m2{};
	area_t maxCrossSection_m2{};
	length4_t minMoment_m2{};
	length4_t maxMoment_m2{};
	
	glm::vec3 rootAxis{};
	
	// projection of smallest eigenvector.
	struct Surface
	{
		area_t   area_m2{};
		length_t chord_m{};
		glm::vec3 normal{};
	} surface;
	
	struct Contact
	{
		SemanticFlags subtree_flags{};
		
		int joint{};
		length_t rest_length_m{};
		length_t stretched_length_m{};
		
		area_t area_m2{};
		glm::vec3 normal{};
				
		bool has_suckers : 1;
		bool has_setae : 1;
		bool has_claws : 1;
		bool has_thumb : 1;
		bool has_wet_grip : 1; // frog!	
	} contact;
	
	length_t  distance_to_parent_m{};
	length5_t unit_inertia_m5{}; // inertia at root/tangent axis.
	
	// AABB
	Cube aabb{};
};

struct Builder
{
	static counted_ptr<const Builder> Factory(
		counted_ptr<SkinnedMesh> const&,
		glm::vec3 body_scale = glm::vec3(1),
		immutable_array<glm::vec3> bone_scales = {}	
	);
	
	// if two immutable arrays in the builders are identical then 
	// the one with the lower refcount gets replaced by the one
	// with the higher ref count. 
	int Merge(const Builder&) const;
	
// appendages found in model...
	immutable_array<Builder_Appendage> appendages;
// list of gait groups, -1 to separate lists of groups. 
	immutable_array<glm::vec3>		    gait_group_centers;
	immutable_array<int16_t>			ipsilateral_inhibition_groups;
	
	int siphon_joint = -1;
		
	struct SemanticAnalysis {
		bool has_sharp_teeth = false;
		bool has_claws = false;
		bool has_talons = false;
		bool has_venom = false;
		bool has_horns = false;
		bool has_weapons = false;
		bool is_predator = false;
		bool has_hearing_organs = false;
		bool has_good_vision = false;
		float eye_body_ratio = 0.1f;
		bool has_lateral_eyes = false;
		bool has_forward_eyes = false;
	} semanticAnalyisis;
	
	struct Physical {
		length_t body_length{};
		volume_t body_volume{};
		length_t tail_length{};
			
		// Body plan characteristics
		area_t surface_area{};
		area_t cross_section_area{};
		
		int16_t		spine_root{};
		bool		upright{};
		CladeFlags  clade{CladeFlags::NONE};
		NicheFlags  niche{NicheFlags::NONE};
	
		std::array<float, 6>  covariance_restPose{1.f, 1.f, 1.f, 0.f, 0.f, 0.f};
	};
	
	Physical physical;
	
};


};

#endif // TONTON_BUILDER_H
