#ifndef TONTON_BUILDER_H
#define TONTON_BUILDER_H
#include "tonton_counted_ptr.hpp"
#include "tonton_analysis.h"
#include "tonton_shared_array.hpp"
#include "tonton_skinnedmesh.h"
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
	SemanticFlags semantic_flags{};
	CladeFlags clade_flags{};
	NicheFlags niche_flags{};
	
	float     area_m2{};	
	float     volume_m3{};
	glm::vec3 centroid{};
	
	float minCrossSection_m2{};
	float maxCrossSection_m2{};
	float minMoment_m2{};
	float maxMoment_m2{};
	
	glm::vec3 rootAxis{};
	
	// projection of smallest eigenvector.
	struct Surface
	{
		float area_m2{};
		float chord_m{};
		glm::vec3 normal{};
	} surface;
	
	struct Contact
	{
		SemanticFlags subtree_flags{};
		
		int joint{};
		float rest_length_m{};
		float stretched_length_m{};
		
		float area_m2{};
		glm::vec3 normal{};
				
		bool has_suckers : 1;
		bool has_setae : 1;
		bool has_claws : 1;
		bool has_thumb : 1;
		bool has_wet_grip : 1; // frog!	
	} contact;
	
	float distance_to_parent_m{};
	float contactArea_m2{};
	float unit_inertia_m5{}; // inertia at root/tangent axis.
	
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
	
	Analysis_Physical physical;
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
	
	
};


};

#endif // TONTON_BUILDER_H
