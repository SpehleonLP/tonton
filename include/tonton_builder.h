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

using position_b = glm::vec<3, length_b>;

// input is really heavy so we try to construct something faster.

struct Builder_Chain
{
	// stop ik chain here
	uint16_t root{};     
	// ik target
	uint16_t tip{};
	
	int16_t commonAncestor{};
	int16_t noJoints{};
	
	area_b    surface_area{};	
	volume_b  volume{};
	position_b centroid{0};
	
	length_b stretched_length{};
	length_b rest_length{};
	
	area_b minCrossSection{};
	area_b avgCrossSection{};
	area_b maxCrossSection{};
	length4_b minMoment{};
	length4_b avgMoment{};
	length4_b maxMoment{};
	
	void copy_into(Analysis_Chain &, length_b_to_m) const;
};

struct Builder_Tail;

struct Builder_Appendage;

struct Builder
{
	static counted_ptr<const Builder> Factory(
		SkinnedMesh const&,
		glm::vec3 body_scale = glm::vec3(1),
		std::span<const glm::vec3> bone_scales = {}	
	);
		
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
		bool has_incisor_teeth = false;
	};
	
	struct Physical {
		length_b body_length{};
		volume_b body_volume{};
		length_b tail_length{};
			
		// Body plan characteristics
		area_b surface_area{};
		area_b cross_section_area{};
		
		int16_t		spine_root{};
		bool		upright{};
		CladeFlags  clade{CladeFlags::NONE};
		NicheFlags  niche{NicheFlags::NONE};
	
		std::array<float, 6>  covariance_restPose{1.f, 1.f, 1.f, 0.f, 0.f, 0.f};
	};
	
	struct Sensory
	{
		bool has_snout{};
		bool has_external_ears{};
		
		area_b nasal_surface_area{};
		area_b ear_surface_area{};
	
		struct Antennae
		{
			immutable_array<Builder_Chain> chains;
			bool is_sensory{};
			area_b surface_area{};
		} antennae;
		
		struct Vision
		{
			struct EyeInfo {
				uint16_t joint_index{};
				bool is_on_stalk{};
				position_b position{0};           // In rest pose
				position_b base_position{0};      // Where eyestalk attaches (if applicable)
				glm::vec3 pointing_direction{0}; // Forward vector of this eye
				length_b stalk_length{0};
				length_b eye_diameter{0};
				angle_rad mobility_rad{0};           // How much can it rotate? (stalk vs fixed)
			};

			immutable_array<EyeInfo> eyes;
		
			angle_rad angular_separation_rad = 0.3f; // Assume some overlap
			float centering = 0.0f;
		} vision;
	};
	
	// if two immutable arrays in the builders are identical then 
	// the one with the lower refcount gets replaced by the one
	// with the higher ref count. 
	int Merge(const Builder&) const;
		
	SemanticAnalysis  semanticAnalyisis;	
	Physical physical;
	Sensory sensory;
	
	immutable_array<Builder_Tail> tails;
	std::optional<Builder_Chain> bodyWave;
	
// appendages found in model...
	immutable_array<Builder_Appendage> appendages;
	immutable_array<glm::vec3>		    gait_group_centers;
// list of gait groups, -1 to separate lists of groups. 
	immutable_array<int16_t>			ipsilateral_inhibition_groups;
	
	int siphon_joint = -1;
	
			
	
	void AddRef() const { ++_refCount; };
	void Release() const { if(--_refCount == 0) delete this; }
	
private:
	struct BuilderCommand;
	Builder(BuilderCommand&);

	mutable std::atomic<int> _refCount{1};
	
	template<typename T>
	static std::vector<Builder_Chain> GetChainsFromRoot(BuilderCommand const& in, T const& function);
	static std::vector<Builder_Chain> GetChainsFromRoot(BuilderCommand const& in, SemanticFlags flags, SemanticFlags child_flags);
	static std::vector<Builder_Chain> GetChainsFromRoot(BuilderCommand const& in, std::span<Word> words);
	static TonTon::Builder_Chain GetChain(TonTon::Builder::BuilderCommand const& in, int leaf, int root);
};


struct Builder_Appendage : public Builder_Chain
{
	void copy_into(Analysis_Appendage &, length_b_to_m) const;

	uint16_t gait_group{}; 
	uint16_t id{}; // index in appendage array in builder.
	length_b  distance_to_parent{};
	
	SemanticFlags semantic_flags{};
	CladeFlags clade_flags{};
	
	glm::vec3 rootAxis{};
	
	// projection of smallest eigenvector.
	struct Surface
	{
		area_b   area{};
		length_b chord{};
		glm::mat3 surface_matrix{};
		length5_b unit_inertia{}; // inertia at root/tangent axis.
	} surface;
	
	struct Contact
	{
		SemanticFlags subtree_flags{};
		
		int joint{};
		length_b rest_length{};
		length_b stretched_length{};
		
		area_b area{};
		glm::vec3 normal{};
				
		bool has_suckers : 1;
		bool has_setae : 1;
		bool has_claws : 1;
		bool has_thumb : 1;
		bool has_wet_grip : 1; // frog!	
	} contact;
	
	
	// AABB
	struct AABB
	{
		position_b min{0}, max{0};
	} aabb;
};

struct Builder_Tail : public Builder_Chain
{
	volume_b total_volume;
	
	using Flags = Analysis_Tail::Flags;
	Flags used_for{0};
	
	immutable_array<Builder_Tail> branches{};  // empty for single tail
    int venom_joint{}; // how?
	volume_b venom_joint_volume{};
	
	// we want to sort by chonkiest to least chonk.
	bool operator<(Builder_Tail const& it) const { return it.total_volume < total_volume; }
};


};

#endif // TONTON_BUILDER_H
