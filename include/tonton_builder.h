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

struct BuilderCommand
{
	counted_ptr<const SkinnedMesh> skinnedMesh;
	glm::vec3 body_scale{1};
	immutable_array<glm::vec3> bone_scales;
};

struct Builder_Chain
{
	// stop ik chain here
	uint16_t root{};     
	// ik target
	uint16_t tip{};
	
	int noJoints{};
	
	length_b stretched_length{};
	length_b rest_length{};
	
	void copy_into(Analysis_Chain &, length_b_to_m) const;
};

struct Builder_Appendage : public Builder_Chain
{
	void copy_into(Analysis_Appendage &, length_b_to_m) const;

	int16_t common_ancestor{}; 
	uint16_t gait_group{}; 
	uint16_t id{}; // index in appendage array in builder.
	float   phase_offset{};
	
	SemanticFlags semantic_flags{};
	CladeFlags clade_flags{};
	NicheFlags niche_flags{};
	
	area_b    surface_area{};	
	volume_b  volume{};
	position_b centroid;
	
	area_b minCrossSection{};
	area_b maxCrossSection{};
	length4_b minMoment{};
	length4_b maxMoment{};
	
	glm::vec3 rootAxis{};
	
	// projection of smallest eigenvector.
	struct Surface
	{
		area_b   area{};
		length_b chord{};
		glm::vec3 normal{};
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
	
	length_b  distance_to_parent{};
	length5_b unit_inertia{}; // inertia at root/tangent axis.
	
	// AABB
	struct AABB
	{
		position_b min, max;
	} aabb;
};

struct Builder
{
	static counted_ptr<const Builder> Factory(
		SkinnedMesh const&,
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
		bool has_incisor_teeth = false;
	} semanticAnalyisis;
	
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
	
	Physical physical;
	
	/*
	 * 
    // Scan for vertebrate olfactory structures
    for(uint32_t i = 0; i < sk.skin->names.size(); ++i) {
        for(auto word : sk.skin->tags[i]) {
            if(word == Word::snout || word == Word::nose || word == Word::nostril) {
                has_snout = true;
                nasal_surface_area += sk.surfaceArea[i] * in.behavior.area_scale();
                break;
            }
        }
    }*/
    
	struct Sensory
	{
		bool has_snout{};
		area_b nasal_surface_area{};
	
		struct Antennae
		{
			immutable_array<Builder_Chain> chains;
			bool is_sensory{};
			area_b surface_area{};
		} antennae;
		
		/*
		 * 
    for(uint32_t i = 0; i < in.skinnedMesh->skin->names.size(); ++i) {
        for(auto word : in.skinnedMesh->skin->tags[i]) {
            if(word == Word::ear || word == Word::pinna) {
                has_external_ears = true;
                ear_surface_area += in.skinnedMesh->surfaceArea[i] * in.behavior.area_scale();
                break;
            }
        }
    }
        */
        
		struct Hearing
		{
			area_b ear_surface_area{};
			bool has_external_ears{};
		} hearing;
		
		struct Vision
		{
			struct EyeInfo {
				uint16_t joint_index;
				position_b position;           // In rest pose
				position_b base_position;      // Where eyestalk attaches (if applicable)
				glm::vec3 pointing_direction; // Forward vector of this eye
				bool is_on_stalk;
				length_m stalk_length_m;
				length_m eye_diameter_m;
				angle_rad mobility_rad;           // How much can it rotate? (stalk vs fixed)
			};

			immutable_array<EyeInfo> eyes;
		
			float binocular_overlap = 0.3f; // Assume some overlap
			float centering = 0.0f;
		} vision;
		
		/*
		 * 
    if(eyes.size() >= 2) {
        // Find the two most "forward-facing" eyes for binocular vision
        // (The ones with the most similar pointing directions)
        
        int best_pair[2] = {0, 1};
        auto best_alignment = -1.0f;
        
        for(size_t i = 0; i < eyes.size(); ++i) {
            for(size_t j = i + 1; j < eyes.size(); ++j) {
                auto alignment = glm::dot(eyes[i].pointing_direction, 
                                          eyes[j].pointing_direction);
                if(alignment > best_alignment) {
                    best_alignment = alignment;
                    best_pair[0] = i;
                    best_pair[1] = j;
                }
            }
        }
        
        auto & eye_A = eyes[best_pair[0]];
        auto & eye_B = eyes[best_pair[1]];
        
        // Get their common ancestor
        int common_root = gcr_table[eye_A.joint_index * N + eye_B.joint_index];
        
        glm::vec3 common_pos = in.position(common_root);
        
        // Vectors from common root to each eye
        glm::vec3 to_A = eye_A.position - common_pos;
        glm::vec3 to_B = eye_B.position - common_pos;
        
        // Plane normal (perpendicular to both eye vectors)
        glm::vec3 plane_normal = glm::normalize(glm::cross(to_A, to_B));
        
        // Forward direction (average of eye pointing directions)
        glm::vec3 forward = glm::normalize(eye_A.pointing_direction + 
                                          eye_B.pointing_direction);
        
        // Project eye positions onto the plane perpendicular to forward
        glm::vec3 right = glm::normalize(glm::cross(forward, plane_normal));
        
        // Angular separation between eyes (in the horizontal plane)
        auto angle_A = std::atan2(glm::dot(to_A, right), glm::dot(to_A, forward));
        auto angle_B = std::atan2(glm::dot(to_B, right), glm::dot(to_B, forward));
        
        auto angular_separation_rad = std::abs(angle_B - angle_A);
        
        // Typical FOV per eye: ~90-180 degrees depending on eye type
        // Predators: narrow FOV (~60°), Prey: wide FOV (~180°)
        auto fov_per_eye_rad = glm::mix(glm::radians(60.0f), 
                                     glm::radians(150.0f),
                                     1.0f - in.behavior.aggression_adjustment);
        
        // Binocular overlap = (2 × FOV - separation) / FOV
        // If eyes are close together and pointing same direction: high overlap
        // If eyes are on sides of head: low/no overlap
        auto overlap_angle = 2.0f * fov_per_eye_rad - angular_separation_rad;
        vision.binocular_overlap = std::clamp(overlap_angle / fov_per_eye_rad, 0.0f, 1.0f);
        
        // ===============================================================
        // CENTERING - for asymmetric creatures (flatfish!)
        // ===============================================================
        
        // Check if eyes are on opposite sides (normal) or same side (flatfish)
        auto left_right_balance = (angle_A + angle_B) / 2.0f;
        
        // If both eyes are on left (negative) or both on right (positive): asymmetric
        vision.centering = std::clamp(left_right_balance / glm::radians(90.0f), -1.0f, 1.0f);
        */
        
		/*
    if(!antennae.empty()) {
        for(auto const& antenna : antennae) {
            // Get all joints in the antenna chain
            auto relevant_joints = sk.skin->memo()->GetAllChildrenOfRoot(antenna.root);
            
            // Check if antenna is tagged as sensory
            bool is_sensory = false;
            for(auto joint : relevant_joints) {
                for(auto word : sk.skin->tags[joint]) {
                    if(word == Word::sensory || word == Word::chemoreceptor) {
                        is_sensory = true;
                        break;
                    }
                }
                if(is_sensory) break;
            }
            
            if(is_sensory || antenna.stretched_length_m > 0) {
                has_sensory_antennae = true;
                
                // Estimate surface area of antenna
                for(auto joint : relevant_joints) {
                    antennal_surface_area += sk.surfaceArea[joint] * in.area_scale();
                }
            }
        }
    }*/
	} sensory;
	
	struct Specialized
	{
		struct Digging
		{
			bool has_incisor_teeth{};
			bool has_digging_claws{};
			bool has_strong_forelimbs{};
		} digging;
	
	} specialized;
	
};


};

#endif // TONTON_BUILDER_H
