#include "../include/tonton_builder.h"
#include "../include/tonton_rigsolver.h"
#include "Memos/tonton_armaturememo.h"
#include "Memos/tonton_skinnedmeshmemo.h"
#include "dodeedum.h"
#include "tonton_skinnedmesh.h"
#include <functional>
#include <cfloat>

using SF = TonTon::SemanticFlags;

struct TonTon::Builder::BuilderCommand
{
	BuilderCommand(SkinnedMesh const&,
	glm::vec3 body_scale,
	std::span<const BuilderScale> bone_scales,
	const Articulations& articulations);

	SkinnedMesh const& skinnedMesh;
	const Articulations& articulations;
	
	// should be pose * invbindpose
	shared_array<glm::mat4> transforms;
	shared_array<float>     area_scale; 
	shared_array<float>     volume_scale;
	
	inline glm::vec3 positions(int i) const { return transforms[i] * glm::vec4(skinnedMesh.skin->position[i], 1); };
	inline glm::vec3 bone_tails(int i) const { return transforms[i] * glm::vec4(_bone_tails[i], 1); };
	inline glm::vec3 centroid(int i) const { return transforms[i] * glm::vec4(skinnedMesh.centroid[i], 1); }
	
	SemanticAnalysis GetSemanticAnalysis(Builder&) const;
	Physical GetPhysicalAnalysis(Builder&);
	Sensory GetSensory(Builder&);	
	
	std::vector<Builder_Tail> GetTails(Builder&);
	std::optional<Builder_Chain> GetBodyWave(Builder&);
	
	std::vector<Builder_Appendage> GetAppendages(Builder&);
	std::vector<glm::vec3> GetGaitGroupCenters(Builder&);
//	immutable_array<int16_t> GetInhibitionGroups(Builder&);
	
	int32_t GetSiphonJoint(Builder&) const;
	
private:
	immutable_array<glm::vec3> _bone_tails;	
};

counted_ptr<const TonTon::Builder> TonTon::Builder::Factory(
	counted_ptr<const SkinnedMesh> skinnedMesh,
	glm::vec3 body_scale,
	std::span<const BuilderScale> bone_scales,
	Articulations articulations
)
{
	if(skinnedMesh == nullptr
	|| skinnedMesh->mesh == nullptr
	|| skinnedMesh->skin == nullptr)
		throw std::invalid_argument("skinnedMesh");

	if(bone_scales.size() != 0 && bone_scales.size() != skinnedMesh->skin->position.size())
		throw std::invalid_argument("bone_scales");

	if(!articulations.empty() && articulations.joints.size() != skinnedMesh->skin->position.size())
		throw std::invalid_argument("articulations");

	BuilderCommand cmd(*skinnedMesh, body_scale, bone_scales, articulations);
	auto builder = UncountedWrap(new Builder(cmd));
	const_cast<Builder*>(builder.get())->articulations = std::move(articulations);
	return builder;
};

TonTon::Builder::BuilderCommand::BuilderCommand(
	SkinnedMesh const& skinnedMesh,
	glm::vec3 body_scale,
	std::span<const BuilderScale> bone_scales,
	const Articulations& articulations) :
	skinnedMesh(skinnedMesh),
	articulations(articulations),
	_bone_tails(skinnedMesh->GetBoneTails())
{
	auto skin = skinnedMesh.skin;
	
	// Build transforms array
	transforms = shared_array<glm::mat4>(skinnedMesh.skin->position.size());
	
	// Step 1: Reconstruct bind pose matrices (invert the inverse!)
	std::vector<glm::mat4> bindPose(skin->inverseBindPoseMatrices.size());
	for(auto i = 0u; i < bindPose.size(); ++i)
	{
		bindPose[i] = glm::inverse(skin->inverseBindPoseMatrices[i]);
	}
	
	// Step 2: Apply bone scales hierarchically
	if(!bone_scales.empty())
	{
		for(auto i = 0u; i < bindPose.size(); ++i)
		{
			auto& bs = bone_scales[i];
			glm::vec3 scale_vec(
				bs.scale * bs.chonk,
				bs.scale * bs.stretch,
				bs.scale * bs.chonk
			);
			
			glm::mat4 scale_matrix = glm::scale(glm::mat4(1.0f), scale_vec);
			
			// Apply scale to this bone's transform
			bindPose[i] = bindPose[i] * scale_matrix;
			
			// Move all children to compensate for this bone's scale
			for(auto j = 0u; j < bindPose.size(); ++j)
			{
				if(skin->parents[j] == i)
				{
					// Child's position needs to be scaled relative to parent
					bindPose[j][3] = bindPose[i] * glm::inverse(bindPose[skin->parents[i] >= 0 ? skin->parents[i] : i]) * bindPose[j][3];
				}
			}
		}
	}
	
	// Step 3: Apply body scale globally
	transforms = shared_array<glm::mat4>(bindPose.size());
	for(auto i = 0u; i < transforms.size(); ++i)
	{
		glm::mat4 scaled_pose = glm::scale(glm::mat4(1.0f), body_scale) * bindPose[i];
		
		// transforms = pose * invBindPose
		transforms[i] = scaled_pose * skin->inverseBindPoseMatrices[i];
	}
		
	area_scale = shared_array<float>(transforms.size(), 0); 
	volume_scale = shared_array<float>(transforms.size(), 0); 
	
	for(auto i = 0u; i < transforms.size(); ++i)
	{
		volume_scale[i] = glm::determinant(glm::mat3(transforms[i]));
		area_scale[i] = std::cbrt(volume_scale[i]);
		area_scale[i] *= area_scale[i];
	}
}


TonTon::Builder::Builder(BuilderCommand & cmd) :
	semanticAnalyisis(cmd.GetSemanticAnalysis(*this)),
	physical(cmd.GetPhysicalAnalysis(*this)),
	sensory(cmd.GetSensory(*this)),	
	
	tails(shared_array<Builder_Tail>::FromArray(cmd.GetTails(*this))),
	bodyWave(cmd.GetBodyWave(*this)),
	
	appendages(shared_array<Builder_Appendage>::FromArray(cmd.GetAppendages(*this))),
	gait_group_centers(shared_array<glm::vec3>::FromArray(cmd.GetGaitGroupCenters(*this))),
//	ipsilateral_inhibition_groups(cmd.GetInhibitionGroups(*this)),
	
	siphon_joint(cmd.GetSiphonJoint(*this))
{
}

void TonTon::Builder_Chain::copy_into(Analysis_Chain & dst, length_b_to_m scale) const
{
	dst.root = root;
	dst.tip = tip;
	dst.noJoints = noJoints;
	dst.rest_length_m = scale_to<0>(rest_length, scale);
	dst.stretched_length_m = scale_to<0>(stretched_length, scale);
}

void TonTon::Builder_Appendage::copy_into(Analysis_Appendage & dst, length_b_to_m scale) const
{
	Builder_Chain::copy_into(dst, scale);
		
	dst.common_ancestor = commonAncestor;
	dst.gait_group = gait_group;
	dst.id = id;
}

TonTon::Builder::SemanticAnalysis TonTon::Builder::BuilderCommand::GetSemanticAnalysis(Builder&) const {
    SemanticAnalysis result;
    
    auto semantic_flags = skinnedMesh.skin->memo()->GetSemanticFlags();
    auto tags = skinnedMesh.skin->tags;
    
    // Scan all bones for relevant semantic flags
    for (size_t i = 0; i < semantic_flags.size(); ++i) {
        auto flags = semantic_flags[i];
        
        // Dental weapons
        if (HasFlag(flags, SemanticFlags::TEETH)) {
            // Check for predatory teeth types in tags
            for (auto word : tags[i]) {
                if (word == Word::fang || word == Word::canine || word == Word::incisor
                || word == Word::carnivore || word == Word::predator) {
                    result.has_sharp_teeth = true;
                    result.has_weapons = true;
                }
                if (word == Word::incisor) {
                    result.has_sharp_teeth = true;
                    result.has_incisor_teeth = true;
                }
            }
        }
        
        // Claw weapons
        if (HasFlag(flags, SemanticFlags::NAIL)) {
            for (auto word : tags[i]) {
                if (word == Word::claw || word == Word::talon) {
                    result.has_claws = true;
                    result.has_weapons = true;
                    if (word == Word::talon) {
                        result.has_talons = true;
                    }
                }
            }
        }
        
        // Horns/antlers
        if (HasFlag(flags, SemanticFlags::HORN_ANTLER)) {
            result.has_horns = true;
            result.has_weapons = true;
        }
        
        // Sensory organs
        if (HasFlag(flags, SemanticFlags::HEARING)) {
            result.has_hearing_organs = true;
        }
        
        if (HasFlag(flags, SemanticFlags::VISION)) {
            result.has_good_vision = true;
            
            // Determine eye position from flags
            bool is_lateral = HasFlag(flags, SemanticFlags::LATERAL);
            bool is_anterior = HasFlag(flags, SemanticFlags::ANTERIOR);
            
            if (is_lateral) {
                result.has_lateral_eyes = true;
            }
            if (is_anterior) {
                result.has_forward_eyes = true;
            }
        }
    }
    
	for(auto i = 0u; i < skinnedMesh.skin->tags.size(); ++i)
	{
		for(auto tag : skinnedMesh.skin->tags[i])
			if(tag == Word::stinger
			|| tag == Word::venom)
			{
				result.has_venom = true;
				result.has_weapons = true;
			}
	}
    
    // Determine if predator from morphology
    result.is_predator = HasFlag(skinnedMesh.skin->memo()->GetNicheFlags(), NicheFlags::PREDATOR|NicheFlags::CARNIVORE);
    
    float eye_volume = 0.0;
    float body_volume = 0.0;
    
	for(auto i = 0u; i < skinnedMesh.skin->tags.size(); ++i)
	{			
		float bone_scale = volume_scale[i];
		float volume = skinnedMesh.volume[i] * bone_scale;
		
		body_volume += volume;
		if(HasFlag(semantic_flags[i], SemanticFlags::VISION))
		{
			eye_volume+= volume;
		}
	}
	
    result.eye_body_ratio = eye_volume / body_volume;
    
    return result;
}

TonTon::Builder::Physical TonTon::Builder::BuilderCommand::GetPhysicalAnalysis(Builder&)
{
	auto & sk = skinnedMesh;
	auto * sk_memo = sk.skin->memo();
	
	auto parents = sk.skin->parents.data();
	
	float volume{};
	float surfaceArea{};
	
	for(auto i = 0u; i < sk.volume.size(); ++i)
	{
		volume += sk.volume[i] * volume_scale[i];
		surfaceArea += sk.surfaceArea[i] * area_scale[i]; 
	}
	
	auto relative_flags = sk_memo->GetRelativeFlags();
	auto children		= sk_memo->GetChildren();
	auto semantic_flags = sk_memo->GetSemanticFlags();
	auto dfs_ordering   = sk_memo->GetDfsOrdering();
	auto gcr_table      = sk_memo->GetGcrTable();
	auto leaves         = sk_memo->GetLeaves();
	auto names			= sk.skin->names.data();
	auto root = dfs_ordering[0];
	(void)names;
	
	SF SPINE_FLAGS = SF::SPINE | SF::ABDOMEN;
	SF NOT_SPINE_FLAGS = SF::LIMB | SF::HEAD | SF::NECK | SF::TAIL | SF::DIGIT
					   | SF::FACIAL | SF::TEETH | SF::NAIL | SF::HORN_ANTLER
					   | SF::WING | SF::FIN | SF::TENTACLE | SF::MOUTH_PARTS;


	const uint32_t noJoints = dfs_ordering.size();
	float max_tail_length = 0;
	float max_body_length = 0;
	float crossSectionArea = 0;
	double tail_to_root_dist = 0;
	bool is_upright = false;
	int spine_root = 0;
	
	auto ProcessHead = [&](const uint32_t begin)
	{
		const auto root = dfs_ordering[begin];
		Cube aabb = sk.aabb[root];
		
		for(auto idx = begin; idx < dfs_ordering.size(); ++idx)
		{
			auto joint = dfs_ordering[idx];
			if(gcr_table[root*noJoints + joint] != root)
				break;
		
			if(HasFlag(semantic_flags[joint], SF::FACIAL|SF::HEAD))
				aabb = aabb | sk.aabb[joint];
		}
				
		double snout_to_base_of_skull = glm::distance(
			glm::vec3(
				(aabb.min.x+aabb.max.x)/2.f, 
				(aabb.min.y+aabb.max.y)/2.f,
				(aabb.max.z)), 
			positions(root));
		double spine_length = 0;
		bool is_this_upright = false;
		bool in_spine = false;
		int this_spine_root{};
	
	// walk down spine
		for(int joint = root; joint >= 0; joint = parents[joint])
		{
			glm::vec3 delta = bone_tails(joint) - positions(joint);
			
			if(root == joint)
			{
				float projection = glm::dot(delta, glm::vec3(0, 1, 0));
				is_this_upright = (projection > 0.707);					
			}
			
			if(HasFlag(semantic_flags[joint], SPINE_FLAGS))
			{
				spine_length += glm::length(delta);
				this_spine_root = joint;
				in_spine = true;
			}
			else if(in_spine)
				break;
		}
		
		// walk up spine	
		bool shared_by_head = true;	
		for(auto  joint = this_spine_root; joint != -1; )
		{
			int candidate = -1;
			bool candidate_shared_by_head = false;
			
			for(auto child : children[joint])
			{
				auto flags = semantic_flags[child]|relative_flags[child].child_flags;
				
				if( HasFlag(flags, SPINE_FLAGS)
				&& !HasFlag(semantic_flags[child], NOT_SPINE_FLAGS))
				{
					bool child_shared_by_head = (gcr_table[root*noJoints + child] == child);
				
					if((child_shared_by_head == true && candidate == -1)
					|| (child_shared_by_head == false))
					{
						candidate = child;
						candidate_shared_by_head = child_shared_by_head;
					}
				}
			}
			
			glm::vec3 delta = glm::vec3(0);
			
			if(candidate == -1)
				delta = positions(joint) - bone_tails(joint);
			else if(candidate_shared_by_head == false)
			{
				if(in_spine == false)
					delta = positions(root) - positions(candidate);
				else
					delta = positions(joint) - positions(candidate);
			}
			
			joint = candidate;			
			shared_by_head = candidate_shared_by_head;
			spine_length += glm::length(delta);	
		}
	
		double SVL = snout_to_base_of_skull + spine_length;
		
		if(SVL > max_body_length)
		{
			max_body_length = SVL;
			is_upright = is_this_upright;
			spine_root = this_spine_root;
		}
	};
	
	auto ProcessTail = [&](uint32_t begin)
	{
		auto root = dfs_ordering[begin];
		
		double tail_length = 0;
		
		for(auto leaf : leaves)
		{
			if(gcr_table[root*noJoints + leaf] != root)
				continue;
				
			double length = 0;
			
			for(int joint = leaf; joint > root; joint = parents[joint])
			{
				auto p = parents[joint];
				
				if(p >= 0)
				{			
					length += glm::distance(positions(joint), positions(p));
				}
			}
			
			tail_length = std::max(tail_length, length);
		}
		
		if(tail_length < max_tail_length)
			return;
			
		max_tail_length = tail_length;
		tail_length = 0;
			
		for(int joint = root; joint >= 0; joint = parents[joint])
		{
			auto p = parents[joint];
			
			if(p >= 0)
			{	
				tail_length += glm::distance(positions(joint), positions(p));	
			}
			
			if(HasFlag(semantic_flags[joint], SPINE_FLAGS))
			{
				tail_to_root_dist = tail_length;
				break;
			}
		}
	};
		
	for(auto i = 0u; i < dfs_ordering.size(); ++i)
	{
		auto node = dfs_ordering[i];
		
		if( HasFlag(semantic_flags[node], SF::TAIL)
		&& !HasFlag(relative_flags[node].parent_flags, SF::TAIL))
		{
			ProcessTail(i);
		}
		if(HasFlag(semantic_flags[node], SF::HEAD)
		&& !HasFlag(relative_flags[node].parent_flags, SF::HEAD))
		{
			ProcessHead(i);
		}
		
		if(HasFlag(semantic_flags[node], SPINE_FLAGS))
		{
			auto relevant_nodes = sk_memo->GetAllChildren(node, SPINE_FLAGS, NOT_SPINE_FLAGS);
				
			glm::vec3 vec = bone_tails(node) - positions(node);
			for(auto child : children[node])
			{
				if(HasFlag(relative_flags[child].child_flags|semantic_flags[child], SF::HEAD))
				{
					vec = positions(child) - positions(node);
					break;
				}
			}	
				
			auto area =	skinnedMesh.EstimateCrossSection(relevant_nodes, transforms, vec);	
			crossSectionArea = std::max<double>(crossSectionArea, area);
		}		
	}
	
	// failed to find body, try treating front of spine as head?
	if(max_body_length == 0.0)
	{
		for(auto i = 0u; i < dfs_ordering.size(); ++i)
		{
			auto node = dfs_ordering[i];
			
			if(HasFlag(semantic_flags[node], SF::SPINE)
			&& !HasFlag(relative_flags[node].child_flags, SF::SPINE))
			{
				ProcessHead(i);
			}
		}
	}
	
	if(max_body_length == 0.0)
	{
		throw std::runtime_error("unable to find head of model.");
	}
	
	max_body_length += tail_to_root_dist;
	
	auto C = skinnedMesh.GetCovariance(std::span<uint16_t>{}, transforms);
	auto bodyCentroid = skinnedMesh.GetCentroid(std::span<uint16_t>{}, transforms);

	return {
		.body_length=max_body_length+max_tail_length,
		.body_volume=volume,
		.tail_length=max_tail_length,

		.surface_area=surfaceArea,
		.cross_section_area=crossSectionArea,
		.body_centroid=glm::vec3(bodyCentroid),
		.spine_root=static_cast<int16_t>(spine_root),
		.upright=is_upright,
		.clade=(sk_memo->GetCladeFlags()[root]|sk_memo->GetRelativeCladeFlags()[root].child_flags),
		.covariance_restPose=std::array<float, 6>{
			float(C[0]),
			float(C[1]),
			float(C[2]),
			float(C[3]),
			float(C[4]),
			float(C[5]),
		}
	};
}


TonTon::Builder::Sensory TonTon::Builder::BuilderCommand::GetSensory(Builder&)
{
	TonTon::Builder::Sensory r;
	
   for(uint32_t i = 0; i < skinnedMesh.skin->tags.size(); ++i) {
        for(auto word : skinnedMesh.skin->tags[i]) {
            if(word == Word::ear || word == Word::pinna) {
                r.has_external_ears = true;
                r.ear_surface_area += skinnedMesh.surfaceArea[i] * area_scale[i];
                break;
            }
            
            if(word == Word::nose 
            || word == Word::nostril 
            || word == Word::snout) {
                r.has_snout = true;
                r.nasal_surface_area += skinnedMesh.surfaceArea[i] * area_scale[i];
                break;
            }
        }
    }

	std::array<Word, 1> words = {Word::antenna}; 
	auto chains = GetChainsFromRoot(*this, std::span<Word>{words});
	
	r.antennae.chains = shared_array<Builder_Chain>::FromArray(chains);
	
	for(auto const& antenna : r.antennae.chains) {
		// Get all joints in the antenna chain
		auto relevant_joints = skinnedMesh.skin->memo()->GetAllChildrenOfRoot(antenna.root);
		
		// Check if antenna is tagged as sensory
		bool is_sensory = false;
		for(auto joint : relevant_joints) {
			for(auto word : skinnedMesh.skin->tags[joint]) {
				if(word == Word::sensory || word == Word::chemoreceptor) {
					is_sensory = true;
					break;
				}
			}
			if(is_sensory) break;
		}
		
		if(is_sensory || antenna.stretched_length > 0) {
			r.antennae.is_sensory = true;
			
			// Estimate surface area of antenna
			for(auto joint : relevant_joints) {
				r.antennae.surface_area += skinnedMesh.surfaceArea[joint] * area_scale[joint];
			}
		}
	}
	
    auto parents = skinnedMesh.skin->parents.data();
    auto semantic_flags = skinnedMesh.skin->memo()->GetSemanticFlags();
    auto gcr_table = skinnedMesh.skin->memo()->GetGcrTable();
    auto N = semantic_flags.size();
    
    std::vector<Sensory::Vision::EyeInfo> eyes;
    
    for(uint32_t i = 0; i < N; ++i) 
    {      
        if(!HasFlag(semantic_flags[i], SF::VISION)) continue;
        
        Sensory::Vision::EyeInfo eye;
        eye.joint_index = i;
        eye.position = positions(i);
        
        // Estimate eye size
        double eye_volume = skinnedMesh.volume[i] * volume_scale[i];
        eye.eye_diameter = std::cbrt(eye_volume * 6.0 / 3.14159);
        
        // ===================================================================
        // DETECT EYESTALK - find nearest body attachment
        // ===================================================================
        
        // Walk up to find head/body (not tagged as appendage)
        int body_attachment = -1;
        for(int j = parents[i]; j >= 0; j = parents[j]) {
        
            if(HasFlag(semantic_flags[i], SF::HEAD|SF::SPINE|SF::ABDOMEN|SF::LIMB|SF::TAIL))
            {
                body_attachment = j;
                break;
            }
        }
        
        if(body_attachment < 0) {
            body_attachment = parents[i]; // Fallback to parent
        }
        
        // Check if there's a stalk between body and eye
        SkinnedMesh::StalkData stalk;
        eye.is_on_stalk = skinnedMesh.GetStalkData(stalk, body_attachment, i, transforms);
        
        if(eye.is_on_stalk) {
            eye.base_position = positions(stalk.root);
            eye.stalk_length = stalk.length_m;
            
            // Eye points away from base
            eye.pointing_direction = glm::normalize(glm::vec3(eye.position - eye.base_position));
            
            // Stalks are mobile
            eye.mobility_rad = glm::radians(90.0f);
            
        } else {
            // Fixed eye
            eye.base_position = positions(body_attachment);
            
            // Eye points radially + respects local rotation
            glm::vec3 radial = glm::normalize(glm::vec3(eye.position - eye.base_position));
            glm::quat local_rot = skinnedMesh.skin->rotation[i];
            glm::vec3 local_forward = local_rot * glm::vec3(0, 0, 1);
            
            eye.pointing_direction = glm::normalize(radial * 0.3f + local_forward * 0.7f);
            eye.mobility_rad = glm::radians(15.0f);
            eye.stalk_length = 0.0f;
        }
        
        eyes.push_back(eye);
    }
    
    r.vision.eyes = shared_array<Sensory::Vision::EyeInfo>::FromArray(eyes);
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
        
        glm::vec3 common_pos = positions(common_root);
        
        // Vectors from common root to each eye
        glm::vec3 to_A = glm::vec3(eye_A.position) - common_pos;
        glm::vec3 to_B = glm::vec3(eye_B.position) - common_pos;
        
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
        
        r.vision.angular_separation_rad = angular_separation_rad;
        
        // ===============================================================
        // CENTERING - for asymmetric creatures (flatfish!)
        // ===============================================================
        
        // Check if eyes are on opposite sides (normal) or same side (flatfish)
        auto left_right_balance = (angle_A + angle_B) / 2.0f;
        
        // If both eyes are on left (negative) or both on right (positive): asymmetric
        r.vision.centering = std::clamp(left_right_balance / glm::radians(90.0f), -1.0f, 1.0f);
    }
    
    return r;
}


std::vector<TonTon::Builder_Tail>   TonTon::Builder::BuilderCommand::GetTails(Builder & in) 
{
	auto & sk = skinnedMesh;
	auto * sk_memo = sk.skin->memo();
	
	auto parents        = skinnedMesh.skin->parents.data();
	auto volume         = skinnedMesh.volume.data();
	auto tags           = skinnedMesh.skin->tags.data();
	auto children       = skinnedMesh.skin->memo()->GetChildren();
	auto relative_flags = skinnedMesh.skin->memo()->GetRelativeFlags();
	auto semantic_flags = skinnedMesh.skin->memo()->GetSemanticFlags();
	auto dfs_ordering   = sk_memo->GetDfsOrdering();
	auto tube_table     = skinnedMesh.memo()->GetTubeTable();
	
	if(HasFlag(relative_flags[dfs_ordering[0]].child_flags, SF::TAIL) == false)
		return {};
	
	auto N = semantic_flags.size();
	
	shared_array<uint8_t> tube_marks = shared_array<uint8_t>(N, 0);
	
	for(auto i = 0u; i < N; ++i)
	{
	// leaves first
		auto node = dfs_ordering[(N-1)-i];
		
		// children[node].empty means a cache miss, so try to short circuit. 
		if(tube_marks[node] == false 
		&& tube_table[node] > 0.98)
		{
			tube_marks[node] |= children[node].size()? 0x02 : 0x01;
		}
		
		auto p = parents[node];
		
		if(p >= 0 && tube_marks[node])
		{
			tube_marks[p] = 0x04;
		}
	}

	std::function<Builder_Tail(int)> GetTail = [&](const int root) -> Builder_Tail
	{	
		auto node = root;
		float stretched_length = 0;
		int   no_links = 0;
		
		std::vector<Builder_Tail> branches;
		std::vector<int> stack;
		stack.reserve(8);
		
		for(;;)
		{
			glm::vec3 accumulator{0};
			for(auto child : children[node])
			{
				if(tube_marks[child] && HasFlag(semantic_flags[child], SF::TAIL))
				{
					stack.push_back(child);
					accumulator += positions(child);
				}
			}
			
			if(stack.empty()) break;
			
			accumulator /= stack.size();
			
			auto vec = positions(stack[0]) - positions(node);
			stretched_length += glm::length(vec);
			++no_links;
						
			if(stack.size() == 1)
			{
				node = stack[0];
				stack.clear();
				continue;
			}
			
			for(auto branch : stack)
			{
				branches.push_back(GetTail(branch));
			}
					
			break;
		}
		
		Builder_Tail r;
		static_cast<Builder_Chain&>(r) = GetChain(*this, node, root);
		
		r.root=uint16_t(root);
		r.tip=uint16_t(node);
		r.noJoints=no_links;
		
		if(HasFlag(relative_flags[root].child_flags, SF::WEAPON))
		{
			r.used_for = Analysis_Tail::Flags(int(r.used_for) | int(Analysis_Tail::Flags::Combat));
			
			std::vector<int> stack;
			stack.reserve(children.size());
			stack.push_back(root);
			
			while(stack.size())
			{
				auto item = stack.back();
				stack.pop_back();
				
				bool is_in_branch = false;
				
				for(auto & b : branches)
				{
					if(b.root == item)
					{
						is_in_branch = true;
						break;
					}
				}
				
				if(is_in_branch)
					continue;
					
				for(auto tag : tags[item])
				{
					if(tag == Word::venom)
					{
						r.venom_joint = item;
						r.venom_joint_volume = volume[item] * volume_scale[item];
						break;
					}
				}
				
				if(r.venom_joint)
					break;
					
				stack.insert(stack.end(), children[item].begin(), children[item].end());
			}
		}
		
		r.total_volume = r.volume;
		
		for(auto & b : branches)
		{
			r.total_volume += b.total_volume;
		}
		
		if(HasFlag(relative_flags[root].child_flags, SF::GRASPER))
		{
			r.used_for = Builder_Tail::Flags(int(r.used_for) | int(Builder_Tail::Flags::Grasping));
		}
		
		r.branches=shared_array<Builder_Tail>::FromArray(branches);
		
		return r;
	};
	
	std::vector<Builder_Tail> r;
// find tail roots		
	for(auto node : dfs_ordering)
	{		
		if(HasFlag(semantic_flags[node], SF::TAIL)
		&& HasFlag(relative_flags[node].parent_flags, SF::TAIL) == false)
		{
			r.push_back(GetTail(node));
		}
	}
	
	std::sort(r.begin(), r.end());
	
	return r;
}

std::optional<TonTon::Builder_Chain> TonTon::Builder::BuilderCommand::GetBodyWave(Builder& it) 
{
	if(it.tails.empty())
		return std::nullopt;
		
	int32_t tip = it.tails[0].tip;
	
	for(auto ptr = it.tails.data();;)
	{
		if(ptr->branches.size())
		{
			ptr = ptr->branches.data();
		}	
		else
		{
			tip = ptr->tip;
			break;
		}
	}
	
	auto semantic_flags = skinnedMesh.skin->memo()->GetSemanticFlags();	
	auto relative_flags = skinnedMesh.skin->memo()->GetRelativeFlags();	
	auto parents = skinnedMesh.skin->parents.data();	
	auto root = it.tails[0].root;
	
	for(int32_t j = root, p = root; j >= 0; j = p)
	{
		p = parents[j];
		
		if(p == -1
		|| HasFlag(relative_flags[p].child_flags, SF::LIMB)
		|| !HasFlag(semantic_flags[p], SF::SPINE))
		{
			return GetChain(*this, tip, j);
		}
	}

	return std::nullopt;
}

std::vector<TonTon::Builder_Appendage> TonTon::Builder::BuilderCommand::GetAppendages(Builder& in)
{
	auto chains = Builder::GetChainsFromRoot(*this, SF::LIMB|SF::WING|SF::FIN, SF::NONE);
	
	auto & sk = skinnedMesh;
	auto parents =  skinnedMesh.skin->parents.data();
	auto semantic_flags =  skinnedMesh.skin->memo()->GetSemanticFlags();
	auto relative_flags =  skinnedMesh.skin->memo()->GetRelativeFlags();
	auto clade_flags =  skinnedMesh.skin->memo()->GetCladeFlags();
	auto relative_clade_flags =  skinnedMesh.skin->memo()->GetRelativeCladeFlags();
	auto children =  skinnedMesh.skin->memo()->GetChildren();
	auto gcr_table = skinnedMesh.skin->memo()->GetGcrTable();
	auto N = children.size();

	auto cliques = sk.memo()->GetCliques();
	std::vector<TonTon::Builder_Appendage> r;
	r.resize(chains.size());
	
	auto GetRootAxis = [&](int root, int tip) 
	{
		for(auto child : children[root])
		{
			if(gcr_table[child*N + tip] == child)
			{
				return glm::normalize(positions(child) - positions(root));
			}
		}
		
		return glm::vec3(0);
	};
	
	auto GetDistanceToParent = [&](int root) 
	{
		int p = parents[root];
		
		if(p >= 0)
		{
			return glm::length(positions(root) - positions(p));
		}
		
		return 0.f;
	};
	
	std::vector<int> stack;
	stack.reserve(N);
	
	auto GetAABB = [&](int root) -> Builder_Appendage::AABB
	{
		stack.clear();
		stack.push_back(root);
		
		glm::vec3 min{FLT_MAX};
		glm::vec3 max{-FLT_MAX};
		
		while(stack.size())
		{
			auto node = stack.back();
			stack.pop_back();
			
			stack.insert(stack.end(), children[node].begin(), children[node].end());
			
			auto old_pos = skinnedMesh.skin->position[node];
			glm::vec3 aabb_min = transforms[node] * glm::vec4(skinnedMesh.aabb[node].min, 1);
			glm::vec3 aabb_max = transforms[node] * glm::vec4(skinnedMesh.aabb[node].max, 1);
			
			min = glm::min(min, aabb_min);
			max = glm::max(max, aabb_max);
		}
		
		return {.min=min, .max=max};
	};
	
	auto GetSurface = [&](int root, int tip, std::vector<uint16_t> const& relevant_joints) -> Builder_Appendage::Surface
	{
		SkinnedMesh::LimbMetrics limb_metrics;
		std::pair<glm::quat, glm::vec3> eigen_decomposition;
	
		// Get fin area from silhouette
		auto projection = skinnedMesh.memo()->GetProjectionMatrix(
			EigenValue::Small, 
			transforms,
			std::span<const uint16_t>(relevant_joints), 
			&limb_metrics, 
			&eigen_decomposition);
			
		auto silhouette = skinnedMesh.memo()->GetSilhouettes(projection, glm::vec3(1), transforms, relevant_joints);
		
		auto surface_normal = GetProjectionDirection(EigenValue::Small, eigen_decomposition.first);	
		auto x_axis = glm::normalize(positions(root) - bone_tails(tip));
		auto bitangent = glm::cross(x_axis, surface_normal);
			
		return Builder_Appendage::Surface{
			.area=silhouette.area,
			.chord=silhouette.MeasureWidth_Segment(positions(root), bone_tails(tip)).length,
			.surface_matrix={
				x_axis,
				bitangent,
				surface_normal,
			},
			.unit_inertia=limb_metrics.GetInertia(positions(root), 1.f, bitangent)
		};
	};
	
	auto GetContact = [&](int root, int tip) -> Builder_Appendage::Contact
	{
		// find joint
		auto joint = tip;
		
		while(HasFlag(relative_flags[joint].parent_flags, SF::CONTACT|SF::GRASPER))
		{
			auto p = parents[joint];
			
			if(p == root)
				break;
			else
				joint = p;	
		}
		
		// just weird naming...
		if(joint == root) 
			joint = tip;
			
		Builder_Appendage::Contact contact;
		
		contact.subtree_flags = semantic_flags[joint]|relative_flags[joint].child_flags;
		contact.joint = joint;
		
		contact.rest_length = glm::length(positions(joint) - positions(root));
		contact.stretched_length = [&]() -> float
		{
			float accumulator= 0;
			
			for(auto j = joint; ; j = parents[j])
			{
				accumulator += glm::distance(bone_tails(j), positions(j));
				if(j == root) break;
			}
			
			return accumulator;
		}();
		
		std::vector<uint16_t> relevant_joints = skinnedMesh.skin->memo()->GetAllChildrenOfRoot(joint);
		
		for(auto j : relevant_joints)
		{
			for(auto word : skinnedMesh.skin->tags[j])
			{
				switch(word)
				{
				case Word::sucker: 
					contact.has_suckers=true;
					break;
				case Word::setae: 
					contact.has_setae=true;
					break;
				case Word::claw: 
					contact.has_claws=true;
					break;
				case Word::thumb: 
					contact.has_thumb=true;
					break;
				case Word::pad: 
					contact.has_wet_grip=true;
					break;
				default: break;
				}
			}
		}
		
		if(joint == tip)
		{
			auto j = skinnedMesh.volume[joint]? joint : parents[joint];
			
			contact.normal = glm::normalize(bone_tails(j) - positions(j));
			contact.area = skinnedMesh.EstimateCrossSection(
				j,
				transforms[j],
				contact.normal
			 );
		}
		else
		{
			Builder_Appendage::Surface surface = GetSurface(joint, tip, relevant_joints);
			
			contact.normal = surface.surface_matrix[2];
			contact.area = surface.area;
		}
		
		return contact;
	};
	
	
	for(auto i = 0u; i < r.size(); ++i)
	{
		static_cast<Builder_Chain&>(r[i]) = chains[i];
		auto idx =  chains[i].root;
			
		r[i].semantic_flags = semantic_flags[idx]|relative_flags[idx].child_flags;
		r[i].clade_flags = clade_flags[idx]|relative_clade_flags[idx].child_flags;
		
		r[i].rootAxis = GetRootAxis(r[i].root, r[i].tip);
		r[i].distance_to_parent = GetDistanceToParent(r[i].root);
		r[i].aabb = GetAABB(r[i].root);
		
		r[i].contact = GetContact(r[i].root, r[i].tip);
		r[i].surface = GetSurface(r[i].root, r[i].tip, skinnedMesh.skin->memo()->GetAllChildrenOfRoot(r[i].root));
	}
	
	uint32_t current_gait_group = 0;
	for(auto i = 0u; i < r.size(); ++i)
	{
		if(r[i].gait_group >= 0) continue;
		
		for(auto & clique : cliques)
		{
			if(r[i].commonAncestor != (int32_t)clique.parent)
				continue;
				
			int clique_size{};
			for(auto j = i; j < r.size(); ++j)
			{
				if(std::find(clique.children.data(), clique.children.end(), r[j].root) == clique.children.end())
					continue;
			
				r[j].gait_group = current_gait_group;
				++clique_size;
			}
			
			if(clique_size)
			{
				++current_gait_group;
				break;
			}
		}
		
		if(r[i].gait_group < 0)
		{
			r[i].gait_group = (current_gait_group++);
		}
	}
	
	std::sort(r.begin(), r.end(), [](auto & a, auto & b)
	{
		if(a.gait_group != b.gait_group)
			return a.gait_group < b.gait_group;
		
		return a.root < b.root;
	});
	
	for(auto i = 0u; i < r.size(); ++i)
		r[i].id = i;
	
	return r;
}

std::vector<glm::vec3>  TonTon::Builder::BuilderCommand::GetGaitGroupCenters(Builder& builder)
{	
	int no_groups = 0;
	
	for(auto & p : builder.appendages)
	{	
		no_groups= std::max(int(p.gait_group+1), no_groups);
	}
		
	std::vector<glm::vec3> positions(no_groups, glm::vec3(0));
	std::vector<int> count(no_groups, 0);
	
	for(auto & p : builder.appendages)
	{
		count[p.gait_group] += 1;
		positions[p.gait_group] += this->positions(p.root);
	}

	for(auto j = 0u; j < count.size(); ++j)
	{
		positions[j] *= count[j]? (1.0 / count[j]) : 0.0;
	}
	
	return positions;
}
		
int32_t TonTon::Builder::BuilderCommand::GetSiphonJoint(Builder&) const
{
	for(auto i = 0u; i < skinnedMesh.skin->tags.size(); ++i)
	{
		for(auto tag : skinnedMesh.skin->tags[i])
			if(tag == Word::siphon)
				return i;
	}
	
	return -1;
}


template<typename T>
std::vector<TonTon::Builder_Chain> TonTon::Builder::GetChainsFromRoot(TonTon::Builder::BuilderCommand const& in, T const& function)
{
	std::vector<TonTon::Builder_Chain> r;
	auto & sk = in.skinnedMesh;
	auto * sk_memo = sk.skin->memo();
	
	auto parents =  in.skinnedMesh.skin->parents.data();
	
	auto dfs_ordering = sk_memo->GetDfsOrdering();
	auto leaves       = sk_memo->GetLeaves();
	auto gcr			 = sk_memo->GetGcrTable();
	
	auto noJoints = dfs_ordering.size();
	std::vector<bool> marks(noJoints, false);
	
	for(auto node : dfs_ordering)
	{
		if(marks[node]) continue;
		
		if(function(node) == false)
			continue;
		
		double max_length = -1;
		uint32_t best_leaf = node;
		int jointsInChain = 0;
		
		double accCrossSection=0;
		
		
		// for each node that uses this as a root.
		// so we go from each finger and compute distance to shoulder.
		for(auto leaf : leaves)
		{
			if(gcr[node*noJoints + leaf] != node)
				continue;
			
			double length = 0;
			int counter = 0;
			for(int32_t j = leaf, p; j >= node; j = p)	
			{
				p = parents[j];
				marks[j] = true;
				
				if(p >= node)
				{
					length += glm::distance(in.positions(j), in.positions(p));
				}
			}
			
			if(length > max_length)
			{
				max_length = length;
				best_leaf = leaf; 
				jointsInChain = counter;
			}
		}
		
		r.push_back(GetChain(in, best_leaf, node));
	}
	
	return r;
}

std::vector<TonTon::Builder_Chain> TonTon::Builder::GetChainsFromRoot(TonTon::Builder::BuilderCommand const& in, std::span<Word> words)
{
	auto tags = in.skinnedMesh.skin->tags;
	
	return GetChainsFromRoot(in, [&](int node) -> bool
	{
		for(auto word : words)
		{
			if(tags[node].find(word) >= 0)
			{
				return true;
			}
		}
		
		return false;
	});
}

std::vector<TonTon::Builder_Chain> TonTon::Builder::GetChainsFromRoot(TonTon::Builder::BuilderCommand const& in, SemanticFlags flags, SemanticFlags child_flags)
{
	auto semantic_flags = in.skinnedMesh.skin->memo()->GetSemanticFlags();
	auto relative_flags = in.skinnedMesh.skin->memo()->GetRelativeFlags();
	
	return GetChainsFromRoot(in, [&](int node) -> bool
	{
		return HasFlag(semantic_flags[node], flags)
			&& (child_flags == SemanticFlags::NONE
			||  HasFlag(relative_flags[node].child_flags, child_flags));
	});
}

TonTon::Builder_Chain TonTon::Builder::GetChain(TonTon::Builder::BuilderCommand const& in, int leaf, int root)
{
	auto parents =  in.skinnedMesh.skin->parents.data();
	
	float minCrossSection=FLT_MAX;
	float maxCrossSection=-FLT_MAX;
	float accCrossSection={};
	float minMoment=FLT_MAX;
	float maxMoment=-FLT_MAX;
	float accMoment={};
	float minRadius_=FLT_MAX;
	float maxRadius_=-FLT_MAX;
	float surface_area={};
	float length={};
	float volume={};
	glm::dvec3 accCentroid={};
	int32_t jointsInChain{};

	for(int32_t j = leaf, p; ; j = parents[j])
	{
		double vol = in.skinnedMesh.volume[j] * in.volume_scale[j];

		auto vec = in.bone_tails(j) - in.positions(j);
		double moment{}, radius{};
		auto crossSection = in.skinnedMesh.EstimateCrossSection(j, in.transforms[j], vec, &moment, &radius);
					
		if(crossSection)
		{
			minCrossSection= std::min<float>(minCrossSection, crossSection);
			minMoment= std::min<float>(minMoment, moment);
		}

		if(radius > 0) {
			minRadius_= std::min<float>(minRadius_, radius);
		}

		maxCrossSection= std::max<float>(maxCrossSection, crossSection);
		maxMoment= std::max<float>(maxMoment, moment);
		maxRadius_= std::max<float>(maxRadius_, radius);
		
		accCrossSection += crossSection;
		accMoment += moment;
		surface_area= in.skinnedMesh.surfaceArea[j] * in.area_scale[j];
		length= glm::length(vec);
		volume= vol;
		accCentroid= glm::dvec3(in.centroid(j)) * vol;
		jointsInChain += 1;
		
		if(j == root) break;
	}
	
	if(minCrossSection == FLT_MAX)
	{
		minMoment       = maxMoment;
		maxCrossSection = maxCrossSection;
	}
	if(minRadius_ == FLT_MAX)
	{
		minRadius_ = maxRadius_;
	}

	double invVolume = volume? 1.0 / volume : 0.0;
	float invJoints = jointsInChain? 1.0 / jointsInChain : 0.0;

	auto rest_len = glm::distance(in.positions(leaf), in.positions(root));

	// Compute crouch_length via CCD solver with capsule collision when articulations available
	float crouch_len;
	if (!in.articulations.empty())
	{
		auto solver = RigSolver::Factory(
			counted_ptr<const SkinnedMesh>::Wrap(const_cast<SkinnedMesh*>(&in.skinnedMesh)),
			in.articulations);
		crouch_len = solver->SolveCrouchLength(uint16_t(root), uint16_t(leaf));
	}
	else
	{
		crouch_len = rest_len * 0.3f; // placeholder: leg compresses to ~30% of rest length
	}

	return Builder_Chain{
			.root=uint16_t(root),
			.tip=uint16_t(leaf),
			.commonAncestor=int16_t(parents[root]),
			.noJoints=int16_t(jointsInChain),

			.surface_area=surface_area,
			.volume=volume,
			.centroid=glm::vec3(accCentroid * invVolume),

			.stretched_length=float(length),
			.rest_length=rest_len,

			.minCrossSection=minCrossSection,
			.avgCrossSection=accCrossSection * invJoints,
			.maxCrossSection=maxCrossSection,
			.minMoment=minMoment,
			.avgMoment=accMoment * invJoints,
			.maxMoment=maxMoment,

			.minRadius=minRadius_,
			.maxRadius=maxRadius_,
			.crouch_length=crouch_len,
			.root_position=in.positions(root),
		};
}





