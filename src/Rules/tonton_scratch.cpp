#include "tonton_scratch.h"
#include "Memos/tonton_armaturememo.h"
#include "Rules/tonton_aerial.h"
#include "Rules/tonton_sensory.h"
#include "Rules/tonton_specialized.h"
#include "Rules/tonton_climbing.h"
#include "Rules/tonton_serpentine.h"
#include "Rules/tonton_metabolic.h"
#include "../include/tonton_input.h"

#include "Memos/tonton_armaturememo.h"
#include "Memos/tonton_skinnedmeshmemo.h"
#include "Memos/tonton_meshmemo.h"
#include "Rules/tonton_behavior.h"
#include "Rules/tonton_aquatic.h"
#include "Rules/tonton_terrestrial.h"
#include "dodeedum.h"
#include <cfloat>
#include <functional>

using Warning = TonTon::Output_Diagnostics::Warning::Severity;
using SF = TonTon::SemanticFlags;

namespace TonTon
{
static	Output_Physical ComputePhysical(Input const& in, Scratch & s);

static	std::vector<Output_Manipulator>   ComputeManipulation(Input const& in, Scratch & s);
static	std::vector<Output_Tail>   ComputeTails(Input const& in, Scratch & s);

}

namespace TonTon
{
Output_TakeoffAnalysis TakeoffAnalysis_Compute(const Scratch& output);
}

TonTon::Scratch::Scratch(Input const& in)
{

	physical    = ComputePhysical(in, *this);
	sensory     = ComputeSensory(in, *this);

	appendages.manipulation = shared_array<Output_Manipulator>::FromArray(ComputeManipulation(in, *this));
	appendages.tails = shared_array<Output_Tail>::FromArray(ComputeTails(in, *this));

	// Metabolic rates computed early (after physical, before locomotion)
	// Locomotion modes need metabolic budget, but metabolic needs clade flags from physical
	// For multi-clade creatures (pegasus), metabolic blends all clade contributions
	metabolic   = ComputeMetabolic(in, *this);

	terrestrial = ComputeTerrestrial(in, *this);
	serpentine  = ComputeSerpentine(in, *this);
	jumping     = ComputeJumping(in, *this);

	aerial      = ComputeAerial(in, *this);
	aquatic     = ComputeAquatic(in, *this);

	climbing    = ComputeClimbing(in, *this);
	brachiation = ComputeBrachiation(in, *this);

	specialized.digging = ComputeDigging(in, *this);
	specialized.constriction = ComputeConstriction(in, *this);


	if(aerial.has_value())
	{
		aerial->takeoff = TakeoffAnalysis_Compute(*this);
	}

	behavior =  ComputeBehavior(in, *this);


};

using namespace TonTon;

static Output_Physical TonTon::ComputePhysical(Input const& in, Scratch & out)
{
	auto & sk = *in.skinnedMesh;
	auto * sk_memo = sk.skin->memo();
	
	auto parents = sk.skin->parents.data();
	
	float volume{};
	float surfaceArea{};
	
	for(auto i = 0u; i < sk.volume.size(); ++i)
	{
		volume += sk.volume[i];
		surfaceArea += sk.surfaceArea[i]; 
	}
	
	volume *= in.behavior.volume_scale();
	surfaceArea *= in.behavior.area_scale();

	auto relative_flags = sk_memo->GetRelativeFlags();
	auto semantic_flags = sk_memo->GetSemanticFlags();
	auto dfs_ordering   = sk_memo->GetDfsOrdering();
	auto gcr_table      = sk_memo->GetGcrTable();
	auto leaves         = sk_memo->GetLeaves();
	auto names			= sk.skin->names.data();
	auto root = dfs_ordering[0];
	(void)names;
	
	SF SPINE_FLAGS = SF::SPINE | SF::THORAX | SF::PELVIS;
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
	
	auto ProcessHead = [&](uint32_t begin)
	{
		auto root = dfs_ordering[begin];
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
			in.position(root));
		double spine_length = 0;
		double this_cross_section_area = 0;
		bool is_this_upright = false;
		int this_spine_root{};
	
	// walk down spine
		for(int joint = root; joint >= 0; joint = parents[joint])
		{
			auto p = parents[joint];
			
			if(p >= 0)
			{
				glm::vec3 delta = in.position(joint) - in.position(p);
				
				if(root == joint)
				{
					float projection = glm::dot(delta, glm::vec3(0, 1, 0));
					is_this_upright = (projection > 0.707);					
				}
				
				if(HasFlag(semantic_flags[joint], SPINE_FLAGS))
				{
					spine_length += glm::length(delta);
				}
			}
			
			if(HasFlag(semantic_flags[joint], SPINE_FLAGS))
			{
				auto children = sk_memo->GetAllChildren(joint, SPINE_FLAGS, NOT_SPINE_FLAGS);
				children.push_back(joint);
				
				auto silhouette = in.skinnedMesh->memo()->GetSilhouettes(
					Axis::Z, 
					in.behavior.scale,
					std::span(children.data(), children.size()),
					0.5,
					false
				);
				
				this_cross_section_area = std::max(this_cross_section_area, silhouette.area);
			}
		}
	
		double SVL = snout_to_base_of_skull + spine_length;
		
		if(SVL > max_body_length)
		{
			max_body_length = SVL;
			is_upright = is_this_upright;
			crossSectionArea = this_cross_section_area;
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
					length += glm::distance(in.position(joint), in.position(p));
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
				tail_length += glm::distance(in.position(joint), in.position(p));
				
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
		out.diagnostics.warnings.push_back({
			.level=Warning::ERROR,
			.message="unable to find head of model."
		});
	}
	
	max_body_length += tail_to_root_dist;
	double diameter = 2.0 * (sqrt(crossSectionArea) / M_PI);	
	
	double body_density= in.body_density();
	auto C = in.skinnedMesh->GetCovariance(std::span<uint16_t>{}, in.behavior.scale);
		
	
	return {
		.body_mass_kg=float(volume * body_density),
		.body_length_m=max_body_length,
		.body_volume_m3=volume,
		.tail_length_m=max_tail_length,
		
		.surface_area_m2=surfaceArea,
		.cross_sectional_area_m2=crossSectionArea,
		.fineness_ratio=float(max_body_length/diameter),
		.spine_root=static_cast<int16_t>(spine_root),
		.upright=is_upright,		
		.clade=(sk_memo->GetCladeFlags()[root]|sk_memo->GetRelativeCladeFlags()[root].child_flags),
		.covariance_restPose=std::array<float, 6>{
			float(C[0]*body_density),
			float(C[1]*body_density),
			float(C[2]*body_density),
			float(C[3]*body_density),
			float(C[4]*body_density),
			float(C[5]*body_density),
		}
	};
}

template<typename T>
static std::vector<Output_Chain> GetChainsFromRoot(TonTon::Input const& in, T const& function)
{
	std::vector<TonTon::Output_Chain> r;
	auto & sk = *in.skinnedMesh;
	auto * sk_memo = sk.skin->memo();
	
	auto parents =  in.skinnedMesh->skin->parents.data();
	
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
					length += glm::distance(in.position(j), in.position(p));
				}
			}
			
			if(length > max_length)
			{
				max_length = length;
				best_leaf = leaf; 
				jointsInChain = counter;
			}
		}
		
		r.push_back(Output_Chain{
			.root=node,
			.tip=uint16_t(best_leaf),
			.noJoints=jointsInChain,
			.stretched_length_m=float(max_length),
			.rest_length_m=glm::distance(in.position(best_leaf), in.position(node))
		});
	}
	
	return r;
}


std::vector<Output_Chain> TonTon::GetChainsFromRoot(Input const& in, std::span<Word> words)
{
	auto tags = in.skinnedMesh->skin->tags;
	
	return ::GetChainsFromRoot(in, [&](int node) -> bool
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

std::vector<Output_Chain> TonTon::GetChainsFromRoot(TonTon::Input const& in, SemanticFlags flags)
{
	auto semantic_flags = in.skinnedMesh->skin->memo()->GetSemanticFlags();
	
	return ::GetChainsFromRoot(in, [&](int node) -> bool
	{
		return HasFlag(semantic_flags[node], flags);
	});
}

std::vector<Output_Chain> TonTon::GetChainsFromTip(TonTon::Input const& in, SemanticFlags include_flags, SemanticFlags exclude_flags)
{
	std::vector<TonTon::Output_Chain> chains;
	auto & sk = *in.skinnedMesh;
	auto * sk_memo = sk.skin->memo();
	
	auto dfs_ordering = sk_memo->GetDfsOrdering();
	auto parents       = sk.skin->parents.data();
	auto semantic_flags = sk_memo->GetSemanticFlags();
	auto relative_flags = sk_memo->GetRelativeFlags();
	auto gcr			 = sk_memo->GetGcrTable();

	for(auto i = 0u; i < semantic_flags.size(); ++i)
	{
		if(!HasFlag(semantic_flags[i], include_flags))
			continue;
			
		auto root = i;
		float length = 0;
		int counter = 0;
		
		for(;;)
		{
			auto p = parents[root];
			if(p < 0) break;
			
				
			if(HasFlag(semantic_flags[i], exclude_flags))
				break;
				
			length += glm::distance(in.position(root), in.position(p));
			root = p;
			++counter;
		}
		
		Output_Chain chain{
			.root=uint16_t(root),
			.tip=uint16_t(i),
			.noJoints=counter,
			.stretched_length_m=length,
			.rest_length_m=glm::distance(in.position(root), in.position(i)),
		};
		
		chains.push_back(chain);
	}
	
	return chains;
}

std::vector<Output_Appendage> TonTon::GetAppendages(Input const& in, std::vector<Output_Chain> && chains)
{
	auto & sk = *in.skinnedMesh;
	auto parents =  in.skinnedMesh->skin->parents.data();

	auto cliques = sk.memo()->GetCliques();
	std::vector<TonTon::Output_Appendage> r;
	r.resize(chains.size());
	
	for(auto i = 0u; i < r.size(); ++i)
	{
		(Output_Chain&)r[i] = chains[i];
		r[i].common_ancestor = parents[r[i].root];
		r[i].gait_group = -1;
	}
	
	uint32_t current_gait_group = 0;
	for(auto i = 0u; i < r.size(); ++i)
	{
		if(r[i].gait_group >= 0) continue;
		
		for(auto & clique : cliques)
		{
			if(r[i].common_ancestor != (int32_t)clique.parent)
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
	
	return r;
}

std::vector<Output_Manipulator>   TonTon::ComputeManipulation(Input const& in, Scratch&)
{// walk back parents until we get something thats not limb-ish
	SF constexpr NOT_LIMB_FLAGS = SF(
		int64_t(SF::HEAD)|
		int64_t(SF::NECK)|
		int64_t(SF::SPINE)|
		int64_t(SF::ABDOMEN)
	);
	
	auto appendages = GetAppendages(in, GetChainsFromTip(in, SF::GRASPER, NOT_LIMB_FLAGS));
	return ComputeManipulation(in, appendages);
} 


static	std::vector<Output_Tail>   TonTon::ComputeTails(Input const& in, Scratch &out) 
{
	auto & sk = *in.skinnedMesh;
	auto * sk_memo = sk.skin->memo();
	
	auto parents        = in.skinnedMesh->skin->parents.data();
	auto position       = in.skinnedMesh->skin->position.data();
	auto volume         = in.skinnedMesh->volume.data();
	auto children       = in.skinnedMesh->skin->memo()->GetChildren();
	auto relative_flags = in.skinnedMesh->skin->memo()->GetRelativeFlags();
	auto semantic_flags = in.skinnedMesh->skin->memo()->GetSemanticFlags();
	auto dfs_ordering   = sk_memo->GetDfsOrdering();
	auto tube_table     = in.skinnedMesh->memo()->GetTubeTable();
	
	if(HasFlag(relative_flags[dfs_ordering[0]].child_flags, SF::TAIL) == false)
		return {};
	
	auto N = semantic_flags.size();
	
	shared_array<bool> tube_marks = shared_array<bool>(N, 0);
	
	for(auto i = 0u; i < N; ++i)
	{
	// leaves first
		auto node = dfs_ordering[(N-1)-i];
		
		// children[node].empty means a cache miss, so try to short circuit. 
		if(tube_marks[node] == false && tube_table[node] > 0.98 && children[node].empty())
			tube_marks[node] = true;
		
		auto p = parents[node];
		
		if(p >= 0 && tube_marks[node])
		{
			tube_marks[p] = true;
		}
	}

	std::function<Output_Tail(int)> GetTail = [&](const int root) -> Output_Tail
	{	
		auto node = root;
		auto node_children = children[node].size() == 0;
		float stretched_length = 0;
		float total_volume = 0;
		float min_area = FLT_MAX;
		float max_area =-FLT_MAX;
		int   no_links = 0;
		
		std::vector<Output_Tail> branches;
		std::vector<int> stack;
		stack.reserve(8);
		
		for(;;)
		{
			glm::vec3 accumulator{0};
			for(auto child : children[node])
			{
				if(tube_marks[child] && HasFlag(semantic_flags[node], SF::TAIL))
				{
					stack.push_back(child);
					accumulator += position[child];
				}
			}
			
			if(stack.empty()) break;
			
			accumulator /= stack.size();
			
			auto vec = position[stack[0]] - position[node];
			total_volume += volume[node];
			float area = in.skinnedMesh->EstimateCrossSection(node, in.behavior.scale, vec);
			
			min_area = std::min(min_area, area);
			max_area = std::max(max_area, area);
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
		
		Output_Tail r;
		
		r.root=uint16_t(root);
		r.tip=uint16_t(node);
		r.noJoints=no_links;
		
		r.stretched_length_m=stretched_length;
		r.rest_length_m=glm::length(position[node] - position[root]);
		
		r.common_ancestor=parents[root];
		r.gait_group=0;
		r.phase_offset=0;
	
		r.mass_kg = in.body_density() * total_volume;
		r.max_cross_section_m2=max_area;
		r.min_cross_section_m2=min_area;
		
		r.natural_sway_frequency_Hz = std::sqrt(in.environment.gravity_m_s2 / r.stretched_length_m) / M_PI;
		r.branches=shared_array<Output_Tail>::FromArray(branches);
		
		if(HasFlag(relative_flags[root].child_flags, SF::WEAPON))
		{
			r.used_for = Output_Tail::Flags(int(r.used_for) | int(Output_Tail::Flags::Combat));
		}
		
		if(HasFlag(relative_flags[root].child_flags, SF::GRASPER))
		{
			r.used_for = Output_Tail::Flags(int(r.used_for) | int(Output_Tail::Flags::Grasping));
		}
		
		return r;
	};
	
	std::vector<Output_Tail> r;
// find tail roots		
	for(auto node : dfs_ordering)
	{
		if(HasFlag(semantic_flags[node], SF::TAIL)
		&& HasFlag(relative_flags[node].parent_flags, SF::TAIL) == false)
		{
			r.push_back(GetTail(node));
		}
	}
	
	return r;
}

std::vector<glm::vec3> TonTon::GetGaitGroupCenters(Input const& in, Output_Appendage * data, size_t size, size_t stride)
{
	std::vector<glm::vec3> positions;
	std::vector<std::pair<int, int>> count;
	positions.reserve(2);
	count.reserve(2);
	
	for(auto i = 0u; i < size; ++i)
	{
		Output_Appendage * p = (Output_Appendage*)(((uint8_t*)data) + stride);
	
		for(auto j = 0u; j < count.size(); ++j)
		{
			if(count[j].first == p->gait_group)
			{
				positions[j] += in.position(p->root);
				count[j].second += 1;
				goto found;
			}
			
			if(count[j].first > p->gait_group)
			{
				count.insert(count.begin()+j, {	int(p->gait_group),1});
				positions.insert(positions.begin()+j, in.position(p->root));
				goto found;
			}
		}

		count.push_back({int(p->gait_group),1});
		positions.push_back(in.position(p->root));
		
	found:
		(void)0;
	}

	for(auto j = 0u; j < count.size(); ++j)
	{
		positions[j] *= 1.0 / count[j].second;
	}
	
	return positions;
}
