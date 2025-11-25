#include "../include/tonton_builder.h"
#include "Memos/tonton_armaturememo.h"
#include "tonton_skinnedmesh.h"

#if 0

using SF = TonTon::SemanticFlags;

static TonTon::Analysis_Physical TonTon::ComputePhysical(BuilderCommand const& in, Builder & out)
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

	auto position       = sk.skin->position.data();
	auto relative_flags = sk_memo->GetRelativeFlags();
	auto children		= sk_memo->GetChildren();
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
			in.position(root));
		double spine_length = 0;
		double this_cross_section_area = 0;
		bool is_this_upright = false;
		bool in_spine = false;
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
				
				if(HasFlag(semantic_flags[p], SPINE_FLAGS))
				{
					spine_length += glm::length(delta);
					this_spine_root = p;
					in_spine = true;
				}
				else if(in_spine)
					break;
			}
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
				delta = in.position(joint) - in.skinnedMesh->memo()->GetBoneTails()[joint];
			else if(candidate_shared_by_head == false)
			{
				if(in_spine == false)
					delta = in.position(root) - in.position(candidate);
				else
					delta = in.position(joint) - in.position(candidate);
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
		
		if(HasFlag(semantic_flags[node], SF::SPINE))
		{
			if(parents[node] >= 0)
			{
				auto children = sk_memo->GetAllChildren(node, SPINE_FLAGS, NOT_SPINE_FLAGS);
				children.push_back(node);
					
				auto area =	in.skinnedMesh->EstimateCrossSection(children, in.behavior.scale, position[node] - position[parents[node]]);	
				crossSectionArea = std::max<double>(crossSectionArea, area);
			}
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
		.body_length_m=max_body_length+max_tail_length,
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
static std::vector<Analysis_Chain> GetChainsFromRoot(TonTon::Input const& in, T const& function)
{
	std::vector<TonTon::Analysis_Chain> r;
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
		
		r.push_back(Analysis_Chain{
			.root=node,
			.tip=uint16_t(best_leaf),
			.noJoints=jointsInChain,
			.stretched_length_m=float(max_length),
			.rest_length_m=glm::distance(in.position(best_leaf), in.position(node))
		});
	}
	
	return r;
}


std::vector<Analysis_Chain> TonTon::GetChainsFromRoot(Input const& in, std::span<Word> words)
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

std::vector<Analysis_Chain> TonTon::GetChainsFromRoot(TonTon::Input const& in, SemanticFlags flags, SemanticFlags child_flags)
{
	auto semantic_flags = in.skinnedMesh->skin->memo()->GetSemanticFlags();
	auto relative_flags = in.skinnedMesh->skin->memo()->GetRelativeFlags();
	
	return ::GetChainsFromRoot(in, [&](int node) -> bool
	{
		return HasFlag(semantic_flags[node], flags)
			&& (child_flags == SemanticFlags::NONE
			||  HasFlag(relative_flags[node].child_flags, child_flags));
	});
}

std::vector<Analysis_Appendage> TonTon::GetAppendages(Input const& in, std::vector<Analysis_Chain> && chains)
{
	auto & sk = *in.skinnedMesh;
	auto parents =  in.skinnedMesh->skin->parents.data();

	auto cliques = sk.memo()->GetCliques();
	std::vector<TonTon::Analysis_Appendage> r;
	r.resize(chains.size());
	
	for(auto i = 0u; i < r.size(); ++i)
	{
		(Analysis_Chain&)r[i] = chains[i];
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

std::vector<Analysis_Manipulator>   TonTon::ComputeManipulation(Input const& in)
{// walk back parents until we get something thats not limb-ish
	
	auto appendages = GetAppendages(in, GetChainsFromRoot(in, SF::LIMB|SF::TAIL|SF::FACIAL, SF::GRASPER));
	return ComputeManipulation(in, appendages);
} 


static	std::vector<Analysis_Tail>   TonTon::ComputeTails(Input const& in) 
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

	std::function<Analysis_Tail(int)> GetTail = [&](const int root) -> Analysis_Tail
	{	
		auto node = root;
		float stretched_length = 0;
		float total_volume = 0;
		float min_area = FLT_MAX;
		float max_area =-FLT_MAX;
		int   no_links = 0;
		
		std::vector<Analysis_Tail> branches;
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
		
		Analysis_Tail r;
		
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
		r.branches=shared_array<Analysis_Tail>::FromArray(branches);
		
		if(HasFlag(relative_flags[root].child_flags, SF::WEAPON))
		{
			r.used_for = Analysis_Tail::Flags(int(r.used_for) | int(Analysis_Tail::Flags::Combat));
		}
		
		if(HasFlag(relative_flags[root].child_flags, SF::GRASPER))
		{
			r.used_for = Analysis_Tail::Flags(int(r.used_for) | int(Analysis_Tail::Flags::Grasping));
		}
		
		return r;
	};
	
	std::vector<Analysis_Tail> r;
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

std::vector<glm::vec3> TonTon::GetGaitGroupCenters(Input const& in, Analysis_Appendage * data, size_t size, size_t stride)
{
	std::vector<glm::vec3> positions;
	std::vector<std::pair<int, int>> count;
	positions.reserve(2);
	count.reserve(2);
	
	for(auto i = 0u; i < size; ++i)
	{
		Analysis_Appendage * p = (Analysis_Appendage*)(((uint8_t*)data) + stride);
	
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

std::vector<TonTon::Analysis_Manipulator>   TonTon::ComputeManipulation(Input const& in, std::vector<Analysis_Appendage> & appendages) 
{ 		
	auto & sk = *in.skinnedMesh;
	auto * sk_memo = sk.skin->memo();
	auto position = sk.skin->position.data();
	auto parents = sk.skin->parents.data();
	
	auto relative_flags = in.skinnedMesh->skin->memo()->GetRelativeFlags();
	auto semantic_flags = in.skinnedMesh->skin->memo()->GetSemanticFlags();
	
	std::vector<Analysis_Manipulator> manipulators(appendages.size());
	
	for(auto i = 0u; i < appendages.size(); ++i)
	{
		static_cast<Analysis_Appendage&>(	manipulators[i]) = appendages[i];
		auto idx = manipulators[i].tip;
		
		int relevant_root = manipulators[i].tip;
		for(auto j = manipulators[i].tip; j >= manipulators[i].root; j = parents[j])
		{
			if(HasFlag(semantic_flags[j], SF::CONTACT|SF::GRASPER))
				relevant_root = j;		
		}
		
		auto relevant_joints = sk_memo->GetAllChildrenOfRoot(relevant_root);
		// get all *excluding grasper itself* (just fingers)
		auto which = std::span<uint16_t>(relevant_joints.data(), relevant_joints.size());
			
		auto metrics = in.skinnedMesh->GetMetrics(which, in.behavior.scale);
		TonTon::Silhouette silhouette;
		
		manipulators[i].subtree_flags = relative_flags[idx].child_flags|semantic_flags[idx];
		
		// it is a spike.. or trunk.. or something, no fingers!
		// trunks have fingers but in an armature they probably don't.
		if(metrics.volume == 0.0)
		{
			auto tip = manipulators[i].tip;
		
			manipulators[i].surface_area_m2 = sk.EstimateCrossSection(tip, in.behavior.scale, position[tip] - position[parents[tip]]);
			manipulators[i].surface_normal  = glm::normalize(position[tip] - position[parents[tip]]);
		}
		else
		{
		// get all *including grasper itself*
			which = std::span<uint16_t>(relevant_joints.data(), relevant_joints.size());
					
			std::pair<glm::quat, glm::vec3> eigen_decomp;
			auto projection = in.skinnedMesh->memo()->GetProjectionMatrix(EigenValue::Small, in.behavior.scale, which, &metrics, &eigen_decomp);
			silhouette = in.skinnedMesh->memo()->GetSilhouettes(projection, in.behavior.scale, which);
			
			// get projection direction.
			manipulators[i].surface_area_m2 = silhouette.area;
			manipulators[i].surface_normal =  glm::mat3(eigen_decomp.first)[2];
		}
		
		auto counter = 0;
		float accumulator = 0;
		float accumulator_2nd_moment = 0;
		for(auto j = manipulators[i].tip; j != manipulators[i].root; j = parents[j])
		{
			auto p =  parents[j];
			if(p < 0) break;
			double secondMoment = 0;
			auto area_m2 = sk.EstimateCrossSection(p, in.behavior.scale, position[j] - position[p], &secondMoment);
		
			accumulator += area_m2;
			accumulator_2nd_moment += secondMoment;
			++counter;
		}
		
		if(counter)
		{
			float avg_area = accumulator / counter;
			float avg_moment = accumulator_2nd_moment / counter;
			
			// 1. MUSCLE VOLUME ESTIMATION
			// Assume ~40-60% of limb volume is muscle (depends on type)
			float muscle_fraction = glm::mix(0.35f, 0.65f, in.behavior.endurance_vs_power);
		//	double muscle_volume_m3 = metrics.volume * muscle_fraction;
		//	double muscle_mass_kg = muscle_volume_m3 * glm::mix(700.0, 1050.0, in.average_density);
			
			// 2. GRIP FORCE (from finger/digit flexors)
			// Peak muscle stress: 20-40 N/cm² (200,000-400,000 Pa)
			// Quality affects max stress achievable
			float muscle_stress_Pa = glm::mix(200000.0f, 400000.0f, in.muscle_quality);
			
			// Effective cross-sectional area (perpendicular to force direction)
			// For grip, this is roughly the cross-section of the muscles
			float grip_muscle_area_m2 = avg_area * muscle_fraction;
			float base_grip_force_N = muscle_stress_Pa * grip_muscle_area_m2;

			// Grip force depends on mechanical advantage / limb geometry
			// Short thick limbs (penguin legs) have poor leverage despite high muscle mass
			// Compare limb length to thickness (sqrt of cross-sectional area)
			float limb_thickness_m = std::sqrt(avg_area); // Characteristic thickness
			float aspect_ratio = manipulators[i].stretched_length_m / std::max(0.001f, limb_thickness_m);
			// Typical limb: aspect_ratio ~5-15 (length is 5-15x thickness)
			// Penguin legs: aspect_ratio ~2-3 (short and thick), poor mechanical advantage
			// Primate arms: aspect_ratio ~10-20 (long and slender), good leverage
			float length_factor = glm::clamp(aspect_ratio / 10.0f, 0.3f, 1.2f); // 0.3x at stubby, 1.0x at normal

			manipulators[i].max_grip_force_N = base_grip_force_N * length_factor;
			
			// 3. LIFT FORCE (constrained by joint torque limits)
			// Torque = Force × moment_arm
			// Max torque ≈ muscle_PCSA × muscle_stress × moment_arm
			float moment_arm_m = std::sqrt(avg_moment / avg_area); // Radius of gyration
			float max_torque_Nm = muscle_stress_Pa * grip_muscle_area_m2 * moment_arm_m;
			
			// Convert torque to force at tip (lever arm = chain length)
			float lever_arm_m = manipulators[i].stretched_length_m;
			if(lever_arm_m > 0.001f) {
				manipulators[i].max_lift_force_N = max_torque_Nm / lever_arm_m;
			} else {
				manipulators[i].max_lift_force_N = 0.0f;
			}
			
			// Apply structure vs weight scaling
			// More robust structure = can handle more force
			float structure_bonus = glm::mix(0.7f, 1.3f, in.structure_vs_weight);
			manipulators[i].max_lift_force_N *= structure_bonus;
			manipulators[i].max_grip_force_N *= structure_bonus;
		}
		
		// 4. ADHESION FORCE (for specialized structures)
		manipulators[i].max_adhesion_force_N = 0.0f;
		
	// grab each thing in the subtree		
		bool has_suckers = false;
		bool has_setae = false;
		bool has_claws = false;
		bool has_thumb = false;
		bool has_wet_grip = false; // what word do i look for for a treefrog?		
		
		for(auto node : relevant_joints)
		{
			for(auto word : sk.skin->tags[node])
			{
				switch(word)
				{
				default: break;
				case Word::sucker:
					has_suckers = true;
					break;
				case Word::setae:
					has_setae = true;
					break;		
				case Word::claw:
				case Word::talon:
					has_claws = true;
					break;		
				case Word::thumb:
					has_thumb = true;
					break;	
				case Word::pad:
				case Word::adhesive:
					has_wet_grip = true;
					break;
				}
			}
		}
		
		manipulators[i].has_claws=has_claws;
		manipulators[i].has_suckers=has_suckers;
		manipulators[i].has_setae=has_setae;
		manipulators[i].has_thumb=has_thumb;
		manipulators[i].has_wet_grip=has_wet_grip;
		
		if(has_suckers) {
			// Suction force: F = ΔP × Area
			// Assume can create ~0.8 atm (80 kPa) pressure differential
			float suction_pressure_Pa = 80000.0f;
			
			// Estimate sucker area from surface area
			float sucker_coverage = 0.3f; // ~30% of surface is actual suction cups
			float effective_sucker_area = manipulators[i].surface_area_m2 * sucker_coverage;
			
			manipulators[i].max_adhesion_force_N = suction_pressure_Pa * effective_sucker_area;
		}
		
		if(has_setae) {
			// Van der Waals adhesion (gecko-like)
			// ~10 N/cm² for optimal setae density
			float setae_stress_Pa = 100000.0f; // 10 N/cm²
			
			// Assume setae cover the contact surface
			manipulators[i].max_adhesion_force_N += 
				setae_stress_Pa * manipulators[i].surface_area_m2;
		}
			
		if(has_wet_grip) {
			// Tree frog adhesion: capillary forces + mucus adhesion
			// ~1-5 N/cm² depending on surface wetness
			// Formula: F = 2πRγ (capillary) + μ×Area (mucus)
			float wet_adhesion_Pa = 30000.0f; // ~3 N/cm² typical
			
			// Pad coverage (tree frogs have ~60% of toe surface as pad)
			float pad_coverage = 0.6f;
			float effective_pad_area = manipulators[i].surface_area_m2 * pad_coverage;
			
			manipulators[i].max_adhesion_force_N = 
				wet_adhesion_Pa * effective_pad_area;
			
			// Requires wet/humid surface - note this in metadata somewhere?
		}
		
		if(manipulators[i].has_friction_pads()) {
			// Primate friction grip (no true adhesion, just high friction)
			// Not really "adhesion" but contributes to grip force
			// Coefficient of friction ~0.5-1.5 for primate skin
			float friction_coef = 1.0f;
			
			// This multiplies the normal force (which comes from grip force)
			// So it enhances grip_force rather than being separate
			manipulators[i].max_grip_force_N *= (1.0f + friction_coef * 0.3f);
		}

		// 5. SCALING ADJUSTMENTS
		// Square-cube law: Force scales with cross-section (area), not volume
		float size_scale = in.area_scale(); // Already accounts for anisotropic scaling
		
		// Apply conservative allometric scaling
		// Smaller animals have relatively stronger muscles (force/mass ratio)
		float allometric_factor = std::pow(size_scale, 0.67f); // Between area (1.0) and volume (0.67)
		
		manipulators[i].max_lift_force_N *= allometric_factor;
		manipulators[i].max_grip_force_N *= allometric_factor;
		manipulators[i].max_adhesion_force_N *= allometric_factor;
		
		// 6. TYPE-SPECIFIC ADJUSTMENTS
		if(HasFlag(manipulators[i].subtree_flags, SF::TENTACLE)) {
			// Tentacles are hydrostatic - different force characteristics
			// Generally weaker in grip but good adhesion
			manipulators[i].max_grip_force_N *= 0.6f;
			if(has_suckers) {
				manipulators[i].max_adhesion_force_N *= 1.5f; // Octopus-level suction
			}
		}
		
		// trunks and monkey tails.
		if(HasFlag(manipulators[i].subtree_flags, SF::FACIAL|SF::TAIL)) {
			// Elephant trunks: excellent lift, moderate grip
			manipulators[i].max_lift_force_N *= 1.4f;
			manipulators[i].max_grip_force_N *= 0.8f;
		}		
	}
	
	return manipulators;

}

std::vector<TonTon::Analysis_Aquatic::Fin> TonTon::Compute_Fins(const Builder &in)
{		
	std::vector<TonTon::Analysis_Aquatic::Fin>  fins;
	fins.reserve(fin_appendages.size());
	
	for(auto & appendage : in.appendages)
	{
		appendage.rest_length_m += glm::length(tails[appendage.tip] - position[appendage.root]);
		appendage.stretched_length_m += glm::length(tails[appendage.tip] - position[appendage.tip]);
		
		Analysis_Aquatic::Fin fin;
		static_cast<Analysis_Appendage&>(fin) = appendage;

		SkinnedMesh::LimbMetrics limb_metrics;
		std::pair<glm::quat, glm::vec3> eigen_decomposition;
	
		// Get fin area from silhouette
		auto fin_joints = sk_memo->GetAllChildrenOfRoot(appendage.root);
		auto projection = in.skinnedMesh->memo()->GetProjectionMatrix(
			EigenValue::Small, 
			in.behavior.scale,
			fin_joints, 
			&limb_metrics, 
			&eigen_decomposition);
			
		auto& silhouette = in.skinnedMesh->memo()->GetSilhouettes(projection, in.behavior.scale, fin_joints);
		
		fin.area_m2 = silhouette.area;
		fin.chord_m = silhouette.MeasureWidth_Segment(position[fin.root], tails[fin.tip]).length;
		
		// aspect ratio wasn't really meaningful because a shark's body is so fat!

		fin.normal_vector = GetProjectionDirection(EigenValue::Small, eigen_decomposition.first);
		fin.type = relative_flags[appendage.root].child_flags|semantic_flags[appendage.root];
		auto surfaceArea = in.skinnedMesh->GetSurfaceArea(fin_joints, area_scale);

		// Add this analysis before your flipper check
		float percent_area = (silhouette.area*2.0) / surfaceArea; // volume-to-area ratio
	//	float thickness = surfaceArea / limb_metrics.volume;
		int joint_count = fin_joints.size();
		float aspect_ratio = fin.rest_length_m / fin.chord_m;
		
		// Flipper characteristics
		bool has_internal_structure = joint_count < 3; // flippers don't have digitgrade etc complexity. usually one bone. 
		bool is_thick_enough = percent_area > 0.75; // tune this value
		bool is_paddle_shaped = aspect_ratio < 3.0f; // flippers are relatively short and wide
		
		bool is_flipper = has_internal_structure && is_thick_enough && is_paddle_shaped;

		if(HasFlag(fin.type, SF::FIN) 
		|| is_flipper)
			fins.push_back(fin);
	}
	
	return fins;
}


static SemanticAnalysis AnalyzeSemantics(SkinnedMesh const& skinnedMesh, Scratch const& scratch) {
    SemanticAnalysis result;
    
    auto semantic_flags = skinnedMesh->skin->memo()->GetSemanticFlags();
    auto tags = skinnedMesh->skin->tags;
    
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
    
    // Check for venom in output structures
    for (auto& tail : scratch.appendages.tails) {
        if (tail.venom.has_value()) {
            result.has_venom = true;
            result.has_weapons = true;
        }
    }
    
    // Determine if predator from morphology
    result.is_predator = result.has_sharp_teeth || result.has_talons || 
                        (result.has_claws && !scratch.climbing);
    
    // Estimate eye size from head and vision system
    if (scratch.sensory.vision) {
        // Larger detection range suggests larger eyes
        float detection = scratch.sensory.vision->detection_range_m;
        float body_length = scratch.physical.body_length_m;
        result.eye_body_ratio = glm::clamp(detection / (body_length * 100.0f), 0.05f, 0.25f);
    }
    
    return result;
}


struct EyeInfo {
    uint16_t joint_index;
    glm::vec3 position;           // In rest pose
    glm::vec3 base_position;      // Where eyestalk attaches (if applicable)
    glm::vec3 pointing_direction; // Forward vector of this eye
    bool is_on_stalk;
    length_m stalk_length_m;
    length_m eye_diameter_m;
    angle_rad mobility_rad;           // How much can it rotate? (stalk vs fixed)
};

static std::vector<EyeInfo> FindEyes(TonTon::SkinnedMesh const& sk) {
	using SF = SemanticFlags;
	
    auto parents = sk.skin->parents.data();
    auto semantic_flags = sk.skin->memo()->GetSemanticFlags();
    
    std::vector<EyeInfo> eyes;
    
    for(uint32_t i = 0; i < sk.skin->names.size(); ++i) 
    {      
        if(!HasFlag(semantic_flags[i], SF::VISION)) continue;
        
        EyeInfo eye;
        eye.joint_index = i;
        eye.position = in.position(i);
        
        // Estimate eye size
        double eye_volume = sk.volume[i] * in.behavior.volume_scale();
        eye.eye_diameter_m = std::cbrt(eye_volume * 6.0 / 3.14159);
        
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
        eye.is_on_stalk = sk.GetStalkData(stalk, body_attachment, i, in.behavior.scale);
        
        if(eye.is_on_stalk) {
            eye.base_position = in.position(stalk.root);
            eye.stalk_length_m = stalk.length_m;
            
            // Eye points away from base
            eye.pointing_direction = glm::normalize(eye.position - eye.base_position);
            
            // Stalks are mobile
            eye.mobility_rad = glm::radians(90.0f);
            
        } else {
            // Fixed eye
            eye.base_position = in.position(body_attachment);
            
            // Eye points radially + respects local rotation
            glm::vec3 radial = glm::normalize(eye.position - eye.base_position);
            glm::quat local_rot = sk.skin->rotation[i];
            glm::vec3 local_forward = local_rot * glm::vec3(0, 0, 1);
            
            eye.pointing_direction = glm::normalize(radial * 0.3f + local_forward * 0.7f);
            eye.mobility_rad = glm::radians(15.0f);
            eye.stalk_length_m = 0.0f;
        }
        
        eyes.push_back(eye);
    }
    
    return eyes;
}


#endif
