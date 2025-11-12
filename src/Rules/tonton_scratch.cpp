#include "tonton_scratch.h"
#include "Memos/tonton_armaturememo.h"
#include "Rules/tonton_aerial.h"
#include "Rules/tonton_sensory.h"
#include "Rules/tonton_specialized.h"
#include "Rules/tonton_serpentine.h"
#include "../include/tonton_input.h"

#include "Memos/tonton_armaturememo.h"
#include "Memos/tonton_skinnedmeshmemo.h"
#include "Memos/tonton_meshmemo.h"
#include "Rules/tonton_behavior.h"
#include "dodeedum.h"

using Warning = TonTon::Output_Diagnostics::Warning::Severity;
using SF = TonTon::SemanticFlags;

namespace TonTon
{
static	Output_Physical ComputePhysical(Input const& in, Scratch & s);

static	std::optional<Output_Terrestrial>  ComputeTerrestrial(Input const& in, Scratch & s);
// ComputeSerpentine is now in tonton_serpentine.cpp
static	std::optional<Output_Aquatic>  ComputeAquatic(Input const& in, Scratch & s);
static	std::optional<Output_Jumping>  ComputeJumping(Input const& in, Scratch & s);
static	std::optional<Output_Climbing>  ComputeClimbing(Input const& in, Scratch & s);
static	std::optional<Output_Brachiation>  ComputeBrachiation(Input const& in, Scratch & s);

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
	
	metabolic   = ComputeMetabolic(in, *this);
	behavior =  ComputeBehavior(in, *this);
	
	
};

using namespace TonTon;

static Output_Physical TonTon::ComputePhysical(Input const& in, Scratch & out)
{
	auto & sk = *in.armature;
	auto * sk_memo = sk.armature->memo();
	
	auto parents = sk.armature->parents.data();
	
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
	auto names			= sk.armature->names.data();
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
				
				auto silhouette = in.armature->memo()->GetSilhouettes(
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
	
	double body_density=glm::mix(700.0, 1050.0, in.average_density);
	auto C = in.armature->GetCovariance(std::span<uint16_t>{}, in.behavior.scale);
	
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
	auto & sk = *in.armature;
	auto * sk_memo = sk.armature->memo();
	
	auto parents =  in.armature->armature->parents.data();
	
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
		
		// for each node that uses this as a root.
		// so we go from each finger and compute distance to shoulder.
		for(auto leaf : leaves)
		{
			if(gcr[node*noJoints + leaf] != node)
				continue;
			
			double length = 0;
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
			}
		}
		
		r.push_back(Output_Chain{
			.root=node,
			.tip=uint16_t(best_leaf),
			.stretched_length_m=float(max_length),
			.rest_length_m=glm::distance(in.position(best_leaf), in.position(node))
		});
	}
	
	return r;
}


std::vector<Output_Chain> TonTon::GetChainsFromRoot(Input const& in, std::span<Word> words)
{
	auto tags = in.armature->armature->tags;
	
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
	auto semantic_flags = in.armature->armature->memo()->GetSemanticFlags();
	
	return ::GetChainsFromRoot(in, [&](int node) -> bool
	{
		return HasFlag(semantic_flags[node], flags);
	});
}

std::vector<Output_Chain> TonTon::GetChainsFromTip(TonTon::Input const& in, SemanticFlags include_flags, SemanticFlags exclude_flags)
{
	std::vector<TonTon::Output_Chain> chains;
	auto & sk = *in.armature;
	auto * sk_memo = sk.armature->memo();
	
	auto dfs_ordering = sk_memo->GetDfsOrdering();
	auto parents       = sk.armature->parents.data();
	auto semantic_flags = sk_memo->GetSemanticFlags();
	auto relative_flags = sk_memo->GetRelativeFlags();
	auto gcr			 = sk_memo->GetGcrTable();

	for(auto i = 0u; i < semantic_flags.size(); ++i)
	{
		if(!HasFlag(semantic_flags[i], include_flags))
			continue;
			
		auto root = i;
		float length = 0;
		
		for(;;)
		{
			auto p = parents[root];
			if(p < 0) break;
			
				
			if(HasFlag(semantic_flags[i], exclude_flags))
				break;
				
			length += glm::distance(in.position(root), in.position(p));
			root = p;
		}
		
		Output_Chain chain{
			.root=uint16_t(root),
			.tip=uint16_t(i),
			.stretched_length_m=length,
			.rest_length_m=glm::distance(in.position(root), in.position(i)),
		};
		
		chains.push_back(chain);
	}
	
	return chains;
}

std::vector<Output_Appendage> TonTon::GetAppendages(Input const& in, std::vector<Output_Chain> && chains)
{
	auto & sk = *in.armature;
	auto parents =  in.armature->armature->parents.data();

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

static	std::optional<Output_Terrestrial>  TonTon::ComputeTerrestrial(Input const& in, Scratch &out) { return {}; }
// ComputeSerpentine is now implemented in tonton_serpentine.cpp
static	std::optional<Output_Aquatic>  TonTon::ComputeAquatic(Input const& in, Scratch &out) { return {}; }
static	std::optional<Output_Climbing>  TonTon::ComputeClimbing(Input const& in, Scratch &out) { return {}; }
static	std::optional<Output_Brachiation>  TonTon::ComputeBrachiation(Input const& in, Scratch &out) { return {}; }
static	std::optional<Output_Jumping>  TonTon::ComputeJumping(Input const& in, Scratch &out) { return {}; }

static	std::vector<Output_Manipulator>   TonTon::ComputeManipulation(Input const& in, Scratch &) 
{ 
	// walk back parents until we get something thats not limb-ish
	SF constexpr NOT_LIMB_FLAGS = SF(
		int64_t(SF::HEAD)|
		int64_t(SF::NECK)|
		int64_t(SF::SPINE)|
		int64_t(SF::ABDOMEN)
	);
		
	auto & sk = *in.armature;
	auto * sk_memo = sk.armature->memo();
	auto position = sk.armature->position.data();
	auto parents = sk.armature->parents.data();
	
	auto appendages = GetAppendages(in, GetChainsFromTip(in, SF::GRASPER, NOT_LIMB_FLAGS));
	auto relative_flags = in.armature->armature->memo()->GetRelativeFlags();
	auto semantic_flags = in.armature->armature->memo()->GetSemanticFlags();
	
	std::vector<Output_Manipulator> manipulators(appendages.size());
	
	for(auto i = 0u; i < appendages.size(); ++i)
	{
		static_cast<Output_Appendage&>(	manipulators[i]) = appendages[i];
		auto idx = manipulators[i].tip;
		
		auto relevant_joints = sk_memo->GetAllChildrenOfRoot(manipulators[i].root);
		// get all *excluding grasper itself* (just fingers)
		auto which = std::span<uint16_t>(relevant_joints.data(), relevant_joints.size()-1);
			
		auto metrics = in.armature->GetMetrics(which, in.behavior.scale);
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
			auto projection = in.armature->memo()->GetProjectionMatrix(EigenValue::Small, in.behavior.scale, which, &metrics, &eigen_decomp);
			silhouette = in.armature->memo()->GetSilhouettes(projection, in.behavior.scale, which);
			
			// get projection direction.
			manipulators[i].surface_area_m2 = silhouette.area;
			manipulators[i].surface_normal =  glm::mat3(eigen_decomp.first)[2];
		}
		
		auto counter = 0;
		float accumulator = 0;
		float accumulator_2nd_moment = 0;
		for(auto j = manipulators[i].tip; j != manipulators[j].root; j = parents[j])
		{
			auto p =  parents[j];
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
			manipulators[i].max_grip_force_N = muscle_stress_Pa * grip_muscle_area_m2;
			
			// 3. LIFT FORCE (constrained by joint torque limits)
			// Torque = Force × moment_arm
			// Max torque ≈ muscle_PCSA × muscle_stress × moment_arm
			float moment_arm_m = std::sqrt(avg_moment / 3.14159f); // Approximate from 2nd moment
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
			for(auto word : sk.armature->tags[node])
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
		float size_scale = in.behavior.area_scale(); // Already accounts for anisotropic scaling
		
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

static	std::vector<Output_Tail>   TonTon::ComputeTails(Input const& in, Scratch &out) { return {}; }
