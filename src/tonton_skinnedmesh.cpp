#include "dodeedum_mesh.h"
#include "Memos/tonton_armaturememo.h"
#include "Memos/tonton_skinnedmeshmemo.h"
#include "Memos/tonton_meshmemo.h"
#include "tonton_eigen.h"
#include "../include/tonton.h"
#include <cfloat>


TonTon::Armature::Armature() = default;
TonTon::Armature::~Armature() = default;
TonTon::Mesh::Mesh() = default;
TonTon::Mesh::~Mesh() = default;
TonTon::SkinnedMesh::SkinnedMesh() = default;
TonTon::SkinnedMesh::~SkinnedMesh() = default;

counted_ptr<const TonTon::Armature> TonTon::Armature::Factory(
		immutable_array<std::string> names, 
		immutable_array<int> parents,
		immutable_array<glm::vec3>	position,
		immutable_array<glm::quat> rotation,
		immutable_array<immutable_array<Word>> tags)
{
	counted_ptr<TonTon::Armature> r = UncountedWrap(new TonTon::Armature());
	r->names = std::move(names);
	r->parents = std::move(parents);
	r->position = std::move(position);
	r->rotation = std::move(rotation);
	r->tags = tags;
	
	if(r->tags.empty())
	{
		std::vector<Word> words;
				
		r->tags = shared_array<immutable_array<Word>>::Build(r->names.size(), 
			[&](int i)
			{
				words.clear();
				StringToWords(words, r->names[i]);
				
				return shared_array<Word>::FromArray(words);
			});
	}
	
	return r;
}

counted_ptr<const TonTon::Mesh> TonTon::Mesh::Factory(DoDeeDum::Mesh && m)
{
	counted_ptr<TonTon::Mesh> r = UncountedWrap(new TonTon::Mesh());
	r->mesh = std::move(m);
	return r;
}

counted_ptr<const TonTon::SkinnedMesh> TonTon::SkinnedMesh::Factory(
		counted_ptr<const Mesh>	mesh, 
		counted_ptr<const Armature> armature,
		
		immutable_array<Cube>		aabb,
		immutable_array<float>		surfaceArea,
			
		immutable_array<float>		volume,
		immutable_array<glm::vec3>	centroid,
		immutable_array<std::array<float, 6>>	covariance)
{
	counted_ptr<TonTon::SkinnedMesh> r = UncountedWrap(new TonTon::SkinnedMesh());
	
	r->mesh = std::move(mesh);
	r->skin = std::move(armature);
	
	r->aabb = std::move(aabb);
	r->surfaceArea = std::move(surfaceArea);
	
	r->volume = std::move(volume);
	r->centroid = std::move(centroid);
	r->covariance = std::move(covariance);
	
	return r;
}

TonTon::ArmatureMemo * TonTon::Armature::memo() const
{
	if(_memo == nullptr)
		_memo = std::make_unique<ArmatureMemo>(this);

	return _memo.get();
}
	
TonTon::MeshMemo * TonTon::Mesh::memo() const
{
	if(_memo == nullptr)
		_memo = std::make_unique<MeshMemo>(this);

	return _memo.get();
}
	
TonTon::SkinnedMeshMemo * TonTon::SkinnedMesh::memo() const
{
	if(_memo == nullptr)
		_memo = std::make_unique<SkinnedMeshMemo>(this);

	return _memo.get();
}


glm::dvec3 TonTon::SkinnedMesh::GetCentroid(std::span<uint16_t> joints, glm::dvec3 const& scale, double * volume_out) const
{
	glm::dvec3 numerator{0};
	double denominator{0};
	
	double vol_scale = scale.x*scale.y*scale.z;
	
	if(joints.size())
	{
		for(auto & i : joints)
		{
			auto vol = volume[i] * vol_scale;
			numerator += glm::dvec3(centroid[i]) * scale * vol;
			denominator += vol;
		}
	}
	
	else
	{
		for(auto i = 0u; i < centroid.size(); ++i)
		{
			auto vol = volume[i] * vol_scale;
			numerator += glm::dvec3(centroid[i]) * scale * double(vol);
			denominator += vol;
		}
	}
	
	if(volume_out) *volume_out = denominator;
	
	if(denominator)
		return numerator / denominator;
	
	return numerator;
}

glm::dmat3 TonTon::SkinnedMesh::GetInertia(std::span<uint16_t> joints, glm::dvec3 const& scale, glm::dvec3 *centroid_out, double * volume_out) const
{
	auto I = GetCovariance(joints, scale, centroid_out, volume_out);
	
	return glm::dmat3{
		 I[1] +I[2],-I[3],-I[4],
		-I[3], I[0] +I[2],-I[5],
		-I[4],-I[5], I[0] +I[1]
	};
}

glm::dmat3 TonTon::SkinnedMesh::GetInertia(uint32_t i, glm::dvec3 scale) const
{
	std::array<double, 6> I = GetCovariance(i, scale);
		
	return glm::dmat3{
		 I[1] +I[2],-I[3],-I[4],
		-I[3], I[0] +I[2],-I[5],
		-I[4],-I[5], I[0] +I[1]
	};
}

std::array<double, 6>  TonTon::SkinnedMesh::GetCovariance(uint32_t i, glm::dvec3 scale) const
{
	return
	{
		covariance[i][0] * scale.x*scale.x,
		covariance[i][1] * scale.y*scale.y,
		covariance[i][2] * scale.z*scale.z,
		covariance[i][3] * scale.x*scale.y,
		covariance[i][4] * scale.x*scale.z,
		covariance[i][5] * scale.y*scale.z
	};
}

	
static double EstimateCrossSection(std::array<double, 6> const& I, double volume, glm::vec3 direction, double * second_moment_area);

double  TonTon::SkinnedMesh::EstimateCrossSection(std::span<uint16_t> joints, glm::dvec3 scale, glm::vec3 direction, double * second_moment_area) const
{
	double volume;
	auto cov = GetCovariance(joints, scale, nullptr, &volume);
	
	return ::EstimateCrossSection(cov, volume, direction, second_moment_area);
}
	
double TonTon::SkinnedMesh::EstimateCrossSection(uint32_t i, glm::dvec3 scale, glm::vec3 direction, double * second_moment_area) const
{
	return ::EstimateCrossSection(GetCovariance(i, scale), volume[i], direction, second_moment_area);
}
   
static double EstimateCrossSection(std::array<double, 6> const& I, double volume, glm::vec3 direction, double * second_moment_area) 
{
    // its covariance * volume already 
    // so inertia is shuffled covariance * density
    // meaning that units are M^5
    glm::dmat3 cov{
        I[0], I[3], I[4],
        I[3], I[1], I[5],
        I[4], I[5], I[2]
    };
    
    cov = cov / double(volume? volume :  1.0);
   
    glm::dvec3 d = glm::normalize(direction);
    
    // Project covariance onto plane perpendicular to d
    // Create projection matrix: P = I - d⊗d
    glm::dmat3 proj = glm::dmat3(1.0) - glm::outerProduct(d, d);
    glm::dmat3 projected_cov = proj * cov * proj;
    
	std::pair<glm::quat, glm::vec3> eigen_decomp = TonTon::EigenDecomposition(projected_cov);
	
    auto & eigenvalues = eigen_decomp.second;
    
    // eigenvalues are sorted smallest to largest
    // The two largest (y and z) define the ellipse in the perpendicular plane
    double a = 2.0 * std::sqrt(std::abs(eigenvalues.y));
    double b = 2.0 * std::sqrt(std::abs(eigenvalues.z));
    
    // Second moment of area
    if (second_moment_area) {
        // For a solid ellipse: I = (π/4) * a * b³ (about major axis)
        // and I = (π/4) * a³ * b (about minor axis)
        
        // Typically for bending strength, use the minimum (weakest axis)
        double I_major = (glm::pi<double>() / 4.0) * a * b * b * b;
        double I_minor = (glm::pi<double>() / 4.0) * a * a * a * b;
        
        *second_moment_area = std::min(I_major, I_minor);
        
        // Or if you want both or the polar moment (for torsion):
        // Polar moment: J = I_major + I_minor = (π/4) * a * b * (a² + b²)
        // *second_moment_area = (glm::pi<double>() / 4.0) * a * b * (a*a + b*b);
    }
    
    // Cross-sectional area (ellipse)
    auto elipse_cross_section = glm::pi<double>() * a * b;
    return elipse_cross_section;
}
	
std::array<double, 6>  TonTon::SkinnedMesh::GetCovariance(std::span<uint16_t> joints, glm::dvec3 const& scale, glm::dvec3 *centroid_out, double * volume_out) const
{

	glm::dvec3 center = GetCentroid(joints, scale, volume_out);
	if(centroid_out) *centroid_out = center;
	
	if(joints.size() == 1)
		return GetCovariance(joints[0], scale);

	std::array<double, 6> accumulator{0};
	double vol_scale = scale.x*scale.y*scale.z;
	
	auto axis_theorem = [&](size_t i)
	{
		glm::dvec3 c = glm::dvec3(centroid[i])*scale - center;
		double mulv =  (volume[i] * vol_scale);
		
		std::array<double, 6> offset
		{
			c.x*c.x * mulv,
			c.y*c.y * mulv,
			c.z*c.z * mulv,
			
			c.x*c.y * mulv, 
			c.x*c.z * mulv,
			c.y*c.z * mulv,
		};
	
		return offset;
	};
	
	auto opAdd = [](std::array<double, 6> const& a, std::array<double, 6> const& b)
	{
		return std::array<double, 6>{
			a[0]+b[0], a[1]+b[1], a[2]+b[2],
			a[3]+b[3], a[4]+b[4], a[5]+b[5]
		};
	};
	
	if(joints.size())
	{
		for(auto & i : joints)
		{
			accumulator = opAdd(accumulator, opAdd(GetCovariance(i, scale), axis_theorem(i)));
		}
	}
	
	else
	{
		for(auto i = 0u; i < centroid.size(); ++i)
		{
			accumulator = opAdd(accumulator, opAdd(GetCovariance(i, scale), axis_theorem(i)));
		}
	}
	
	
	return accumulator;
}

double TonTon::SkinnedMesh::GetSurfaceArea(uint16_t i, double area_scale) const
{
	return surfaceArea[i] * area_scale;
}

double TonTon::SkinnedMesh::GetSurfaceArea(std::span<uint16_t> joints, double area_scale) const
{
	double accumulator = 0;
	
	for(auto i : joints)
		accumulator += surfaceArea[i] * area_scale;
		
	return accumulator;
}

TonTon::SkinnedMesh::LimbMetrics TonTon::SkinnedMesh::GetMetrics(std::span<uint16_t> joints, glm::dvec3 const& scale) const
{
	LimbMetrics r;
	r.unitInertia = GetInertia(joints, scale, &r.centroid, &r.volume);
	return r;
}

glm::dmat3 TonTon::SkinnedMesh::LimbMetrics::GetInertia(glm::vec3 measured_at, float density) const
{
    // Start with the inertia tensor at the centroid
    glm::dmat3 inertia_at_centroid = unitInertia * (volume * density);
    
    // Apply parallel axis theorem to shift to the measurement point
    glm::dvec3 offset = glm::dvec3(measured_at) - centroid;
    double mass = volume * density;
    
    // Parallel axis theorem: I = I_cm + m * (|r|² * I₃ - r ⊗ r)
    double offset_squared = glm::dot(offset, offset);
    glm::dmat3 offset_tensor = glm::outerProduct(offset, offset);
    
    glm::dmat3 parallel_axis_term = mass * (offset_squared * glm::dmat3(1.0) - offset_tensor);
    
    return inertia_at_centroid + parallel_axis_term;
}

double TonTon::SkinnedMesh::LimbMetrics::GetInertia(glm::vec3 measured_at, float density, glm::vec3 axis) const
{
    // Get the full inertia tensor at the measurement point
    glm::dmat3 inertia = GetInertia(measured_at, density);
    
    // Project onto the specified axis: I_axis = axis^T * I * axis
    glm::dvec3 axis_normalized = glm::normalize(glm::dvec3(axis));
    
    return glm::dot(axis_normalized, inertia * axis_normalized);
}

bool  TonTon::SkinnedMesh::GetStalkData(StalkData & dst, int root, int tip, glm::vec3 scale) const
{
    auto parents = skin->parents.data();
    auto position = skin->position.data();
    auto children = skin->memo()->GetChildren();
    
    // Build the chain from tip back to root
    std::vector<int> chain;
    for(int j = tip; j != root && j >= 0; j = parents[j]) {
        chain.push_back(j);
        if(j == root) break;
    }
    chain.push_back(root);
    std::reverse(chain.begin(), chain.end()); // Now root -> tip
    
    if(chain.size() < 2) {
        return false; // Need at least 2 joints for a stalk
    }
    
    struct Memo
    {
		glm::vec3 delta{};
		float length{};
		float thickness{};
		int branches{};
    };

    std::vector<Memo> memo(chain.size()-1);
    
    for(auto i = 0u; i < chain.size()-1; ++i)
    {
		memo[i].delta = position[chain[i+1]] - position[chain[i]];
		memo[i].length = glm::length(memo[i].delta);
		memo[i].thickness = EstimateCrossSection(chain[i], scale, memo[i].delta);
		// stalks only have one child.
		memo[i].branches = children[chain[i]].size() > 1;
    }
    
    // ===================================================================
    // SCAN FOR STALK-LIKE REGIONS
    // ===================================================================
    // A stalk is:
    // - Long and thin (high aspect ratio)
    // - Relatively uniform thickness
    // - Elongated inertia tensor (one eigenvalue >> others)
    
    struct RegionCandidate {
        int start_idx{}; // Index into chain[]
        int end_idx{};
        float score{};   // Higher = more stalk-like
        float length{};
        float avg_thickness{};
        float thickness_variation{}; // Lower = more uniform
    };
    
    std::vector<RegionCandidate> candidates;
    
    // Try all possible sub-chains of length >= 2
    for(size_t start = 0; start < chain.size(); ++start) {
        for(size_t end = start + 1; end < chain.size()-1; ++end) {
            RegionCandidate candidate;
            candidate.start_idx = start;
            candidate.end_idx = end;
            
            // Compute length
            candidate.length = 0.0f;
            
            auto region = std::span(memo).subspan(start, end-start);
            
            Memo accumulator;
            bool has_branches = false;
            float sum = 0, sum_sq = 0;
            float min_cs = FLT_MAX, max_cs = 0;
            
            for(auto & item : region) {
				accumulator.branches += item.branches;
				accumulator.delta += item.delta;
				accumulator.length += item.length;
				accumulator.thickness += item.thickness;
				
				auto cs = item.thickness;
                sum += cs;
                sum_sq += cs * cs;
                min_cs = std::min(min_cs, cs);
                max_cs = std::max(max_cs, cs);
            }
            
            glm::vec3 stalk_direction = accumulator.delta;
            candidate.length = accumulator.length;
            
            // stalks don't branch
            if(has_branches) continue;
            if(candidate.length < 0.001f) continue; // Too short to be a stalk;
            
            stalk_direction = glm::normalize(stalk_direction);
                        
            float mean = sum / region.size();
            float variance = (sum_sq / region.size()) - (mean * mean);
            float std_dev = std::sqrt(std::max(0.0f, variance));
            
            candidate.avg_thickness = mean;
            candidate.thickness_variation = std_dev / (mean + 1e-6f); // Coefficient of variation
            
            // ===============================================================
            // STALK SCORE - higher is more stalk-like
            // ===============================================================
            
            // 1. Aspect ratio (length / diameter)
            float avg_diameter = std::sqrt(mean / 3.14159f) * 2.0f;
            float aspect_ratio = candidate.length / (avg_diameter + 1e-6f);
            
            // Good stalks have aspect ratio > 5, excellent stalks > 10
            float aspect_score = std::clamp(aspect_ratio / 10.0f, 0.0f, 1.0f);
            
            // 2. Uniformity (low variation in thickness)
            float uniformity_score = 1.0f / (1.0f + candidate.thickness_variation * 10.0f);
            
            // 3. Check inertia tensor elongation for the whole region
            std::vector<uint16_t> region_joints;
            for(size_t i = start; i <= end; ++i) {
                region_joints.push_back(chain[i]);
            }
            
            auto metrics = GetMetrics(region_joints, scale);
            auto inertia = metrics.GetInertia(metrics.centroid, 1.0f); // Unit density
            
            auto eigen_decomp = EigenDecomposition(inertia);
            glm::vec3 eigenvalues = eigen_decomp.second;
            
            // For a long thin rod: λ₀ << λ₁ ≈ λ₂
            // Check if smallest eigenvalue is much smaller than others
            float elongation_ratio = (eigenvalues[1] + eigenvalues[2]) / (2.0f * eigenvalues[0] + 1e-6f);
            float elongation_score = std::clamp((elongation_ratio - 3.0f) / 7.0f, 0.0f, 1.0f); // 3-10 range
            
            // 4. Straightness - how aligned are segments?
            float straightness = 0.0f;
            if(end > start + 1) {
                glm::vec3 overall_direction = glm::normalize(
                    position[chain[end]] * scale - position[chain[start]] * scale
                );
                
                float alignment_sum = 0.0f;
                for(size_t i = start; i < end; ++i) {
                    glm::vec3 p0 = position[chain[i]] * scale;
                    glm::vec3 p1 = position[chain[i+1]] * scale;
                    glm::vec3 seg_dir = glm::normalize(p1 - p0);
                    alignment_sum += glm::dot(seg_dir, overall_direction);
                }
                straightness = alignment_sum / (end - start);
            } else {
                straightness = 1.0f; // Single segment is perfectly straight
            }
            float straightness_score = (straightness + 1.0f) * 0.5f; // Map [-1,1] to [0,1]
            
            // COMBINED SCORE
            candidate.score = 
                aspect_score * 0.35f +
                uniformity_score * 0.25f +
                elongation_score * 0.25f +
                straightness_score * 0.15f;
            
            // Bonus for longer stalks (prefer full chains over fragments)
            float length_bonus = std::clamp(candidate.length / 0.1f, 1.0f, 1.5f);
            candidate.score *= length_bonus;
            
            // Penalty for regions that are too short to be meaningful stalks
            if(candidate.length < 0.01f) {
                candidate.score *= 0.1f;
            }
            
            candidates.push_back(candidate);
        }
    }
    
    if(candidates.empty()) {
        return false;
    }
    
    // ===================================================================
    // SELECT BEST STALK REGION
    // ===================================================================
    
    // Find highest scoring candidate
    auto best = std::max_element(candidates.begin(), candidates.end(),
        [](auto const& a, auto const& b) { return a.score < b.score; });
    
    // Require minimum score to be considered a "stalk"
    if(best->score < 0.3f) {
        return false; // Not stalk-like enough
    }
    
    // Fill output
    dst.root = chain[best->start_idx];
    dst.tip = chain[best->end_idx];
    dst.length_m = best->length;
    
    // Re-compute exact min/max cross-sections for the selected region
    dst.thickestCrossSection_m2 = 0.0f;
    dst.thinestCrossSection_m2 = FLT_MAX;
    
    glm::vec3 stalk_dir{0};
    for(int i = best->start_idx; i < best->end_idx; ++i) {
        glm::vec3 p0 = position[chain[i]] * scale;
        glm::vec3 p1 = position[chain[i+1]] * scale;
        stalk_dir += glm::normalize(p1 - p0);
    }
    stalk_dir = glm::normalize(stalk_dir);
    
    for(int i = best->start_idx; i <= best->end_idx; ++i) {
        double cs = EstimateCrossSection(chain[i], scale, stalk_dir);
        dst.thickestCrossSection_m2 = std::max(dst.thickestCrossSection_m2, (float)cs);
        dst.thinestCrossSection_m2 = std::min(dst.thinestCrossSection_m2, (float)cs);
    }
    
    return true;
}

