#include "tonton_skinnedmeshmemo.h"
#include "Memos/tonton_armaturememo.h"
#include <iostream>
#include "Memos/tonton_meshmemo.h"
#include "../include/tonton_input.h"
#include "tonton_eigen.h"
#include "dodeedum_mesh.h"
#include <functional>
#include <unordered_set>

static std::vector<std::vector<int>> GetCliques(std::vector<std::pair<int, int>> const& edges);

TonTon::SkinnedMeshMemo::SkinnedMeshMemo(SkinnedMesh const* ptr) : in(*ptr)
{
}

TonTon::SkinnedMeshMemo::~SkinnedMeshMemo() = default;

TonTon::Silhouette & TonTon::SkinnedMeshMemo::GetSilhouettes(glm::mat4 const& projection, glm::vec3 scale, std::span<uint16_t> joints,  float cutoff, bool secondMoment)
{
	return in.mesh->memo()->GetSilhouettes(projection, scale, joints, cutoff, secondMoment);
}

glm::vec3 TonTon::GetTangentAxis(EigenValue projection, glm::quat eigen_direction, glm::vec3 root, glm::vec3 centroid)
{
	auto UP = GetProjectionDirection(projection, eigen_direction);
	auto FWD = glm::normalize(centroid - root);
	
	return glm::cross(UP, FWD);
}

glm::mat4 TonTon::GetProjectionMatrix(EigenValue projection, glm::quat eigen_direction)
{
	glm::mat3 rotation = glm::mat3(eigen_direction);
	rotation = glm::transpose(rotation);

	switch(projection)
	{
	case EigenValue::Small:
		// View along the axis with smallest eigenvalue (thinnest direction)
		return glm::mat4(rotation);  // rotation columns are the eigenvectors
	case EigenValue::Medium:
		// Rotate so we're looking along the medium eigenvector
		return glm::mat4(
			glm::vec4(rotation[2], 0), 
			glm::vec4(rotation[1], 0), 
			glm::vec4(rotation[0], 0), 
			glm::vec4(0,0,0,1)
		);
	case EigenValue::Large:
		// View along the axis with largest eigenvalue (widest direction)
		return glm::mat4(
			glm::vec4(rotation[1], 0), 
			glm::vec4(rotation[2], 0), 
			glm::vec4(rotation[0], 0), 
			glm::vec4(0,0,0,1)
		);
	}
	
	return glm::mat4(1);
}

// like if its a plane we want the normal of the plane from EigenValue::Smal?
glm::vec3 TonTon::GetProjectionDirection(EigenValue projection, glm::quat eigen_direction)
{
    glm::mat3 rotation = glm::mat3(eigen_direction);
 //   rotation = glm::transpose(rotation);

    switch(projection)
    {
    case EigenValue::Small:
        return rotation[2];  // Third column (z-axis)
    case EigenValue::Medium:
        return rotation[0];  // First column (x-axis)
    case EigenValue::Large:
        return rotation[1];  // Second column (y-axis)
    }
    
    return glm::vec3(0, 0, 1);
}

glm::mat4 TonTon::SkinnedMeshMemo::GetProjectionMatrix(EigenValue projection, glm::vec3 scale, std::span<uint16_t> joints, SkinnedMesh::LimbMetrics * metrics, std::pair<glm::quat, glm::vec3> * eigen_decomp) const
{
	auto m = in.GetMetrics(joints, scale);
	// convert inertia tensor to [rotation, eigenvectors, form]
	auto[rotation_q, vectors] = EigenDecomposition(m.unitInertia);
	auto position = in.skin->position.data();
	
	glm::mat4 matrix = ::TonTon::GetProjectionMatrix(projection, rotation_q);
	
	if(joints.size())
	{
		auto offset = -matrix * glm::vec4(position[joints[0]] * scale, 1);
		if(offset.w)
		{
			(glm::vec3&)matrix[3] = offset / offset.w;
		}
	}
	
	if(metrics)  *metrics = m;
	if(eigen_decomp) *eigen_decomp = {rotation_q, vectors};
	return matrix; 
}
	
TonTon::Silhouette & TonTon::SkinnedMeshMemo::GetSilhouettes(EigenValue projection, glm::vec3 scale, std::span<uint16_t> joints, float cutoff, bool secondMoment) 
{
	std::lock_guard lock(_mutex);
	
	Key key;
	key.cutoff = std::clamp<int>(cutoff * 255, 0, 255);
	key.eigenValue = int(projection);
	key.second_moment=secondMoment;
	key.joints = in.mesh->memo()->get_index(joints);
	
	ScaledKey skey = {key, scale};
	
	auto itr = _cache.find(skey);
	
	if(itr != _cache.end())
		return *itr->second;

	glm::mat4 matrix = GetProjectionMatrix(projection, scale, joints);
	
	auto & s = in.mesh->memo()->GetSilhouettes(matrix, scale, {(uint16_t*)joints.data(), joints.size()}, cutoff, secondMoment);
	_cache[skey] = &s;
	return s;
}

TonTon::Silhouette & TonTon::SkinnedMeshMemo::GetSilhouettes(Axis axis, glm::vec3 scale, std::span<uint16_t> joints,  float cutoff, bool secondMoment) 
{	
	glm::mat4 projection_matrix{0};
	projection_matrix[0][(int(axis)+1)%3] = 1;
	projection_matrix[1][(int(axis)+2)%3] = 1;
	projection_matrix[2][(int(axis)+3)%3] = 1;
	projection_matrix[3][3] = 1;
	
	return in.mesh->memo()->GetSilhouettes(projection_matrix, scale, joints, cutoff, secondMoment);
}
	

immutable_array<float>  TonTon::SkinnedMeshMemo::GetTubeTable()
{
	if(_tubeTable != nullptr) 
		return _tubeTable;
	
	auto children = in.skin->memo()->GetChildren();
	auto parents = in.skin->parents;
	
	shared_array<float> r(parents.size(), 0);
	
	for(auto i = 0u; i < parents.size(); ++i)
	{
		r[i] = in.mesh->memo()->GetTubiness(parents[i], i, std::span(children[i].data(), children[i].size()));
	}
	
	return (_tubeTable = r);
}
	
immutable_array<TonTon::SkinnedMeshMemo::Clique> TonTon::SkinnedMeshMemo::GetCliques()
{
	std::lock_guard lock(_mutex);
	if(cliques.size()) return cliques;
	
	auto children = in.skin->memo()->GetChildren();
	auto position = in.skin->position.data();
	auto names = in.skin->names.data();
	
	std::vector<Clique> r;

// step 1: construct a graph of what is symmetrical with what.
	std::vector<std::pair<int, int>> edges;
	edges.reserve(8);		

	Cube aabb = in.aabb[0];
	
	for(auto & box : in.aabb)
	{
		aabb = aabb | box;
	}

	double general_scale = std::cbrt((aabb.max.x - aabb.min.x) * (aabb.max.y - aabb.min.y) * (aabb.max.z - aabb.min.z));
	general_scale = 1.0 / general_scale;
	
	for(auto p = 0u; p < children.size(); ++p)
	{
		edges.clear();
		auto & group = children[p];
		auto N = group.size();
		
		if(N <= 1) 
			continue;
		
		for(auto idx0 = 0u; idx0 < N; ++idx0)
		{
			for(auto idx1 = idx0+1; idx1 < N; ++idx1)
			{
				auto i = group[idx0];
				auto j = group[idx1];
				
				glm::vec3 point = (position[i] + position[j]) / 2.f;
				glm::vec3 normal = glm::normalize(position[i] - position[j]);
			
			// project onto plane.
				glm::vec3 delta = position[p] - point;
				float projection = std::fabs(dot(delta, normal)) * general_scale;
			
			// symmetrical
				if(projection < 0.001)
				{
					edges.push_back({i, j});
				}
			}
		}
	
		if(edges.empty())
			continue;
		if(edges.size() == 1)
		{
	//		std::cout << names[edges[0].first] << "\n";
	//		std::cout << names[edges[0].second] << "\n";
	//			std::cout << "\n";
					
			r.push_back({
				.parent=p,
				.children=shared_array<int32_t>::FromArray(&edges[0].first, 2)
			});		
				
			continue;
		}		
	
	
		auto cliques = ::GetCliques(edges);
		
		if(cliques.size() > 0)
		{
			for(auto & c : cliques)
			{
				/*for(auto joint : c)
				{
					std::cout << names[joint] << "\n";
				}
				
				std::cout << "\n";*/
			
				r.push_back({
					.parent=p,
					.children=shared_array<int32_t>::FromArray(c.data(), c.size())
				});
			}
		}
	}
	
	return (cliques = shared_array<Clique>::FromArray(r.data(), r.size()));
}


static std::vector<std::vector<int>> GetCliques(std::vector<std::pair<int, int>> const& edges) {
    // Build adjacency list
    std::unordered_map<int, std::unordered_set<int>> adj;
    std::unordered_set<int> all_vertices;
    
    for (const auto& [u, v] : edges) {
        adj[u].insert(v);
        adj[v].insert(u);
        all_vertices.insert(u);
        all_vertices.insert(v);
    }
    
    std::vector<std::vector<int>> cliques;
    
    // Bron-Kerbosch algorithm
    std::function<void(std::unordered_set<int>, std::unordered_set<int>, std::unordered_set<int>)> 
    bronKerbosch = [&](std::unordered_set<int> R, std::unordered_set<int> P, std::unordered_set<int> X) {
        if (P.empty() && X.empty()) {
            // Found a maximal clique
            auto clique = std::vector<int>(R.begin(), R.end());
            if(clique.size() > 1)
            {
				std::sort(clique.begin(), clique.end());
				cliques.push_back(std::move(clique));
            }
            return;
        }
        
        // Choose pivot with most connections in P
        int pivot = P.empty() ? *X.begin() : *P.begin();
        
        // Make a copy of P to iterate over
        auto P_copy = P;
        for (int v : P_copy) {
            // Skip neighbors of pivot for efficiency
            if (adj[pivot].count(v)) continue;
            
            // Build neighbors set
            std::unordered_set<int> neighbors = adj[v];
            
            // R ∪ {v}
            std::unordered_set<int> R_new = R;
            R_new.insert(v);
            
            // P ∩ N(v)
            std::unordered_set<int> P_new;
            for (int n : P) {
                if (neighbors.count(n)) P_new.insert(n);
            }
            
            // X ∩ N(v)
            std::unordered_set<int> X_new;
            for (int n : X) {
                if (neighbors.count(n)) X_new.insert(n);
            }
            
            bronKerbosch(R_new, P_new, X_new);
            
            P.erase(v);
            X.insert(v);
        }
    };
    
    // Start algorithm
    std::unordered_set<int> R, X;
    std::unordered_set<int> P = all_vertices;
    bronKerbosch(R, P, X);
    
    return cliques;
}


/// gltf doesn't store tails its a problem... 
immutable_array<glm::vec3>  TonTon::SkinnedMeshMemo::GetBoneTails()
{
	if(_boneTails.size())	return _boneTails;

	double accumulator = 0;
	int counter = 0;
	
	auto positions = in.skin->position;
	auto parents   = in.skin->parents;
	auto rotations = in.skin->rotation.data();
	
	shared_array<glm::vec3> result = positions.clone();
	shared_array<glm::vec3> pointing(positions.size(), glm::vec3(0));
	auto children = in.skin->memo()->GetChildren();
	
	for(auto i = 0u; i < children.size(); ++i)
	{
		if(children[i].size() != 1)
			continue;
	
		result[i] = positions[children[i][0]];
		
		pointing[i] = glm::normalize(result[i] - positions[i]);
		
		if(rotations)
		{
			accumulator += glm::dot(pointing[i], rotations[i] * glm::vec3(0, 1, 0));
			++counter;
		}
	}
	
	if(counter)
	{
		accumulator = glm::clamp(accumulator / counter, -1.0, 1.0);
	}
	
	for(auto i = 0u; i < children.size(); ++i)
	{
		if(children[i].size() <= 1) continue;
	
		glm::vec3 sum{0};
		
		for(auto child : children[i])
			sum += positions[child];
		
		sum /= children[i].size();
	
		// basically weight how much we trust the rotation by how well it corresponds to previous data. 
		pointing[i] = glm::normalize(sum - positions[i]);
		
		if(rotations)
			pointing[i] = glm::mix(
				pointing[i], 
				rotations[i] * glm::vec3(0, 1, 0), 
				accumulator);
				
		result[i] = positions[i] + pointing[i] * glm::dot(pointing[i], sum - positions[i]);
	}
	
	// leaf bones
	for(auto i = 0u; i < children.size(); ++i)
	{
		if(children[i].empty() == false) continue;
		
		if(parents[i] >= 0)
		{
			result[i] = positions[i] + (positions[i] - positions[parents[i]]) * 0.01f;
			pointing[i] = glm::normalize(positions[i] - positions[parents[i]]);
		}
		
		if(in.surfaceArea[i] == 0) 
			continue;
			
		if(rotations)
			pointing[i] = glm::mix(
				pointing[i], 
				rotations[i] * glm::vec3(0, 1, 0), 
				accumulator);
				
		glm::vec3 furthest_vert = positions[i];
		double furthest_distance_2 = 0;
		float furthest_projected_vert = 0;
			
		for(auto & primitive : in.mesh->mesh)
		{
			primitive.for_each_vertex([&](DoDeeDum::Vert & vert)
			{
				float weight = 0;
				for(auto j = 0u; j < 4; ++j)
				{
					weight += vert.weights[j] * (vert.joints[j] == i);
				}
				
				if(weight)
				{
					auto vec = glm::vec3(vert.position) - positions[i];
					auto len2 = glm::dot(vec, vec);
					
					if(len2 > furthest_distance_2)
					{
						furthest_distance_2 = len2;
						furthest_vert = vert.position;
					}
					
					furthest_projected_vert = glm::max(furthest_projected_vert, glm::dot(vec, pointing[i]));					
				}
				
			// don't short circuit
				return false;
			});
		}
		
		result[i] = positions[i] + glm::mix(
			furthest_vert, 
			pointing[i] * furthest_projected_vert, 
			accumulator);	
	}
	
	return (_boneTails = result);
}
