#ifndef TONTON_SKINNED_MESH_MEMO_H
#define TONTON_SKINNED_MESH_MEMO_H
#include "../include/tonton_input.h"
#include <cstdint>
#include <glm/gtc/quaternion.hpp>
#include <glm/vec3.hpp>
#include <glm/mat3x3.hpp>
#include <map>
#include <span>

namespace DoDeeDum { struct Silhouette; }

namespace TonTon
{

struct SkinnedMesh;
using Silhouette = DoDeeDum::Silhouette;

enum class Axis
{
	X, Y, Z
};

enum class EigenValue {
	Small,
	Medium,
	Large
};

glm::mat4 GetProjectionMatrix(EigenValue projection, glm::quat eigen_direction);
glm::vec3 GetProjectionDirection(EigenValue projection, glm::quat eigen_direction);
glm::vec3 GetTangentAxis(EigenValue projection, glm::quat eigen_direction, glm::vec3 root, glm::vec3 centroid);
	
struct SkinnedMeshMemo
{
	
	SkinnedMeshMemo(SkinnedMesh const* ptr);
	SkinnedMeshMemo(SkinnedMeshMemo const&) = delete;
	~SkinnedMeshMemo();
	
	// basically all children of a clique are symmetrical about some random point.
	struct Clique
	{
		uint32_t parent;
		immutable_array<int32_t> children;
	};
	
	const SkinnedMesh & in;
	
	Silhouette & GetSilhouettes(glm::mat4 const& projection, glm::vec3 scale, std::span<uint16_t> joints = {},  float cutoff = 0.5f, bool secondMoment = true);
	Silhouette & GetSilhouettes(EigenValue projection, glm::vec3 scale, std::span<uint16_t> joints, float cutoff=0.5, bool secondMoment=false);
	Silhouette & GetSilhouettes(Axis axis, glm::vec3 scale, std::span<uint16_t> joints = {},  float cutoff = 0.5f, bool secondMoment = true);
	
	glm::mat4 GetProjectionMatrix(EigenValue projection, glm::vec3 scale, std::span<uint16_t> joints, SkinnedMesh::LimbMetrics * metrics = nullptr, std::pair<glm::quat, glm::vec3> * eigen_decomp = nullptr) const;

	
	immutable_array<Clique> GetCliques();
	
	// how tube-y is each thing?
	// tubiness is based on skinning data.
	// verticies affected by (this AND PARENT + ONLY this + this AND children) / (this)
	// so a tube has a tubiness of 1, while the left side of a whale fluke has low tubiness.
	immutable_array<float>  GetTubeTable();

private:
	immutable_array<float>   _tubeTable;
	immutable_array<Clique> cliques;

	union Key
	{
		struct
		{
			uint32_t joints;
			uint8_t cutoff;
			uint8_t eigenValue;
			uint8_t second_moment;
			uint8_t pad00;
		};
		
		uint64_t key{};
	};
	
	struct ScaledKey
	{
		Key key;
		glm::vec3 scale;
		
		bool operator<(const ScaledKey & it) const
		{
			if(key.key != it.key.key) return key.key < it.key.key;
			if(scale.x < it.scale.x) return scale.x < it.scale.x;
			if(scale.y < it.scale.y) return scale.y < it.scale.y;
			return scale.z < it.scale.z;
		}
	};
	
	std::mutex _mutex;
	std::map<ScaledKey, Silhouette*> _cache;
};


}

#endif
