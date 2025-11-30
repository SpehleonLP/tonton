#ifndef TONTON_SKINNED_MESH_MEMO_H
#define TONTON_SKINNED_MESH_MEMO_H
#include "../include/tonton_skinnedmesh.h"
#include <cstdint>
#include <glm/gtc/quaternion.hpp>
#include <glm/vec3.hpp>
#include <glm/mat3x3.hpp>
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
	
	Silhouette GetSilhouettes(glm::mat4 const& projection, glm::vec3 scale, std::span<const uint16_t> joints = {},  float cutoff = 0.5f, bool secondMoment = true);
	Silhouette GetSilhouettes(Axis axis, glm::vec3 scale, std::span<const uint16_t> joints = {},  float cutoff = 0.5f, bool secondMoment = true);
	
	glm::mat4 GetProjectionMatrix(EigenValue projection, std::span<const glm::vec3> positions, std::span<const glm::vec3> scale, std::span<const uint16_t> joints, SkinnedMesh::LimbMetrics * metrics = nullptr, std::pair<glm::quat, glm::vec3> * eigen_decomp = nullptr) const;

	
	immutable_array<Clique> GetCliques();
	
	// how tube-y is each thing?
	// tubiness is based on skinning data.
	// verticies affected by (this AND PARENT + ONLY this + this AND children) / (this)
	// so a tube has a tubiness of 1, while the left side of a whale fluke has low tubiness.
	// tubiness is based on topology not geometry. 
	immutable_array<float>  GetTubeTable();
	immutable_array<glm::vec3> GetBoneTails();

private:
	immutable_array<float>   _tubeTable;
	immutable_array<Clique> cliques;
	immutable_array<glm::vec3> _boneTails;
	
	std::mutex _mutex;
};


}

#endif
