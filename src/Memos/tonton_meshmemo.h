#ifndef TONTON_MESH_MEMO_H
#define TONTON_MESH_MEMO_H
#include "tonton_shared_array.hpp"
#include <cstdint>
#include <glm/mat4x4.hpp>
#include <mutex>
#include <map>
#include <span>

namespace DoDeeDum { struct Silhouette; }

namespace TonTon
{

struct Mesh;
using Silhouette = DoDeeDum::Silhouette;

struct MeshMemo
{
	MeshMemo(Mesh const* ptr);
	MeshMemo(MeshMemo const&) = delete;
	~MeshMemo();
	
	const Mesh & in;
	
	uint32_t get_index(std::span<uint16_t>);
	Silhouette GetSilhouettes(glm::mat4 const& projection, glm::dvec3 const& scale, std::span<const glm::mat4> pose, std::span<const uint16_t> joints,  float cutoff, bool secondMoment);

	int GetVertexOverlap(int a, int b);
	float GetVertexOverlapPercent(int a, int b);
	float GetTubiness(int parent, int node, std::span<const uint16_t> children);

private:
	std::map<immutable_array<uint16_t>, uint32_t, std::less<>> _span_indices;
	void BuildVertexOverlapMemos();
	
	immutable_array<int> _vertexOverlapTable;
	immutable_array<int> _totalVertsAffected;
	int _maxJoint{};
	
	std::mutex _mutex;
};

}

#endif
