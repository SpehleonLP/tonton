#ifndef TONTON_SKINNEDMESH_H
#define TONTON_SKINNEDMESH_H
#include "tonton_shared_array.hpp"
#include "tonton_counted_ptr.hpp"
#include "tonton_wordlist.h"
#include <glm/gtc/quaternion.hpp>
#include <glm/vec3.hpp>
#include <memory>
#include <array>
#include <span>
#include <vector>

namespace DoDeeDum
{
struct Primitive;
using Mesh = std::vector<Primitive>;
}

namespace TonTon
{

struct Armature;
struct SkinnedMesh;
struct ArmatureMemo;
struct MeshMemo;
struct SkinnedMeshMemo;
struct Environment;
struct Input;

struct Cube
{
	glm::vec3 min{0}, max{0};
	Cube operator|(Cube const& it) const { return {glm::min(min, it.min), glm::max(max, it.max) }; }
};

struct Armature
{
	static counted_ptr<const Armature> Factory(
		immutable_array<std::string> names, 
		immutable_array<int> parents,
		immutable_array<glm::vec3>	position,
		immutable_array<glm::quat> rotation,
		immutable_array<immutable_array<Word>> tags = {}); 
	
	inline void AddRef() const { ++_refCount; };
	inline void Release() const { if(--_refCount == 0) delete this; } 
	
	ArmatureMemo * memo() const;
	inline ArmatureMemo * operator->() const { return memo(); }
	inline ArmatureMemo & operator*() const { return *memo(); }
	
	immutable_array<std::string> names;	
	immutable_array<immutable_array<Word>> tags;	
	immutable_array<int> parents;	
	immutable_array<glm::vec3>	position;
	immutable_array<glm::quat>	rotation;
	
private:
	Armature();
	~Armature();
	
	mutable std::atomic<int> _refCount{1};
	mutable std::unique_ptr<ArmatureMemo> _memo;
};

struct Mesh
{
	static counted_ptr<const Mesh> Factory(DoDeeDum::Mesh &&); 

	inline void AddRef() const { ++_refCount; };
	inline void Release() const { if(--_refCount == 0) delete this; }
	
	MeshMemo * memo() const;
	inline MeshMemo * operator->() const { return memo(); }
	inline MeshMemo & operator*() const { return *memo(); }
	
	DoDeeDum::Mesh mesh;
		
private:
	Mesh();
	~Mesh();
	
	mutable std::atomic<int> _refCount{1};
	mutable std::unique_ptr<MeshMemo> _memo;
};

struct SkinnedMesh
{
	static counted_ptr<const SkinnedMesh> Factory(
		counted_ptr<const Mesh>	mesh, 
		counted_ptr<const Armature> armature,
		
		immutable_array<Cube>		aabb,
		immutable_array<float>		surfaceArea,
			
		immutable_array<float>		volume,
		immutable_array<glm::vec3>	centroid,
		immutable_array<std::array<float, 6>>	covariance
	);

	void AddRef() const { ++_refCount; };
	void Release() const { if(--_refCount == 0) delete this; }
	
	SkinnedMeshMemo * memo() const;
	inline SkinnedMeshMemo * operator->() const { return memo(); }
	inline SkinnedMeshMemo & operator*() const { return *memo(); }
	
	counted_ptr<const Mesh>		mesh;
	counted_ptr<const Armature>  skin;
	
	immutable_array<Cube>		aabb; 
	immutable_array<float>		surfaceArea;
		
	immutable_array<float>		volume;
	immutable_array<glm::vec3>	centroid;
	immutable_array<std::array<float, 6>>	covariance;
	
	glm::dmat3 GetInertia(uint32_t i, glm::dvec3 scale) const;	
	double EstimateCrossSection(uint32_t i, glm::dvec3 scale, glm::vec3 direction, double * second_moment_area = nullptr) const;
	double EstimateCrossSection(std::span<uint16_t> joints, glm::dvec3 scale, glm::vec3 direction, double * second_moment_area = nullptr) const;
	
	glm::dvec3 GetCentroid(std::span<uint16_t> joints, glm::dvec3 const& scale, double * volume_out = nullptr) const;
	glm::dmat3 GetInertia(std::span<uint16_t> joints, glm::dvec3 const& scale, glm::dvec3 *centroid_out = nullptr, double * volume_out = nullptr) const;
	
	std::array<double, 6> GetCovariance(uint32_t i, glm::dvec3 scale) const;		
	std::array<double, 6> GetCovariance(std::span<uint16_t> joints, glm::dvec3 const& scale, glm::dvec3 *centroid_out = nullptr, double * volume_out = nullptr) const;
	
	double GetSurfaceArea(uint16_t i, double area_scale) const;
	double GetSurfaceArea(std::span<uint16_t> joints, double area_scale) const;
		
	struct LimbMetrics
	{
		double volume{};
		glm::dvec3 centroid{0};
		glm::dmat3 unitInertia{1};
		
		glm::dmat3 GetInertia(glm::vec3 measured_at, float density) const;
		double GetInertia(glm::vec3 measured_at, float density, glm::vec3 axis) const;
	};	
		
	LimbMetrics GetMetrics(std::span<uint16_t> joints, glm::dvec3 const& scale) const;

	struct StalkData
	{
		int root{};
		int tip{};
		
		float thickestCrossSection_m2{};
		float thinestCrossSection_m2{};
		float length_m{};
	};
	
	// find the subset of the chain between tip and root thats most stalk-y and get the data on it.
	bool GetStalkData(StalkData & dst, int root, int tip, glm::vec3 scale) const;
	
private:
	SkinnedMesh();
	~SkinnedMesh();
	
	mutable std::atomic<int> _refCount{1};
	mutable std::unique_ptr<SkinnedMeshMemo> _memo;
};

}

#endif
