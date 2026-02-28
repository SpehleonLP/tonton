#ifndef TONTON_RIGSOLVER_H
#define TONTON_RIGSOLVER_H
#include "tonton_articulations.h"
#include "tonton_counted_ptr.hpp"
#include "tonton_shared_array.hpp"
#include "tonton_skinnedmesh.h"
#include <glm/gtc/quaternion.hpp>
#include <glm/vec3.hpp>
#include <atomic>

struct CCD_Command;

namespace TonTon
{

struct Capsule
{
	glm::vec3 a{0}, b{0}; // endpoints in bind pose
	float radius{0};       // from cross-section perpendicular to bone axis
	uint16_t joint{0};     // which joint this belongs to
	bool valid{false};     // false if joint had zero volume or non-PD covariance
};

struct SolveResult
{
	immutable_array<glm::quat> joint_deltas; // per chain joint, rotation from rest pose
	float tip_distance{0};                    // remaining distance to target (error)
};

struct RigSolver
{
	static counted_ptr<const RigSolver> Factory(
		counted_ptr<const SkinnedMesh> skinnedMesh,
		Articulations articulations = {}
	);

	immutable_array<Capsule> capsules; // indexed by joint

	SolveResult SolveCCD(uint16_t root, uint16_t tip,
	                     glm::vec3 target,
	                     int max_iterations = 10) const;

	float SolveCrouchLength(uint16_t root, uint16_t tip) const;

	void AddRef() const { ++_refCount; }
	void Release() const { if(--_refCount == 0) delete this; }

private:
	friend struct ::CCD_Command;

	RigSolver();
	~RigSolver();

	counted_ptr<const SkinnedMesh> _skinnedMesh;
	Articulations _articulations;
	mutable std::atomic<int> _refCount{1};
};

}

#endif // TONTON_RIGSOLVER_H
