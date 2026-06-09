#include "../include/tonton_rigsolver.h"
#include "Memos/tonton_skinnedmeshmemo.h"
#include "tonton_eigen.h"
#include "tonton_tensors.hpp"
#include <glm/gtx/norm.hpp>
#include <algorithm>
#include <cmath>
#include <vector>

using namespace TonTon;

// ── Capsule-capsule collision ────────────────────────────────────────────────

// Closest distance squared between two line segments (a0,a1) and (b0,b1).
static float SegmentDistanceSq(glm::vec3 a0, glm::vec3 a1, glm::vec3 b0, glm::vec3 b1)
{
	glm::vec3 d1 = a1 - a0;
	glm::vec3 d2 = b1 - b0;
	glm::vec3 r  = a0 - b0;

	float a = glm::dot(d1, d1); // |d1|^2
	float e = glm::dot(d2, d2); // |d2|^2
	float f = glm::dot(d2, r);

	float s, t;

	if (a <= 1e-12f && e <= 1e-12f)
	{
		// Both degenerate to points
		return glm::dot(r, r);
	}

	if (a <= 1e-12f)
	{
		s = 0.f;
		t = std::clamp(f / e, 0.f, 1.f);
	}
	else
	{
		float c = glm::dot(d1, r);
		if (e <= 1e-12f)
		{
			t = 0.f;
			s = std::clamp(-c / a, 0.f, 1.f);
		}
		else
		{
			float b_ = glm::dot(d1, d2);
			float denom = a * e - b_ * b_;

			if (denom != 0.f)
				s = std::clamp((b_ * f - c * e) / denom, 0.f, 1.f);
			else
				s = 0.f;

			t = (b_ * s + f) / e;

			if (t < 0.f)
			{
				t = 0.f;
				s = std::clamp(-c / a, 0.f, 1.f);
			}
			else if (t > 1.f)
			{
				t = 1.f;
				s = std::clamp((b_ - c) / a, 0.f, 1.f);
			}
		}
	}

	glm::vec3 closest = r + d1 * s - d2 * t;
	return glm::dot(closest, closest);
}

static bool CapsuleCollision(Capsule const& ca, glm::vec3 ca_a, glm::vec3 ca_b,
                             Capsule const& cb, glm::vec3 cb_a, glm::vec3 cb_b)
{
	float sum_r = ca.radius + cb.radius;
	return SegmentDistanceSq(ca_a, ca_b, cb_a, cb_b) < sum_r * sum_r;
}

// ── CCD solver command ───────────────────────────────────────────────────────

struct CCD_Command
{
	const RigSolver& solver;
	std::vector<uint16_t> chain; // joint indices root→tip
	glm::vec3 target;
	int max_iterations;

	// Workspace
	std::vector<glm::quat> deltas;    // per chain joint, rotation from rest
	std::vector<glm::vec3> positions; // current posed positions

	void BuildChain(uint16_t root, uint16_t tip);
	void ForwardKinematics();
	void ClampToLimits(int chain_idx);
	bool CheckCapsuleCollision(int chain_idx);
	SolveResult Execute();
};

void CCD_Command::BuildChain(uint16_t root, uint16_t tip)
{
	auto const& parents = solver._skinnedMesh->skin->parents;

	// Walk from tip to root via parents
	chain.clear();
	for (int j = tip; ; j = parents[j])
	{
		chain.push_back(uint16_t(j));
		if (j == root || j < 0) break;
	}

	// Reverse so chain[0] = root, chain.back() = tip
	std::reverse(chain.begin(), chain.end());

	deltas.assign(chain.size(), glm::quat(1, 0, 0, 0));
}

void CCD_Command::ForwardKinematics()
{
	auto const& rest_pos = solver._skinnedMesh->skin->position;
	auto const& rest_rot = solver._skinnedMesh->skin->rotation;
	auto const& parents = solver._skinnedMesh->skin->parents;

	positions.resize(chain.size());

	// For each joint in the chain, compute its posed position by applying
	// all ancestor deltas in the chain above it.
	// Position of joint i = apply rotations from root down to i-1 around their pivots.
	for (size_t i = 0; i < chain.size(); ++i)
	{
		glm::vec3 pos = rest_pos[chain[i]];

		// Apply each ancestor rotation in the chain (from root toward this joint)
		for (size_t k = 0; k < i; ++k)
		{
			glm::vec3 pivot = (k == 0) ? rest_pos[chain[0]] : positions[k];
			pos = pivot + deltas[k] * (pos - pivot);
		}

		positions[i] = pos;
	}
}

void CCD_Command::ClampToLimits(int chain_idx)
{
	if (solver._articulations.empty()) return;

	uint16_t joint = chain[chain_idx];
	auto stages = solver._articulations[joint];
	if (stages.empty()) return;

	// Decompose the current delta into Euler angles and clamp per-axis
	glm::quat& q = deltas[chain_idx];
	glm::vec3 euler = glm::eulerAngles(q); // pitch, yaw, roll (XYZ)

	for (auto& stage : stages)
	{
		float* angle = nullptr;
		switch (stage.type)
		{
		case ArticulationDOF::xRotate: angle = &euler.x; break;
		case ArticulationDOF::yRotate: angle = &euler.y; break;
		case ArticulationDOF::zRotate: angle = &euler.z; break;
		default: continue;
		}

		float range_min = stage.min_value - stage.initial_value;
		float range_max = stage.max_value - stage.initial_value;
		*angle = std::clamp(*angle, range_min, range_max);
	}

	q = glm::quat(euler);
}

bool CCD_Command::CheckCapsuleCollision(int chain_idx)
{
	auto const& capsules = solver.capsules;
	if (capsules.empty()) return false;

	uint16_t joint = chain[chain_idx];
	auto const& cap = capsules[joint];
	if (!cap.valid) return false;

	// Compute posed capsule endpoints for this joint
	glm::vec3 a = cap.a, b = cap.b;
	for (int k = 0; k <= chain_idx; ++k)
	{
		glm::vec3 pivot = positions[k];
		a = pivot + deltas[k] * (a - pivot);
		b = pivot + deltas[k] * (b - pivot);
	}

	// Test against adjacent chain capsules (parent and child only)
	auto testAgainst = [&](int other_idx) -> bool
	{
		uint16_t other_joint = chain[other_idx];
		auto const& other_cap = capsules[other_joint];
		if (!other_cap.valid) return false;

		glm::vec3 oa = other_cap.a, ob = other_cap.b;
		for (int k = 0; k <= other_idx; ++k)
		{
			glm::vec3 pivot = positions[k];
			oa = pivot + deltas[k] * (oa - pivot);
			ob = pivot + deltas[k] * (ob - pivot);
		}

		return CapsuleCollision(cap, a, b, other_cap, oa, ob);
	};

	// Check parent capsule (chain_idx - 1) and grandparent (chain_idx - 2)
	// Skip immediate parent since they share a joint and will always "collide"
	if (chain_idx >= 2 && testAgainst(chain_idx - 2)) return true;

	// Check child capsule
	if (chain_idx + 2 < (int)chain.size() && testAgainst(chain_idx + 2)) return true;

	return false;
}

SolveResult CCD_Command::Execute()
{
	ForwardKinematics();

	float convergence_threshold = 1e-5f;

	for (int iter = 0; iter < max_iterations; ++iter)
	{
		// Walk from tip-1 to root (standard CCD order)
		for (int ci = (int)chain.size() - 2; ci >= 0; --ci)
		{
			glm::vec3 pivot = positions[ci];
			glm::vec3 tip_pos = positions.back();

			glm::vec3 to_tip = tip_pos - pivot;
			glm::vec3 to_target = target - pivot;

			float len_tip = glm::length(to_tip);
			float len_target = glm::length(to_target);

			if (len_tip < 1e-8f || len_target < 1e-8f) continue;

			to_tip /= len_tip;
			to_target /= len_target;

			float dot = std::clamp(glm::dot(to_tip, to_target), -1.f, 1.f);
			if (dot > 0.9999f) continue;

			glm::vec3 axis = glm::cross(to_tip, to_target);
			float axis_len = glm::length(axis);
			if (axis_len < 1e-8f) continue;

			axis /= axis_len;
			float angle = std::acos(dot);

			glm::quat rotation = glm::angleAxis(angle, axis);
			deltas[ci] = rotation * deltas[ci];

			// Clamp to joint limits
			ClampToLimits(ci);

			// Recompute FK after this joint's update
			ForwardKinematics();

			// Check capsule collision, back off if needed
			if (CheckCapsuleCollision(ci))
			{
				// Binary search for largest non-colliding fraction
				glm::quat saved = deltas[ci];
				glm::quat original = glm::quat(1, 0, 0, 0); // identity as baseline

				// We already know saved collides; find how much we can keep
				// by interpolating between identity and saved
				float lo = 0.f, hi = 1.f;
				glm::quat best = glm::quat(1, 0, 0, 0);
				for (int bs = 0; bs < 8; ++bs)
				{
					float mid = (lo + hi) * 0.5f;
					deltas[ci] = glm::slerp(original, saved, mid);
					ForwardKinematics();
					if (CheckCapsuleCollision(ci))
						hi = mid;
					else
					{
						best = deltas[ci];
						lo = mid;
					}
				}
				deltas[ci] = best;
				ForwardKinematics();
			}
		}

		float dist = glm::distance(positions.back(), target);
		if (dist < convergence_threshold) break;
	}

	float final_dist = glm::distance(positions.back(), target);

	shared_array<glm::quat> result_deltas(chain.size());
	for (size_t i = 0; i < chain.size(); ++i)
		result_deltas[i] = deltas[i];

	return SolveResult{
		.joint_deltas = std::move(result_deltas),
		.tip_distance = final_dist
	};
}

// ── RigSolver ────────────────────────────────────────────────────────────────

RigSolver::RigSolver() = default;
RigSolver::~RigSolver() = default;

counted_ptr<const RigSolver> RigSolver::Factory(
	counted_ptr<const SkinnedMesh> skinnedMesh,
	Articulations articulations)
{
	auto solver = UncountedWrap(new RigSolver());

	solver->_skinnedMesh = skinnedMesh;
	solver->_articulations = std::move(articulations);

	// Build capsules from covariance tensors
	auto const& skin = skinnedMesh->skin;
	auto bone_tails = skinnedMesh->memo()->GetBoneTails();
	uint32_t n_joints = skin->position.size();

	shared_array<Capsule> caps(n_joints);
	glm::mat4 identity(1.f);

	for (uint32_t i = 0; i < n_joints; ++i)
	{
		caps[i].joint = uint16_t(i);
		caps[i].valid = false;

		if (skinnedMesh->volume[i] <= 0.f) continue;

		// Reconstruct covariance matrix
		auto const& cov = skinnedMesh->covariance[i];
		glm::dmat3 C(
			cov[0], cov[3], cov[4],
			cov[3], cov[1], cov[5],
			cov[4], cov[5], cov[2]
		);

		// Normalize by volume to get per-unit covariance
		C = C / double(skinnedMesh->volume[i]);

		// Check positive definiteness
		auto classification = ClassifySymmetricMatrix(C);
		if (!(classification & POSITIVE) || !(classification & DEFINITE))
			continue;

		// Capsule axis = bone direction
		glm::vec3 a = skin->position[i];
		glm::vec3 b = bone_tails[i];
		glm::vec3 bone_dir = b - a;
		float bone_len = glm::length(bone_dir);

		if (bone_len < 1e-8f) continue;

		glm::dvec3 d = glm::normalize(glm::dvec3(bone_dir));

		// Project covariance onto plane perpendicular to bone axis
		glm::dmat3 proj = glm::dmat3(1.0) - glm::outerProduct(d, d);
		glm::dmat3 projected_cov = proj * C * proj;

		auto [rotation, eigenvalues] = EigenDecomposition(SecondMomentTensor{projected_cov});

		// Radius = 2 * sqrt(max perpendicular eigenvalue)
		// eigenvalues sorted smallest→largest; largest is z
		double max_perp_eigenvalue = std::max(double(eigenvalues.y), double(eigenvalues.z));
		if (max_perp_eigenvalue <= 0.0) continue;

		caps[i].a = a;
		caps[i].b = b;
		caps[i].radius = float(2.0 * std::sqrt(max_perp_eigenvalue));
		caps[i].valid = true;
	}

	solver->capsules = std::move(caps);

	return solver;
}

SolveResult RigSolver::SolveCCD(uint16_t root, uint16_t tip,
                                glm::vec3 target,
                                int max_iterations) const
{
	CCD_Command cmd{
		.solver = *this,
		.target = target,
		.max_iterations = max_iterations
	};

	cmd.BuildChain(root, tip);
	return cmd.Execute();
}

float RigSolver::SolveCrouchLength(uint16_t root, uint16_t tip) const
{
	// Target = root position (fully compress the chain)
	glm::vec3 root_pos = _skinnedMesh->skin->position[root];
	auto result = SolveCCD(root, tip, root_pos, 20);

	// The tip's final position after maximum compression
	// Recompute the tip position from the deltas
	auto const& rest_pos = _skinnedMesh->skin->position;
	auto const& parents = _skinnedMesh->skin->parents;

	// Rebuild the chain to get joint indices
	std::vector<uint16_t> chain;
	for (int j = tip; ; j = parents[j])
	{
		chain.push_back(uint16_t(j));
		if (j == root || j < 0) break;
	}
	std::reverse(chain.begin(), chain.end());

	// Compute final tip position via FK with the solved deltas
	glm::vec3 tip_pos = rest_pos[tip];
	for (size_t k = 0; k < chain.size() && k < result.joint_deltas.size(); ++k)
	{
		glm::vec3 pivot;
		if (k == 0)
			pivot = rest_pos[chain[0]];
		else
		{
			// Recompute pivot position
			pivot = rest_pos[chain[k]];
			for (size_t m = 0; m < k; ++m)
			{
				glm::vec3 p = (m == 0) ? rest_pos[chain[0]] : rest_pos[chain[m]];
				// Apply prior rotations to this pivot
				for (size_t n = 0; n < m; ++n)
				{
					glm::vec3 pp = rest_pos[chain[n]];
					p = pp + result.joint_deltas[n] * (p - pp);
				}
				pivot = p + result.joint_deltas[m] * (pivot - p);
			}
		}
		tip_pos = pivot + result.joint_deltas[k] * (tip_pos - pivot);
	}

	return glm::distance(root_pos, tip_pos);
}
