#ifndef TONTON_ARTICULATIONS_H
#define TONTON_ARTICULATIONS_H
#include "tonton_shared_array.hpp"
#include <cstdint>
#include <span>
#include <utility>

namespace TonTon
{

enum class ArticulationDOF : uint8_t {
	xTranslate, yTranslate, zTranslate,
	xRotate, yRotate, zRotate,
	xScale, yScale, zScale,
	uniformScale
};

struct ArticulationStage {
	ArticulationDOF type{};
	float min_value{};       // radians (rotation) or meters (translation)
	float max_value{};
	float initial_value{};   // rest pose value (usually 0)
	float max_velocity{};    // rad/s or m/s (0 = unlimited)
};

struct Articulations {
	immutable_array<std::pair<uint16_t, uint16_t>> joints;  // per joint index: {begin, end} into stages
	immutable_array<ArticulationStage> stages;              // flat array

	bool empty() const { return joints.empty(); }

	std::span<const ArticulationStage> operator[](uint16_t joint) const {
		if (joint >= joints.size()) return {};
		auto [begin, end] = joints[joint];
		return {stages.data() + begin, stages.data() + end};
	}
};

};

#endif // TONTON_ARTICULATIONS_H
