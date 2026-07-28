#ifndef TONTON_MYOPIC_H
#define TONTON_MYOPIC_H

#include <cstdint>
#include <glm/vec3.hpp>
#include <glm/gtc/quaternion.hpp>

namespace TonTon {

struct Output;

enum class LocomotionMode : uint8_t {
	TERRESTRIAL, AERIAL, AQUATIC, SERPENTINE, CLIMBING, BRACHIATION,
	// JUMPING is a one-shot event, not a sustained mode.
};

enum class Substrate : uint8_t { GROUND, WATER, PERCH, CLIFF_EDGE };

enum class BlockingReason : uint8_t {
	NONE, NEEDS_RUNWAY_SPEED, NEEDS_ELEVATION, NEEDS_PERCH,
	WING_LOADING, POWER_LOADING, ASPECT_RATIO, LEG_STRENGTH,
	NO_AERIAL_ANALYSIS,
};

// All members are plain floats with units in their names so the engine can
// fill this in without depending on TonTon's Quantity system.
struct MyopicInput {
	LocomotionMode mode{LocomotionMode::TERRESTRIAL};
	LocomotionMode target_mode{LocomotionMode::TERRESTRIAL};

	glm::vec3 position{0};
	glm::quat orientation{1, 0, 0, 0};
	glm::vec3 velocity_m_s{0};          // world space, relative to the ground
	glm::vec3 angular_velocity_rad_s{0}; // body space

	glm::vec3 target_position{0};
	float     desired_speed_m_s{-1.f};  // < 0 -> controller picks cruise

	glm::vec3 medium_velocity_m_s{0};   // wind or current, world space
	float     height_above_ground_m{0};
	Substrate substrate{Substrate::GROUND};

	// A world fact the caller owns, like the two above. Analysis_Physical has
	// no gravity field and Input::environment never reaches Output, so this is
	// the only honest source. Do NOT hardcode 9.81 anywhere in the module --
	// the library explicitly supports low-gravity settings (see the Titan
	// dragonfly example in README.md).
	float gravity_m_s2{9.81f};

	int   current_gait{0};              // terrestrial: 0 walk, 1 trot, 2 gallop
	float dt_s{1.f / 60.f};
};

struct MyopicOutput {
	glm::vec3 linear_acceleration_m_s2{0};   // world space
	glm::vec3 angular_acceleration_rad_s2{0}; // body space

	float stability{1.f};        // 1 comfortable, 0 at limits, < 0 exceeding
	float speed_headroom{1.f};
	float turn_headroom{1.f};
	bool  suggest_gait_change{false};

	float          transition_readiness{0.f}; // [0,1]; meaningful when target != mode
	BlockingReason blocking_reason{BlockingReason::NONE};
	float          touchdown_speed_m_s{0.f};  // aerial only

	// Descriptive only. The caller owns all jump timing: a jump has a crouch
	// pre-animation, so no single frame is the right one to apply an impulse.
	float     required_jump_velocity_m_s{0.f};
	glm::vec3 jump_direction{0};
	bool      jump_feasible{false};
};

// Caller-owned, one per creature. Zero-initialised is a valid cold start.
struct MyopicState {
	glm::vec3 prev_linear_accel_m_s2{0};
	glm::vec3 prev_angular_accel_rad_s2{0};
	float     bank_angle_rad{0};
};

MyopicOutput ComputeMyopicControl(
	const Output& analysis, const MyopicInput& input, MyopicState& state);

} // namespace TonTon

#endif // TONTON_MYOPIC_H
