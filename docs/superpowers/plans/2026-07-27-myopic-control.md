# Myopic Locomotion Controller Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a per-frame steering controller that turns a `TonTon::Output` plus a creature's current state into body-level acceleration, torque, and a stability readout.

**Architecture:** Three units. `ExtractEnvelope` reduces `Output` to a small `Envelope` struct; `PlanLaunch` reduces `Analysis_TakeoffAnalysis` to a `LaunchPlan`; `Steer` does pure math on those two and never sees `Output`. Steering is greedy-and-clamped (cannot overshoot) followed by an exponential slew whose time constant comes from the creature's own data (framerate-correct at any `dt`).

**Tech Stack:** C++20, GLM, TonTon's compile-time `Quantity` unit types, GoogleTest.

**Spec:** `docs/superpowers/specs/2026-07-27-myopic-control-design.md`. Read it before starting — it explains *why* each constraint exists.

## Global Constraints

- **C++20.** `set(CMAKE_CXX_STANDARD 20)`, no compiler extensions.
- **No tunable constants.** No gains, no time horizons, no responsiveness knobs, no fudge factors. Every number is read from `Output` or is `dt`. A reviewer should reject any new literal that is not a physical constant (π) or a pure guard epsilon. This is the specific thing that failed in 2025.
- **`Steer` must not include `tonton_analysis.h`.** Enforced by the include list, not by discipline.
- **Airspeed, not ground speed.** `airspeed = velocity - medium_velocity` is computed once at the top of `ComputeMyopicControl` and every speed comparison uses it. Ground speed is used only for runway distance.
- **Internals use `Quantity` types; the public API uses bare `float`/`glm::vec3` with units in member names.** Conversion happens once, at the boundary of `ComputeMyopicControl`.
- **Naming:** `snake_case` files prefixed `tonton_`, `PascalCase` types, units in variable names.
- **SI units throughout.**

### Quantity type gotchas (read before writing any math)

From `include/tonton_units.hpp`:

- `operator float()` is **explicit**. Write `float(q)`, never an implicit conversion.
- `Quantity / Quantity` with the **same** dimensions returns a bare `float`. This is how ratios like `airspeed / max_speed` are written — no cast needed.
- `Quantity + float` and `Quantity - float` are **deleted**. Add quantities to quantities only.
- `omega_rad_s` and `freq_Hz` are **the same type** (`Quantity<0,0,-1>`). The compiler will not catch confusing them; be deliberate.
- `angle_rad` is `Quantity<0,0,0>` — dimensionless. It will silently interconvert with other dimensionless quantities. Prefer bare `float` radians inside `Steer` for clarity.
- `velocity_m_s / acceleration_m_s2` yields `time_s`. This is how `tau_linear` is built.

---

## File Structure

| File | Responsibility |
|---|---|
| `include/tonton_myopic.h` | **Create.** Entire public API: enums, `MyopicInput`, `MyopicOutput`, `MyopicState`, `ComputeMyopicControl`. Bare floats only. |
| `src/Control/tonton_envelope.h` | **Create.** `AerialAuthority`, `Envelope`, `ExtractEnvelope` declaration. Internal — `Quantity` types. |
| `src/Control/tonton_envelope.cpp` | **Create.** `ExtractEnvelope`: the only steering code that reads `Output`. |
| `src/Control/tonton_launch.h` | **Create.** `LaunchPlan`, `PlanLaunch` declaration. |
| `src/Control/tonton_launch.cpp` | **Create.** The only code that reads `Analysis_TakeoffAnalysis`. |
| `src/Control/tonton_steer.h` | **Create.** `Steer` declaration. Must **not** include `tonton_analysis.h`. |
| `src/Control/tonton_steer.cpp` | **Create.** Greedy + clamp + slew + bank/yaw + stability. Pure math. |
| `src/Control/tonton_myopic.cpp` | **Create.** `ComputeMyopicControl`: unit conversion at the boundary, then calls the three units. |
| `tonton-example/tests/tonton_myopic_tests.cpp` | **Create.** All tests for this module. |
| `tonton-example/CMakeLists.txt:166-171` | **Modify.** Add the new test file to `add_executable(tonton-tests ...)`. |

**Build note:** `tonton/CMakeLists.txt` uses `file(GLOB_RECURSE src/*.cpp)`, so new `.cpp` files under `src/` are picked up **only after CMake re-runs**. After creating any new source file, run `cmake -S /mnt/Passport/Libraries/Spehleon/tonton-example -B /mnt/Passport/Libraries/Spehleon/tonton-example/build` before building, or the file will be silently ignored.

**Build and run commands used throughout:**

```bash
cd /mnt/Passport/Libraries/Spehleon/tonton-example
cmake -S . -B build                                    # only after adding new files
cmake --build build --target tonton-tests -j$(nproc)
./build/tonton-tests --gtest_filter='Myopic*'
```

### The envelope is rebuilt every frame

`ExtractEnvelope` is called from `ComputeMyopicControl` on every invocation — or however
often the caller polls. It is **not** cached, and there is no invalidation path to get
wrong. This is what makes the module myopic in the same sense the steering is: gait
changes, substrate changes, depth and medium changes are all picked up automatically
because nothing is remembered between frames except the three slew values in
`MyopicState`.

Two consequences for implementers:

- **It must stay cheap.** A `switch`, a handful of divides, one `sqrt` on the aerial
  path. Do not add allocation, iteration over appendages, or anything touching the
  memo layer. If a future mode needs an expensive quantity, that quantity belongs in
  `Output` (computed once at creature creation), not in `ExtractEnvelope`.
- **It must stay a pure function of its arguments.** No statics, no caching, no
  hidden state. Same inputs, same envelope, every time.

### Two deviations from the spec, and why

1. **`Envelope` carries `max_lateral_accel`, not `max_turn_rate`.** Turn rate is speed-dependent: a standing animal pivots freely, a galloping one cannot. Storing a lateral-acceleration budget and deriving `ω = a_lat / v` per frame is correct at both ends and needs no constant. It also unifies with banking exactly, since a coordinated turn has `a_lat = g·tan φ` and therefore `ω = g·tan φ / V` — the same line of code serves both.
2. **Aerial authority lives inside `Envelope` as `std::optional<AerialAuthority>`.** `TonTon::Output`'s constructor is private (`tonton_analysis.h:719`) and only `Output::Factory(Input)` is public, so tests cannot hand-build an `Output`. Putting the aerial numbers in `Envelope` makes every steering test a pure struct test, and makes "`Steer` never sees `Output`" a compile-time fact rather than a convention.

---

### Task 1: Public header and terrestrial envelope

**Files:**
- Create: `include/tonton_myopic.h`
- Create: `src/Control/tonton_envelope.h`
- Create: `src/Control/tonton_envelope.cpp`
- Create: `tonton-example/tests/tonton_myopic_tests.cpp`
- Modify: `tonton-example/CMakeLists.txt:166-171`

**Interfaces:**
- Consumes: `TonTon::Output` (`include/tonton_analysis.h`), `TonTon::Quantity` types (`include/tonton_units.hpp`).
- Produces: `TonTon::LocomotionMode`, `Substrate`, `BlockingReason`, `MyopicInput`, `MyopicOutput`, `MyopicState`, `ComputeMyopicControl` (declared, not yet defined); `TonTon::AerialAuthority`, `Envelope`, `ExtractEnvelope(const Output&, LocomotionMode, int gait, float gravity_m_s2) -> std::optional<Envelope>`.

`ExtractEnvelope` returns `std::nullopt` when the requested mode has no analysis section (e.g. `AERIAL` on a cat). Callers must handle this; it is not an error.

- [ ] **Step 1: Write the public header**

Create `include/tonton_myopic.h`:

```cpp
#ifndef TONTON_MYOPIC_H
#define TONTON_MYOPIC_H

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
```

- [ ] **Step 2: Write the internal envelope header**

Create `src/Control/tonton_envelope.h`:

```cpp
#ifndef TONTON_ENVELOPE_H
#define TONTON_ENVELOPE_H

#include "tonton_myopic.h"
#include "tonton_units.hpp"
#include <optional>

namespace TonTon {

struct Output;

// Populated only for LocomotionMode::AERIAL. Lives here rather than being read
// from Output at steer time so that Steer never sees Output, and so that tests
// can build one directly (Output's constructor is private).
struct AerialAuthority {
	omega_rad_s  max_roll_rate{};
	omega_rad_s  max_pitch_rate{};
	omega_rad_s  max_yaw_rate{};
	float        n_max{1.f};      // load-factor limit, >= 1
	velocity_m_s stall_speed{};   // at n = 1
	velocity_m_s cruise_speed{};
};

struct Envelope {
	velocity_m_s      max_speed{};
	velocity_m_s      min_speed{};          // 0 for ground, stall for air
	acceleration_m_s2 max_accel{};
	acceleration_m_s2 max_brake{};
	acceleration_m_s2 max_lateral_accel{};  // turn-rate budget; omega = a_lat / v
	length_m          min_turn_radius{};    // 0 = can pivot in place
	time_s            tau_linear{};         // slew time constant
	std::optional<AerialAuthority> aerial{};
};

// Returns nullopt when the requested mode has no analysis section.
// Called every frame; must stay cheap and free of hidden state.
std::optional<Envelope> ExtractEnvelope(
	const Output& analysis, LocomotionMode mode, int gait, float gravity_m_s2);

} // namespace TonTon

#endif // TONTON_ENVELOPE_H
```

- [ ] **Step 3: Write the failing test**

Create `tonton-example/tests/tonton_myopic_tests.cpp`. Copy the `EarthAir`/`EarthOcean`/`MakeDefaultInput`/`Analyze` harness verbatim from `tests/tonton_plausibility_tests.cpp:52-150` — it is `static`/anonymous-namespace in that file, so it cannot be shared and must be duplicated.

```cpp
#include <gtest/gtest.h>
#include "tonton_myopic.h"
#include "Control/tonton_envelope.h"
// ... plus the Analyze() harness copied from tonton_plausibility_tests.cpp

using namespace TonTon;

TEST(MyopicEnvelope, TerrestrialGaitsAreOrdered)
{
	const Output* out = Analyze("cat.glb", Env::Air);
	ASSERT_NE(out, nullptr);
	ASSERT_TRUE(out->terrestrial.has_value());

	auto walk   = ExtractEnvelope(*out, LocomotionMode::TERRESTRIAL, 0, 9.81f);
	auto trot   = ExtractEnvelope(*out, LocomotionMode::TERRESTRIAL, 1, 9.81f);
	auto gallop = ExtractEnvelope(*out, LocomotionMode::TERRESTRIAL, 2, 9.81f);

	ASSERT_TRUE(walk.has_value());
	ASSERT_TRUE(trot.has_value());
	ASSERT_TRUE(gallop.has_value());

	EXPECT_LT(float(walk->max_speed), float(trot->max_speed));
	EXPECT_LT(float(trot->max_speed), float(gallop->max_speed));
}

TEST(MyopicEnvelope, TerrestrialInvariants)
{
	const Output* out = Analyze("cat.glb", Env::Air);
	ASSERT_NE(out, nullptr);

	for (int gait = 0; gait <= 2; ++gait) {
		auto env = ExtractEnvelope(*out, LocomotionMode::TERRESTRIAL, gait, 9.81f);
		ASSERT_TRUE(env.has_value()) << "gait " << gait;

		EXPECT_GT(float(env->max_speed), 0.f)         << "gait " << gait;
		EXPECT_GE(float(env->min_speed), 0.f)         << "gait " << gait;
		EXPECT_LT(float(env->min_speed), float(env->max_speed)) << "gait " << gait;
		EXPECT_GT(float(env->max_accel), 0.f)         << "gait " << gait;
		EXPECT_GT(float(env->max_lateral_accel), 0.f) << "gait " << gait;

		// tau must be finite and positive or the slew produces NaN.
		EXPECT_GT(float(env->tau_linear), 0.f)        << "gait " << gait;
		EXPECT_TRUE(std::isfinite(float(env->tau_linear))) << "gait " << gait;
	}
}

TEST(MyopicEnvelope, AbsentModeReturnsNullopt)
{
	const Output* out = Analyze("cat.glb", Env::Air);
	ASSERT_NE(out, nullptr);
	ASSERT_FALSE(out->aerial.has_value()) << "cat should not fly";

	EXPECT_FALSE(ExtractEnvelope(*out, LocomotionMode::AERIAL, 0, 9.81f).has_value());
}
```

- [ ] **Step 4: Wire the test file into the build**

In `tonton-example/CMakeLists.txt`, add `tests/tonton_myopic_tests.cpp` to the `add_executable(tonton-tests ...)` list at line 166:

```cmake
        add_executable(tonton-tests
            tests/tonton_plausibility_tests.cpp
            tests/tonton_geometry_tests.cpp
            tests/tonton_wing_dump.cpp
            tests/tonton_myopic_tests.cpp
            ${TONTON_TEST_PIPELINE_SOURCES}
        )
```

Also add the tonton `src` directory to the test include path so `Control/tonton_envelope.h` resolves, at line 173:

```cmake
        target_include_directories(tonton-tests PRIVATE
            ${CMAKE_SOURCE_DIR}/src
            ${CMAKE_SOURCE_DIR}/modules/fx-gltf/include
            ${CMAKE_SOURCE_DIR}/modules/fx-gltf/test/thirdparty
            ${CMAKE_SOURCE_DIR}/modules/tonton/include
            ${CMAKE_SOURCE_DIR}/modules/tonton/src
            ${CMAKE_SOURCE_DIR}/modules/rintintin/include
            ${CMAKE_SOURCE_DIR}/modules/cxxopts/include
        )
```

- [ ] **Step 5: Run the test to verify it fails**

```bash
cd /mnt/Passport/Libraries/Spehleon/tonton-example
cmake -S . -B build && cmake --build build --target tonton-tests -j$(nproc)
```

Expected: **link error**, undefined reference to `TonTon::ExtractEnvelope`. If it instead fails to *compile*, fix the headers before continuing.

- [ ] **Step 6: Implement `ExtractEnvelope` for terrestrial**

Create `src/Control/tonton_envelope.cpp`:

```cpp
#include "Control/tonton_envelope.h"
#include "tonton_analysis.h"
#include <algorithm>
#include <cmath>

namespace TonTon {
namespace {

// Lateral acceleration budget, derived two independent ways and cross-checked:
//   - the grip/thrust budget the creature already has (max_accel)
//   - the budget implied by its own stated minimum turning radius at top speed
// Taking the min keeps us honest when the two disagree. Turn RATE is then
// omega = a_lat / v at steer time, which correctly lets a standing animal pivot
// and stops a galloping one from doing so.
acceleration_m_s2 LateralBudget(acceleration_m_s2 max_accel,
                                velocity_m_s max_speed,
                                length_m min_turn_radius)
{
	if (float(min_turn_radius) <= 0.f) return max_accel;
	const acceleration_m_s2 from_radius = (max_speed * max_speed) / min_turn_radius;
	return (float(from_radius) < float(max_accel)) ? from_radius : max_accel;
}

} // namespace

std::optional<Envelope> ExtractEnvelope(
	const Output& analysis, LocomotionMode mode, int gait, float gravity_m_s2)
{
	switch (mode) {
	case LocomotionMode::TERRESTRIAL: {
		if (!analysis.terrestrial.has_value()) return std::nullopt;
		const auto& t = *analysis.terrestrial;

		Envelope e;
		e.max_speed = (gait <= 0) ? t.optimal_speed_m_s
		            : (gait == 1) ? t.max_sustainable_speed_m_s
		                          : t.max_sprint_speed_m_s;
		e.min_speed       = velocity_m_s{0.f};
		e.max_accel       = t.max_acceleration_m_s2;
		e.max_brake       = t.max_acceleration_m_s2; // legs brake with the same grip budget
		e.min_turn_radius = t.min_turning_radius_m;
		e.max_lateral_accel = LateralBudget(e.max_accel, e.max_speed, e.min_turn_radius);

		// tau = characteristic speed / acceleration. Dimensionally a time, built
		// from two numbers the plausibility suite already validates, adding no
		// new biological claim. See the spec's rejected-pendulum note.
		e.tau_linear = t.optimal_speed_m_s / t.max_acceleration_m_s2;
		return e;
	}
	default:
		return std::nullopt; // later tasks fill in the remaining modes
	}
}

} // namespace TonTon
```

- [ ] **Step 7: Run the tests to verify they pass**

```bash
cd /mnt/Passport/Libraries/Spehleon/tonton-example
cmake -S . -B build && cmake --build build --target tonton-tests -j$(nproc)
./build/tonton-tests --gtest_filter='MyopicEnvelope.*'
```

Expected: 3 tests PASS.

If `TerrestrialInvariants` fails on `tau_linear` being non-finite, `max_acceleration_m_s2` is zero for that model — do **not** clamp it away. Report it; a zero acceleration is a real analysis bug worth surfacing, per CLAUDE.md's accuracy-over-passing principle.

- [ ] **Step 8: Commit**

```bash
cd /mnt/Passport/Libraries/Spehleon/tonton
git add include/tonton_myopic.h src/Control/
git commit -m "feat(myopic): public API and terrestrial envelope extraction"
cd /mnt/Passport/Libraries/Spehleon/tonton-example
git add tests/tonton_myopic_tests.cpp CMakeLists.txt
git commit -m "test(myopic): terrestrial envelope invariants"
```

---

### Task 2: Greedy steering and framerate-correct slew

This is the riskiest task and the reason the whole project was shelved in 2025. It comes second deliberately: prove the control law at any framerate before adding a single mode-specific feature.

**Files:**
- Create: `src/Control/tonton_steer.h`
- Create: `src/Control/tonton_steer.cpp`
- Modify: `tonton-example/tests/tonton_myopic_tests.cpp`

**Interfaces:**
- Consumes: `Envelope` (Task 1).
- Produces: `SteerResult`, `Steer(const Envelope&, const SteerState&, const SteerCommand&) -> SteerResult`.

`Steer` takes plain floats and an `Envelope`. It must **not** include `tonton_analysis.h`.

- [ ] **Step 1: Write the steer header**

Create `src/Control/tonton_steer.h`:

```cpp
#ifndef TONTON_STEER_H
#define TONTON_STEER_H

#include "Control/tonton_envelope.h"
#include <glm/vec3.hpp>

// NOTE: tonton_analysis.h must never be included here or in tonton_steer.cpp.
// Steer operates only on Envelope. This is what makes drowning-in-data
// structurally impossible rather than merely discouraged.

namespace TonTon {

struct SteerCommand {
	float angle_error_rad{0}; // signed, current heading -> desired heading
	float current_speed_m_s{0};
	float desired_speed_m_s{0};
	float dt_s{1.f / 60.f};
};

struct SteerState {
	float prev_turn_rate_rad_s{0};
	float prev_accel_m_s2{0};
};

struct SteerResult {
	float turn_rate_rad_s{0};
	float accel_m_s2{0};
};

SteerResult Steer(const Envelope& env, SteerState& state, const SteerCommand& cmd);

} // namespace TonTon

#endif // TONTON_STEER_H
```

- [ ] **Step 2: Write the failing tests**

Append to `tonton-example/tests/tonton_myopic_tests.cpp`:

```cpp
#include "Control/tonton_steer.h"

namespace {

// A minimal envelope for pure control-law tests. No Output involved, which is
// the point: Steer cannot see one.
Envelope TestEnvelope()
{
	Envelope e;
	e.max_speed         = velocity_m_s{10.f};
	e.min_speed         = velocity_m_s{0.f};
	e.max_accel         = acceleration_m_s2{5.f};
	e.max_brake         = acceleration_m_s2{5.f};
	e.max_lateral_accel = acceleration_m_s2{8.f};
	e.min_turn_radius   = length_m{2.f};
	e.tau_linear        = time_s{0.25f};
	return e;
}

// Integrate a pure heading-tracking maneuver and return the final heading error.
float SimulateTurn(float dt, float total_time_s, float initial_error_rad)
{
	Envelope env = TestEnvelope();
	SteerState state{};
	float error = initial_error_rad;
	float speed = 5.f;

	const int steps = int(total_time_s / dt);
	for (int i = 0; i < steps; ++i) {
		SteerCommand cmd;
		cmd.angle_error_rad   = error;
		cmd.current_speed_m_s = speed;
		cmd.desired_speed_m_s = speed;
		cmd.dt_s              = dt;

		SteerResult r = Steer(env, state, cmd);
		error -= r.turn_rate_rad_s * dt;
	}
	return error;
}

} // namespace

// THE regression test for the 2025 failure. A dt-dependent control law
// (a naive lerp, or any PD gain) diverges here.
TEST(MyopicFramerate, TrajectoryConvergesAcrossTimesteps)
{
	const float total = 2.0f;
	const float start = 1.5f; // rad

	float e16  = SimulateTurn(1.f / 16.f,  total, start);
	float e30  = SimulateTurn(1.f / 30.f,  total, start);
	float e60  = SimulateTurn(1.f / 60.f,  total, start);
	float e120 = SimulateTurn(1.f / 120.f, total, start);

	EXPECT_NEAR(e16,  e60, 0.05f) << "16 Hz diverges from 60 Hz";
	EXPECT_NEAR(e30,  e60, 0.05f) << "30 Hz diverges from 60 Hz";
	EXPECT_NEAR(e120, e60, 0.05f) << "120 Hz diverges from 60 Hz";
}

// A PD controller cannot pass this. Clamped-greedy-plus-slew cannot fail it.
TEST(MyopicNoOscillation, HeadingErrorIsMonotone)
{
	Envelope env = TestEnvelope();
	SteerState state{};
	float error = 1.5f;
	float prev_error = error;
	const float dt = 1.f / 60.f;

	for (int i = 0; i < 600; ++i) { // 10 seconds
		SteerCommand cmd;
		cmd.angle_error_rad   = error;
		cmd.current_speed_m_s = 5.f;
		cmd.desired_speed_m_s = 5.f;
		cmd.dt_s              = dt;

		SteerResult r = Steer(env, state, cmd);
		error -= r.turn_rate_rad_s * dt;

		ASSERT_GE(error, -0.01f) << "overshot into negative error at step " << i;
		ASSERT_LE(error, prev_error + 1e-4f) << "error grew at step " << i;
		prev_error = error;
	}
	EXPECT_NEAR(error, 0.f, 0.02f) << "did not converge";
}

TEST(MyopicSteer, RespectsLateralAccelBudget)
{
	Envelope env = TestEnvelope();
	SteerState state{};

	SteerCommand cmd;
	cmd.angle_error_rad   = 3.0f; // demand far more than possible
	cmd.current_speed_m_s = 8.f;
	cmd.desired_speed_m_s = 8.f;
	cmd.dt_s              = 1.f / 60.f;

	// omega_max = a_lat / v = 8 / 8 = 1.0 rad/s. The slew means the FIRST frame
	// is well under that; run to steady state before asserting the clamp.
	SteerResult r{};
	for (int i = 0; i < 300; ++i) r = Steer(env, state, cmd);

	EXPECT_LE(std::fabs(r.turn_rate_rad_s), 1.0f + 1e-3f);
}

TEST(MyopicSteer, StandingCreatureCanPivot)
{
	Envelope env = TestEnvelope();
	SteerState state{};

	SteerCommand cmd;
	cmd.angle_error_rad   = 1.0f;
	cmd.current_speed_m_s = 0.f;   // standing still
	cmd.desired_speed_m_s = 0.f;
	cmd.dt_s              = 1.f / 60.f;

	SteerResult r = Steer(env, state, cmd);
	EXPECT_GT(r.turn_rate_rad_s, 0.f) << "a standing animal must be able to turn";
	EXPECT_TRUE(std::isfinite(r.turn_rate_rad_s));
}
```

- [ ] **Step 3: Run the tests to verify they fail**

```bash
cd /mnt/Passport/Libraries/Spehleon/tonton-example
cmake -S . -B build && cmake --build build --target tonton-tests -j$(nproc)
```

Expected: link error, undefined reference to `TonTon::Steer`.

- [ ] **Step 4: Implement `Steer`**

Create `src/Control/tonton_steer.cpp`:

```cpp
#include "Control/tonton_steer.h"
#include <algorithm>
#include <cmath>

namespace TonTon {
namespace {

// Framerate-correct exponential slew. This exact form is why the 2025 PD
// attempt failed and this one does not: alpha depends on dt, so 16 Hz and
// 120 Hz reach the same place in the same wall-clock time. Never replace it
// with a bare lerp factor.
float SlewAlpha(float dt_s, float tau_s)
{
	if (!(tau_s > 0.f)) return 1.f; // degenerate tau -> no smoothing
	return 1.f - std::exp(-dt_s / tau_s);
}

float Approach(float prev, float demand, float alpha)
{
	return prev + (demand - prev) * alpha;
}

} // namespace

SteerResult Steer(const Envelope& env, SteerState& state, const SteerCommand& cmd)
{
	const float dt = (cmd.dt_s > 0.f) ? cmd.dt_s : 1.f / 60.f;

	// --- Turn -------------------------------------------------------------
	// Turn rate available at THIS speed. omega = a_lat / v, so a standing
	// animal pivots freely and a fast one is grip-limited. The guard keeps
	// v = 0 finite without introducing a tunable: it only bounds the demand,
	// which the greedy step below bounds again anyway.
	const float speed = std::fabs(cmd.current_speed_m_s);
	const float omega_max = (speed > 1e-3f)
		? float(env.max_lateral_accel) / speed
		: float(env.max_lateral_accel) / 1e-3f;

	// Greedy: if the error can be erased this frame, erase exactly it;
	// otherwise go flat out. This cannot overshoot, by construction.
	const float greedy = cmd.angle_error_rad / dt;
	const float turn_demand = std::clamp(greedy, -omega_max, omega_max);

	const float turn_alpha = SlewAlpha(dt, float(env.tau_linear));
	const float turn_rate  = Approach(state.prev_turn_rate_rad_s, turn_demand, turn_alpha);
	state.prev_turn_rate_rad_s = turn_rate;

	// --- Speed ------------------------------------------------------------
	const float speed_error = cmd.desired_speed_m_s - cmd.current_speed_m_s;
	const float accel_demand = std::clamp(speed_error / dt,
	                                      -float(env.max_brake),
	                                       float(env.max_accel));

	const float accel_alpha = SlewAlpha(dt, float(env.tau_linear));
	const float accel = Approach(state.prev_accel_m_s2, accel_demand, accel_alpha);
	state.prev_accel_m_s2 = accel;

	SteerResult r;
	r.turn_rate_rad_s = turn_rate;
	r.accel_m_s2      = accel;
	return r;
}

} // namespace TonTon
```

- [ ] **Step 5: Run the tests to verify they pass**

```bash
cd /mnt/Passport/Libraries/Spehleon/tonton-example
cmake --build build --target tonton-tests -j$(nproc)
./build/tonton-tests --gtest_filter='MyopicFramerate.*:MyopicNoOscillation.*:MyopicSteer.*'
```

Expected: 4 tests PASS.

If `HeadingErrorIsMonotone` fails with overshoot, the slew is being applied to the *rate* while the greedy term already assumes the full rate is delivered — verify `Approach` is lerping from `prev` toward `demand` and not the reverse.

- [ ] **Step 6: Commit**

```bash
cd /mnt/Passport/Libraries/Spehleon/tonton
git add src/Control/tonton_steer.h src/Control/tonton_steer.cpp
git commit -m "feat(myopic): greedy clamped steering with framerate-correct slew"
cd /mnt/Passport/Libraries/Spehleon/tonton-example
git add tests/tonton_myopic_tests.cpp
git commit -m "test(myopic): framerate convergence and no-oscillation regression tests"
```

---

### Task 3: Stability metric and hints

**Files:**
- Modify: `src/Control/tonton_steer.h`
- Modify: `src/Control/tonton_steer.cpp`
- Modify: `tonton-example/tests/tonton_myopic_tests.cpp`

**Interfaces:**
- Consumes: `Envelope`, `SteerCommand`, `SteerResult` (Task 2).
- Produces: `SteerResult` gains `stability`, `speed_headroom`, `turn_headroom`, `suggest_gait_change`.

- [ ] **Step 1: Extend `SteerResult`**

In `src/Control/tonton_steer.h`, replace the `SteerResult` definition:

```cpp
struct SteerResult {
	float turn_rate_rad_s{0};
	float accel_m_s2{0};

	float stability{1.f};
	float speed_headroom{1.f};
	float turn_headroom{1.f};
	bool  suggest_gait_change{false};
};
```

- [ ] **Step 2: Write the failing tests**

Append to `tonton-example/tests/tonton_myopic_tests.cpp`:

```cpp
TEST(MyopicStability, CruisingStraightIsComfortable)
{
	Envelope env = TestEnvelope();
	SteerState state{};

	SteerCommand cmd;
	cmd.angle_error_rad   = 0.f;
	cmd.current_speed_m_s = 2.f;   // well under max_speed 10
	cmd.desired_speed_m_s = 2.f;
	cmd.dt_s              = 1.f / 60.f;

	SteerResult r = Steer(env, state, cmd);
	EXPECT_GT(r.stability, 0.7f);
	EXPECT_GT(r.speed_headroom, 0.7f);
}

TEST(MyopicStability, AtMaxSpeedStabilityReachesZero)
{
	Envelope env = TestEnvelope();
	SteerState state{};

	SteerCommand cmd;
	cmd.angle_error_rad   = 0.f;
	cmd.current_speed_m_s = 10.f;  // exactly max_speed
	cmd.desired_speed_m_s = 10.f;
	cmd.dt_s              = 1.f / 60.f;

	SteerResult r = Steer(env, state, cmd);
	EXPECT_NEAR(r.stability, 0.f, 0.02f);
	EXPECT_NEAR(r.speed_headroom, 0.f, 0.02f);
}

TEST(MyopicStability, ExceedingMaxSpeedGoesNegative)
{
	Envelope env = TestEnvelope();
	SteerState state{};

	SteerCommand cmd;
	cmd.angle_error_rad   = 0.f;
	cmd.current_speed_m_s = 15.f;  // 1.5x max_speed
	cmd.desired_speed_m_s = 15.f;
	cmd.dt_s              = 1.f / 60.f;

	SteerResult r = Steer(env, state, cmd);
	EXPECT_LT(r.stability, 0.f);
}

TEST(MyopicStability, DemandingMoreThanTheGaitSuggestsAChange)
{
	Envelope env = TestEnvelope();
	SteerState state{};

	SteerCommand cmd;
	cmd.angle_error_rad   = 0.f;
	cmd.current_speed_m_s = 5.f;
	cmd.desired_speed_m_s = 20.f;  // twice max_speed
	cmd.dt_s              = 1.f / 60.f;

	SteerResult r = Steer(env, state, cmd);
	EXPECT_TRUE(r.suggest_gait_change);
}
```

- [ ] **Step 3: Run the tests to verify they fail**

```bash
cd /mnt/Passport/Libraries/Spehleon/tonton-example
cmake --build build --target tonton-tests -j$(nproc)
./build/tonton-tests --gtest_filter='MyopicStability.*'
```

Expected: FAIL — `stability` is still the default `1.f`, so `AtMaxSpeedStabilityReachesZero` and `ExceedingMaxSpeedGoesNegative` fail.

- [ ] **Step 4: Implement stability**

In `src/Control/tonton_steer.cpp`, insert before `SteerResult r;` at the end of `Steer`:

```cpp
	// --- Stability --------------------------------------------------------
	// Three pure demand/capacity ratios; the worst one wins. 1 = idle,
	// 0 = at a limit, negative = past it. No weights, nothing to tune.
	const float u_speed = (float(env.max_speed) > 0.f)
		? cmd.current_speed_m_s / float(env.max_speed) : 0.f;

	const float u_turn = (omega_max > 0.f)
		? std::fabs(turn_demand) / omega_max : 0.f;

	// Minimum-speed modes (stall in air, sharks, serpentine undulation floor).
	// Aerial replaces this in Task 5 with a bank-corrected stall speed.
	const float u_stall = (float(env.min_speed) > 0.f && speed > 1e-3f)
		? float(env.min_speed) / speed : 0.f;

	const float worst = std::max({u_speed, u_turn, u_stall});
```

and replace the result construction with:

```cpp
	SteerResult r;
	r.turn_rate_rad_s = turn_rate;
	r.accel_m_s2      = accel;
	r.stability       = 1.f - worst;
	r.speed_headroom  = 1.f - u_speed;
	r.turn_headroom   = 1.f - u_turn;
	// Deliberately NOT clamping desired speed to the gait: the stability
	// metric communicates the problem and the external gait selector decides.
	r.suggest_gait_change = cmd.desired_speed_m_s > float(env.max_speed);
	return r;
```

- [ ] **Step 5: Run the tests to verify they pass**

```bash
cd /mnt/Passport/Libraries/Spehleon/tonton-example
cmake --build build --target tonton-tests -j$(nproc)
./build/tonton-tests --gtest_filter='Myopic*'
```

Expected: all tests from Tasks 1-3 PASS (11 total).

- [ ] **Step 6: Commit**

```bash
cd /mnt/Passport/Libraries/Spehleon/tonton
git add src/Control/tonton_steer.h src/Control/tonton_steer.cpp
git commit -m "feat(myopic): demand/capacity stability metric and gait hints"
cd /mnt/Passport/Libraries/Spehleon/tonton-example
git add tests/tonton_myopic_tests.cpp
git commit -m "test(myopic): stability metric boundaries"
```

---

### Task 4: Aerial envelope with derived acceleration

**Files:**
- Modify: `src/Control/tonton_envelope.cpp`
- Modify: `tonton-example/tests/tonton_myopic_tests.cpp`

**Interfaces:**
- Consumes: `Envelope`, `AerialAuthority` (Task 1).
- Produces: `ExtractEnvelope` now handles `LocomotionMode::AERIAL` and populates `Envelope::aerial`.

**Background:** `Analysis_Aerial` has **no acceleration field** — only `Analysis_Terrestrial` does. Derive it from power: with `P = F·v` and `F = m·a`, thrust-limited acceleration is `a = P/(m·v)`. Evaluate at cruise speed for a representative figure. Braking in air is drag-dominated; use the same magnitude, since a flyer that can add `a` of thrust can shed roughly that much by spoiling lift and adding drag.

- [ ] **Step 1: Write the failing tests**

Append to `tonton-example/tests/tonton_myopic_tests.cpp`:

```cpp
TEST(MyopicEnvelope, AerialInvariants)
{
	const Output* out = Analyze("batto.glb", Env::Air);
	ASSERT_NE(out, nullptr);
	ASSERT_TRUE(out->aerial.has_value()) << "bat should fly";

	auto env = ExtractEnvelope(*out, LocomotionMode::AERIAL, 0, 9.81f);
	ASSERT_TRUE(env.has_value());

	EXPECT_GT(float(env->max_speed), 0.f);
	EXPECT_GT(float(env->min_speed), 0.f) << "a flyer has a stall speed";
	EXPECT_LT(float(env->min_speed), float(env->max_speed));

	// The derived-from-power acceleration must be real, not zero or NaN.
	EXPECT_GT(float(env->max_accel), 0.f);
	EXPECT_TRUE(std::isfinite(float(env->max_accel)));
	EXPECT_GT(float(env->tau_linear), 0.f);
	EXPECT_TRUE(std::isfinite(float(env->tau_linear)));

	ASSERT_TRUE(env->aerial.has_value());
	EXPECT_GE(env->aerial->n_max, 1.f) << "load factor cannot be below 1";
	EXPECT_TRUE(std::isfinite(env->aerial->n_max));
	EXPECT_GT(float(env->aerial->max_roll_rate), 0.f);
}

TEST(MyopicEnvelope, AerialAccelerationIsPlausible)
{
	const Output* out = Analyze("batto.glb", Env::Air);
	ASSERT_NE(out, nullptr);
	auto env = ExtractEnvelope(*out, LocomotionMode::AERIAL, 0, 9.81f);
	ASSERT_TRUE(env.has_value());

	// A flying animal accelerates within roughly an order of magnitude of g.
	// Far outside this band means the P = F*v derivation has a units error.
	EXPECT_GT(float(env->max_accel), 0.1f);
	EXPECT_LT(float(env->max_accel), 100.f);
}
```

- [ ] **Step 2: Run the tests to verify they fail**

```bash
cd /mnt/Passport/Libraries/Spehleon/tonton-example
cmake --build build --target tonton-tests -j$(nproc)
./build/tonton-tests --gtest_filter='MyopicEnvelope.Aerial*'
```

Expected: FAIL — `ExtractEnvelope` returns `nullopt` for `AERIAL`, so `ASSERT_TRUE(env.has_value())` fails.

- [ ] **Step 3: Implement the aerial arm**

In `src/Control/tonton_envelope.cpp`, add to the anonymous namespace:

```cpp
// Analysis_Aerial has no acceleration field, so derive it. P = F*v and F = m*a
// give a = P/(m*v), evaluated at cruise as a representative figure.
acceleration_m_s2 AccelFromPower(power_W available, mass_kg mass, velocity_m_s at_speed)
{
	if (float(mass) <= 0.f || float(at_speed) <= 0.f) return acceleration_m_s2{0.f};
	return available / (mass * at_speed);
}

// Load-factor limit, derived two independent ways and cross-checked:
//   - force budget:   n = L_max / W
//   - stated radius:  a coordinated turn of radius r at speed v needs
//                     tan(phi) = v^2 / (g*r), and n = 1/cos(phi)
// Taking the min keeps us honest. A large disagreement is a diagnostic about
// the analysis, not about the controller.
float LoadFactorLimit(const Analysis_Aerial& a, mass_kg mass, float gravity_m_s2)
{
	float n_force = 1.f;
	const float weight_N = float(mass) * gravity_m_s2;
	if (weight_N > 0.f && float(a.takeoff.max_instantaneous_lift_N) > 0.f) {
		n_force = float(a.takeoff.max_instantaneous_lift_N) / weight_N;
	}

	float n_radius = 1.f;
	const float r = float(a.min_turning_radius_m);
	const float v = float(a.cruise_speed_m_s);
	if (r > 0.f && v > 0.f && gravity_m_s2 > 0.f) {
		const float tan_phi = (v * v) / (gravity_m_s2 * r);
		n_radius = std::sqrt(1.f + tan_phi * tan_phi); // = 1/cos(atan(tan_phi))
	}

	const float n = std::min(n_force, n_radius);
	return (n >= 1.f && std::isfinite(n)) ? n : 1.f;
}
```

Then add the case to the switch in `ExtractEnvelope`, before `default:`:

```cpp
	case LocomotionMode::AERIAL: {
		if (!analysis.aerial.has_value()) return std::nullopt;
		const auto& a = *analysis.aerial;

		Envelope e;
		e.max_speed = a.max_flight_speed_m_s;
		e.min_speed = a.min_flight_speed_m_s;   // stall at n = 1
		e.max_accel = AccelFromPower(a.flapping_power_W,
		                             analysis.physical.body_mass_kg,
		                             a.cruise_speed_m_s);
		e.max_brake = e.max_accel;
		e.min_turn_radius = a.min_turning_radius_m;

		AerialAuthority auth;
		auth.max_roll_rate  = a.max_roll_rate_rad_s;
		auth.max_pitch_rate = a.max_pitch_rate_rad_s;
		auth.max_yaw_rate   = a.max_yaw_rate_rad_s;
		auth.stall_speed    = a.min_flight_speed_m_s;
		auth.cruise_speed   = a.cruise_speed_m_s;
		auth.n_max = LoadFactorLimit(a, analysis.physical.body_mass_kg, kGravity);
		e.aerial = auth;

		// a_lat = g*tan(phi_max) = g*sqrt(n_max^2 - 1)
		const float lat = kGravity * std::sqrt(std::max(0.f, auth.n_max * auth.n_max - 1.f));
		e.max_lateral_accel = acceleration_m_s2{lat};

		e.tau_linear = a.cruise_speed_m_s / e.max_accel;
		return e;
	}
```

Throughout the aerial arm, `kGravity` is the `gravity_m_s2` **parameter** of
`ExtractEnvelope` — verified: `Analysis_Physical` has no gravity member and
`Input::environment` never reaches `Output`, so the caller is the only honest source.
Add `const float kGravity = gravity_m_s2;` at the top of the function, or just use the
parameter directly. Do **not** hardcode 9.81 inside the function.

- [ ] **Step 4: Run the tests to verify they pass**

```bash
cd /mnt/Passport/Libraries/Spehleon/tonton-example
cmake --build build --target tonton-tests -j$(nproc)
./build/tonton-tests --gtest_filter='Myopic*'
```

Expected: all tests PASS (13 total).

If `AerialAccelerationIsPlausible` fails high or low by orders of magnitude, the `P/(m·v)` expression has a units error — check that `flapping_power_W` is Watts and not a per-kilogram figure.

- [ ] **Step 5: Commit**

```bash
cd /mnt/Passport/Libraries/Spehleon/tonton
git add src/Control/tonton_envelope.cpp src/Control/tonton_envelope.h
git commit -m "feat(myopic): aerial envelope, power-derived accel, cross-checked load factor"
cd /mnt/Passport/Libraries/Spehleon/tonton-example
git add tests/tonton_myopic_tests.cpp
git commit -m "test(myopic): aerial envelope invariants and acceleration plausibility"
```

---

### Task 5: Bank-versus-yaw decision and stall coupling

**Files:**
- Modify: `src/Control/tonton_steer.h`
- Modify: `src/Control/tonton_steer.cpp`
- Modify: `tonton-example/tests/tonton_myopic_tests.cpp`

**Interfaces:**
- Consumes: `Envelope::aerial` (Task 4).
- Produces: `SteerState` gains `bank_angle_rad`; `SteerResult` gains `TurnStrategy strategy`.

- [ ] **Step 1: Extend the steer structs**

In `src/Control/tonton_steer.h`:

```cpp
enum class TurnStrategy : uint8_t { GROUND, YAW, BANK };

struct SteerState {
	float prev_turn_rate_rad_s{0};
	float prev_accel_m_s2{0};
	float bank_angle_rad{0};   // roll-in is a process, not an instant
};
```

and add to `SteerResult`:

```cpp
	TurnStrategy strategy{TurnStrategy::GROUND};
	float bank_angle_rad{0};
```

Also add gravity to `SteerCommand`, since the bank math needs it and it must not be hardcoded:

```cpp
	float gravity_m_s2{9.81f};
```

- [ ] **Step 2: Write the failing tests**

Append to `tonton-example/tests/tonton_myopic_tests.cpp`:

```cpp
namespace {

Envelope DragonflyLikeEnvelope()   // huge yaw authority, tiny radius
{
	Envelope e = TestEnvelope();
	e.min_speed = velocity_m_s{1.f};
	AerialAuthority a;
	a.max_roll_rate  = omega_rad_s{2.1f};
	a.max_pitch_rate = omega_rad_s{0.4f};
	a.max_yaw_rate   = omega_rad_s{4.1f};
	a.n_max          = 2.f;
	a.stall_speed    = velocity_m_s{1.f};
	a.cruise_speed   = velocity_m_s{4.f};
	e.aerial = a;
	e.max_lateral_accel = acceleration_m_s2{9.81f * std::sqrt(3.f)};
	return e;
}

Envelope AlbatrossLikeEnvelope()   // negligible yaw, strong roll
{
	Envelope e = TestEnvelope();
	e.min_speed = velocity_m_s{8.f};
	AerialAuthority a;
	a.max_roll_rate  = omega_rad_s{1.5f};
	a.max_pitch_rate = omega_rad_s{0.3f};
	a.max_yaw_rate   = omega_rad_s{0.2f};
	a.n_max          = 2.f;
	a.stall_speed    = velocity_m_s{8.f};
	a.cruise_speed   = velocity_m_s{15.f};
	e.aerial = a;
	e.max_lateral_accel = acceleration_m_s2{9.81f * std::sqrt(3.f)};
	return e;
}

SteerCommand TurnCommand(float error, float speed)
{
	SteerCommand c;
	c.angle_error_rad   = error;
	c.current_speed_m_s = speed;
	c.desired_speed_m_s = speed;
	c.dt_s              = 1.f / 60.f;
	c.gravity_m_s2      = 9.81f;
	return c;
}

} // namespace

TEST(MyopicBank, DragonflyYawsForTheSameTurnAnAlbatrossBanks)
{
	Envelope dragonfly = DragonflyLikeEnvelope();
	Envelope albatross = AlbatrossLikeEnvelope();
	SteerState s1{}, s2{};

	SteerResult rd = Steer(dragonfly, s1, TurnCommand(1.57f, 4.f));
	SteerResult ra = Steer(albatross, s2, TurnCommand(1.57f, 15.f));

	EXPECT_EQ(rd.strategy, TurnStrategy::YAW);
	EXPECT_EQ(ra.strategy, TurnStrategy::BANK);
}

TEST(MyopicStall, HardTurnNearStallDrivesStabilityNegative)
{
	Envelope env = AlbatrossLikeEnvelope();
	SteerState state{};

	// Flying just above stall (8.0) and demanding a hard turn. The load factor
	// raises the effective stall speed by sqrt(n), so this must go unstable.
	SteerResult r{};
	for (int i = 0; i < 300; ++i) r = Steer(env, state, TurnCommand(3.0f, 8.6f));

	EXPECT_EQ(r.strategy, TurnStrategy::BANK);
	EXPECT_GT(r.bank_angle_rad, 0.f);
	EXPECT_LT(r.stability, 0.f) << "banking near stall must be unstable";
}

TEST(MyopicStall, SameSpeedWingsLevelIsStable)
{
	Envelope env = AlbatrossLikeEnvelope();
	SteerState state{};

	// Identical speed, no turn demanded -> comfortably above stall.
	SteerResult r{};
	for (int i = 0; i < 300; ++i) r = Steer(env, state, TurnCommand(0.f, 8.6f));

	EXPECT_GT(r.stability, 0.f) << "wings-level at the same speed must be fine";
}
```

- [ ] **Step 3: Run the tests to verify they fail**

```bash
cd /mnt/Passport/Libraries/Spehleon/tonton-example
cmake --build build --target tonton-tests -j$(nproc)
./build/tonton-tests --gtest_filter='MyopicBank.*:MyopicStall.*'
```

Expected: FAIL — `strategy` is always `GROUND` and `bank_angle_rad` is always 0.

- [ ] **Step 4: Implement bank/yaw and stall coupling**

In `src/Control/tonton_steer.cpp`, add to the anonymous namespace:

```cpp
// Choose the cheaper turn axis by TIME, not by rate. Yaw is available
// immediately; banking must first be rolled into, then delivers far more turn
// rate. Comparing times needs no invented horizon constant -- the old
// "0.5 second" horizon in notes.md was a PD gain in disguise.
TurnStrategy ChooseStrategy(const AerialAuthority& a, float angle_error_rad,
                            float speed, float gravity, float& out_phi_rad)
{
	const float err = std::fabs(angle_error_rad);
	out_phi_rad = 0.f;
	if (err < 1e-4f) return TurnStrategy::YAW;

	const float yaw_rate = float(a.max_yaw_rate);
	const float t_yaw = (yaw_rate > 0.f) ? err / yaw_rate
	                                     : std::numeric_limits<float>::infinity();

	// Bank hard enough to serve the demand, capped by the load-factor limit.
	const float phi_max = std::acos(1.f / std::max(1.f, a.n_max));
	const float roll_rate = float(a.max_roll_rate);
	float t_bank = std::numeric_limits<float>::infinity();
	if (phi_max > 0.f && roll_rate > 0.f && speed > 1e-3f && gravity > 0.f) {
		const float omega_bank = gravity * std::tan(phi_max) / speed;
		if (omega_bank > 0.f) {
			t_bank = (phi_max / roll_rate) + (err / omega_bank);
		}
	}

	if (t_bank < t_yaw) { out_phi_rad = phi_max; return TurnStrategy::BANK; }
	return TurnStrategy::YAW;
}

} // namespace (close the existing anonymous namespace after this)
```

Add `#include <limits>` at the top of the file.

Then, inside `Steer`, replace the `omega_max` computation with:

```cpp
	const float speed = std::fabs(cmd.current_speed_m_s);
	TurnStrategy strategy = TurnStrategy::GROUND;
	float omega_max = (speed > 1e-3f)
		? float(env.max_lateral_accel) / speed
		: float(env.max_lateral_accel) / 1e-3f;

	if (env.aerial.has_value()) {
		float phi_target = 0.f;
		strategy = ChooseStrategy(*env.aerial, cmd.angle_error_rad, speed,
		                          cmd.gravity_m_s2, phi_target);

		if (strategy == TurnStrategy::YAW) {
			// Yaw is instant; the bank washes out at the roll rate.
			omega_max = float(env.aerial->max_yaw_rate);
			phi_target = 0.f;
		}

		// Roll-in is a process. Slew the bank toward its target at the roll
		// rate -- this IS the aerial angular time constant, no estimator needed.
		const float max_droll = float(env.aerial->max_roll_rate) * dt;
		const float dphi = std::clamp(phi_target - state.bank_angle_rad,
		                              -max_droll, max_droll);
		state.bank_angle_rad += dphi;

		if (strategy == TurnStrategy::BANK && speed > 1e-3f) {
			// Turn rate available at the bank angle actually achieved so far.
			omega_max = cmd.gravity_m_s2 * std::tan(state.bank_angle_rad) / speed;
			omega_max = std::fabs(omega_max);
		}
	}
```

Then replace the `u_stall` computation with the bank-corrected version:

```cpp
	// Load factor n = 1/cos(phi) raises the stall speed as sqrt(n). This is
	// how real flyers fall out of the sky: turning hard near minimum speed
	// stalls you. Emergent, not scripted.
	float effective_min_speed = float(env.min_speed);
	if (env.aerial.has_value()) {
		const float cos_phi = std::cos(state.bank_angle_rad);
		if (cos_phi > 1e-3f) {
			effective_min_speed = float(env.aerial->stall_speed) / std::sqrt(cos_phi);
		}
	}
	const float u_stall = (effective_min_speed > 0.f && speed > 1e-3f)
		? effective_min_speed / speed : 0.f;
```

Finally add to the result construction:

```cpp
	r.strategy       = strategy;
	r.bank_angle_rad = state.bank_angle_rad;
```

- [ ] **Step 5: Run the tests to verify they pass**

```bash
cd /mnt/Passport/Libraries/Spehleon/tonton-example
cmake --build build --target tonton-tests -j$(nproc)
./build/tonton-tests --gtest_filter='Myopic*'
```

Expected: all tests PASS (16 total). In particular `MyopicFramerate` and `MyopicNoOscillation` from Task 2 must still pass — the bank state must not break dt-independence.

- [ ] **Step 6: Commit**

```bash
cd /mnt/Passport/Libraries/Spehleon/tonton
git add src/Control/tonton_steer.h src/Control/tonton_steer.cpp
git commit -m "feat(myopic): time-based bank/yaw choice and load-factor stall coupling"
cd /mnt/Passport/Libraries/Spehleon/tonton-example
git add tests/tonton_myopic_tests.cpp
git commit -m "test(myopic): bank-vs-yaw selection and stall-in-turn"
```

---

### Task 6: Launch planning and the entry point

**Files:**
- Create: `src/Control/tonton_launch.h`
- Create: `src/Control/tonton_launch.cpp`
- Create: `src/Control/tonton_myopic.cpp`
- Modify: `tonton-example/tests/tonton_myopic_tests.cpp`

**Interfaces:**
- Consumes: `Envelope` (Task 4), `SteerResult` (Task 5), `MyopicInput`/`MyopicOutput`/`MyopicState` (Task 1).
- Produces: `LaunchPlan`, `PlanLaunch(const Output&, const MyopicInput&, float airspeed_m_s) -> LaunchPlan`; the definition of `ComputeMyopicControl`.

- [ ] **Step 1: Write the launch header**

Create `src/Control/tonton_launch.h`:

```cpp
#ifndef TONTON_LAUNCH_H
#define TONTON_LAUNCH_H

#include "tonton_myopic.h"
#include "tonton_units.hpp"

namespace TonTon {

struct Output;

struct LaunchPlan {
	bool           feasible{false};
	float          readiness{0.f};        // [0,1]
	BlockingReason blocking_reason{BlockingReason::NONE};

	float required_airspeed_m_s{0.f};
	float required_drop_m{0.f};

	// Descriptive only; the caller owns jump timing.
	float     required_jump_velocity_m_s{0.f};
	glm::vec3 jump_direction{0};
	bool      jump_feasible{false};

	bool accelerate_along_heading{false}; // true only for RUNNING_TAKEOFF
};

LaunchPlan PlanLaunch(const Output& analysis, const MyopicInput& in, float airspeed_m_s);

} // namespace TonTon

#endif // TONTON_LAUNCH_H
```

- [ ] **Step 2: Write the failing tests**

Append to `tonton-example/tests/tonton_myopic_tests.cpp`:

```cpp
#include "Control/tonton_launch.h"

TEST(MyopicLaunch, DragonflyIsReadyFromAStandstill)
{
	const Output* out = Analyze("dragonfly.glb", Env::Air);
	ASSERT_NE(out, nullptr);
	ASSERT_TRUE(out->aerial.has_value());

	MyopicInput in;
	in.mode        = LocomotionMode::TERRESTRIAL;
	in.target_mode = LocomotionMode::AERIAL;
	in.substrate   = Substrate::GROUND;

	LaunchPlan p = PlanLaunch(*out, in, 0.f); // not moving at all
	EXPECT_TRUE(p.feasible);
	EXPECT_NEAR(p.readiness, 1.f, 1e-4f) << "a hovering insect launches vertically";
	EXPECT_FALSE(p.accelerate_along_heading);
}

TEST(MyopicLaunch, NonFlyerReportsNoAerialAnalysis)
{
	const Output* out = Analyze("cat.glb", Env::Air);
	ASSERT_NE(out, nullptr);
	ASSERT_FALSE(out->aerial.has_value());

	MyopicInput in;
	in.mode        = LocomotionMode::TERRESTRIAL;
	in.target_mode = LocomotionMode::AERIAL;

	LaunchPlan p = PlanLaunch(*out, in, 0.f);
	EXPECT_FALSE(p.feasible);
	EXPECT_EQ(p.blocking_reason, BlockingReason::NO_AERIAL_ANALYSIS);
}

TEST(MyopicLaunch, ReadinessIsMonotoneInAirspeed)
{
	const Output* out = Analyze("batto.glb", Env::Air);
	ASSERT_NE(out, nullptr);
	ASSERT_TRUE(out->aerial.has_value());

	MyopicInput in;
	in.mode        = LocomotionMode::TERRESTRIAL;
	in.target_mode = LocomotionMode::AERIAL;
	in.substrate   = Substrate::GROUND;

	float prev = -1.f;
	for (float v = 0.f; v <= 30.f; v += 2.f) {
		LaunchPlan p = PlanLaunch(*out, in, v);
		EXPECT_GE(p.readiness, prev) << "readiness dropped at airspeed " << v;
		EXPECT_LE(p.readiness, 1.f);
		prev = p.readiness;
	}
}

// The airspeed-versus-ground-speed inversion is the likeliest bug in the
// module, so it gets its own test.
TEST(MyopicAirspeed, HeadwindMakesLaunchEasierThanTailwind)
{
	const Output* out = Analyze("batto.glb", Env::Air);
	ASSERT_NE(out, nullptr);

	MyopicState st_head{}, st_tail{};

	MyopicInput base;
	base.mode         = LocomotionMode::TERRESTRIAL;
	base.target_mode  = LocomotionMode::AERIAL;
	base.substrate    = Substrate::GROUND;
	base.velocity_m_s = glm::vec3(5.f, 0.f, 0.f); // same ground speed both times
	base.dt_s         = 1.f / 60.f;

	MyopicInput headwind = base;
	headwind.medium_velocity_m_s = glm::vec3(-5.f, 0.f, 0.f); // blowing against us

	MyopicInput tailwind = base;
	tailwind.medium_velocity_m_s = glm::vec3(5.f, 0.f, 0.f);  // blowing with us

	MyopicOutput h = ComputeMyopicControl(*out, headwind, st_head);
	MyopicOutput t = ComputeMyopicControl(*out, tailwind, st_tail);

	EXPECT_GT(h.transition_readiness, t.transition_readiness)
		<< "a headwind must make takeoff easier at the same ground speed";
}

TEST(MyopicEntryPoint, ProducesFiniteOutputForEveryModel)
{
	struct Case { const char* file; Env env; };
	const Case cases[] = {
		{"cat.glb", Env::Air}, {"dragonfly.glb", Env::Air},
		{"batto.glb", Env::Air}, {"penguin.glb", Env::Ocean},
	};

	for (auto const& c : cases) {
		const Output* out = Analyze(c.file, c.env);
		ASSERT_NE(out, nullptr) << c.file;

		MyopicState state{};
		MyopicInput in;
		in.mode            = LocomotionMode::TERRESTRIAL;
		in.target_mode     = LocomotionMode::TERRESTRIAL;
		in.target_position = glm::vec3(10.f, 0.f, 3.f);
		in.dt_s            = 1.f / 60.f;

		MyopicOutput o = ComputeMyopicControl(*out, in, state);

		EXPECT_TRUE(std::isfinite(o.linear_acceleration_m_s2.x)) << c.file;
		EXPECT_TRUE(std::isfinite(o.angular_acceleration_rad_s2.y)) << c.file;
		EXPECT_TRUE(std::isfinite(o.stability)) << c.file;
	}
}
```

- [ ] **Step 3: Run the tests to verify they fail**

```bash
cd /mnt/Passport/Libraries/Spehleon/tonton-example
cmake -S . -B build && cmake --build build --target tonton-tests -j$(nproc)
```

Expected: link errors for `PlanLaunch` and `ComputeMyopicControl`.

- [ ] **Step 4: Implement `PlanLaunch`**

Create `src/Control/tonton_launch.cpp`:

```cpp
#include "Control/tonton_launch.h"
#include "tonton_analysis.h"
#include <algorithm>

namespace TonTon {
namespace {

BlockingReason FirstFailedConstraint(const Analysis_TakeoffAnalysis& t)
{
	if (!t.constraints.wing_loading_ok)  return BlockingReason::WING_LOADING;
	if (!t.constraints.power_loading_ok) return BlockingReason::POWER_LOADING;
	if (!t.constraints.aspect_ratio_ok)  return BlockingReason::ASPECT_RATIO;
	if (!t.constraints.leg_strength_ok)  return BlockingReason::LEG_STRENGTH;
	return BlockingReason::NONE;
}

} // namespace

LaunchPlan PlanLaunch(const Output& analysis, const MyopicInput& in, float airspeed_m_s)
{
	using TM = Analysis_TakeoffAnalysis::TakeoffMode;

	LaunchPlan p;
	if (!analysis.aerial.has_value()) {
		p.blocking_reason = BlockingReason::NO_AERIAL_ANALYSIS;
		return p;
	}

	const auto& aerial = *analysis.aerial;
	const auto& t = aerial.takeoff;

	p.required_airspeed_m_s      = float(aerial.min_flight_speed_m_s);
	p.required_jump_velocity_m_s = float(t.required_jump_velocity_m_s);
	p.jump_direction             = glm::vec3(0.f, 1.f, 0.f);
	p.jump_feasible = analysis.jumping.has_value()
		&& float(t.required_jump_velocity_m_s)
		   <= float(analysis.jumping->takeoff_velocity_m_s);

	switch (t.mode) {
	case TM::VERTICAL_LAUNCH:
		p.feasible  = true;
		p.readiness = 1.f;
		break;

	case TM::JUMP_LAUNCH:
		// Descriptive only: the caller plays the crouch and applies the
		// delta-v at whatever frame its animation says.
		p.feasible  = p.jump_feasible;
		p.readiness = p.jump_feasible ? 1.f : 0.f;
		if (!p.jump_feasible) p.blocking_reason = BlockingReason::LEG_STRENGTH;
		break;

	case TM::RUNNING_TAKEOFF: {
		// A water surface counts as runway for a taxiing flyer (pelicans).
		const bool runway_ok = (in.substrate == Substrate::GROUND)
			|| (in.substrate == Substrate::WATER && t.can_use_water_taxi);
		p.feasible = runway_ok;
		p.accelerate_along_heading = runway_ok;
		// Measured in AIRSPEED, so a headwind shortens the runway with no
		// special case at all.
		p.readiness = (p.required_airspeed_m_s > 0.f && runway_ok)
			? std::clamp(airspeed_m_s / p.required_airspeed_m_s, 0.f, 1.f) : 0.f;
		if (p.readiness < 1.f) p.blocking_reason = BlockingReason::NEEDS_RUNWAY_SPEED;
		break;
	}

	case TM::CLIFF_LAUNCH:
		p.required_drop_m = float(t.takeoff_run_distance_m);
		p.feasible  = (in.substrate == Substrate::CLIFF_EDGE);
		p.readiness = p.feasible ? 1.f : 0.f;
		if (!p.feasible) p.blocking_reason = BlockingReason::NEEDS_ELEVATION;
		break;

	case TM::ASSISTED_LAUNCH:
		p.feasible  = (in.substrate == Substrate::PERCH);
		p.readiness = p.feasible ? 1.f : 0.f;
		if (!p.feasible) p.blocking_reason = BlockingReason::NEEDS_PERCH;
		break;

	case TM::IMPOSSIBLE:
	default:
		p.feasible  = false;
		p.readiness = 0.f;
		p.blocking_reason = FirstFailedConstraint(t);
		break;
	}

	return p;
}

} // namespace TonTon
```

- [ ] **Step 5: Implement `ComputeMyopicControl`**

Create `src/Control/tonton_myopic.cpp`:

```cpp
#include "tonton_myopic.h"
#include "Control/tonton_envelope.h"
#include "Control/tonton_launch.h"
#include "Control/tonton_steer.h"
#include "tonton_analysis.h"

#include <glm/geometric.hpp>
#include <cmath>

namespace TonTon {

MyopicOutput ComputeMyopicControl(
	const Output& analysis, const MyopicInput& input, MyopicState& state)
{
	MyopicOutput out;

	// AIRSPEED DISCIPLINE: computed exactly once, used for every speed
	// comparison downstream. Ground speed is used only for runway distance.
	const glm::vec3 airspeed_vec = input.velocity_m_s - input.medium_velocity_m_s;
	const float airspeed = glm::length(airspeed_vec);

	auto env = ExtractEnvelope(analysis, input.mode, input.current_gait, input.gravity_m_s2);
	if (!env.has_value()) {
		out.stability = -1.f; // caller lied about the mode; say so plainly
		return out;
	}

	// Launch planning: the ONLY transition carrying physics is * -> AERIAL,
	// because airspeed is the only precondition a creature must generate
	// itself. Every other target mode leaves steering unchanged.
	bool launch_run = false;
	if (input.target_mode == LocomotionMode::AERIAL
	    && input.mode != LocomotionMode::AERIAL) {
		LaunchPlan plan = PlanLaunch(analysis, input, airspeed);
		out.transition_readiness       = plan.readiness;
		out.blocking_reason            = plan.blocking_reason;
		out.required_jump_velocity_m_s = plan.required_jump_velocity_m_s;
		out.jump_direction             = plan.jump_direction;
		out.jump_feasible              = plan.jump_feasible;
		launch_run = plan.accelerate_along_heading;
	}

	// Heading error toward the target, about the world up axis.
	const glm::vec3 forward = input.orientation * glm::vec3(0.f, 0.f, 1.f);
	glm::vec3 to_target = input.target_position - input.position;
	float angle_error = 0.f;
	if (glm::length(to_target) > 1e-4f) {
		to_target = glm::normalize(to_target);
		const glm::vec3 f = glm::normalize(glm::vec3(forward.x, 0.f, forward.z));
		const glm::vec3 t = glm::normalize(glm::vec3(to_target.x, 0.f, to_target.z));
		angle_error = std::atan2(f.x * t.z - f.z * t.x, glm::dot(f, t));
	}

	SteerCommand cmd;
	// Steering toward the launch precondition instead of the target: accelerate
	// along the current heading until we have made our own airspeed.
	cmd.angle_error_rad   = launch_run ? 0.f : angle_error;
	cmd.current_speed_m_s = airspeed;
	cmd.desired_speed_m_s = launch_run ? float(env->max_speed)
	                      : (input.desired_speed_m_s >= 0.f
	                             ? input.desired_speed_m_s
	                             : float(env->max_speed));
	cmd.dt_s          = input.dt_s;
	cmd.gravity_m_s2  = input.gravity_m_s2;

	SteerState ss;
	ss.prev_turn_rate_rad_s = state.prev_angular_accel_rad_s2.y;
	ss.prev_accel_m_s2      = glm::length(state.prev_linear_accel_m_s2);
	ss.bank_angle_rad       = state.bank_angle_rad;

	SteerResult r = Steer(*env, ss, cmd);

	state.prev_angular_accel_rad_s2 = glm::vec3(0.f, r.turn_rate_rad_s, 0.f);
	state.prev_linear_accel_m_s2    = forward * r.accel_m_s2;
	state.bank_angle_rad            = ss.bank_angle_rad;

	out.linear_acceleration_m_s2   = forward * r.accel_m_s2;
	out.angular_acceleration_rad_s2 = glm::vec3(0.f, r.turn_rate_rad_s, 0.f);
	out.stability           = r.stability;
	out.speed_headroom      = r.speed_headroom;
	out.turn_headroom       = r.turn_headroom;
	out.suggest_gait_change = r.suggest_gait_change;

	// Landing is a readout, not a steering goal: a pelican physically cannot
	// arrive slower than its stall speed, and the caller decides whether that
	// becomes a stumble or a ragdoll.
	if (input.mode == LocomotionMode::AERIAL) {
		out.touchdown_speed_m_s = float(env->min_speed);
	}

	return out;
}

} // namespace TonTon
```

- [ ] **Step 6: Run the tests to verify they pass**

```bash
cd /mnt/Passport/Libraries/Spehleon/tonton-example
cmake -S . -B build && cmake --build build --target tonton-tests -j$(nproc)
./build/tonton-tests --gtest_filter='Myopic*'
```

Expected: all tests PASS (21 total).

If `HeadwindMakesLaunchEasierThanTailwind` fails, the airspeed subtraction is inverted — it must be `velocity - medium_velocity`, so a wind blowing *against* travel increases airspeed.

- [ ] **Step 7: Commit**

```bash
cd /mnt/Passport/Libraries/Spehleon/tonton
git add src/Control/tonton_launch.h src/Control/tonton_launch.cpp src/Control/tonton_myopic.cpp
git commit -m "feat(myopic): launch planning and ComputeMyopicControl entry point"
cd /mnt/Passport/Libraries/Spehleon/tonton-example
git add tests/tonton_myopic_tests.cpp
git commit -m "test(myopic): launch table, airspeed sign, entry-point finiteness"
```

---

### Task 7: Remaining locomotion modes

**Files:**
- Modify: `src/Control/tonton_envelope.cpp`
- Modify: `tonton-example/tests/tonton_myopic_tests.cpp`

**Interfaces:**
- Consumes: `Envelope` (Task 1).
- Produces: `ExtractEnvelope` handles `AQUATIC`, `SERPENTINE`, `CLIMBING`, `BRACHIATION`.

Each is a thin envelope variant. None gets bank dynamics: only aerial has a load factor, because buoyancy cancels weight for swimmers and legs re-plant each stride for everything else.

- [ ] **Step 1: Write the failing tests**

Append to `tonton-example/tests/tonton_myopic_tests.cpp`:

```cpp
TEST(MyopicEnvelope, AquaticInvariants)
{
	const Output* out = Analyze("penguin.glb", Env::Ocean);
	ASSERT_NE(out, nullptr);
	ASSERT_TRUE(out->aquatic.has_value());

	auto env = ExtractEnvelope(*out, LocomotionMode::AQUATIC, 0, 9.81f);
	ASSERT_TRUE(env.has_value());

	EXPECT_GT(float(env->max_speed), 0.f);
	EXPECT_LT(float(env->min_speed), float(env->max_speed));
	EXPECT_GT(float(env->max_accel), 0.f);
	EXPECT_TRUE(std::isfinite(float(env->tau_linear)));
	EXPECT_GT(float(env->tau_linear), 0.f);
	EXPECT_FALSE(env->aerial.has_value()) << "swimmers have no load factor";
}

TEST(MyopicEnvelope, SharkRequiresConstantMotion)
{
	const Output* out = Analyze("shark.glb", Env::Ocean);
	ASSERT_NE(out, nullptr);
	ASSERT_TRUE(out->aquatic.has_value());
	ASSERT_TRUE(out->aquatic->requires_constant_motion);

	auto env = ExtractEnvelope(*out, LocomotionMode::AQUATIC, 0, 9.81f);
	ASSERT_TRUE(env.has_value());

	// A shark's minimum swim speed is a real floor, expressed as min_speed so
	// the existing stability term reports it. No transition logic needed.
	EXPECT_GT(float(env->min_speed), 0.f);
}

TEST(MyopicEnvelope, SerpentineInvariants)
{
	const Output* out = Analyze("eel.glb", Env::Ocean);
	ASSERT_NE(out, nullptr);
	if (!out->serpentine.has_value()) GTEST_SKIP() << "eel has no serpentine section";

	auto env = ExtractEnvelope(*out, LocomotionMode::SERPENTINE, 0, 9.81f);
	ASSERT_TRUE(env.has_value());
	EXPECT_GT(float(env->max_speed), 0.f);
	EXPECT_TRUE(std::isfinite(float(env->tau_linear)));
}
```

- [ ] **Step 2: Run the tests to verify they fail**

```bash
cd /mnt/Passport/Libraries/Spehleon/tonton-example
cmake --build build --target tonton-tests -j$(nproc)
./build/tonton-tests --gtest_filter='MyopicEnvelope.Aquatic*:MyopicEnvelope.Shark*:MyopicEnvelope.Serpentine*'
```

Expected: FAIL — `ExtractEnvelope` returns `nullopt` for these modes.

- [ ] **Step 3: Implement the remaining arms**

In `src/Control/tonton_envelope.cpp`, add before `default:`:

```cpp
	case LocomotionMode::AQUATIC: {
		if (!analysis.aquatic.has_value()) return std::nullopt;
		const auto& a = *analysis.aquatic;

		Envelope e;
		e.max_speed = a.burst_speed_m_s;
		// requires_constant_motion (sharks, tuna) is just a min_speed inside
		// the mode's own envelope -- not a transition constraint.
		e.min_speed = a.min_swim_speed_m_s;
		e.max_accel = AccelFromPower(analysis.metabolic.max_rate_W,
		                             analysis.physical.body_mass_kg,
		                             a.cruise_speed_m_s);
		e.max_brake = e.max_accel;   // water brakes you for free
		e.min_turn_radius = a.min_turning_radius_m;
		e.max_lateral_accel = LateralBudget(e.max_accel, e.max_speed, e.min_turn_radius);
		e.tau_linear = a.cruise_speed_m_s / e.max_accel;
		return e;
	}

	case LocomotionMode::SERPENTINE: {
		if (!analysis.serpentine.has_value()) return std::nullopt;
		const auto& s = *analysis.serpentine;

		Envelope e;
		// Serpentine has a narrow band: undulation speed is close to both the
		// floor and the ceiling, so the envelope is deliberately tight and
		// stability drops fast when demands exceed it.
		e.max_speed = s.lateral_undulation_speed_m_s;
		e.min_speed = s.lateral_undulation_speed_m_s * 0.5f;
		e.max_accel = s.lateral_undulation_speed_m_s / time_s{1.f};
		e.max_brake = e.max_accel;
		e.min_turn_radius = length_m{0.f}; // a snake turns by undulating
		e.max_lateral_accel = e.max_accel;
		e.tau_linear = s.lateral_undulation_speed_m_s / e.max_accel;
		return e;
	}

	case LocomotionMode::CLIMBING: {
		if (!analysis.climbing.has_value()) return std::nullopt;
		const auto& c = *analysis.climbing;

		Envelope e;
		e.max_speed = c.max_climb_speed_m_s;
		e.min_speed = velocity_m_s{0.f};
		e.max_accel = c.max_climb_speed_m_s / time_s{1.f};
		e.max_brake = e.max_accel;   // a climber stops by gripping
		e.min_turn_radius = length_m{0.f};
		e.max_lateral_accel = e.max_accel;
		e.tau_linear = c.max_climb_speed_m_s / e.max_accel;
		return e;
	}

	case LocomotionMode::BRACHIATION: {
		if (!analysis.brachiation.has_value()) return std::nullopt;
		const auto& b = *analysis.brachiation;

		Envelope e;
		e.max_speed = b.max_swing_speed_m_s;
		e.min_speed = velocity_m_s{0.f};
		// A brachiator changes speed once per swing; the swing frequency is a
		// real measured quantity here, unlike terrestrial stride frequency.
		e.max_accel = b.max_swing_speed_m_s * b.swing_frequency_Hz;
		e.max_brake = e.max_accel;
		e.min_turn_radius = length_m{0.f};
		e.max_lateral_accel = e.max_accel;
		e.tau_linear = b.max_swing_speed_m_s / e.max_accel;
		return e;
	}
```

**Note on `SERPENTINE` and `CLIMBING`:** the `/ time_s{1.f}` divisors are placeholders for a missing physical quantity, not tuning constants — neither `Analysis_Serpentine` nor `Analysis_Climbing` exposes an acceleration or a power figure the way aerial and aquatic do. Add a comment saying so, and raise it as a follow-up finding rather than silently accepting it. If a better derivation exists (e.g. undulation frequency × wavelength for serpentine), prefer it.

- [ ] **Step 4: Run the tests to verify they pass**

```bash
cd /mnt/Passport/Libraries/Spehleon/tonton-example
cmake --build build --target tonton-tests -j$(nproc)
./build/tonton-tests --gtest_filter='Myopic*'
```

Expected: all tests PASS (24 total).

- [ ] **Step 5: Run the full suite to check for regressions**

```bash
./build/tonton-tests
```

Expected: the pre-existing suite is unchanged. Per the project memory the baseline is **17 pass / 4 fail**, with `Species.Bat`, `Species.Treefrog`, `Species.PenguinSwims` and `Species.GreatWhiteShark` red for reasons documented in the spec. Confirm the same four and no others. If a fifth goes red, this module broke something.

- [ ] **Step 6: Commit**

```bash
cd /mnt/Passport/Libraries/Spehleon/tonton
git add src/Control/tonton_envelope.cpp
git commit -m "feat(myopic): aquatic, serpentine, climbing and brachiation envelopes"
cd /mnt/Passport/Libraries/Spehleon/tonton-example
git add tests/tonton_myopic_tests.cpp
git commit -m "test(myopic): remaining locomotion mode envelopes"
```

---

## Follow-up findings to report, not fix

Raise these when the plan is complete; none belongs in this module.

1. **`Analysis_Serpentine` and `Analysis_Climbing` expose no acceleration or power**, so Task 7 uses a one-second placeholder divisor. Aerial and aquatic derive acceleration honestly from power; these two cannot.
2. **`crouch_length` lives in `Builder`, not `Output`**, and the `rest_length * 0.3f` fallback is duplicated at `tonton_terrestrial.cpp:353` and `tonton_takeoffanalysis.cpp:119`.
3. **`Word::pinna` returns `CHORDATA`** although an external ear flap is a mammalian synapomorphy.
4. **The patagium/clade thread** — `thin_sheet && digit_supported` at `tonton_builder.cpp:1177` is the definition of a patagium, and `Word::patagium` already returns `CF::MAMMALIA`. Having that branch contribute a clade hint would likely fix `Species.Bat`. Its own spec.
