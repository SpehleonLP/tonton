# Myopic Locomotion Controller — Design

**Date:** 2026-07-27
**Status:** Design approved, pending implementation plan
**Supersedes:** `notes.md` (2026-02-27) — retained for history; this document is authoritative where they disagree.

## Context

TonTon analyses a rigged skeleton + mesh once and produces `TonTon::Output`: a large
struct with optional locomotion sections (terrestrial, aerial, aquatic, serpentine,
climbing, brachiation, jumping) plus physical, metabolic, behavioural and sensory data.
A single creature routinely has several locomotion modes at once.

The game engine needs a per-frame steering controller that takes a current state and a
target and returns body-level acceleration, torque, and a stability metric. It is
*myopic*: it only considers this frame, it trusts the caller about what mode the
creature is in, and it does not plan ahead. If the caller claims the creature is on the
ground when it is not, that is the caller's problem.

This module is additive. It requires no changes to the analysis library.

## Prior failed approaches, and what this design does about them

Three attempts were made in 2025 and abandoned. Each failure mode gets a structural
answer here, not a disciplinary one.

**1. PD controller — untunable.** Robotics PD gains assume ~1 kHz control loops; papers
never state their timestep because 1 kHz is assumed. At 16–60 Hz game timesteps the
gains did not transfer and the controller oscillated.

*Answer:* the module contains no tunable constants at all (see "No magic constants").
Steering is greedy-and-clamped, which cannot overshoot, and smoothing uses
`α = 1 - exp(-dt/τ)` with τ derived from the creature's own data, which is correct at
any framerate. Enforced by the `Framerate.*` and `NoOscillation.*` tests.

**2. AI-authored bird flight controller — drowned in data.** Given the full TonTon
output, the model tried to use all of it and produced an over-engineered controller.

*Answer:* exactly two functions may read `TonTon::Output` — `ExtractEnvelope` and
`PlanLaunch`. All steering math operates on a six-field `Envelope` and a small
`LaunchPlan`. The steering code cannot drown in data it has no access to.

**3. Control-theory framing.** Calling the problem "myopic control" invited force models,
torque computation and stability margins. The actual problem is simpler.

*Answer:* this is constrained steering behaviours (Reynolds, 1999). The controller does
not simulate forces. It computes a desired heading change and speed change, clamps them
to what TonTon says is possible, and reports how hard it is working relative to its
limits.

(`notes.md` attributes attempt 2 to Sonnet 4.5; it was 3.8. Corrected here.)

## Core insight: transitions

`MyopicInput` carries both a current and a target locomotion mode. When they differ, the
controller steers toward the *transition's precondition* rather than toward the target
position, and reports readiness.

**A transition only needs physics when the creature must generate the precondition
itself.** Airspeed over the airfoil is the only precondition an animal must manufacture
with its own body before a mode becomes available. Every other transition's precondition
is a fact about the world that the caller owns and TonTon cannot know:

| Transition | Precondition | Owner |
|---|---|---|
| ground → aerial | airspeed / lift-to-weight | **TonTon** |
| water → aerial | airspeed (water taxi) | **TonTon** |
| water → ground | "is there a shore here" | caller |
| ground → brachiation | "is a branch in IK reach" | caller |
| ground → climbing | "is this surface climbable" | caller |
| ground → serpentine | nothing to generate | caller |

So target-mode steering reduces to **`* → AERIAL`**, the launch problem — which is also
the one TonTon already has an entire struct about. All other target modes leave steering
unchanged.

Two near-misses that look like transition constraints but are not: shark
`requires_constant_motion` and the serpentine undulation floor are both just `min_speed`
inside the mode's own envelope. `CLIFF_LAUNCH` is a launch whose precondition is a world
fact ("be on a cliff"), so it degrades to a readout.

**Landing is a readout, not a steering goal.** Steering in `AERIAL` toward a target near
the ground already flies the creature there, and the stability metric already reports
stall. The controller adds only `touchdown_speed_m_s`, so the caller knows whether this
is a hummingbird settling or an 11 m/s pelican arrival it should convert into a stumble
or a ragdoll.

## Architecture

Three units, one entry point.

```cpp
MyopicOutput ComputeMyopicControl(
    const Output& analysis,
    const MyopicInput& input,
    MyopicState& state);
```

### `ExtractEnvelope(analysis, mode, gait) -> Envelope`

The data reduction. The only code that reads `Output` for steering purposes.

```cpp
struct Envelope {
    velocity_m_s     max_speed;
    velocity_m_s     min_speed;        // 0 for ground, stall speed for air
    acceleration_m_s2 max_accel;
    acceleration_m_s2 max_brake;       // may differ from max_accel
    omega_rad_s      max_turn_rate;    // best available axis
    length_m         min_turn_radius;  // 0 = can pivot in place
    time_s           tau_linear_s;     // slew time constant
};
```

For terrestrial the envelope depends on `current_gait`: 0 = `optimal_speed_m_s`,
1 = `max_sustainable_speed_m_s`, 2 = `max_sprint_speed_m_s`, with turn rate tightening
as gait rises.

**Known gap: only `Analysis_Terrestrial` exposes an acceleration.** `Analysis_Aerial`
has none, and `Analysis_Aquatic` has acceleration only inside `c_start`. For those modes
`ExtractEnvelope` derives it from power, `a = P_available/(m·V)`, with braking from drag.
This is defensible and built from fields that exist, but it is concentrated work in one
function and is the second-likeliest place for a units bug.

### `PlanLaunch(analysis, input) -> LaunchPlan`

The only code that reads `Analysis_TakeoffAnalysis`. Runs only when
`target_mode == AERIAL && mode != AERIAL`. A switch on `takeoff.mode`; each arm is short
because the analysis already did the thinking.

| `TakeoffMode` | readiness | steering goal |
|---|---|---|
| `VERTICAL_LAUNCH` | 1.0 immediately | none — go whenever |
| `JUMP_LAUNCH` | 1.0 when standing | none — see below |
| `RUNNING_TAKEOFF` | `airspeed / required_airspeed` | accelerate along heading |
| `CLIFF_LAUNCH` | 0 unless `substrate == CLIFF_EDGE` | none; report `required_drop_m` |
| `ASSISTED_LAUNCH` | 0 unless `substrate == PERCH` | none |
| `IMPOSSIBLE` | `feasible = false` | none |

`can_use_water_taxi` promotes `substrate == WATER` to a valid runway for
`RUNNING_TAKEOFF`. Headwind shortens the runway with no special case, because readiness
is measured in airspeed and the wind is already in it.

`blocking_reason` derives almost verbatim from `takeoff.constraints`, so an `IMPOSSIBLE`
creature can report *which* of wing loading / power loading / aspect ratio / leg strength
failed.

**Jumping is descriptive only.** A jump has a crouch pre-animation, so there is no frame
on which "apply this impulse now" is correct, and a per-frame function has no business
owning an animation timeline. `LaunchPlan` reports the requirement and the caller owns
all timing:

```cpp
velocity_m_s required_jump_velocity_m_s;  // Analysis_TakeoffAnalysis
glm::vec3    jump_direction;              // unit, world space
bool         feasible;                    // required <= jumping.takeoff_velocity_m_s
```

That feasibility check is a genuine cross-struct question TonTon can answer and the
caller cannot: `Analysis_TakeoffAnalysis` says what the launch needs, `Analysis_Jumping`
says what the legs have. `MyopicOutput` stays purely acceleration-valued — no impulse
channel exists.

### `Steer(envelope, launch_plan, input, state) -> MyopicOutput`

Pure math. Never sees `Output`. Contains the bank/yaw decision and stability.

### State

```cpp
struct MyopicState {
    glm::vec3 prev_linear_accel{0};
    glm::vec3 prev_angular_accel{0};
    float     bank_angle_rad{0};
};
```

Caller-owned, one per creature. Zero-initialised is a valid cold start. Bank angle is
state because roll-in is a process, not an instant. `notes.md` assumed a pure function;
the slew makes that untrue.

## Interface

```cpp
enum class LocomotionMode : uint8_t {
    TERRESTRIAL, AERIAL, AQUATIC, SERPENTINE, CLIMBING, BRACHIATION,
    // JUMPING is a one-shot event, not a sustained mode
};

enum class Substrate : uint8_t { GROUND, WATER, PERCH, CLIFF_EDGE };

enum class BlockingReason : uint8_t {
    NONE, NEEDS_RUNWAY_SPEED, NEEDS_ELEVATION, NEEDS_PERCH,
    WING_LOADING, POWER_LOADING, ASPECT_RATIO, LEG_STRENGTH, NO_AERIAL_ANALYSIS,
};

struct MyopicInput {
    LocomotionMode mode;              // what the caller says we are in; we trust them
    LocomotionMode target_mode;       // only changes steering when == AERIAL && != mode

    glm::vec3 position;
    glm::quat orientation;
    glm::vec3 velocity;               // world space, relative to ground
    glm::vec3 angular_velocity;       // body space

    glm::vec3 target_position;
    float     desired_speed = -1.f;   // < 0 -> controller picks cruise

    glm::vec3 medium_velocity{0};     // wind or current, world space
    float     height_above_ground_m{0};
    Substrate substrate{Substrate::GROUND};

    int   current_gait = 0;           // terrestrial: 0 walk, 1 trot, 2 gallop
    float dt;
};

struct MyopicOutput {
    glm::vec3 linear_acceleration;    // world space
    glm::vec3 angular_acceleration;   // body space

    float stability;                  // 1 comfortable, 0 at limits, <0 exceeding
    float speed_headroom;
    float turn_headroom;
    bool  suggest_gait_change;

    float          transition_readiness;  // [0,1]; only meaningful when target != mode
    BlockingReason blocking_reason;
    velocity_m_s   touchdown_speed_m_s;   // aerial only; what a landing would cost
};
```

### Units

The public structs above use bare `float` / `glm::vec3` with units in the member names,
per `notes.md`, so the engine can fill them without depending on the quantity system.
Dimensionless outputs (`stability`, headrooms, `transition_readiness`) are plain floats
by nature. The two exceptions are `touchdown_speed_m_s` and the `LaunchPlan` velocities,
which are `velocity_m_s` because they are passed straight through from `Output` and
retyping them would discard checking for no benefit.

All *internal* computation uses TonTon's compile-time `Quantity<M,L,T,Temp,Stage>` types.
Conversion happens exactly once, at the boundary of `ComputeMyopicControl`. Given that
the two most likely bugs in this module are both unit errors (airspeed inversion, derived
acceleration), the internals are where the checking earns its keep.

## Algorithm

**Airspeed discipline.** `airspeed = velocity - medium_velocity` is computed once at the
top, and *every* speed comparison downstream uses it. Ground speed is used only for the
runway-distance readout, because runway is measured against the ground. Inverting this is
the single likeliest bug in the module and gets a dedicated test.

**Turn.** Greedy demand `ω_demand = angle_error / dt`, clamped to the envelope. For
aerial, the yaw-versus-bank choice is made in *time*, not rate:

```
t_yaw  = angle_error / max_yaw_rate_rad_s               // available immediately
t_bank = φ_needed / max_roll_rate_rad_s                 // roll-in cost
       + angle_error / (g·tan(φ_needed)/V)              // then the turn itself
```

Take the cheaper. This replaces the invented `0.5 s` horizon in `notes.md`, which was a
PD gain in disguise. Banking is rate-limited by roll, not by turn rate: achievable turn
rate this frame is `g·tan(φ_now)/V`, where `φ_now` slews toward `φ_demand` at
`max_roll_rate_rad_s`.

The bank cap comes from two independent derivations of the load factor, cross-checked:

```
n_max = min( max_instantaneous_lift_N / (m·g),              // force budget
             1 / cos(atan(V_cruise² / (g·min_turning_radius_m))) )   // observed radius
φ_max = acos(1/n_max)
```

A large disagreement between the two is a diagnostic about the analysis and is logged
rather than silently averaged.

Non-aerial modes skip bank dynamics entirely: `min_turning_radius_m` is the whole answer,
because legs and fins re-plant each cycle instead of rolling in. Aquatic additionally has
no meaningful load factor, since buoyancy cancels weight.

**Slew.** `α = 1 - exp(-dt/τ)`, `command = lerp(prev, demand, α)`.
`τ_linear = characteristic_speed / max_accel` for every mode — dimensionally a time,
built from two numbers TonTon already computes and the plausibility suite already
validates, adding no new biological claim. `τ_angular` is the roll-in time for aerial and
`τ_linear` elsewhere. Lerping toward a clamped target cannot overshoot.

*Rejected alternative:* leg-as-pendulum, `f = (1/2π)√(g/L)`. Not unsound — it is a
mechanical fact about a pendulum, not a Froude claim, and the dead claim in
`FROUDE_NUMBERS.md` is specifically that gait-change Fr is constant across taxa (it
ranges 0.18 for emu to >3.4 for elephant). But the envelope-derived τ introduces no new
estimator to defend, so it wins on those grounds. Noted here so it is not re-litigated.

**Stability.** Three pure ratios; the worst one wins.

```
u_turn  = |ω_demand| / ω_max_available
u_speed = airspeed / max_speed
u_stall = V_stall_effective / airspeed
          // aerial:  min_flight_speed_m_s / √(cos φ)
          // shark / serpentine: min_speed / airspeed
          // otherwise: 0

stability      = 1 - max(u_turn, u_speed, u_stall)
speed_headroom = 1 - u_speed
turn_headroom  = 1 - u_turn
```

1 when idle, 0 at any limit, negative past it. The stall term is where banking pays off:
load factor `n = 1/cos φ` raises stall speed as `√n`, so a bird at 60° bank stalls 1.41×
faster and **turning hard near minimum speed drives stability negative on its own**. That
is how real flyers fall out of the sky, and it is emergent rather than scripted.

`suggest_gait_change` is set when demanded speed exceeds the current gait's max. The
speed is deliberately *not* clamped to the current gait — the stability metric
communicates the problem and the external gait selector decides.

## No magic constants

The module contains no gains, no time horizons, no responsiveness knobs, and no fudge
factors. Every number is either read from `Output` or is `dt`. This is the specific thing
that failed in 2025, so it is a structural property to be defended in review, not a style
preference.

## Testing

In `tonton-example` alongside the existing `tonton-tests` target. Two tests carry the
weight:

- **`Framerate.*`** — identical scripted maneuvers at 16 / 30 / 60 / 120 Hz must produce
  converging trajectories. Direct regression test for failure 1; goes red if
  `exp(-dt/τ)` is ever replaced by a naive lerp.
- **`NoOscillation.*`** — step heading input; error must be monotone with no sign flips.
  A PD controller cannot pass this; clamped-greedy-plus-slew cannot fail it.

Supporting tests:

- **`Envelope.*`** — every sample model × every present mode: `min_speed < max_speed`,
  τ finite and positive, derived acceleration non-zero for aerial and aquatic.
- **`Airspeed.*`** — head/tailwind must order runway requirement correctly (catches the
  sign inversion).
- **`Launch.*`** — walk the takeoff table: dragonfly ready at standstill; running-takeoff
  readiness monotone in airspeed; `IMPOSSIBLE` reports a specific `blocking_reason`.
- **`Bank.*`** — dragonfly yaws, a low-yaw/high-roll creature banks, for the same input.
- **`Stall.*`** — hard turn near `min_flight_speed` drives stability negative.

**Sample-model caveats.** The Treefrog and Bat samples have no aerial section (see "Known
limitation" below), so flyer tests key off the **dragonfly**. No sample has albatross-like
low-yaw/high-roll authority, so `Bank.*` uses a synthetic hand-built `Output` — which is
better anyway, since it tests the decision boundary directly rather than incidentally.

## Out of scope

Mode selection; gait selection; foot placement (engine CPG + arc interpolation + IK);
limb control; trajectory planning; terrain awareness; obstacle avoidance (caller folds
avoidance into `target_position` before calling); transition sequencing beyond the launch
readiness readout.

## Known limitation, not addressed here

The **Bat** sample produces no aerial analysis because its wing bones are named `hand` —
which is anatomically correct, since a bat wing *is* a hand. `GetSemanticFlags(Word::hand)`
yields grasper/forelimb rather than `WING`, so no wing is ever found. Treefrog has a
related clade misclassification.

This is a wordlist/robustness problem in the analysis layer, orthogonal to this module,
and is explicitly **not** in scope. It is recorded because it constrains the test plan
above, and because the likely direction is worth capturing: semantic flags from bone
names are *priors*, and TonTon already computes the geometry (silhouette area, tubiness,
cross-sections) that could promote or override them — a flat, membranous forelimb with
silhouette area wildly out of proportion to a grasping hand is a wing regardless of its
name. That would be the "let physics determine what's possible" principle applied to
classification rather than to locomotion. Separate spec.

## Implementation order

1. `Envelope` + `ExtractEnvelope` for terrestrial, with `Envelope.*` tests.
2. `Steer` with greedy + slew, terrestrial only. `Framerate.*` and `NoOscillation.*` land
   here — the riskiest part is proven first.
3. Stability metric and hints.
4. Aerial envelope (including derived acceleration), bank/yaw decision, stall coupling.
5. `PlanLaunch` and the takeoff table.
6. Aquatic, then serpentine, then climbing and brachiation, as thin envelope variants.
