# TonTon

TonTon is a procedural creature analysis library that generates physically-grounded locomotion parameters, behavioral profiles, and sensory capabilities from skeletal armatures and 3D meshes. Given a rigged character, TonTon analyzes its anatomy and computes realistic movement capabilities, metabolic requirements, and AI behavior hints suitable for games, simulations, and procedural animation systems.

**The core idea:** Instead of hand-tuning hundreds of parameters for each creature, provide an anatomically-structured skeleton and let physics and biology determine what's possible.

Inspired by the principled BSDF the input configuration is all normalized ranges from 0 to 1, constrained by what is possible for real animals. 

## Overview

TonTon answers questions like:
- Can this creature fly? If so, what's its wingbeat frequency and stall speed?
- How fast can it run? What gaits should it use?
- Is it a predator or prey? Diurnal or nocturnal?
- What's its basal metabolic rate? How long can it sprint?
- How maneuverable is it in the air/water/land?

The system parses bone names semantically (e.g., "left_wing", "right_hind_leg", "tail_tip"), measures geometric properties from the 3D mesh, and applies biomechanical models to compute physically-realistic capabilities.

## Example Output:

    DRAGONFLY.glb

    body_mass_kg: 0.00194157
    body_length_m: 0.0241147
    tail_length_m: 0.0362214
    wingbeat_frequency_Hz: 79.7041
    speeds (min/cruise/max): 3.20421 / 9.09497 / 13.6425 m/s
    cost (flap/glide/soar/hover): 8.25367 / 1.48141 / 1.03699 / 11.0826 W/N

and these are actually plausible results for a dragonfly!

but what about a dragonfly with a 50cm wingspan?

    body_mass_kg: 1.94157
    wingbeat_frequency_Hz: 59.7907
    speeds (min/cruise/max): 10.1374 / 68.2267 / 102.34 m/s
    cost (flap/glide/soar/hover): 20317.6 / 11.1025 / 7.77174 / 24374.7 W/N

or a 5m wingspan on Saturn's moon titan ?

    wingbeat_frequency_Hz: 10.5567
    speeds (min/cruise/max): 1.77609 / 12.0462 / 18.0692 m/s
    cost (flap/glide/soar/hover): 821.821 / 1.96026 / 1.37218 / 985.058 W/N

## Features

### Locomotion Analysis
- **Aerial**: Wingbeat frequency, flight speeds, hovering capability, takeoff analysis, maneuverability (supports birds, bats, insects, dragonflies)
- **Terrestrial**: Gait patterns, sprint/sustainable speeds, posture types (erect, sprawling, semi-erect)
- **Aquatic**: Swimming modes (BCF, MPF, jet propulsion), buoyancy, C-start escape responses
- **Serpentine**: Lateral undulation, sidewinding, rectilinear, concertina modes
- **Specialized**: Climbing, brachiation, jumping, digging, constriction

### Physical Properties
- Body mass, volume, surface area, cross-sections
- Center of mass, moment of inertia (per joint and whole-body)
- Limb metrics (volume, mass, inertia per appendage)

### Metabolic & Behavioral
- Basal and maximum metabolic rates (Kleiber's Law for endotherms, temperature-scaled for ectotherms)
- Muscle mass and available power
- Behavioral archetype inference (ambush hunter, pack coordinator, schooling prey, etc.)
- Activity patterns, aggression, social tendency, territoriality
- Thermoregulation strategies

### Sensory Systems
- **Vision**: Acuity, binocular overlap, color/night vision, detection range, asymmetry detection (flatfish!)
- **Hearing**: Sensitivity, frequency range (allometric scaling), detection range
- **Olfaction**: Sensitivity (vertebrate noses + arthropod antennae), detection range

## Architecture

TonTon requires two types of geometric data as input:

### 3D Volumetric Data (Pre-computed)
TonTon expects you to provide:
- Volumes per joint/segment
- 3D centroids (center of mass)
- Covariance matrices → inertia tensors

**Note on licensing**: TonTon does **not** depend on any specific volumetric analysis library. You must compute these values separately and pass them in. The [rintintin](https://github.com/SpehleonLP/rintintin) library (GPL-2.0) is currently the only known implementation capable of computing these from skinned meshes, but TonTon itself is Apache-licensed and has no GPL dependencies.

### 2D Projection/Silhouette Analysis (Integrated)
TonTon includes **dodeedum** as a submodule for:
- Projected areas (silhouette size from any viewpoint)
- Second moment of area (2D mass distribution)
- Chord length, span measurements
- Weighted area calculations (accounting for skeletal influence)

**TonTon** combines both data sources:
- Uses pre-computed volumetric data for body mass, limb volumes, inertia matrices
- Uses **dodeedum** internally for wing area, cross-sections (drag), projected measurements
- Applies biomechanical models and allometric scaling laws
- Infers behavior from morphology

## How It Works

### 1. Semantic Parsing
TonTon analyzes bone names using a dictionary of anatomical terms:
```cpp
// Recognizes words like: wing, fin, tail, jaw, leg, eye, antenna, etc.
// Flags: LIMB, WING, ANTERIOR, POSTERIOR, LEFT, RIGHT, VISION, etc.
```

From "left_front_wing_tip", it extracts:
- Laterality: LEFT
- Position: ANTERIOR (front)
- Type: WING
- Extremity: DISTAL (tip)

### 2. Geometric Measurement
```cpp
// Pre-computed 3D properties (from external volumetric analysis)
// You provide these when creating the SkinnedMesh:
//   - volumes per joint
//   - centroids per joint
//   - covariance matrices per joint

// TonTon uses dodeedum internally for 2D projections:
auto projection = GetProjectionMatrix(viewpoint);
auto silhouette = GetSilhouettes(projection, joints);
auto wing_area = silhouette.area;
auto chord_length = silhouette.MeasureWidth(...);
```

### 3. Physics-Based Analysis
Applies established biomechanical models:
- **Wingbeat frequency**: Pennycuick (1996) - `f ∝ sqrt(g/L) × (wing_loading)^0.375`
- **Metabolic rate**: Kleiber's Law - `BMR ∝ M^0.75`
- **Strouhal number**: Optimal oscillatory locomotion (0.2-0.4)
- **Reynolds number**: Determines aerodynamic regime, LEV contributions
- **Allometric scaling**: Body size relationships for sensory organs

### 4. Behavioral Inference
Morphology reveals behavior:
- **Lateral eyes** → Prey species, schooling behavior
- **Forward eyes** + **fangs/claws** → Predator, binocular vision for depth perception
- **Large eye-to-body ratio** → Nocturnal
- **Horns/antlers** → Territorial, intraspecies combat
- **High aerobic scope** → Sustained activity, pursuit hunting
- **Stocky body** → Ambush predator

### 5. Output
Returns comprehensive `Output` structure with:
- Animation parameters (IK chains, gait cycles, phase offsets)
- Physics values (forces, speeds, turning radii, power requirements)
- AI hints (archetype, aggression, social tendency, prey preferences)
- Diagnostics (confidence scores, physics validation, warnings)

## Example Usage

```cpp
#include <tonton.h>

// Create armature with semantic names
auto armature = TonTon::Armature::Factory(
    names,      // ["spine_root", "left_wing_root", "left_wing_tip", ...]
    parents,    // Parent indices for hierarchy
    positions,  // Rest pose joint positions
    rotations   // Rest pose joint rotations
);

// Create skinned mesh with pre-computed volumetric data
// (e.g., from rintintin or your own volumetric analysis)
auto skinned_mesh = TonTon::SkinnedMesh::Factory(
    mesh,           // DoDeeDum mesh with positions, joints, weights
    armature,
    aabb,           // Bounding boxes per joint
    surfaceArea,    // From volumetric analysis
    volume,         // From volumetric analysis
    centroid,       // From volumetric analysis
    covariance      // From volumetric analysis (6 values: Ixx, Iyy, Izz, Ixy, Ixz, Iyz)
);

// Set up analysis parameters
TonTon::Input input;
input.armature = skinned_mesh;
input.environment.fluidDensity_Kg_m3 = 1.225f; // Air
input.environment.gravity_m_s2 = 9.81f;
input.behavior.scale = glm::vec3(1.0f);
input.feather_quality = 0.8f;        // 0=poor, 1=optimal
input.muscle_quality = 0.7f;
input.metabolic_efficiency = 0.6f;
input.stability_vs_speed = 0.3f;     // 0=forward flight, 1=hovering

// Analyze creature
auto output = TonTon::Output::Factory(input);

// Use the results
if (output->aerial) {
    std::cout << "Wingbeat frequency: "
              << output->aerial->wingbeat_frequency_Hz << " Hz\n";
    std::cout << "Cruise speed: "
              << output->aerial->cruise_speed_m_s << " m/s\n";
    std::cout << "Can hover: "
              << (output->aerial->hovering_efficiency > 0.5f ? "Yes" : "No") << "\n";
}

if (output->terrestrial) {
    std::cout << "Max sprint: "
              << output->terrestrial->max_sprint_speed_m_s << " m/s\n";
    std::cout << "Posture: ";
    switch(output->terrestrial->posture) {
        case TonTon::Output_Terrestrial::PostureType::ERECT:
            std::cout << "Erect\n"; break;
        case TonTon::Output_Terrestrial::PostureType::SPRAWLING:
            std::cout << "Sprawling\n"; break;
        // ...
    }
}

std::cout << "AI Archetype: ";
switch(output->behavior.suggested_archetype) {
    case TonTon::Output_Behavior::AIArchetype::AERIAL_PREDATOR:
        std::cout << "Aerial Predator\n"; break;
    case TonTon::Output_Behavior::AIArchetype::PACK_COORDINATOR:
        std::cout << "Pack Coordinator\n"; break;
    // ...
}
```

## Implementation Details

### Structural Analysis (Hand-Coded)
The geometric measurement pipeline is entirely hand-coded:
- Semantic parsing and bone name recognition
- Input data handling (volumes, covariance matrices from external sources)
- dodeedum integration (projections, silhouettes, areas)
- Data structure management and memoization

### Rules Engine (AI-Assisted + Research-Based)
The biomechanical rules in `src/Rules/` were developed using **Claude Sonnet 4.5** vai deep research queries. This approach was necessary because:

1. **Domain Expertise**: The rules encode specialized knowledge from biomechanics, zoology, comparative physiology, and animal behavior - domains outside my expertise
2. **Literature Synthesis**: Each rule is based on peer-reviewed research (see [bibliography.md](bibliography.md))
3. **Rapid Prototyping**: AI-assisted development allowed quick exploration of complex biological relationships

**This is intentional and transparent:** The rules represent *best-effort synthesis of published research*, not authoritative biological knowledge.

### **Contributions Wanted!**

If you have expertise in:
- Biomechanics
- Comparative physiology
- Animal locomotion
- Aerodynamics/hydrodynamics
- Zoology/ethology

**Please review the rules and contribute improvements!** The [bibliography.md](bibliography.md) file contains citations for the models used. If you:
- Spot errors in the physics/biology
- Know of better models or more recent research
- Have empirical data that contradicts the current rules
- Want to add new locomotion modes or behaviors

**Pull requests are extremely welcome.** The goal is to make TonTon increasingly accurate by incorporating actual domain knowledge.

Alternatively, if you have the attention span to read through the papers in the bibliography and verify/correct the implementations, that would be invaluable.

## Building

TonTon is a C++20 library. See [CMakeLists.txt](CMakeLists.txt) for build configuration.

### Dependencies
- **GLM**: Vector/matrix math
- **C++17**: Standard library features
- **dodeedum**: 2D silhouette/projection analysis (submodule)

**Note**: You'll need a way to compute volumetric data (volumes, centroids, covariance matrices) for your meshes. The [rintintin](https://github.com/SpehleonLP/rintintin) library (GPL-2.0) can do this, but due to licensing incompatibility, it's not included as a dependency. You must compute these values separately and pass them to TonTon.

### Build with CMake
```bash
mkdir build && cd build
cmake ..
make
```

## References

See [bibliography.md](bibliography.md) for full citations of the research papers and models used in the rules engine.

Key sources include:
- Pennycuick (1996) - Wingbeat frequency scaling
- Kleiber (1932) - Metabolic scaling laws
- Ellington et al. (1990) - Insect flight mechanics
- Webb (1984) - Fish escape responses
- Bennett & Dawson (1976) - Ectotherm metabolism
- Many others (50+ papers cited)

## License

Apache License 2.0 - Copyright (c) 2025 Spehleon LP

See [LICENSE](LICENSE) for full details.

## Why "TonTon"?

Why not?
