# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

TonTon is a procedural creature analysis library that generates physically-grounded locomotion parameters, behavioral profiles, and sensory capabilities from skeletal armatures and 3D meshes. It applies biomechanical models and allometric scaling laws to compute realistic movement capabilities, metabolic requirements, and AI behavior hints for games and simulations.

**Key principle:** Instead of hand-tuning hundreds of parameters, provide an anatomically-structured skeleton and let physics and biology determine what's possible.

## Build System

```bash
# Build the project
mkdir build && cd build
cmake ..
make
```

**Build Configuration:**
- C++20 required
- Dependencies: GLM (vector/matrix math), dodeedum submodule (2D projections)
- CMake 3.15+
- Static library output

**Important:** TonTon does NOT include volumetric analysis capabilities. Users must pre-compute volumes, centroids, and covariance matrices externally (e.g., via the GPL-licensed rintintin library) and pass them to TonTon.

## Architecture

### Two-Tier Data Processing

**1. Input Layer** (`tonton_input.h/cpp`):
- **Armature**: Skeletal hierarchy with semantic bone names
  - Parsed via `tonton_wordlist.h` - recognizes anatomical terms (wing, leg, tail, etc.)
  - Extracts laterality (LEFT/RIGHT), position (ANTERIOR/POSTERIOR), body part type
- **Mesh**: DoDeeDum mesh wrapper for 2D projections
- **SkinnedMesh**: Combines armature + mesh + pre-computed volumetric data
  - Requires external computation of: volumes, centroids, covariance matrices per joint
  - Uses dodeedum internally for silhouettes, projected areas, chord lengths

**2. Analysis Layer** (`src/Rules/`):
- Physics-based rule implementations for different locomotion modes
- Each file corresponds to a locomotion/behavior category:
  - `tonton_aerial.cpp`: Flight mechanics (wingbeat, speeds, hovering)
  - `tonton_terrestrial.cpp`: Walking/running gaits, posture types
  - `tonton_aquatic.cpp`: Swimming modes, buoyancy, C-starts
  - `tonton_serpentine.cpp`: Undulation, sidewinding, concertina
  - `tonton_climbing.cpp`: Arboreal locomotion
  - `tonton_sensory.cpp`: Vision, hearing, olfaction analysis
  - `tonton_behavior.cpp`: AI archetype inference from morphology
  - `tonton_specialized.cpp`: Jumping, digging, constriction
  - `tonton_takeoffanalysis.cpp`: Takeoff feasibility

**3. Memoization Layer** (`src/Memos/`):
- Caches expensive geometric computations
- `tonton_armaturememo.cpp`: Bone hierarchy analysis, symmetry detection
- `tonton_meshmemo.cpp`: Mesh-level caching
- `tonton_skinnedmeshmemo.cpp`: Combined armature+mesh analysis
- `tonton_wordlist.cpp`: Semantic parsing and flag assignment

### Key Data Structures

**Word System** (`tonton_wordlist.h`):
- `Word` enum: 260+ anatomical terms
- `SemanticFlags`: Bit flags for anatomy categorization
  - Location: LEFT, RIGHT, ANTERIOR, POSTERIOR, DORSAL, VENTRAL
  - Body regions: HEAD, NECK, SPINE, LIMB, TAIL
  - Specializations: WING, FIN, TENTACLE, GRASPER, SENSORY
  - Animal types: AVIAN, AQUATIC, ARTHROPOD, EQUINE
- `CladeFlags`: Taxonomic classification (CHORDATA, MAMMALIA, AVES, REPTILIA, PISCES, ARTHROPODA, etc.)
- **Two flag functions:**
  - `GetSemanticFlags(Word)`: Returns anatomical/functional flags
  - `GetCladeFlags(Word)`: Returns taxonomic clade inference

**Output Structure** (`tonton_output.h`):
- `Output_Physical`: Mass, volume, dimensions, inertia (always present)
- `Output_Aerial`: Wingbeat frequency, flight speeds, power costs
- `Output_Terrestrial`: Gaits, sprint speeds, posture types
- `Output_Aquatic`: Swimming modes, buoyancy, maneuverability
- `Output_Behavior`: AI archetypes, aggression, sociality, activity patterns
- `Output_Sensory`: Vision acuity, hearing range, olfaction sensitivity

### Critical External Dependency

**EigenDecomposition function** (`tonton.h`):
```cpp
std::pair<glm::quat, glm::vec3> EigenDecomposition(glm::dmat3 const& I);
```
- **NOT defined by TonTon** - users must implement this
- Computes principal axes of inertia from covariance matrix
- Expected output: vec3 (eigenvalues sorted least→greatest), quat (rotation to align principal axes)

## Semantic Parsing System

TonTon's core capability is parsing bone names into anatomical meaning:

```cpp
// "left_front_wing_tip" → {Word::left, Word::front, Word::wing, Word::tip}
// GetSemanticFlags(Word::wing) → WING | FORELIMB | AVIAN
// GetCladeFlags(Word::wing) → AVES
```

**String tokenization** (`StringToWords`):
- Splits camelCase/snake_case: "leftForeWing" → ["left", "fore", "wing"]
- Handles capital letter boundaries: "LEFTWing" → ["left", "wing"]
- Binary search lookup against sorted anatomical dictionary

**Flag assignment:**
- `GetSemanticFlags`: Functional/anatomical categorization for physics rules
- `GetCladeFlags`: Taxonomic hints for behavior/metabolism defaults

## Rules Engine Development Notes

**Critical context from README:**
- Rules in `src/Rules/` were developed via AI-assisted research synthesis
- Based on 50+ peer-reviewed papers (see `bibliography.md`)
- **This is intentional and transparent** - represents best-effort synthesis, not authoritative biology
- **Contributions from domain experts are extremely welcome**
  - If you spot physics/biology errors
  - If you know better models or more recent research
  - If you have empirical data contradicting current rules

**When modifying rules:**
1. Check `bibliography.md` for relevant papers
2. Maintain physical unit consistency (SI units throughout)
3. Comment complex biomechanical formulas with paper citations
4. Test against known animal data (e.g., dragonfly example in README)

## Code Style

**Memory Management:**
- Uses custom `counted_ptr<T>` for reference counting (see `tonton_counted_ptr.hpp`)
- `immutable_array<T>` for shared, immutable data (see `tonton_shared_array.hpp`)
- Factory pattern for main data structures: `Armature::Factory(...)`, `Output::Factory(...)`

**Naming Conventions:**
- `snake_case` for files: `tonton_aerial.cpp`
- `PascalCase` for types: `Output_Aerial`, `SemanticFlags`
- `lowercase` for enums: `Word::wing`, `CladeFlags::AVES`
- Units in variable names: `wingbeat_frequency_Hz`, `cruise_speed_m_s`, `body_mass_kg`

**Flag Operations:**
- Bitwise operators overloaded for both `SemanticFlags` and `CladeFlags`
- Use `HasFlag(flags, Flag::VALUE)` for testing
- Use `|` for combining: `CF::MAMMALIA | CF::AVES`

## Common Patterns

**Adding new anatomical terms:**
1. Add to `Word` enum in `tonton_wordlist.h`
2. Add string mapping in `word_strings` array in `tonton_wordlist.cpp` (maintains sorted order)
3. Add semantic flags in `GetSemanticFlags()` switch statement
4. Add clade flags in `GetCladeFlags()` switch statement (use top-level clade; return `NONE` if too general)

**Adding new locomotion modes:**
1. Create `src/Rules/tonton_<mode>.h/cpp`
2. Define output structure in `tonton_output.h` (e.g., `Output_<Mode>`)
3. Add factory/analysis function in rules file
4. Integrate into main `Output::Factory()` in `tonton_output.cpp`

**Memoization pattern:**
- Expensive computations cached in `*Memo` classes
- Access via `armature->computed_property()` (operator-> on counted_ptr)
- Lazy evaluation: computed on first access, cached thereafter

## Important Implementation Details

**Coordinate Systems:**
- GLM vectors/quaternions throughout
- Rest pose stored in armature
- Inertia tensors as 6-element covariance arrays: `{Ixx, Iyy, Izz, Ixy, Ixz, Iyz}`

**DoDeeDum Integration:**
- Submodule at `modules/dodeedum/`
- Provides 2D silhouette analysis: projected areas, chord lengths, span measurements
- Used internally by TonTon - not directly exposed to users

**Physics Models:**
- Pennycuick (1996): Wingbeat frequency scaling
- Kleiber's Law: Metabolic rate ∝ M^0.75
- Strouhal number: Oscillatory locomotion optimization (0.2-0.4)
- Reynolds number: Aerodynamic regime classification
- See `bibliography.md` for complete citations

## Memoization Cheat Sheet

When writing rules in `src/Rules/`, you have access to precomputed data through memo objects. Access via `input.skinnedMesh->method()` or `input.skinnedMesh->skin->method()`.

### ArmatureMemo (`armature->...`)

**Hierarchy Traversal:**
```cpp
auto children = armature->GetChildren();              // Direct children per joint
auto graph = armature->GetDirectedGraph();           // Full directed graph
auto leaves = armature->GetLeaves();                 // All leaf nodes
auto dfs = armature->GetDfsOrdering();               // Depth-first traversal order
auto gcr = armature->GetGcrTable();                  // Greatest common root table

// Filter children by semantic flags
auto wings = armature->GetAllChildren(root_joint,
    SemanticFlags::WING,           // include these
    SemanticFlags::RIGGING_CONTROL // exclude these
);
```

**Semantic Analysis:**
```cpp
auto flags = armature->GetSemanticFlags();           // Flags per joint (indexed)
auto relative = armature->GetRelativeFlags();        // Parent-child flag pairs

// Check individual joint
SemanticFlags joint_flags = flags[joint_index];
if (HasFlag(joint_flags, SemanticFlags::WING | SemanticFlags::LEFT)) {
    // This is a left wing bone
}
```

### SkinnedMesh Direct Methods (`skinnedMesh->...`)

**Volumetric Data (per joint or joint span):**
```cpp
// Single joint
auto inertia = skinnedMesh->GetInertia(joint_id, scale);
auto covariance = skinnedMesh->GetCovariance(joint_id, scale);
double cross_section = skinnedMesh->EstimateCrossSection(joint_id, scale, direction);

// Joint span (e.g., entire limb)
std::vector<uint16_t> limb_joints = {shoulder, elbow, wrist};
double volume_m3;
glm::dvec3 centroid;
auto limb_inertia = skinnedMesh->GetInertia(limb_joints, scale, &centroid, &volume_m3);
auto limb_covariance = skinnedMesh->GetCovariance(limb_joints, scale, &centroid, &volume_m3);
```

**Limb Metrics (complete characterization):**
```cpp
SkinnedMesh::LimbMetrics metrics = skinnedMesh->GetMetrics(limb_joints, scale);
// metrics.volume        - total volume
// metrics.centroid      - center of mass
// metrics.unitInertia   - normalized inertia tensor

// Compute inertia about a point
glm::dmat3 inertia = metrics.GetInertia(pivot_point, density_kg_m3);
double axial_inertia = metrics.GetInertia(pivot_point, density_kg_m3, rotation_axis);
```

**Stalk Analysis (for limbs, necks, tails):**
```cpp
SkinnedMesh::StalkData stalk;
if (skinnedMesh->GetStalkData(stalk, root_joint, tip_joint, scale)) {
    // stalk.root/tip                 - refined joint range
    // stalk.thickestCrossSection_m2  - max cross-section
    // stalk.thinestCrossSection_m2   - min cross-section
    // stalk.length_m                 - stalk length
}
```

### SkinnedMeshMemo (`skinnedMesh->...`)

**2D Projections & Silhouettes:**
```cpp
// Project along principal axes (uses eigen decomposition)
auto& silhouette = skinnedMesh->GetSilhouettes(
    EigenValue::Small,    // Small/Medium/Large eigenvalue
    scale,
    limb_joints,          // empty = whole creature
    0.5f,                 // skinning weight cutoff
    true                  // compute second moment of area
);

// Project along world axes
auto& sil_x = skinnedMesh->GetSilhouettes(Axis::X, scale, joints);
auto& sil_y = skinnedMesh->GetSilhouettes(Axis::Y, scale, joints);

// Custom projection matrix
glm::mat4 proj = GetProjectionMatrix(view_direction);
auto& custom_sil = skinnedMesh->GetSilhouettes(proj, scale, joints, 0.5f, true);

// Silhouette data:
// silhouette.area              - projected area (m²)
// silhouette.second_moment     - 2D mass distribution
// silhouette.MeasureWidth(...) - chord length measurements
```

**Symmetry & Structure:**
```cpp
auto cliques = skinnedMesh->GetCliques();
// cliques[i].parent    - common ancestor
// cliques[i].children  - symmetrical siblings (e.g., left/right legs)

auto tube_table = skinnedMesh->GetTubeTable();
float tubiness = tube_table[joint_id];  // 0=flat (fluke), 1=cylindrical (limb)
```

**Projection Helpers:**
```cpp
// Get projection matrix from eigen analysis
glm::mat4 proj_matrix = skinnedMesh->GetProjectionMatrix(
    EigenValue::Medium, scale, joints, &metrics, &eigen_decomp
);

// Standalone projection utilities
glm::mat4 proj = GetProjectionMatrix(EigenValue::Large, eigen_quat);
glm::vec3 view_dir = GetProjectionDirection(EigenValue::Small, eigen_quat);
glm::vec3 tangent = GetTangentAxis(EigenValue::Medium, eigen_quat, root_pos, centroid);
```

### MeshMemo (`mesh->...`)

**Vertex Skinning Analysis:**
```cpp
int overlap_verts = mesh->GetVertexOverlap(joint_a, joint_b);
float overlap_pct = mesh->GetVertexOverlapPercent(joint_a, joint_b);

// Tubiness from topology (alternative to SkinnedMeshMemo version)
auto children_list = armature->GetChildren()[joint_id];
float tubiness = mesh->GetTubiness(parent_joint, current_joint, children_list);
```

**Silhouettes (lower-level):**
```cpp
// Usually use SkinnedMeshMemo version instead
auto& sil = mesh->GetSilhouettes(projection, scale, joints, cutoff, second_moment);
```

### Common Usage Patterns in Rules

**Finding wings:**
```cpp
auto wing_flags = SemanticFlags::WING | SemanticFlags::FORELIMB;
auto all_flags = input.skinnedMesh->skin->GetSemanticFlags();

std::vector<int> wing_roots;
for (int i = 0; i < all_flags.size(); ++i) {
    if (HasFlag(all_flags[i], wing_flags)) {
        wing_roots.push_back(i);
    }
}
```

**Computing wing area:**
```cpp
auto wing_joints = armature->GetAllChildren(wing_root, SemanticFlags::WING);
auto& wing_silhouette = skinnedMesh->GetSilhouettes(
    EigenValue::Small,  // Project down primary body axis
    input.behavior.scale,
    wing_joints,
    0.5f, false
);
float wing_area_m2 = wing_silhouette.area;
```

**Mass properties:**
```cpp
auto metrics = skinnedMesh->GetMetrics(limb_joints, input.behavior.scale);
float limb_mass_kg = metrics.volume * input.body_density();
glm::dmat3 limb_inertia = metrics.GetInertia(shoulder_pos, input.body_density());
```

## Testing Approach

No formal test suite exists. Validation is via:
1. Known animal comparisons (e.g., dragonfly metrics in README)
2. Physics sanity checks (positive masses, realistic speeds)
3. Allometric scaling laws (body size relationships)

When making changes, verify against README examples:
- Standard dragonfly (24mm body)
- Giant dragonfly (50cm wingspan)
- Titan dragonfly (5m wingspan, low gravity)
