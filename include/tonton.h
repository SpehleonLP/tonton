#ifndef TONTON_H
#define TONTON_H

// Bumped whenever Builder / Articulations / SkinnedMesh in-memory layout
// changes in a way that breaks binary cache compatibility. Engine-side
// caches stamp this in the file header and reject mismatched files.
#define TONTON_VERSION 1

#include "tonton_skinnedmesh.h"
#include "tonton_analysis.h"
#include "tonton_wordlist.h"
#include "tonton_formatter.h"
#include "tonton_tensors.hpp"

#include <glm/fwd.hpp>
#include <utility>

namespace TonTon {

// Typed entry points. Polarity differs by kind — see tonton_tensors.hpp.
//   InertiaTensor      → eigenvalues ascending; Small = principal/long axis.
//   SecondMomentTensor → eigenvalues ascending; Small = thinnest axis.
std::pair<glm::quat, glm::vec3> EigenDecomposition(InertiaTensor const& i);
std::pair<glm::quat, glm::vec3> EigenDecomposition(SecondMomentTensor const& c);

} // namespace TonTon

#endif

