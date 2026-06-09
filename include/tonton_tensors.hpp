#ifndef TONTON_TENSORS_HPP
#define TONTON_TENSORS_HPP
#include <glm/glm.hpp>

namespace TonTon {

// Second moment of volume:  C_ij = ∫ x_i x_j dV   (units m^5, taken about the centroid).
// This is what rintintin returns and what SkinnedMesh stores per joint.
// Eigen-polarity: the SMALLEST eigenvalue is the THINNEST physical axis.
struct SecondMomentTensor { glm::dmat3 C{1.0}; };

// Unit-density inertia tensor:  I = tr(C)·𝟙 − C.
// Eigen-polarity: the SMALLEST eigenvalue is the PRINCIPAL / LONGEST physical axis
// (opposite of SecondMomentTensor — this reversal is the footgun this header prevents).
struct InertiaTensor { glm::dmat3 I{1.0}; };

// The ONE place the C → I conversion lives.
inline InertiaTensor ToInertia(const SecondMomentTensor& c)
{
    const glm::dmat3& C = c.C;
    const double tr = C[0][0] + C[1][1] + C[2][2];
    return InertiaTensor{ tr * glm::dmat3(1.0) - C };
}

} // namespace TonTon
#endif // TONTON_TENSORS_HPP
