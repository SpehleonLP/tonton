#ifndef TONTON_EIGEN_H
#define TONTON_EIGEN_H
#include <glm/fwd.hpp>
#include <utility>

namespace TonTon
{

enum MatrixClassification
{
	INDEFINITE = 0,
	POSITIVE = 1 << 0,
	NEGATIVE = 1 << 1,
	DEFINITE = 1 << 2,
	SEMI_DEFINITE = 1 << 3
};

std::pair<glm::quat, glm::vec3> EigenDecomposition(glm::dmat3 const& m);
TonTon::MatrixClassification ClassifySymmetricMatrix(glm::dmat3 const& it);
}

#endif // TONTON_EIGEN_H
