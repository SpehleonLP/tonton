#include "tonton_eigen.h"
#include <glm/gtc/quaternion.hpp>


struct tonton_eigen
{
	glm::dmat3 vectors;
	glm::dvec3 values;
};

using dvec3 = glm::dvec3;

static glm::dmat3x3 compute_rotation_3x3(tonton_eigen const& eig)
{
	glm::dmat3 R;
	
    // Treat eigenvectors as columns, directly
    dvec3 c0 = glm::normalize(eig.vectors[0]);
    dvec3 c1 = glm::normalize(eig.vectors[1]);
    // Gram-Schmidt orthogonalize
    double dot01 = glm::dot(c0, c1);
    dvec3 tmp1 = {c1.x - dot01 * c0.x,
                 c1.y - dot01 * c0.y,
                 c1.z - dot01 * c0.z};
    c1 = glm::normalize(tmp1);
    // Third column from cross → right-handed basis
    dvec3 c2 = glm::cross(c0, c1);
    R[0] = c0;
    R[1] = c1;
    R[2] = c2;

    // Determinant check AFTER orthonormalization
    double det =
          c0.x * (c1.y * c2.z - c1.z * c2.y)
        - c0.y * (c1.x * c2.z - c1.z * c2.x)
        + c0.z * (c1.x * c2.y - c1.y * c2.x);
    if (det < 0.0) {
        // Flip third column
        R[2].x = -R[2].x;
        R[2].y = -R[2].y;
        R[2].z = -R[2].z;
    }

	return R;
}

static glm::quat compute_rotation_quat(tonton_eigen const& eigen)
{
	glm::mat3x3 rotMatrix = compute_rotation_3x3(eigen);
	return glm::quat_cast(rotMatrix);
}

static tonton_eigen sort_eigen(tonton_eigen & in)
{
	int _min = in.values[0] < in.values[1]? 0 : 1;
	int _max = 1 - _min;
	
	_min = in.values[_min] <= in.values[2]? _min : 2;
	_max = in.values[_max] > in.values[2]? _max : 2;
	//the three indices must sum to 0+1+2=3.
	int _mid = 3 - _min - _max;
		
	tonton_eigen r = {
		.vectors={in.vectors[_min], in.vectors[_mid], in.vectors[_max]},
		.values={in.values[_min], in.values[_mid], in.values[_max]},
	};
	
	return r;
}

static void makeConsistent(tonton_eigen* result) {
      // Choose the eigenvector component with largest absolute value
      // and make it positive (this matches Eigen's typical behavior)
      for (int i = 0; i < 3; i++) {
          dvec3 * v = &result->vectors[i];

          // Find component with largest absolute value
          int max_idx = 0;
          if (fabs(v->y) > fabs(v->x)) max_idx = 1;
          if (fabs(v->z) > fabs((&v->x)[max_idx])) max_idx = 2;

          // Make that component positive
          if ((&v->x)[max_idx] < 0) {
              v->x = -v->x;
              v->y = -v->y;
              v->z = -v->z;
          }
      }
  }
  
#define getElement(a, b, c) (a[b][c])

static tonton_eigen compute_eigen_m3(glm::dmat3x3 const& I) {
    auto A = I;
    
    double V[3][3] = {
        {1, 0, 0},
        {0, 1, 0},
        {0, 0, 1}
    };
    
    const int maxIterations = 50;
    
    // Compute adaptive tolerance based on matrix scale
    double matrixScale = 0.0;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            matrixScale += fabs(getElement(A, i, j));
        }
    }
    
    if(matrixScale == 0.0)
    {
		return (tonton_eigen)
		{
			.vectors = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}},
			.values = {0, 0, 0}
		};
    }
    
    matrixScale /= 9.0; // Average absolute value of matrix elements
    
	// Scale matrix up to reasonable range
	double scale_factor = 1.0;
	if (matrixScale < 1e-6) {  // or whatever threshold makes sense
		scale_factor = 1e6 / matrixScale;  // bring into ~1.0 range
	}
	if(matrixScale > 10.0)
	{
		scale_factor = 10.0 / matrixScale;
	}

	for(int i = 0; i < 9; ++i)		
		A[i/3][i%3] *= scale_factor;

    // Adaptive tolerance: scale with matrix magnitude, but keep reasonable bounds
    double tolerance = fmax(1e-12, fmin(1e-6, matrixScale * scale_factor * 1e-6));
  
	int iter = 0;
    for (iter = 0; iter < maxIterations; iter++) {
        int p = 0, q = 1;
        double maxOffDiag = 0.0;
        
        for (int i = 0; i < 3; i++) {
            for (int j = i + 1; j < 3; j++) {
                double val = fabs(getElement(A, i, j));
                if (val > maxOffDiag) {
                    maxOffDiag = val;
                    p = i;
                    q = j;
                }
            }
        }
        
        if (maxOffDiag < tolerance) {
            break;
        }
        
        if (fabs(getElement(A, p, q)) < 1e-15) continue;
        
        double tau = (getElement(A, q, q) - getElement(A, p, p)) / (2.0 * getElement(A, p, q));
        double t = (tau >= 0 ? 1.0 : -1.0) / (fabs(tau) + sqrt(1.0 + tau * tau));
        double c = 1.0 / sqrt(1.0 + t * t);
        double s = t * c;
        
        double App = getElement(A, p, p);
        double Aqq = getElement(A, q, q);
        double Apq = getElement(A, p, q);
        
        getElement(A, p, p) = c * c * App - 2.0 * s * c * Apq + s * s * Aqq;
        getElement(A, q, q) = s * s * App + 2.0 * s * c * Apq + c * c * Aqq;
        getElement(A, p, q) = getElement(A, q, p) = 0.0;
        
        for (int r = 0; r < 3; r++) {
            if (r != p && r != q) {
                double Arp = getElement(A, r, p);
                double Arq = getElement(A, r, q);
                getElement(A, r, p) = getElement(A, p, r) = c * Arp - s * Arq;
                getElement(A, r, q) = getElement(A, q, r) = s * Arp + c * Arq;
            }
        }
        
        for (int i = 0; i < 3; i++) {
            double Vip = getElement(V, i, p);
            double Viq = getElement(V, i, q);
            getElement(V, i, p) = c * Vip - s * Viq;
            getElement(V, i, q) = s * Vip + c * Viq;
        }
    }
    
    tonton_eigen result;
    for (int i = 0; i < 3; i++) {
        result.values[i] = getElement(A, i, i) / scale_factor;
        result.vectors[i].x = getElement(V, 0, i);
        result.vectors[i].y = getElement(V, 1, i);
        result.vectors[i].z = getElement(V, 2, i);
    }
        
    makeConsistent(&result);    
    return sort_eigen(result);
}


std::pair<glm::quat, glm::vec3> TonTon::EigenDecomposition(glm::dmat3 const& m)
{
	auto tmp = compute_eigen_m3(m);
	return {compute_rotation_quat(tmp), tmp.values};
}

TonTon::MatrixClassification TonTon::ClassifySymmetricMatrix(glm::dmat3 const& it) {
    
    // Use Sylvester's criterion - check signs of leading principal minors
    // For 3x3 symmetric matrix:
    // M1 = xx
    // M2 = det([[xx, xy], [xy, yy]]) = xx*yy - xy*xy
    // M3 = det(full matrix)
    
    const double tolerance = 1e-12;
    
    // First leading principal minor
    double M1 = it[0].x;
    
    // Second leading principal minor  
    double M2 = it[0].x * it[1].y - it[0].y * it[0].y;
    
    // Third leading principal minor (full determinant)
    double M3 = it[0].x * (it[1].y * it[2].z - it[1].z * it[1].z)
              - it[0].y * (it[0].y * it[2].z - it[1].z * it[0].z)
              + it[0].z * (it[0].y * it[1].z - it[1].y * it[0].z);
    
    // Check for positive definite: all leading principal minors > 0
    if (M1 > tolerance && M2 > tolerance && M3 > tolerance) {
        return MatrixClassification(POSITIVE | DEFINITE);
    }
    
    // Check for negative definite: alternating signs starting with M1 < 0
    if (M1 < -tolerance && M2 > tolerance && M3 < -tolerance) {
        return MatrixClassification(NEGATIVE | DEFINITE);
    }
    
    // For semi-definite cases, we need to check if all eigenvalues have the same sign
    // This requires checking all principal minors of all sizes
    
    // All 1x1 principal minors
    double minor_xx = it[0].x;
    double minor_yy = it[1].y;
    double minor_zz = it[2].z;
    
    // All 2x2 principal minors
    double minor_xy = it[0].x * it[1].y - it[0].y * it[0].y;  // top-left 2x2
    double minor_xz = it[0].x * it[2].z - it[0].z * it[0].z;  // (0,0),(2,2) 2x2
    double minor_yz = it[1].y * it[2].z - it[1].z * it[1].z;  // bottom-right 2x2
    
    // Check for positive semi-definite
    // All principal minors >= 0, at least one = 0
    if (minor_xx >= -tolerance && minor_yy >= -tolerance && minor_zz >= -tolerance &&
        minor_xy >= -tolerance && minor_xz >= -tolerance && minor_yz >= -tolerance &&
        M3 >= -tolerance) {
        
        // Check if at least one minor is zero (making it semi-definite, not definite)
        if (fabs(minor_xx) <= tolerance || fabs(minor_yy) <= tolerance || fabs(minor_zz) <= tolerance ||
            fabs(minor_xy) <= tolerance || fabs(minor_xz) <= tolerance || fabs(minor_yz) <= tolerance ||
            fabs(M3) <= tolerance) {
            return MatrixClassification(POSITIVE | SEMI_DEFINITE);
        }
    }
    
    // Check for negative semi-definite
    // This is trickier - we need (-1)^k * M_k >= 0 for all principal minors M_k of size k
    // For negative semi-definite: (-A) should be positive semi-definite
    if (-minor_xx >= -tolerance && -minor_yy >= -tolerance && -minor_zz >= -tolerance &&
        minor_xy >= -tolerance && minor_xz >= -tolerance && minor_yz >= -tolerance &&
        -M3 >= -tolerance) {
        
        // Check if at least one condition makes it semi- rather than definite
        if (fabs(minor_xx) <= tolerance || fabs(minor_yy) <= tolerance || fabs(minor_zz) <= tolerance ||
            fabs(minor_xy) <= tolerance || fabs(minor_xz) <= tolerance || fabs(minor_yz) <= tolerance ||
            fabs(M3) <= tolerance) {
            return MatrixClassification(NEGATIVE | SEMI_DEFINITE);
        }
    }
    
    // If none of the above, matrix is indefinite
    return MatrixClassification(0);
}


