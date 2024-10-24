#ifndef __MATRIXTYPES_H__
#define __MATRIXTYPES_H__

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Sparse>

#include "FloatType.h"

namespace FrictionFrenzy {

typedef Eigen::SparseMatrix<FloatType, Eigen::RowMajor> SparseMat;
typedef Eigen::ConjugateGradient<SparseMat, Eigen::Lower | Eigen::Upper> CG;
typedef Eigen::SimplicialLDLT<SparseMat> LDLT;
typedef Eigen::SparseLU<SparseMat, Eigen::COLAMDOrdering<int>> LU;
typedef Eigen::Matrix<FloatType, -1, 1> VectorX;
typedef Eigen::Matrix<FloatType, 2, 1> Vector2;
typedef Eigen::Matrix<FloatType, 3, 1> Vector3;
typedef Eigen::Matrix<FloatType, 4, 1> Vector4;
typedef Eigen::Matrix<FloatType, 6, 1> Vector6;
typedef Eigen::Matrix<FloatType, 2, 2> Matrix2;
typedef Eigen::Matrix<FloatType, 3, 3> Matrix3;
typedef Eigen::Matrix<FloatType, 4, 4> Matrix4;
typedef Eigen::Matrix<FloatType, 6, 6> Matrix6;
typedef Eigen::Matrix<FloatType, -1, 3> MatrixX3;
typedef Eigen::Matrix<FloatType, -1, 2> MatrixX2;
typedef Eigen::Matrix<FloatType, 3, -1> Matrix3X;
typedef Eigen::Matrix<FloatType, -1, -1> MatrixX;
typedef Eigen::Matrix<int, -1, 1> VectorXi;
typedef Eigen::Matrix<int, 3, 1> Vector3i;
typedef Eigen::Matrix<int, -1, 3> MatrixX3i;
typedef Eigen::Matrix<int, -1, -1> MatrixXi;
typedef Eigen::Transform<FloatType, 3, Eigen::Affine> Affine;
typedef Eigen::Transform<FloatType, 3, Eigen::Isometry> Isometry;
typedef Eigen::Quaternion<FloatType> Quaternion;
typedef Eigen::AngleAxis<FloatType> AngleAxis;

}  // namespace FrictionFrenzy

#endif
