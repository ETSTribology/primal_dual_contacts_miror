#include "Cube.h"

namespace FrictionFrenzy {
namespace CollisionObject {
void Cube::initializeFCLObject() {
	Affine scaled;
	Isometry rigid;
	FloatType vol;
	std::vector<Vector3> transVertices;
	preprocessMesh(rigid, scaled, transVertices, vol);

	// Compute a box with sides that have the identical inertia tensor.
	Matrix3 m = momentOfInertia(transVertices, *mp_mesh->indices_Eigen);
	Eigen::JacobiSVD svd(m);
	svd.compute(m, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Vector3 svFac = 12.f / vol * svd.singularValues();
	FloatType sSums = svFac.sum() / 2;
	m_sides = (sSums - svFac.array()).sqrt();

	Matrix3 r = svd.matrixU();

	// Using SVD results in sides of descending length. Now compute a
	// permutation of the sides to align the sides of the original mesh.
	Eigen::Vector3i perm(-1, -1, -1);
	for (int i = 0; i < 3; ++i) {
		FloatType maxi = std::abs(r(i, i));
		int col = i;
		for (int j = 0; j < 3; ++j) {
			if (std::abs(r(i, j)) > maxi) {
				bool used = false;
				for (int k = 0; k < j; ++k) {
					if (perm[k] == j) {
						used = true;
						break;
					}
				}
				if (!used) {
					maxi = std::abs(r(i, j));
					col = j;
				}
			}
		}
		perm[i] = col;
	}
	Matrix3 permMat = Matrix3::Zero(3, 3);
	for (int i = 0; i < 3; ++i) {
		permMat(perm[i], i) = 1;
	}
	r = r * permMat;
	m_sides = permMat.transpose() * m_sides;

	// Force the rotation to have positive determinant.
	if (r.determinant() < 0) {
		r.row(2) *= -1;
	}

	// Initialize FCL object and transformations.
	mp_geo = std::make_unique<fcl::Box<FloatType>>(m_sides);
	rigid.linear() = rigid.linear() * r;
	Affine meshScaled = scaled;
	meshScaled.linear() = r.transpose() * meshScaled.linear();
	scaled.linear() = (m_sides / 2).asDiagonal();
	scaled.translation().setZero();
	initializeConfig(rigid, scaled, meshScaled);
}
FloatType Cube::obbMinLength() const {
	// The oriented bounding box is just the box itself.
	auto* box = dynamic_cast<fcl::Box<FloatType>*>(mp_geo.get());
	return box->side.minCoeff();
}
std::string Cube::toString() const {
	std::stringstream ss;
	ss << "Type: cube\n"
	   << "dimensions: " << m_sides.transpose() << "\n"
	   << RigidObjectBase::toString();
	return ss.str();
}
}  // namespace CollisionObject
}  // namespace FrictionFrenzy
