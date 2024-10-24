#include "Ellipsoid.h"

#include "CollisionObject/RigidObjectBase.h"
namespace FrictionFrenzy {
namespace CollisionObject {
void Ellipsoid::initializeFCLObject() {
	Affine scaled;
	Isometry rigid;
	FloatType vol;
	std::vector<Vector3> transVertices;
	preprocessMesh(rigid, scaled, transVertices, vol);

	// Compute a ellipsoid with sides that have the identical inertia tensor.
	Matrix3 m = momentOfInertia(transVertices, *mp_mesh->indices_Eigen);
	Eigen::JacobiSVD svd(m);
	svd.compute(m, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Vector3 svFac = 5.f / vol * svd.singularValues();
	FloatType sSums = svFac.sum() / 2;
	m_dimensions = (sSums - svFac.array()).sqrt();
	mp_geo = std::make_unique<fcl::Ellipsoid<FloatType>>(m_dimensions);
	Matrix3 r = svd.matrixU();

	// Force the rotation to have positive determinant.
	if (r.determinant() < 0) {
		r.row(2) *= -1;
	}

	// Initialize FCL object and transformations.
	rigid.linear() = rigid.linear() * r;
	Affine meshScaled = scaled;
	meshScaled.linear() = r.transpose() * meshScaled.linear();
	scaled.linear() = (m_dimensions).asDiagonal();
	scaled.translation().setZero();

	initializeConfig(rigid, scaled, meshScaled);
}
FloatType Ellipsoid::obbMinLength() const {
	// Minimum length of the OBB is the minimum length of the ellipsoid
	auto* ellipsoid = dynamic_cast<fcl::Ellipsoid<FloatType>*>(mp_geo.get());
	return ellipsoid->radii.minCoeff();
}
std::string Ellipsoid::toString() const {
	std::stringstream ss;
	ss << "Type: ellipsoid\n"
	   << "dimensions: " << m_dimensions.transpose() << "\n"
	   << RigidObjectBase::toString();
	return ss.str();
}
}  // namespace CollisionObject
}  // namespace FrictionFrenzy
