#include "CollisionObject/RigidObjectBase.h"
#include <Eigen/src/Core/util/Constants.h>
#include <Eigen/src/Geometry/AngleAxis.h>
#include <imgui/imgui.h>
#include "Common/ImGUIConfigurable.h"
#include "Common/MatrixTypes.h"

namespace FrictionFrenzy {
namespace CollisionObject {

#pragma region MagnumCollisionObject
void RigidObjectBase::setPermanentTrans(Eigen::Ref<const Matrix4> trans) {
	m_permanentTrans = trans;
	updateFCLObject();
}
void RigidObjectBase::setConfiguration(Eigen::Ref<const Vector3> pos,
                                       const Quaternion& quat) {
	m_translation = pos;
	m_rotation = quat;
	updateFCLObject();
}
void RigidObjectBase::setTranslation(Eigen::Ref<const Vector3> pos) {
	m_translation = pos;
	updateFCLObject();
}
void RigidObjectBase::setRotation(const Quaternion& quat) {
	m_rotation = Quaternion(quat);
	updateFCLObject();
}
void RigidObjectBase::setConfiguration(const Affine& affine) {
	FrictionFrenzy::Quaternion q(affine.rotation());
	FrictionFrenzy::Vector3 t = affine.translation();
	setConfiguration(t, q);
	updateFCLObject();
}
void RigidObjectBase::updateFCLObject() {
	// This is called when the object was first created, before the FCL
	// collision object is called. So we need to test if m_collisionObject is
	// defined.
	if (mp_collisionObject) {
		mp_collisionObject->setTransform(getRigidTransMatrix());
		mp_collisionObject->computeAABB();
	}
}
void RigidObjectBase::updateDensity(FloatType newDensity) {
	m_density = newDensity;
	m_mass = m_volume * m_density;
	m_iMass = 1. / m_mass;
	m_moment0 = m_moment0Vol1 * m_density;
	m_iMoment0 = m_iMoment0Vol1 / m_density;
}
void RigidObjectBase::reset() {
	m_rotation = m_rotationOrig;
	m_translation = m_translationOrig;
	if (!m_isStatic) {
		m_velocity = {0, 0, 0};
		m_angularVel = {0, 0, 0};
	}
	updateFCLObject();
}
void RigidObjectBase::initializeConfig(const Isometry& rigid,
                                       const Affine& scaled,
                                       const Affine& meshScaled) {
	// Create FCL collision object
	mp_collisionObject =
		std::make_unique<fcl::CollisionObject<FloatType>>(mp_geo, rigid);

	// Initializing configurations.
	m_permanentTrans = scaled;
	m_translation = rigid.translation();
	m_rotation = rigid.rotation();
	m_translationOrig = m_translation;
	m_rotationOrig = m_rotation;
	m_origMeshTrans = meshScaled;

	// Initializing mass, volume, and inertia tensors.
	m_volume = mp_geo->computeVolume();
	m_mass = m_volume * m_density;
	m_iMass = 1. / m_mass;
	m_moment0Vol1 = mp_geo->computeMomentofInertiaRelatedToCOM();
	m_iMoment0Vol1 = m_moment0Vol1.inverse();
	m_moment0 = m_moment0Vol1 * m_density;
	m_iMoment0 = m_iMoment0Vol1 / m_density;
	updateFCLObject();
}
Eigen::Matrix<FloatType, 3, 8> RigidObjectBase::aabbVel() const {
	const auto& aabb = mp_collisionObject->getAABB();
	Vector3 aabbCenter = aabb.center();
	Vector3 halfSize(aabb.width() / 2, aabb.height() / 2, aabb.depth() / 2);

	// Find the positions of all 8 corners of the AABB.
	Eigen::Matrix<FloatType, 3, 8> aabbVertPos;
	for (int i = 0; i < 8; ++i) {
		Vector3 sign((i & 4) ? 1 : -1, (i & 2) ? 1 : -1, (i & 1) ? 1 : -1);
		aabbVertPos.col(i) << aabbCenter + halfSize.cwiseProduct(sign);
	}
	aabbVertPos.colwise() -= m_translation;

	// Find the velocities of all 8 corners of the AABB.
	Eigen::Matrix<FloatType, 3, 8> vertVel;
	vertVel.colwise() = m_velocity;
	vertVel -= aabbVertPos.colwise().cross(m_angularVel);

	return vertVel;
}
Matrix6 RigidObjectBase::genMassMat() const {
	Matrix6 ret = Matrix6::Zero();
	Matrix3 rot = m_rotation.toRotationMatrix();
	ret.block<3, 3>(0, 0) = Matrix3::Identity() * m_mass;
	ret.block<3, 3>(3, 3) = rot * m_moment0 * rot.transpose();
	return ret;
}
Matrix6 RigidObjectBase::invGenMassMat() const {
	Matrix6 ret = Matrix6::Zero();
	Matrix3 rot = m_rotation.toRotationMatrix();
	ret.block<3, 3>(0, 0) = Matrix3::Identity() * m_iMass;
	ret.block<3, 3>(3, 3) = rot * m_iMoment0 * rot.transpose();
	return ret;
}

void RigidObjectBase::polarDecomposition(const Affine& in,
                                         Isometry& rigid,
                                         Affine& scaled) {
	Vector3 translate = in.translation();
	Matrix3 lin = in.linear();

	Eigen::JacobiSVD<Matrix3> svd(lin,
	                              Eigen::ComputeFullU | Eigen::ComputeFullV);
	Matrix3 rot = svd.matrixU() * svd.matrixV().transpose();
	if (rot.determinant() < 0) {
		rot = svd.matrixU();
		rot.col(0) *= -1;
		rot = rot * svd.matrixV().transpose();
	}
	Matrix3 spd = rot.transpose() * lin;

	scaled = Affine::Identity();
	scaled.linear() = spd;
	rigid.linear() = rot;
	rigid.translation() = translate;
}
void RigidObjectBase::stripTransformation(
	const CPUMeshData& mesh,
	const Affine& scaledEigen,
	std::vector<Vector3>& transformedVertices,
	Eigen::Ref<Vector3> centerOfMass,
	FloatType& volume) {
	centerOfMass = Vector3::Zero(3);
	transformedVertices.resize(mesh.positions_Eigen.rows());
	for (size_t i = 0; i < transformedVertices.size(); ++i) {
		transformedVertices[i] =
			(scaledEigen *
		     mesh.positions_Eigen.row(i).transpose().homogeneous())
				.head(3);
	}
	volume = 0;
	MatrixX3i& indices = *(mesh.indices_Eigen);
	for (Eigen::Index i = 0; i < indices.rows(); ++i) {
		Vector3 v1 = transformedVertices[indices(i, 0)];
		Vector3 v2 = transformedVertices[indices(i, 1)];
		Vector3 v3 = transformedVertices[indices(i, 2)];
		FloatType volTet = v1.cross(v2).dot(v3) / 6;
		Vector3 cTet = (v1 + v2 + v3) / 4;
		centerOfMass += volTet * cTet;
		volume += volTet;
	}
	centerOfMass /= volume;
	for (Eigen::Index i = 0; i < mesh.positions_Eigen.rows(); ++i) {
		transformedVertices[i] -= centerOfMass;
	}
	return;
}
Matrix3 RigidObjectBase::momentOfInertia(const std::vector<Vector3>& vertices,
                                         Eigen::Ref<const MatrixX3i> indices) {
	Matrix3 C = Matrix3::Zero();

	Matrix3 C_canonical;
	C_canonical << 1 / 60.0, 1 / 120.0, 1 / 120.0, 1 / 120.0, 1 / 60.0,
		1 / 120.0, 1 / 120.0, 1 / 120.0, 1 / 60.0;

	for (int i = 0; i < indices.rows(); ++i) {
		const Vector3& v1 = vertices[indices(i, 0)];
		const Vector3& v2 = vertices[indices(i, 1)];
		const Vector3& v3 = vertices[indices(i, 2)];
		FloatType d_six_vol = (v1.cross(v2)).dot(v3);
		Matrix3 A;
		A.row(0) = v1;
		A.row(1) = v2;
		A.row(2) = v3;
		C.noalias() += A.transpose() * C_canonical * A * d_six_vol;
	}

	FloatType trace_C = C(0, 0) + C(1, 1) + C(2, 2);

	Matrix3 m;
	m << trace_C - C(0, 0), -C(0, 1), -C(0, 2), -C(1, 0), trace_C - C(1, 1),
		-C(1, 2), -C(2, 0), -C(2, 1), trace_C - C(2, 2);
	return m;
}
void RigidObjectBase::preprocessMesh(Isometry& rigid,
                                     Affine& scaled,
                                     std::vector<Vector3>& transVertices,
                                     FloatType& vol) {
	Vector3 com;
	polarDecomposition(m_permanentTrans, rigid, scaled);
	stripTransformation(*mp_mesh, scaled, transVertices, com, vol);

	rigid.translation() += rigid.rotation() * com;
	scaled.translation() -= com;
}

void RigidObjectBase::showImGUIMenu() {
	std::stringstream ss;
	ss << "Object " << m_id;
	if (ImGui::CollapsingHeader(ss.str().c_str(),
	                            ImGuiTreeNodeFlags_DefaultOpen)) {
		ss.str(std::string());
		ImGui::Text("%s", toString().c_str());
		if (ImGui::SliderFloatType("density", &m_density, 0.01, 100)) {
			updateDensity(m_density);
		}
		Vector3 posVec = this->getTranslation();
		if (ImGui::InputFloatType3("position", posVec.data(), "%.4g")) {
			this->setTranslation(posVec);
		}
		Eigen::AngleAxis<FloatType> rot(this->getRotation());
		bool updateRot =
			ImGui::InputFloatType3("axis", rot.axis().data(), "%.4g");
		updateRot |=
			ImGui::InputFloatType("angle", &rot.angle(), 0, M_PI, "%.4f");
		if (updateRot) {
			this->setRotation(Quaternion(rot));
		}

		ImGui::InputFloatType3("velocity", this->velocity().data(), "%.4g");
		ImGui::InputFloatType3("angular velocity", this->angularVel().data(),
		                       "%.4g");
		ImGui::Checkbox("Is static", &m_isStatic);
	}
}
std::string RigidObjectBase::toString() const {
	std::stringstream ss;
	ss << "Mass: " << m_mass << "\nVolume: " << m_mass / m_density;
	return ss.str();
}
}  // namespace CollisionObject
}  // namespace FrictionFrenzy
