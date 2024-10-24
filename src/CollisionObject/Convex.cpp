#include "Convex.h"
#include "fcl/geometry/shape/convex.h"

#include <Eigen/src/Core/util/Meta.h>
#include <Magnum/EigenIntegration/Integration.h>
#include <igl/barycentric_coordinates.h>
#include <igl/per_edge_normals.h>
#include <igl/per_face_normals.h>
#include <igl/per_vertex_normals.h>
#include <igl/project_to_line_segment.h>
#include <igl/signed_distance.h>

namespace FrictionFrenzy {
namespace CollisionObject {
void Convex::initializeFCLObject() {
	// Extract all transformations and transformed vertices.
	Affine scaled;
	Isometry rigid;
	FloatType vol;
	mp_transVertices = std::make_shared<std::vector<Vector3>>();
	preprocessMesh(rigid, scaled, *mp_transVertices, vol);

	// Fill in the mesh indices in FCL's designated format.
	const MatrixX3i& indicesEigen = *(mp_mesh->indices_Eigen);
	mp_indices = std::make_shared<std::vector<int>>();
	for (int i = 0; i < indicesEigen.rows(); ++i) {
		Vector3i tri = indicesEigen.row(i).transpose();
		(*mp_indices).push_back(3);
		(*mp_indices).push_back(tri[0]);
		(*mp_indices).push_back(tri[1]);
		(*mp_indices).push_back(tri[2]);
	}
	// Initialize FCL collision geometry
	mp_geo = std::make_shared<fcl::Convex<FloatType>>(
		mp_transVertices, indicesEigen.rows(), mp_indices);

	// Initialize from transformations.
	initializeConfig(rigid, scaled, scaled);
}

FloatType Convex::obbMinLength() const {
	// Compute an oriented bounding box with SVD and return the minimum length.
	Vector3 com = mp_geo->computeCOM();
	Matrix3X vToCOM(3, mp_transVertices->size());
	for (size_t i = 0; i < mp_transVertices->size(); ++i) {
		vToCOM.col(i) << (*mp_transVertices)[i] - com;
	}
	Matrix3 pcaMat = vToCOM * vToCOM.transpose() / mp_transVertices->size();
	Eigen::JacobiSVD<Matrix3, Eigen::ComputeFullU> svd(pcaMat,
	                                                   Eigen::ComputeFullU);
	Matrix3 uT = svd.matrixU().transpose();
	Matrix3X localVerts = uT * vToCOM;
	Vector3 boxSize =
		localVerts.rowwise().maxCoeff() - localVerts.rowwise().minCoeff();
	return boxSize.minCoeff();
}

std::string Convex::toString() const {
	std::stringstream ss;
	ss << "Type: convex\n"
	   << mp_transVertices->size() << " vertices\n"
	   << mp_indices->size() / 4 << " triangles\n"
	   << RigidObjectBase::toString();
	return ss.str();
}
}  // namespace CollisionObject
}  // namespace FrictionFrenzy
