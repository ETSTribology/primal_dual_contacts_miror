#include "Mesh.h"

#include <Eigen/src/Core/Matrix.h>
#include <Magnum/EigenIntegration/Integration.h>
#include <igl/barycentric_coordinates.h>
#include <igl/per_edge_normals.h>
#include <igl/per_face_normals.h>
#include <igl/per_vertex_normals.h>
#include <igl/project_to_line_segment.h>
#include <igl/signed_distance.h>
#include <limits>
#include <vector>
#include "CollisionObject/RigidObjectBase.h"
#include "Common/MatrixTypes.h"

namespace FrictionFrenzy {
namespace CollisionObject {
void Mesh::initializeFCLObject() {
	// Extract all transformations and transformed vertices.
	Affine scaled;
	Isometry rigid;
	FloatType vol;
	std::vector<Vector3> transVertices;
	preprocessMesh(rigid, scaled, transVertices, vol);

	// Construct triangles for the FCL structure.
	std::vector<fcl::Triangle> indices;
	const MatrixX3i& indicesEigen = *(mp_mesh->indices_Eigen);
	for (int i = 0; i < indicesEigen.rows(); ++i) {
		Vector3i tri = indicesEigen.row(i).transpose();
		indices.emplace_back(tri[0], tri[1], tri[2]);
	}

	// Construct the FCL BVH model.
	mp_geo = std::make_shared<BVH>();
	auto* bvh = dynamic_cast<BVH*>(mp_geo.get());
	bvh->beginModel();
	bvh->addSubModel(transVertices, indices);
	bvh->endModel();

	// Calculate the face areas;
	m_faceAreas.reserve(indicesEigen.rows());
	for (int i = 0; i < indicesEigen.rows(); ++i) {
		Vector3 v1 = (transVertices[indicesEigen(i, 1)] -
		              transVertices[indicesEigen(i, 0)]);
		Vector3 v2 = (transVertices[indicesEigen(i, 2)] -
		              transVertices[indicesEigen(i, 0)]);
		m_faceAreas.push_back(0.5 * v1.cross(v2).norm());
	}

	// Construct SDF query structure
	m_sdfQuery.transformedPositions.resize(transVertices.size(), 3);
	for (size_t i = 0; i < transVertices.size(); ++i) {
		m_sdfQuery.transformedPositions.row(i) << transVertices[i].transpose();
	}
	m_sdfQuery.p_indices = mp_mesh->indices_Eigen;
	const MatrixX3& V = m_sdfQuery.transformedPositions;
	const MatrixX3i& F = *(m_sdfQuery.p_indices.lock());
	Eigen::MatrixX2i E;
	m_sdfQuery.aabbTree.init(V, F);
	igl::per_face_normals(V, F, m_sdfQuery.faceNormals);
	igl::per_vertex_normals(V, F, igl::PER_VERTEX_NORMALS_WEIGHTING_TYPE_ANGLE,
	                        m_sdfQuery.faceNormals, m_sdfQuery.vertNormals);
	igl::per_edge_normals(V, F, igl::PER_EDGE_NORMALS_WEIGHTING_TYPE_UNIFORM,
	                      m_sdfQuery.faceNormals, m_sdfQuery.edgeNormals, E,
	                      m_sdfQuery.edgeMap);

	// Construct vertex adjacency map
	std::vector<std::vector<int>> adjMap_Inner(V.rows());
	for (int i = 0; i < F.rows(); ++i) {
		for (int v0 = 0; v0 < 3; ++v0) {
			for (int v1 = v0 + 1; v1 < 3; ++v1) {
				adjMap_Inner[v0].push_back(v1);
				adjMap_Inner[v1].push_back(v0);
			}
		}
	}

	auto& adjMap = m_sdfQuery.vertAdjMap;
	size_t mapSize = V.rows();
	for (int i = 0; i < adjMap_Inner.size(); ++i) {
		mapSize += adjMap_Inner[i].size();
	}
	adjMap.reserve(V.rows() + 1 + mapSize);
	adjMap.resize(V.rows() + 1);
	for (int i = 0; i < V.rows(); ++i) {
		size_t dataOffset = adjMap.size();
		adjMap[i] = dataOffset;
		adjMap.insert(adjMap.end(), adjMap_Inner[i].begin(),
		              adjMap_Inner[i].end());
	}
	adjMap[V.rows()] = adjMap.size();

	// Calculate vertex areas
	m_sdfQuery.vertArea.resize(V.rows());
	m_sdfQuery.vertArea.setZero();
	for (int i = 0; i < F.rows(); ++i) {
		for (int j = 0; j < 3; ++j) {
			m_sdfQuery.vertArea(F(i, j)) += m_faceAreas[i];
		}
	}
	m_sdfQuery.vertArea /= 3;

	initializeConfig(rigid, scaled, scaled);
}

FloatType Mesh::obbMinLength() const {
	// Construct OBB around mesh with SVD and return the shortest length.
	auto* bvh = dynamic_cast<BVH*>(mp_geo.get());
	Vector3* verts = bvh->vertices;
	Vector3 com = bvh->computeCOM();
	Matrix3X vToCOM(3, bvh->num_vertices);
	for (int i = 0; i < bvh->num_vertices; ++i) {
		vToCOM.col(i) << verts[i] - com;
	}
	Matrix3 pcaMat = vToCOM * vToCOM.transpose() / bvh->num_vertices;
	Eigen::JacobiSVD<Matrix3, Eigen::ComputeFullU> svd(pcaMat,
	                                                   Eigen::ComputeFullU);
	Matrix3 uT = svd.matrixU().transpose();
	Matrix3X localVerts = uT * vToCOM;
	Vector3 boxSize =
		localVerts.rowwise().maxCoeff() - localVerts.rowwise().minCoeff();
	return boxSize.minCoeff();
}
std::pair<FloatType, Vector3> Mesh::signedDistance(
	Eigen::Ref<const Vector3> q) const {
	// Convert q to local coordinates.
	Isometry trans = this->getRigidTransMatrix();
	Isometry invTrans = trans.inverse();
	Vector3 localQ = (invTrans * q.homogeneous()).head(3);

	// Calculate the signed distance.
	FloatType distSign, dist2;
	Vector3 n;
	Eigen::RowVector3<FloatType> cT;
	int idx;
	igl::signed_distance_pseudonormal(
		m_sdfQuery.aabbTree, m_sdfQuery.transformedPositions,
		*(m_sdfQuery.p_indices.lock()), m_sdfQuery.faceNormals,
		m_sdfQuery.vertNormals, m_sdfQuery.edgeNormals, m_sdfQuery.edgeMap,
		localQ.transpose(), distSign, dist2, idx, cT, n);
	FloatType signedDist = distSign * std::sqrt(dist2);

	// Find barycentric coordinates of the closest point to q on the mesh.
	const MatrixX3& V = m_sdfQuery.transformedPositions;
	const MatrixX3i& F = *(m_sdfQuery.p_indices.lock());
	std::array<int, 3> vidx = {F(idx, 0), F(idx, 1), F(idx, 2)};
	std::array<Vector3, 3> v = {V.row(vidx[0]).transpose(),
	                            V.row(vidx[1]).transpose(),
	                            V.row(vidx[2]).transpose()};
	Eigen::RowVector3<FloatType> barCoor;
	igl::barycentric_coordinates(cT, v[0].transpose(), v[1].transpose(),
	                             v[2].transpose(), barCoor);

	// Recalculate signed-distance gradient n by interpolating from the vertex
	// normals on the closest surface.
	Matrix3 vNormals;
	vNormals << m_sdfQuery.vertNormals.row(vidx[0]).transpose(),
		m_sdfQuery.vertNormals.row(vidx[1]).transpose(),
		m_sdfQuery.vertNormals.row(vidx[2]).transpose();
	n = vNormals * barCoor.transpose();
	n.normalize();

	// Convert n to global space.
	n = trans.linear() * n;
	return {signedDist, n};
}
igl::Hit Mesh::rayIntersection(Eigen::Ref<const Vector3> o,
                               Eigen::Ref<const Vector3> ray) const {
	// Convert o and ray to local coordinates.
	Isometry invTrans = getRigidTransMatrix().inverse();
	Vector3 localO = (invTrans * o.homogeneous()).head(3);
	Vector3 localRay = invTrans.linear() * ray;

	// Call libIGL ray intersection routine.
	igl::Hit hit;
	const MatrixX3& V = m_sdfQuery.transformedPositions;
	const MatrixX3i& F = *(m_sdfQuery.p_indices.lock());
	bool didHit = m_sdfQuery.aabbTree.intersect_ray(V, F, localO.transpose(),
	                                                localRay.transpose(), hit);
	if (!didHit) {
		hit.t = std::numeric_limits<FloatType>::infinity();
	}
	return hit;
}

std::string Mesh::toString() const {
	std::stringstream ss;
	ss << "Type: mesh\n"
	   << getVertCount() << " vertices\n"
	   << getIndexCount() << " triangles\n"
	   << RigidObjectBase::toString();
	return ss.str();
}
}  // namespace CollisionObject
}  // namespace FrictionFrenzy
