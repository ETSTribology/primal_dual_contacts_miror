#pragma once

#include <igl/Hit.h>
#include "Common/MatrixTypes.h"
#include "RigidObjectBase.h"

namespace FrictionFrenzy {
namespace CollisionObject {

/**
 * The structure to handle signed-distance field queries for mesh objects via
 * primarily solved by LibIGL. The AABB tree omits the need for a regular grid
 * typical in most implementations, at the cost of longer query times.
 */
class SDFQueryStruct {
   public:
	MatrixX3
		transformedPositions,  ///<  The positions of the vertices. This may be
	                           ///<  unique from object to object due to
	                           ///<  transformations, and as such need to be
	                           ///<  separate from the data stored in mp_mesh.
		faceNormals,           ///< The normals at each face.
		vertNormals,           ///< The angle-weighted vertex normals.
		edgeNormals;           ///< The averaged edge normals.
	VectorXi edgeMap;          ///< Edge map used by the AABB tree.
	VectorX vertArea;          ///< The baricentric areas of each vertex.
	igl::AABB<MatrixX3, 3> aabbTree;     ///< The AABB tree used for SDF queries
	std::weak_ptr<MatrixX3i> p_indices;  ///< The triangle indices. This is the
	                                     ///< same as stored in mp_mesh.

	// std::vector<std::vector<int>> vertAdjMap;  ///< Adjacency map of
	// vertices.
	std::vector<int>
		vertAdjMap;  ///< Adjacency map of vertices, following the format:
	                 ///< 1. The first n integers point to the index of the
	                 ///< data for vertex i in [0, n-1].
	                 ///< 2. The vertexAdjMap[n] denotes the size of the vector.
	                 ///< 3. The remainder is the data itself.
	                 ///< The number of neighbors of vertex i can then be
	                 ///< calculated as vertAdjMap[i + 1] - vertAdjMap[i].
};

/**
 A mesh collision object. The collision detection is split into two phases:
 first FCL detects triangle-triangle intersections, and then a vertex-SDF
 contact generation (See *Nonconvex RigidBodies with stacking* by Guendelman
 et al. 2003) is performed.
*/
class Mesh : public RigidObjectBase {
	typedef fcl::BVHModel<fcl::OBBRSS<FloatType>> BVH;

   public:
	Mesh(size_t id,
	     CPUMeshData* p_mesh,
	     CPUMeshData* p_collMesh,
	     Affine transformation = Affine::Identity())
		: RigidObjectBase(id, p_mesh, p_collMesh, transformation) {}
	void initializeFCLObject() override;
	std::string toString() const override;
	CollisionObject::Type getType() const override {
		return CollisionObject::Type::MESH;
	}
	FloatType obbMinLength() const override;

	/* Mesh specific functions*/
	/**
	 * @brief Find the signed distance and its gradient w.r.t. the current
	 *        object at a given point
	 * @param[in] q  the point in *global* space.
	 * @return       a pair {signed_distance, signed_distance_gradient}
	 */
	std::pair<FloatType, Vector3> signedDistance(
		Eigen::Ref<const Vector3> q) const;
	/**
	 * @brief From an origin, find an intersection for a given ray.
	 * @param[in] o    the origin in *global* space.
	 * @param[in] ray  the ray direction in *global* space.
	 * @return         The intersection information as an igl::Hit object. See
	 *                 its documentation for detail.
	 */
	igl::Hit rayIntersection(Eigen::Ref<const Vector3> o,
	                         Eigen::Ref<const Vector3> ray) const;
	/**
	 * @brief Returns the SDF query structure
	 */
	const SDFQueryStruct& sdfQuery() const { return m_sdfQuery; }
	size_t getVertCount() const { return mp_mesh->positions_Eigen.rows(); }
	size_t getIndexCount() const { return mp_mesh->indices_Eigen->rows(); }

   protected:
	std::vector<FloatType> m_faceAreas;
	SDFQueryStruct m_sdfQuery;
};
}  // namespace CollisionObject
}  // namespace FrictionFrenzy
