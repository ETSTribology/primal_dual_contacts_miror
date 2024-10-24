#include <boost/circular_buffer.hpp>
#include "CollisionObject/Mesh.h"
#include "ContactGeneratorBase.h"

namespace FrictionFrenzy {
namespace Contact {
void MeshMeshContactGenerator::operator()(
	const std::vector<fcl::Contact<FloatType>>& contacts,
	const CollisionObject::RigidObjectBase* pObj0,
	const CollisionObject::RigidObjectBase* pObj1,
	std::vector<ContactInformation>& out) const {
	// Declare usefule data structures;
	enum ContactQueryState { PUSHED, OUTSIDE, VISITED };
	struct ContactQueryResult {
		ContactQueryState state;
		FloatType depth;
	};
	typedef std::queue<int, boost::circular_buffer<int>> queue;
	using CollisionObject::Mesh;

	// Cast collision object pointers.
	auto* meshPtr0 = dynamic_cast<const Mesh*>(pObj0);
	auto* meshPtr1 = dynamic_cast<const Mesh*>(pObj1);
	std::vector<fcl::Contact<FloatType>> toPush;

	// Calculate the global position of vertex *vert* of a collision object.
	auto globalVertPosition = [](int vert, const Mesh* queryPtr) {
		Vector3 localPos =
			queryPtr->sdfQuery().transformedPositions.row(vert).transpose();
		Matrix4 trans = queryPtr->getRigidTransMatrix().matrix();
		Vector3 globalPos = (trans * localPos.homogeneous()).head(3);
		return globalPos;
	};

	// Helper function to push vertices of queryPtr inside compPtr into the
	// queue for BFS.
	auto pushVertexContact = [globalVertPosition, &meshPtr0, &meshPtr1,
	                          &toPush](int vert, int qObj, const Mesh* queryPtr,
	                                   const Mesh* compPtr, queue& q,
	                                   std::unordered_set<int>& visited,
	                                   std::unordered_set<int>& inserted) {
		ContactQueryResult res;
		if (!visited.count(vert)) {
			Vector3 globalPos = globalVertPosition(vert, queryPtr);
			Vector3 n;
			FloatType dist;
			std::tie(dist, n) = compPtr->signedDistance(globalPos);
			if (dist < 0) {
				fcl::Contact<FloatType> c;
				int sign = qObj ? 1 : -1;
				c.o1 = meshPtr0->getFCLObject()->collisionGeometry().get();
				c.o2 = meshPtr1->getFCLObject()->collisionGeometry().get();
				c.pos = globalPos;
				c.penetration_depth = -dist;
				c.normal = sign * n;

				toPush.push_back(c);
				q.push(vert);
				inserted.insert(vert);
				res = {ContactQueryState::PUSHED, dist};
			} else {
				res = {ContactQueryState::OUTSIDE, dist};
			}
			visited.insert(vert);
		} else {
			res = {ContactQueryState::VISITED,
			       std::numeric_limits<FloatType>::infinity()};
		}
		return res;
	};

	std::array<std::unordered_set<int>, 2> visitedVerts;
	std::array<std::unordered_set<int>, 2> insertedVerts;
	std::array<const MatrixX3i*, 2> triIdx = {
		meshPtr0->getCollisionMesh()->indices_Eigen.get(),
		meshPtr1->getCollisionMesh()->indices_Eigen.get()};
	std::array<queue, 2> vertQ;
	std::array<std::array<const Mesh*, 2>, 2> meshPtrs;
	meshPtrs[0] = {meshPtr0, meshPtr1};
	meshPtrs[1] = {meshPtr1, meshPtr0};

	// For each pair of intersecting triangles, insert intersecting vertices
	// into respective queues.
	for (size_t i = 0; i < contacts.size(); ++i) {
		for (int qObj = 0; qObj < 2; ++qObj) {
			int elem = (qObj == 0) ? contacts[i].b1 : contacts[i].b2;
			for (int j = 0; j < 3; ++j) {
				int vert = (*triIdx[qObj])(elem, j);
				ContactQueryResult res = pushVertexContact(
					vert, qObj, meshPtrs[qObj][0], meshPtrs[qObj][1],
					vertQ[qObj], visitedVerts[qObj], insertedVerts[qObj]);
				if (res.state == ContactQueryState::PUSHED ||
				    (res.state == ContactQueryState::VISITED &&
				     insertedVerts[qObj].count(vert))) {
				}
			}
		}
	}

	// For each queue, perform BFS to get all intersecting vertices.
	for (int qObj = 0; qObj < 2; ++qObj) {
		while (!vertQ[qObj].empty()) {
			int currVert = vertQ[qObj].front();
			vertQ[qObj].pop();
			const auto& adjMap = meshPtrs[qObj][0]->sdfQuery().vertAdjMap;
			int offset = adjMap[currVert], next = adjMap[currVert + 1];
			for (size_t j = offset; j < next; ++j) {
				int vert = adjMap[j];
				pushVertexContact(vert, (qObj == 0) ? -1 : 1, meshPtrs[qObj][0],
				                  meshPtrs[qObj][1], vertQ[qObj],
				                  visitedVerts[qObj], insertedVerts[qObj]);
			}
		}
	}

	for (size_t i = 0; i < toPush.size(); ++i) {
		out.push_back(Contact::convertContact(toPush[i], pObj0, pObj1));
	}
}
}  // namespace Contact
}  // namespace FrictionFrenzy
