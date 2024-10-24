#pragma once
#include "RigidObjectBase.h"

namespace FrictionFrenzy {
namespace CollisionObject {
/**
 * Box object that computes collisions with FCL's box-box intersection
 */
class Cube : public RigidObjectBase {
   public:
	Cube(size_t id,
	     CPUMeshData* p_mesh,
	     CPUMeshData* p_collMesh,
	     Affine transformation = Affine::Identity())
		: RigidObjectBase(id, p_mesh, p_collMesh, transformation) {}
	void initializeFCLObject() override;
	CollisionObject::Type getType() const override {
		return CollisionObject::Type::CUBE;
	}
	FloatType obbMinLength() const override;
	std::string toString() const override;

   protected:
	Vector3 m_sides;
};
}  // namespace CollisionObject
}  // namespace FrictionFrenzy
