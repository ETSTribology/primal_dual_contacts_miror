#pragma once
#include "RigidObjectBase.h"
namespace FrictionFrenzy {
namespace CollisionObject {

// A sphere collision object.
class Sphere : public RigidObjectBase {
   public:
	Sphere(size_t id,
	       CPUMeshData* p_mesh,
	       CPUMeshData* p_collMesh,
	       Affine transformation = Affine::Identity())
		: RigidObjectBase(id, p_mesh, p_collMesh, transformation) {}
	void initializeFCLObject() override;
	CollisionObject::Type getType() const override {
		return CollisionObject::Type::SPHERE;
	}
	FloatType obbMinLength() const override;
	std::string toString() const override;

   protected:
	FloatType m_radius;
};
}  // namespace CollisionObject
}  // namespace FrictionFrenzy
