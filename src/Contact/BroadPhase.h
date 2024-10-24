#pragma once

#include <fcl/fcl.h>

#include "Common/MatrixTypes.h"
namespace FrictionFrenzy {
namespace Contact {
/**
 * A wrapper for FCL's broad-phase collision manager. This only registers FCL
 * objects (not RigidObjectBase!), and is mainly used to reduce compilation
 * time.
 * */
class BroadPhaseManager {
   public:
	BroadPhaseManager();
	/**
	 * @brief Collide objects in the broad-phase manager
	 */
	void collide(fcl::DefaultCollisionData<FloatType>& cData);
	/**
	 * @brief Register an FCL object.
	 */
	void registerObjects(
		std::vector<fcl::CollisionObject<FloatType>*>& m_fclObjectPointers);

	/**
	 * @brief Update an FCL object,
	 */
	void update(fcl::CollisionObject<FloatType>* pobj);
	/**
	 * @brief Update all FCL objects.
	 */
	void update();

   protected:
	std::unique_ptr<fcl::BroadPhaseCollisionManager<FloatType>> mp_fclManager;
};
}  // namespace Contact
}  // namespace FrictionFrenzy
