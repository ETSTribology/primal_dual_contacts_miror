#include "BroadPhase.h"
namespace FrictionFrenzy {
namespace Contact {
BroadPhaseManager::BroadPhaseManager() {
	mp_fclManager =
		std::make_unique<fcl::DynamicAABBTreeCollisionManager<FloatType>>();
}
void BroadPhaseManager::collide(fcl::DefaultCollisionData<FloatType>& cData) {
	mp_fclManager->collide(&cData, fcl::DefaultCollisionFunction);
}
void BroadPhaseManager::registerObjects(
	std::vector<fcl::CollisionObject<FloatType>*>& m_fclObjectPointers) {
	mp_fclManager->clear();
	mp_fclManager->registerObjects(m_fclObjectPointers);
	mp_fclManager->setup();
}
void BroadPhaseManager::update(fcl::CollisionObject<FloatType>* pobj) {
	mp_fclManager->update(pobj);
}
void BroadPhaseManager::update() {
	mp_fclManager->update();
}
}  // namespace Contact
}  // namespace FrictionFrenzy
