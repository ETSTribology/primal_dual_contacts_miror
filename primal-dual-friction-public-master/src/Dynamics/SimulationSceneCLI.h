#pragma once

#include <memory>

#include "Common/MatrixTypes.h"
#include "DynamicSystem.h"

namespace FrictionFrenzy {
namespace Dynamics {
/**
 * A simulation scene. This includes information about baking, cameras, and
 * rendering, if applicable.
 */
class SimulationSceneCLI {
   public:
	SimulationSceneCLI();
	/**
	 * Advance one frame of the simulation. This can consist of mny substeps.
	 */
	virtual void advanceOneFrame();

	/**
	 * Clear dynamics.
	 */
	virtual void clearAll() {
		m_dynamics.clear();
		m_dynamics.clearSavedStates();
	}

	/**
	 * Access rigid body world
	 */

	const DynamicSystem& dynamicSystem() const { return m_dynamics; }

	/**
	 * Access rigid body world
	 */
	DynamicSystem& dynamicSystem() { return m_dynamics; }

	/**
	 * Access baking time.
	 */
	FloatType& bakingTime() { return m_bakingTime; }

	/**
	 * Set whether scene is baking.
	 */
	void setBaking(bool bake) { m_baking = bake; }

	/**
	 * Check if scene is baking.
	 */
	bool isBaking() { return m_baking; }

	std::shared_ptr<unsigned int> mp_logging;  /// Logging parameters.

   protected:
	DynamicSystem m_dynamics;                  /// Rigid body world.

	// Baking controls
	bool m_baking = false;
	FloatType m_bakingTime = 10.0;
};
}  // namespace Dynamics
}  // namespace FrictionFrenzy
