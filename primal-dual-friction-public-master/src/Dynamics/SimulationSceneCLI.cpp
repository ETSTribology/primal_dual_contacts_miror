#include "SimulationSceneCLI.h"
namespace FrictionFrenzy {
namespace Dynamics {
SimulationSceneCLI::SimulationSceneCLI()
	: mp_logging(new unsigned int(0)), m_dynamics(mp_logging) {
	*mp_logging = 0;
}

void SimulationSceneCLI::advanceOneFrame() {
	if (m_baking) {
		for (int i = 0; i < m_dynamics.substeps(); ++i) {
			m_dynamics.step();
		}
		m_dynamics.saveAllStates();
		if (m_bakingTime < m_dynamics.time()) {
			m_baking = false;
			std::cout << m_dynamics.printTimings() << std::endl;
		}
	}
}

}  // namespace Dynamics
}  // namespace FrictionFrenzy
