#include <filesystem>
#include <iostream>
#include <memory>
#include <vector>

#include "Dynamics/SimulationSceneCLI.h"
#include "IO/FileLoader.h"
#include "IO/FileSaver.h"

int main(int argc, char* argv[]) {
	std::string m_sceneFileName = "";
	std::string m_outputFileName = "";
	FrictionFrenzy::IO::FileLoader m_fileLoader;
	FrictionFrenzy::IO::FileSaver m_fileSaver;

	FrictionFrenzy::Dynamics::SimulationSceneCLI m_scene3D;

	std::string m_collisionOutput = "";
	bool m_allset = false;
	std::cout << std::thread::hardware_concurrency() << std::endl;
	if (argc >= 3) {
		m_sceneFileName = std::filesystem::path(argv[1]);
		m_outputFileName = std::filesystem::path(argv[2]);
		m_fileLoader.loadFile(m_scene3D, m_sceneFileName, false);
		m_allset = true;
	}
	if (m_allset == true) {
		m_scene3D.setBaking(true);
		while (m_scene3D.isBaking()) {
			m_scene3D.advanceOneFrame();
		}
		m_fileSaver.saveToFile(m_scene3D.dynamicSystem(), m_outputFileName);
	}
	return 0;
}
