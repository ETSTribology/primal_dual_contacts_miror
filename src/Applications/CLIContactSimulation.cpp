#include <filesystem>
#include <iostream>
#include <memory>
#include <vector>

#include "Dynamics/SimulationSceneCLI.h"
#include "IO/FileLoader.h"
#include "IO/FileSaver.h"

void printHelp() {
	std::cout << "Usage: CLIContactSimulation <input_file> <output_file>\n"
	          << "Runs a CLI simulation based on the specified input file and saves results to the output file.\n\n"
	          << "Arguments:\n"
	          << "  input_file    Path to the YAML input file describing the simulation scene.\n"
	          << "  output_file   Path where the simulation output (GLTF) will be saved.\n\n"
	          << "Options:\n"
	          << "  --help        Display this help message and exit.\n";
}

int main(int argc, char* argv[]) {
	std::string m_sceneFileName = "";
	std::string m_outputFileName = "";
	FrictionFrenzy::IO::FileLoader m_fileLoader;
	FrictionFrenzy::IO::FileSaver m_fileSaver;

	FrictionFrenzy::Dynamics::SimulationSceneCLI m_scene3D;

	std::string m_collisionOutput = "";
	bool m_allset = false;

	if (argc == 2 && std::string(argv[1]) == "--help") {
		printHelp();
		return 0;
	}

	std::cout << "Available hardware threads: " << std::thread::hardware_concurrency() << std::endl;

	if (argc >= 3) {
		m_sceneFileName = std::filesystem::path(argv[1]);
		m_outputFileName = std::filesystem::path(argv[2]);
		m_fileLoader.loadFile(m_scene3D, m_sceneFileName, false);
		m_allset = true;
	} else {
		std::cerr << "Error: Missing required arguments.\n";
		printHelp();
		return 1;
	}

	if (m_allset) {
		m_scene3D.setBaking(true);
		while (m_scene3D.isBaking()) {
			m_scene3D.advanceOneFrame();
		}
		m_fileSaver.saveToFile(m_scene3D.dynamicSystem(), m_outputFileName);
	}
	return 0;
}
