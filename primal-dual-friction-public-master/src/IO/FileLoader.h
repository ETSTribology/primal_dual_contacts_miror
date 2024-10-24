#pragma once

#include <Corrade/Containers/Array.h>
#include <Corrade/Containers/Optional.h>
#include <Corrade/Containers/Pair.h>
#include <Corrade/PluginManager/Manager.h>
#include <Magnum/Magnum.h>
#include <Magnum/Primitives/Cube.h>
#include <Magnum/Primitives/Icosphere.h>
#include <Magnum/Trade/AbstractImporter.h>
#include <Magnum/Trade/MeshData.h>
#include <Magnum/Trade/SceneData.h>
#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <string>

#include "Dynamics/DynamicSystem.h"
#include "Dynamics/SimulationScene.h"
#include "Solver/Solver.h"
namespace FrictionFrenzy {
namespace IO {
class FileLoader {
   public:
	/**
	 * Load a YAML file
	 */
	void loadFile(Dynamics::SimulationSceneCLI& sScene,
	              const std::string& fileName,
	              bool GUI);
	void loadLoggingOptions(Dynamics::SimulationSceneCLI& sScene,
	                        const YAML::Node& node);
	void loadScene(Dynamics::SimulationSceneCLI& sScene,
	               const YAML::Node& yamlNode,
	               std::filesystem::path& path,
	               bool GUI);
	template <typename T>
	void loadPrimitives(
		Dynamics::SimulationSceneCLI& sScene,
		const YAML::Node& node,
		std::filesystem::path& path,
		const Magnum::Containers::Optional<Magnum::Trade::MeshData>&
			renderMeshData,
		bool GUI);
};
}  // namespace IO
}  // namespace FrictionFrenzy
