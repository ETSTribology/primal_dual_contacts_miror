#include "FileLoader.h"
#include <Magnum/Magnum.h>
#include <stdexcept>
#include "CollisionObject/CollisionObject.h"
#include "Common/InputParse.h"
#include "Common/MatrixTypes.h"
#include "Common/RNG.h"
#include "Drawables/ColoredDrawable.h"
#include "Dynamics/SimulationScene.h"
#include "Magnum/EigenIntegration/Integration.h"

namespace FrictionFrenzy {
namespace IO {
void FileLoader::loadFile(Dynamics::SimulationSceneCLI& sScene,
                          const std::string& fileName,
                          bool GUI) {
	YAML::Node rootNode = YAML::LoadFile(fileName);
	YAML::Node yamlNode = rootNode["scene"];
	std::filesystem::path path(fileName);
	std::cout << path << std::endl;
	sScene.clearAll();
	loadScene(sScene, yamlNode, path, GUI);
	sScene.dynamicSystem().fillFromYAML(rootNode["simulation"]);

	sScene.bakingTime() =
		parseScalar<FloatType>(rootNode["baking"], "time", 10.0, "baking");
	sScene.dynamicSystem().renderCollisionGeometry() = parseScalar<bool>(
		rootNode["baking"], "output_collision_geometry", true, "baking");
	loadLoggingOptions(sScene, rootNode);
}
void FileLoader::loadLoggingOptions(Dynamics::SimulationSceneCLI& sScene,
                                    const YAML::Node& node) {
	if (node["logging"].IsDefined()) {
		std::string logOp = node["logging"].as<std::string>();
		if (logOp == "all") {
			*sScene.mp_logging = 0 - 1;
		} else if (logOp == "none") {
			*sScene.mp_logging = 0;
		}
	}
}
void FileLoader::loadScene(Dynamics::SimulationSceneCLI& sScene,
                           const YAML::Node& yamlNode,
                           std::filesystem::path& path,
                           bool GUI) {
	if (yamlNode["meshes"].IsDefined()) {
		loadPrimitives<CollisionObject::Mesh>(
			sScene, yamlNode["meshes"], path,
			Magnum::Containers::Optional<Magnum::Trade::MeshData>(), GUI);
	}
	if (yamlNode["convex"].IsDefined()) {
		loadPrimitives<CollisionObject::Convex>(
			sScene, yamlNode["convex"], path,
			Magnum::Containers::Optional<Magnum::Trade::MeshData>(), GUI);
	}
	if (yamlNode["cubes"].IsDefined()) {
		loadPrimitives<CollisionObject::Cube>(
			sScene, yamlNode["cubes"], path,
			Magnum::Containers::Optional<Magnum::Trade::MeshData>(
				Magnum::Primitives::cubeSolid()),
			GUI);
	}
	if (yamlNode["ellipsoids"].IsDefined()) {
		loadPrimitives<CollisionObject::Ellipsoid>(
			sScene, yamlNode["ellipsoids"], path,
			Magnum::Containers::Optional<Magnum::Trade::MeshData>(
				Magnum::Primitives::icosphereSolid(3)),
			GUI);
	}
	if (yamlNode["spheres"].IsDefined()) {
		loadPrimitives<CollisionObject::Sphere>(
			sScene, yamlNode["spheres"], path,
			Magnum::Containers::Optional<Magnum::Trade::MeshData>(
				Magnum::Primitives::icosphereSolid(3)),
			GUI);
	}

	auto& dynamics = sScene.dynamicSystem();
	dynamics.initialize();

	if (yamlNode["static"].IsDefined() && yamlNode["static"].IsSequence()) {
		const YAML::Node& sts = yamlNode["static"];
		for (size_t i = 0; i < sts.size(); ++i) {
			dynamics.getObject(sts[i].as<size_t>()).setStatic(true);
		}
	}
	if (yamlNode["initialize"].IsDefined() &&
	    yamlNode["initialize"].IsSequence()) {
		const YAML::Node& initNode = yamlNode["initialize"];
		for (size_t i = 0; i < initNode.size(); ++i) {
			const YAML::Node& obj = initNode[i];
			int id = parseScalar<int>(obj, "id", "object_init");
			auto& o = dynamics.getObject(id);
			const YAML::Node& density = obj["density"];
			const YAML::Node& velocity = obj["velocity"];
			const YAML::Node& angVel = obj["angular_velocity"];
			if (density.IsDefined()) {
				o.updateDensity(
					parseScalar<FloatType>(obj, "density", "object_init"));
			}
			if (velocity.IsDefined()) {
				o.velocity() = parseVectorEigen<FloatType, 3>(obj, "velocity",
				                                              "object_init");
			}
			if (angVel.IsDefined()) {
				o.angularVel() = parseVectorEigen<FloatType, 3>(
					obj, "angular_velocity", "object_init");
			}
		}
	}
}
template <typename T>
void FileLoader::loadPrimitives(
	Dynamics::SimulationSceneCLI& sScene,
	const YAML::Node& node,
	std::filesystem::path& path,
	const Magnum::Containers::Optional<Magnum::Trade::MeshData>& renderMeshData,
	bool GUI) {
	auto convertMagnumMeshData =
		[](const Magnum::Trade::MeshData& data,
	       std::unique_ptr<CollisionObject::CPUMeshData>& meshData) {
			meshData = std::make_unique<CollisionObject::CPUMeshData>();
			for (size_t i = 0; i < data.attributeCount(); ++i) {
				if (data.attributeName(i) ==
			        Magnum::Trade::MeshAttribute::Position) {
					const auto pArray =
						Magnum::Containers::arrayCast<1, const Magnum::Vector3,
				                                      2>(data.attribute(i));
					meshData->positions_Eigen.resize(pArray.size(), 3);
					for (size_t j = 0; j < pArray.size(); ++j) {
						meshData->positions_Eigen.row(j) << pArray[j].x(),
							pArray[j].y(), pArray[j].z();
					}
				}
			}
			auto iArrayRaw = data.indicesAsArray();
			auto iArray =
				Magnum::Containers::arrayCast<Magnum::Vector3ui>(iArrayRaw);
			meshData->indices_Eigen =
				std::make_shared<MatrixX3i>(iArray.size(), 3);
			for (size_t j = 0; j < iArray.size(); ++j) {
				(*meshData->indices_Eigen)(j, 0) = iArray[j][0];
				(*meshData->indices_Eigen)(j, 1) = iArray[j][1];
				(*meshData->indices_Eigen)(j, 2) = iArray[j][2];
			}
		};

	Magnum::PluginManager::Manager<Magnum::Trade::AbstractImporter> manager;
	Corrade::Containers::Pointer<Magnum::Trade::AbstractImporter> importer =
		manager.loadAndInstantiate("AnySceneImporter");

	Dynamics::SimulationScene* p_sSceneGUI =
		dynamic_cast<Dynamics::SimulationScene*>(&sScene);

	int collisionMeshId = -1;
	if (renderMeshData) {
		if (GUI) {
			p_sSceneGUI->addMesh(*renderMeshData);
		}
		std::unique_ptr<CollisionObject::CPUMeshData> meshData;
		convertMagnumMeshData(*renderMeshData, meshData);
		sScene.dynamicSystem().addMesh(std::move(meshData));
		collisionMeshId = sScene.dynamicSystem().nMeshes() - 1;
	}

	const int loops = node.IsSequence() ? node.size() : 1;
	for (int i = 0; i < loops; ++i) {
		std::string fileName =
			path.parent_path() / (node.IsSequence() ? node[i].as<std::string>()
		                                            : node.as<std::string>());
		Magnum::Debug{} << "loading GLTF file: " << fileName.c_str();

		if (!importer || !importer->openFile(fileName)) {
			throw std::runtime_error("Cannot open GLTF file" + fileName + "!");
		}

		Magnum::Containers::Optional<Magnum::Trade::SceneData> sceneData;
		if (!(sceneData = importer->scene(importer->defaultScene())) ||
		    !sceneData->is3D() ||
		    !sceneData->hasField(Magnum::Trade::SceneField::Parent) ||
		    !sceneData->hasField(Magnum::Trade::SceneField::Mesh)) {
			std::cerr << "Cannot load scene" << std::endl;
		}
		auto& dynamics = sScene.dynamicSystem();
		const size_t objOffset = dynamics.nObjects();
		const size_t meshOffset = dynamics.nMeshes();
		for (size_t i = 0; i != importer->meshCount(); ++i) {
			Magnum::Containers::Optional<Magnum::Trade::MeshData> meshData =
				importer->mesh(i);
			if (meshData) {
				std::unique_ptr<CollisionObject::CPUMeshData> cpuMeshData;
				convertMagnumMeshData(*meshData, cpuMeshData);
				dynamics.addMesh(std::move(cpuMeshData));
				if (!renderMeshData && GUI) {
					p_sSceneGUI->addMesh(*meshData);
				}
			}
		}
		// size_t n_objects = sceneData->mappingBound();
		UniformFloat rngHue(0, 360, time(NULL));
		UniformFloat rngSat(0.0, 0.3, time(NULL) + 1);
		UniformFloat rngVal(0.6, 1.0, time(NULL) + 10);
		for (const auto& mm : sceneData->meshesMaterialsAsArray()) {
			size_t objId = mm.first() + objOffset;
			size_t meshId = mm.second().first() + meshOffset;
			CollisionObject::CPUMeshData* meshData =
				dynamics.getMeshData(meshId);
			CollisionObject::CPUMeshData* collMeshData =
				dynamics.getMeshData(renderMeshData ? collisionMeshId : meshId);
			std::unique_ptr<CollisionObject::RigidObjectBase> obj;

			obj = std::make_unique<T>(objId, meshData, collMeshData);

			if (GUI) {
				std::unique_ptr<Magnum::RigidDrawable3D> drb;
				Magnum::Color4 c = Magnum::Color4::fromHsv(
					Magnum::ColorHsv(Magnum::Deg(rngHue.sample()),
				                     rngSat.sample(), rngVal.sample()),
					1.0);
				size_t glMeshId = (renderMeshData)
				                      ? p_sSceneGUI->nMeshes() - 1
				                      : mm.second().first() + meshOffset;
				Magnum::GL::Mesh* mesh = p_sSceneGUI->getMesh(glMeshId);
				drb = std::make_unique<Magnum::RigidDrawable3D>(
					&p_sSceneGUI->magnumScene(), p_sSceneGUI->getObjectShader(),
					mesh, obj.get(), c, p_sSceneGUI->getDrawableGroup(),
					p_sSceneGUI->lightPositions(), p_sSceneGUI->lightColors(),
					objId, Magnum::Matrix4());
				p_sSceneGUI->addDrawable(std::move(drb));
			}

			dynamics.addObject(std::move(obj));
		}
		if (GUI) {
			Magnum::Containers::Array<
				Magnum::Containers::Pair<Magnum::UnsignedInt, Magnum::Int>>
				parents = sceneData->parentsAsArray();
			for (const auto& parent : parents) {
				auto& drb =
					p_sSceneGUI->getDrawable(parent.first() + objOffset);
				drb.setParent(parent.second() == -1
				                  ? &p_sSceneGUI->magnumScene()
				                  : dynamic_cast<Magnum::Object3D*>(
										&p_sSceneGUI->getDrawable(
											parent.second() + objOffset)));
			}
		}
		for (const auto& t : sceneData->transformations3DAsArray()) {
			Matrix4 transEigen = Magnum::EigenIntegration::cast<Matrix4>(
				MagnumMatrix4(t.second()));
			dynamics.getObject(t.first() + objOffset)
				.setPermanentTrans(transEigen);
			if (GUI) {
				p_sSceneGUI->getDrawable(t.first() + objOffset)
					.updateTransformation();
			}
		}
	}
}
}  // namespace IO
}  // namespace FrictionFrenzy
