#include "FileSaver.h"
#include <Eigen/src/Geometry/Translation.h>
#include <Magnum/EigenIntegration/Integration.h>
#include "CollisionObject/RigidObjectBase.h"
#include "Common/MagnumTypes.h"
#include "Common/MatrixTypes.h"
#include "Dynamics/DynamicSystem.h"
#include "Magnum/Magnum.h"

#include <filesystem>
#include <fstream>

#include <Magnum/Magnum.h>
#include <Magnum/Math/Algorithms/GramSchmidt.h>
#include <Magnum/Math/Algorithms/Svd.h>

#include <zip.h>


namespace FrictionFrenzy {
namespace IO {
template <typename T>
static std::pair<Magnum::Containers::Array<char>,
                 Magnum::Containers::StridedArrayView1D<T>>
copyToCharArray(std::vector<T>& vec) {
	Magnum::Containers::ArrayView<T> arrayMagnum(vec.data(), vec.size());
	Magnum::Containers::ArrayView<char> dataChar =
		Magnum::Containers::arrayCast<char>(arrayMagnum);
	Magnum::Containers::Array<char> rawData(vec.size() * sizeof(T));
	Magnum::Utility::copy(dataChar, rawData);

	Magnum::Containers::StridedArrayView1D<T> rawDataTypeView =
		Magnum::Containers::arrayCast<T>(rawData);

	return {std::move(rawData), rawDataTypeView};
}
template <typename T>
static Magnum::Containers::Array<T> copyToMagnumArray(std::vector<T>& vec) {
	Magnum::Containers::ArrayView<T> arrayViewMagnum(vec.data(), vec.size());
	Magnum::Containers::Array<T> arrayMagnum(vec.size());
	Magnum::Utility::copy(arrayViewMagnum, arrayMagnum);
	return std::move(arrayMagnum);
}
void FileSaver::saveToFile(Dynamics::DynamicSystem& dynamics,
                           const std::string& fileName) {
	std::filesystem::path filePath(fileName);
	filePath.replace_extension("gltf");
	std::string gltfFileName = filePath.c_str();
	Magnum::PluginManager::Manager<Magnum::Trade::AbstractSceneConverter>
		manager;
	Corrade::Containers::Pointer<Magnum::Trade::AbstractSceneConverter>
		converter = manager.loadAndInstantiate("AnySceneConverter");
	converter->beginFile(gltfFileName);
	std::cout << gltfFileName << std::endl;

	char meshPrefix[] = "mesh_";

	// Meshes
	for (size_t i = 0; i < dynamics.nObjects(); ++i) {
		const auto& obj = dynamics.getObject(i);
		const auto& pMesh = dynamics.renderCollisionGeometry()
		                        ? obj.getCollisionMesh()
		                        : obj.getInputMesh();
		std::vector<Magnum::Vector3> points;
		std::vector<Magnum::UnsignedInt> indices;
		points.reserve(pMesh->positions_Eigen.rows());
		indices.reserve(pMesh->indices_Eigen->size());
		for (Eigen::Index j = 0; j < pMesh->positions_Eigen.rows(); ++j) {
			const auto& p = pMesh->positions_Eigen.row(j);
			points.emplace_back(p[0], p[1], p[2]);
		}
		for (Eigen::Index j = 0; j < pMesh->indices_Eigen->rows(); ++j) {
			const auto& t = pMesh->indices_Eigen->row(j);
			indices.push_back(t[0]);
			indices.push_back(t[1]);
			indices.push_back(t[2]);
		}
		Magnum::Containers::ArrayView<const Magnum::Vector3> pointMagnumArray(
			points.data(), points.size());
		Magnum::Containers::ArrayView<const Magnum::UnsignedInt> idxMagnum(
			indices.data(), indices.size());
		Magnum::Trade::MeshIndexData meshIndexData(
			Magnum::MeshIndexType::UnsignedInt, idxMagnum);
		Magnum::Trade::MeshData data{
			Magnum::MeshPrimitive::Triangles,
			Magnum::Trade::DataFlags{},
			idxMagnum,
			meshIndexData,
			Magnum::Trade::DataFlag::Mutable,
			pointMagnumArray,
			{Magnum::Trade::MeshAttributeData{
				Magnum::Trade::MeshAttribute::Position,
				Magnum::VertexFormat::Vector3, pointMagnumArray}}};
		char str[20];
		sprintf(str, "%s%zu", meshPrefix, i);
		converter->add(data, str);
	}

	// Objects and transformations
	struct objInfo {
		Magnum::UnsignedInt objId;
		Magnum::Int parent;
		Magnum::UnsignedInt mesh;
		Magnum::Matrix4x4 transformation;
	};
	std::vector<objInfo> objectVector;
	const size_t n_objects = dynamics.nObjects();
	objectVector.reserve(n_objects);
	for (size_t i = 0; i < n_objects; ++i) {
		const auto& pObj = dynamics.getObject(i);
		MagnumMatrix4 tMagnum(pObj.getTotalTransMatrix().matrix());
		objectVector.push_back({Magnum::UnsignedInt(i), -1,
		                        Magnum::UnsignedInt(i),
		                        Magnum::Matrix4x4(tMagnum)});
	}

	Magnum::Containers::Array<char> rawDataObj;
	Magnum::Containers::StridedArrayView1D<objInfo> rawDataObjView;
	std::tie(rawDataObj, rawDataObjView) = copyToCharArray(objectVector);

	Magnum::Trade::SceneData sceneData(
		Magnum::Trade::SceneMappingType::UnsignedInt, n_objects,
		std::move(rawDataObj),
		{Magnum::Trade::SceneFieldData{Magnum::Trade::SceneField::Parent,
	                                   rawDataObjView.slice(&objInfo::objId),
	                                   rawDataObjView.slice(&objInfo::parent)},
	     Magnum::Trade::SceneFieldData{Magnum::Trade::SceneField::Mesh,
	                                   rawDataObjView.slice(&objInfo::objId),
	                                   rawDataObjView.slice(&objInfo::mesh)},
	     Magnum::Trade::SceneFieldData{
			 Magnum::Trade::SceneField::Transformation,
			 rawDataObjView.slice(&objInfo::objId),
			 rawDataObjView.slice(&objInfo::transformation)}});
	converter->add(sceneData, "scene");
	converter->endFile();

	// Animation as zipped numpy arrays
	filePath.replace_extension("npz");
	libzip::archive zipObj(filePath, ZIP_TRUNCATE | ZIP_CREATE);
	uint64_t zipIndex = 0;
	{
		std::vector<FloatType> times;
		auto& states = dynamics.getObject(0).getSavedStates();
		times.reserve(states.size());
		for (const auto& state : states) {
			times.push_back(state.time);
		}
		zipIndex =
			cnpy::add_npy_to_zip(zipObj, "time", times.data(), {times.size()});
		// cnpy::npz_save(filePath.c_str(), "time", times.data(),
		// {times.size()},
		//                "w");
	}

#pragma omp parallel for
	for (size_t i = 0; i < n_objects; ++i) {
		const auto& pObj = dynamics.getObject(i);
		const auto& states = pObj.getSavedStates();
		const size_t stateSize = 7;
		char str[20];
		sprintf(str, "%s%zu", meshPrefix, i);

		std::vector<FloatType> timeTransRot;
		timeTransRot.reserve((states.size() + 1) * stateSize);

		Affine transMat = dynamics.renderCollisionGeometry()
		                      ? pObj.getPermanantTransformation()
		                      : pObj.getOrigMeshTransMatrix();

		MagnumVector3 scale = MagnumMatrix4(transMat.matrix()).scaling();
		timeTransRot.push_back(scale[0]);
		timeTransRot.push_back(scale[1]);
		timeTransRot.push_back(scale[2]);
		for (size_t idx = 0; idx < stateSize - 3; ++idx) {
			timeTransRot.push_back(0);
		}

		for (const auto& state : states) {
			Affine transform = CollisionObject::RigidObjectBase::rigidMatrix(
								   state.translation, state.rotation) *
			                   transMat;
			Affine rotM, scaling;
			transform.computeRotationScaling(&rotM, &scaling);

			Vector3 trans = transform.translation();
			Quaternion rot(rotM.rotation());

			for (int idx = 0; idx < 3; ++idx) {
				timeTransRot.push_back(trans[idx]);
			}
			for (int idx = 0; idx < 4; ++idx) {
				timeTransRot.push_back(rot.coeffs()[idx]);
			}
		}
#pragma omp critical
		zipIndex = cnpy::add_npy_to_zip(zipObj, str, timeTransRot.data(),
		                                {states.size() + 1, stateSize});
		// cnpy::npz_save(filePath.c_str(), str, timeTransRot.data(),
		//                {states.size() + 1, stateSize}, "a");
	}
	zipObj.set_file_compression(zipIndex, ZIP_CM_STORE);

	filePath.replace_extension("info");
	std::ofstream fileInfo(filePath);
	fileInfo << dynamics.printTimings();
}
};  // namespace IO
}  // namespace FrictionFrenzy
