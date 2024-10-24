#pragma once

#include <Corrade/Containers/Array.h>
#include <Corrade/Containers/GrowableArray.h>
#include <Corrade/Containers/Optional.h>
#include <Corrade/Containers/Pair.h>
#include <Corrade/PluginManager/Manager.h>
#include <Corrade/Utility/Algorithms.h>
#include <Magnum/Math/Algorithms/GramSchmidt.h>
#include <Magnum/Math/Algorithms/Svd.h>
#include <Magnum/Trade/AbstractSceneConverter.h>
#include <Magnum/Trade/AnimationData.h>
#include <Magnum/Trade/MeshData.h>
#include <Magnum/Trade/SceneData.h>
#include <cnpy/cnpy.h>

#include "Dynamics/DynamicSystem.h"
namespace FrictionFrenzy {
namespace IO {
class FileSaver {
   public:
	void saveToFile(Dynamics::DynamicSystem& dynamics,
	                const std::string& fileName);
};
}  // namespace IO
}  // namespace FrictionFrenzy
