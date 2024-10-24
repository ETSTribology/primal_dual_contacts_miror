#ifndef __MAGNUMTYPES_H__
#define __MAGNUMTYPES_H__

#include <Magnum/SceneGraph/Camera.h>
#include <Magnum/SceneGraph/MatrixTransformation3D.h>
#include <Magnum/SceneGraph/Scene.h>
#include "FloatType.h"

namespace Magnum {
using namespace Math::Literals;
typedef SceneGraph::Object<SceneGraph::MatrixTransformation3D> Object3D;
typedef SceneGraph::Scene<SceneGraph::MatrixTransformation3D> Scene3D;

typedef SceneGraph::Object<SceneGraph::MatrixTransformation2D> Object2D;
typedef SceneGraph::Scene<SceneGraph::MatrixTransformation2D> Scene2D;
}  // namespace Magnum
namespace FrictionFrenzy {
typedef Magnum::UnsignedInt UnsignedInt;
typedef Magnum::Byte Byte;

typedef Magnum::Math::Vector3<FloatType> MagnumVector3;
typedef Magnum::Math::Vector2<int> MagnumVector2i;
typedef Magnum::Math::Matrix3<FloatType> MagnumMatrix3;
typedef Magnum::Math::Matrix4<FloatType> MagnumMatrix4;
typedef Magnum::Math::Quaternion<FloatType> MagnumQuaternion;
typedef Magnum::Math::DualQuaternion<FloatType> DualQuaternion;
typedef Magnum::Math::Rad<FloatType> Rad;
typedef Magnum::Math::Deg<FloatType> Deg;
}  // namespace FrictionFrenzy

#endif
