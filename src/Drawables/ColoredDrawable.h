#ifndef __COLORED_DRAWABLE_H__
#define __COLORED_DRAWABLE_H__
#include <Corrade/Containers/GrowableArray.h>
#include <Magnum/EigenIntegration/Integration.h>
#include <Magnum/Magnum.h>
#include <Magnum/Math/Color.h>
#include <Magnum/SceneGraph/Camera.h>
#include <Magnum/SceneGraph/Drawable.h>
#include <Magnum/SceneGraph/MatrixTransformation2D.h>
#include <Magnum/SceneGraph/MatrixTransformation3D.h>
#include <Magnum/SceneGraph/Scene.h>
#include <Magnum/SceneGraph/SceneGraph.h>
#include <Magnum/Shaders/FlatGL.h>
#include <Magnum/Shaders/PhongGL.h>

#include <memory>

#include "CollisionObject/RigidObjectBase.h"
#include "Common/MagnumTypes.h"
#include "Common/MatrixTypes.h"

namespace Magnum {


class ColoredDrawable3D : public SceneGraph::Drawable3D {
   public:
	explicit ColoredDrawable3D(Object3D& object,
	                           Shaders::PhongGL* shader,
	                           GL::Mesh* mesh,
	                           const Color4& color,
	                           SceneGraph::DrawableGroup3D& group,
	                           const std::vector<Vector4>& lightPositions,
	                           const std::vector<Color3>& lightColors,
	                           Matrix4 transformation = Matrix4())
		: SceneGraph::Drawable3D{object, &group},
		  m_transformation(transformation),
		  m_shader(shader),
		  m_mesh(mesh),
		  m_color(color),
		  m_lightPositions(lightPositions),
		  m_lightColors(lightColors) {}
	Matrix4 m_transformation;

   private:
	void draw(const Matrix4& transformationMatrix,
	          SceneGraph::Camera3D& camera) override;

   protected:
	Shaders::PhongGL* m_shader;
	GL::Mesh* m_mesh;
	Color4 m_color;
	const std::vector<Vector4>& m_lightPositions;
	const std::vector<Color3>& m_lightColors;
};

class RigidDrawable3D : public Object3D, public ColoredDrawable3D {
   public:
	RigidDrawable3D(Object3D* parent,
	                Shaders::PhongGL* shader,
	                GL::Mesh* mesh,
	                FrictionFrenzy::CollisionObject::RigidObjectBase* p_object,
	                const Color4& color,
	                SceneGraph::DrawableGroup3D& group,
	                std::vector<Vector4>& lightPositions,
	                std::vector<Color3>& lightColors,
	                UnsignedInt id,
	                Matrix4 transformation = Matrix4())
		: Magnum::Object3D(),
		  Magnum::ColoredDrawable3D(*this,
	                                shader,
	                                mesh,
	                                color,
	                                group,
	                                lightPositions,
	                                lightColors,
	                                transformation),
		  m_id(id),
		  mp_object(p_object) {
		setParent(parent);
	}

	void updateTransformation() {
		FrictionFrenzy::MagnumMatrix4 mm4(
			mp_object->getTotalTransMatrix().matrix());
		m_transformation = Matrix4(mm4);
	}

   private:
	void draw(const Magnum::Matrix4& transformationMatrix,
	          Magnum::SceneGraph::Camera3D& camera) override;

   protected:
	size_t m_id;
	FrictionFrenzy::CollisionObject::RigidObjectBase* mp_object;
};

class FlatDrawable3D : public SceneGraph::Drawable3D {
   public:
	explicit FlatDrawable3D(Object3D& object,
	                        std::weak_ptr<Shaders::FlatGL3D> shader,
	                        std::weak_ptr<GL::Mesh> mesh,
	                        const Color4& color,
	                        SceneGraph::DrawableGroup3D& group,
	                        Matrix4 transformation = Matrix4())
		: SceneGraph::Drawable3D{object, &group},
		  m_transformation(transformation),
		  m_shader(shader),
		  m_mesh(mesh),
		  m_color(color) {}
	Matrix4 m_transformation;

   private:
	void draw(const Matrix4& transformationMatrix,
	          SceneGraph::Camera3D& camera) override;
	std::weak_ptr<Shaders::FlatGL3D> m_shader;
	std::weak_ptr<GL::Mesh> m_mesh;
	Color4 m_color;
};
class ColoredDrawable2D : public SceneGraph::Drawable2D {
   public:
	explicit ColoredDrawable2D(Object2D& object,
	                           std::weak_ptr<Shaders::FlatGL2D> shader,
	                           std::weak_ptr<GL::Mesh> mesh,
	                           const Color4& color,
	                           SceneGraph::DrawableGroup2D& group,
	                           Matrix3 transformation = Matrix3())
		: SceneGraph::Drawable2D(object, &group),
		  m_transformation(transformation),
		  m_shader(shader),
		  m_mesh(mesh),
		  m_color(color) {}
	Matrix3 m_transformation;

   private:
	void draw(const Matrix3& transformationMatrix,
	          SceneGraph::Camera2D& camera) override;
	std::weak_ptr<Shaders::FlatGL2D> m_shader;
	std::weak_ptr<GL::Mesh> m_mesh;
	Color4 m_color;
};

// // struct InstanceData {
// //     Matrix3 transformationMatrix;
// //     Color3 color;
// // };
// class ColoredDrawableInstanced2D : public SceneGraph::Drawable2D {
//    public:
//     ColoredDrawableInstanced2D(Object2D &object,
//                                Containers::Array<InstanceData> &instance,
//                                const Color3 &color,
//                                const Matrix3 primitiveTransformation,
//                                SceneGraph::DrawableGroup2D &drawables)
//         : SceneGraph::Drawable2D{object, &drawables},
//           m_instanceData(instance),
//           m_color{color},
//           m_transformation{primitiveTransformation} {}
//     void draw(const Matrix3 &transformation,
//               SceneGraph::Camera2D &camera) override {
//         const Matrix3 t = transformation * m_transformation;
//         Containers::arrayAppend(m_instanceData, InPlaceInit, t, m_color);
//     }
//     Containers::Array<Magnum::InstanceData> &m_instanceData;
//     Color3 m_color;
//     Matrix3 m_transformation;
// };
}  // namespace Magnum

#endif
