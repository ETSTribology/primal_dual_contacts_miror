#include "ColoredDrawable.h"

namespace Magnum {
void ColoredDrawable3D::draw(const Matrix4& transformationMatrix,
                             SceneGraph::Camera3D& camera) {
	std::vector<Vector3> lightVec{{-5.f, 2.f, 5.f}, {5.f, 2.f, 5.f}};
	(*m_shader)
		.setDiffuseColor(m_color)
		.setLightPositions(Containers::ArrayView<const Vector4>(
			m_lightPositions.data(), m_lightPositions.size()))
		.setLightColors(Containers::ArrayView<const Color3>(
			m_lightColors.data(), m_lightColors.size()))
		.setTransformationMatrix(transformationMatrix * m_transformation)
		.setNormalMatrix(
			(transformationMatrix * m_transformation).normalMatrix())
		.setProjectionMatrix(camera.projectionMatrix())
		.draw(*m_mesh);
}
void RigidDrawable3D::draw(const Magnum::Matrix4& transformationMatrix,
                           Magnum::SceneGraph::Camera3D& camera) {
	(*m_shader)
		.setObjectId(m_id + 1)
		.setDiffuseColor(m_color)
		.setLightPositions(Magnum::Containers::ArrayView<const Magnum::Vector4>(
			m_lightPositions.data(), m_lightPositions.size()))
		.setLightColors(Magnum::Containers::ArrayView<const Magnum::Color3>(
			m_lightColors.data(), m_lightColors.size()))
		.setTransformationMatrix(transformationMatrix * m_transformation)
		.setNormalMatrix(
			(transformationMatrix * m_transformation).normalMatrix())
		.setProjectionMatrix(camera.projectionMatrix())
		.draw(*m_mesh);
}
void ColoredDrawable2D::draw(const Matrix3& transformationMatrix,
                             SceneGraph::Camera2D& camera) {
	(*m_shader.lock())
		.setColor(m_color)
		.setTransformationProjectionMatrix(
			camera.projectionMatrix() * transformationMatrix * m_transformation)
		.draw(*m_mesh.lock());
}
void FlatDrawable3D::draw(const Matrix4& transformationMatrix,
                          SceneGraph::Camera3D& camera) {
	std::vector<Vector3> lightVec{{-5.f, 2.f, 5.f}, {5.f, 2.f, 5.f}};
	(*m_shader.lock())
		.setColor(m_color)
		.setTransformationProjectionMatrix(
			camera.projectionMatrix() * transformationMatrix * m_transformation)
		.draw(*(m_mesh.lock()));
}
}  // namespace Magnum
