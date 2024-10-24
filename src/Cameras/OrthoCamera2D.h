#ifndef __ORTHO_CAMERA2D_H__
#define __ORTHO_CAMERA2D_H__
#include <Magnum/Magnum.h>
#include <Magnum/Math/Vector2.h>
#include <Magnum/SceneGraph/Camera.h>
#include <Magnum/SceneGraph/Object.h>
#include <Magnum/SceneGraph/Scene.h>

#include "Common/MagnumTypes.h"

namespace Magnum {
namespace Camera {
class OrthoCamera2D {
   public:
	OrthoCamera2D(Scene2D& scene,
	              Vector2i frameBufferSize,
	              Float zoom = 1,
	              Vector2 pos = {0.f, 0.f})
		: m_zoom(zoom),
		  m_pos(pos),
		  m_camera_obj(&scene),
		  m_camera(m_camera_obj),
		  m_frameBufferSize(frameBufferSize) {
		m_camera.setAspectRatioPolicy(SceneGraph::AspectRatioPolicy::Extend)
			.setProjectionMatrix(Matrix3::projection({zoom, zoom}))
			.setViewport(frameBufferSize);
		m_camera_obj.setTransformation(Matrix3::translation(pos));
	}

	void draw(SceneGraph::DrawableGroup2D& group) { m_camera.draw(group); }
	void reshape(const Vector2i& frameBufferSize) {
		m_frameBufferSize = frameBufferSize;
		m_camera.setViewport(frameBufferSize);
	}
	void update() {
		m_camera.setProjectionMatrix(Matrix3::projection({m_zoom, m_zoom}));
		m_camera_obj.setTransformation(Matrix3::translation(m_pos));
	}
	void zoom(Float delta) { m_zoom *= std::pow(2, 0.5 * delta); }
	void initTransformation(Vector2i pos) {
		m_clickInitPos = pos;
		m_pos_old = m_pos;
	}
	void move(Vector2i pos) {
		int frameMax = std::min(m_frameBufferSize.x(), m_frameBufferSize.y());
		Vector2i diff = m_clickInitPos - pos;
		Vector2 inc(diff);
		inc *= m_zoom / frameMax;
		inc.y() *= -1;
		m_pos = m_pos_old + inc;
	}
	Float m_zoom;
	Vector2 m_pos, m_pos_old;
	Object2D m_camera_obj;
	SceneGraph::Camera2D m_camera;
	Vector2i m_frameBufferSize;
	Vector2i m_clickInitPos;
};
}  // namespace Camera
}  // namespace Magnum

#endif
