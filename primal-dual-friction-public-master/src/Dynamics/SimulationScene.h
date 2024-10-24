#pragma once

#include <Magnum/Animation/Track.h>
#include <Magnum/GL/Mesh.h>
#include <Magnum/Magnum.h>
#include <Magnum/MeshTools/Compile.h>
#include <Magnum/Primitives/Axis.h>
#include <Magnum/SceneGraph/Drawable.h>

#include <memory>
#include <vector>

#include <Magnum/EigenIntegration/Integration.h>
#include <Magnum/Magnum.h>
#include <Magnum/SceneGraph/SceneGraph.h>
#include <Magnum/Shaders/PhongGL.h>
#include "Cameras/ArcBallCamera.h"
#include "Common/ImGUIConfigurable.h"
#include "Common/MagnumTypes.h"
#include "Common/MatrixTypes.h"
#include "Drawables/ColoredDrawable.h"
#include "Dynamics/SimulationSceneCLI.h"

namespace FrictionFrenzy {
namespace Dynamics {

using Magnum::SceneGraph::DrawableGroup3D, Magnum::Shaders::PhongGL;

/**
 * Simulation scene for the GUI program, which includes information for cameras,
 * rendered 3D meshes, and shaders.
 */
class SimulationScene : public SimulationSceneCLI, public ImGUIConfigurable {
   public:
	SimulationScene();
	/* Inherited from SimulationSceneCLI */
	void showImGUIMenu() override;
	/* Inherited from SimulationSceneCLI */
	void advanceOneFrame() override;
	/* Inherited from SimulationSceneCLI */
	void clearAll() override;

	/**
	 * Setup the Magnum animation track after the simulation ends.
	 */
	void finalizeBake();

	/**
	 * Draw the current frame
	 */
	void drawScene(Magnum::Camera::ArcBallCamera& camera);

	/**
	 * Return the last recorded time in the simulation.
	 */
	FloatType lastRecordedTime() {
		return (m_objStatesSingle[0].empty())
		           ? 0.0
		           : m_objStatesSingle[0].back().first;
	}

	/**
	 * Update all Magnum drawables.
	 */
	void updateAllDrawables() {
		for (size_t i = 0; i < m_rigidDrawables.size(); ++i) {
			m_rigidDrawables[i]->updateTransformation();
		}
	}

	/**
	 * Update a single Magnum drawables.
	 *
	 * @param[in] id: the drawable id
	 */
	void updateDrawable(size_t id) {
		m_rigidDrawables[id]->updateTransformation();
	}

	/**
	 * Jump to a specific playback time.
	 *
	 * @param[in] time: the time to jump to.
	 */
	void jumpToPlaybackTime(FloatType time);

	// Accessors

	/**
	 * Set whether scene is running.
	 */
	void setSimulate(bool sim) { m_simulate = sim; }

	/**
	 * Check if scene is running.
	 */
	bool isSimulating() { return m_simulate; }

	/**
	 * Whether contacts are show in GUI.
	 */
	bool& showContacts() { return m_showContacts; }

	/**
	 * Return scale of the contact visualization.
	 */
	FloatType& contactAxisScale() { return m_contactAxisScale; }

	/**
	 * Access Magnum scene graph data.
	 */
	Magnum::Scene3D& magnumScene() { return m_scene; }

	/**
	 * Access Phong shader for objects.
	 */
	PhongGL* getObjectShader() { return mp_coloredShader.get(); }

	/**
	 * Access light positions.
	 */
	std::vector<Magnum::Vector4>& lightPositions() { return m_lightPositions; }

	/**
	 * Access light colors.
	 */
	std::vector<Magnum::Color3>& lightColors() { return m_lightColors; }

	/**
	 * Access Magnum drawable group.
	 */
	DrawableGroup3D& getDrawableGroup() { return m_drawables; }

	/**
	 * Return number of GL meshes.
	 */
	size_t nMeshes() { return m_meshes.size(); }

	/**
	 * Access the GL mesh with ID
	 *
	 * @param[in] id: The mesh ID to access.
	 */
	Magnum::GL::Mesh* getMesh(size_t id) { return m_meshes[id].get(); }

	/**
	 * Add a GL mesh from mesh data.
	 *
	 * @params[in] meshData: the mesh data to add.
	 */
	void addMesh(const Magnum::Trade::MeshData& meshData) {
		Magnum::MeshTools::CompileFlags flags;
		m_meshes.push_back(
			std::make_unique<Magnum::GL::Mesh>(Magnum::NoCreate));
		*m_meshes.back() = Magnum::MeshTools::compile(meshData, flags);
	}

	/**
	 * Access the drawable with ID
	 *
	 * @param[in] id: The id to access.
	 */
	Magnum::RigidDrawable3D& getDrawable(size_t id) {
		return *(m_rigidDrawables[id]);
	}

	/**
	 * Add one drawable.
	 *
	 * @param[in] drb: the Magnum drawable to add.
	 */
	void addDrawable(std::unique_ptr<Magnum::RigidDrawable3D> drb) {
		m_rigidDrawables.push_back(std::move(drb));
	}

   protected:
	Magnum::Scene3D m_scene;
	DrawableGroup3D m_drawables;
	std::vector<Magnum::Vector4> m_lightPositions{{-5, 1, 5, 0},
	                                              {5, 1, 5, 0},
	                                              {2, 1, -5, 0}};
	std::vector<Magnum::Color3> m_lightColors{Magnum::Color3(1.f, 1.f, 1.f),
	                                          Magnum::Color3(0.5f, 0.5f, 0.5f),
	                                          Magnum::Color3(1, 1, 1)};
	FloatType m_playbackTimestep = 1. / 60;
	FloatType m_contactAxisScale = 1.0;
	bool m_simulate = false;
	bool m_stepSimulation = false;
	bool m_showContacts = false;

	// Playback controls
	bool m_playback = false;
	FloatType m_playbackTime = 0.0;

	/// The Magnum animation tracks for each object.
	std::vector<
		Magnum::Animation::TrackView<Magnum::Float, Magnum::DualQuaternion>>
		m_objTracks;

	/// The object configurations through time, each object occupying one
	/// element of the outer array. Magnum only interpolates quaternions with
	/// single-precision, hence the separate structure.
	std::vector<std::vector<std::pair<Magnum::Float, Magnum::DualQuaternion>>>
		m_objStatesSingle;

	std::vector<std::unique_ptr<Magnum::GL::Mesh>> m_meshes;
	std::vector<std::unique_ptr<Magnum::RigidDrawable3D>> m_rigidDrawables;
	std::unique_ptr<PhongGL> mp_coloredShader;
	std::unique_ptr<Magnum::Shaders::FlatGL3D> mp_axisShader;
	Magnum::GL::Mesh m_axisMesh;
};
}  // namespace Dynamics
}  // namespace FrictionFrenzy
