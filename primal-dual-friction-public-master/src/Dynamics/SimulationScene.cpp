#include "SimulationScene.h"
#include <imgui.h>
#include "Common/LoggingOptions.h"
#include "Common/MagnumTypes.h"
#include "Common/MatrixTypes.h"
#include "Dynamics/SimulationSceneCLI.h"
#include "Magnum/EigenIntegration/Integration.h"
#include "Magnum/Magnum.h"
namespace FrictionFrenzy {
namespace Dynamics {
SimulationScene::SimulationScene() : SimulationSceneCLI() {
	using namespace Magnum::Math::Literals;

	mp_coloredShader = std::make_unique<Magnum::Shaders::PhongGL>(
		Magnum::Shaders::PhongGL::Flag::ObjectId, m_lightPositions.size());
	(*mp_coloredShader)
		.setAmbientColor(0x010101_rgbf)
		.setSpecularColor(0x010101_rgbf)
		.setShininess(2.f);
	mp_axisShader = std::make_unique<Magnum::Shaders::FlatGL3D>(
		Magnum::Shaders::FlatGL3D::Flag::VertexColor);
	(*mp_axisShader).setColor(0xffffff_rgbf);

	m_axisMesh = Magnum::MeshTools::compile(Magnum::Primitives::axis3D());
	*mp_logging = 0;
}

void SimulationScene::drawScene(Magnum::Camera::ArcBallCamera& camera) {
	camera.draw(m_drawables);
	if (m_showContacts) {
		for (size_t i = 0; i < m_dynamics.nContacts(); ++i) {
			Matrix3 rot = m_dynamics.contactInfo()[i].localFrame();
			MagnumMatrix4 t =
				MagnumMatrix4::from(
					MagnumMatrix3(rot),
					MagnumVector3(m_dynamics.contactInfo()[i].pos)) *
				MagnumMatrix4::scaling({m_contactAxisScale, m_contactAxisScale,
			                            m_contactAxisScale});
			mp_axisShader->setTransformationProjectionMatrix(
				camera.getProjection() * camera.getMatrix() *
				Magnum::Matrix4(t));
			mp_axisShader->draw(m_axisMesh);
		}
	}
}
void SimulationScene::jumpToPlaybackTime(FloatType time) {
	for (size_t i = 0; i < m_dynamics.nObjects(); ++i) {
		DualQuaternion dq = DualQuaternion(m_objTracks[i].at(time));
		dq = dq.normalized();
		Vector3 t = Magnum::EigenIntegration::cast<Vector3>(dq.translation());
		Quaternion q(dq.rotation().data());
		m_dynamics.getObject(i).setConfiguration(t, q);
	}
}
void SimulationScene::finalizeBake() {
	m_objTracks.clear();
	m_objTracks.reserve(m_dynamics.nObjects());
	m_objStatesSingle.clear();
	m_objStatesSingle.reserve(m_dynamics.nObjects());
	for (size_t i = 0; i < m_dynamics.nObjects(); ++i) {
		auto& savedStates = m_dynamics.getObject(i).getSavedStates();
		m_objStatesSingle.emplace_back();
		m_objStatesSingle.back().reserve(savedStates.size());

		// Convert object state to Magnum format
		for (auto& state : savedStates) {
			Magnum::Quaternion q;
			for (int i = 0; i < 4; ++i) {
				q.data()[i] = state.rotation.coeffs()[i];
			}
			MagnumVector3 posD(state.translation);

			m_objStatesSingle.back().emplace_back(
				Magnum::Float(state.time),
				Magnum::DualQuaternion::from(q, Magnum::Vector3(posD)));
		}
		m_objTracks.emplace_back(
			Magnum::Containers::ArrayView<Magnum::Animation::TrackView<
				Magnum::Float, Magnum::DualQuaternion>::KeyValueType>(
				m_objStatesSingle.back().data(),
				m_objStatesSingle.back().size()),
			Magnum::Animation::Interpolation::Linear);
	}
}

void SimulationScene::clearAll() {
	m_playbackTime = 0.0;
	m_baking = false;
	m_playback = false;
	m_dynamics.clearSavedStates();
	m_dynamics.clear();
	m_scene.features().clear();
	for (size_t i = m_drawables.size() - 1; !m_drawables.isEmpty(); --i) {
		m_drawables.remove(m_drawables[i]);
	}
	m_meshes.clear();
	m_rigidDrawables.clear();
	m_objTracks.clear();
	m_objStatesSingle.clear();
}
void SimulationScene::advanceOneFrame() {
	if (m_baking) {
		for (int i = 0; i < m_dynamics.substeps(); ++i) {
			m_dynamics.step();
		}
		m_dynamics.saveAllStates();
		if (m_bakingTime < m_dynamics.time()) {
			m_baking = false;
			m_objTracks.clear();
			m_objStatesSingle.clear();
			finalizeBake();
			std::cout << m_dynamics.printTimings() << std::endl;
		}
	} else if (m_simulate || m_stepSimulation) {
		for (int i = 0; i < m_dynamics.substeps(); ++i) {
			m_dynamics.step();
		}
		if (m_stepSimulation) {
			m_stepSimulation = false;
		}
	} else if (m_playback) {
		jumpToPlaybackTime(m_playbackTime);
		m_playbackTime += m_playbackTimestep;
		if (m_playbackTime > m_dynamics.time()) {
			m_playbackTime = 0;
		}
	}
	updateAllDrawables();
}

void SimulationScene::showImGUIMenu() {
	m_dynamics.showImGUIMenu();
	if (!m_rigidDrawables.empty()) {
		if (ImGui::CollapsingHeader("Simulation Playback",
		                            ImGuiTreeNodeFlags_DefaultOpen)) {
			if (!m_baking && !m_playback) {
				if (m_simulate) {
					if (ImGui::Button("Stop simulation")) {
						m_simulate = false;
					}
				} else {
					if (ImGui::Button("Start simulation")) {
						m_simulate = true;
					}
				}
				ImGui::SameLine();
				if (ImGui::Button("Step simulation")) {
					m_stepSimulation = true;
				}
				ImGui::SameLine();
				if (ImGui::Button("Reset")) {
					m_dynamics.reset();
				}
			}
		}
		if (ImGui::CollapsingHeader("Simulation logging")) {
			auto logOption = [&](const std::string& name, LoggingOptions opt) {
				bool logOn = (*mp_logging) & (unsigned int)opt;
				if (ImGui::Checkbox(name.c_str(), &logOn)) {
					setLoggingOption(mp_logging, opt, logOn);
				}
			};
			logOption("Time per step", LoggingOptions::PER_STEP_TIME);
			logOption("Error per iteration", LoggingOptions::PER_ITER_ERRORS);
			logOption("Error per step", LoggingOptions::PER_STEP_ERRORS);
			logOption("Contacts per step", LoggingOptions::PER_STEP_CONTACTS);
			logOption("Max tangential speed", LoggingOptions::PER_STEP_SPEEDS);
			logOption("Non-dimensionalizing parameters",
			          LoggingOptions::PER_STEP_CHARFACTOR);
			logOption("Equivalent friction coefficient",
			          LoggingOptions::PER_STEP_FRICTION_COEFF);
			logOption("Max penetration depth",
			          LoggingOptions::MAX_PENETRATION_DEPTH);
			logOption("Log misc", LoggingOptions::MISC_DEBUG);
		}

		if (!m_simulate) {
			if (ImGui::CollapsingHeader("Baking")) {
				ImGui::InputFloatType("Time", &m_bakingTime);
				if (!m_playback) {
					if (!m_baking) {
						if (ImGui::Button("Bake")) {
							m_baking = true;
							m_dynamics.saveAllStates();
						}
						ImGui::SameLine();
						if (ImGui::Button("Clear")) {
							m_dynamics.clearSavedStates();
							m_dynamics.time() = 0;
						}
					} else {
						if (m_dynamics.time()) {
							ImGui::Text("Time: %f / %f", m_dynamics.time(),
							            m_bakingTime);
							ImGui::Text("Press ESC to pause baking.");
						}
						if (ImGui::Button("Stop bake")) {
							m_baking = false;
							m_objTracks.clear();
							finalizeBake();
						}
					}
				}
				if (!m_simulate && !m_baking) {
					if (ImGui::CollapsingHeader("Playback")) {
						if (ImGui::SliderFloatType("Time (secs)",
						                           &m_playbackTime, 0,
						                           m_dynamics.time())) {
							jumpToPlaybackTime(m_playbackTime);
						}
						ImGui::InputFloatType("Playback timestep: (secs)",
						                      &m_playbackTimestep);
						if (!m_playback) {
							if (ImGui::Button("Play")) {
								m_playback = true;
							}
						} else {
							if (ImGui::Button("Pause")) {
								m_playback = false;
							}
						}
						ImGui::SameLine();
						if (ImGui::Button("Rewind")) {
							m_playbackTime = 0.0;
							jumpToPlaybackTime(0.0);
							updateAllDrawables();
						}
						ImGui::SameLine();
						if (ImGui::Button("Go to last")) {
							m_playbackTime = lastRecordedTime();
							jumpToPlaybackTime(m_playbackTime);
							updateAllDrawables();
						}
					}
				}
			}
		}
	}
}

}  // namespace Dynamics
}  // namespace FrictionFrenzy
