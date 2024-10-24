#include "DynamicSystem.h"
#include "CollisionObject/RigidObjectBase.h"
#include "Common/ImGUIConfigurable.h"
#include "Common/LoggingOptions.h"
#include "Common/Cores.h"
#include "Common/MatrixTypes.h"
#include "Contact/ContactGenerator/ContactGeneratorBase.h"
#include "Solver/Solver.h"

#include <imgui.h>
#include <omp.h>
#include <cmath>
#include <limits>
#include <memory>

namespace FrictionFrenzy {
namespace Dynamics {
using CollisionObject::RigidObjectBase;
DynamicSystem::DynamicSystem(std::shared_ptr<unsigned int> logging)
	: mp_logging(logging) {
	const size_t n_types = static_cast<size_t>(CollisionObject::Type::N_TYPES);
	for (size_t i = 0; i < n_types; ++i) {
		for (size_t j = 0; j < n_types; ++j) {
			if (i == static_cast<size_t>(CollisionObject::Type::MESH) &&
			    j == static_cast<size_t>(CollisionObject::Type::MESH)) {
				m_contactGeneratorMatrix[i][j] =
					std::make_unique<Contact::MeshMeshContactGenerator>();
			} else {
				m_contactGeneratorMatrix[i][j] =
					std::make_unique<Contact::ContactGeneratorBase>();
			}
		}
	}
}
void DynamicSystem::addObject(std::unique_ptr<RigidObjectBase> p_object) {
	m_objects.push_back(std::move(p_object));
}
void DynamicSystem::clear() {
	m_contactInfo.clear();
	m_pointerToObjectId.clear();
	m_fclObjectPointers.clear();
	m_objects.clear();
	m_meshData.clear();
	m_totalTime = 0;
	m_time = 0;
}
void DynamicSystem::initialize() {
	m_fclObjectPointers.clear();
	m_pointerToObjectId.clear();
	m_minObjLength = INFINITY;
	m_fclObjectPointers.resize(m_objects.size());
#pragma omp parallel for default(shared) reduction(min : m_minObjLength)
	for (size_t i = 0; i < m_objects.size(); ++i) {
		m_objects[i]->initializeFCLObject();
		m_fclObjectPointers[i] = m_objects[i]->getFCLObject();
		m_minObjLength = std::min(m_minObjLength, m_objects[i]->obbMinLength());
	}
	for (size_t i = 0; i < m_objects.size(); ++i) {
		const fcl::CollisionGeometry<FloatType>* geo =
			m_objects[i]->getFCLObject()->collisionGeometry().get();
		m_pointerToObjectId[geo] = i;
	}

	m_broadPhase.registerObjects(m_fclObjectPointers);
}
void DynamicSystem::updateObject(size_t id) {
	RigidObjectBase& obj = *m_objects[id];
	m_broadPhase.update(obj.getFCLObject());
}
void DynamicSystem::updateAllObjects() {
	m_broadPhase.update();
}
void DynamicSystem::saveAllStates() {
	for (size_t i = 0; i < m_objects.size(); ++i) {
		RigidObjectBase& obj = *m_objects[i];
		obj.saveState(m_time);
	}
}
void DynamicSystem::clearSavedStates() {
	for (size_t i = 0; i < m_objects.size(); ++i) {
		RigidObjectBase& obj = *m_objects[i];
		obj.clearSavedStates();
	}
}
std::tuple<Vector3, FloatType> DynamicSystem::getApproxBoundingSphere() {
	Vector3 minExt, maxExt;
	minExt.setConstant(std::numeric_limits<FloatType>::infinity());
	maxExt.setConstant(-std::numeric_limits<FloatType>::infinity());

	// Calculate the AABB of the current scene.
	for (size_t i = 0; i < m_fclObjectPointers.size(); ++i) {
		if (m_objects[i]->isActive()) {
			const fcl::AABB<FloatType>& aabb =
				m_fclObjectPointers[i]->getAABB();
			Vector3 c = aabb.center();
			Vector3 dims(aabb.width(), aabb.height(), aabb.depth());
			minExt = minExt.cwiseMin(c - 0.5 * dims);
			maxExt = maxExt.cwiseMax(c + 0.5 * dims);
		}
	}
	Vector3 center = 0.5 * (minExt + maxExt);
	// Set radius to AABB diagonal.
	FloatType radius = 0.5 * (maxExt - minExt).norm();
	return {center, radius};
}
void DynamicSystem::contactDetection() {
	std::vector<fcl::Contact<FloatType>> contactsRaw;

	m_contactDetectionTimer.Resume();
	generateFCLContacts(contactsRaw);
	m_totalContactsBeforeMerge += contactsRaw.size();
	m_contactMergeTimer.Resume();
	generateContacts(m_contactInfo, contactsRaw);
	m_contactMergeTimer.Pause();
	m_contactDetectionTimer.Pause();

	m_totalContacts += m_contactInfo.size();

	if (hasLoggingOption(mp_logging, LoggingOptions::PER_STEP_CONTACTS)) {
		Magnum::Debug{} << "Solving " << m_contactInfo.size() << " contacts.";
	}
}
void DynamicSystem::fillForces(FloatType timestep) {
	contactDetection();
	mp_forceSolver->fillForces(timestep, m_gravity, m_contactInfo, m_objects);
	if (!m_contactInfo.empty()) {
		m_totalContactSubsteps++;
	}
}
void DynamicSystem::step() {
	if (hasLoggingOption(mp_logging, LoggingOptions::PER_STEP_TIME)) {
		Magnum::Debug{} << "Current time: " << m_time;
	}
	FloatType timestep = m_timestep / m_substeps;

	// Helper function to update the velocity and positions of all objects.
	auto updateObjects = [&](FloatType dt) {
#pragma omp parallel for
		for (size_t i = 0; i < m_objects.size(); ++i) {
			auto* obj = m_objects[i].get();
			if (obj->isActive()) {
				obj->angularVel() += obj->angularAcc();
				obj->velocity() += obj->acceleration();
			}
			Vector3 rotInc3D(obj->angularVel() * dt + obj->oriCorrect());
			Vector3 pos = obj->getTranslation();
			Quaternion rot = obj->getRotation();
			Quaternion rotInc =
				rotInc3D.squaredNorm() > 0
					? Quaternion(
						  AngleAxis(rotInc3D.norm(), rotInc3D.normalized()))
					: Quaternion(AngleAxis(0, Vector3(1, 0, 0)));
			Vector3 posInc = obj->velocity() * dt + obj->posCorrect();
			pos += posInc;
			rot = rotInc * rot;
			obj->setConfiguration(pos, rot);
		}
		updateAllObjects();
		m_totalSubsteps++;
		m_time += dt;
	};

	Timer stepTimer;
	stepTimer.Resume();
	if (m_adaptiveTimestep) {
		FloatType currStep = 0.;
		while (currStep < 1) {
			Timer substepTimer;
			substepTimer.Resume();

			// Find the maximum (normalized) velocity of AABB of objects.
			Vector3 minComp, maxComp;
			minComp.setConstant(std::numeric_limits<FloatType>::infinity());
			maxComp.setConstant(-std::numeric_limits<FloatType>::infinity());
			for (size_t i = 0; i < m_objects.size(); ++i) {
				auto vertVels = m_objects[i]->aabbVel();
				minComp = minComp.cwiseMin(vertVels.rowwise().minCoeff());
				maxComp = maxComp.cwiseMax(vertVels.rowwise().maxCoeff());
			}
			Vector3 avgVel = 0.5 * (minComp + maxComp);
			FloatType maxVertSpeed = 0;
			for (size_t i = 0; i < m_objects.size(); ++i) {
				auto vertVels = m_objects[i]->aabbVel();
				vertVels.colwise() -= avgVel;
				FloatType maxVertSpeedObj =
					vertVels.colwise().norm().maxCoeff();
				maxVertSpeed = std::max(maxVertSpeedObj, maxVertSpeed);
			}

			// Determine maximum allowed timestep size
			FloatType recStepSize =
				m_minObjLength /
				(2 * (maxVertSpeed * timestep + 1e-8) * m_substepFactor);
			recStepSize = std::min(1 - currStep, recStepSize);
			Magnum::Debug{} << "Substep size (0-1): " << recStepSize;

			fillForces(recStepSize * timestep);
			updateObjects(recStepSize * timestep);

			currStep += recStepSize;
			substepTimer.Pause();
			if (hasLoggingOption(mp_logging, LoggingOptions::PER_STEP_TIME)) {
				Magnum::Debug{} << "Total substep calculation time: "
								<< substepTimer.GetSeconds();
			}
		}
	} else {
		fillForces(timestep);
		updateObjects(timestep);
	}
	stepTimer.Pause();
	if (hasLoggingOption(mp_logging, LoggingOptions::PER_STEP_TIME)) {
		Magnum::Debug{} << "Total step calculation time: "
						<< stepTimer.GetSeconds();
	}
	m_totalTime += stepTimer.GetSeconds();
	m_totalSteps++;
}
void DynamicSystem::generateFCLContacts(
	std::vector<fcl::Contact<FloatType>>& contacts) {
	fcl::DefaultCollisionData<FloatType> collisionData;
	collisionData.request.num_max_contacts = 1000000;
	collisionData.request.enable_contact = true;

	m_broadPhase.collide(collisionData);
	contacts.clear();
	collisionData.result.getContacts(contacts);
}
void DynamicSystem::reset() {
	mp_forceSolver->reset();
	for (size_t i = 0; i < m_objects.size(); ++i) {
		m_objects[i]->reset();
	}
	updateAllObjects();
	m_contactInfo.clear();

	m_contactDetectionTimer.Reset();
	m_contactMergeTimer.Reset();
	m_totalSteps = 0;
	m_totalSubsteps = 0;
	m_totalContactSubsteps = 0;
	m_totalContacts = 0;
	m_totalContactsBeforeMerge = 0;
	m_totalTime = 0;
}

void DynamicSystem::generateContacts(std::vector<ContactInformation>& out,
                                     std::vector<fcl::Contact<FloatType>>& in) {
	omp_set_num_threads((in.size() < 300) ? 1 : physicalProcessors());

#pragma omp parallel for
	for (size_t i = 0; i < in.size(); ++i) {
		// Order the contacts by object ID;
		if (m_pointerToObjectId[in[i].o1] > m_pointerToObjectId[in[i].o2]) {
			std::swap(in[i].o1, in[i].o2);
			std::swap(in[i].b1, in[i].b2);
			in[i].normal *= -1;
		}
	}

	// Sort the contacts lexographically by objects.
	std::sort(in.begin(), in.end(),
	          [&](const fcl::Contact<FloatType>& a,
	              const fcl::Contact<FloatType>& b) {
				  size_t ida1 = m_pointerToObjectId[a.o1];
				  size_t ida2 = m_pointerToObjectId[a.o2];
				  size_t idb1 = m_pointerToObjectId[b.o1];
				  size_t idb2 = m_pointerToObjectId[b.o2];
				  if (ida1 != idb1) {
					  return ida1 < idb1;
				  } else if (ida2 != idb2) {
					  return ida2 < idb2;
				  } else {
					  return false;
				  }
			  });
	out.clear();
	std::vector<size_t> breakPoints;
	breakPoints.push_back(0);
	for (size_t i = 0; i < in.size();) {
		size_t j = i + 1;
		for (; j < in.size(); ++j) {
			if (in[i].o1 != in[j].o1 || in[i].o2 != in[j].o2) {
				break;
			}
		}
		breakPoints.push_back(j);
		i = j;
	}

	// Convert contacts pair (of objects) by pair
#pragma omp declare reduction (merge : std::vector<ContactInformation> :\
			   omp_out.insert(omp_out.end(), omp_in.begin(), omp_in.end()))
#pragma omp parallel for reduction(merge : out)
	for (size_t i = 0; i < breakPoints.size() - 1; ++i) {
		addPairwiseObjectContacts(out, in, breakPoints[i], breakPoints[i + 1]);
	}
}
std::string DynamicSystem::printTimings() const {
	std::stringstream ss;

	ss << "Times per substep (seconds):\n\tContact Detection: "
	   << m_contactDetectionTimer.GetSeconds() / m_totalContactSubsteps
	   << "\n\tTotal time: " << m_totalTime / m_totalContactSubsteps
	   << "\nAverage #contacts (before merge): "
	   << (double)m_totalContactsBeforeMerge / m_totalContactSubsteps
	   << "\nAverage #contacts (after merge): "
	   << (double)m_totalContacts / m_totalContactSubsteps
	   << "\nAverage substeps per step: "
	   << (double)m_totalSubsteps / m_totalSteps
	   << "\nTotal #solids: " << m_objects.size() << "\n"
	   << mp_forceSolver->printTimings(m_totalContactSubsteps);

	return ss.str();
}

void DynamicSystem::addPairwiseObjectContacts(
	std::vector<ContactInformation>& out,
	std::vector<fcl::Contact<FloatType>>& in,
	int begin,
	int end) {
	std::vector<fcl::Contact<FloatType>> contacts;

	auto* oPtr = m_objects[m_pointerToObjectId[in[begin].o1]].get();
	auto* oPtr2 = m_objects[m_pointerToObjectId[in[begin].o2]].get();
	if (oPtr->isStatic() && oPtr2->isStatic()) {
		return;
	}
	if (oPtr == oPtr2) {
		return;
	}
	contacts.reserve(end - begin);
	contacts.insert(contacts.begin(), in.begin() + begin, in.begin() + end);

	// Call contact handlers to generate contacts.
	std::vector<ContactInformation> inter;
	(*m_contactGeneratorMatrix[static_cast<size_t>(oPtr->getType())]
	                          [static_cast<size_t>(oPtr2->getType())])(
		contacts, oPtr, oPtr2, inter);

	if (inter.size() > 0) {
		// Merge contacts that are too close.
		std::sort(
			inter.begin(), inter.end(),
			[&](const ContactInformation& a, const ContactInformation& b) {
				if (a.pos[0] != b.pos[0]) {
					return a.pos[0] < b.pos[0];
				} else if (a.pos[1] != b.pos[1]) {
					return a.pos[1] < b.pos[1];
				} else if (a.pos[2] != b.pos[2]) {
					return a.pos[2] < b.pos[2];
				} else {
					return false;
				}
			});
		std::vector<size_t> breakPoints;
		breakPoints.push_back(0);
		for (size_t i = 1; i < inter.size(); ++i) {
			if ((inter[i].pos - inter[i - 1].pos).squaredNorm() > 1e-8) {
				breakPoints.push_back(i);
			}
		}
		breakPoints.push_back(inter.size());
		for (size_t i = 0; i < breakPoints.size() - 1; ++i) {
			if (breakPoints[i + 1] - breakPoints[i] == 1) {
				out.push_back(inter[breakPoints[i]]);
			} else {
				ContactInformation temp = inter[breakPoints[i]];
				Vector3 normal = temp.normal();
				for (size_t j = breakPoints[i] + 1; j < breakPoints[i + 1];
				     ++j) {
					normal += inter[j].normal();
					temp.depth = std::min(inter[j].depth, temp.depth);
				}
				normal.normalize();
				normalToTransform(temp.pos, normal,
				                  m_objects[temp.objId[0]].get(),
				                  m_objects[temp.objId[1]].get(), temp);
				out.push_back(temp);
			}
		}
	}
}

void DynamicSystem::fillFromYAML(const YAML::Node& node) {
	m_timestep = parseScalar<FloatType>(node, "timestep", 1e-2, "simulation");
	if (node["solver"].IsDefined()) {
		const YAML::Node& solverNode = node["solver"];
		std::string solverName = solverNode["name"].as<std::string>();
		if (solverName == "primal_dual") {
			mp_forceSolver =
				std::make_unique<Solver::PrimalDualForceSolver>(mp_logging);
		} else {
			throw std::runtime_error("No solver of name: " + solverName +
			                         "found!");
		}
		mp_forceSolver->fillFromYAML(solverNode);
	} else {
		throw std::runtime_error("No force solver specified!");
	}
	if (node["gravity"].IsDefined()) {
		m_gravity =
			parseVectorEigen<FloatType, 3>(node, "gravity", "simulation");
	}
	m_adaptiveTimestep =
		parseScalar<bool>(node, "adaptive_timestep", false, "simulation");
	if (m_adaptiveTimestep) {
		m_substepFactor =
			parseScalar<FloatType>(node, "substep_factor", 10, "simulation");
	}
}
void DynamicSystem::showImGUIMenu() {
	if (ImGui::CollapsingHeader("Simulation Parameters")) {
		if (ImGui::SliderFloatType("Timestep", &m_timestep, 1e-4, 1e-1, "%.4f",
		                           ImGuiSliderFlags_Logarithmic)) {
		}
		ImGui::Checkbox("Adaptive timestep", &m_adaptiveTimestep);
		if (m_adaptiveTimestep) {
			ImGui::SliderFloatType("Substep Factor", &m_substepFactor, 1, 5);
		}
		ImGui::SliderInt("Substeps", &m_substeps, 1, 10);
		ImGui::InputFloatType3("Gravity", m_gravity.data(), "%.4f");

		if (mp_forceSolver) {
			mp_forceSolver->showImGUIMenu();
		}
	}
}
void DynamicSystem::showImGUICollisionMenu() {
	std::stringstream ss;
	ss << "Contacts (" << m_contactInfo.size() << " total)";
	if (ImGui::TreeNodeEx(ss.str().c_str())) {
		for (size_t i = 0; i < m_contactInfo.size(); ++i) {
			ss.str(std::string());
			ss << "Collision " << i << " (objects " << m_contactInfo[i].objId[0]
			   << ", " << m_contactInfo[i].objId[1] << ")";
			if (ImGui::CollapsingHeader(ss.str().c_str())) {
				ImGui::Text("%s", m_contactInfo[i].toString().c_str());
			}
		}
		ImGui::TreePop();
	}
}
}  // namespace Dynamics
}  // namespace FrictionFrenzy
