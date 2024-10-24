#ifndef __DYNAMIC_SYSTEM_H__
#define __DYNAMIC_SYSTEM_H__

#include <Magnum/Trade/MeshData.h>

#include <Eigen/Sparse>
#include <memory>

#include "CollisionObject/RigidObjectBase.h"
#include "Common/ImGUIConfigurable.h"
#include "Common/InputParse.h"
#include "Common/Timer.h"
#include "Contact/BroadPhase.h"
#include "Contact/ContactGenerator/ContactGeneratorBase.h"
#include "Contact/ContactInfo.h"
#include "Solver/ForceSolverBase.h"

namespace FrictionFrenzy {
namespace Dynamics {
using CollisionObject::RigidObjectBase;
using Contact::ContactInformation;

/**
 * The rigid body world for the simulation
 */
class DynamicSystem : public Configurable, public ImGUIConfigurable {
   public:
	DynamicSystem(std::shared_ptr<unsigned int> logging);

	/**
	 * Inherited from Configurable
	 */
	void fillFromYAML(const YAML::Node& node) override;

	/**
	 * Inherited from ImGUIConfigurable
	 */
	void showImGUIMenu() override;

	/**
	 * Show collisions in ImGUI, exclusive to GUI.
	 */
	void showImGUICollisionMenu();

	/**
	 * Initialize the rigid body world, after all rigid objects are imported.
	 */
	void initialize();

	/**
	 * Clear rigid body world. Used when reloading.
	 */
	void clear();

	/**
	 * Clear saved states of all rigid bodies.
	 */
	void clearSavedStates();

	/**
	 * Reset the rigid body world.
	 */
	void reset();

	/**
	 * Add mesh to the rigid body world.
	 */
	void addMesh(std::unique_ptr<CollisionObject::CPUMeshData> meshData) {
		m_meshData.push_back(std::move(meshData));
	}

	/**
	 * Add rigid body.
	 */
	void addObject(std::unique_ptr<RigidObjectBase> p_object);

	/**
	 * Update object after modifying its configuration
	 *
	 * @param[in] id The id of the object to object.
	 */
	void updateObject(size_t id);

	/**
	 * Update all rigid bodies after modifying their configurations.
	 */
	void updateAllObjects();

	/**
	 * Save the configurations of all rigid bodies.
	 */
	void saveAllStates();

	/**
	 * Detect intersections in rigid bodies and fill m_contactInfo.
	 */
	void contactDetection();

	/**
	 * Print the timing of the simulation
	 *
	 * @return the printed string
	 */
	std::string printTimings() const;

	/**
	 * Perform one time step
	 */
	void step();

	/**
	 * Solve the contact forces and object velocities.
	 *
	 * @param[in] timestep: The discrete time step. Can differ from m_timestep
	 * due to adaptive timestepping.
	 */
	void fillForces(FloatType timestep);

	/**
	 * Get an estimate of a bounding sphere of the current scene. Used for the camera in GUI.
	 *
	 * @return The origin (Vector3) and radius (FloatType) of the bounding sphere
	 */
	std::tuple<Vector3, FloatType> getApproxBoundingSphere();

	// Accessors
	
	/**
	 * Access a mesh
	 *
	 * @param[in] id The mesh ID to access
	 */
	CollisionObject::CPUMeshData* getMeshData(size_t id) {
		return m_meshData[id].get();
	}

	/**
	 * Return number of meshes
	 */
	size_t nMeshes() { return m_meshData.size(); }
	
	/**
	 * Access an object
	 *
	 * @param[in] id The object ID to access
	 */
	RigidObjectBase& getObject(size_t id) { return *m_objects[id]; }
	
	/**
	 * Access an object
	 *
	 * @param[in] id The object ID to access
	 */
	const RigidObjectBase& getObject(size_t id) const { return *m_objects[id]; }

	/**
	 * Return number of objects
	 */
	size_t nObjects() const { return m_objects.size(); }

	/**
	 * Access contact data.
	 * @return the contact vector.
	 */
	const std::vector<ContactInformation>& contactInfo() const {
		return m_contactInfo;
	}

	/**
	 * Return number of contacts
	 * @return the number fo contacts.
	 */
	size_t nContacts() const { return m_contactInfo.size(); }

	/**
	 * Return the current simulation time.
	 * @return the current simulation time.
	 */
	FloatType& time() { return m_time; }

	/**
	 * Return the number of substeps
	 * @return the number fo substeps.
	 */
	int substeps() { return m_substeps; }

	/**
	 * Access m_renderCollisionGeometry.
	 */
	bool renderCollisionGeometry() const { return m_renderCollisionGeometry; }

	/**
	 * Access m_renderCollisionGeometry.
	 */
	bool& renderCollisionGeometry() { return m_renderCollisionGeometry; }

   private:
	void generateFCLContacts(std::vector<fcl::Contact<FloatType>>& contacts);
	void addPairwiseObjectContacts(std::vector<ContactInformation>& out,
	                       std::vector<fcl::Contact<FloatType>>& in,
	                       int begin,
	                       int end);
	void generateContacts(std::vector<ContactInformation>& out,
	                  std::vector<fcl::Contact<FloatType>>& in);

	/* Member variables */

	/* Rigid objects and their wrappers */

	/// List of all contacts
	std::vector<ContactInformation> m_contactInfo;

	/// List of all rigid objects
	std::vector<std::unique_ptr<RigidObjectBase>> m_objects;

	/// List of all FCL objects
	std::vector<fcl::CollisionObject<FloatType>*> m_fclObjectPointers;

	/// List of all meshes
	std::vector<std::unique_ptr<CollisionObject::CPUMeshData>> m_meshData;

	/// Map from FCL collision geometry to internal object ID.
	std::unordered_map<const fcl::CollisionGeometry<FloatType>*, size_t>
		m_pointerToObjectId;

	/// Wrapper around FCL's broad phase collision manager.
	Contact::BroadPhaseManager m_broadPhase;

	/// Handler matrix to convert FCL contacts to our representation.
	std::array<std::array<std::unique_ptr<Contact::ContactGeneratorBase>,
	                      static_cast<size_t>(CollisionObject::Type::N_TYPES)>,
	           static_cast<size_t>(CollisionObject::Type::N_TYPES)>
		m_contactGeneratorMatrix;

	/// Contact force solver.
	std::unique_ptr<Solver::ForceSolver> mp_forceSolver;

	/// Logging options
	std::shared_ptr<unsigned int> mp_logging;

	Vector3 m_gravity = {0.0, -9.81, 0.0};
	FloatType m_time = 0.;

	FloatType m_timestep = 1e-2;
	int m_substeps = 1;

	bool m_adaptiveTimestep;
	bool m_renderCollisionGeometry = true;

	FloatType m_minObjLength;
	FloatType m_substepFactor;

	Timer m_contactDetectionTimer;
	Timer m_contactMergeTimer;
	FloatType m_totalTime = 0;

	unsigned long long m_totalContacts = 0;
	unsigned long long m_totalContactsBeforeMerge = 0;
	size_t m_totalSteps = 0;
	size_t m_totalSubsteps = 0;
	size_t m_totalContactSubsteps = 0;
};
}  // namespace Dynamics
}  // namespace FrictionFrenzy
#endif
