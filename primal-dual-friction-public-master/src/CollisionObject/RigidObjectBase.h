#pragma once
#include <fcl/fcl.h>
#include <igl/AABB.h>

#include "Common/ImGUIConfigurable.h"
#include "Common/MatrixTypes.h"

namespace FrictionFrenzy {
namespace CollisionObject {
class CPUMeshData {
   public:
	CPUMeshData() {}
	MatrixX3 positions_Eigen;
	std::shared_ptr<MatrixX3i> indices_Eigen;
};

struct RigidState {
	FloatType time;
	Vector3 translation;
	Quaternion rotation;
};

/**
 * An enum class of all rigid body types
 */
enum class Type { NONE, MESH, CUBE, SPHERE, ELLIPSOID, CONVEX, N_TYPES };

/**
 * The base class for a rigid object.
 */
class RigidObjectBase : public ImGUIConfigurable {
   public:
	RigidObjectBase(size_t id,
	                CPUMeshData* p_mesh,
	                CPUMeshData* p_collMesh,
	                Affine transformation = Affine::Identity())
		: m_id(id),
		  m_permanentTrans(transformation),
		  mp_mesh(p_mesh),
		  mp_collisionMesh(p_collMesh) {
		m_translation.setZero();
		m_rotation.setIdentity();
		updateFCLObject();
	}
	virtual ~RigidObjectBase() {}
	static Isometry rigidMatrix(Eigen::Ref<const Vector3> translate,
	                            const Quaternion& quat) {
		return Eigen::Translation<FloatType, 3>(translate) * quat;
	}

	/*************************************************
	 * Virtual functions to implement for each class
	 *************************************************/
	/**
	 * @brief Expose ImGui menu to adjust positions and velocities.
	 * Overridden from ImGUIConfigurable.
	 */
	void showImGUIMenu() override;
	/**
	 * @brief Initialize the underlying FCL object.
	 * */
	virtual void initializeFCLObject() = 0;
	/**
	 * @brief Provide a brief summary of the object as a printable string.
	 *        The base implementation prints out the mass and volume.
	 * */
	virtual std::string toString() const;
	/**
	 * @brief Return the type of the rigid body. Returns NONE if undefined.
	 */
	virtual CollisionObject::Type getType() const {
		return CollisionObject::Type::NONE;
	}
	/**
	 * @brief Calculate the minimum length of the oriented bounding box.
	 *        Used for adaptive timestepping.
	 */
	virtual FloatType obbMinLength() const = 0;

	/*************************************************
	 * Accessors
	 *************************************************/
	/**
	 * @brief Get the ID of the object.
	 */
	size_t getID() const { return m_id; }
	/**
	 * @brief Get the pointer to CPU mesh data.
	 */
	const CPUMeshData* getInputMesh() const { return mp_mesh; }
	/**
	 * @brief Get the pointer to CPU mesh data.
	 */
	const CPUMeshData* getCollisionMesh() const { return mp_collisionMesh; }
	/**
	 * @brief Get the permanant transformation (the scaling and initial rigid
	 *        transformation)
	 */
	const Affine& getPermanantTransformation() const {
		return m_permanentTrans;
	}
	/**
	 * @brief Get the underlying FCL object.
	 */
	fcl::CollisionObject<FloatType>* getFCLObject() {
		return mp_collisionObject.get();
	}
	const fcl::CollisionObject<FloatType>* getFCLObject() const {
		return mp_collisionObject.get();
	}
	/**
	 * @brief Get the mass of the object.
	 */
	FloatType mass() const { return m_mass; }
	/**
	 * @brief Get the rigid transformation of the object as a 4x4 magnum matrix.
	 */
	Isometry getRigidTransMatrix() const {
		return rigidMatrix(m_translation, m_rotation);
	}
	/**
	 * @brief Calculate the transformation of the object as a 4x4 magnum matrix.
	 */
	Affine getTotalTransMatrix() const {
		return getRigidTransMatrix() * m_permanentTrans;
	}
	const Affine& getOrigMeshTransMatrix() const { return m_origMeshTrans; }
	/**
	 * @brief Get the rigid transformation of the object as a dual quaternion.
	 */
	std::pair<Vector3, Quaternion> getConfiguration() const {
		return {m_translation, m_rotation};
	}
	/**
	 * @brief Get a read-only expression of the translation.
	 */
	const Vector3& getTranslation() const { return m_translation; }
	/**
	 * @brief Get the rotation of the object as a quaternion.
	 */
	Quaternion getRotation() const { return m_rotation; }

	/**
	 * @brief Get the read-only velocity of the object.
	 */
	const Vector3& velocity() const { return m_velocity; }
	/**
	 * @brief Get the writable velocity of the object.
	 */
	Vector3& velocity() { return m_velocity; }
	/**
	 * @brief Get the read-only angular velocity of the object in axis-angle
	 *        notation.
	 */
	const Vector3& angularVel() const { return m_angularVel; }
	/**
	 * @brief Get the writable angular velocity of the object in axis-angle
	 *        notation.
	 */
	Vector3& angularVel() { return m_angularVel; }
	/**
	 * @brief Get the read-only linear acceleration of the object (multiplied by
	 *        timestep)
	 */
	const Vector3& acceleration() const { return m_accel; }
	/**
	 * @brief Get the writable linear acceleration of the object (multiplied by
	 *        timestep)
	 */
	Vector3& acceleration() { return m_accel; }
	/**
	 * @brief Get the read-only angular acceleration of the object (multiplied
	 *        by timestep)
	 */
	const Vector3& angularAcc() const { return m_angularAcc; }
	/**
	 * @brief Get the writable angular acceleration of the object (multiplied by
	 *        timestep)
	 */
	Vector3& angularAcc() { return m_angularAcc; }

	/**
	 * @brief Get the position correction of the object. (deprecated)
	 */
	Vector3& posCorrect() { return m_posCorrect; }
	/**
	 * @brief Get the orientation correction of the object. (deprecated)
	 */
	Vector3& oriCorrect() { return m_orientCorrect; }

	/**
	 * @brief Return whether object is static
	 */
	bool isStatic() const { return m_isStatic; }
	/**
	 * @brief Return whether object is active (i.e. not static)
	 */
	bool isActive() const { return !m_isStatic; }
	/**
	 * @brief Returns the velocity at the 8 corners of the AABB, for adaptive
	 *        timestepping.
	 */
	Eigen::Matrix<FloatType, 3, 8> aabbVel() const;
	/**
	 * @brief Calculate the generalized mass matrix.
	 */
	Matrix6 genMassMat() const;
	/**
	 * @brief Calculate the inverse generalized mass matrix.
	 */
	Matrix6 invGenMassMat() const;

	/**
	 * @brief Assign a permanent transformation.
	 * @param[in] trans  the new transformation matrix.
	 */
	void setPermanentTrans(Eigen::Ref<const Matrix4> trans);
	/**
	 * @brief Set the translation from a vector.
	 * @param[in] pos  the new position.
	 */
	void setTranslation(Eigen::Ref<const Vector3> pos);
	/**
	 * @brief Set the translation from a quaternion.
	 * @param[in] pos  the new position.
	 */
	void setRotation(const Quaternion& quat);
	/**
	 * @brief Simultaeneously set the position and rotation.
	 * @param[in] pos  the new position.
	 * @param[in] quat the new rotation, as a quaternion.
	 */
	void setConfiguration(Eigen::Ref<const Vector3> pos,
	                      const Quaternion& quat);
	/**
	 * @brief Simultaeneously set the position and rotation from a Eigen affine
	 *        transformation.
	 * @param[in] affine  the affine transformation.
	 */
	void setConfiguration(const Affine& affine);
	/**
	 * @brief Set whether the object is static (passive).
	 * @param[in] isStatic  the static toggle.
	 */
	void setStatic(bool isStatic) { m_isStatic = isStatic; }
	/**
	 * @brief Update the Set whether the object is sticky (has adhesion).
	 * (deprecated)
	 * @param[in] isSticky  the sticky toggle.
	 */
	void updateFCLObject();
	/**
	 * @brief Update the mass matrix and density from a new density.
	 * @param[in] newDensity  the new density.
	 */
	void updateDensity(FloatType newDensity);
	/**
	 * @brief Reset the configuration to the initial state, and set velocities
	 *        to zero.
	 */
	void reset();
	/**
	 * @brief Save the current configuration with the given time.
	 * @param[in] time  The current time.
	 */
	void saveState(FloatType time) {
		m_savedStates.push_back({time, m_translation, m_rotation});
	}
	/**
	 * @brief Clear the saved configurations.
	 */
	void clearSavedStates() { m_savedStates.clear(); }
	std::vector<RigidState>& getSavedStates() { return m_savedStates; }
	const std::vector<RigidState>& getSavedStates() const {
		return m_savedStates;
	}

   protected:
	/**
	 * @brief Initialize all configurations with the given transformations.
	 * @param[in] rigid       the rigid transformation.
	 * @param[in] scaled      the scaling of the *collision geometry*
	 * @param[in] meshScaled  the scaling of the *original mesh gemoetry*
	 */
	void initializeConfig(const Isometry& rigid,
	                      const Affine& scaled,
	                      const Affine& meshScaled);
	/**
	 * @brief Decompose the input affine matrix A into components T, R, and S
	 *        such that A = R * S.
	 *        The function assumes that the A does not flip. That is,
	 *        det(A.block<3, 3>(0, 0)) > 0
	 * @param[in]  in        the input affine transformation
	 * @param[out] rigid     an affine transformation such that
	 *                       rigid.block<3, 3>(0, 0) is an rotation matrix
	 *                       and rigid.block<3, 1>(0, 3) is a translation
	 * @param[out] scaled    a symmetric scaling matrix.
	 */
	void polarDecomposition(const Affine& in, Isometry& rigid, Affine& scaled);

	/**
	 * @brief Calculate the inertia tensor *with respect to the origin*.
	 *        For off-center meshes, make sure that the vertices are aligned
	 *        such that the center of mass is at the origin.
	 *
	 * @param[in] vertices  the mesh vertices.
	 * @param[in] indices   the mesh triangle indices.
	 * @return    the inertia tensor
	 */
	Matrix3 momentOfInertia(const std::vector<Vector3>& vertices,
	                        Eigen::Ref<const MatrixX3i> indices);

	/**
	 * @brief Given a CPUMeshData structure and scale matrix
	 *        (see polarDecomposition() ), generate new vertex positions that
	 *        1. have a center of mass at the origin
	 *        2. incorporate the scaling matrix.
	 * @param[in] mesh                  the mesh data
	 * @param[in] scaledEigen           the symmetric scaling matrix
	 * @param[out] transformedVertices  the new vertex positions.
	 * @params[out] centerOfMass        the center of mass
	 * @params[out] volume              the volume of the mesh
	 */
	void stripTransformation(const CPUMeshData& mesh,
	                         const Affine& scaledEigen,
	                         std::vector<Vector3>& transformedVertices,
	                         Eigen::Ref<Vector3> centerOfMass,
	                         FloatType& volume);
	/**
	 * @brief Preprocesse the object and its transformation, returning the
	 *        folowing:
	 * @param[out] rigidEigen     the rigid transformation of the object.
	 * @param[out] scaledEigen    the scaling of the current object.
	 * @param[out] transVertices  the vertex positions, with the center of mass
	 *                            at the origin.
	 * @param[out] vol            the volume of the object.
	 */
	void preprocessMesh(Isometry& rigidEigen,
	                    Affine& scaledEigen,
	                    std::vector<Vector3>& transVertices,
	                    FloatType& vol);
	size_t m_id;  ///< The ID of the rigid body, starting from 0. Should be the
	              ///< same as the position in m_objects.
	bool m_isStatic = false;  ///< Is the object static (passive)?

	/* Mass, density and volume */
	FloatType m_mass,     ///< The mass of the object.
		m_iMass,          ///< The inverse of the mass.
		m_density = 1.f,  ///< The density of the object.
		m_volume;         ///< The volume of the object.
	Matrix3 m_moment0,    ///< The inertia tensor at original configuration.
		m_iMoment0,  ///< The inverse inertia tensor at original configuration.
		m_moment0Vol1,   ///< The inertia tensor at original configuration,
	                     ///< assuming density of 1.
		m_iMoment0Vol1;  ///< The inverse inertia tensor at original
	                     ///< configuration, assuming density of 1.

	/* Transformations */
	Vector3 m_translation,            ///< The current translation of object.
		m_translationOrig;            ///< The initial translation of object.
	Quaternion m_rotation,            ///< The current orientation.
		m_rotationOrig;               ///< The initial orientation.
	Vector3 m_velocity = {0, 0, 0},   ///< Linear velocity
		m_angularVel = {0, 0, 0},     ///< Angular velocity
		m_accel = {0, 0, 0},          ///< Linear acceleration
		m_angularAcc = {0, 0, 0},     ///< Angular acceleration
		m_posCorrect = {0, 0, 0},     ///< Position correction (deprecated)
		m_orientCorrect = {0, 0, 0};  ///< Orientation correction (deprecated)
	Affine m_permanentTrans,  ///< The permanant scaling of the *collision
	                          ///< geometry*
		m_origMeshTrans;      ///< The permanant scaling of the *original mesh*

	/* Geometry */
	CPUMeshData* mp_mesh;  ///< The mesh from the input file.
	CPUMeshData*
		mp_collisionMesh;  ///< The mesh representation of the collision
	                       ///< geometry. (= mp_mesh for mesh-based collisions)
	std::unique_ptr<fcl::CollisionObject<FloatType>>
		mp_collisionObject;  ///< The underlying FCL collision object. (with
	                         ///< ownership)
	std::shared_ptr<fcl::CollisionGeometry<FloatType>>
		mp_geo;  ///< The underlying FCL collision geometry. (requires
	             ///< shared_ptr from FCL)

	/* Past configurations */
	std::vector<RigidState> m_savedStates;  ///< Saved configurations.
};
}  // namespace CollisionObject
}  // namespace FrictionFrenzy
