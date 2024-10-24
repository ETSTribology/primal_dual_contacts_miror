#pragma once

#include <memory>

#include "CollisionObject/RigidObjectBase.h"
#include "Common/ImGUIConfigurable.h"
#include "Common/InputParse.h"
#include "Common/MatrixTypes.h"
#include "Common/Hashing.h"
#include "Contact/ContactInfo.h"
#include "ForceConstraint/ForceConstraintBase.h"

namespace FrictionFrenzy {
namespace Solver {

enum class NonSmoothForceType {
	SECOND_ORDER_CONE,
	NON_SMOOTH_CONTACTS,
	NON_SMOOTH_CONTACTS_NEWTON,
	ONLY_NORMAL_FORCE,
	NONE
};
typedef Contact::ContactInformation ContactInformation;
typedef CollisionObject::RigidObjectBase RigidObjectBase;

/**
 * @brief Base class for a non-smooth contact force.
 */
class NonSmoothForceBase : public Configurable, public ImGUIConfigurable {
   public:
	NonSmoothForceBase(std::shared_ptr<unsigned int> logging)
		: mp_logging(logging) {}
	virtual ~NonSmoothForceBase() {}

	/**
	 * Return the number of dimensions per force.
	 */
	virtual int dimensions() const = 0;

	/**
	 * Return the number fo constraints per contact.
	 */
	virtual int constraintsPerForce() const = 0;

	/**
	 * Return the number of contacts per type.
	 */
	virtual NonSmoothForceType getType() const {
		return NonSmoothForceType::NONE;
	}

	/**
	 * @brief Initialize the forces with some default value.
	 *
	 * @param[out] forces: The vector of force unknowns
	 * @param[out] lambda: The vector of Lagrange multipliers.
	 */
	virtual void initForces(VectorX& forces, VectorX& lambda) = 0;

	/**
	 * @brief Initialization per time step.
	 *
	 * @param[in] contactInfo: List of contacts,
	 * @param[in] objects: List of rigid object pointers.
	 * @param[in] vels: List of rigid object velocites.
	 * @param[in] timestep: Time step size.
	 * @param[in] charMass: The characteristic mass.
	 * @param[in] charSpeed: The characteristic speed.
	 */
	virtual void preprocess(const std::vector<ContactInformation>& contactInfo,
	                        const std::vector<RigidObjectBase*>& objects,
	                        const Eigen::Ref<const VectorX> vels,
	                        const FloatType timestep,
	                        const FloatType charMass,
	                        const FloatType charSpeed) = 0;

	/**
	 * @brief Calculate the surrogate duality gap.
	 *
	 * @param[in]: lambda: The Lagrange multipliers.
	 */
	virtual FloatType surrogateDualGap(
		const Eigen::Ref<const VectorX> lambda) const {
		assert(lambda.size() == m_constraintVals.size());
		return -lambda.cwiseProduct(m_constraintVals).mean();
	}

	/**
	 * @brief Calculate the residuals for the Newton solve.
	 *
	 * @param[in] contactInfo: List of contacts,
	 * @param[in] objects: List of rigid object pointers.
	 * @param[in] vels: The vector of rigid object velocites.
	 * @param[in] forces: The vector of force unknowns
	 * @param[in] lambda: The vector of Lagrange multipliers.
	 * @param[in] mu: The centering parameter.
	 * @param[in] useStoredConstraints: whether to use constraint values
	 *            previously calculated and stored.
	 * @param[out] ru: The velocity residual.
	 * @param[out] rf: The force residual.
	 * @param[out] rl: The residuals of Lagrange multipliers.
	 */
	virtual void nonSmoothResiduals(
		const std::vector<ContactInformation>& contactInfo,
		const std::vector<RigidObjectBase*>& objects,
		const Eigen::Ref<const VectorX> vels,
		const Eigen::Ref<const VectorX> forces,
		const Eigen::Ref<const VectorX> lambda,
		const FloatType mu,
		const bool useStoredConstraints,
		VectorX& ru,
		VectorX& rf,
		VectorX& rl) = 0;

	/**
	 * @brief Calculate the constraint values, gradients, and Hessians.
	 *
	 * @param[in] vels: The vector of rigid object velocites.
	 * @param[in] forces: The vector of force unknowns
	 * @param[in] lambda: The vector of Lagrange multipliers.
	 */
	virtual void calculateConstraints(
		const Eigen::Ref<const VectorX> vels,
		const Eigen::Ref<const VectorX> forces,
		const Eigen::Ref<const VectorX> lambda) = 0;

	/**
	 * @brief Calculate the tangential weight.
	 */
	virtual VectorX calculateTangentialWeight(
		const std::vector<ContactInformation>& contactInfo,
		const std::vector<RigidObjectBase*>& objects,
		const Eigen::Ref<const VectorX> vels,
		const Eigen::Ref<const VectorX> forces,
		const FloatType charMass) {
		return VectorX::Ones(forces.size());
	}

	/**
	 * @brief Locate non-zero sparse matrix triplets. Since the matrix does not
	 * change structure between Newton iterations, this can be done just once
	 * per time step.
	 *
	 * @param[in] contactInfo: List of contacts,
	 * @param[in] objects: List of rigid object pointers.
	 * @param[in,out] objectPairs: Set containing an entry per contacting object
	 *                pair. This is filled during calculation.
	 * @param[in,out] triplets: Triplets locating non-zero entries in the linear
	 *                system.
	 */
	virtual void linsysReserve(
		const std::vector<ContactInformation>& contactInfo,
		const std::vector<RigidObjectBase*>& objects,
		std::unordered_set<std::pair<size_t, size_t>>& objectPairs,
		std::vector<Eigen::Triplet<FloatType>>& triplets) {
		for (size_t i = 0; i < contactInfo.size(); ++i) {
			size_t idSmaller = contactInfo[i].objId[0];
			size_t idLarger = contactInfo[i].objId[1];
			if (idSmaller > idLarger) {
				std::swap(idSmaller, idLarger);
			}
			if (objectPairs.count({idSmaller, idLarger})) {
				continue;
			}
			objectPairs.insert({idSmaller, idLarger});

			for (int j = 0; j < 2; ++j) {
				int id1 = contactInfo[i].objId[j];
				if (objects[id1]->isStatic()) {
					continue;
				}

				for (int k = 0; k < 2; ++k) {
					int id2 = contactInfo[i].objId[k];
					if (objects[id2]->isStatic()) {
						continue;
					}
					for (int row = 0; row < 6; ++row) {
						for (int col = 0; col < 6; ++col) {
							triplets.emplace_back(id1 * 6 + row, id2 * 6 + col,
							                      1);
						}
					}
				}
			}
		}
	}

	/**
	 * @brief Calculate terms in the linear system to add.
	 *
	 * @param[in] contactInfo: List of contacts,
	 * @param[in] objects: List of rigid object pointers.
	 * @param[in] lambda: The vector of Lagrange multipliers.
	 * @param[in] rf: The force residual.
	 * @param[in] rl: The residuals of Lagrange multipliers.
	 * @param[in] eps: The regularizer.
	 * @param[out] y: The addition to the right hand side.
	 * @param[out] matPointers: The pointers to matrix components
	 */
	virtual void linsysAddition(
		const std::vector<ContactInformation>& contactInfo,
		const std::vector<RigidObjectBase*>& objects,
		const Eigen::Ref<const VectorX> lambda,
		const Eigen::Ref<const VectorX> rf,
		const Eigen::Ref<const VectorX> rl,
		FloatType eps,
		VectorX& y,
		std::unordered_map<std::pair<size_t, size_t>, FloatType*>&
			matPointers) = 0;

	/**
	 * @brief Retrieve the force and multiplier increment after the linear
	 *        solve.
	 *
	 * @param[in] contactInfo: List of contacts
	 * @param[in] lambda: The vector of Lagrange multipliers.
	 * @param[in] du: The velocity increment
	 * @param[in] rf: The force residual.
	 * @param[in] rl: The residuals of Lagrange multipliers.
	 * @param[in] mu: The centering parameter.
	 * @param[out] df: The force increment.
	 * @param[out] dl: The multiplier increment.
	 */
	virtual void retrieveNonSmoothForceInc(
		const std::vector<ContactInformation>& contactInfo,
		const Eigen::Ref<const VectorX> lambda,
		const Eigen::Ref<const VectorX> du,
		const Eigen::Ref<const VectorX> rf,
		const Eigen::Ref<const VectorX> rl,
		const FloatType mu,
		VectorX& df,
		VectorX& dl) = 0;

	/**
	 * @brief Filter the line search.
	 *
	 * @param[in] contactInfo: List of contacts,
	 * @param[in] objects: List of rigid object pointers.
	 * @param[in] vels: The vector of rigid object velocites.
	 * @param[in] forces: The vector of force unknowns
	 * @param[in] lambda: The vector of Lagrange multipliers.
	 * @param[in] mu: The centering parameter.
	 * @param[in] charMass: The characteristic mass.
	 * @param[in,out] du: The velocity increment
	 * @param[in,out] df: The force increment.
	 * @param[in,out] dl: The multiplier increment.
	 */
	virtual bool filterLineSearch(
		const std::vector<ContactInformation>& contactInfo,
		const std::vector<RigidObjectBase*>& objects,
		const Eigen::Ref<const VectorX> vels,
		const Eigen::Ref<const VectorX> forces,
		const Eigen::Ref<const VectorX> lambda,
		FloatType mu,
		FloatType charMass,
		VectorX& du,
		VectorX& df,
		VectorX& dl) = 0;

	/**
	 * @brief evaluate the Alart-Cournier function.
	 *
	 * @param[in] contactInfo: List of contacts,
	 * @param[in] vels: The vector of rigid object velocites.
	 * @param[in] forces: The vector of force unknowns
	 *
	 * @return The Alart-Cournier function of each contact.
	 */
	virtual VectorX ACVector(const std::vector<ContactInformation>& contactInfo,
	                         const Eigen::Ref<const VectorX> vels,
	                         const Eigen::Ref<const VectorX> forces) const {
		return VectorX::Zero(forces.size());
	}

   protected:
	std::shared_ptr<unsigned int> mp_logging;
	std::unique_ptr<ForceConstraintBase> mp_constraint;
	VectorX m_constraintVals;
};

}  // namespace Solver
}  // namespace FrictionFrenzy
