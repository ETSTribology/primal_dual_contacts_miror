#pragma once

#include <Corrade/Utility/Debug.h>

#include "../ForceSolverBase.h"
#include "Common/MatrixTypes.h"
#include "Common/Timer.h"
#include "NonSmoothForces/NonSmoothForceBase.h"

namespace FrictionFrenzy {
namespace Solver {
class PrimalDualForceSolver : public ForceSolver {
   public:
	/**
	 * Constructor
	 */
	PrimalDualForceSolver(std::shared_ptr<unsigned int> logging)
		: ForceSolver(logging) {}

	/* Inherited from `Configurable` */
	void fillFromYAML(const YAML::Node& node) override;

	/* Inherited from `Configurable` */
	void showImGUIMenu() override;

	/* Inherited from `ForceSolver` */
	void reset() override {
		m_forceProcessTimer.Reset();
		m_systemBuildTimer.Reset();
		m_systemSolveTimer.Reset();
		m_lineSearchTimer.Reset();
		m_totalIterations = 0;
	}

	/* Inherited from `ForceSolver` */
	ForceSolverType getType() override { return ForceSolverType::PrimalDual; }

	/* Inherited from `ForceSolver` */
	void fillForces(
		const FloatType timestep,
		const Vector3& gravity,
		std::vector<ContactInformation>& contactInfo,
		std::vector<std::unique_ptr<RigidObjectBase>>& objects) override;

	/* Inherited from `ForceSolver` */
	std::string printTimings(size_t totalSteps) override;

   protected:
	/**
	 * Struct for line search results after Newton solves.
	 */
	struct LineSearchResult {
		VectorX newU;
		VectorX newF;
		VectorX newL;
		FloatType stepSize;
		FloatType sErr;
		FloatType acErr;
	};

	/**
	 * @brief Internal implementation of force computation.
	 *
	 * @param[in] timestep: The timestep size
	 * @param[in] gravity: The acceleration from gravity
	 * @param[in, out] contactInfo: The list of contacts
	 * @param[in, out] objects: The list of objects.
	 */
	void fillForcesIntl(const FloatType timestep,
	                    const Vector3& gravity,
	                    std::vector<ContactInformation>& contactInfo,
	                    std::vector<RigidObjectBase*>& pObjects);

	/**
	 * @brief Return the characteristic mass and speed.
	 *
	 * @param[in] timestep: The current timestep size.
	 * @param[in] gravity: The acceleration from gravity.
	 * @param[in] contactInfo: The list of contacts.
	 * @param[in] pObjects: The list of rigid body objects (pointers)
	 *
	 * @return A tuple containing (in order) the characteristic mass and speed.
	 */
	std::tuple<FloatType, FloatType> nonDimensionalParams(
		const FloatType timestep,
		const Vector3& gravity,
		const std::vector<ContactInformation>& contactInfo,
		const std::vector<RigidObjectBase*>& pObjects);

	/**
	 * @brief Calculate residuals.
	 */
	template <typename D>
	void calculateResiduals(bool useStoredConstraints,
	                        const std::vector<ContactInformation>& contactInfo,
	                        const std::vector<RigidObjectBase*>& pObjects,
	                        const Eigen::MatrixBase<D>& vels,
	                        const Eigen::MatrixBase<D>& forces,
	                        const Eigen::MatrixBase<D>& lambda,
	                        const Eigen::MatrixBase<D>& velsOld,
	                        Eigen::Ref<const SparseMat> massMat,
	                        const FloatType charMass,
	                        const FloatType charSpeed,
	                        const FloatType timestep,
	                        const FloatType mu,
	                        Eigen::MatrixBase<D>& ru,
	                        Eigen::MatrixBase<D>& rf,
	                        Eigen::MatrixBase<D>& wrf,
	                        Eigen::MatrixBase<D>& rl);

	/**
	 * @brief Given a line search direction, find a suitable step size and
	 * projected step direction.
	 */
	LineSearchResult lineSearch(
		const std::vector<ContactInformation>& contactInfo,
		const std::vector<RigidObjectBase*>& pObjects,
		const SparseMat& massMat,
		const VectorX& vels,
		const VectorX& forces,
		const VectorX& lambda,
		const VectorX& du,
		const VectorX& df,
		const VectorX& dl,
		const VectorX& velsOld,
		const FloatType sErr,
		const FloatType sacErr,
		const FloatType mu,
		const FloatType timestep,
		const FloatType charSpeed,
		const FloatType charMass,
		const int maxLineSearch);

	/**
	 * @brief Calculate the residual error for line searching, given the
	 * residuals.
	 */
	FloatType searchError(Eigen::Ref<const VectorX> ru,
	                      Eigen::Ref<const VectorX> wrf,
	                      Eigen::Ref<const VectorX> rl) {
		return (ru.squaredNorm() + wrf.squaredNorm() + rl.squaredNorm());
	}

	/**
	 * @brief Calculate the residual error for line searching, given the
	 * residuals and Alart-Cournier errors.
	 */
	FloatType searchACError(Eigen::Ref<const VectorX> ru,
	                        Eigen::Ref<const VectorX> ac,
	                        Eigen::Ref<const VectorX> rl) {
		return (ru.squaredNorm() + ac.squaredNorm() + rl.squaredNorm());
	}

	int m_maxIter = 100;
	FloatType m_tol = 1e-4;

	Timer m_forceProcessTimer;
	Timer m_systemBuildTimer;
	Timer m_systemSolveTimer;
	Timer m_lineSearchTimer;
	size_t m_totalIterations = 0;

	std::unique_ptr<NonSmoothForceBase> mp_nonSmoothForce;
};
}  // namespace Solver
}  // namespace FrictionFrenzy
