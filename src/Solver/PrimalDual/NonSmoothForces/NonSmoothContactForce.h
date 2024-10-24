#pragma once
#include "NonSmoothForceBase.h"
#include "ForceConstraint/LorentzCircleConstraint.h"

namespace FrictionFrenzy {
namespace Solver {
class NonSmoothContactForce : public NonSmoothForceBase {
   public:
	NonSmoothContactForce(std::shared_ptr<unsigned int> logging)
		: NonSmoothForceBase(logging) {
		mp_frictionCoeff = std::make_shared<FloatType>(0.5);
		mp_constraint =
			std::make_unique<LorentzCircleConstraint>(mp_frictionCoeff);
	}
	void fillFromYAML(const YAML::Node& node) override;
	void showImGUIMenu() override;
	int dimensions() const override { return 3; }
	int constraintsPerForce() const override { return 2; }
	NonSmoothForceType getType() const override {
		return NonSmoothForceType::NON_SMOOTH_CONTACTS;
	}
	void preprocess(const std::vector<ContactInformation>& contactInfo,
	                const std::vector<RigidObjectBase*>& objects,
	                const Eigen::Ref<const VectorX> vels,
	                const FloatType timestep,
	                const FloatType charMass,
	                const FloatType charSpeed) override;
	void initForces(VectorX& forces, VectorX& lambda) override;
	FloatType surrogateDualGap(
		const Eigen::Ref<const VectorX> lambda) const override {
		assert(lambda.size() == m_constraintVals.size());
		return -lambda.cwiseProduct(m_constraintVals).mean();
	}
	void calculateConstraints(const Eigen::Ref<const VectorX> vels,
	                          const Eigen::Ref<const VectorX> forces,
	                          const Eigen::Ref<const VectorX> lambda) override;
	void nonSmoothResiduals(const std::vector<ContactInformation>& contactInfo,
	                        const std::vector<RigidObjectBase*>& objects,
	                        const Eigen::Ref<const VectorX> vels,
	                        const Eigen::Ref<const VectorX> forces,
	                        const Eigen::Ref<const VectorX> lambda,
	                        const FloatType mu,
	                        const bool useStoredConstraints,
	                        VectorX& ru,
	                        VectorX& rf,
	                        VectorX& rl) override;
	void linsysAddition(const std::vector<ContactInformation>& contactInfo,
	                    const std::vector<RigidObjectBase*>& objects,
	                    const Eigen::Ref<const VectorX> lambda,
	                    const Eigen::Ref<const VectorX> rf,
	                    const Eigen::Ref<const VectorX> rl,
	                    FloatType eps,
	                    VectorX& y,
	                    std::unordered_map<std::pair<size_t, size_t>,
	                                       FloatType*>& matPointers) override;
	void retrieveNonSmoothForceInc(
		const std::vector<ContactInformation>& contactInfo,
		const Eigen::Ref<const VectorX> lambda,
		const Eigen::Ref<const VectorX> du,
		const Eigen::Ref<const VectorX> rf,
		const Eigen::Ref<const VectorX> rl,
		const FloatType mu,
		VectorX& df,
		VectorX& dl) override;
	bool filterLineSearch(const std::vector<ContactInformation>& contactInfo,
	                      const std::vector<RigidObjectBase*>& objects,
	                      const Eigen::Ref<const VectorX> vels,
	                      const Eigen::Ref<const VectorX> forces,
	                      const Eigen::Ref<const VectorX> lambda,
	                      FloatType mu,
	                      FloatType charMass,
	                      VectorX& du,
	                      VectorX& df,
	                      VectorX& dl) override;

	VectorX ACVector(const std::vector<ContactInformation>& contactInfo,
	                 const Eigen::Ref<const VectorX> vels,
	                 const Eigen::Ref<const VectorX> forces) const override;
	VectorX calculateTangentialWeight(
		const std::vector<ContactInformation>& contactInfo,
		const std::vector<RigidObjectBase*>& objects,
		const Eigen::Ref<const VectorX> vels,
		const Eigen::Ref<const VectorX> forces,
		const FloatType charMass) override;
	FloatType m_restitution = 0.0;
	std::shared_ptr<FloatType> mp_frictionCoeff;
	bool m_springBasedForce = false;
	FloatType m_springK = 1e4;
	FloatType m_springDamp = 0.5;

   protected:
	MatrixX2 m_gradients;
	MatrixX2 m_hessians;
	VectorX m_nonSmoothK;
	VectorX m_invConstraintVals;
	VectorX m_KnormInv;
	VectorX m_sf;
	FloatType m_compliance;
	FloatType m_reduction;
	std::vector<Matrix3> m_WeightMat;
	std::vector<Eigen::JacobiSVD<Matrix2>> m_KmatSVD;

	FloatType m_currAvgMass;
};
}  // namespace Solver
}  // namespace FrictionFrenzy
