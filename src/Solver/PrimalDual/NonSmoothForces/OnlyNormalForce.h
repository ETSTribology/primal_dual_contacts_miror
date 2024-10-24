#pragma once
#include "NonSmoothForceBase.h"

namespace FrictionFrenzy {
namespace Solver {
class OnlyNormalForce : public NonSmoothForceBase {
   public:
	OnlyNormalForce(std::shared_ptr<unsigned int> logging)
		: NonSmoothForceBase(logging) {
	}
	void fillFromYAML(const YAML::Node& node) override;
	void showImGUIMenu() override;
	int dimensions() const override { return 1; }
	int constraintsPerForce() const override { return 1; }
	NonSmoothForceType getType() const override {
		return NonSmoothForceType::ONLY_NORMAL_FORCE;
	}
	void preprocess(const std::vector<ContactInformation>& contactInfo,
	                const std::vector<RigidObjectBase*>& objects,
	                const Eigen::Ref<const VectorX> vels,
	                const FloatType timestep,
	                const FloatType charMass,
	                const FloatType charSpeed) override;
	void initForces(VectorX& forces, VectorX& lambda) override;
	;
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

	VectorX calculateTangentialWeight(
		const std::vector<ContactInformation>& contactInfo,
		const std::vector<RigidObjectBase*>& objects,
		const Eigen::Ref<const VectorX> vels,
		const Eigen::Ref<const VectorX> forces,
		const FloatType charMass) override {
		return VectorX::Ones(forces.size());
	}
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

	FloatType m_restitution = 0.5;
	bool m_springBasedForce = false;
	FloatType m_springK = 1e4;
	FloatType m_springDamp = 0.5;

   protected:
	VectorX m_nonSmoothK;
	VectorX m_invConstraintVals;
	VectorX m_invK;
	VectorX m_sf;
	FloatType m_compliance;
	FloatType m_reduction;

	FloatType m_currAvgMass;
};
}  // namespace Solver
}  // namespace FrictionFrenzy
