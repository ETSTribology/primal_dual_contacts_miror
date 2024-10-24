#pragma once
#include "ForceConstraintBase.h"

namespace FrictionFrenzy {
namespace Solver {
class LorentzCircleConstraint : public ForceConstraintBase {
   public:
	LorentzCircleConstraint(std::shared_ptr<FloatType> frictionCoeff)
		: mp_frictionCoeff(frictionCoeff) {}
	FloatType constraint(const Eigen::Ref<const VectorX> in) const override;
	VectorX gradient(const Eigen::Ref<const VectorX> in) const override;
	MatrixX hessian(const Eigen::Ref<const VectorX> in) const override;
	VectorX project(const Eigen::Ref<const VectorX> in,
	                const Eigen::Ref<const VectorX> start,
	                FloatType target = 0) const override;
	FloatType radiusAt(FloatType i0) const override;
	std::shared_ptr<FloatType> mp_frictionCoeff;
	FloatType m_perturbation = 1e-16;
	FloatType m_shift = 1e-8;

   protected:
	// FloatType m_adhesion;
};
}  // namespace Solver
}  // namespace FrictionFrenzy
