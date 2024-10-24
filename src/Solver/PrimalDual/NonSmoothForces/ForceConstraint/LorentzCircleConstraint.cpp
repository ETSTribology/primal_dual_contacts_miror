#include "LorentzCircleConstraint.h"

namespace FrictionFrenzy {
namespace Solver {
FloatType LorentzCircleConstraint::constraint(
	const Eigen::Ref<const VectorX> in) const {
	assert(in.size() == 3);
	FloatType r = in[1] * in[1] + in[2] * in[2] + m_perturbation;
	FloatType f = *mp_frictionCoeff;
	FloatType c =
		std::sqrt(r) - f * in[0] - std::sqrt(m_perturbation) - m_shift;
	return c;
}
VectorX LorentzCircleConstraint::gradient(
	const Eigen::Ref<const VectorX> in) const {
	assert(in.size() == 3);
	FloatType r2 = in[1] * in[1] + in[2] * in[2] + m_perturbation;
	FloatType r = std::sqrt(r2);
	Vector2 ret;
	ret << in[1] / r, in[2] / r;
	return ret;
}
MatrixX LorentzCircleConstraint::hessian(
	const Eigen::Ref<const VectorX> in) const {
	assert(in.size() == 3);
	FloatType p = std::max(m_perturbation, 1e-6);
	FloatType r2 = in[1] * in[1] + in[2] * in[2] + p;
	Matrix2 ret = Matrix2::Zero();
	FloatType r3 = std::pow(r2, -1.5);
	Vector2 zmy;
	zmy << in[2], -in[1];
	ret << zmy * zmy.transpose() + p * Matrix2::Identity();
	ret *= r3;
	return ret;
}
VectorX LorentzCircleConstraint::project(const Eigen::Ref<const VectorX> in,
                                        const Eigen::Ref<const VectorX> start,
                                        FloatType target) const {
	Vector3 out = in;
	if (in(0) < 0) {
		out(0) = 0;
	}
	if (constraint(out) > target) {
		FloatType r = std::max(radiusAt(in(0)) + target, FloatType(0));
		out.tail(2) *= r / out.tail(2).norm();
	}

	return out;
}
FloatType LorentzCircleConstraint::radiusAt(FloatType i0) const {
	FloatType f = *mp_frictionCoeff;
	FloatType k = f * i0 + std::sqrt(m_perturbation) + m_shift;
	FloatType r = std::sqrt(k * k - m_perturbation);
	return r;
}
}  // namespace Solver
}  // namespace FrictionFrenzy
