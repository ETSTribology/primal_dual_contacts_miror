#include "NonSmoothContactForce.h"
#include <Corrade/Utility/Debug.h>
#include <imgui.h>
#include "Common/LoggingOptions.h"
#include "Common/MatrixTypes.h"
namespace FrictionFrenzy {
namespace Solver {
void NonSmoothContactForce::preprocess(
	const std::vector<ContactInformation>& contactInfo,
	const std::vector<RigidObjectBase*>& objects,
	const Eigen::Ref<const VectorX> vels,
	const FloatType timestep,
	const FloatType charMass,
	const FloatType charSpeed) {
	const size_t n_contacts = contactInfo.size();
	m_constraintVals.resize(2 * n_contacts);
	m_gradients.resize(n_contacts, 2);
	m_hessians.resize(n_contacts * 2, 2);
	m_nonSmoothK = VectorX::Zero(n_contacts * 3);
	m_KnormInv.resize(n_contacts);
	m_KmatSVD.resize(n_contacts);
	m_WeightMat.resize(n_contacts);
	m_invConstraintVals = VectorX::Zero(n_contacts * 2);
	m_currAvgMass = charMass;

	FloatType damp = m_springDamp * m_springK;

	if (hasLoggingOption(mp_logging, LoggingOptions::MISC_DEBUG)) {
		Corrade::Utility::Debug{} << "Dampening: " << damp / m_springK;
	}

	FloatType tk = timestep * m_springK;
	m_compliance = 1. / (timestep * (tk + damp));
	m_reduction = tk / (tk + damp);

#pragma omp parallel for
	for (size_t i = 0; i < n_contacts; ++i) {
		if (m_springBasedForce) {
			FloatType red =
				-m_reduction * contactInfo[i].depth / timestep / charSpeed;
			m_nonSmoothK(i * 3) += red;
		} else {
			Vector3 hu = Vector3::Zero();
			hu = contactInfo[i].toLocal(vels);
			m_nonSmoothK(i * 3) += hu(0) * m_restitution;
		}
	}
}
void NonSmoothContactForce::initForces(VectorX& forces, VectorX& lambda) {
	forces.reshaped(3, forces.size() / 3).colwise() = Vector3(1, 0, 0);
	lambda.setConstant(1e-2);
}
VectorX NonSmoothContactForce::ACVector(
	const std::vector<ContactInformation>& contactInfo,
	const Eigen::Ref<const VectorX> vels,
	const Eigen::Ref<const VectorX> forces) const {
	VectorX acVec = VectorX::Zero(forces.size());
#pragma omp parallel for
	for (size_t i = 0; i < contactInfo.size(); ++i) {
		Vector3 localU = contactInfo[i].toLocal(vels);
		FloatType toProjectN =
			forces(3 * i) -
			(localU(0) + m_nonSmoothK(3 * i) +
		     m_springBasedForce * m_compliance * m_currAvgMass * forces(3 * i));
		acVec(3 * i) = std::max(toProjectN, FloatType(0)) - forces(3 * i);
		FloatType r = mp_constraint->radiusAt(forces(3 * i));
		Vector2 localTan = forces.segment<2>(3 * i + 1);
		Vector2 tDiff = localTan - localU.tail(2);
		Vector2 proj = (tDiff.norm() < r) ? tDiff : tDiff * r / tDiff.norm();
		acVec.segment<2>(3 * i + 1) = proj - localTan;
	}
	return acVec;
}
VectorX NonSmoothContactForce::calculateTangentialWeight(
	const std::vector<ContactInformation>& contactInfo,
	const std::vector<RigidObjectBase*>& objects,
	const Eigen::Ref<const VectorX> vels,
	const Eigen::Ref<const VectorX> forces,
	const FloatType charMass) {
	VectorX fWeight = VectorX::Ones(forces.size());
	FloatType div = 0;
#pragma omp declare reduction(maximum \
:FloatType : omp_out = std::max(omp_out, omp_in)) initializer(omp_priv = 0)
#pragma omp parallel for reduction(maximum : div)
	for (size_t i = 0; i < contactInfo.size(); ++i) {
		Vector2 tanVel = contactInfo[i].toTangent(vels);
		FloatType frictionWeight = 0;
		for (int j = 0; j < 2; ++j) {
			int oId = contactInfo[i].objId[j];
			if (objects[oId]->isActive()) {
				Vector6 hh = -contactInfo[i].toGlobalTangent(
					j, tanVel.tail<2>().normalized());
				frictionWeight +=
					hh.dot(objects[oId]->invGenMassMat() * hh) * charMass;
			}
		}
		FloatType fricRadius = mp_constraint->radiusAt(forces[3 * i]);
		FloatType contactWeight = std::min(
			fricRadius / (tanVel.norm() + 1e-8) * frictionWeight, FloatType(1));
		fWeight.segment<2>(3 * i + 1).array() = contactWeight;
		div = std::max(div, contactWeight);
	}
	return fWeight;
};
void NonSmoothContactForce::calculateConstraints(
	const Eigen::Ref<const VectorX> vels,
	const Eigen::Ref<const VectorX> forces,
	const Eigen::Ref<const VectorX> lambda) {
#pragma omp parallel for
	for (int i = 0; i < forces.size() / 3; ++i) {
		const Vector3& f = forces.segment<3>(i * 3);
		m_constraintVals[2 * i] = -f[0];

		m_constraintVals[2 * i + 1] = mp_constraint->constraint(f);
		m_gradients.row(i) = mp_constraint->gradient(f).transpose();
		m_hessians.block<2, 2>(i * 2, 0) = mp_constraint->hessian(f);
	}
}
void NonSmoothContactForce::nonSmoothResiduals(
	const std::vector<ContactInformation>& contactInfo,
	const std::vector<RigidObjectBase*>& objects,
	const Eigen::Ref<const VectorX> vels,
	const Eigen::Ref<const VectorX> forces,
	const Eigen::Ref<const VectorX> lambda,
	const FloatType mu,
	const bool useStoredConstraints,
	VectorX& ru,
	VectorX& rf,
	VectorX& rl) {
	ru = VectorX::Zero(objects.size() * 6);
	rf = VectorX::Zero(contactInfo.size() * 3);
	rl = VectorX::Zero(contactInfo.size());

#pragma omp parallel for
	for (size_t i = 0; i < contactInfo.size(); ++i) {
		const Vector3& f = forces.segment<3>(i * 3);
		rf.segment<3>(3 * i) -= contactInfo[i].toLocal(vels);
		for (int j = 0; j < 2; ++j) {
			const int id = contactInfo[i].objId[j];
			const auto& obj = *objects[id];
			Vector6 hTf = -contactInfo[i].toGlobal(j, f);
			if (obj.isActive()) {
				for (int rui = 0; rui < 6; ++rui) {
#pragma omp atomic
					ru[id * 6 + rui] += hTf[rui];
				}
			}
		}
		if (m_springBasedForce) {
			rf(3 * i) -= m_compliance * m_currAvgMass * f[0];
		}

		rf(3 * i) += lambda[2 * i];
		const Vector2& grad = (useStoredConstraints)
		                          ? m_gradients.row(i).transpose()
		                          : mp_constraint->gradient(f);
		rf.segment<2>(3 * i + 1) -= lambda[2 * i + 1] * grad;
	}
	rf -= m_nonSmoothK;
	if (useStoredConstraints) {
		rl = lambda.cwiseProduct(m_constraintVals).array() + mu;
	} else {
		VectorX c(contactInfo.size() * 2);
#pragma omp parallel for
		for (size_t i = 0; i < contactInfo.size(); ++i) {
			const Vector3& f = forces.segment<3>(i * 3);
			c[2 * i] = -f[0];
			c[2 * i + 1] = mp_constraint->constraint(f);
		}
		rl = lambda.cwiseProduct(c).array() + mu;
	}
}
void NonSmoothContactForce::linsysAddition(
	const std::vector<ContactInformation>& contactInfo,
	const std::vector<RigidObjectBase*>& objects,
	const Eigen::Ref<const VectorX> lambda,
	const Eigen::Ref<const VectorX> rf,
	const Eigen::Ref<const VectorX> rl,
	FloatType eps,
	VectorX& y,
	std::unordered_map<std::pair<size_t, size_t>, FloatType*>& matPointers) {
	const size_t n_contacts = contactInfo.size();

	m_sf = rf;
	std::vector<Matrix2> lMats(n_contacts, Matrix2::Zero());
	VectorX lNorms(n_contacts);
	VectorX sfAdd(rf.size());
	sfAdd.setZero();
	m_invConstraintVals =
		1. / (m_constraintVals.array() - eps * lambda.array());

#pragma omp parallel for
	for (size_t i = 0; i < n_contacts; ++i) {
		const Vector2& grad = m_gradients.row(i).transpose();
		lNorms[i] = -lambda[2 * i] * m_invConstraintVals[2 * i];
		if (m_springBasedForce) {
			lNorms[i] += m_compliance * m_currAvgMass;
		}
		sfAdd(3 * i) = -rl(2 * i) * m_invConstraintVals[2 * i];
		lMats[i] = -lambda[2 * i + 1] * m_invConstraintVals[2 * i + 1] * grad *
		           grad.transpose();
		sfAdd.segment<2>(3 * i + 1) =
			grad * rl(2 * i + 1) * m_invConstraintVals[2 * i + 1];
	}
	m_sf += sfAdd;

#pragma omp parallel for
	for (size_t i = 0; i < n_contacts; ++i) {
		FloatType regN = eps + lNorms[i];
		m_KnormInv[i] = 1. / regN;
		Matrix2 reg;
		reg = Matrix2::Identity() * (eps);
		reg += lMats[i];
		reg += lambda[2 * i + 1] * m_hessians.block<2, 2>(i * 2, 0);
		m_KmatSVD[i].compute(reg, Eigen::ComputeFullU | Eigen::ComputeFullV);

		Vector3 kInvSf = Vector3::Zero();
		kInvSf(0) = m_sf(i * 3) * m_KnormInv[i];
		kInvSf.tail<2>() = m_KmatSVD[i].solve(m_sf.segment<2>(i * 3 + 1));
		for (int j = 0; j < 2; ++j) {
			int id1 = contactInfo[i].objId[j];
			if (objects[id1]->isStatic()) {
				continue;
			}
			const auto& trans1 = contactInfo[i].getTrans(j);
			Vector6 yadd_block = trans1 * kInvSf;
			for (int k = 0; k < 6; ++k) {
#pragma omp atomic
				y(id1 * 6 + k, 0) += yadd_block(k);
			}

			for (int k = 0; k < 2; ++k) {
				int id2 = contactInfo[i].objId[k];
				if (objects[id2]->isActive()) {
					const auto& trans2 = contactInfo[i].getTrans(k);
					Matrix6 massBlock = trans1.col(0) * m_KnormInv[i] *
					                    trans2.col(0).transpose();
					massBlock += trans1.block<6, 2>(0, 1) *
					             m_KmatSVD[i].solve(
									 trans2.block<6, 2>(0, 1).transpose());
					for (int row = 0; row < 6; ++row) {
						for (int col = 0; col < 6; ++col) {
#pragma omp atomic
							*(matPointers[{id1 * 6 + row, id2 * 6 + col}]) +=
								massBlock(row, col);
						}
					}
				}
			}
		}
	}
	return;
}
void NonSmoothContactForce::retrieveNonSmoothForceInc(
	const std::vector<ContactInformation>& contactInfo,
	const Eigen::Ref<const VectorX> lambda,
	const Eigen::Ref<const VectorX> du,
	const Eigen::Ref<const VectorX> rf,
	const Eigen::Ref<const VectorX> rl,
	const FloatType mu,
	VectorX& df,
	VectorX& dl) {
	df = VectorX::Zero(contactInfo.size() * 3);
	dl = VectorX::Zero(contactInfo.size() * 2);
#pragma omp parallel for
	for (size_t i = 0; i < contactInfo.size(); ++i) {
		Vector3 dfi = m_sf.segment<3>(i * 3);
		dfi -= contactInfo[i].toLocal(du);
		dfi(0) = m_KnormInv[i] * dfi(0);
		dfi.tail<2>() = m_KmatSVD[i].solve(dfi.tail<2>());
		df.segment<3>(i * 3) = dfi;

		dl(2 * i) =
			-m_invConstraintVals[2 * i] * (-lambda[2 * i] * dfi[0] + rl(2 * i));
		dl(2 * i + 1) =
			-m_invConstraintVals[2 * i + 1] *
			(lambda[2 * i + 1] * m_gradients.row(i).dot(dfi.tail(2)) +
		     rl(2 * i + 1));
	}
}
bool NonSmoothContactForce::filterLineSearch(
	const std::vector<ContactInformation>& contactInfo,
	const std::vector<RigidObjectBase*>& objects,
	const Eigen::Ref<const VectorX> vels,
	const Eigen::Ref<const VectorX> forces,
	const Eigen::Ref<const VectorX> lambda,
	FloatType mu,
	FloatType charMass,
	VectorX& du,
	VectorX& df,
	VectorX& dl) {
	bool projected = false;
#pragma omp parallel for
	for (size_t i = 0; i < contactInfo.size(); ++i) {
		const Vector3& cf = forces.segment<3>(i * 3);
		const Vector3& dfi = df.segment<3>(i * 3);
		Vector3 tf = cf + dfi;

		if (tf[0] < 0) {
			tf[0] = 0;
			df.segment<3>(i * 3) = tf - cf;
			projected = true;
		}

		FloatType co = mp_constraint->constraint(tf);
		if (co > 0) {
			Vector3 pf = mp_constraint->project(tf, tf);
			df.segment<3>(i * 3) = pf - cf;
			projected = true;
		}
	}
#pragma omp parallel for
	for (size_t i = 0; i < 2 * contactInfo.size(); ++i) {
		if (dl[i] < 0 && lambda[i] + dl[i] < 0) {
			dl[i] = -lambda[i];
		}
	}
	return projected;
}
void NonSmoothContactForce::fillFromYAML(const YAML::Node& node) {
	m_springBasedForce = parseScalar<bool>(node, "spring_based_force", false,
	                                       "non_smooth_contact_force");
	if (m_springBasedForce) {
		m_springK = parseScalar<FloatType>(node, "spring_k", 1e6,
		                                   "non_smooth_contact_force");
		m_springDamp = parseScalar<FloatType>(node, "spring_damp", 0.01,
		                                      "non_smooth_contact_force");
	} else {
		m_restitution = parseScalar<FloatType>(node, "restitution", 0.0,
		                                       "non_smooth_contact_force");
	}
	*mp_frictionCoeff = parseScalar<FloatType>(node, "static_coeff", 0.5,
	                                           "non_smooth_contact_force");
}
void NonSmoothContactForce::showImGUIMenu() {
	if (ImGui::TreeNodeEx("Non-smooth contact force",
	                      ImGuiTreeNodeFlags_DefaultOpen)) {
		ImGui::Checkbox("Spring-based force", &m_springBasedForce);
		if (m_springBasedForce) {
			ImGui::SliderFloatType("Spring K", &m_springK, 1e1, 1e6, "%.6g",
			                       ImGuiSliderFlags_Logarithmic);
			ImGui::SliderFloatType("Dampening (rel. to K)", &m_springDamp, 0.,
			                       10., "%.4g", ImGuiSliderFlags_Logarithmic);
		} else {
			ImGui::SliderFloatType("Restitution", &m_restitution, 0., 1.);
		}
		ImGui::SliderFloatType("Friction", mp_frictionCoeff.get(), 0, 2);
		ImGui::TreePop();
	}
}
}  // namespace Solver
}  // namespace FrictionFrenzy
