#include "PrimalDualForceSolver.h"
#include "Common/Cores.h"
#include "Common/LoggingOptions.h"
#include "Common/MatrixTypes.h"
#include "NonSmoothForces/NonSmoothForces.h"

#include <Corrade/Utility/Debug.h>
#include <Eigen/src/IterativeLinearSolvers/ConjugateGradient.h>
#include <imgui/imgui.h>
#include <omp.h>
#include <amgcl/adapter/block_matrix.hpp>
#include <amgcl/amg.hpp>
#include <amgcl/backend/eigen.hpp>
#include <amgcl/coarsening/smoothed_aggregation.hpp>
#include <amgcl/make_solver.hpp>
#include <amgcl/relaxation/as_preconditioner.hpp>
#include <amgcl/relaxation/gauss_seidel.hpp>
#include <amgcl/solver/cg.hpp>
#include <amgcl/value_type/static_matrix.hpp>

namespace FrictionFrenzy {
namespace Solver {
std::tuple<FloatType, FloatType> PrimalDualForceSolver::nonDimensionalParams(
	const FloatType timestep,
	const Vector3& gravity,
	const std::vector<ContactInformation>& contactInfo,
	const std::vector<RigidObjectBase*>& pObjects) {
	FloatType avgMass = 0;
	int actObjs = 0;
	for (size_t i = 0; i < pObjects.size(); ++i) {
		if (pObjects[i]->isActive()) {
			avgMass += pObjects[i]->genMassMat().trace();
			actObjs++;
		}
	}
	avgMass = (actObjs > 0) ? avgMass / actObjs / 6 : 1;

	VectorX objSpeed = VectorX::Zero(pObjects.size());
	for (size_t i = 0; i < contactInfo.size(); ++i) {
		bool hasStatic = false;
		Vector3 relVel = Vector3::Zero();
		for (int j = 0; j < 2; ++j) {
			int objId = contactInfo[i].objId[j];
			Vector6 velExt;
			velExt << pObjects[objId]->velocity(),
				pObjects[objId]->angularVel();
			if (pObjects[objId]->isActive()) {
				velExt.head(3) += gravity * timestep;
			} else {
				hasStatic = true;
			}
			Vector3 relVelHalf = contactInfo[i].toLocal(j, velExt);
			relVel += relVelHalf;

			objSpeed[objId] = std::max(objSpeed[objId], relVelHalf.norm());
		}
		if (hasStatic) {
			for (int j = 0; j < 2; ++j) {
				int objId = contactInfo[i].objId[j];
				objSpeed[objId] = std::max(objSpeed[objId], relVel.norm());
			}
		}
	}
	FloatType charSpeed = 0;
	FloatType massSum = 0;
	for (size_t i = 0; i < pObjects.size(); ++i) {
		if (pObjects[i]->isActive()) {
			massSum += pObjects[i]->mass();
			charSpeed += objSpeed[i] * pObjects[i]->mass();
		}
	}
	charSpeed /= massSum;

	charSpeed = std::max(charSpeed, gravity.norm() * timestep * 0.25);
	charSpeed = std::max(charSpeed, 1e-4);

	return {avgMass, charSpeed};
}
void PrimalDualForceSolver::fillForces(
	const FloatType timestep,
	const Vector3& gravity,
	std::vector<ContactInformation>& contactInfo,
	std::vector<std::unique_ptr<RigidObjectBase>>& objects) {
	// Initialize the object forces with gravity
	for (size_t i = 0; i < objects.size(); ++i) {
		objects[i]->acceleration() = gravity * timestep;
		objects[i]->angularAcc() << 0, 0, 0;
	}

	// Terminate if no contacts are present.
	if (contactInfo.empty()) {
		return;
	}

	// Renumber objects to exclude ones not in contact.
	std::unordered_map<int, int> globalToLocalObjId;
	std::vector<int> localToGlobalObjId;
	for (size_t i = 0; i < contactInfo.size(); ++i) {
		for (int j = 0; j < 2; ++j) {
			int id = contactInfo[i].objId[j];
			if (!globalToLocalObjId.count(id)) {
				int localId = globalToLocalObjId.size();
				globalToLocalObjId[id] = localId;
				localToGlobalObjId.push_back(id);
			}
		}
	}
	std::vector<RigidObjectBase*> localObjs(globalToLocalObjId.size());
	for (size_t i = 0; i < localObjs.size(); ++i) {
		localObjs[i] = objects[localToGlobalObjId[i]].get();
	}
#pragma omp parallel for
	for (size_t i = 0; i < contactInfo.size(); ++i) {
		for (int j = 0; j < 2; ++j) {
			int id = contactInfo[i].objId[j];
			contactInfo[i].objId[j] = globalToLocalObjId[id];
		}
	}

	fillForcesIntl(timestep, gravity, contactInfo, localObjs);

	// Restore the numbering of objects.
#pragma omp parallel for
	for (size_t i = 0; i < contactInfo.size(); ++i) {
		for (int j = 0; j < 2; ++j) {
			int id = contactInfo[i].objId[j];
			contactInfo[i].objId[j] = localToGlobalObjId[id];
		}
	}
}

template <typename D>
void PrimalDualForceSolver::calculateResiduals(
	bool useStoredConstraints,
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
	Eigen::MatrixBase<D>& rl) {
	VectorX ruadd;
	ru = massMat * (vels - velsOld);
#pragma omp parallel for
	for (size_t i = 0; i < pObjects.size(); ++i) {
		if (pObjects[i]->isStatic()) {
			ru.segment(i * 6, 6).setZero();
		}
	}
	VectorX fWeight = mp_nonSmoothForce->calculateTangentialWeight(
		contactInfo, pObjects, vels, forces, charMass);
	mp_nonSmoothForce->nonSmoothResiduals(contactInfo, pObjects, vels, forces,
	                                      lambda, mu, useStoredConstraints,
	                                      ruadd, rf.derived(), rl.derived());
	ru += ruadd;
	wrf = rf.cwiseProduct(fWeight);
}

void PrimalDualForceSolver::fillForcesIntl(
	const FloatType timestep,
	const Vector3& gravity,
	std::vector<ContactInformation>& contactInfo,
	std::vector<RigidObjectBase*>& pObjects) {
	// Number of different unknowns for convenience.
	const size_t n_contacts = contactInfo.size();
	const size_t n_objects = pObjects.size();
	const size_t n_rigidDof = pObjects.size() * 6;
	const size_t n_forceDof = n_contacts * mp_nonSmoothForce->dimensions();
	const size_t n_constraintDof =
		n_contacts * mp_nonSmoothForce->constraintsPerForce();
	const size_t n_unknowns = n_rigidDof + n_forceDof + n_constraintDof;

	// The unknowns themselves.
	VectorX velsOld(n_rigidDof);
	VectorX vels(n_rigidDof);
	VectorX forces(n_forceDof);
	VectorX lambda(n_constraintDof);

	// The linear system structures.
	VectorX rhs(n_rigidDof);
	SparseMat A(n_rigidDof, n_rigidDof);
	std::unordered_map<std::pair<size_t, size_t>, FloatType*> matPointers;

	VectorX du(rhs.size()), df, dl;

	Eigen::setNbThreads((n_unknowns < 1000) ? 1 : physicalProcessors());
	omp_set_num_threads((n_unknowns < 1000) ? 1 : physicalProcessors());
	if (hasLoggingOption(mp_logging, LoggingOptions::MISC_DEBUG)) {
		Corrade::Utility::Debug{} << "Solving forces";
	}

	m_forceProcessTimer.Resume();

	// Initialize velocities.
#pragma omp parallel for
	for (size_t i = 0; i < n_objects; ++i) {
		vels.segment<6>(i * 6) << pObjects[i]->velocity(),
			pObjects[i]->angularVel();
		velsOld.segment<6>(i * 6) = vels.segment<6>(i * 6);
		if (pObjects[i]->isActive()) {
			velsOld.segment<3>(i * 6) += gravity * timestep;
		}
	}

	FloatType charMass = 1, charSpeed = 1;
	std::tie(charMass, charSpeed) =
		nonDimensionalParams(timestep, gravity, contactInfo, pObjects);
	vels /= charSpeed;
	velsOld /= charSpeed;

	// Pre solve logging
	if (hasLoggingOption(mp_logging, LoggingOptions::PER_STEP_CHARFACTOR)) {
		Corrade::Utility::Debug{} << "Characteristic mass / speed: " << charMass
								  << " " << charSpeed;
	}
	if (hasLoggingOption(mp_logging, LoggingOptions::MAX_PENETRATION_DEPTH)) {
		FloatType depth = 0;
		for (size_t i = 0; i < contactInfo.size(); ++i) {
			depth = std::max(depth, std::abs(contactInfo[i].depth));
		}
		Corrade::Utility::Debug{} << "Max penetration depth: " << depth;
	}

	// Initialize mass matrix
	std::vector<Eigen::Triplet<FloatType>> massTriplets;
	for (size_t i = 0; i < n_objects; ++i) {
		if (pObjects[i]->isStatic()) {
			for (int j = 0; j < 6; ++j) {
				massTriplets.emplace_back(i * 6 + j, i * 6 + j, 1);
			}
		} else {
			Matrix6 masses = pObjects[i]->genMassMat() / charMass;
			for (int j = 0; j < 6; ++j) {
				for (int k = 0; k < 6; ++k) {
					if (masses(j, k)) {
						massTriplets.emplace_back(i * 6 + j, i * 6 + k,
						                          masses(j, k));
					}
				}
			}
		}
	}
	SparseMat massMat(n_rigidDof, n_rigidDof);
	massMat.setFromTriplets(massTriplets.begin(), massTriplets.end());

	// Initialize the sparse matrix structures.
	std::unordered_set<std::pair<size_t, size_t>> objectPairs;
	mp_nonSmoothForce->linsysReserve(contactInfo, pObjects, objectPairs,
	                                 massTriplets);
	A.setFromTriplets(massTriplets.begin(), massTriplets.end());
	for (int k = 0; k < A.outerSize(); ++k) {
		for (SparseMat::InnerIterator it(A, k); it; ++it) {
			matPointers[{it.row(), it.col()}] = &(it.valueRef());
		}
	}

	// Initialize force solvers.
	mp_nonSmoothForce->preprocess(contactInfo, pObjects, vels, timestep,
	                              charMass, charSpeed);
	// Initialize forces and constraints.
	mp_nonSmoothForce->initForces(forces, lambda);

	int itCoupled = 0;
	LDLT ldlt;
	CG cg;
	bool earlyTermination = false;

	FloatType Hu = 1e-4;
	FloatType exitErr;

	// Main Newton's method loop
	auto mainIteration = [&](int maxIters, FloatType tol, int& it) {
		const FloatType power = 0.66667;
		VectorX ru, ruadd, rf, wrf, rl;
		for (it = 0; it < maxIters; ++it) {
			// Build the linear system
			m_systemBuildTimer.Resume();

			// Calculate constraints, residuals, and errors.
			mp_nonSmoothForce->calculateConstraints(vels, forces, lambda);
			FloatType sDualGap = mp_nonSmoothForce->surrogateDualGap(lambda);
			const FloatType mu = sDualGap * 0.1;

			calculateResiduals(true, contactInfo, pObjects, vels, forces,
			                   lambda, velsOld, massMat, charMass, charSpeed,
			                   timestep, mu, ru, rf, wrf, rl);
			VectorX acVec =
				mp_nonSmoothForce->ACVector(contactInfo, vels, forces);
			FloatType err =
				ru.squaredNorm() + wrf.squaredNorm() + rl.squaredNorm();
			FloatType sErr = searchError(ru, wrf, rl);
			FloatType sacErr = searchACError(ru, acVec, rl);
			FloatType eps = Hu * std::pow(err, power * 0.5);
			FloatType momenErr = ru.norm() / n_rigidDof;
			exitErr = std::max(momenErr, wrf.norm() / n_forceDof);
			exitErr = std::max(exitErr, rl.norm() / n_constraintDof);
			exitErr = std::max(exitErr, sDualGap);
			exitErr = std::min(exitErr,
			                   std::max(momenErr, acVec.norm() / n_contacts));
			if (hasLoggingOption(mp_logging, LoggingOptions::PER_ITER_ERRORS)) {
				Corrade::Utility::Debug{}
					<< "Error: " << ru.norm() / ru.size() << " "
					<< wrf.norm() / n_forceDof << " "
					<< rl.norm() / n_constraintDof
					<< ", search errors: " << sErr << " " << sacErr
					<< ", AC Error: " << acVec.norm() / acVec.size()
					<< ", dual gap: " << sDualGap << " H: " << Hu
					<< ", eps: " << eps;
			}
			if (exitErr < tol) {
				break;
			}

			// Set linear system to zero
			rhs.setZero();
#pragma omp parallel
			{
#pragma omp for
				for (int k = 0; k < A.outerSize(); ++k) {
					for (SparseMat::InnerIterator Ait(A, k); Ait; ++Ait) {
						Ait.valueRef() = 0;
					}
				}
#pragma omp for
				for (int k = 0; k < massMat.outerSize(); ++k) {
					for (SparseMat::InnerIterator Mit(massMat, k); Mit; ++Mit) {
						*matPointers[{Mit.row(), Mit.col()}] += Mit.value();
						if (Mit.row() == Mit.col()) {
							*matPointers[{Mit.row(), Mit.col()}] += eps;
						}
					}
				}
			}

			// Calculate linear system terms.
			mp_nonSmoothForce->linsysAddition(contactInfo, pObjects, lambda, rf,
			                                  rl, eps, rhs, matPointers);
			rhs -= ru;

			VectorX D(A.rows());
			// Set static objects to zero.
#pragma omp parallel
			{
#pragma omp for
				for (size_t i = 0; i < pObjects.size(); ++i) {
					if (pObjects[i]->isStatic()) {
						rhs.segment<6>(i * 6).setZero();
					}
				}

				// Scale linear system for better convergence.
#pragma omp for
				for (Eigen::Index k = 0; k < A.outerSize(); ++k) {
					for (SparseMat::InnerIterator Mit(A, k); Mit; ++Mit) {
						if (Mit.row() == Mit.col()) {
							D[k] = 1 / std::sqrt(Mit.value());
							break;
						}
					}
				}
#pragma omp for
				for (int k = 0; k < A.outerSize(); ++k) {
					for (SparseMat::InnerIterator Mit(A, k); Mit; ++Mit) {
						Mit.valueRef() *= (D[Mit.row()] * D[Mit.col()]);
					}
				}
			}
			rhs = rhs.cwiseProduct(D);
			m_systemBuildTimer.Pause();

			// Solve linear system with either Eigen or AMGCL.
			m_systemSolveTimer.Resume();
			size_t maxCGIt = 1000;
			size_t iters = 0;
			FloatType linError;

			if (pObjects.size() < 2000) {
				if (it == 0) {
					ldlt.analyzePattern(A);
				}
				ldlt.factorize(A);
				if (ldlt.info() == Eigen::Success) {
					du = ldlt.solve(rhs);
				} else {
					du.setZero();
				}
			} else {
				typedef amgcl::static_matrix<FloatType, 6, 6> dmat_type;
				typedef amgcl::static_matrix<FloatType, 6, 1> dvec_type;
				typedef amgcl::backend::builtin<dmat_type> SBackend;
				typedef amgcl::make_solver<
					amgcl::amg<SBackend,
				               amgcl::coarsening::smoothed_aggregation,
				               amgcl::relaxation::gauss_seidel>,
					amgcl::solver::cg<SBackend>>
					amgSolver;

				auto pRhs = reinterpret_cast<dvec_type*>(rhs.data());
				auto pX = reinterpret_cast<dvec_type*>(du.data());
				auto B =
					amgcl::make_iterator_range(pRhs, pRhs + pObjects.size());
				auto X = amgcl::make_iterator_range(pX, pX + pObjects.size());
				amgSolver::params prm;
				auto Ab = amgcl::adapter::block_matrix<dmat_type>(A);
				prm.solver.maxiter = maxCGIt;
				amgSolver amgs(Ab, prm);
				std::tie(iters, linError) = amgs(B, X);
				if (!std::isfinite(linError)) {
					CG cg;
					du = cg.compute(A).solve(rhs);
					iters = cg.iterations();
					linError = cg.error();
				}
				if (hasLoggingOption(mp_logging,
				                     LoggingOptions::PER_ITER_ERRORS)) {
					Corrade::Utility::Debug{} << "Solver: " << iters
											  << " iters, error: " << linError;
				}
			}
			du = du.cwiseProduct(D);

			// Retrieve force and lambda step size.
			mp_nonSmoothForce->retrieveNonSmoothForceInc(
				contactInfo, lambda, du, rf, rl, mu, df, dl);
			m_systemSolveTimer.Pause();

			// Perform a line search.
			m_lineSearchTimer.Resume();
			LineSearchResult stepSearch =
				lineSearch(contactInfo, pObjects, massMat, vels, forces, lambda,
			               du, df, dl, velsOld, sErr, sacErr, mu, timestep,
			               charSpeed, charMass, 10);
			bool useGD = false;

			// Fall back to gradient descent if failed.
			if (stepSearch.stepSize == 0) {
				if (hasLoggingOption(mp_logging,
				                     LoggingOptions::PER_ITER_ERRORS)) {
					Corrade::Utility::Debug{}
						<< "Falling back to gradient descent...";
				}
				useGD = true;
				du = -ru;
				df = rf;
				dl = rl;
				stepSearch =
					lineSearch(contactInfo, pObjects, massMat, vels, forces,
				               lambda, du, df, dl, velsOld, sErr, sacErr, mu,
				               timestep, charSpeed, charMass, 20);
				if (earlyTermination) {
					if (hasLoggingOption(mp_logging,
					                     LoggingOptions::PER_ITER_ERRORS)) {
						Corrade::Utility::Warning{}
							<< "Ending without converging...";
					}
					++it;
					break;
				}
			}

			vels = stepSearch.newU;
			forces = stepSearch.newF;
			lambda = stepSearch.newL;

			// Update regularizer.
			if (!useGD && stepSearch.stepSize == 1) {
				if (iters == maxCGIt) {
					Hu *= 1.3;
				} else {
					Hu /= 4;
				}
			} else {
				Hu *= 4;
			}
			Hu = std::max(Hu, FloatType(1e-6));
			m_lineSearchTimer.Pause();
		}
	};

	mainIteration(m_maxIter, m_tol, itCoupled);
	m_totalIterations += itCoupled;

#pragma omp parallel
	{
		// Apply new velocities
#pragma omp for
		for (size_t i = 0; i < pObjects.size(); ++i) {
			Vector6 newVel = vels.segment<6>(i * 6) * charSpeed;
			pObjects[i]->acceleration() =
				newVel.head<3>() - pObjects[i]->velocity();
			pObjects[i]->angularAcc() =
				newVel.tail<3>() - pObjects[i]->angularVel();
		}

		// Write new forces
#pragma omp for
		for (size_t i = 0; i < n_contacts; ++i) {
			int dim = mp_nonSmoothForce->dimensions();
			contactInfo[i].localForce.head(dim) =
				forces.segment(i * dim, dim) * charSpeed * charMass;
		}
	}

	m_forceProcessTimer.Pause();

	// Post solve logging
	if (hasLoggingOption(mp_logging, LoggingOptions::PER_STEP_ERRORS)) {
		VectorX acVec = mp_nonSmoothForce->ACVector(contactInfo, vels, forces);
		Corrade::Utility::Debug{} << "Coupled iterations: " << itCoupled;
		Corrade::Utility::Debug{
			Corrade::Utility::Debug::Flag::NoNewlineAtTheEnd}
			<< "Exit Error: " << exitErr;
		Corrade::Utility::Debug{}
			<< " acMax: "
			<< acVec
				   .reshaped(mp_nonSmoothForce->dimensions(),
		                     acVec.size() / mp_nonSmoothForce->dimensions())
				   .colwise()
				   .norm()
				   .maxCoeff();
	}
	if (mp_nonSmoothForce->getType() ==
	        NonSmoothForceType::NON_SMOOTH_CONTACTS &&
	    hasLoggingOption(mp_logging, LoggingOptions::PER_STEP_FRICTION_COEFF)) {
		FloatType averageFrictionCoeff = 0;
		FloatType averageFCWeight = 0;
		const FloatType coeff =
			*(dynamic_cast<NonSmoothContactForce*>(mp_nonSmoothForce.get())
		          ->mp_frictionCoeff);
		for (size_t i = 0; i < contactInfo.size(); ++i) {
			FloatType currCoeff = contactInfo[i].localForce.tail(2).norm() /
			                      contactInfo[i].localForce[0];
			if (currCoeff > coeff) {
				FloatType weight = contactInfo[i].localForce[0];
				averageFrictionCoeff += weight * currCoeff;
				averageFCWeight += weight;
			}
		}
		Corrade::Utility::Debug{}
			<< "Average friction coefficient: "
			<< (averageFCWeight ? averageFrictionCoeff / averageFCWeight
		                        : coeff);
	}
	if (hasLoggingOption(mp_logging, LoggingOptions::PER_STEP_SPEEDS)) {
		FloatType tanSpeeds =
			objToContact(contactInfo, vels * charSpeed, {0, 1, 1})
				.reshaped(2, contactInfo.size())
				.colwise()
				.norm()
				.maxCoeff();
		Corrade::Utility::Debug{} << "Max tangential speed: " << tanSpeeds;
	}
}
PrimalDualForceSolver::LineSearchResult PrimalDualForceSolver::lineSearch(
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
	const int maxLineSearch) {
	FloatType step = 1.0;
	LineSearchResult res;
	res.newU = vels;
	res.newF = forces;
	res.newL = lambda;
	res.sErr = sErr;
	res.acErr = sacErr;
	res.stepSize = 0;
	for (int stepIt = 0; stepIt < maxLineSearch; ++stepIt) {
		VectorX duIn = step * du;
		VectorX dfIn = step * df;
		VectorX dlIn = step * dl;

		mp_nonSmoothForce->filterLineSearch(contactInfo, pObjects, vels, forces,
		                                    lambda, mu, charMass, duIn, dfIn,
		                                    dlIn);
		VectorX lInner = lambda + 0.99 * dlIn;
		VectorX vInner = vels + 0.99 * duIn;
		VectorX fInner = forces + 0.99 * dfIn;

		VectorX ruIn, rfIn, wrfIn, rlIn;
		calculateResiduals(false, contactInfo, pObjects, vInner, fInner, lInner,
		                   velsOld, massMat, charMass, charSpeed, timestep, mu,
		                   ruIn, rfIn, wrfIn, rlIn);

		FloatType inErr = searchError(ruIn, wrfIn, rlIn);
		VectorX acVecIn =
			mp_nonSmoothForce->ACVector(contactInfo, vInner, fInner);
		FloatType sacErrIn = searchACError(ruIn, acVecIn, rlIn);
		FloatType gErr = -duIn.dot(ruIn) + dfIn.dot(rfIn) +
		                 dlIn.cwiseQuotient(lInner).dot(rlIn);
		bool inErrAcpt = inErr < sErr;
		bool acErrAcpt = sacErrIn < sacErr;
		bool acpt = inErrAcpt | acErrAcpt;
		if ((acpt && gErr > 0) || (stepIt == maxLineSearch - 1)) {
			if (hasLoggingOption(mp_logging, LoggingOptions::PER_ITER_ERRORS)) {
				Corrade::Utility::Debug{} << step << " " << ruIn.squaredNorm()
										  << " " << rfIn.squaredNorm() << " "
										  << rlIn.squaredNorm() << " "
										  << acVecIn.squaredNorm();
			}
			if (acpt) {
				res.newU = vInner;
				res.newF = fInner;
				res.newL = lInner;
				res.stepSize = step;
				res.sErr = inErr;
				res.acErr = sacErrIn;
				return res;
			}
			break;
		}
		step *= 0.5;
	}
	return res;
}
std::string PrimalDualForceSolver::printTimings(size_t totalSteps) {
	std::stringstream ss;

	ss << "\n\tAvg. force calc time per substep: "
	   << m_forceProcessTimer.GetSeconds() / totalSteps
	   << "\n\t\tBuild system: " << m_systemBuildTimer.GetSeconds() / totalSteps
	   << "\n\t\tSolve system: " << m_systemSolveTimer.GetSeconds() / totalSteps
	   << "\n\t\tLine search: " << m_lineSearchTimer.GetSeconds() / totalSteps
	   << "\n\tAverage #iterations: " << (double)m_totalIterations / totalSteps
	   << "\n";

	return ss.str();
}

void PrimalDualForceSolver::fillFromYAML(const YAML::Node& node) {
	m_tol = parseScalar<FloatType>(node, "tolerance", 1e-6, "simulation");
	m_maxIter = parseScalar<int>(node, "iterations", 100, "simulation");
	std::shared_ptr<FloatType> frictionCoeff = std::make_shared<FloatType>(0.);

	if (!node["non_smooth_force"].IsDefined()) {
		throw std::runtime_error("No non-smooth force defined!");
	}
	const YAML::Node nonSmoothForceNode = node["non_smooth_force"];
	if (!nonSmoothForceNode["name"].IsDefined()) {
		throw std::runtime_error("Non-smooth force: no name provided!");
	}
	std::string nonSmoothForceName =
		nonSmoothForceNode["name"].as<std::string>();
	if (nonSmoothForceName == "non_smooth_contact") {
		mp_nonSmoothForce =
			std::make_unique<Solver::NonSmoothContactForce>(mp_logging);
		auto pForce = dynamic_cast<Solver::NonSmoothContactForce*>(
			mp_nonSmoothForce.get());
		frictionCoeff = pForce->mp_frictionCoeff;
	} else if (nonSmoothForceName == "only_normal") {
		mp_nonSmoothForce =
			std::make_unique<Solver::OnlyNormalForce>(mp_logging);
	}
	mp_nonSmoothForce->fillFromYAML(nonSmoothForceNode);
}

void PrimalDualForceSolver::showImGUIMenu() {
	if (ImGui::TreeNodeEx("Solver: Primal Dual Solver",
	                      ImGuiTreeNodeFlags_DefaultOpen)) {
		ImGui::SliderInt("Iterations", &m_maxIter, 1, 50);
		ImGui::SliderFloatType("Tolerance", &m_tol, 1e-8, 1e-1, "%.8g",
		                       ImGuiSliderFlags_Logarithmic);
		mp_nonSmoothForce->showImGUIMenu();
		ImGui::TreePop();
	}
}
}  // namespace Solver
}  // namespace FrictionFrenzy
