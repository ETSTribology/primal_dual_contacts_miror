#pragma once

#include "Common/MatrixTypes.h"

namespace FrictionFrenzy {
namespace Contact {

/**
 * Data structure to store contact information. This is converted from FCL's
 * contact structure after contact detection.
 */
class ContactInformation {
   public:
	typedef Eigen::Matrix<FloatType, 6, 3> HalfDelassus;
	
	/* Member functions */
	
	/**
	 * Constructor
	 */
	ContactInformation() {}
	/**
	 * @return The contact normal for the contact
	 */
	Vector3 normal() const { return transformation[0].block<3, 1>(0, 0); }
	/**
	 * @return The local reference frame for the contact.
	 */
	Matrix3 localFrame() const { return transformation[0].block<3, 3>(0, 0); }

	/**
	 * Calculate the local velocity at contact point.
	 * @param T The object id (stored as objId[T])
	 * @param globalQuant The quantity to convert to local frame.
	 * @return the 3-dim local quantity
	 */
	Vector3 toLocal(bool T,
	                const Eigen::Ref<const Vector6>& globalQuant) const {
		return -transformation[T].transpose() * globalQuant;
	}
	/**
	 * Calculate the local relative velocity at contact point.
	 * @param globalQuants The full velctor of object velocities.
	 * @return the 3-dim local velocity.
	 */
	Vector3 toLocal(const Eigen::Ref<const VectorX>& globalQuants) const {
		return -transformation[0].transpose() *
		           globalQuants.segment<6>(objId[0] * 6) -
		       transformation[1].transpose() *
		           globalQuants.segment<6>(objId[1] * 6);
	}
	/**
	 * Calculate the relative normal velocity at contact point.
	 * @param globalQuants The full velctor of object velocities.
	 * @return the scalar normal velocity.
	 */
	FloatType toNormal(const Eigen::Ref<const VectorX>& globalQuants) const {
		return -transformation[0].col(0).dot(
				   globalQuants.segment<6>(objId[0] * 6)) -
		       transformation[1].col(0).dot(
				   globalQuants.segment<6>(objId[1] * 6));
	}
	/**
	 * Calculate the relative tangential velocity at contact point.
	 * @param globalQuants The full velctor of object velocities.
	 * @return the 2-dim tangential velocity.
	 */
	Vector2 toTangent(const Eigen::Ref<const VectorX>& globalQuants) const {
		return -transformation[0].block<6, 2>(0, 1).transpose() *
		           globalQuants.segment<6>(objId[0] * 6) -
		       transformation[1].block<6, 2>(0, 1).transpose() *
		           globalQuants.segment<6>(objId[1] * 6);
	}
	/**
	 * Calculate a global quantity of object objId[T] from local quantity.
	 * @param T The index to object id
	 * @param localQuant The local quantity
	 * @returns The 6-dim global quantity
	 */
	Vector6 toGlobal(bool T,
	                 const Eigen::Ref<const Vector3>& localQuant) const {
		return -transformation[T] * localQuant;
	}
	/**
	 * Calculate a global quantity of object objId[T] from local normal
	 * quantity.
	 * @param T The index to object id
	 * @param localQuant The local normal quantity
	 * @returns The 6-dim global quantity
	 */
	Vector6 toGlobalNormal(bool T, const FloatType& localQuant) const {
		return -transformation[T].col(0) * localQuant;
	}
	/**
	 * Calculate a global quantity of object objId[T] from local tangential
	 * quantity.
	 * @param T The index to object id
	 * @param localQuant The local tangential quantity
	 * @returns The 6-dim global quantity
	 */
	Vector6 toGlobalTangent(bool T,
	                        const Eigen::Ref<const Vector2>& localQuant) const {
		return -transformation[T].block<6, 2>(0, 1) * localQuant;
	}
	/**
	 * Get the constraint jacobian w.r.t. object T.
	 * @param T The index to object id
	 * @returns The constraint jacobian.
	 */
	HalfDelassus getTrans(bool T) const { return -transformation[T]; }
	/**
	 * Get the constraint jacobian in the normal direction w.r.t. object T.
	 * @param T The index to object id
	 * @returns The constraint jacobian.
	 */
	Vector6 getTransNormal(bool T) const { return -transformation[T].col(0); }
	/**
	 * Get the constraint jacobian in the tangential direction w.r.t. object T.
	 * @param T The index to object id
	 * @returns The constraint jacobian.
	 */
	Eigen::Matrix<FloatType, 6, 2> getTransTangent(bool T) const {
		return -transformation[T].block<6, 2>(0, 1);
	}
	/**
	 * Output contact information to a string for debugging purposes.
	 */
	std::string toString() {
		std::stringstream ss;
		ss.str(std::string());
		ss << "Position: " << pos.transpose()
		   << "\nNormal: " << normal().transpose() << "\nDepth: " << depth
		   << "\nContact force: " << localForce.transpose();
		return ss.str();
	}

	/* Member variables */
	Vector3 pos;              /// The (global) collision position.
	Vector3 localForce;       /// The contact force in local frame.
	FloatType depth;  /// Penetration depth.
	//FloatType area;   /// Contact area.
	size_t objId[2];  /// The object IDs of contacting objects.
	HalfDelassus transformation[2];  /// The (negative) constraint jacobians
	                                 /// w.r.t. both objects.
};
}  // namespace Contact
}  // namespace FrictionFrenzy
