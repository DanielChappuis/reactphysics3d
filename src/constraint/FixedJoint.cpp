/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2019 Daniel Chappuis                                       *
*********************************************************************************
*                                                                               *
* This software is provided 'as-is', without any express or implied warranty.   *
* In no event will the authors be held liable for any damages arising from the  *
* use of this software.                                                         *
*                                                                               *
* Permission is granted to anyone to use this software for any purpose,         *
* including commercial applications, and to alter it and redistribute it        *
* freely, subject to the following restrictions:                                *
*                                                                               *
* 1. The origin of this software must not be misrepresented; you must not claim *
*    that you wrote the original software. If you use this software in a        *
*    product, an acknowledgment in the product documentation would be           *
*    appreciated but is not required.                                           *
*                                                                               *
* 2. Altered source versions must be plainly marked as such, and must not be    *
*    misrepresented as being the original software.                             *
*                                                                               *
* 3. This notice may not be removed or altered from any source distribution.    *
*                                                                               *
********************************************************************************/

// Libraries
#include "FixedJoint.h"
#include "engine/ConstraintSolver.h"

using namespace reactphysics3d;

// Static variables definition
const decimal FixedJoint::BETA = decimal(0.2);

// Constructor
FixedJoint::FixedJoint(uint id, const FixedJointInfo& jointInfo)
           : Joint(id, jointInfo), mImpulseTranslation(0, 0, 0), mImpulseRotation(0, 0, 0) {

    // Compute the local-space anchor point for each body
    const Transform& transform1 = mBody1->getTransform();
    const Transform& transform2 = mBody2->getTransform();
    mLocalAnchorPointBody1 = transform1.getInverse() * jointInfo.anchorPointWorldSpace;
    mLocalAnchorPointBody2 = transform2.getInverse() * jointInfo.anchorPointWorldSpace;

	// Store inverse of initial rotation from body 1 to body 2 in body 1 space:
	//
	// q20 = q10 r0 
	// <=> r0 = q10^-1 q20 
	// <=> r0^-1 = q20^-1 q10
	//
	// where:
	//
	// q20 = initial orientation of body 2
	// q10 = initial orientation of body 1
	// r0 = initial rotation rotation from body 1 to body 2
	mInitOrientationDifferenceInv = transform2.getOrientation().getInverse() * transform1.getOrientation();
}

// Initialize before solving the constraint
void FixedJoint::initBeforeSolve(const ConstraintSolverData& constraintSolverData) {

    // Initialize the bodies index in the velocity array
    mIndexBody1 = mBody1->mArrayIndex;
    mIndexBody2 = mBody2->mArrayIndex;

    // Get the bodies positions and orientations
    const Vector3& x1 = mBody1->mCenterOfMassWorld;
    const Vector3& x2 = mBody2->mCenterOfMassWorld;
    const Quaternion& orientationBody1 = mBody1->getTransform().getOrientation();
    const Quaternion& orientationBody2 = mBody2->getTransform().getOrientation();

    // Get the inertia tensor of bodies
    mI1 = mBody1->getInertiaTensorInverseWorld();
    mI2 = mBody2->getInertiaTensorInverseWorld();

    // Compute the vector from body center to the anchor point in world-space
    mR1World = orientationBody1 * mLocalAnchorPointBody1;
    mR2World = orientationBody2 * mLocalAnchorPointBody2;

    // Compute the corresponding skew-symmetric matrices
    Matrix3x3 skewSymmetricMatrixU1= Matrix3x3::computeSkewSymmetricMatrixForCrossProduct(mR1World);
    Matrix3x3 skewSymmetricMatrixU2= Matrix3x3::computeSkewSymmetricMatrixForCrossProduct(mR2World);

    // Compute the matrix K=JM^-1J^t (3x3 matrix) for the 3 translation constraints
    decimal inverseMassBodies = mBody1->mMassInverse + mBody2->mMassInverse;
    Matrix3x3 massMatrix = Matrix3x3(inverseMassBodies, 0, 0,
                                    0, inverseMassBodies, 0,
                                    0, 0, inverseMassBodies) +
                           skewSymmetricMatrixU1 * mI1 * skewSymmetricMatrixU1.getTranspose() +
                           skewSymmetricMatrixU2 * mI2 * skewSymmetricMatrixU2.getTranspose();

    // Compute the inverse mass matrix K^-1 for the 3 translation constraints
    mInverseMassMatrixTranslation.setToZero();
    if (mBody1->getType() == BodyType::DYNAMIC || mBody2->getType() == BodyType::DYNAMIC) {
        mInverseMassMatrixTranslation = massMatrix.getInverse();
    }

    // Compute the bias "b" of the constraint for the 3 translation constraints
    decimal biasFactor = (BETA / constraintSolverData.timeStep);
    mBiasTranslation.setToZero();
    if (mPositionCorrectionTechnique == JointsPositionCorrectionTechnique::BAUMGARTE_JOINTS) {
        mBiasTranslation = biasFactor * (x2 + mR2World - x1 - mR1World);
    }

    // Compute the inverse of the mass matrix K=JM^-1J^t for the 3 rotation
    // contraints (3x3 matrix)
    mInverseMassMatrixRotation = mI1 + mI2;
    if (mBody1->getType() == BodyType::DYNAMIC || mBody2->getType() == BodyType::DYNAMIC) {
        mInverseMassMatrixRotation = mInverseMassMatrixRotation.getInverse();
    }

    // Compute the bias "b" for the 3 rotation constraints
    mBiasRotation.setToZero();

    if (mPositionCorrectionTechnique == JointsPositionCorrectionTechnique::BAUMGARTE_JOINTS) {
        const Quaternion qError = orientationBody2 * mInitOrientationDifferenceInv * orientationBody1.getInverse();
        mBiasRotation = biasFactor * decimal(2.0) * qError.getVectorV();
    }

    // If warm-starting is not enabled
    if (!constraintSolverData.isWarmStartingActive) {

        // Reset the accumulated impulses
        mImpulseTranslation.setToZero();
        mImpulseRotation.setToZero();
    }
}

// Warm start the constraint (apply the previous impulse at the beginning of the step)
void FixedJoint::warmstart(const ConstraintSolverData& constraintSolverData) {

    // Get the velocities
    Vector3& v1 = constraintSolverData.linearVelocities[mIndexBody1];
    Vector3& v2 = constraintSolverData.linearVelocities[mIndexBody2];
    Vector3& w1 = constraintSolverData.angularVelocities[mIndexBody1];
    Vector3& w2 = constraintSolverData.angularVelocities[mIndexBody2];

    // Get the inverse mass of the bodies
    const decimal inverseMassBody1 = mBody1->mMassInverse;
    const decimal inverseMassBody2 = mBody2->mMassInverse;

    // Compute the impulse P=J^T * lambda for the 3 translation constraints for body 1
    Vector3 linearImpulseBody1 = -mImpulseTranslation;
    Vector3 angularImpulseBody1 = mImpulseTranslation.cross(mR1World);

    // Compute the impulse P=J^T * lambda for the 3 rotation constraints for body 1
    angularImpulseBody1 += -mImpulseRotation;

    // Apply the impulse to the body 1
    v1 += inverseMassBody1 * linearImpulseBody1;
    w1 += mI1 * angularImpulseBody1;

    // Compute the impulse P=J^T * lambda for the 3 translation constraints for body 2
    Vector3 angularImpulseBody2 = -mImpulseTranslation.cross(mR2World);

    // Compute the impulse P=J^T * lambda for the 3 rotation constraints for body 2
    angularImpulseBody2 += mImpulseRotation;

    // Apply the impulse to the body 2
    v2 += inverseMassBody2 * mImpulseTranslation;
    w2 += mI2 * angularImpulseBody2;
}

// Solve the velocity constraint
void FixedJoint::solveVelocityConstraint(const ConstraintSolverData& constraintSolverData) {

    // Get the velocities
    Vector3& v1 = constraintSolverData.linearVelocities[mIndexBody1];
    Vector3& v2 = constraintSolverData.linearVelocities[mIndexBody2];
    Vector3& w1 = constraintSolverData.angularVelocities[mIndexBody1];
    Vector3& w2 = constraintSolverData.angularVelocities[mIndexBody2];

    // Get the inverse mass of the bodies
    decimal inverseMassBody1 = mBody1->mMassInverse;
    decimal inverseMassBody2 = mBody2->mMassInverse;

    // --------------- Translation Constraints --------------- //

    // Compute J*v for the 3 translation constraints
    const Vector3 JvTranslation = v2 + w2.cross(mR2World) - v1 - w1.cross(mR1World);

    // Compute the Lagrange multiplier lambda
    const Vector3 deltaLambda = mInverseMassMatrixTranslation *
                               (-JvTranslation - mBiasTranslation);
    mImpulseTranslation += deltaLambda;

    // Compute the impulse P=J^T * lambda for body 1
    const Vector3 linearImpulseBody1 = -deltaLambda;
    Vector3 angularImpulseBody1 = deltaLambda.cross(mR1World);

    // Apply the impulse to the body 1
    v1 += inverseMassBody1 * linearImpulseBody1;
    w1 += mI1 * angularImpulseBody1;

    // Compute the impulse P=J^T * lambda  for body 2
    const Vector3 angularImpulseBody2 = -deltaLambda.cross(mR2World);

    // Apply the impulse to the body 2
    v2 += inverseMassBody2 * deltaLambda;
    w2 += mI2 * angularImpulseBody2;

    // --------------- Rotation Constraints --------------- //

    // Compute J*v for the 3 rotation constraints
    const Vector3 JvRotation = w2 - w1;

    // Compute the Lagrange multiplier lambda for the 3 rotation constraints
    Vector3 deltaLambda2 = mInverseMassMatrixRotation * (-JvRotation - mBiasRotation);
    mImpulseRotation += deltaLambda2;

    // Compute the impulse P=J^T * lambda for the 3 rotation constraints for body 1
    angularImpulseBody1 = -deltaLambda2;

    // Apply the impulse to the body 1
    w1 += mI1 * angularImpulseBody1;

    // Apply the impulse to the body 2
    w2 += mI2 * deltaLambda2;
}

// Solve the position constraint (for position error correction)
void FixedJoint::solvePositionConstraint(const ConstraintSolverData& constraintSolverData) {

    // If the error position correction technique is not the non-linear-gauss-seidel, we do
    // do not execute this method
    if (mPositionCorrectionTechnique != JointsPositionCorrectionTechnique::NON_LINEAR_GAUSS_SEIDEL) return;

    // Get the bodies positions and orientations
    Vector3& x1 = constraintSolverData.positions[mIndexBody1];
    Vector3& x2 = constraintSolverData.positions[mIndexBody2];
    Quaternion& q1 = constraintSolverData.orientations[mIndexBody1];
    Quaternion& q2 = constraintSolverData.orientations[mIndexBody2];

    // Get the inverse mass and inverse inertia tensors of the bodies
    decimal inverseMassBody1 = mBody1->mMassInverse;
    decimal inverseMassBody2 = mBody2->mMassInverse;

    // Recompute the inverse inertia tensors
    mI1 = mBody1->getInertiaTensorInverseWorld();
    mI2 = mBody2->getInertiaTensorInverseWorld();

    // Compute the vector from body center to the anchor point in world-space
    mR1World = q1 * mLocalAnchorPointBody1;
    mR2World = q2 * mLocalAnchorPointBody2;

    // Compute the corresponding skew-symmetric matrices
    Matrix3x3 skewSymmetricMatrixU1= Matrix3x3::computeSkewSymmetricMatrixForCrossProduct(mR1World);
    Matrix3x3 skewSymmetricMatrixU2= Matrix3x3::computeSkewSymmetricMatrixForCrossProduct(mR2World);

    // --------------- Translation Constraints --------------- //

    // Compute the matrix K=JM^-1J^t (3x3 matrix) for the 3 translation constraints
    decimal inverseMassBodies = mBody1->mMassInverse + mBody2->mMassInverse;
    Matrix3x3 massMatrix = Matrix3x3(inverseMassBodies, 0, 0,
                                    0, inverseMassBodies, 0,
                                    0, 0, inverseMassBodies) +
                           skewSymmetricMatrixU1 * mI1 * skewSymmetricMatrixU1.getTranspose() +
                           skewSymmetricMatrixU2 * mI2 * skewSymmetricMatrixU2.getTranspose();
    mInverseMassMatrixTranslation.setToZero();
    if (mBody1->getType() == BodyType::DYNAMIC || mBody2->getType() == BodyType::DYNAMIC) {
        mInverseMassMatrixTranslation = massMatrix.getInverse();
    }

    // Compute position error for the 3 translation constraints
    const Vector3 errorTranslation = x2 + mR2World - x1 - mR1World;

    // Compute the Lagrange multiplier lambda
    const Vector3 lambdaTranslation = mInverseMassMatrixTranslation * (-errorTranslation);

    // Compute the impulse of body 1
    Vector3 linearImpulseBody1 = -lambdaTranslation;
    Vector3 angularImpulseBody1 = lambdaTranslation.cross(mR1World);

    // Compute the pseudo velocity of body 1
    const Vector3 v1 = inverseMassBody1 * linearImpulseBody1;
    Vector3 w1 = mI1 * angularImpulseBody1;

    // Update the body position/orientation of body 1
    x1 += v1;
    q1 += Quaternion(0, w1) * q1 * decimal(0.5);
    q1.normalize();

    // Compute the impulse of body 2
    Vector3 angularImpulseBody2 = -lambdaTranslation.cross(mR2World);

    // Compute the pseudo velocity of body 2
    const Vector3 v2 = inverseMassBody2 * lambdaTranslation;
    Vector3 w2 = mI2 * angularImpulseBody2;

    // Update the body position/orientation of body 2
    x2 += v2;
    q2 += Quaternion(0, w2) * q2 * decimal(0.5);
    q2.normalize();

    // --------------- Rotation Constraints --------------- //

    // Compute the inverse of the mass matrix K=JM^-1J^t for the 3 rotation
    // contraints (3x3 matrix)
    mInverseMassMatrixRotation = mI1 + mI2;
    if (mBody1->getType() == BodyType::DYNAMIC || mBody2->getType() == BodyType::DYNAMIC) {
        mInverseMassMatrixRotation = mInverseMassMatrixRotation.getInverse();
    }

	// Calculate difference in rotation
	//
	// The rotation should be:
	//
	// q2 = q1 r0
	//
	// But because of drift the actual rotation is:
	//
	// q2 = qError q1 r0
	// <=> qError = q2 r0^-1 q1^-1
	//
	// Where:
	// q1 = current rotation of body 1
	// q2 = current rotation of body 2
	// qError = error that needs to be reduced to zero
	Quaternion qError = q2 * mInitOrientationDifferenceInv * q1.getInverse();

	// A quaternion can be seen as:
	//
	// q = [sin(theta / 2) * v, cos(theta/2)]
	//
	// Where:
	// v = rotation vector
	// theta = rotation angle
	// 
	// If we assume theta is small (error is small) then sin(x) = x so an approximation of the error angles is:
    const Vector3 errorRotation = decimal(2.0) * qError.getVectorV();

    // Compute the Lagrange multiplier lambda for the 3 rotation constraints
    Vector3 lambdaRotation = mInverseMassMatrixRotation * (-errorRotation);

    // Compute the impulse P=J^T * lambda for the 3 rotation constraints of body 1
    angularImpulseBody1 = -lambdaRotation;

    // Compute the pseudo velocity of body 1
    w1 = mI1 * angularImpulseBody1;

    // Update the body position/orientation of body 1
    q1 += Quaternion(0, w1) * q1 * decimal(0.5);
    q1.normalize();

    // Compute the pseudo velocity of body 2
    w2 = mI2 * lambdaRotation;

    // Update the body position/orientation of body 2
    q2 += Quaternion(0, w2) * q2 * decimal(0.5);
    q2.normalize();
}

