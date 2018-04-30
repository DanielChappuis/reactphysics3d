/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2018 Daniel Chappuis                                       *
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
#include "SliderJoint.h"
#include "engine/ConstraintSolver.h"

using namespace reactphysics3d;

// Static variables definition
const decimal SliderJoint::BETA = decimal(0.2);

// Constructor
SliderJoint::SliderJoint(uint id, const SliderJointInfo& jointInfo)
            : Joint(id, jointInfo), mImpulseTranslation(0, 0), mImpulseRotation(0, 0, 0),
              mImpulseLowerLimit(0), mImpulseUpperLimit(0), mImpulseMotor(0),
              mIsLimitEnabled(jointInfo.isLimitEnabled), mIsMotorEnabled(jointInfo.isMotorEnabled),
              mLowerLimit(jointInfo.minTranslationLimit),
              mUpperLimit(jointInfo.maxTranslationLimit), mIsLowerLimitViolated(false),
              mIsUpperLimitViolated(false), mMotorSpeed(jointInfo.motorSpeed),
              mMaxMotorForce(jointInfo.maxMotorForce){

    assert(mUpperLimit >= decimal(0.0));
    assert(mLowerLimit <= decimal(0.0));
    assert(mMaxMotorForce >= decimal(0.0));

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

    // Compute the slider axis in local-space of body 1
    mSliderAxisBody1 = mBody1->getTransform().getOrientation().getInverse() *
                       jointInfo.sliderAxisWorldSpace;
    mSliderAxisBody1.normalize();
}

// Initialize before solving the constraint
void SliderJoint::initBeforeSolve(const ConstraintSolverData& constraintSolverData) {

    // Initialize the bodies index in the veloc ity array
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

    // Vector from body center to the anchor point
    mR1 = orientationBody1 * mLocalAnchorPointBody1;
    mR2 = orientationBody2 * mLocalAnchorPointBody2;

    // Compute the vector u (difference between anchor points)
    const Vector3 u = x2 + mR2 - x1 - mR1;

    // Compute the two orthogonal vectors to the slider axis in world-space
    mSliderAxisWorld = orientationBody1 * mSliderAxisBody1;
    mSliderAxisWorld.normalize();
    mN1 = mSliderAxisWorld.getOneUnitOrthogonalVector();
    mN2 = mSliderAxisWorld.cross(mN1);

    // Check if the limit constraints are violated or not
    decimal uDotSliderAxis = u.dot(mSliderAxisWorld);
    decimal lowerLimitError = uDotSliderAxis - mLowerLimit;
    decimal upperLimitError = mUpperLimit - uDotSliderAxis;
    bool oldIsLowerLimitViolated = mIsLowerLimitViolated;
    mIsLowerLimitViolated = lowerLimitError <= 0;
    if (mIsLowerLimitViolated != oldIsLowerLimitViolated) {
        mImpulseLowerLimit = 0.0;
    }
    bool oldIsUpperLimitViolated = mIsUpperLimitViolated;
    mIsUpperLimitViolated = upperLimitError <= 0;
    if (mIsUpperLimitViolated != oldIsUpperLimitViolated) {
        mImpulseUpperLimit = 0.0;
    }

    // Compute the cross products used in the Jacobians
    mR2CrossN1 = mR2.cross(mN1);
    mR2CrossN2 = mR2.cross(mN2);
    mR2CrossSliderAxis = mR2.cross(mSliderAxisWorld);
    const Vector3 r1PlusU = mR1 + u;
    mR1PlusUCrossN1 = (r1PlusU).cross(mN1);
    mR1PlusUCrossN2 = (r1PlusU).cross(mN2);
    mR1PlusUCrossSliderAxis = (r1PlusU).cross(mSliderAxisWorld);

    // Compute the inverse of the mass matrix K=JM^-1J^t for the 2 translation
    // constraints (2x2 matrix)
    decimal sumInverseMass = mBody1->mMassInverse + mBody2->mMassInverse;
    Vector3 I1R1PlusUCrossN1 = mI1 * mR1PlusUCrossN1;
    Vector3 I1R1PlusUCrossN2 = mI1 * mR1PlusUCrossN2;
    Vector3 I2R2CrossN1 = mI2 * mR2CrossN1;
    Vector3 I2R2CrossN2 = mI2 * mR2CrossN2;
    const decimal el11 = sumInverseMass + mR1PlusUCrossN1.dot(I1R1PlusUCrossN1) +
                         mR2CrossN1.dot(I2R2CrossN1);
    const decimal el12 = mR1PlusUCrossN1.dot(I1R1PlusUCrossN2) +
                         mR2CrossN1.dot(I2R2CrossN2);
    const decimal el21 = mR1PlusUCrossN2.dot(I1R1PlusUCrossN1) +
                         mR2CrossN2.dot(I2R2CrossN1);
    const decimal el22 = sumInverseMass + mR1PlusUCrossN2.dot(I1R1PlusUCrossN2) +
                         mR2CrossN2.dot(I2R2CrossN2);
    Matrix2x2 matrixKTranslation(el11, el12, el21, el22);
    mInverseMassMatrixTranslationConstraint.setToZero();
    if (mBody1->getType() == BodyType::DYNAMIC || mBody2->getType() == BodyType::DYNAMIC) {
        mInverseMassMatrixTranslationConstraint = matrixKTranslation.getInverse();
    }

    // Compute the bias "b" of the translation constraint
    mBTranslation.setToZero();
    decimal biasFactor = (BETA / constraintSolverData.timeStep);
    if (mPositionCorrectionTechnique == JointsPositionCorrectionTechnique::BAUMGARTE_JOINTS) {
        mBTranslation.x = u.dot(mN1);
        mBTranslation.y = u.dot(mN2);
        mBTranslation *= biasFactor;
    }

    // Compute the inverse of the mass matrix K=JM^-1J^t for the 3 rotation
    // contraints (3x3 matrix)
    mInverseMassMatrixRotationConstraint = mI1 + mI2;
    if (mBody1->getType() == BodyType::DYNAMIC || mBody2->getType() == BodyType::DYNAMIC) {
        mInverseMassMatrixRotationConstraint = mInverseMassMatrixRotationConstraint.getInverse();
    }

    // Compute the bias "b" of the rotation constraint
    mBRotation.setToZero();
    if (mPositionCorrectionTechnique == JointsPositionCorrectionTechnique::BAUMGARTE_JOINTS) {
        const Quaternion qError = orientationBody2 * mInitOrientationDifferenceInv * orientationBody1.getInverse();
        mBRotation = biasFactor * decimal(2.0) * qError.getVectorV();
    }

    // If the limits are enabled
    if (mIsLimitEnabled && (mIsLowerLimitViolated || mIsUpperLimitViolated)) {

        // Compute the inverse of the mass matrix K=JM^-1J^t for the limits (1x1 matrix)
        mInverseMassMatrixLimit = mBody1->mMassInverse + mBody2->mMassInverse +
                                  mR1PlusUCrossSliderAxis.dot(mI1 * mR1PlusUCrossSliderAxis) +
                                  mR2CrossSliderAxis.dot(mI2 * mR2CrossSliderAxis);
        mInverseMassMatrixLimit = (mInverseMassMatrixLimit > 0.0) ?
                                  decimal(1.0) / mInverseMassMatrixLimit : decimal(0.0);

        // Compute the bias "b" of the lower limit constraint
        mBLowerLimit = 0.0;
        if (mPositionCorrectionTechnique == JointsPositionCorrectionTechnique::BAUMGARTE_JOINTS) {
            mBLowerLimit = biasFactor * lowerLimitError;
        }

        // Compute the bias "b" of the upper limit constraint
        mBUpperLimit = 0.0;
        if (mPositionCorrectionTechnique == JointsPositionCorrectionTechnique::BAUMGARTE_JOINTS) {
            mBUpperLimit = biasFactor * upperLimitError;
        }
    }

    // If the motor is enabled
    if (mIsMotorEnabled) {

        // Compute the inverse of mass matrix K=JM^-1J^t for the motor (1x1 matrix)
        mInverseMassMatrixMotor = mBody1->mMassInverse + mBody2->mMassInverse;
        mInverseMassMatrixMotor = (mInverseMassMatrixMotor > 0.0) ?
                    decimal(1.0) / mInverseMassMatrixMotor : decimal(0.0);
    }

    // If warm-starting is not enabled
    if (!constraintSolverData.isWarmStartingActive) {

        // Reset all the accumulated impulses
        mImpulseTranslation.setToZero();
        mImpulseRotation.setToZero();
        mImpulseLowerLimit = 0.0;
        mImpulseUpperLimit = 0.0;
        mImpulseMotor = 0.0;
    }
}

// Warm start the constraint (apply the previous impulse at the beginning of the step)
void SliderJoint::warmstart(const ConstraintSolverData& constraintSolverData) {

    // Get the velocities
    Vector3& v1 = constraintSolverData.linearVelocities[mIndexBody1];
    Vector3& v2 = constraintSolverData.linearVelocities[mIndexBody2];
    Vector3& w1 = constraintSolverData.angularVelocities[mIndexBody1];
    Vector3& w2 = constraintSolverData.angularVelocities[mIndexBody2];

    // Get the inverse mass and inverse inertia tensors of the bodies
    const decimal inverseMassBody1 = mBody1->mMassInverse;
    const decimal inverseMassBody2 = mBody2->mMassInverse;

    // Compute the impulse P=J^T * lambda for the lower and upper limits constraints of body 1
    decimal impulseLimits = mImpulseUpperLimit - mImpulseLowerLimit;
    Vector3 linearImpulseLimits = impulseLimits * mSliderAxisWorld;

    // Compute the impulse P=J^T * lambda for the motor constraint of body 1
    Vector3 impulseMotor = mImpulseMotor * mSliderAxisWorld;

    // Compute the impulse P=J^T * lambda for the 2 translation constraints of body 1
    Vector3 linearImpulseBody1 = -mN1 * mImpulseTranslation.x - mN2 * mImpulseTranslation.y;
    Vector3 angularImpulseBody1 = -mR1PlusUCrossN1 * mImpulseTranslation.x -
            mR1PlusUCrossN2 * mImpulseTranslation.y;

    // Compute the impulse P=J^T * lambda for the 3 rotation constraints of body 1
    angularImpulseBody1 += -mImpulseRotation;

    // Compute the impulse P=J^T * lambda for the lower and upper limits constraints of body 1
    linearImpulseBody1 += linearImpulseLimits;
    angularImpulseBody1 += impulseLimits * mR1PlusUCrossSliderAxis;

    // Compute the impulse P=J^T * lambda for the motor constraint of body 1
    linearImpulseBody1 += impulseMotor;

    // Apply the impulse to the body 1
    v1 += inverseMassBody1 * linearImpulseBody1;
    w1 += mI1 * angularImpulseBody1;

    // Compute the impulse P=J^T * lambda for the 2 translation constraints of body 2
    Vector3 linearImpulseBody2 = mN1 * mImpulseTranslation.x + mN2 * mImpulseTranslation.y;
    Vector3 angularImpulseBody2 = mR2CrossN1 * mImpulseTranslation.x +
            mR2CrossN2 * mImpulseTranslation.y;

    // Compute the impulse P=J^T * lambda for the 3 rotation constraints of body 2
    angularImpulseBody2 += mImpulseRotation;

    // Compute the impulse P=J^T * lambda for the lower and upper limits constraints of body 2
    linearImpulseBody2 += -linearImpulseLimits;
    angularImpulseBody2 += -impulseLimits * mR2CrossSliderAxis;

    // Compute the impulse P=J^T * lambda for the motor constraint of body 2
    linearImpulseBody2 += -impulseMotor;

    // Apply the impulse to the body 2
    v2 += inverseMassBody2 * linearImpulseBody2;
    w2 += mI2 * angularImpulseBody2;
}

// Solve the velocity constraint
void SliderJoint::solveVelocityConstraint(const ConstraintSolverData& constraintSolverData) {

    // Get the velocities
    Vector3& v1 = constraintSolverData.linearVelocities[mIndexBody1];
    Vector3& v2 = constraintSolverData.linearVelocities[mIndexBody2];
    Vector3& w1 = constraintSolverData.angularVelocities[mIndexBody1];
    Vector3& w2 = constraintSolverData.angularVelocities[mIndexBody2];

    // Get the inverse mass and inverse inertia tensors of the bodies
    decimal inverseMassBody1 = mBody1->mMassInverse;
    decimal inverseMassBody2 = mBody2->mMassInverse;

    // --------------- Translation Constraints --------------- //

    // Compute J*v for the 2 translation constraints
    const decimal el1 = -mN1.dot(v1) - w1.dot(mR1PlusUCrossN1) +
                         mN1.dot(v2) + w2.dot(mR2CrossN1);
    const decimal el2 = -mN2.dot(v1) - w1.dot(mR1PlusUCrossN2) +
                         mN2.dot(v2) + w2.dot(mR2CrossN2);
    const Vector2 JvTranslation(el1, el2);

    // Compute the Lagrange multiplier lambda for the 2 translation constraints
    Vector2 deltaLambda = mInverseMassMatrixTranslationConstraint * (-JvTranslation -mBTranslation);
    mImpulseTranslation += deltaLambda;

    // Compute the impulse P=J^T * lambda for the 2 translation constraints of body 1
    const Vector3 linearImpulseBody1 = -mN1 * deltaLambda.x - mN2 * deltaLambda.y;
    Vector3 angularImpulseBody1 = -mR1PlusUCrossN1 * deltaLambda.x -
            mR1PlusUCrossN2 * deltaLambda.y;

    // Apply the impulse to the body 1
    v1 += inverseMassBody1 * linearImpulseBody1;
    w1 += mI1 * angularImpulseBody1;

    // Compute the impulse P=J^T * lambda for the 2 translation constraints of body 2
    const Vector3 linearImpulseBody2 = mN1 * deltaLambda.x + mN2 * deltaLambda.y;
    Vector3 angularImpulseBody2 = mR2CrossN1 * deltaLambda.x + mR2CrossN2 * deltaLambda.y;

    // Apply the impulse to the body 2
    v2 += inverseMassBody2 * linearImpulseBody2;
    w2 += mI2 * angularImpulseBody2;

    // --------------- Rotation Constraints --------------- //

    // Compute J*v for the 3 rotation constraints
    const Vector3 JvRotation = w2 - w1;

    // Compute the Lagrange multiplier lambda for the 3 rotation constraints
    Vector3 deltaLambda2 = mInverseMassMatrixRotationConstraint * (-JvRotation - mBRotation);
    mImpulseRotation += deltaLambda2;

    // Compute the impulse P=J^T * lambda for the 3 rotation constraints of body 1
    angularImpulseBody1 = -deltaLambda2;

    // Apply the impulse to the body to body 1
    w1 += mI1 * angularImpulseBody1;

    // Compute the impulse P=J^T * lambda for the 3 rotation constraints of body 2
    angularImpulseBody2 = deltaLambda2;

    // Apply the impulse to the body 2
    w2 += mI2 * angularImpulseBody2;

    // --------------- Limits Constraints --------------- //

    if (mIsLimitEnabled) {

        // If the lower limit is violated
        if (mIsLowerLimitViolated) {

            // Compute J*v for the lower limit constraint
            const decimal JvLowerLimit = mSliderAxisWorld.dot(v2) + mR2CrossSliderAxis.dot(w2) -
                                         mSliderAxisWorld.dot(v1) - mR1PlusUCrossSliderAxis.dot(w1);

            // Compute the Lagrange multiplier lambda for the lower limit constraint
            decimal deltaLambdaLower = mInverseMassMatrixLimit * (-JvLowerLimit -mBLowerLimit);
            decimal lambdaTemp = mImpulseLowerLimit;
            mImpulseLowerLimit = std::max(mImpulseLowerLimit + deltaLambdaLower, decimal(0.0));
            deltaLambdaLower = mImpulseLowerLimit - lambdaTemp;

            // Compute the impulse P=J^T * lambda for the lower limit constraint of body 1
            const Vector3 linearImpulseBody1 = -deltaLambdaLower * mSliderAxisWorld;
            const Vector3 angularImpulseBody1 = -deltaLambdaLower * mR1PlusUCrossSliderAxis;

            // Apply the impulse to the body 1
            v1 += inverseMassBody1 * linearImpulseBody1;
            w1 += mI1 * angularImpulseBody1;

            // Compute the impulse P=J^T * lambda for the lower limit constraint of body 2
            const Vector3 linearImpulseBody2 = deltaLambdaLower * mSliderAxisWorld;
            const Vector3 angularImpulseBody2 = deltaLambdaLower * mR2CrossSliderAxis;

            // Apply the impulse to the body 2
            v2 += inverseMassBody2 * linearImpulseBody2;
            w2 += mI2 * angularImpulseBody2;
        }

        // If the upper limit is violated
        if (mIsUpperLimitViolated) {

            // Compute J*v for the upper limit constraint
            const decimal JvUpperLimit = mSliderAxisWorld.dot(v1) + mR1PlusUCrossSliderAxis.dot(w1)
                                        - mSliderAxisWorld.dot(v2) - mR2CrossSliderAxis.dot(w2);

            // Compute the Lagrange multiplier lambda for the upper limit constraint
            decimal deltaLambdaUpper = mInverseMassMatrixLimit * (-JvUpperLimit -mBUpperLimit);
            decimal lambdaTemp = mImpulseUpperLimit;
            mImpulseUpperLimit = std::max(mImpulseUpperLimit + deltaLambdaUpper, decimal(0.0));
            deltaLambdaUpper = mImpulseUpperLimit - lambdaTemp;

            // Compute the impulse P=J^T * lambda for the upper limit constraint of body 1
            const Vector3 linearImpulseBody1 = deltaLambdaUpper * mSliderAxisWorld;
            const Vector3 angularImpulseBody1 = deltaLambdaUpper * mR1PlusUCrossSliderAxis;

            // Apply the impulse to the body 1
            v1 += inverseMassBody1 * linearImpulseBody1;
            w1 += mI1 * angularImpulseBody1;

            // Compute the impulse P=J^T * lambda for the upper limit constraint of body 2
            const Vector3 linearImpulseBody2 = -deltaLambdaUpper * mSliderAxisWorld;
            const Vector3 angularImpulseBody2 = -deltaLambdaUpper * mR2CrossSliderAxis;

            // Apply the impulse to the body 2
            v2 += inverseMassBody2 * linearImpulseBody2;
            w2 += mI2 * angularImpulseBody2;
        }
    }

    // --------------- Motor --------------- //

    if (mIsMotorEnabled) {

        // Compute J*v for the motor
        const decimal JvMotor = mSliderAxisWorld.dot(v1) - mSliderAxisWorld.dot(v2);

        // Compute the Lagrange multiplier lambda for the motor
        const decimal maxMotorImpulse = mMaxMotorForce * constraintSolverData.timeStep;
        decimal deltaLambdaMotor = mInverseMassMatrixMotor * (-JvMotor - mMotorSpeed);
        decimal lambdaTemp = mImpulseMotor;
        mImpulseMotor = clamp(mImpulseMotor + deltaLambdaMotor, -maxMotorImpulse, maxMotorImpulse);
        deltaLambdaMotor = mImpulseMotor - lambdaTemp;

        // Compute the impulse P=J^T * lambda for the motor of body 1
        const Vector3 linearImpulseBody1 = deltaLambdaMotor * mSliderAxisWorld;

        // Apply the impulse to the body 1
        v1 += inverseMassBody1 * linearImpulseBody1;

        // Compute the impulse P=J^T * lambda for the motor of body 2
        const Vector3 linearImpulseBody2 = -deltaLambdaMotor * mSliderAxisWorld;

        // Apply the impulse to the body 2
        v2 += inverseMassBody2 * linearImpulseBody2;
    }
}

// Solve the position constraint (for position error correction)
void SliderJoint::solvePositionConstraint(const ConstraintSolverData& constraintSolverData) {

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

    // Recompute the inertia tensor of bodies
    mI1 = mBody1->getInertiaTensorInverseWorld();
    mI2 = mBody2->getInertiaTensorInverseWorld();

    // Vector from body center to the anchor point
    mR1 = q1 * mLocalAnchorPointBody1;
    mR2 = q2 * mLocalAnchorPointBody2;

    // Compute the vector u (difference between anchor points)
    const Vector3 u = x2 + mR2 - x1 - mR1;

    // Compute the two orthogonal vectors to the slider axis in world-space
    mSliderAxisWorld = q1 * mSliderAxisBody1;
    mSliderAxisWorld.normalize();
    mN1 = mSliderAxisWorld.getOneUnitOrthogonalVector();
    mN2 = mSliderAxisWorld.cross(mN1);

    // Check if the limit constraints are violated or not
    decimal uDotSliderAxis = u.dot(mSliderAxisWorld);
    decimal lowerLimitError = uDotSliderAxis - mLowerLimit;
    decimal upperLimitError = mUpperLimit - uDotSliderAxis;
    mIsLowerLimitViolated = lowerLimitError <= 0;
    mIsUpperLimitViolated = upperLimitError <= 0;

    // Compute the cross products used in the Jacobians
    mR2CrossN1 = mR2.cross(mN1);
    mR2CrossN2 = mR2.cross(mN2);
    mR2CrossSliderAxis = mR2.cross(mSliderAxisWorld);
    const Vector3 r1PlusU = mR1 + u;
    mR1PlusUCrossN1 = (r1PlusU).cross(mN1);
    mR1PlusUCrossN2 = (r1PlusU).cross(mN2);
    mR1PlusUCrossSliderAxis = (r1PlusU).cross(mSliderAxisWorld);

    // --------------- Translation Constraints --------------- //

    // Recompute the inverse of the mass matrix K=JM^-1J^t for the 2 translation
    // constraints (2x2 matrix)
    decimal sumInverseMass = mBody1->mMassInverse + mBody2->mMassInverse;
    Vector3 I1R1PlusUCrossN1 = mI1 * mR1PlusUCrossN1;
    Vector3 I1R1PlusUCrossN2 = mI1 * mR1PlusUCrossN2;
    Vector3 I2R2CrossN1 = mI2 * mR2CrossN1;
    Vector3 I2R2CrossN2 = mI2 * mR2CrossN2;
    const decimal el11 = sumInverseMass + mR1PlusUCrossN1.dot(I1R1PlusUCrossN1) +
                         mR2CrossN1.dot(I2R2CrossN1);
    const decimal el12 = mR1PlusUCrossN1.dot(I1R1PlusUCrossN2) +
                         mR2CrossN1.dot(I2R2CrossN2);
    const decimal el21 = mR1PlusUCrossN2.dot(I1R1PlusUCrossN1) +
                         mR2CrossN2.dot(I2R2CrossN1);
    const decimal el22 = sumInverseMass + mR1PlusUCrossN2.dot(I1R1PlusUCrossN2) +
                         mR2CrossN2.dot(I2R2CrossN2);
    Matrix2x2 matrixKTranslation(el11, el12, el21, el22);
    mInverseMassMatrixTranslationConstraint.setToZero();
    if (mBody1->getType() == BodyType::DYNAMIC || mBody2->getType() == BodyType::DYNAMIC) {
        mInverseMassMatrixTranslationConstraint = matrixKTranslation.getInverse();
    }

    // Compute the position error for the 2 translation constraints
    const Vector2 translationError(u.dot(mN1), u.dot(mN2));

    // Compute the Lagrange multiplier lambda for the 2 translation constraints
    Vector2 lambdaTranslation = mInverseMassMatrixTranslationConstraint * (-translationError);

    // Compute the impulse P=J^T * lambda for the 2 translation constraints of body 1
    const Vector3 linearImpulseBody1 = -mN1 * lambdaTranslation.x - mN2 * lambdaTranslation.y;
    Vector3 angularImpulseBody1 = -mR1PlusUCrossN1 * lambdaTranslation.x -
                                        mR1PlusUCrossN2 * lambdaTranslation.y;

    // Apply the impulse to the body 1
    const Vector3 v1 = inverseMassBody1 * linearImpulseBody1;
    Vector3 w1 = mI1 * angularImpulseBody1;

    // Update the body position/orientation of body 1
    x1 += v1;
    q1 += Quaternion(0, w1) * q1 * decimal(0.5);
    q1.normalize();

    // Compute the impulse P=J^T * lambda for the 2 translation constraints of body 2
    const Vector3 linearImpulseBody2 = mN1 * lambdaTranslation.x + mN2 * lambdaTranslation.y;
    Vector3 angularImpulseBody2 = mR2CrossN1 * lambdaTranslation.x +
            mR2CrossN2 * lambdaTranslation.y;

    // Apply the impulse to the body 2
    const Vector3 v2 = inverseMassBody2 * linearImpulseBody2;
    Vector3 w2 = mI2 * angularImpulseBody2;

    // Update the body position/orientation of body 2
    x2 += v2;
    q2 += Quaternion(0, w2) * q2 * decimal(0.5);
    q2.normalize();

    // --------------- Rotation Constraints --------------- //

    // Compute the inverse of the mass matrix K=JM^-1J^t for the 3 rotation
    // contraints (3x3 matrix)
    mInverseMassMatrixRotationConstraint = mI1 + mI2;
    if (mBody1->getType() == BodyType::DYNAMIC || mBody2->getType() == BodyType::DYNAMIC) {
        mInverseMassMatrixRotationConstraint = mInverseMassMatrixRotationConstraint.getInverse();
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
    Vector3 lambdaRotation = mInverseMassMatrixRotationConstraint * (-errorRotation);

    // Compute the impulse P=J^T * lambda for the 3 rotation constraints of body 1
    angularImpulseBody1 = -lambdaRotation;

    // Apply the impulse to the body 1
    w1 = mI1 * angularImpulseBody1;

    // Update the body position/orientation of body 1
    q1 += Quaternion(0, w1) * q1 * decimal(0.5);
    q1.normalize();

    // Compute the impulse P=J^T * lambda for the 3 rotation constraints of body 2
    angularImpulseBody2 = lambdaRotation;

    // Apply the impulse to the body 2
    w2 = mI2 * angularImpulseBody2;

    // Update the body position/orientation of body 2
    q2 += Quaternion(0, w2) * q2 * decimal(0.5);
    q2.normalize();

    // --------------- Limits Constraints --------------- //

    if (mIsLimitEnabled) {

        if (mIsLowerLimitViolated || mIsUpperLimitViolated) {

            // Compute the inverse of the mass matrix K=JM^-1J^t for the limits (1x1 matrix)
            mInverseMassMatrixLimit = mBody1->mMassInverse + mBody2->mMassInverse +
                                    mR1PlusUCrossSliderAxis.dot(mI1 * mR1PlusUCrossSliderAxis) +
                                    mR2CrossSliderAxis.dot(mI2 * mR2CrossSliderAxis);
            mInverseMassMatrixLimit = (mInverseMassMatrixLimit > 0.0) ?
                                      decimal(1.0) / mInverseMassMatrixLimit : decimal(0.0);
        }

        // If the lower limit is violated
        if (mIsLowerLimitViolated) {

            // Compute the Lagrange multiplier lambda for the lower limit constraint
            decimal lambdaLowerLimit = mInverseMassMatrixLimit * (-lowerLimitError);

            // Compute the impulse P=J^T * lambda for the lower limit constraint of body 1
            const Vector3 linearImpulseBody1 = -lambdaLowerLimit * mSliderAxisWorld;
            const Vector3 angularImpulseBody1 = -lambdaLowerLimit * mR1PlusUCrossSliderAxis;

            // Apply the impulse to the body 1
            const Vector3 v1 = inverseMassBody1 * linearImpulseBody1;
            const Vector3 w1 = mI1 * angularImpulseBody1;

            // Update the body position/orientation of body 1
            x1 += v1;
            q1 += Quaternion(0, w1) * q1 * decimal(0.5);
            q1.normalize();

            // Compute the impulse P=J^T * lambda for the lower limit constraint of body 2
            const Vector3 linearImpulseBody2 = lambdaLowerLimit * mSliderAxisWorld;
            const Vector3 angularImpulseBody2 = lambdaLowerLimit * mR2CrossSliderAxis;

            // Apply the impulse to the body 2
            const Vector3 v2 = inverseMassBody2 * linearImpulseBody2;
            const Vector3 w2 = mI2 * angularImpulseBody2;

            // Update the body position/orientation of body 2
            x2 += v2;
            q2 += Quaternion(0, w2) * q2 * decimal(0.5);
            q2.normalize();
        }

        // If the upper limit is violated
        if (mIsUpperLimitViolated) {

            // Compute the Lagrange multiplier lambda for the upper limit constraint
            decimal lambdaUpperLimit = mInverseMassMatrixLimit * (-upperLimitError);

            // Compute the impulse P=J^T * lambda for the upper limit constraint of body 1
            const Vector3 linearImpulseBody1 = lambdaUpperLimit * mSliderAxisWorld;
            const Vector3 angularImpulseBody1 = lambdaUpperLimit * mR1PlusUCrossSliderAxis;

            // Apply the impulse to the body 1
            const Vector3 v1 = inverseMassBody1 * linearImpulseBody1;
            const Vector3 w1 = mI1 * angularImpulseBody1;

            // Update the body position/orientation of body 1
            x1 += v1;
            q1 += Quaternion(0, w1) * q1 * decimal(0.5);
            q1.normalize();

            // Compute the impulse P=J^T * lambda for the upper limit constraint of body 2
            const Vector3 linearImpulseBody2 = -lambdaUpperLimit * mSliderAxisWorld;
            const Vector3 angularImpulseBody2 = -lambdaUpperLimit * mR2CrossSliderAxis;

            // Apply the impulse to the body 2
            const Vector3 v2 = inverseMassBody2 * linearImpulseBody2;
            const Vector3 w2 = mI2 * angularImpulseBody2;

            // Update the body position/orientation of body 2
            x2 += v2;
            q2 += Quaternion(0, w2) * q2 * decimal(0.5);
            q2.normalize();
        }
    }
}

// Enable/Disable the limits of the joint
/**
 * @param isLimitEnabled True if you want to enable the joint limits and false
 *                       otherwise
 */
void SliderJoint::enableLimit(bool isLimitEnabled) {

    if (isLimitEnabled != mIsLimitEnabled) {

        mIsLimitEnabled = isLimitEnabled;

        // Reset the limits
        resetLimits();
    }
}

// Enable/Disable the motor of the joint
/**
 * @param isMotorEnabled True if you want to enable the joint motor and false
 *                       otherwise
 */
void SliderJoint::enableMotor(bool isMotorEnabled) {

    mIsMotorEnabled = isMotorEnabled;
    mImpulseMotor = 0.0;

    // Wake up the two bodies of the joint
    mBody1->setIsSleeping(false);
    mBody2->setIsSleeping(false);
}

// Return the current translation value of the joint
/**
 * @return The current translation distance of the joint (in meters)
 */
decimal SliderJoint::getTranslation() const {

    // TODO : Check if we need to compare rigid body position or center of mass here

    // Get the bodies positions and orientations
    const Vector3& x1 = mBody1->getTransform().getPosition();
    const Vector3& x2 = mBody2->getTransform().getPosition();
    const Quaternion& q1 = mBody1->getTransform().getOrientation();
    const Quaternion& q2 = mBody2->getTransform().getOrientation();

    // Compute the two anchor points in world-space coordinates
    const Vector3 anchorBody1 = x1 + q1 * mLocalAnchorPointBody1;
    const Vector3 anchorBody2 = x2 + q2 * mLocalAnchorPointBody2;

    // Compute the vector u (difference between anchor points)
    const Vector3 u = anchorBody2 - anchorBody1;

    // Compute the slider axis in world-space
    Vector3 sliderAxisWorld = q1 * mSliderAxisBody1;
    sliderAxisWorld.normalize();

    // Compute and return the translation value
    return u.dot(sliderAxisWorld);
}

// Set the minimum translation limit
/**
 * @param lowerLimit The minimum translation limit of the joint (in meters)
 */
void SliderJoint::setMinTranslationLimit(decimal lowerLimit) {

    assert(lowerLimit <= mUpperLimit);

    if (lowerLimit != mLowerLimit) {

        mLowerLimit = lowerLimit;

        // Reset the limits
        resetLimits();
    }
}

// Set the maximum translation limit
/**
 * @param lowerLimit The maximum translation limit of the joint (in meters)
 */
void SliderJoint::setMaxTranslationLimit(decimal upperLimit) {

    assert(mLowerLimit <= upperLimit);

    if (upperLimit != mUpperLimit) {

        mUpperLimit = upperLimit;

        // Reset the limits
        resetLimits();
    }
}

// Reset the limits
void SliderJoint::resetLimits() {

    // Reset the accumulated impulses for the limits
    mImpulseLowerLimit = 0.0;
    mImpulseUpperLimit = 0.0;

    // Wake up the two bodies of the joint
    mBody1->setIsSleeping(false);
    mBody2->setIsSleeping(false);
}

// Set the motor speed
/**
 * @param motorSpeed The speed of the joint motor (in meters per second)
 */
void SliderJoint::setMotorSpeed(decimal motorSpeed) {

    if (motorSpeed != mMotorSpeed) {

        mMotorSpeed = motorSpeed;

        // Wake up the two bodies of the joint
        mBody1->setIsSleeping(false);
        mBody2->setIsSleeping(false);
    }
}

// Set the maximum motor force
/**
 * @param maxMotorForce The maximum force of the joint motor (in Newton x meters)
 */
void SliderJoint::setMaxMotorForce(decimal maxMotorForce) {

    if (maxMotorForce != mMaxMotorForce) {

        assert(mMaxMotorForce >= 0.0);
        mMaxMotorForce = maxMotorForce;

        // Wake up the two bodies of the joint
        mBody1->setIsSleeping(false);
        mBody2->setIsSleeping(false);
    }
}
