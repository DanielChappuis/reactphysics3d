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
#include "HingeJoint.h"
#include "engine/ConstraintSolver.h"

using namespace reactphysics3d;

// Static variables definition
const decimal HingeJoint::BETA = decimal(0.2);

// Constructor
HingeJoint::HingeJoint(uint id, const HingeJointInfo& jointInfo)
           : Joint(id, jointInfo), mImpulseTranslation(0, 0, 0), mImpulseRotation(0, 0),
             mImpulseLowerLimit(0), mImpulseUpperLimit(0), mImpulseMotor(0),
             mIsLimitEnabled(jointInfo.isLimitEnabled), mIsMotorEnabled(jointInfo.isMotorEnabled),
             mLowerLimit(jointInfo.minAngleLimit), mUpperLimit(jointInfo.maxAngleLimit),
             mIsLowerLimitViolated(false), mIsUpperLimitViolated(false),
             mMotorSpeed(jointInfo.motorSpeed), mMaxMotorTorque(jointInfo.maxMotorTorque) {

    assert(mLowerLimit <= decimal(0) && mLowerLimit >= decimal(-2.0) * PI);
    assert(mUpperLimit >= decimal(0) && mUpperLimit <= decimal(2.0) * PI);

    // Compute the local-space anchor point for each body
    Transform transform1 = mBody1->getTransform();
    Transform transform2 = mBody2->getTransform();
    mLocalAnchorPointBody1 = transform1.getInverse() * jointInfo.anchorPointWorldSpace;
    mLocalAnchorPointBody2 = transform2.getInverse() * jointInfo.anchorPointWorldSpace;

    // Compute the local-space hinge axis
    mHingeLocalAxisBody1 = transform1.getOrientation().getInverse() * jointInfo.rotationAxisWorld;
    mHingeLocalAxisBody2 = transform2.getOrientation().getInverse() * jointInfo.rotationAxisWorld;
    mHingeLocalAxisBody1.normalize();
    mHingeLocalAxisBody2.normalize();

    // Compute the inverse of the initial orientation difference between the two bodies
    mInitOrientationDifferenceInv = transform2.getOrientation() *
                                    transform1.getOrientation().getInverse();
    mInitOrientationDifferenceInv.normalize();
    mInitOrientationDifferenceInv.inverse();
}

// Initialize before solving the constraint
void HingeJoint::initBeforeSolve(const ConstraintSolverData& constraintSolverData) {

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

    // Compute the current angle around the hinge axis
    decimal hingeAngle = computeCurrentHingeAngle(orientationBody1, orientationBody2);

    // Check if the limit constraints are violated or not
    decimal lowerLimitError = hingeAngle - mLowerLimit;
    decimal upperLimitError = mUpperLimit - hingeAngle;
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

    // Compute vectors needed in the Jacobian
    mA1 = orientationBody1 * mHingeLocalAxisBody1;
    Vector3 a2 = orientationBody2 * mHingeLocalAxisBody2;
    mA1.normalize();
    a2.normalize();
    const Vector3 b2 = a2.getOneUnitOrthogonalVector();
    const Vector3 c2 = a2.cross(b2);
    mB2CrossA1 = b2.cross(mA1);
    mC2CrossA1 = c2.cross(mA1);

    // Compute the corresponding skew-symmetric matrices
    Matrix3x3 skewSymmetricMatrixU1= Matrix3x3::computeSkewSymmetricMatrixForCrossProduct(mR1World);
    Matrix3x3 skewSymmetricMatrixU2= Matrix3x3::computeSkewSymmetricMatrixForCrossProduct(mR2World);

    // Compute the inverse mass matrix K=JM^-1J^t for the 3 translation constraints (3x3 matrix)
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

    // Compute the bias "b" of the translation constraints
    mBTranslation.setToZero();
    decimal biasFactor = (BETA / constraintSolverData.timeStep);
    if (mPositionCorrectionTechnique == JointsPositionCorrectionTechnique::BAUMGARTE_JOINTS) {
        mBTranslation = biasFactor * (x2 + mR2World - x1 - mR1World);
    }

    // Compute the inverse mass matrix K=JM^-1J^t for the 2 rotation constraints (2x2 matrix)
    Vector3 I1B2CrossA1 = mI1 * mB2CrossA1;
    Vector3 I1C2CrossA1 = mI1 * mC2CrossA1;
    Vector3 I2B2CrossA1 = mI2 * mB2CrossA1;
    Vector3 I2C2CrossA1 = mI2 * mC2CrossA1;
    const decimal el11 = mB2CrossA1.dot(I1B2CrossA1) +
                         mB2CrossA1.dot(I2B2CrossA1);
    const decimal el12 = mB2CrossA1.dot(I1C2CrossA1) +
                         mB2CrossA1.dot(I2C2CrossA1);
    const decimal el21 = mC2CrossA1.dot(I1B2CrossA1) +
                         mC2CrossA1.dot(I2B2CrossA1);
    const decimal el22 = mC2CrossA1.dot(I1C2CrossA1) +
                         mC2CrossA1.dot(I2C2CrossA1);
    const Matrix2x2 matrixKRotation(el11, el12, el21, el22);
    mInverseMassMatrixRotation.setToZero();
    if (mBody1->getType() == BodyType::DYNAMIC || mBody2->getType() == BodyType::DYNAMIC) {
        mInverseMassMatrixRotation = matrixKRotation.getInverse();
    }

    // Compute the bias "b" of the rotation constraints
    mBRotation.setToZero();
    if (mPositionCorrectionTechnique == JointsPositionCorrectionTechnique::BAUMGARTE_JOINTS) {
        mBRotation = biasFactor * Vector2(mA1.dot(b2), mA1.dot(c2));
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

    // If the motor or limits are enabled
    if (mIsMotorEnabled || (mIsLimitEnabled && (mIsLowerLimitViolated || mIsUpperLimitViolated))) {

        // Compute the inverse of the mass matrix K=JM^-1J^t for the limits and motor (1x1 matrix)
        mInverseMassMatrixLimitMotor = mA1.dot(mI1 * mA1) + mA1.dot(mI2 * mA1);
        mInverseMassMatrixLimitMotor = (mInverseMassMatrixLimitMotor > 0.0) ?
                                  decimal(1.0) / mInverseMassMatrixLimitMotor : decimal(0.0);

        if (mIsLimitEnabled) {

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
    }
}

// Warm start the constraint (apply the previous impulse at the beginning of the step)
void HingeJoint::warmstart(const ConstraintSolverData& constraintSolverData) {

    // Get the velocities
    Vector3& v1 = constraintSolverData.linearVelocities[mIndexBody1];
    Vector3& v2 = constraintSolverData.linearVelocities[mIndexBody2];
    Vector3& w1 = constraintSolverData.angularVelocities[mIndexBody1];
    Vector3& w2 = constraintSolverData.angularVelocities[mIndexBody2];

    // Get the inverse mass and inverse inertia tensors of the bodies
    const decimal inverseMassBody1 = mBody1->mMassInverse;
    const decimal inverseMassBody2 = mBody2->mMassInverse;

    // Compute the impulse P=J^T * lambda for the 2 rotation constraints
    Vector3 rotationImpulse = -mB2CrossA1 * mImpulseRotation.x - mC2CrossA1 * mImpulseRotation.y;

    // Compute the impulse P=J^T * lambda for the lower and upper limits constraints
    const Vector3 limitsImpulse = (mImpulseUpperLimit - mImpulseLowerLimit) * mA1;

    // Compute the impulse P=J^T * lambda for the motor constraint
    const Vector3 motorImpulse = -mImpulseMotor * mA1;

    // Compute the impulse P=J^T * lambda for the 3 translation constraints of body 1
    Vector3 linearImpulseBody1 = -mImpulseTranslation;
    Vector3 angularImpulseBody1 = mImpulseTranslation.cross(mR1World);

    // Compute the impulse P=J^T * lambda for the 2 rotation constraints of body 1
    angularImpulseBody1 += rotationImpulse;

    // Compute the impulse P=J^T * lambda for the lower and upper limits constraints of body 1
    angularImpulseBody1 += limitsImpulse;

    // Compute the impulse P=J^T * lambda for the motor constraint of body 1
    angularImpulseBody1 += motorImpulse;

    // Apply the impulse to the body 1
    v1 += inverseMassBody1 * linearImpulseBody1;
    w1 += mI1 * angularImpulseBody1;

    // Compute the impulse P=J^T * lambda for the 3 translation constraints of body 2
    Vector3 angularImpulseBody2 = -mImpulseTranslation.cross(mR2World);

    // Compute the impulse P=J^T * lambda for the 2 rotation constraints of body 2
    angularImpulseBody2 += -rotationImpulse;

    // Compute the impulse P=J^T * lambda for the lower and upper limits constraints of body 2
    angularImpulseBody2 += -limitsImpulse;

    // Compute the impulse P=J^T * lambda for the motor constraint of body 2
    angularImpulseBody2 += -motorImpulse;

    // Apply the impulse to the body 2
    v2 += inverseMassBody2 * mImpulseTranslation;
    w2 += mI2 * angularImpulseBody2;
}

// Solve the velocity constraint
void HingeJoint::solveVelocityConstraint(const ConstraintSolverData& constraintSolverData) {

    // Get the velocities
    Vector3& v1 = constraintSolverData.linearVelocities[mIndexBody1];
    Vector3& v2 = constraintSolverData.linearVelocities[mIndexBody2];
    Vector3& w1 = constraintSolverData.angularVelocities[mIndexBody1];
    Vector3& w2 = constraintSolverData.angularVelocities[mIndexBody2];

    // Get the inverse mass and inverse inertia tensors of the bodies
    decimal inverseMassBody1 = mBody1->mMassInverse;
    decimal inverseMassBody2 = mBody2->mMassInverse;

    // --------------- Translation Constraints --------------- //

    // Compute J*v
    const Vector3 JvTranslation = v2 + w2.cross(mR2World) - v1 - w1.cross(mR1World);

    // Compute the Lagrange multiplier lambda
    const Vector3 deltaLambdaTranslation = mInverseMassMatrixTranslation *
                                          (-JvTranslation - mBTranslation);
    mImpulseTranslation += deltaLambdaTranslation;

    // Compute the impulse P=J^T * lambda of body 1
    const Vector3 linearImpulseBody1 = -deltaLambdaTranslation;
    Vector3 angularImpulseBody1 = deltaLambdaTranslation.cross(mR1World);

    // Apply the impulse to the body 1
    v1 += inverseMassBody1 * linearImpulseBody1;
    w1 += mI1 * angularImpulseBody1;

    // Compute the impulse P=J^T * lambda of body 2
    Vector3 angularImpulseBody2 = -deltaLambdaTranslation.cross(mR2World);

    // Apply the impulse to the body 2
    v2 += inverseMassBody2 * deltaLambdaTranslation;
    w2 += mI2 * angularImpulseBody2;

    // --------------- Rotation Constraints --------------- //

    // Compute J*v for the 2 rotation constraints
    const Vector2 JvRotation(-mB2CrossA1.dot(w1) + mB2CrossA1.dot(w2),
                             -mC2CrossA1.dot(w1) + mC2CrossA1.dot(w2));

    // Compute the Lagrange multiplier lambda for the 2 rotation constraints
    Vector2 deltaLambdaRotation = mInverseMassMatrixRotation * (-JvRotation - mBRotation);
    mImpulseRotation += deltaLambdaRotation;

    // Compute the impulse P=J^T * lambda for the 2 rotation constraints of body 1
    angularImpulseBody1 = -mB2CrossA1 * deltaLambdaRotation.x -
                                        mC2CrossA1 * deltaLambdaRotation.y;

    // Apply the impulse to the body 1
    w1 += mI1 * angularImpulseBody1;

    // Compute the impulse P=J^T * lambda for the 2 rotation constraints of body 2
    angularImpulseBody2 = mB2CrossA1 * deltaLambdaRotation.x +
            mC2CrossA1 * deltaLambdaRotation.y;

    // Apply the impulse to the body 2
    w2 += mI2 * angularImpulseBody2;

    // --------------- Limits Constraints --------------- //

    if (mIsLimitEnabled) {

        // If the lower limit is violated
        if (mIsLowerLimitViolated) {

            // Compute J*v for the lower limit constraint
            const decimal JvLowerLimit = (w2 - w1).dot(mA1);

            // Compute the Lagrange multiplier lambda for the lower limit constraint
            decimal deltaLambdaLower = mInverseMassMatrixLimitMotor * (-JvLowerLimit -mBLowerLimit);
            decimal lambdaTemp = mImpulseLowerLimit;
            mImpulseLowerLimit = std::max(mImpulseLowerLimit + deltaLambdaLower, decimal(0.0));
            deltaLambdaLower = mImpulseLowerLimit - lambdaTemp;

            // Compute the impulse P=J^T * lambda for the lower limit constraint of body 1
            const Vector3 angularImpulseBody1 = -deltaLambdaLower * mA1;

            // Apply the impulse to the body 1
            w1 += mI1 * angularImpulseBody1;

            // Compute the impulse P=J^T * lambda for the lower limit constraint of body 2
            const Vector3 angularImpulseBody2 = deltaLambdaLower * mA1;

            // Apply the impulse to the body 2
            w2 += mI2 * angularImpulseBody2;
        }

        // If the upper limit is violated
        if (mIsUpperLimitViolated) {

            // Compute J*v for the upper limit constraint
            const decimal JvUpperLimit = -(w2 - w1).dot(mA1);

            // Compute the Lagrange multiplier lambda for the upper limit constraint
            decimal deltaLambdaUpper = mInverseMassMatrixLimitMotor * (-JvUpperLimit -mBUpperLimit);
            decimal lambdaTemp = mImpulseUpperLimit;
            mImpulseUpperLimit = std::max(mImpulseUpperLimit + deltaLambdaUpper, decimal(0.0));
            deltaLambdaUpper = mImpulseUpperLimit - lambdaTemp;

            // Compute the impulse P=J^T * lambda for the upper limit constraint of body 1
            const Vector3 angularImpulseBody1 = deltaLambdaUpper * mA1;

            // Apply the impulse to the body 1
            w1 += mI1 * angularImpulseBody1;

            // Compute the impulse P=J^T * lambda for the upper limit constraint of body 2
            const Vector3 angularImpulseBody2 = -deltaLambdaUpper * mA1;

            // Apply the impulse to the body 2
            w2 += mI2 * angularImpulseBody2;
        }
    }

    // --------------- Motor --------------- //

    // If the motor is enabled
    if (mIsMotorEnabled) {

        // Compute J*v for the motor
        const decimal JvMotor = mA1.dot(w1 - w2);

        // Compute the Lagrange multiplier lambda for the motor
        const decimal maxMotorImpulse = mMaxMotorTorque * constraintSolverData.timeStep;
        decimal deltaLambdaMotor = mInverseMassMatrixLimitMotor * (-JvMotor - mMotorSpeed);
        decimal lambdaTemp = mImpulseMotor;
        mImpulseMotor = clamp(mImpulseMotor + deltaLambdaMotor, -maxMotorImpulse, maxMotorImpulse);
        deltaLambdaMotor = mImpulseMotor - lambdaTemp;

        // Compute the impulse P=J^T * lambda for the motor of body 1
        const Vector3 angularImpulseBody1 = -deltaLambdaMotor * mA1;

        // Apply the impulse to the body 1
        w1 += mI1 * angularImpulseBody1;

        // Compute the impulse P=J^T * lambda for the motor of body 2
        const Vector3 angularImpulseBody2 = deltaLambdaMotor * mA1;

        // Apply the impulse to the body 2
        w2 += mI2 * angularImpulseBody2;
    }
}

// Solve the position constraint (for position error correction)
void HingeJoint::solvePositionConstraint(const ConstraintSolverData& constraintSolverData) {

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

    // Compute the current angle around the hinge axis
    decimal hingeAngle = computeCurrentHingeAngle(q1, q2);

    // Check if the limit constraints are violated or not
    decimal lowerLimitError = hingeAngle - mLowerLimit;
    decimal upperLimitError = mUpperLimit - hingeAngle;
    mIsLowerLimitViolated = lowerLimitError <= 0;
    mIsUpperLimitViolated = upperLimitError <= 0;

    // Compute vectors needed in the Jacobian
    mA1 = q1 * mHingeLocalAxisBody1;
    Vector3 a2 = q2 * mHingeLocalAxisBody2;
    mA1.normalize();
    a2.normalize();
    const Vector3 b2 = a2.getOneUnitOrthogonalVector();
    const Vector3 c2 = a2.cross(b2);
    mB2CrossA1 = b2.cross(mA1);
    mC2CrossA1 = c2.cross(mA1);

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

    // Compute the inverse mass matrix K=JM^-1J^t for the 2 rotation constraints (2x2 matrix)
    Vector3 I1B2CrossA1 = mI1 * mB2CrossA1;
    Vector3 I1C2CrossA1 = mI1 * mC2CrossA1;
    Vector3 I2B2CrossA1 = mI2 * mB2CrossA1;
    Vector3 I2C2CrossA1 = mI2 * mC2CrossA1;
    const decimal el11 = mB2CrossA1.dot(I1B2CrossA1) +
                         mB2CrossA1.dot(I2B2CrossA1);
    const decimal el12 = mB2CrossA1.dot(I1C2CrossA1) +
                         mB2CrossA1.dot(I2C2CrossA1);
    const decimal el21 = mC2CrossA1.dot(I1B2CrossA1) +
                         mC2CrossA1.dot(I2B2CrossA1);
    const decimal el22 = mC2CrossA1.dot(I1C2CrossA1) +
                         mC2CrossA1.dot(I2C2CrossA1);
    const Matrix2x2 matrixKRotation(el11, el12, el21, el22);
    mInverseMassMatrixRotation.setToZero();
    if (mBody1->getType() == BodyType::DYNAMIC || mBody2->getType() == BodyType::DYNAMIC) {
        mInverseMassMatrixRotation = matrixKRotation.getInverse();
    }

    // Compute the position error for the 3 rotation constraints
    const Vector2 errorRotation = Vector2(mA1.dot(b2), mA1.dot(c2));

    // Compute the Lagrange multiplier lambda for the 3 rotation constraints
    Vector2 lambdaRotation = mInverseMassMatrixRotation * (-errorRotation);

    // Compute the impulse P=J^T * lambda for the 3 rotation constraints of body 1
    angularImpulseBody1 = -mB2CrossA1 * lambdaRotation.x - mC2CrossA1 * lambdaRotation.y;

    // Compute the pseudo velocity of body 1
    w1 = mI1 * angularImpulseBody1;

    // Update the body position/orientation of body 1
    q1 += Quaternion(0, w1) * q1 * decimal(0.5);
    q1.normalize();

    // Compute the impulse of body 2
    angularImpulseBody2 = mB2CrossA1 * lambdaRotation.x + mC2CrossA1 * lambdaRotation.y;

    // Compute the pseudo velocity of body 2
    w2 = mI2 * angularImpulseBody2;

    // Update the body position/orientation of body 2
    q2 += Quaternion(0, w2) * q2 * decimal(0.5);
    q2.normalize();

    // --------------- Limits Constraints --------------- //

    if (mIsLimitEnabled) {

        if (mIsLowerLimitViolated || mIsUpperLimitViolated) {

            // Compute the inverse of the mass matrix K=JM^-1J^t for the limits (1x1 matrix)
            mInverseMassMatrixLimitMotor = mA1.dot(mI1 * mA1) + mA1.dot(mI2 * mA1);
            mInverseMassMatrixLimitMotor = (mInverseMassMatrixLimitMotor > 0.0) ?
                                      decimal(1.0) / mInverseMassMatrixLimitMotor : decimal(0.0);
        }

        // If the lower limit is violated
        if (mIsLowerLimitViolated) {

            // Compute the Lagrange multiplier lambda for the lower limit constraint
            decimal lambdaLowerLimit = mInverseMassMatrixLimitMotor * (-lowerLimitError );

            // Compute the impulse P=J^T * lambda of body 1
            const Vector3 angularImpulseBody1 = -lambdaLowerLimit * mA1;

            // Compute the pseudo velocity of body 1
            const Vector3 w1 = mI1 * angularImpulseBody1;

            // Update the body position/orientation of body 1
            q1 += Quaternion(0, w1) * q1 * decimal(0.5);
            q1.normalize();

            // Compute the impulse P=J^T * lambda of body 2
            const Vector3 angularImpulseBody2 = lambdaLowerLimit * mA1;

            // Compute the pseudo velocity of body 2
            const Vector3 w2 = mI2 * angularImpulseBody2;

            // Update the body position/orientation of body 2
            q2 += Quaternion(0, w2) * q2 * decimal(0.5);
            q2.normalize();
        }

        // If the upper limit is violated
        if (mIsUpperLimitViolated) {

            // Compute the Lagrange multiplier lambda for the upper limit constraint
            decimal lambdaUpperLimit = mInverseMassMatrixLimitMotor * (-upperLimitError);

            // Compute the impulse P=J^T * lambda of body 1
            const Vector3 angularImpulseBody1 = lambdaUpperLimit * mA1;

            // Compute the pseudo velocity of body 1
            const Vector3 w1 = mI1 * angularImpulseBody1;

            // Update the body position/orientation of body 1
            q1 += Quaternion(0, w1) * q1 * decimal(0.5);
            q1.normalize();

            // Compute the impulse P=J^T * lambda of body 2
            const Vector3 angularImpulseBody2 = -lambdaUpperLimit * mA1;

            // Compute the pseudo velocity of body 2
            const Vector3 w2 = mI2 * angularImpulseBody2;

            // Update the body position/orientation of body 2
            q2 += Quaternion(0, w2) * q2 * decimal(0.5);
            q2.normalize();
        }
    }
}


// Enable/Disable the limits of the joint
/**
 * @param isLimitEnabled True if you want to enable the limits of the joint and
 *                       false otherwise
 */
void HingeJoint::enableLimit(bool isLimitEnabled) {

    if (isLimitEnabled != mIsLimitEnabled) {

        mIsLimitEnabled = isLimitEnabled;

        // Reset the limits
        resetLimits();
    }
}

// Enable/Disable the motor of the joint
/**
 * @param isMotorEnabled True if you want to enable the motor of the joint and
 *                       false otherwise
 */
void HingeJoint::enableMotor(bool isMotorEnabled) {

    mIsMotorEnabled = isMotorEnabled;
    mImpulseMotor = 0.0;

    // Wake up the two bodies of the joint
    mBody1->setIsSleeping(false);
    mBody2->setIsSleeping(false);
}

// Set the minimum angle limit
/**
 * @param lowerLimit The minimum limit angle of the joint (in radian)
 */
void HingeJoint::setMinAngleLimit(decimal lowerLimit) {

    assert(mLowerLimit <= 0 && mLowerLimit >= -2.0 * PI);

    if (lowerLimit != mLowerLimit) {

        mLowerLimit = lowerLimit;

        // Reset the limits
        resetLimits();
    }
}

// Set the maximum angle limit
/**
 * @param upperLimit The maximum limit angle of the joint (in radian)
 */
void HingeJoint::setMaxAngleLimit(decimal upperLimit) {

    assert(upperLimit >= 0 && upperLimit <= 2.0 * PI);

    if (upperLimit != mUpperLimit) {

        mUpperLimit = upperLimit;

        // Reset the limits
        resetLimits();
    }
}

// Reset the limits
void HingeJoint::resetLimits() {

    // Reset the accumulated impulses for the limits
    mImpulseLowerLimit = 0.0;
    mImpulseUpperLimit = 0.0;

    // Wake up the two bodies of the joint
    mBody1->setIsSleeping(false);
    mBody2->setIsSleeping(false);
}

// Set the motor speed
void HingeJoint::setMotorSpeed(decimal motorSpeed) {

    if (motorSpeed != mMotorSpeed) {

        mMotorSpeed = motorSpeed;

        // Wake up the two bodies of the joint
        mBody1->setIsSleeping(false);
        mBody2->setIsSleeping(false);
    }
}

// Set the maximum motor torque
/**
 * @param maxMotorTorque The maximum torque (in Newtons) of the joint motor
 */
void HingeJoint::setMaxMotorTorque(decimal maxMotorTorque) {

    if (maxMotorTorque != mMaxMotorTorque) {

        assert(mMaxMotorTorque >= 0.0);
        mMaxMotorTorque = maxMotorTorque;

        // Wake up the two bodies of the joint
        mBody1->setIsSleeping(false);
        mBody2->setIsSleeping(false);
    }
}

// Given an angle in radian, this method returns the corresponding angle in the range [-pi; pi]
decimal HingeJoint::computeNormalizedAngle(decimal angle) const {

    // Convert it into the range [-2*pi; 2*pi]
    angle = fmod(angle, PI_TIMES_2);

    // Convert it into the range [-pi; pi]
    if (angle < -PI) {
        return angle + PI_TIMES_2;
    }
    else if (angle > PI) {
        return angle - PI_TIMES_2;
    }
    else {
        return angle;
    }
}

// Given an "inputAngle" in the range [-pi, pi], this method returns an
// angle (modulo 2*pi) in the range [-2*pi; 2*pi] that is closest to one of the
// two angle limits in arguments.
decimal HingeJoint::computeCorrespondingAngleNearLimits(decimal inputAngle, decimal lowerLimitAngle,
                                                        decimal upperLimitAngle) const {
    if (upperLimitAngle <= lowerLimitAngle) {
        return inputAngle;
    }
    else if (inputAngle > upperLimitAngle) {
        decimal diffToUpperLimit = fabs(computeNormalizedAngle(inputAngle - upperLimitAngle));
        decimal diffToLowerLimit = fabs(computeNormalizedAngle(inputAngle - lowerLimitAngle));
        return (diffToUpperLimit > diffToLowerLimit) ? (inputAngle - PI_TIMES_2) : inputAngle;
    }
    else if (inputAngle < lowerLimitAngle) {
        decimal diffToUpperLimit = fabs(computeNormalizedAngle(upperLimitAngle - inputAngle));
        decimal diffToLowerLimit = fabs(computeNormalizedAngle(lowerLimitAngle - inputAngle));
        return (diffToUpperLimit > diffToLowerLimit) ? inputAngle : (inputAngle + PI_TIMES_2);
    }
    else {
        return inputAngle;
    }
}

// Compute the current angle around the hinge axis
decimal HingeJoint::computeCurrentHingeAngle(const Quaternion& orientationBody1,
                                             const Quaternion& orientationBody2) {

    decimal hingeAngle;

    // Compute the current orientation difference between the two bodies
    Quaternion currentOrientationDiff = orientationBody2 * orientationBody1.getInverse();
    currentOrientationDiff.normalize();

    // Compute the relative rotation considering the initial orientation difference
    Quaternion relativeRotation = currentOrientationDiff * mInitOrientationDifferenceInv;
    relativeRotation.normalize();

    // A quaternion q = [cos(theta/2); sin(theta/2) * rotAxis] where rotAxis is a unit
    // length vector. We can extract cos(theta/2) with q.w and we can extract |sin(theta/2)| with :
    // |sin(theta/2)| = q.getVectorV().length() since rotAxis is unit length. Note that any
    // rotation can be represented by a quaternion q and -q. Therefore, if the relative rotation
    // axis is not pointing in the same direction as the hinge axis, we use the rotation -q which
    // has the same |sin(theta/2)| value but the value cos(theta/2) is sign inverted. Some details
    // about this trick is explained in the source code of OpenTissue (http://www.opentissue.org).
    decimal cosHalfAngle = relativeRotation.w;
    decimal sinHalfAngleAbs = relativeRotation.getVectorV().length();

    // Compute the dot product of the relative rotation axis and the hinge axis
    decimal dotProduct = relativeRotation.getVectorV().dot(mA1);

    // If the relative rotation axis and the hinge axis are pointing the same direction
    if (dotProduct >= decimal(0.0)) {
        hingeAngle = decimal(2.0) * std::atan2(sinHalfAngleAbs, cosHalfAngle);
    }
    else {
        hingeAngle = decimal(2.0) * std::atan2(sinHalfAngleAbs, -cosHalfAngle);
    }

    // Convert the angle from range [-2*pi; 2*pi] into the range [-pi; pi]
    hingeAngle = computeNormalizedAngle(hingeAngle);

    // Compute and return the corresponding angle near one the two limits
    return computeCorrespondingAngleNearLimits(hingeAngle, mLowerLimit, mUpperLimit);
}

