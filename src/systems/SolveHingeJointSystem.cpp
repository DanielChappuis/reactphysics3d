/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2022 Daniel Chappuis                                       *
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
#include <reactphysics3d/systems/SolveHingeJointSystem.h>
#include <reactphysics3d/engine/PhysicsWorld.h>
#include <reactphysics3d/body/RigidBody.h>

using namespace reactphysics3d;

// Static variables definition
const decimal SolveHingeJointSystem::BETA = decimal(0.2);

// Constructor
SolveHingeJointSystem::SolveHingeJointSystem(PhysicsWorld& world, RigidBodyComponents& rigidBodyComponents,
                                                             TransformComponents& transformComponents,
                                                             JointComponents& jointComponents,
                                                             HingeJointComponents& hingeJointComponents)
              :mWorld(world), mRigidBodyComponents(rigidBodyComponents), mTransformComponents(transformComponents),
               mJointComponents(jointComponents), mHingeJointComponents(hingeJointComponents),
               mTimeStep(0), mIsWarmStartingActive(true) {

}

// Initialize before solving the constraint
void SolveHingeJointSystem::initBeforeSolve() {

    const decimal biasFactor = (BETA / mTimeStep);

    // For each joint
    const uint32 nbJoints = mHingeJointComponents.getNbEnabledComponents();
    for (uint32 i=0; i < nbJoints; i++) {

        const Entity jointEntity = mHingeJointComponents.mJointEntities[i];
        const uint32 jointIndex = mJointComponents.getEntityIndex(jointEntity);

        // Get the bodies entities
        const Entity body1Entity = mJointComponents.mBody1Entities[jointIndex];
        const Entity body2Entity = mJointComponents.mBody2Entities[jointIndex];

        const uint32 componentIndexBody1 = mRigidBodyComponents.getEntityIndex(body1Entity);
        const uint32 componentIndexBody2 = mRigidBodyComponents.getEntityIndex(body2Entity);

        assert(!mRigidBodyComponents.getIsEntityDisabled(body1Entity));
        assert(!mRigidBodyComponents.getIsEntityDisabled(body2Entity));

        // Get the inertia tensor of bodies
        mHingeJointComponents.mI1[i] = mRigidBodyComponents.mInverseInertiaTensorsWorld[componentIndexBody1];
        mHingeJointComponents.mI2[i] = mRigidBodyComponents.mInverseInertiaTensorsWorld[componentIndexBody2];

        const Quaternion& orientationBody1 = mTransformComponents.getTransform(body1Entity).getOrientation();
        const Quaternion& orientationBody2 = mTransformComponents.getTransform(body2Entity).getOrientation();

        // Compute the vector from body center to the anchor point in world-space
        mHingeJointComponents.mR1World[i] = orientationBody1 * (mHingeJointComponents.mLocalAnchorPointBody1[i] - mRigidBodyComponents.mCentersOfMassLocal[componentIndexBody1]);
        mHingeJointComponents.mR2World[i] = orientationBody2 * (mHingeJointComponents.mLocalAnchorPointBody2[i] - mRigidBodyComponents.mCentersOfMassLocal[componentIndexBody2]);

        // Compute vectors needed in the Jacobian
        Vector3& a1 = mHingeJointComponents.mA1[i];
        a1 = orientationBody1 * mHingeJointComponents.mHingeLocalAxisBody1[i];
        Vector3 a2 = orientationBody2 * mHingeJointComponents.mHingeLocalAxisBody2[i];

        a1.normalize();
        a2.normalize();
        const Vector3 b2 = a2.getOneUnitOrthogonalVector();
        const Vector3 c2 = a2.cross(b2);
        mHingeJointComponents.mB2CrossA1[i] = b2.cross(a1);
        mHingeJointComponents.mC2CrossA1[i] = c2.cross(a1);

        // Compute the bias "b" of the rotation constraints
        mHingeJointComponents.mBiasRotation[i].setToZero();
        if (mJointComponents.mPositionCorrectionTechniques[jointIndex] == JointsPositionCorrectionTechnique::BAUMGARTE_JOINTS) {
            mHingeJointComponents.mBiasRotation[i] = biasFactor * Vector2(a1.dot(b2), a1.dot(c2));
        }

        // Compute the corresponding skew-symmetric matrices
        Matrix3x3 skewSymmetricMatrixU1= Matrix3x3::computeSkewSymmetricMatrixForCrossProduct(mHingeJointComponents.mR1World[i]);
        Matrix3x3 skewSymmetricMatrixU2= Matrix3x3::computeSkewSymmetricMatrixForCrossProduct(mHingeJointComponents.mR2World[i]);

        // Compute the inverse mass matrix K=JM^-1J^t for the 3 translation constraints (3x3 matrix)
        decimal body1MassInverse = mRigidBodyComponents.mInverseMasses[componentIndexBody1];
        decimal body2MassInverse = mRigidBodyComponents.mInverseMasses[componentIndexBody2];
        decimal inverseMassBodies = body1MassInverse + body2MassInverse;
        Matrix3x3 massMatrix = Matrix3x3(inverseMassBodies, 0, 0,
                                        0, inverseMassBodies, 0,
                                        0, 0, inverseMassBodies) +
                               skewSymmetricMatrixU1 * mHingeJointComponents.mI1[i] * skewSymmetricMatrixU1.getTranspose() +
                               skewSymmetricMatrixU2 * mHingeJointComponents.mI2[i] * skewSymmetricMatrixU2.getTranspose();
        Matrix3x3& inverseMassMatrixTranslation = mHingeJointComponents.mInverseMassMatrixTranslation[i];
        inverseMassMatrixTranslation.setToZero();
        decimal massMatrixDeterminant = massMatrix.getDeterminant();
        if (std::abs(massMatrixDeterminant) > MACHINE_EPSILON) {
            if (mRigidBodyComponents.mBodyTypes[componentIndexBody1] == BodyType::DYNAMIC ||
                mRigidBodyComponents.mBodyTypes[componentIndexBody2] == BodyType::DYNAMIC) {
                mHingeJointComponents.mInverseMassMatrixTranslation[i] = massMatrix.getInverse(massMatrixDeterminant);
            }
        }

        // Get the bodies positions and orientations
        const Vector3& x1 = mRigidBodyComponents.mCentersOfMassWorld[componentIndexBody1];
        const Vector3& x2 = mRigidBodyComponents.mCentersOfMassWorld[componentIndexBody2];

        // Compute the bias "b" of the translation constraints
        mHingeJointComponents.mBiasTranslation[i].setToZero();
        if (mJointComponents.mPositionCorrectionTechniques[jointIndex] == JointsPositionCorrectionTechnique::BAUMGARTE_JOINTS) {
            mHingeJointComponents.mBiasTranslation[i] = biasFactor * (x2 + mHingeJointComponents.mR2World[i] - x1 - mHingeJointComponents.mR1World[i]);
        }

        const Matrix3x3& i1 = mHingeJointComponents.mI1[i];
        const Matrix3x3& i2 = mHingeJointComponents.mI2[i];
        const Vector3& b2CrossA1 = mHingeJointComponents.mB2CrossA1[i];
        const Vector3& c2CrossA1 = mHingeJointComponents.mC2CrossA1[i];

        // Compute the inverse mass matrix K=JM^-1J^t for the 2 rotation constraints (2x2 matrix)
        Vector3 i1B2CrossA1 = i1 * b2CrossA1;
        Vector3 i1C2CrossA1 = i1 * c2CrossA1;
        Vector3 i2B2CrossA1 = i2 * b2CrossA1;
        Vector3 i2C2CrossA1 = i2 * c2CrossA1;
        const decimal el11 = b2CrossA1.dot(i1B2CrossA1) + b2CrossA1.dot(i2B2CrossA1);
        const decimal el12 = b2CrossA1.dot(i1C2CrossA1) + b2CrossA1.dot(i2C2CrossA1);
        const decimal el21 = c2CrossA1.dot(i1B2CrossA1) + c2CrossA1.dot(i2B2CrossA1);
        const decimal el22 = c2CrossA1.dot(i1C2CrossA1) + c2CrossA1.dot(i2C2CrossA1);
        const Matrix2x2 matrixKRotation(el11, el12, el21, el22);
        mHingeJointComponents.mInverseMassMatrixRotation[i].setToZero();
        decimal matrixKRotationDeterminant = matrixKRotation.getDeterminant();
        if (std::abs(matrixKRotationDeterminant) > MACHINE_EPSILON) {
            if (mRigidBodyComponents.mBodyTypes[componentIndexBody1] == BodyType::DYNAMIC ||
                mRigidBodyComponents.mBodyTypes[componentIndexBody2] == BodyType::DYNAMIC) {
                mHingeJointComponents.mInverseMassMatrixRotation[i] = matrixKRotation.getInverse(matrixKRotationDeterminant);
            }
        }

        // If warm-starting is not enabled
        if (!mIsWarmStartingActive) {

            // Reset all the accumulated impulses
            mHingeJointComponents.mImpulseTranslation[i].setToZero();
            mHingeJointComponents.mImpulseRotation[i].setToZero();
            mHingeJointComponents.mImpulseLowerLimit[i] = decimal(0.0);
            mHingeJointComponents.mImpulseUpperLimit[i] = decimal(0.0);
            mHingeJointComponents.mImpulseMotor[i] = decimal(0.0);
        }

        // Compute the current angle around the hinge axis
        decimal hingeAngle = computeCurrentHingeAngle(jointEntity, orientationBody1, orientationBody2);

        // Check if the limit constraints are violated or not
        decimal lowerLimitError = hingeAngle - mHingeJointComponents.mLowerLimit[i];
        decimal upperLimitError = mHingeJointComponents.mUpperLimit[i] - hingeAngle;
        bool oldIsLowerLimitViolated = mHingeJointComponents.mIsLowerLimitViolated[i];
        bool isLowerLimitViolated = lowerLimitError <= 0;
        mHingeJointComponents.mIsLowerLimitViolated[i] = isLowerLimitViolated;
        if (!isLowerLimitViolated || isLowerLimitViolated != oldIsLowerLimitViolated) {
            mHingeJointComponents.mImpulseLowerLimit[i] = decimal(0.0);
        }
        bool oldIsUpperLimitViolated = mHingeJointComponents.mIsUpperLimitViolated[i];
        bool isUpperLimitViolated = upperLimitError <= 0;
        mHingeJointComponents.mIsUpperLimitViolated[i] = isUpperLimitViolated;
        if (!isUpperLimitViolated || isUpperLimitViolated != oldIsUpperLimitViolated) {
            mHingeJointComponents.mImpulseUpperLimit[i] = decimal(0.0);
        }

        // If the motor or limits are enabled
        if (mHingeJointComponents.mIsMotorEnabled[i] ||
            (mHingeJointComponents.mIsLimitEnabled[i] && (mHingeJointComponents.mIsLowerLimitViolated[i] ||
                                                          mHingeJointComponents.mIsUpperLimitViolated[i]))) {

            Vector3& a1 = mHingeJointComponents.mA1[i];

            // Compute the inverse of the mass matrix K=JM^-1J^t for the limits and motor (1x1 matrix)
            decimal inverseMassMatrixLimitMotor = a1.dot(mHingeJointComponents.mI1[i] * a1) + a1.dot(mHingeJointComponents.mI2[i] * a1);
            inverseMassMatrixLimitMotor = (inverseMassMatrixLimitMotor > decimal(0.0)) ?
                                      decimal(1.0) / inverseMassMatrixLimitMotor : decimal(0.0);
            mHingeJointComponents.mInverseMassMatrixLimitMotor[i] = inverseMassMatrixLimitMotor;

            if (mHingeJointComponents.mIsLimitEnabled[i]) {

                // Compute the bias "b" of the lower limit constraint
                mHingeJointComponents.mBLowerLimit[i] = decimal(0.0);
                if (mJointComponents.mPositionCorrectionTechniques[jointIndex] == JointsPositionCorrectionTechnique::BAUMGARTE_JOINTS) {
                    mHingeJointComponents.mBLowerLimit[i] = biasFactor * lowerLimitError;
                }

                // Compute the bias "b" of the upper limit constraint
                mHingeJointComponents.mBUpperLimit[i] = decimal(0.0);
                if (mJointComponents.mPositionCorrectionTechniques[jointIndex] == JointsPositionCorrectionTechnique::BAUMGARTE_JOINTS) {
                    mHingeJointComponents.mBUpperLimit[i] = biasFactor * upperLimitError;
                }
            }
        }
    }
}

// Warm start the constraint (apply the previous impulse at the beginning of the step)
void SolveHingeJointSystem::warmstart() {

    // For each joint component
    const uint32 nbJoints = mHingeJointComponents.getNbEnabledComponents();
    for (uint32 i=0; i < nbJoints; i++) {

        const Entity jointEntity = mHingeJointComponents.mJointEntities[i];
        const uint32 jointIndex = mJointComponents.getEntityIndex(jointEntity);

        // Get the bodies entities
        const Entity body1Entity = mJointComponents.mBody1Entities[jointIndex];
        const Entity body2Entity = mJointComponents.mBody2Entities[jointIndex];

        const uint32 componentIndexBody1 = mRigidBodyComponents.getEntityIndex(body1Entity);
        const uint32 componentIndexBody2 = mRigidBodyComponents.getEntityIndex(body2Entity);

        // Get the velocities
        Vector3& v1 = mRigidBodyComponents.mConstrainedLinearVelocities[componentIndexBody1];
        Vector3& v2 = mRigidBodyComponents.mConstrainedLinearVelocities[componentIndexBody2];
        Vector3& w1 = mRigidBodyComponents.mConstrainedAngularVelocities[componentIndexBody1];
        Vector3& w2 = mRigidBodyComponents.mConstrainedAngularVelocities[componentIndexBody2];

        // Get the inverse mass and inverse inertia tensors of the bodies
        const decimal inverseMassBody1 = mRigidBodyComponents.mInverseMasses[componentIndexBody1];
        const decimal inverseMassBody2 = mRigidBodyComponents.mInverseMasses[componentIndexBody2];

        const Vector3& impulseTranslation = mHingeJointComponents.mImpulseTranslation[i];
        const Vector2& impulseRotation = mHingeJointComponents.mImpulseRotation[i];

        const decimal impulseLowerLimit = mHingeJointComponents.mImpulseLowerLimit[i];
        const decimal impulseUpperLimit = mHingeJointComponents.mImpulseUpperLimit[i];

        const Vector3& b2CrossA1 = mHingeJointComponents.mB2CrossA1[i];
        const Vector3& a1 = mHingeJointComponents.mA1[i];

        // Compute the impulse P=J^T * lambda for the 2 rotation constraints
        Vector3 rotationImpulse = -b2CrossA1 * impulseRotation.x - mHingeJointComponents.mC2CrossA1[i] * impulseRotation.y;

        // Compute the impulse P=J^T * lambda for the lower and upper limits constraints
        const Vector3 limitsImpulse = (impulseUpperLimit - impulseLowerLimit) * a1;

        // Compute the impulse P=J^T * lambda for the motor constraint
        const Vector3 motorImpulse = -mHingeJointComponents.mImpulseMotor[i] * a1;

        // Compute the impulse P=J^T * lambda for the 3 translation constraints of body 1
        Vector3 linearImpulseBody1 = -impulseTranslation;
        Vector3 angularImpulseBody1 = impulseTranslation.cross(mHingeJointComponents.mR1World[i]);

        // Compute the impulse P=J^T * lambda for the 2 rotation constraints of body 1
        angularImpulseBody1 += rotationImpulse;

        // Compute the impulse P=J^T * lambda for the lower and upper limits constraints of body 1
        angularImpulseBody1 += limitsImpulse;

        // Compute the impulse P=J^T * lambda for the motor constraint of body 1
        angularImpulseBody1 += motorImpulse;

        // Apply the impulse to the body 1
        v1 += inverseMassBody1 * mRigidBodyComponents.mLinearLockAxisFactors[componentIndexBody1] * linearImpulseBody1;
        w1 += mRigidBodyComponents.mAngularLockAxisFactors[componentIndexBody1] * (mHingeJointComponents.mI1[i] * angularImpulseBody1);

        // Compute the impulse P=J^T * lambda for the 3 translation constraints of body 2
        Vector3 angularImpulseBody2 = -impulseTranslation.cross(mHingeJointComponents.mR2World[i]);

        // Compute the impulse P=J^T * lambda for the 2 rotation constraints of body 2
        angularImpulseBody2 += -rotationImpulse;

        // Compute the impulse P=J^T * lambda for the lower and upper limits constraints of body 2
        angularImpulseBody2 += -limitsImpulse;

        // Compute the impulse P=J^T * lambda for the motor constraint of body 2
        angularImpulseBody2 += -motorImpulse;

        // Apply the impulse to the body 2
        v2 += inverseMassBody2 * mRigidBodyComponents.mLinearLockAxisFactors[componentIndexBody2] * impulseTranslation;
        w2 += mRigidBodyComponents.mAngularLockAxisFactors[componentIndexBody2] * (mHingeJointComponents.mI2[i] * angularImpulseBody2);
    }
}

// Solve the velocity constraint
void SolveHingeJointSystem::solveVelocityConstraint() {

    // For each joint component
    const uint32 nbJoints = mHingeJointComponents.getNbEnabledComponents();
    for (uint32 i=0; i < nbJoints; i++) {

        const Entity jointEntity = mHingeJointComponents.mJointEntities[i];
        const uint32 jointIndex = mJointComponents.getEntityIndex(jointEntity);

        // Get the bodies entities
        const Entity body1Entity = mJointComponents.mBody1Entities[jointIndex];
        const Entity body2Entity = mJointComponents.mBody2Entities[jointIndex];

        const uint32 componentIndexBody1 = mRigidBodyComponents.getEntityIndex(body1Entity);
        const uint32 componentIndexBody2 = mRigidBodyComponents.getEntityIndex(body2Entity);

        // Get the velocities
        Vector3& v1 = mRigidBodyComponents.mConstrainedLinearVelocities[componentIndexBody1];
        Vector3& v2 = mRigidBodyComponents.mConstrainedLinearVelocities[componentIndexBody2];
        Vector3& w1 = mRigidBodyComponents.mConstrainedAngularVelocities[componentIndexBody1];
        Vector3& w2 = mRigidBodyComponents.mConstrainedAngularVelocities[componentIndexBody2];

        // Get the inverse mass and inverse inertia tensors of the bodies
        decimal inverseMassBody1 = mRigidBodyComponents.mInverseMasses[componentIndexBody1];
        decimal inverseMassBody2 = mRigidBodyComponents.mInverseMasses[componentIndexBody2];

        const Matrix3x3& i1 = mHingeJointComponents.mI1[i];
        const Matrix3x3& i2 = mHingeJointComponents.mI2[i];

        const Vector3& r1World = mHingeJointComponents.mR1World[i];
        const Vector3& r2World = mHingeJointComponents.mR2World[i];

        const Vector3& a1 = mHingeJointComponents.mA1[i];

        const decimal inverseMassMatrixLimitMotor = mHingeJointComponents.mInverseMassMatrixLimitMotor[i];

        // --------------- Limits Constraints --------------- //

        if (mHingeJointComponents.mIsLimitEnabled[i]) {

            // If the lower limit is violated
            if (mHingeJointComponents.mIsLowerLimitViolated[i]) {

                // Compute J*v for the lower limit constraint
                const decimal JvLowerLimit = (w2 - w1).dot(a1);

                // Compute the Lagrange multiplier lambda for the lower limit constraint
                decimal deltaLambdaLower = inverseMassMatrixLimitMotor * (-JvLowerLimit -mHingeJointComponents.mBLowerLimit[i]);
                decimal lambdaTemp = mHingeJointComponents.mImpulseLowerLimit[i];
                mHingeJointComponents.mImpulseLowerLimit[i] = std::max(mHingeJointComponents.mImpulseLowerLimit[i] + deltaLambdaLower, decimal(0.0));
                deltaLambdaLower = mHingeJointComponents.mImpulseLowerLimit[i] - lambdaTemp;

                // Compute the impulse P=J^T * lambda for the lower limit constraint of body 1
                const Vector3 angularImpulseBody1 = -deltaLambdaLower * a1;

                // Apply the impulse to the body 1
                w1 += mRigidBodyComponents.mAngularLockAxisFactors[componentIndexBody1] * (i1 * angularImpulseBody1);

                // Compute the impulse P=J^T * lambda for the lower limit constraint of body 2
                const Vector3 angularImpulseBody2 = deltaLambdaLower * a1;

                // Apply the impulse to the body 2
                w2 += mRigidBodyComponents.mAngularLockAxisFactors[componentIndexBody2] * (i2 * angularImpulseBody2);
            }

            // If the upper limit is violated
            if (mHingeJointComponents.mIsUpperLimitViolated[i]) {

                // Compute J*v for the upper limit constraint
                const decimal JvUpperLimit = -(w2 - w1).dot(a1);

                // Compute the Lagrange multiplier lambda for the upper limit constraint
                decimal deltaLambdaUpper = inverseMassMatrixLimitMotor * (-JvUpperLimit -mHingeJointComponents.mBUpperLimit[i]);
                decimal lambdaTemp = mHingeJointComponents.mImpulseUpperLimit[i];
                mHingeJointComponents.mImpulseUpperLimit[i] = std::max(mHingeJointComponents.mImpulseUpperLimit[i] + deltaLambdaUpper, decimal(0.0));
                deltaLambdaUpper = mHingeJointComponents.mImpulseUpperLimit[i] - lambdaTemp;

                // Compute the impulse P=J^T * lambda for the upper limit constraint of body 1
                const Vector3 angularImpulseBody1 = deltaLambdaUpper * a1;

                // Apply the impulse to the body 1
                w1 += mRigidBodyComponents.mAngularLockAxisFactors[componentIndexBody1] * (i1 * angularImpulseBody1);

                // Compute the impulse P=J^T * lambda for the upper limit constraint of body 2
                const Vector3 angularImpulseBody2 = -deltaLambdaUpper * a1;

                // Apply the impulse to the body 2
                w2 += mRigidBodyComponents.mAngularLockAxisFactors[componentIndexBody2] * (i2 * angularImpulseBody2);
            }
        }

        // --------------- Motor --------------- //

        // If the motor is enabled
        if (mHingeJointComponents.mIsMotorEnabled[i]) {

            // Compute J*v for the motor
            const decimal JvMotor = a1.dot(w1 - w2);

            // Compute the Lagrange multiplier lambda for the motor
            const decimal maxMotorImpulse = mHingeJointComponents.mMaxMotorTorque[i] * mTimeStep;
            decimal deltaLambdaMotor = mHingeJointComponents.mInverseMassMatrixLimitMotor[i] * (-JvMotor - mHingeJointComponents.mMotorSpeed[i]);
            decimal lambdaTemp = mHingeJointComponents.mImpulseMotor[i];
            mHingeJointComponents.mImpulseMotor[i] = clamp(mHingeJointComponents.mImpulseMotor[i] + deltaLambdaMotor, -maxMotorImpulse, maxMotorImpulse);
            deltaLambdaMotor = mHingeJointComponents.mImpulseMotor[i] - lambdaTemp;

            // Compute the impulse P=J^T * lambda for the motor of body 1
            const Vector3 angularImpulseBody1 = -deltaLambdaMotor * a1;

            // Apply the impulse to the body 1
            w1 += mRigidBodyComponents.mAngularLockAxisFactors[componentIndexBody1] * (i1 * angularImpulseBody1);

            // Compute the impulse P=J^T * lambda for the motor of body 2
            const Vector3 angularImpulseBody2 = deltaLambdaMotor * a1;

            // Apply the impulse to the body 2
            w2 += mRigidBodyComponents.mAngularLockAxisFactors[componentIndexBody2] * (i2 * angularImpulseBody2);
        }

        // --------------- Joint Rotation Constraints --------------- //

        const Vector3& b2CrossA1 = mHingeJointComponents.mB2CrossA1[i];
        const Vector3& c2CrossA1 = mHingeJointComponents.mC2CrossA1[i];

        // Compute J*v for the 2 rotation constraints
        const Vector2 JvRotation(-b2CrossA1.dot(w1) + b2CrossA1.dot(w2),
                                 -c2CrossA1.dot(w1) + c2CrossA1.dot(w2));

        // Compute the Lagrange multiplier lambda for the 2 rotation constraints
        Vector2 deltaLambdaRotation = mHingeJointComponents.mInverseMassMatrixRotation[i] *
                                      (-JvRotation - mHingeJointComponents.mBiasRotation[i]);
        mHingeJointComponents.mImpulseRotation[i] += deltaLambdaRotation;

        // Compute the impulse P=J^T * lambda for the 2 rotation constraints of body 1
        Vector3 angularImpulseBody1 = -b2CrossA1 * deltaLambdaRotation.x - c2CrossA1 * deltaLambdaRotation.y;

        // Apply the impulse to the body 1
        w1 += mRigidBodyComponents.mAngularLockAxisFactors[componentIndexBody1] * (i1 * angularImpulseBody1);

        // Compute the impulse P=J^T * lambda for the 2 rotation constraints of body 2
        Vector3 angularImpulseBody2 = b2CrossA1 * deltaLambdaRotation.x + c2CrossA1 * deltaLambdaRotation.y;

        // Apply the impulse to the body 2
        w2 += mRigidBodyComponents.mAngularLockAxisFactors[componentIndexBody2] * (i2 * angularImpulseBody2);

        // --------------- Joint Translation Constraints --------------- //

        // Compute J*v
        const Vector3 JvTranslation = v2 + w2.cross(r2World) - v1 - w1.cross(r1World);

        // Compute the Lagrange multiplier lambda
        const Vector3 deltaLambdaTranslation = mHingeJointComponents.mInverseMassMatrixTranslation[i] *
                                               (-JvTranslation - mHingeJointComponents.mBiasTranslation[i]);
        mHingeJointComponents.mImpulseTranslation[i] += deltaLambdaTranslation;

        // Compute the impulse P=J^T * lambda of body 1
        const Vector3 linearImpulseBody1 = -deltaLambdaTranslation;
        angularImpulseBody1 = deltaLambdaTranslation.cross(r1World);

        // Apply the impulse to the body 1
        v1 += inverseMassBody1 * mRigidBodyComponents.mLinearLockAxisFactors[componentIndexBody1] * linearImpulseBody1;
        w1 += mRigidBodyComponents.mAngularLockAxisFactors[componentIndexBody1] * (i1 * angularImpulseBody1);

        // Compute the impulse P=J^T * lambda of body 2
        angularImpulseBody2 = -deltaLambdaTranslation.cross(r2World);

        // Apply the impulse to the body 2
        v2 += inverseMassBody2 * mRigidBodyComponents.mLinearLockAxisFactors[componentIndexBody2] * deltaLambdaTranslation;
        w2 += mRigidBodyComponents.mAngularLockAxisFactors[componentIndexBody2] * (i2 * angularImpulseBody2);
    }
}

// Solve the position constraint (for position error correction)
void SolveHingeJointSystem::solvePositionConstraint() {

    // For each joint component
    const uint32 nbEnabledJoints = mHingeJointComponents.getNbEnabledComponents();
    for (uint32 i=0; i < nbEnabledJoints; i++) {

        const Entity jointEntity = mHingeJointComponents.mJointEntities[i];
        const uint32 jointIndex = mJointComponents.getEntityIndex(jointEntity);

        // If the error position correction technique is not the non-linear-gauss-seidel, we do not execute this method
        if (mJointComponents.mPositionCorrectionTechniques[jointIndex] != JointsPositionCorrectionTechnique::NON_LINEAR_GAUSS_SEIDEL) continue;

        // Get the bodies entities
        Entity body1Entity = mJointComponents.mBody1Entities[jointIndex];
        Entity body2Entity = mJointComponents.mBody2Entities[jointIndex];

        const uint32 componentIndexBody1 = mRigidBodyComponents.getEntityIndex(body1Entity);
        const uint32 componentIndexBody2 = mRigidBodyComponents.getEntityIndex(body2Entity);

        Quaternion& q1 = mRigidBodyComponents.mConstrainedOrientations[componentIndexBody1];
        Quaternion& q2 = mRigidBodyComponents.mConstrainedOrientations[componentIndexBody2];

        // Recompute the world inverse inertia tensors
        RigidBody::computeWorldInertiaTensorInverse(q1.getMatrix(), mRigidBodyComponents.mInverseInertiaTensorsLocal[componentIndexBody1],
                                                    mHingeJointComponents.mI1[i]);

        RigidBody::computeWorldInertiaTensorInverse(q2.getMatrix(), mRigidBodyComponents.mInverseInertiaTensorsLocal[componentIndexBody2],
                                                    mHingeJointComponents.mI2[i]);

        // Compute the vector from body center to the anchor point in world-space
        mHingeJointComponents.mR1World[i] = q1 * (mHingeJointComponents.mLocalAnchorPointBody1[i] - mRigidBodyComponents.mCentersOfMassLocal[componentIndexBody1]);
        mHingeJointComponents.mR2World[i] = q2 * (mHingeJointComponents.mLocalAnchorPointBody2[i] - mRigidBodyComponents.mCentersOfMassLocal[componentIndexBody2]);

        // Compute the corresponding skew-symmetric matrices
        Matrix3x3 skewSymmetricMatrixU1 = Matrix3x3::computeSkewSymmetricMatrixForCrossProduct(mHingeJointComponents.mR1World[i]);
        Matrix3x3 skewSymmetricMatrixU2 = Matrix3x3::computeSkewSymmetricMatrixForCrossProduct(mHingeJointComponents.mR2World[i]);


        Vector3& b2CrossA1 = mHingeJointComponents.mB2CrossA1[i];
        Vector3& c2CrossA1 = mHingeJointComponents.mC2CrossA1[i];

        Vector3& a1 = mHingeJointComponents.mA1[i];

        // Compute vectors needed in the Jacobian
        a1 = q1 * mHingeJointComponents.mHingeLocalAxisBody1[i];
        Vector3 a2 = q2 * mHingeJointComponents.mHingeLocalAxisBody2[i];
        a1.normalize();
        mHingeJointComponents.mA1[i] = a1;
        a2.normalize();
        const Vector3 b2 = a2.getOneUnitOrthogonalVector();
        const Vector3 c2 = a2.cross(b2);
        b2CrossA1 = b2.cross(a1);
        mHingeJointComponents.mB2CrossA1[i] = b2CrossA1;
        c2CrossA1 = c2.cross(a1);
        mHingeJointComponents.mC2CrossA1[i] = c2CrossA1;

        // Compute the current angle around the hinge axis
        const decimal hingeAngle = computeCurrentHingeAngle(jointEntity, q1, q2);

        // Check if the limit constraints are violated or not
        decimal lowerLimitError = hingeAngle - mHingeJointComponents.mLowerLimit[i];
        decimal upperLimitError = mHingeJointComponents.mUpperLimit[i] - hingeAngle;
        mHingeJointComponents.mIsLowerLimitViolated[i] = lowerLimitError <= 0;
        mHingeJointComponents.mIsUpperLimitViolated[i] = upperLimitError <= 0;

        // --------------- Limits Constraints --------------- //

        if (mHingeJointComponents.mIsLimitEnabled[i]) {

            decimal inverseMassMatrixLimitMotor = mHingeJointComponents.mInverseMassMatrixLimitMotor[i];

            Vector3& a1 = mHingeJointComponents.mA1[i];

            if (mHingeJointComponents.mIsLowerLimitViolated[i] || mHingeJointComponents.mIsUpperLimitViolated[i]) {

                // Compute the inverse of the mass matrix K=JM^-1J^t for the limits (1x1 matrix)
                mHingeJointComponents.mInverseMassMatrixLimitMotor[i] = a1.dot(mHingeJointComponents.mI1[i] * a1) + a1.dot(mHingeJointComponents.mI2[i] * a1);
                mHingeJointComponents.mInverseMassMatrixLimitMotor[i] = (inverseMassMatrixLimitMotor > decimal(0.0)) ?
                                          decimal(1.0) / mHingeJointComponents.mInverseMassMatrixLimitMotor[i] : decimal(0.0);
            }

            // If the lower limit is violated
            if (mHingeJointComponents.mIsLowerLimitViolated[i]) {

                // Compute the Lagrange multiplier lambda for the lower limit constraint
                decimal lambdaLowerLimit = inverseMassMatrixLimitMotor * (-lowerLimitError );

                // Compute the impulse P=J^T * lambda of body 1
                const Vector3 angularImpulseBody1 = -lambdaLowerLimit * a1;

                // Compute the pseudo velocity of body 1
                const Vector3 w1 = mRigidBodyComponents.mAngularLockAxisFactors[componentIndexBody1] * (mHingeJointComponents.mI1[i] * angularImpulseBody1);

                // Update the body position/orientation of body 1
                q1 += Quaternion(0, w1) * q1 * decimal(0.5);
                q1.normalize();

                // Compute the impulse P=J^T * lambda of body 2
                const Vector3 angularImpulseBody2 = lambdaLowerLimit * a1;

                // Compute the pseudo velocity of body 2
                const Vector3 w2 = mRigidBodyComponents.mAngularLockAxisFactors[componentIndexBody2] * (mHingeJointComponents.mI2[i] * angularImpulseBody2);

                // Update the body position/orientation of body 2
                q2 += Quaternion(0, w2) * q2 * decimal(0.5);
                q2.normalize();
            }

            // If the upper limit is violated
            if (mHingeJointComponents.mIsUpperLimitViolated[i]) {

                // Compute the Lagrange multiplier lambda for the upper limit constraint
                decimal lambdaUpperLimit = inverseMassMatrixLimitMotor * (-upperLimitError);

                // Compute the impulse P=J^T * lambda of body 1
                const Vector3 angularImpulseBody1 = lambdaUpperLimit * a1;

                // Compute the pseudo velocity of body 1
                const Vector3 w1 = mRigidBodyComponents.mAngularLockAxisFactors[componentIndexBody1] * (mHingeJointComponents.mI1[i] * angularImpulseBody1);

                // Update the body position/orientation of body 1
                q1 += Quaternion(0, w1) * q1 * decimal(0.5);
                q1.normalize();

                // Compute the impulse P=J^T * lambda of body 2
                const Vector3 angularImpulseBody2 = -lambdaUpperLimit * a1;

                // Compute the pseudo velocity of body 2
                const Vector3 w2 = mRigidBodyComponents.mAngularLockAxisFactors[componentIndexBody2] * (mHingeJointComponents.mI2[i] * angularImpulseBody2);

                // Update the body position/orientation of body 2
                q2 += Quaternion(0, w2) * q2 * decimal(0.5);
                q2.normalize();
            }
        }

        // --------------- Rotation Constraints --------------- //

        // Compute the inverse mass matrix K=JM^-1J^t for the 2 rotation constraints (2x2 matrix)
        Vector3 I1B2CrossA1 = mHingeJointComponents.mI1[i] * b2CrossA1;
        Vector3 I1C2CrossA1 = mHingeJointComponents.mI1[i] * c2CrossA1;
        Vector3 I2B2CrossA1 = mHingeJointComponents.mI2[i] * b2CrossA1;
        Vector3 I2C2CrossA1 = mHingeJointComponents.mI2[i] * c2CrossA1;
        const decimal el11 = b2CrossA1.dot(I1B2CrossA1) +
                             b2CrossA1.dot(I2B2CrossA1);
        const decimal el12 = b2CrossA1.dot(I1C2CrossA1) +
                             b2CrossA1.dot(I2C2CrossA1);
        const decimal el21 = c2CrossA1.dot(I1B2CrossA1) +
                             c2CrossA1.dot(I2B2CrossA1);
        const decimal el22 = c2CrossA1.dot(I1C2CrossA1) +
                             c2CrossA1.dot(I2C2CrossA1);
        const Matrix2x2 matrixKRotation(el11, el12, el21, el22);
        mHingeJointComponents.mInverseMassMatrixRotation[i].setToZero();
        decimal matrixDeterminant = matrixKRotation.getDeterminant();
        if (std::abs(matrixDeterminant) > MACHINE_EPSILON) {
            if (mRigidBodyComponents.mBodyTypes[componentIndexBody1] == BodyType::DYNAMIC ||
                mRigidBodyComponents.mBodyTypes[componentIndexBody2] == BodyType::DYNAMIC) {
                mHingeJointComponents.mInverseMassMatrixRotation[i] = matrixKRotation.getInverse(matrixDeterminant);
            }

            // Compute the position error for the 3 rotation constraints
            const Vector2 errorRotation = Vector2(a1.dot(b2), a1.dot(c2));

            // Compute the Lagrange multiplier lambda for the 3 rotation constraints
            Vector2 lambdaRotation = mHingeJointComponents.mInverseMassMatrixRotation[i] * (-errorRotation);

            // Compute the impulse P=J^T * lambda for the 3 rotation constraints of body 1
            Vector3 angularImpulseBody1 = -b2CrossA1 * lambdaRotation.x - c2CrossA1 * lambdaRotation.y;

            // Compute the pseudo velocity of body 1
            Vector3 w1 = mRigidBodyComponents.mAngularLockAxisFactors[componentIndexBody1] * (mHingeJointComponents.mI1[i] * angularImpulseBody1);

            // Update the body position/orientation of body 1
            q1 += Quaternion(0, w1) * q1 * decimal(0.5);
            q1.normalize();

            // Compute the impulse of body 2
            Vector3 angularImpulseBody2 = b2CrossA1 * lambdaRotation.x + c2CrossA1 * lambdaRotation.y;

            // Compute the pseudo velocity of body 2
            Vector3 w2 = mRigidBodyComponents.mAngularLockAxisFactors[componentIndexBody2] * (mHingeJointComponents.mI2[i] * angularImpulseBody2);

            // Update the body position/orientation of body 2
            q2 += Quaternion(0, w2) * q2 * decimal(0.5);
            q2.normalize();
        }

        // --------------- Translation Constraints --------------- //

        // Compute the matrix K=JM^-1J^t (3x3 matrix) for the 3 translation constraints
        const decimal body1InverseMass = mRigidBodyComponents.mInverseMasses[componentIndexBody1];
        const decimal body2InverseMass = mRigidBodyComponents.mInverseMasses[componentIndexBody2];
        decimal inverseMassBodies = body1InverseMass + body2InverseMass;
        Matrix3x3 massMatrix = Matrix3x3(inverseMassBodies, 0, 0,
                                        0, inverseMassBodies, 0,
                                        0, 0, inverseMassBodies) +
                               skewSymmetricMatrixU1 * mHingeJointComponents.mI1[i] * skewSymmetricMatrixU1.getTranspose() +
                               skewSymmetricMatrixU2 * mHingeJointComponents.mI2[i] * skewSymmetricMatrixU2.getTranspose();
        mHingeJointComponents.mInverseMassMatrixTranslation[i].setToZero();
        matrixDeterminant = massMatrix.getDeterminant();
        if (std::abs(matrixDeterminant) > MACHINE_EPSILON) {

            if (mRigidBodyComponents.mBodyTypes[componentIndexBody1] == BodyType::DYNAMIC ||
                mRigidBodyComponents.mBodyTypes[componentIndexBody2] == BodyType::DYNAMIC) {
                mHingeJointComponents.mInverseMassMatrixTranslation[i] = massMatrix.getInverse(matrixDeterminant);
            }


            Vector3& x1 = mRigidBodyComponents.mConstrainedPositions[componentIndexBody1];
            Vector3& x2 = mRigidBodyComponents.mConstrainedPositions[componentIndexBody2];

            // Compute position error for the 3 translation constraints
            const Vector3 errorTranslation = x2 + mHingeJointComponents.mR2World[i] - x1 - mHingeJointComponents.mR1World[i];

            // Compute the Lagrange multiplier lambda
            const Vector3 lambdaTranslation = mHingeJointComponents.mInverseMassMatrixTranslation[i] * (-errorTranslation);

            // Compute the impulse of body 1
            Vector3 linearImpulseBody1 = -lambdaTranslation;
            Vector3 angularImpulseBody1 = lambdaTranslation.cross(mHingeJointComponents.mR1World[i]);

            // Get the inverse mass and inverse inertia tensors of the bodies
            decimal inverseMassBody1 = mRigidBodyComponents.mInverseMasses[componentIndexBody1];
            decimal inverseMassBody2 = mRigidBodyComponents.mInverseMasses[componentIndexBody2];

            // Compute the pseudo velocity of body 1
            const Vector3 v1 = inverseMassBody1 * mRigidBodyComponents.mLinearLockAxisFactors[componentIndexBody1] * linearImpulseBody1;
            Vector3 w1 = mRigidBodyComponents.mAngularLockAxisFactors[componentIndexBody1] * (mHingeJointComponents.mI1[i] * angularImpulseBody1);

            // Update the body position/orientation of body 1
            x1 += v1;
            q1 += Quaternion(0, w1) * q1 * decimal(0.5);
            q1.normalize();

            // Compute the impulse of body 2
            Vector3 angularImpulseBody2 = -lambdaTranslation.cross(mHingeJointComponents.mR2World[i]);

            // Compute the pseudo velocity of body 2
            const Vector3 v2 = inverseMassBody2 * mRigidBodyComponents.mLinearLockAxisFactors[componentIndexBody2] * lambdaTranslation;
            Vector3 w2 = mRigidBodyComponents.mAngularLockAxisFactors[componentIndexBody2] * (mHingeJointComponents.mI2[i] * angularImpulseBody2);

            // Update the body position/orientation of body 2
            x2 += v2;
            q2 += Quaternion(0, w2) * q2 * decimal(0.5);
            q2.normalize();
        }
    }
}

// Given an angle in radian, this method returns the corresponding angle in the range [-pi; pi]
decimal SolveHingeJointSystem::computeNormalizedAngle(decimal angle) const {

    // Convert it into the range [-2*pi; 2*pi]
    angle = std::fmod(angle, PI_TIMES_2);

    // Convert it into the range [-pi; pi]
    if (angle < -PI_RP3D) {
        return angle + PI_TIMES_2;
    }
    else if (angle > PI_RP3D) {
        return angle - PI_TIMES_2;
    }
    else {
        return angle;
    }
}

// Given an "inputAngle" in the range [-pi, pi], this method returns an
// angle (modulo 2*pi) in the range [-2*pi; 2*pi] that is closest to one of the
// two angle limits in arguments.
decimal SolveHingeJointSystem::computeCorrespondingAngleNearLimits(decimal inputAngle, decimal lowerLimitAngle, decimal upperLimitAngle) const {
    if (upperLimitAngle <= lowerLimitAngle) {
        return inputAngle;
    }
    else if (inputAngle > upperLimitAngle) {
        decimal diffToUpperLimit = std::abs(computeNormalizedAngle(inputAngle - upperLimitAngle));
        decimal diffToLowerLimit = std::abs(computeNormalizedAngle(inputAngle - lowerLimitAngle));
        return (diffToUpperLimit > diffToLowerLimit) ? (inputAngle - PI_TIMES_2) : inputAngle;
    }
    else if (inputAngle < lowerLimitAngle) {
        decimal diffToUpperLimit = std::abs(computeNormalizedAngle(upperLimitAngle - inputAngle));
        decimal diffToLowerLimit = std::abs(computeNormalizedAngle(lowerLimitAngle - inputAngle));
        return (diffToUpperLimit > diffToLowerLimit) ? inputAngle : (inputAngle + PI_TIMES_2);
    }
    else {
        return inputAngle;
    }
}

// Compute the current angle around the hinge axis
decimal SolveHingeJointSystem::computeCurrentHingeAngle(Entity jointEntity, const Quaternion& orientationBody1, const Quaternion& orientationBody2) {

    decimal hingeAngle;

    // Compute the current orientation difference between the two bodies
    Quaternion currentOrientationDiff = orientationBody2 * orientationBody1.getInverse();
    currentOrientationDiff.normalize();

    // Compute the relative rotation considering the initial orientation difference
    Quaternion relativeRotation = currentOrientationDiff * mHingeJointComponents.getInitOrientationDifferenceInv(jointEntity);
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
    decimal dotProduct = relativeRotation.getVectorV().dot(mHingeJointComponents.getA1(jointEntity));

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
    return computeCorrespondingAngleNearLimits(hingeAngle,
                                               mHingeJointComponents.getLowerLimit(jointEntity),
                                               mHingeJointComponents.getUpperLimit(jointEntity));
}
