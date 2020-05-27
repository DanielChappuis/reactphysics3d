/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2020 Daniel Chappuis                                       *
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
#include <reactphysics3d/systems/SolveSliderJointSystem.h>
#include <reactphysics3d/engine/PhysicsWorld.h>
#include <reactphysics3d/body/RigidBody.h>

using namespace reactphysics3d;

// Static variables definition
const decimal SolveSliderJointSystem::BETA = decimal(0.2);

// Constructor
SolveSliderJointSystem::SolveSliderJointSystem(PhysicsWorld& world, RigidBodyComponents& rigidBodyComponents,
                                                             TransformComponents& transformComponents,
                                                             JointComponents& jointComponents,
                                                             SliderJointComponents& sliderJointComponents)
              :mWorld(world), mRigidBodyComponents(rigidBodyComponents), mTransformComponents(transformComponents),
               mJointComponents(jointComponents), mSliderJointComponents(sliderJointComponents),
               mTimeStep(0), mIsWarmStartingActive(true) {

}

// Initialize before solving the constraint
void SolveSliderJointSystem::initBeforeSolve() {

    // For each joint
    for (uint32 i=0; i < mSliderJointComponents.getNbEnabledComponents(); i++) {

        const Entity jointEntity = mSliderJointComponents.mJointEntities[i];

        // Get the bodies entities
        const Entity body1Entity = mJointComponents.getBody1Entity(jointEntity);
        const Entity body2Entity = mJointComponents.getBody2Entity(jointEntity);

        assert(!mRigidBodyComponents.getIsEntityDisabled(body1Entity));
        assert(!mRigidBodyComponents.getIsEntityDisabled(body2Entity));

        // Get the inertia tensor of bodies
        mSliderJointComponents.mI1[i] = RigidBody::getWorldInertiaTensorInverse(mWorld, body1Entity);
        mSliderJointComponents.mI2[i] = RigidBody::getWorldInertiaTensorInverse(mWorld, body2Entity);
    }

    // For each joint
    for (uint32 i=0; i < mSliderJointComponents.getNbEnabledComponents(); i++) {

        const Entity jointEntity = mSliderJointComponents.mJointEntities[i];

        // Get the bodies entities
        const Entity body1Entity = mJointComponents.getBody1Entity(jointEntity);
        const Entity body2Entity = mJointComponents.getBody2Entity(jointEntity);

        const Quaternion& orientationBody1 = mTransformComponents.getTransform(body1Entity).getOrientation();
        const Quaternion& orientationBody2 = mTransformComponents.getTransform(body2Entity).getOrientation();

        // Vector from body center to the anchor point
        mSliderJointComponents.mR1[i] = orientationBody1 * mSliderJointComponents.mLocalAnchorPointBody1[i];
        mSliderJointComponents.mR2[i] = orientationBody2 * mSliderJointComponents.mLocalAnchorPointBody2[i];
    }

    // For each joint
    for (uint32 i=0; i < mSliderJointComponents.getNbEnabledComponents(); i++) {

        const Entity jointEntity = mSliderJointComponents.mJointEntities[i];

        const Entity body1Entity = mJointComponents.getBody1Entity(jointEntity);
        const Quaternion& orientationBody1 = mTransformComponents.getTransform(body1Entity).getOrientation();

        // Compute the two orthogonal vectors to the slider axis in world-space
        mSliderJointComponents.mSliderAxisWorld[i] = orientationBody1 * mSliderJointComponents.mSliderAxisBody1[i];
        mSliderJointComponents.mSliderAxisWorld[i].normalize();
    }

    // For each joint
    for (uint32 i=0; i < mSliderJointComponents.getNbEnabledComponents(); i++) {

        mSliderJointComponents.mN1[i] = mSliderJointComponents.mSliderAxisWorld[i].getOneUnitOrthogonalVector();
        mSliderJointComponents.mN2[i] = mSliderJointComponents.mSliderAxisWorld[i].cross(mSliderJointComponents.mN1[i]);
    }

    const decimal biasFactor = (BETA / mTimeStep);

    // For each joint
    for (uint32 i=0; i < mSliderJointComponents.getNbEnabledComponents(); i++) {

        const Entity jointEntity = mSliderJointComponents.mJointEntities[i];

        // Get the bodies entities
        const Entity body1Entity = mJointComponents.getBody1Entity(jointEntity);
        const Entity body2Entity = mJointComponents.getBody2Entity(jointEntity);

        const uint32 componentIndexBody1 = mRigidBodyComponents.getEntityIndex(body1Entity);
        const uint32 componentIndexBody2 = mRigidBodyComponents.getEntityIndex(body2Entity);

        const Vector3& x1 = mRigidBodyComponents.mCentersOfMassWorld[componentIndexBody1];
        const Vector3& x2 = mRigidBodyComponents.mCentersOfMassWorld[componentIndexBody2];

        const Vector3& r1 = mSliderJointComponents.mR1[i];
        const Vector3& r2 = mSliderJointComponents.mR2[i];

        // Compute the vector u (difference between anchor points)
        const Vector3 u = x2 + r2 - x1 - r1;

        // Compute the cross products used in the Jacobians
        const Vector3 r1PlusU = mSliderJointComponents.mR1[i] + u;
        mSliderJointComponents.mR1PlusUCrossN1[i] = r1PlusU.cross(mSliderJointComponents.mN1[i]);
        mSliderJointComponents.mR1PlusUCrossN2[i] = r1PlusU.cross(mSliderJointComponents.mN2[i]);
        mSliderJointComponents.mR1PlusUCrossSliderAxis[i] = r1PlusU.cross(mSliderJointComponents.mSliderAxisWorld[i]);

        // Check if the limit constraints are violated or not
        decimal uDotSliderAxis = u.dot(mSliderJointComponents.mSliderAxisWorld[i]);
        decimal lowerLimitError = uDotSliderAxis - mSliderJointComponents.mLowerLimit[i];
        decimal upperLimitError = mSliderJointComponents.mUpperLimit[i] - uDotSliderAxis;
        bool oldIsLowerLimitViolated = mSliderJointComponents.mIsLowerLimitViolated[i];
        mSliderJointComponents.mIsLowerLimitViolated[i] = lowerLimitError <= 0;
        if (!mSliderJointComponents.mIsLowerLimitViolated[i] || mSliderJointComponents.mIsLowerLimitViolated[i] != oldIsLowerLimitViolated) {
            mSliderJointComponents.mImpulseLowerLimit[i] = decimal(0.0);
        }
        bool oldIsUpperLimitViolated = mSliderJointComponents.mIsUpperLimitViolated[i];
        mSliderJointComponents.mIsUpperLimitViolated[i] = upperLimitError <= 0;
        if (!mSliderJointComponents.mIsUpperLimitViolated[i] || mSliderJointComponents.mIsUpperLimitViolated[i] != oldIsUpperLimitViolated) {
            mSliderJointComponents.mImpulseUpperLimit[i] = decimal(0.0);
        }

        // Compute the bias "b" of the translation constraint
        mSliderJointComponents.mBiasTranslation[i].setToZero();
        if (mJointComponents.getPositionCorrectionTechnique(jointEntity) == JointsPositionCorrectionTechnique::BAUMGARTE_JOINTS) {
            mSliderJointComponents.mBiasTranslation[i].x = u.dot(mSliderJointComponents.mN1[i]);
            mSliderJointComponents.mBiasTranslation[i].y = u.dot(mSliderJointComponents.mN2[i]);
            mSliderJointComponents.mBiasTranslation[i] *= biasFactor;
        }

        // If the limits are enabled
        if (mSliderJointComponents.mIsLimitEnabled[i] && (mSliderJointComponents.mIsLowerLimitViolated[i] ||
                                                                          mSliderJointComponents.mIsUpperLimitViolated[i])) {

            const Vector3& r2CrossSliderAxis = mSliderJointComponents.mR2CrossSliderAxis[i];
            const Vector3& r1PlusUCrossSliderAxis = mSliderJointComponents.mR1PlusUCrossSliderAxis[i];

            const decimal body1MassInverse = mRigidBodyComponents.mInverseMasses[componentIndexBody1];
            const decimal body2MassInverse = mRigidBodyComponents.mInverseMasses[componentIndexBody2];
            const decimal sumInverseMass = body1MassInverse + body2MassInverse;

            // Compute the inverse of the mass matrix K=JM^-1J^t for the limits (1x1 matrix)
            mSliderJointComponents.mInverseMassMatrixLimit[i] = sumInverseMass +
                                      r1PlusUCrossSliderAxis.dot(mSliderJointComponents.mI1[i] * r1PlusUCrossSliderAxis) +
                                      r2CrossSliderAxis.dot(mSliderJointComponents.mI2[i] * r2CrossSliderAxis);
            mSliderJointComponents.mInverseMassMatrixLimit[i] = (mSliderJointComponents.mInverseMassMatrixLimit[i] > decimal(0.0)) ?
                                      decimal(1.0) / mSliderJointComponents.mInverseMassMatrixLimit[i] : decimal(0.0);

            // Compute the bias "b" of the lower limit constraint
            mSliderJointComponents.mBLowerLimit[i] = decimal(0.0);
            if (mJointComponents.getPositionCorrectionTechnique(jointEntity) == JointsPositionCorrectionTechnique::BAUMGARTE_JOINTS) {
                mSliderJointComponents.mBLowerLimit[i] = biasFactor * lowerLimitError;
            }

            // Compute the bias "b" of the upper limit constraint
            mSliderJointComponents.mBUpperLimit[i] = decimal(0.0);
            if (mJointComponents.getPositionCorrectionTechnique(jointEntity) == JointsPositionCorrectionTechnique::BAUMGARTE_JOINTS) {
                mSliderJointComponents.mBUpperLimit[i] = biasFactor * upperLimitError;
            }
        }
    }

    // For each joint
    for (uint32 i=0; i < mSliderJointComponents.getNbEnabledComponents(); i++) {

        // Compute the cross products used in the Jacobians
        mSliderJointComponents.mR2CrossN1[i] = mSliderJointComponents.mR2[i].cross(mSliderJointComponents.mN1[i]);
        mSliderJointComponents.mR2CrossN2[i] = mSliderJointComponents.mR2[i].cross(mSliderJointComponents.mN2[i]);
        mSliderJointComponents.mR2CrossSliderAxis[i] = mSliderJointComponents.mR2[i].cross(mSliderJointComponents.mSliderAxisWorld[i]);
    }

    // For each joint
    for (uint32 i=0; i < mSliderJointComponents.getNbEnabledComponents(); i++) {

        const Entity jointEntity = mSliderJointComponents.mJointEntities[i];

        // Get the bodies entities
        const Entity body1Entity = mJointComponents.getBody1Entity(jointEntity);
        const Entity body2Entity = mJointComponents.getBody2Entity(jointEntity);

        const Vector3& r2CrossN1 = mSliderJointComponents.mR2CrossN1[i];
        const Vector3& r2CrossN2 = mSliderJointComponents.mR2CrossN2[i];
        const Vector3& r1PlusUCrossN1 = mSliderJointComponents.mR1PlusUCrossN1[i];
        const Vector3& r1PlusUCrossN2 = mSliderJointComponents.mR1PlusUCrossN2[i];

        const Matrix3x3& i1 = mSliderJointComponents.mI1[i];
        const Matrix3x3& i2 = mSliderJointComponents.mI2[i];

        const uint32 componentIndexBody1 = mRigidBodyComponents.getEntityIndex(body1Entity);
        const uint32 componentIndexBody2 = mRigidBodyComponents.getEntityIndex(body2Entity);

        // Compute the inverse of the mass matrix K=JM^-1J^t for the 2 translation
        // constraints (2x2 matrix)
        const decimal body1MassInverse = mRigidBodyComponents.mInverseMasses[componentIndexBody1];
        const decimal body2MassInverse = mRigidBodyComponents.mInverseMasses[componentIndexBody2];
        const decimal sumInverseMass = body1MassInverse + body2MassInverse;
        Vector3 I1R1PlusUCrossN1 = i1 * r1PlusUCrossN1;
        Vector3 I1R1PlusUCrossN2 = i1 * r1PlusUCrossN2;
        Vector3 I2R2CrossN1 = i2 * r2CrossN1;
        Vector3 I2R2CrossN2 = i2 * r2CrossN2;
        const decimal el11 = sumInverseMass + r1PlusUCrossN1.dot(I1R1PlusUCrossN1) +
                             r2CrossN1.dot(I2R2CrossN1);
        const decimal el12 = r1PlusUCrossN1.dot(I1R1PlusUCrossN2) +
                             r2CrossN1.dot(I2R2CrossN2);
        const decimal el21 = r1PlusUCrossN2.dot(I1R1PlusUCrossN1) +
                             r2CrossN2.dot(I2R2CrossN1);
        const decimal el22 = sumInverseMass + r1PlusUCrossN2.dot(I1R1PlusUCrossN2) +
                             r2CrossN2.dot(I2R2CrossN2);
        Matrix2x2 matrixKTranslation(el11, el12, el21, el22);
        mSliderJointComponents.mInverseMassMatrixTranslation[i].setToZero();
        if (mRigidBodyComponents.mBodyTypes[componentIndexBody1] == BodyType::DYNAMIC ||
            mRigidBodyComponents.mBodyTypes[componentIndexBody2] == BodyType::DYNAMIC) {

            mSliderJointComponents.mInverseMassMatrixTranslation[i] = matrixKTranslation.getInverse();
        }

        // Compute the inverse of the mass matrix K=JM^-1J^t for the 3 rotation
        // contraints (3x3 matrix)
        mSliderJointComponents.mInverseMassMatrixRotation[i] = i1 + i2;
        if (mRigidBodyComponents.mBodyTypes[componentIndexBody1] == BodyType::DYNAMIC ||
            mRigidBodyComponents.mBodyTypes[componentIndexBody2] == BodyType::DYNAMIC) {

            mSliderJointComponents.mInverseMassMatrixRotation[i] = mSliderJointComponents.mInverseMassMatrixRotation[i].getInverse();
        }
    }

    // For each joint
    for (uint32 i=0; i < mSliderJointComponents.getNbEnabledComponents(); i++) {

        const Entity jointEntity = mSliderJointComponents.mJointEntities[i];

        // Get the bodies entities
        const Entity body1Entity = mJointComponents.getBody1Entity(jointEntity);
        const Entity body2Entity = mJointComponents.getBody2Entity(jointEntity);

        const Quaternion& orientationBody1 = mTransformComponents.getTransform(body1Entity).getOrientation();
        const Quaternion& orientationBody2 = mTransformComponents.getTransform(body2Entity).getOrientation();

        // Compute the bias "b" of the rotation constraint
        mSliderJointComponents.mBiasRotation[i].setToZero();
        if (mJointComponents.getPositionCorrectionTechnique(jointEntity) == JointsPositionCorrectionTechnique::BAUMGARTE_JOINTS) {
            const Quaternion qError = orientationBody2 * mSliderJointComponents.mInitOrientationDifferenceInv[i] * orientationBody1.getInverse();
            mSliderJointComponents.mBiasRotation[i] = biasFactor * decimal(2.0) * qError.getVectorV();
        }

        // If the motor is enabled
        if (mSliderJointComponents.mIsMotorEnabled[i]) {

            const decimal body1MassInverse = mRigidBodyComponents.getMassInverse(body1Entity);
            const decimal body2MassInverse = mRigidBodyComponents.getMassInverse(body2Entity);
            const decimal sumInverseMass = body1MassInverse + body2MassInverse;

            // Compute the inverse of mass matrix K=JM^-1J^t for the motor (1x1 matrix)
            mSliderJointComponents.mInverseMassMatrixMotor[i] = sumInverseMass;
            mSliderJointComponents.mInverseMassMatrixMotor[i] = (mSliderJointComponents.mInverseMassMatrixMotor[i] > decimal(0.0)) ?
                        decimal(1.0) / mSliderJointComponents.mInverseMassMatrixMotor[i] : decimal(0.0);
        }
    }

    // If warm-starting is not enabled
    if (!mIsWarmStartingActive) {

        // For each joint
        for (uint32 i=0; i < mSliderJointComponents.getNbEnabledComponents(); i++) {

            // Reset all the accumulated impulses
            mSliderJointComponents.mImpulseTranslation[i].setToZero();
            mSliderJointComponents.mImpulseRotation[i].setToZero();
            mSliderJointComponents.mImpulseLowerLimit[i] = decimal(0.0);
            mSliderJointComponents.mImpulseUpperLimit[i] = decimal(0.0);
            mSliderJointComponents.mImpulseMotor[i] = decimal(0.0);
        }
    }
}

// Warm start the constraint (apply the previous impulse at the beginning of the step)
void SolveSliderJointSystem::warmstart() {

    // For each joint component
    for (uint32 i=0; i < mSliderJointComponents.getNbEnabledComponents(); i++) {

        const Entity jointEntity = mSliderJointComponents.mJointEntities[i];

        // Get the bodies entities
        const Entity body1Entity = mJointComponents.getBody1Entity(jointEntity);
        const Entity body2Entity = mJointComponents.getBody2Entity(jointEntity);

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

        const Vector3& n1 = mSliderJointComponents.mN1[i];
        const Vector3& n2 = mSliderJointComponents.mN2[i];

        // Compute the impulse P=J^T * lambda for the lower and upper limits constraints of body 1
        decimal impulseLimits = mSliderJointComponents.mImpulseUpperLimit[i] - mSliderJointComponents.mImpulseLowerLimit[i];
        Vector3 linearImpulseLimits = impulseLimits * mSliderJointComponents.mSliderAxisWorld[i];

        // Compute the impulse P=J^T * lambda for the motor constraint of body 1
        Vector3 impulseMotor = mSliderJointComponents.mImpulseMotor[i] * mSliderJointComponents.mSliderAxisWorld[i];

        const Vector2& impulseTranslation = mSliderJointComponents.mImpulseTranslation[i];
        const Vector3& impulseRotation = mSliderJointComponents.mImpulseRotation[i];

        // Compute the impulse P=J^T * lambda for the 2 translation constraints of body 1
        Vector3 linearImpulseBody1 = -n1 * impulseTranslation.x - n2 * impulseTranslation.y;
        Vector3 angularImpulseBody1 = -mSliderJointComponents.mR1PlusUCrossN1[i] * impulseTranslation.x -
                mSliderJointComponents.mR1PlusUCrossN2[i] * impulseTranslation.y;

        // Compute the impulse P=J^T * lambda for the 3 rotation constraints of body 1
        angularImpulseBody1 += -impulseRotation;

        // Compute the impulse P=J^T * lambda for the lower and upper limits constraints of body 1
        linearImpulseBody1 += linearImpulseLimits;
        angularImpulseBody1 += impulseLimits * mSliderJointComponents.mR1PlusUCrossSliderAxis[i];

        // Compute the impulse P=J^T * lambda for the motor constraint of body 1
        linearImpulseBody1 += impulseMotor;

        // Apply the impulse to the body 1
        v1 += inverseMassBody1 * linearImpulseBody1;
        w1 += mSliderJointComponents.mI1[i] * angularImpulseBody1;

        // Compute the impulse P=J^T * lambda for the 2 translation constraints of body 2
        Vector3 linearImpulseBody2 = n1 * impulseTranslation.x + n2 * impulseTranslation.y;
        Vector3 angularImpulseBody2 = mSliderJointComponents.mR2CrossN1[i] * impulseTranslation.x +
                mSliderJointComponents.mR2CrossN2[i] * impulseTranslation.y;

        // Compute the impulse P=J^T * lambda for the 3 rotation constraints of body 2
        angularImpulseBody2 += impulseRotation;

        // Compute the impulse P=J^T * lambda for the lower and upper limits constraints of body 2
        linearImpulseBody2 += -linearImpulseLimits;
        angularImpulseBody2 += -impulseLimits * mSliderJointComponents.mR2CrossSliderAxis[i];

        // Compute the impulse P=J^T * lambda for the motor constraint of body 2
        linearImpulseBody2 += -impulseMotor;

        // Apply the impulse to the body 2
        v2 += inverseMassBody2 * linearImpulseBody2;
        w2 += mSliderJointComponents.mI2[i] * angularImpulseBody2;
    }
}

// Solve the velocity constraint
void SolveSliderJointSystem::solveVelocityConstraint() {

    // For each joint component
    for (uint32 i=0; i < mSliderJointComponents.getNbEnabledComponents(); i++) {

        const Entity jointEntity = mSliderJointComponents.mJointEntities[i];

        // Get the bodies entities
        const Entity body1Entity = mJointComponents.getBody1Entity(jointEntity);
        const Entity body2Entity = mJointComponents.getBody2Entity(jointEntity);

        const uint32 componentIndexBody1 = mRigidBodyComponents.getEntityIndex(body1Entity);
        const uint32 componentIndexBody2 = mRigidBodyComponents.getEntityIndex(body2Entity);

        // Get the velocities
        Vector3& v1 = mRigidBodyComponents.mConstrainedLinearVelocities[componentIndexBody1];
        Vector3& v2 = mRigidBodyComponents.mConstrainedLinearVelocities[componentIndexBody2];
        Vector3& w1 = mRigidBodyComponents.mConstrainedAngularVelocities[componentIndexBody1];
        Vector3& w2 = mRigidBodyComponents.mConstrainedAngularVelocities[componentIndexBody2];

        const Matrix3x3& i1 = mSliderJointComponents.mI1[i];
        const Matrix3x3& i2 = mSliderJointComponents.mI2[i];

        const Vector3& n1 = mSliderJointComponents.mN1[i];
        const Vector3& n2 = mSliderJointComponents.mN2[i];

        const Vector3& r2CrossN1 = mSliderJointComponents.mR2CrossN1[i];
        const Vector3& r2CrossN2 = mSliderJointComponents.mR2CrossN2[i];
        const Vector3& r1PlusUCrossN1 = mSliderJointComponents.mR1PlusUCrossN1[i];
        const Vector3& r1PlusUCrossN2 = mSliderJointComponents.mR1PlusUCrossN2[i];

        // Get the inverse mass and inverse inertia tensors of the bodies
        decimal inverseMassBody1 = mRigidBodyComponents.mInverseMasses[componentIndexBody1];
        decimal inverseMassBody2 = mRigidBodyComponents.mInverseMasses[componentIndexBody2];

        // --------------- Translation Constraints --------------- //

        // Compute J*v for the 2 translation constraints
        const decimal el1 = -n1.dot(v1) - w1.dot(r1PlusUCrossN1) +
                             n1.dot(v2) + w2.dot(r2CrossN1);
        const decimal el2 = -n2.dot(v1) - w1.dot(r1PlusUCrossN2) +
                             n2.dot(v2) + w2.dot(r2CrossN2);
        const Vector2 JvTranslation(el1, el2);

        // Compute the Lagrange multiplier lambda for the 2 translation constraints
        Vector2 deltaLambda = mSliderJointComponents.mInverseMassMatrixTranslation[i] * (-JvTranslation - mSliderJointComponents.mBiasTranslation[i]);
        mSliderJointComponents.mImpulseTranslation[i] += deltaLambda;

        // Compute the impulse P=J^T * lambda for the 2 translation constraints of body 1
        const Vector3 linearImpulseBody1 = -n1 * deltaLambda.x - n2 * deltaLambda.y;
        Vector3 angularImpulseBody1 = -r1PlusUCrossN1 * deltaLambda.x -
                r1PlusUCrossN2 * deltaLambda.y;

        // Apply the impulse to the body 1
        v1 += inverseMassBody1 * linearImpulseBody1;
        w1 += i1 * angularImpulseBody1;

        // Compute the impulse P=J^T * lambda for the 2 translation constraints of body 2
        const Vector3 linearImpulseBody2 = n1 * deltaLambda.x + n2 * deltaLambda.y;
        Vector3 angularImpulseBody2 = r2CrossN1 * deltaLambda.x + r2CrossN2 * deltaLambda.y;

        // Apply the impulse to the body 2
        v2 += inverseMassBody2 * linearImpulseBody2;
        w2 += i2 * angularImpulseBody2;
    }

    // For each joint component
    for (uint32 i=0; i < mSliderJointComponents.getNbEnabledComponents(); i++) {

        const Entity jointEntity = mSliderJointComponents.mJointEntities[i];

        // Get the bodies entities
        const Entity body1Entity = mJointComponents.getBody1Entity(jointEntity);
        const Entity body2Entity = mJointComponents.getBody2Entity(jointEntity);

        const uint32 componentIndexBody1 = mRigidBodyComponents.getEntityIndex(body1Entity);
        const uint32 componentIndexBody2 = mRigidBodyComponents.getEntityIndex(body2Entity);

        // --------------- Rotation Constraints --------------- //

        Vector3& w1 = mRigidBodyComponents.mConstrainedAngularVelocities[componentIndexBody1];
        Vector3& w2 = mRigidBodyComponents.mConstrainedAngularVelocities[componentIndexBody2];

        // Compute J*v for the 3 rotation constraints
        const Vector3 JvRotation = w2 - w1;

        // Compute the Lagrange multiplier lambda for the 3 rotation constraints
        Vector3 deltaLambda2 = mSliderJointComponents.mInverseMassMatrixRotation[i] *
                               (-JvRotation - mSliderJointComponents.getBiasRotation(jointEntity));
        mSliderJointComponents.mImpulseRotation[i] += deltaLambda2;

        // Compute the impulse P=J^T * lambda for the 3 rotation constraints of body 1
        Vector3 angularImpulseBody1 = -deltaLambda2;

        // Apply the impulse to the body to body 1
        w1 += mSliderJointComponents.mI1[i] * angularImpulseBody1;

        // Compute the impulse P=J^T * lambda for the 3 rotation constraints of body 2
        Vector3 angularImpulseBody2 = deltaLambda2;

        // Apply the impulse to the body 2
        w2 += mSliderJointComponents.mI2[i] * angularImpulseBody2;
    }

    // For each joint component
    for (uint32 i=0; i < mSliderJointComponents.getNbEnabledComponents(); i++) {

        const Entity jointEntity = mSliderJointComponents.mJointEntities[i];

        // Get the bodies entities
        const Entity body1Entity = mJointComponents.getBody1Entity(jointEntity);
        const Entity body2Entity = mJointComponents.getBody2Entity(jointEntity);

        const uint32 componentIndexBody1 = mRigidBodyComponents.getEntityIndex(body1Entity);
        const uint32 componentIndexBody2 = mRigidBodyComponents.getEntityIndex(body2Entity);

        Vector3& v1 = mRigidBodyComponents.mConstrainedLinearVelocities[componentIndexBody1];
        Vector3& v2 = mRigidBodyComponents.mConstrainedLinearVelocities[componentIndexBody2];

        decimal inverseMassBody1 = mRigidBodyComponents.mInverseMasses[componentIndexBody1];
        decimal inverseMassBody2 = mRigidBodyComponents.mInverseMasses[componentIndexBody2];

        const Vector3& r2CrossSliderAxis = mSliderJointComponents.mR2CrossSliderAxis[i];
        const Vector3& r1PlusUCrossSliderAxis = mSliderJointComponents.mR1PlusUCrossSliderAxis[i];

        const Vector3& sliderAxisWorld = mSliderJointComponents.mSliderAxisWorld[i];

        // --------------- Limits Constraints --------------- //

        if (mSliderJointComponents.mIsLimitEnabled[i]) {

            Vector3& w1 = mRigidBodyComponents.mConstrainedAngularVelocities[componentIndexBody1];
            Vector3& w2 = mRigidBodyComponents.mConstrainedAngularVelocities[componentIndexBody2];

            const decimal inverseMassMatrixLimit = mSliderJointComponents.mInverseMassMatrixLimit[i];

            // If the lower limit is violated
            if (mSliderJointComponents.mIsLowerLimitViolated[i]) {

                // Compute J*v for the lower limit constraint
                const decimal JvLowerLimit = sliderAxisWorld.dot(v2) + r2CrossSliderAxis.dot(w2) -
                                             sliderAxisWorld.dot(v1) - r1PlusUCrossSliderAxis.dot(w1);

                // Compute the Lagrange multiplier lambda for the lower limit constraint
                decimal deltaLambdaLower = inverseMassMatrixLimit * (-JvLowerLimit - mSliderJointComponents.mBLowerLimit[i]);
                decimal lambdaTemp = mSliderJointComponents.mImpulseLowerLimit[i];
                mSliderJointComponents.mImpulseLowerLimit[i] = std::max(mSliderJointComponents.mImpulseLowerLimit[i] + deltaLambdaLower, decimal(0.0));
                deltaLambdaLower = mSliderJointComponents.mImpulseLowerLimit[i] - lambdaTemp;

                // Compute the impulse P=J^T * lambda for the lower limit constraint of body 1
                const Vector3 linearImpulseBody1 = -deltaLambdaLower * sliderAxisWorld;
                const Vector3 angularImpulseBody1 = -deltaLambdaLower * r1PlusUCrossSliderAxis;

                // Apply the impulse to the body 1
                v1 += inverseMassBody1 * linearImpulseBody1;
                w1 += mSliderJointComponents.mI1[i] * angularImpulseBody1;

                // Compute the impulse P=J^T * lambda for the lower limit constraint of body 2
                const Vector3 linearImpulseBody2 = deltaLambdaLower * sliderAxisWorld;
                const Vector3 angularImpulseBody2 = deltaLambdaLower * r2CrossSliderAxis;

                // Apply the impulse to the body 2
                v2 += inverseMassBody2 * linearImpulseBody2;
                w2 += mSliderJointComponents.mI2[i] * angularImpulseBody2;
            }

            // If the upper limit is violated
            if (mSliderJointComponents.mIsUpperLimitViolated[i]) {

                // Compute J*v for the upper limit constraint
                const decimal JvUpperLimit = sliderAxisWorld.dot(v1) + r1PlusUCrossSliderAxis.dot(w1)
                                            - sliderAxisWorld.dot(v2) - r2CrossSliderAxis.dot(w2);

                // Compute the Lagrange multiplier lambda for the upper limit constraint
                decimal deltaLambdaUpper = inverseMassMatrixLimit * (-JvUpperLimit -mSliderJointComponents.mBUpperLimit[i]);
                decimal lambdaTemp = mSliderJointComponents.mImpulseUpperLimit[i];
                mSliderJointComponents.mImpulseUpperLimit[i] = std::max(mSliderJointComponents.mImpulseUpperLimit[i] + deltaLambdaUpper, decimal(0.0));
                deltaLambdaUpper = mSliderJointComponents.mImpulseUpperLimit[i] - lambdaTemp;

                // Compute the impulse P=J^T * lambda for the upper limit constraint of body 1
                const Vector3 linearImpulseBody1 = deltaLambdaUpper * sliderAxisWorld;
                const Vector3 angularImpulseBody1 = deltaLambdaUpper * r1PlusUCrossSliderAxis;

                // Apply the impulse to the body 1
                v1 += inverseMassBody1 * linearImpulseBody1;
                w1 += mSliderJointComponents.mI1[i] * angularImpulseBody1;

                // Compute the impulse P=J^T * lambda for the upper limit constraint of body 2
                const Vector3 linearImpulseBody2 = -deltaLambdaUpper * sliderAxisWorld;
                const Vector3 angularImpulseBody2 = -deltaLambdaUpper * r2CrossSliderAxis;

                // Apply the impulse to the body 2
                v2 += inverseMassBody2 * linearImpulseBody2;
                w2 += mSliderJointComponents.mI2[i] * angularImpulseBody2;
            }
        }

        // --------------- Motor --------------- //

        if (mSliderJointComponents.mIsMotorEnabled[i]) {

            // Compute J*v for the motor
            const decimal JvMotor = sliderAxisWorld.dot(v1) - sliderAxisWorld.dot(v2);

            // Compute the Lagrange multiplier lambda for the motor
            const decimal maxMotorImpulse = mSliderJointComponents.mMaxMotorForce[i] * mTimeStep;
            decimal deltaLambdaMotor = mSliderJointComponents.mInverseMassMatrixMotor[i] * (-JvMotor - mSliderJointComponents.mMotorSpeed[i]);
            decimal lambdaTemp = mSliderJointComponents.mImpulseMotor[i];
            mSliderJointComponents.mImpulseMotor[i] = clamp(mSliderJointComponents.mImpulseMotor[i] + deltaLambdaMotor, -maxMotorImpulse, maxMotorImpulse);
            deltaLambdaMotor = mSliderJointComponents.mImpulseMotor[i] - lambdaTemp;

            // Compute the impulse P=J^T * lambda for the motor of body 1
            const Vector3 linearImpulseBody1 = deltaLambdaMotor * sliderAxisWorld;

            // Apply the impulse to the body 1
            v1 += inverseMassBody1 * linearImpulseBody1;

            // Compute the impulse P=J^T * lambda for the motor of body 2
            const Vector3 linearImpulseBody2 = -deltaLambdaMotor * sliderAxisWorld;

            // Apply the impulse to the body 2
            v2 += inverseMassBody2 * linearImpulseBody2;
        }
    }
}

// Solve the position constraint (for position error correction)
void SolveSliderJointSystem::solvePositionConstraint() {

    // For each joint component
    for (uint32 i=0; i < mSliderJointComponents.getNbEnabledComponents(); i++) {

        const Entity jointEntity = mSliderJointComponents.mJointEntities[i];

        // If the error position correction technique is not the non-linear-gauss-seidel, we do
        // do not execute this method
        if (mJointComponents.getPositionCorrectionTechnique(jointEntity) != JointsPositionCorrectionTechnique::NON_LINEAR_GAUSS_SEIDEL) return;

        // Get the bodies entities
        const Entity body1Entity = mJointComponents.getBody1Entity(jointEntity);
        const Entity body2Entity = mJointComponents.getBody2Entity(jointEntity);

        // Recompute the inverse inertia tensors
        mSliderJointComponents.mI1[i] = RigidBody::getWorldInertiaTensorInverse(mWorld, body1Entity);
        mSliderJointComponents.mI2[i] = RigidBody::getWorldInertiaTensorInverse(mWorld, body2Entity);
    }

    // For each joint component
    for (uint32 i=0; i < mSliderJointComponents.getNbEnabledComponents(); i++) {

        const Entity jointEntity = mSliderJointComponents.mJointEntities[i];

        // If the error position correction technique is not the non-linear-gauss-seidel, we do
        // do not execute this method
        if (mJointComponents.getPositionCorrectionTechnique(jointEntity) != JointsPositionCorrectionTechnique::NON_LINEAR_GAUSS_SEIDEL) return;

        // Get the bodies entities
        const Entity body1Entity = mJointComponents.getBody1Entity(jointEntity);
        const Entity body2Entity = mJointComponents.getBody2Entity(jointEntity);

        const Quaternion& q1 = mRigidBodyComponents.getConstrainedOrientation(body1Entity);
        const Quaternion& q2 = mRigidBodyComponents.getConstrainedOrientation(body2Entity);

        // Vector from body center to the anchor point
        mSliderJointComponents.mR1[i] = q1 * mSliderJointComponents.mLocalAnchorPointBody1[i];
        mSliderJointComponents.mR2[i] = q2 * mSliderJointComponents.mLocalAnchorPointBody2[i];
    }

    // For each joint component
    for (uint32 i=0; i < mSliderJointComponents.getNbEnabledComponents(); i++) {

        const Entity jointEntity = mSliderJointComponents.mJointEntities[i];

        // If the error position correction technique is not the non-linear-gauss-seidel, we do
        // do not execute this method
        if (mJointComponents.getPositionCorrectionTechnique(jointEntity) != JointsPositionCorrectionTechnique::NON_LINEAR_GAUSS_SEIDEL) return;

        // Get the bodies entities
        const Entity body1Entity = mJointComponents.getBody1Entity(jointEntity);
        const Entity body2Entity = mJointComponents.getBody2Entity(jointEntity);

        const uint32 componentIndexBody1 = mRigidBodyComponents.getEntityIndex(body1Entity);
        const uint32 componentIndexBody2 = mRigidBodyComponents.getEntityIndex(body2Entity);

        // Get the inverse mass and inverse inertia tensors of the bodies
        const decimal inverseMassBody1 = mRigidBodyComponents.mInverseMasses[componentIndexBody1];
        const decimal inverseMassBody2 = mRigidBodyComponents.mInverseMasses[componentIndexBody2];

        const Vector3& r1 = mSliderJointComponents.mR1[i];
        const Vector3& r2 = mSliderJointComponents.mR2[i];

        const Vector3& n1 = mSliderJointComponents.mN1[i];
        const Vector3& n2 = mSliderJointComponents.mN2[i];

        Vector3& x1 = mRigidBodyComponents.mConstrainedPositions[componentIndexBody1];
        Vector3& x2 = mRigidBodyComponents.mConstrainedPositions[componentIndexBody2];

        // Compute the vector u (difference between anchor points)
        const Vector3 u = x2 + r2 - x1 - r1;

        Quaternion& q1 = mRigidBodyComponents.mConstrainedOrientations[componentIndexBody1];
        Quaternion& q2 = mRigidBodyComponents.mConstrainedOrientations[componentIndexBody2];

        // Compute the two orthogonal vectors to the slider axis in world-space
        mSliderJointComponents.mSliderAxisWorld[i] = q1 * mSliderJointComponents.mSliderAxisBody1[i];
        mSliderJointComponents.mSliderAxisWorld[i].normalize();
        mSliderJointComponents.mN1[i] = mSliderJointComponents.mSliderAxisWorld[i].getOneUnitOrthogonalVector();
        mSliderJointComponents.mN2[i] = mSliderJointComponents.mSliderAxisWorld[i].cross(n1);

        // Check if the limit constraints are violated or not
        decimal uDotSliderAxis = u.dot(mSliderJointComponents.mSliderAxisWorld[i]);
        decimal lowerLimitError = uDotSliderAxis - mSliderJointComponents.getLowerLimit(jointEntity);
        decimal upperLimitError = mSliderJointComponents.getUpperLimit(jointEntity) - uDotSliderAxis;
        mSliderJointComponents.mIsLowerLimitViolated[i] = lowerLimitError <= 0;
        mSliderJointComponents.mIsUpperLimitViolated[i] = upperLimitError <= 0;

        // Compute the cross products used in the Jacobians
        mSliderJointComponents.mR2CrossN1[i] = r2.cross(n1);
        mSliderJointComponents.mR2CrossN2[i] = r2.cross(n2);
        mSliderJointComponents.mR2CrossSliderAxis[i] = r2.cross(mSliderJointComponents.mSliderAxisWorld[i]);
        const Vector3 r1PlusU = r1 + u;
        mSliderJointComponents.mR1PlusUCrossN1[i] = r1PlusU.cross(n1);
        mSliderJointComponents.mR1PlusUCrossN2[i] = r1PlusU.cross(n2);
        mSliderJointComponents.mR1PlusUCrossSliderAxis[i] = r1PlusU.cross(mSliderJointComponents.mSliderAxisWorld[i]);

        const Vector3& r2CrossN1 = mSliderJointComponents.mR2CrossN1[i];
        const Vector3& r2CrossN2 = mSliderJointComponents.mR2CrossN2[i];
        const Vector3& r1PlusUCrossN1 = mSliderJointComponents.mR1PlusUCrossN1[i];
        const Vector3& r1PlusUCrossN2 = mSliderJointComponents.mR1PlusUCrossN2[i];

        // --------------- Translation Constraints --------------- //

        const Matrix3x3& i1 = mSliderJointComponents.getI1(jointEntity);
        const Matrix3x3& i2 = mSliderJointComponents.getI2(jointEntity);

        // Recompute the inverse of the mass matrix K=JM^-1J^t for the 2 translation
        // constraints (2x2 matrix)
        const decimal body1MassInverse = mRigidBodyComponents.mInverseMasses[componentIndexBody1];
        const decimal body2MassInverse = mRigidBodyComponents.mInverseMasses[componentIndexBody2];
        decimal sumInverseMass = body1MassInverse + body2MassInverse;
        Vector3 I1R1PlusUCrossN1 = i1 * r1PlusUCrossN1;
        Vector3 I1R1PlusUCrossN2 = i1 * r1PlusUCrossN2;
        Vector3 I2R2CrossN1 = i2 * r2CrossN1;
        Vector3 I2R2CrossN2 = i2 * r2CrossN2;
        const decimal el11 = sumInverseMass + r1PlusUCrossN1.dot(I1R1PlusUCrossN1) +
                             r2CrossN1.dot(I2R2CrossN1);
        const decimal el12 = r1PlusUCrossN1.dot(I1R1PlusUCrossN2) +
                             r2CrossN1.dot(I2R2CrossN2);
        const decimal el21 = r1PlusUCrossN2.dot(I1R1PlusUCrossN1) +
                             r2CrossN2.dot(I2R2CrossN1);
        const decimal el22 = sumInverseMass + r1PlusUCrossN2.dot(I1R1PlusUCrossN2) +
                             r2CrossN2.dot(I2R2CrossN2);
        Matrix2x2 matrixKTranslation(el11, el12, el21, el22);
        mSliderJointComponents.mInverseMassMatrixTranslation[i].setToZero();
        if (mRigidBodyComponents.mBodyTypes[componentIndexBody1] == BodyType::DYNAMIC || mRigidBodyComponents.mBodyTypes[componentIndexBody2] == BodyType::DYNAMIC) {

            mSliderJointComponents.mInverseMassMatrixTranslation[i] = matrixKTranslation.getInverse();
        }

        // Compute the position error for the 2 translation constraints
        const Vector2 translationError(u.dot(n1), u.dot(n2));

        // Compute the Lagrange multiplier lambda for the 2 translation constraints
        Vector2 lambdaTranslation = mSliderJointComponents.mInverseMassMatrixTranslation[i] * (-translationError);

        // Compute the impulse P=J^T * lambda for the 2 translation constraints of body 1
        const Vector3 linearImpulseBody1 = -n1 * lambdaTranslation.x - n2 * lambdaTranslation.y;
        Vector3 angularImpulseBody1 = -r1PlusUCrossN1 * lambdaTranslation.x -
                                            r1PlusUCrossN2 * lambdaTranslation.y;

        // Apply the impulse to the body 1
        const Vector3 v1 = inverseMassBody1 * linearImpulseBody1;
        Vector3 w1 = i1 * angularImpulseBody1;

        // Update the body position/orientation of body 1
        x1 += v1;
        q1 += Quaternion(0, w1) * q1 * decimal(0.5);
        q1.normalize();

        // Compute the impulse P=J^T * lambda for the 2 translation constraints of body 2
        const Vector3 linearImpulseBody2 = n1 * lambdaTranslation.x + n2 * lambdaTranslation.y;
        Vector3 angularImpulseBody2 = r2CrossN1 * lambdaTranslation.x + r2CrossN2 * lambdaTranslation.y;

        // Apply the impulse to the body 2
        const Vector3 v2 = inverseMassBody2 * linearImpulseBody2;
        Vector3 w2 = i2 * angularImpulseBody2;

        // Update the body position/orientation of body 2
        x2 += v2;
        q2 += Quaternion(0, w2) * q2 * decimal(0.5);
        q2.normalize();
    }

    // For each joint component
    for (uint32 i=0; i < mSliderJointComponents.getNbEnabledComponents(); i++) {

        const Entity jointEntity = mSliderJointComponents.mJointEntities[i];

        // If the error position correction technique is not the non-linear-gauss-seidel, we do
        // do not execute this method
        if (mJointComponents.getPositionCorrectionTechnique(jointEntity) != JointsPositionCorrectionTechnique::NON_LINEAR_GAUSS_SEIDEL) return;

        // Get the bodies entities
        const Entity body1Entity = mJointComponents.getBody1Entity(jointEntity);
        const Entity body2Entity = mJointComponents.getBody2Entity(jointEntity);

        const uint32 componentIndexBody1 = mRigidBodyComponents.getEntityIndex(body1Entity);
        const uint32 componentIndexBody2 = mRigidBodyComponents.getEntityIndex(body2Entity);

        Quaternion& q1 = mRigidBodyComponents.mConstrainedOrientations[componentIndexBody1];
        Quaternion& q2 = mRigidBodyComponents.mConstrainedOrientations[componentIndexBody2];

        // Get the velocities
        Vector3& w1 = mRigidBodyComponents.mConstrainedAngularVelocities[componentIndexBody1];
        Vector3& w2 = mRigidBodyComponents.mConstrainedAngularVelocities[componentIndexBody2];

        // --------------- Rotation Constraints --------------- //

        // Compute the inverse of the mass matrix K=JM^-1J^t for the 3 rotation
        // contraints (3x3 matrix)
        mSliderJointComponents.mInverseMassMatrixRotation[i] = mSliderJointComponents.mI1[i] + mSliderJointComponents.mI2[i];
        if (mRigidBodyComponents.mBodyTypes[componentIndexBody1] == BodyType::DYNAMIC || mRigidBodyComponents.mBodyTypes[componentIndexBody2] == BodyType::DYNAMIC) {

            mSliderJointComponents.mInverseMassMatrixRotation[i] = mSliderJointComponents.mInverseMassMatrixRotation[i].getInverse();
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
        Quaternion qError = q2 * mSliderJointComponents.mInitOrientationDifferenceInv[i] * q1.getInverse();

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
        Vector3 lambdaRotation = mSliderJointComponents.mInverseMassMatrixRotation[i] * (-errorRotation);

        // Compute the impulse P=J^T * lambda for the 3 rotation constraints of body 1
        Vector3 angularImpulseBody1 = -lambdaRotation;

        // Apply the impulse to the body 1
        w1 = mSliderJointComponents.mI1[i] * angularImpulseBody1;

        // Update the body position/orientation of body 1
        q1 += Quaternion(0, w1) * q1 * decimal(0.5);
        q1.normalize();

        // Compute the impulse P=J^T * lambda for the 3 rotation constraints of body 2
        Vector3 angularImpulseBody2 = lambdaRotation;

        // Apply the impulse to the body 2
        w2 = mSliderJointComponents.mI2[i] * angularImpulseBody2;

        // Update the body position/orientation of body 2
        q2 += Quaternion(0, w2) * q2 * decimal(0.5);
        q2.normalize();

        // --------------- Limits Constraints --------------- //

        if (mSliderJointComponents.mIsLimitEnabled[i]) {

            Vector3& x1 = mRigidBodyComponents.mConstrainedPositions[componentIndexBody1];
            Vector3& x2 = mRigidBodyComponents.mConstrainedPositions[componentIndexBody2];

            const Vector3& r2CrossSliderAxis = mSliderJointComponents.mR2CrossSliderAxis[i];
            const Vector3& r1PlusUCrossSliderAxis = mSliderJointComponents.mR1PlusUCrossSliderAxis[i];

            if (mSliderJointComponents.mIsLowerLimitViolated[i] || mSliderJointComponents.mIsUpperLimitViolated[i]) {

                // Compute the inverse of the mass matrix K=JM^-1J^t for the limits (1x1 matrix)
                const decimal body1MassInverse = mRigidBodyComponents.mInverseMasses[componentIndexBody1];
                const decimal body2MassInverse = mRigidBodyComponents.mInverseMasses[componentIndexBody2];
                mSliderJointComponents.mInverseMassMatrixLimit[i] = body1MassInverse + body2MassInverse +
                                        r1PlusUCrossSliderAxis.dot(mSliderJointComponents.mI1[i] * r1PlusUCrossSliderAxis) +
                                        r2CrossSliderAxis.dot(mSliderJointComponents.mI2[i] * r2CrossSliderAxis);
                mSliderJointComponents.mInverseMassMatrixLimit[i] = (mSliderJointComponents.mInverseMassMatrixLimit[i] > decimal(0.0)) ?
                                          decimal(1.0) / mSliderJointComponents.mInverseMassMatrixLimit[i] : decimal(0.0);
            }

            const decimal inverseMassBody1 = mRigidBodyComponents.mInverseMasses[componentIndexBody1];
            const decimal inverseMassBody2 = mRigidBodyComponents.mInverseMasses[componentIndexBody2];

            // If the lower limit is violated
            if (mSliderJointComponents.mIsLowerLimitViolated[i]) {

                const Vector3& r1 = mSliderJointComponents.mR1[i];
                const Vector3& r2 = mSliderJointComponents.mR2[i];
                const Vector3 u = x2 + r2 - x1 - r1;
                decimal uDotSliderAxis = u.dot(mSliderJointComponents.mSliderAxisWorld[i]);
                decimal lowerLimitError = uDotSliderAxis - mSliderJointComponents.mLowerLimit[i];

                // Compute the Lagrange multiplier lambda for the lower limit constraint
                decimal lambdaLowerLimit = mSliderJointComponents.mInverseMassMatrixLimit[i] * (-lowerLimitError);

                // Compute the impulse P=J^T * lambda for the lower limit constraint of body 1
                const Vector3 linearImpulseBody1 = -lambdaLowerLimit * mSliderJointComponents.mSliderAxisWorld[i];
                const Vector3 angularImpulseBody1 = -lambdaLowerLimit * r1PlusUCrossSliderAxis;

                // Apply the impulse to the body 1
                const Vector3 v1 = inverseMassBody1 * linearImpulseBody1;
                const Vector3 w1 = mSliderJointComponents.mI1[i] * angularImpulseBody1;

                // Update the body position/orientation of body 1
                x1 += v1;
                q1 += Quaternion(0, w1) * q1 * decimal(0.5);
                q1.normalize();

                // Compute the impulse P=J^T * lambda for the lower limit constraint of body 2
                const Vector3 linearImpulseBody2 = lambdaLowerLimit * mSliderJointComponents.mSliderAxisWorld[i];
                const Vector3 angularImpulseBody2 = lambdaLowerLimit * r2CrossSliderAxis;

                // Apply the impulse to the body 2
                const Vector3 v2 = inverseMassBody2 * linearImpulseBody2;
                const Vector3 w2 = mSliderJointComponents.mI2[i] * angularImpulseBody2;

                // Update the body position/orientation of body 2
                x2 += v2;
                q2 += Quaternion(0, w2) * q2 * decimal(0.5);
                q2.normalize();
            }

            // If the upper limit is violated
            if (mSliderJointComponents.mIsUpperLimitViolated[i]) {

                const Vector3& r1 = mSliderJointComponents.mR1[i];
                const Vector3& r2 = mSliderJointComponents.mR2[i];
                const Vector3 u = x2 + r2 - x1 - r1;
                decimal uDotSliderAxis = u.dot(mSliderJointComponents.mSliderAxisWorld[i]);
                decimal upperLimitError = mSliderJointComponents.mUpperLimit[i] - uDotSliderAxis;

                // Compute the Lagrange multiplier lambda for the upper limit constraint
                decimal lambdaUpperLimit = mSliderJointComponents.mInverseMassMatrixLimit[i] * (-upperLimitError);

                // Compute the impulse P=J^T * lambda for the upper limit constraint of body 1
                const Vector3 linearImpulseBody1 = lambdaUpperLimit * mSliderJointComponents.mSliderAxisWorld[i];
                const Vector3 angularImpulseBody1 = lambdaUpperLimit * r1PlusUCrossSliderAxis;

                // Apply the impulse to the body 1
                const Vector3 v1 = inverseMassBody1 * linearImpulseBody1;
                const Vector3 w1 = mSliderJointComponents.mI1[i] * angularImpulseBody1;

                // Update the body position/orientation of body 1
                x1 += v1;
                q1 += Quaternion(0, w1) * q1 * decimal(0.5);
                q1.normalize();

                // Compute the impulse P=J^T * lambda for the upper limit constraint of body 2
                const Vector3 linearImpulseBody2 = -lambdaUpperLimit * mSliderJointComponents.mSliderAxisWorld[i];
                const Vector3 angularImpulseBody2 = -lambdaUpperLimit * r2CrossSliderAxis;

                // Apply the impulse to the body 2
                const Vector3 v2 = inverseMassBody2 * linearImpulseBody2;
                const Vector3 w2 = mSliderJointComponents.mI2[i] * angularImpulseBody2;

                // Update the body position/orientation of body 2
                x2 += v2;
                q2 += Quaternion(0, w2) * q2 * decimal(0.5);
                q2.normalize();
            }
        }
    }
}
