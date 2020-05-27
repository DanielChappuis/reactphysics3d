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
#include <reactphysics3d/systems/SolveFixedJointSystem.h>
#include <reactphysics3d/engine/PhysicsWorld.h>
#include <reactphysics3d/body/RigidBody.h>

using namespace reactphysics3d;

// Static variables definition
const decimal SolveFixedJointSystem::BETA = decimal(0.2);

// Constructor
SolveFixedJointSystem::SolveFixedJointSystem(PhysicsWorld& world, RigidBodyComponents& rigidBodyComponents,
                                             TransformComponents& transformComponents,
                                             JointComponents& jointComponents,
                                             FixedJointComponents& fixedJointComponents)
              :mWorld(world), mRigidBodyComponents(rigidBodyComponents), mTransformComponents(transformComponents),
               mJointComponents(jointComponents), mFixedJointComponents(fixedJointComponents),
               mTimeStep(0), mIsWarmStartingActive(true) {

}

// Initialize before solving the constraint
void SolveFixedJointSystem::initBeforeSolve() {

    // For each joint
    for (uint32 i=0; i < mFixedJointComponents.getNbEnabledComponents(); i++) {

        const Entity jointEntity = mFixedJointComponents.mJointEntities[i];

        // Get the bodies entities
        const Entity body1Entity = mJointComponents.getBody1Entity(jointEntity);
        const Entity body2Entity = mJointComponents.getBody2Entity(jointEntity);

        assert(!mRigidBodyComponents.getIsEntityDisabled(body1Entity));
        assert(!mRigidBodyComponents.getIsEntityDisabled(body2Entity));

        // Get the inertia tensor of bodies
        mFixedJointComponents.mI1[i] = RigidBody::getWorldInertiaTensorInverse(mWorld, body1Entity);
        mFixedJointComponents.mI2[i] = RigidBody::getWorldInertiaTensorInverse(mWorld, body2Entity);
    }

    // For each joint
    for (uint32 i=0; i < mFixedJointComponents.getNbEnabledComponents(); i++) {

        const Entity jointEntity = mFixedJointComponents.mJointEntities[i];

        // Get the bodies entities
        const Entity body1Entity = mJointComponents.getBody1Entity(jointEntity);
        const Entity body2Entity = mJointComponents.getBody2Entity(jointEntity);

        const Quaternion& orientationBody1 = mTransformComponents.getTransform(body1Entity).getOrientation();
        const Quaternion& orientationBody2 = mTransformComponents.getTransform(body2Entity).getOrientation();

        // Compute the vector from body center to the anchor point in world-space
        mFixedJointComponents.mR1World[i] = orientationBody1 * mFixedJointComponents.mLocalAnchorPointBody1[i];
        mFixedJointComponents.mR2World[i] = orientationBody2 * mFixedJointComponents.mLocalAnchorPointBody2[i];
    }

    // For each joint
    for (uint32 i=0; i < mFixedJointComponents.getNbEnabledComponents(); i++) {

        const Entity jointEntity = mFixedJointComponents.mJointEntities[i];

        // Get the bodies entities
        const Entity body1Entity = mJointComponents.getBody1Entity(jointEntity);
        const Entity body2Entity = mJointComponents.getBody2Entity(jointEntity);

        // Compute the corresponding skew-symmetric matrices
        Matrix3x3 skewSymmetricMatrixU1 = Matrix3x3::computeSkewSymmetricMatrixForCrossProduct(mFixedJointComponents.mR1World[i]);
        Matrix3x3 skewSymmetricMatrixU2 = Matrix3x3::computeSkewSymmetricMatrixForCrossProduct(mFixedJointComponents.mR2World[i]);

        const uint32 componentIndexBody1 = mRigidBodyComponents.getEntityIndex(body1Entity);
        const uint32 componentIndexBody2 = mRigidBodyComponents.getEntityIndex(body2Entity);

        // Compute the matrix K=JM^-1J^t (3x3 matrix) for the 3 translation constraints
        const decimal body1MassInverse = mRigidBodyComponents.mInverseMasses[componentIndexBody1];
        const decimal body2MassInverse = mRigidBodyComponents.mInverseMasses[componentIndexBody2];
        const decimal inverseMassBodies = body1MassInverse + body2MassInverse;
        Matrix3x3 massMatrix = Matrix3x3(inverseMassBodies, 0, 0,
                                        0, inverseMassBodies, 0,
                                        0, 0, inverseMassBodies) +
                               skewSymmetricMatrixU1 * mFixedJointComponents.mI1[i] * skewSymmetricMatrixU1.getTranspose() +
                               skewSymmetricMatrixU2 * mFixedJointComponents.mI2[i] * skewSymmetricMatrixU2.getTranspose();

        // Compute the inverse mass matrix K^-1 for the 3 translation constraints
        mFixedJointComponents.mInverseMassMatrixTranslation[i].setToZero();
        if (mRigidBodyComponents.mBodyTypes[componentIndexBody1] == BodyType::DYNAMIC ||
            mRigidBodyComponents.mBodyTypes[componentIndexBody2] == BodyType::DYNAMIC) {
            mFixedJointComponents.mInverseMassMatrixTranslation[i] = massMatrix.getInverse();
        }
    }

    const decimal biasFactor = BETA / mTimeStep;

    // For each joint
    for (uint32 i=0; i < mFixedJointComponents.getNbEnabledComponents(); i++) {

        const Entity jointEntity = mFixedJointComponents.mJointEntities[i];

        // Get the bodies entities
        const Entity body1Entity = mJointComponents.getBody1Entity(jointEntity);
        const Entity body2Entity = mJointComponents.getBody2Entity(jointEntity);

        // Get the bodies positions and orientations
        const Vector3& x1 = mRigidBodyComponents.getCenterOfMassWorld(body1Entity);
        const Vector3& x2 = mRigidBodyComponents.getCenterOfMassWorld(body2Entity);

        const Vector3& r1World = mFixedJointComponents.mR1World[i];
        const Vector3& r2World = mFixedJointComponents.mR2World[i];

        // Compute the bias "b" of the constraint for the 3 translation constraints
        mFixedJointComponents.mBiasTranslation[i].setToZero();
        if (mJointComponents.getPositionCorrectionTechnique(jointEntity) == JointsPositionCorrectionTechnique::BAUMGARTE_JOINTS) {
            mFixedJointComponents.mBiasTranslation[i] = biasFactor * (x2 + r2World - x1 - r1World);
        }
    }

    // For each joint
    for (uint32 i=0; i < mFixedJointComponents.getNbEnabledComponents(); i++) {

        const Entity jointEntity = mFixedJointComponents.mJointEntities[i];

        // Get the bodies entities
        const Entity body1Entity = mJointComponents.getBody1Entity(jointEntity);
        const Entity body2Entity = mJointComponents.getBody2Entity(jointEntity);

        // Compute the inverse of the mass matrix K=JM^-1J^t for the 3 rotation contraints (3x3 matrix)
        mFixedJointComponents.mInverseMassMatrixRotation[i] = mFixedJointComponents.mI1[i] + mFixedJointComponents.mI2[i];
        if (mRigidBodyComponents.getBodyType(body1Entity) == BodyType::DYNAMIC ||
            mRigidBodyComponents.getBodyType(body2Entity) == BodyType::DYNAMIC) {
            mFixedJointComponents.mInverseMassMatrixRotation[i] = mFixedJointComponents.mInverseMassMatrixRotation[i].getInverse();
        }
    }

    // For each joint
    for (uint32 i=0; i < mFixedJointComponents.getNbEnabledComponents(); i++) {

        const Entity jointEntity = mFixedJointComponents.mJointEntities[i];

        // Get the bodies entities
        const Entity body1Entity = mJointComponents.getBody1Entity(jointEntity);
        const Entity body2Entity = mJointComponents.getBody2Entity(jointEntity);

        // Compute the bias "b" for the 3 rotation constraints
        mFixedJointComponents.mBiasRotation[i].setToZero();

        const Quaternion& orientationBody1 = mTransformComponents.getTransform(body1Entity).getOrientation();
        const Quaternion& orientationBody2 = mTransformComponents.getTransform(body2Entity).getOrientation();

        if (mJointComponents.getPositionCorrectionTechnique(jointEntity) == JointsPositionCorrectionTechnique::BAUMGARTE_JOINTS) {
            const Quaternion qError = orientationBody2 * mFixedJointComponents.mInitOrientationDifferenceInv[i] * orientationBody1.getInverse();
            mFixedJointComponents.mBiasRotation[i] = biasFactor * decimal(2.0) * qError.getVectorV();
        }
    }

    // If warm-starting is not enabled
    if (!mIsWarmStartingActive) {

        // For each joint
        for (uint32 i=0; i < mFixedJointComponents.getNbEnabledComponents(); i++) {

            // Reset the accumulated impulses
            mFixedJointComponents.mImpulseTranslation[i].setToZero();
            mFixedJointComponents.mImpulseRotation[i].setToZero();
        }
    }
}

// Warm start the constraint (apply the previous impulse at the beginning of the step)
void SolveFixedJointSystem::warmstart() {

    // For each joint
    for (uint32 i=0; i < mFixedJointComponents.getNbEnabledComponents(); i++) {

        const Entity jointEntity = mFixedJointComponents.mJointEntities[i];

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

        // Get the inverse mass of the bodies
        const decimal inverseMassBody1 = mRigidBodyComponents.mInverseMasses[componentIndexBody1];
        const decimal inverseMassBody2 = mRigidBodyComponents.mInverseMasses[componentIndexBody2];

        const Vector3& impulseTranslation = mFixedJointComponents.mImpulseTranslation[i];
        const Vector3& impulseRotation = mFixedJointComponents.mImpulseRotation[i];

        const Vector3& r1World = mFixedJointComponents.mR1World[i];
        const Vector3& r2World = mFixedJointComponents.mR2World[i];

        // Compute the impulse P=J^T * lambda for the 3 translation constraints for body 1
        Vector3 linearImpulseBody1 = -impulseTranslation;
        Vector3 angularImpulseBody1 = impulseTranslation.cross(r1World);

        // Compute the impulse P=J^T * lambda for the 3 rotation constraints for body 1
        angularImpulseBody1 += -impulseRotation;

        const Matrix3x3& i1 = mFixedJointComponents.mI1[i];

        // Apply the impulse to the body 1
        v1 += inverseMassBody1 * linearImpulseBody1;
        w1 += i1 * angularImpulseBody1;

        // Compute the impulse P=J^T * lambda for the 3 translation constraints for body 2
        Vector3 angularImpulseBody2 = -impulseTranslation.cross(r2World);

        // Compute the impulse P=J^T * lambda for the 3 rotation constraints for body 2
        angularImpulseBody2 += impulseRotation;

        const Matrix3x3& i2 = mFixedJointComponents.mI2[i];

        // Apply the impulse to the body 2
        v2 += inverseMassBody2 * impulseTranslation;
        w2 += i2 * angularImpulseBody2;
    }
}

// Solve the velocity constraint
void SolveFixedJointSystem::solveVelocityConstraint() {

    // For each joint
    for (uint32 i=0; i < mFixedJointComponents.getNbEnabledComponents(); i++) {

        const Entity jointEntity = mFixedJointComponents.mJointEntities[i];

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

        // Get the inverse mass of the bodies
        decimal inverseMassBody1 = mRigidBodyComponents.mInverseMasses[componentIndexBody1];
        decimal inverseMassBody2 = mRigidBodyComponents.mInverseMasses[componentIndexBody2];

        const Vector3& r1World = mFixedJointComponents.mR1World[i];
        const Vector3& r2World = mFixedJointComponents.mR2World[i];

        // --------------- Translation Constraints --------------- //

        // Compute J*v for the 3 translation constraints
        const Vector3 JvTranslation = v2 + w2.cross(r2World) - v1 - w1.cross(r1World);

        const Matrix3x3& inverseMassMatrixTranslation = mFixedJointComponents.mInverseMassMatrixTranslation[i];

        // Compute the Lagrange multiplier lambda
        const Vector3 deltaLambda = inverseMassMatrixTranslation * (-JvTranslation - mFixedJointComponents.mBiasTranslation[i]);
        mFixedJointComponents.mImpulseTranslation[i] += deltaLambda;

        // Compute the impulse P=J^T * lambda for body 1
        const Vector3 linearImpulseBody1 = -deltaLambda;
        Vector3 angularImpulseBody1 = deltaLambda.cross(r1World);

        const Matrix3x3& i1 = mFixedJointComponents.mI1[i];

        // Apply the impulse to the body 1
        v1 += inverseMassBody1 * linearImpulseBody1;
        w1 += i1 * angularImpulseBody1;

        // Compute the impulse P=J^T * lambda  for body 2
        const Vector3 angularImpulseBody2 = -deltaLambda.cross(r2World);

        const Matrix3x3& i2 = mFixedJointComponents.mI2[i];

        // Apply the impulse to the body 2
        v2 += inverseMassBody2 * deltaLambda;
        w2 += i2 * angularImpulseBody2;

        // --------------- Rotation Constraints --------------- //

        // Compute J*v for the 3 rotation constraints
        const Vector3 JvRotation = w2 - w1;

        const Vector3& biasRotation = mFixedJointComponents.mBiasRotation[i];
        const Matrix3x3& inverseMassMatrixRotation = mFixedJointComponents.mInverseMassMatrixRotation[i];

        // Compute the Lagrange multiplier lambda for the 3 rotation constraints
        Vector3 deltaLambda2 = inverseMassMatrixRotation * (-JvRotation - biasRotation);
        mFixedJointComponents.mImpulseRotation[i] += deltaLambda2;

        // Compute the impulse P=J^T * lambda for the 3 rotation constraints for body 1
        angularImpulseBody1 = -deltaLambda2;

        // Apply the impulse to the body 1
        w1 += i1 * angularImpulseBody1;

        // Apply the impulse to the body 2
        w2 += i2 * deltaLambda2;
    }
}

// Solve the position constraint (for position error correction)
void SolveFixedJointSystem::solvePositionConstraint() {

    // For each joint
    for (uint32 i=0; i < mFixedJointComponents.getNbEnabledComponents(); i++) {

        const Entity jointEntity = mFixedJointComponents.mJointEntities[i];

        // If the error position correction technique is not the non-linear-gauss-seidel, we do
        // do not execute this method
        if (mJointComponents.getPositionCorrectionTechnique(jointEntity) != JointsPositionCorrectionTechnique::NON_LINEAR_GAUSS_SEIDEL) continue;

        // Get the bodies entities
        const Entity body1Entity = mJointComponents.getBody1Entity(jointEntity);
        const Entity body2Entity = mJointComponents.getBody2Entity(jointEntity);

        // Recompute the inverse inertia tensors
        mFixedJointComponents.mI1[i] = RigidBody::getWorldInertiaTensorInverse(mWorld, body1Entity);
        mFixedJointComponents.mI2[i] = RigidBody::getWorldInertiaTensorInverse(mWorld, body2Entity);
    }

    // For each joint
    for (uint32 i=0; i < mFixedJointComponents.getNbEnabledComponents(); i++) {

        const Entity jointEntity = mFixedJointComponents.mJointEntities[i];

        // If the error position correction technique is not the non-linear-gauss-seidel, we do
        // do not execute this method
        if (mJointComponents.getPositionCorrectionTechnique(jointEntity) != JointsPositionCorrectionTechnique::NON_LINEAR_GAUSS_SEIDEL) continue;

        // Get the bodies entities
        const Entity body1Entity = mJointComponents.getBody1Entity(jointEntity);
        const Entity body2Entity = mJointComponents.getBody2Entity(jointEntity);

        // Get the bodies positions and orientations
        const Quaternion& q1 = mRigidBodyComponents.getConstrainedOrientation(body1Entity);
        const Quaternion& q2 = mRigidBodyComponents.getConstrainedOrientation(body2Entity);

        // Compute the vector from body center to the anchor point in world-space
        mFixedJointComponents.mR1World[i] = q1 * mFixedJointComponents.getLocalAnchorPointBody1(jointEntity);
        mFixedJointComponents.mR2World[i] = q2 * mFixedJointComponents.getLocalAnchorPointBody2(jointEntity);
    }

    // For each joint
    for (uint32 i=0; i < mFixedJointComponents.getNbEnabledComponents(); i++) {

        const Entity jointEntity = mFixedJointComponents.mJointEntities[i];

        // If the error position correction technique is not the non-linear-gauss-seidel, we do
        // do not execute this method
        if (mJointComponents.getPositionCorrectionTechnique(jointEntity) != JointsPositionCorrectionTechnique::NON_LINEAR_GAUSS_SEIDEL) continue;

        // Get the bodies entities
        const Entity body1Entity = mJointComponents.getBody1Entity(jointEntity);
        const Entity body2Entity = mJointComponents.getBody2Entity(jointEntity);

        const uint32 componentIndexBody1 = mRigidBodyComponents.getEntityIndex(body1Entity);
        const uint32 componentIndexBody2 = mRigidBodyComponents.getEntityIndex(body2Entity);

        // Get the inverse mass and inverse inertia tensors of the bodies
        decimal inverseMassBody1 = mRigidBodyComponents.mInverseMasses[componentIndexBody1];
        decimal inverseMassBody2 = mRigidBodyComponents.mInverseMasses[componentIndexBody2];

        const Vector3& r1World = mFixedJointComponents.mR1World[i];
        const Vector3& r2World = mFixedJointComponents.mR2World[i];

        // Compute the corresponding skew-symmetric matrices
        Matrix3x3 skewSymmetricMatrixU1= Matrix3x3::computeSkewSymmetricMatrixForCrossProduct(r1World);
        Matrix3x3 skewSymmetricMatrixU2= Matrix3x3::computeSkewSymmetricMatrixForCrossProduct(r2World);

        // --------------- Translation Constraints --------------- //

        // Compute the matrix K=JM^-1J^t (3x3 matrix) for the 3 translation constraints
        decimal inverseMassBodies = inverseMassBody1 + inverseMassBody2;
        Matrix3x3 massMatrix = Matrix3x3(inverseMassBodies, 0, 0,
                                        0, inverseMassBodies, 0,
                                        0, 0, inverseMassBodies) +
                               skewSymmetricMatrixU1 * mFixedJointComponents.mI1[i] * skewSymmetricMatrixU1.getTranspose() +
                               skewSymmetricMatrixU2 * mFixedJointComponents.mI2[i] * skewSymmetricMatrixU2.getTranspose();
        mFixedJointComponents.mInverseMassMatrixTranslation[i].setToZero();
        if (mRigidBodyComponents.mBodyTypes[componentIndexBody1] == BodyType::DYNAMIC ||
            mRigidBodyComponents.mBodyTypes[componentIndexBody2] == BodyType::DYNAMIC) {
            mFixedJointComponents.mInverseMassMatrixTranslation[i] = massMatrix.getInverse();
        }
    }

    // For each joint
    for (uint32 i=0; i < mFixedJointComponents.getNbEnabledComponents(); i++) {

        const Entity jointEntity = mFixedJointComponents.mJointEntities[i];

        // If the error position correction technique is not the non-linear-gauss-seidel, we do
        // do not execute this method
        if (mJointComponents.getPositionCorrectionTechnique(jointEntity) != JointsPositionCorrectionTechnique::NON_LINEAR_GAUSS_SEIDEL) continue;

        // Get the bodies entities
        const Entity body1Entity = mJointComponents.getBody1Entity(jointEntity);
        const Entity body2Entity = mJointComponents.getBody2Entity(jointEntity);

        const Vector3& r1World = mFixedJointComponents.mR1World[i];
        const Vector3& r2World = mFixedJointComponents.mR2World[i];

        const uint32 componentIndexBody1 = mRigidBodyComponents.getEntityIndex(body1Entity);
        const uint32 componentIndexBody2 = mRigidBodyComponents.getEntityIndex(body2Entity);

        Vector3& x1 = mRigidBodyComponents.mConstrainedPositions[componentIndexBody1];
        Vector3& x2 = mRigidBodyComponents.mConstrainedPositions[componentIndexBody2];
        Quaternion& q1 = mRigidBodyComponents.mConstrainedOrientations[componentIndexBody1];
        Quaternion& q2 = mRigidBodyComponents.mConstrainedOrientations[componentIndexBody2];

        // Compute position error for the 3 translation constraints
        const Vector3 errorTranslation = x2 + r2World - x1 - r1World;

        // Compute the Lagrange multiplier lambda
        const Vector3 lambdaTranslation = mFixedJointComponents.mInverseMassMatrixTranslation[i] * (-errorTranslation);

        // Compute the impulse of body 1
        Vector3 linearImpulseBody1 = -lambdaTranslation;
        Vector3 angularImpulseBody1 = lambdaTranslation.cross(r1World);

        const decimal inverseMassBody1 = mRigidBodyComponents.mInverseMasses[componentIndexBody1];

        // Compute the pseudo velocity of body 1
        const Vector3 v1 = inverseMassBody1 * linearImpulseBody1;
        Vector3 w1 = mFixedJointComponents.mI2[i] * angularImpulseBody1;

        // Update the body position/orientation of body 1
        x1 += v1;
        q1 += Quaternion(0, w1) * q1 * decimal(0.5);
        q1.normalize();

        // Compute the impulse of body 2
        Vector3 angularImpulseBody2 = -lambdaTranslation.cross(r2World);

        const decimal inverseMassBody2 = mRigidBodyComponents.mInverseMasses[componentIndexBody2];

        // Compute the pseudo velocity of body 2
        const Vector3 v2 = inverseMassBody2 * lambdaTranslation;
        Vector3 w2 = mFixedJointComponents.mI2[i] * angularImpulseBody2;

        // Update the body position/orientation of body 2
        x2 += v2;
        q2 += Quaternion(0, w2) * q2 * decimal(0.5);
        q2.normalize();

        // --------------- Rotation Constraints --------------- //

        // Compute the inverse of the mass matrix K=JM^-1J^t for the 3 rotation
        // contraints (3x3 matrix)
        mFixedJointComponents.mInverseMassMatrixRotation[i] = mFixedJointComponents.mI1[i] + mFixedJointComponents.mI2[i];
        if (mRigidBodyComponents.mBodyTypes[componentIndexBody1] == BodyType::DYNAMIC ||
            mRigidBodyComponents.mBodyTypes[componentIndexBody2] == BodyType::DYNAMIC) {
            mFixedJointComponents.mInverseMassMatrixRotation[i] = mFixedJointComponents.mInverseMassMatrixRotation[i].getInverse();
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
        Quaternion qError = q2 * mFixedJointComponents.mInitOrientationDifferenceInv[i] * q1.getInverse();

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
        Vector3 lambdaRotation = mFixedJointComponents.mInverseMassMatrixRotation[i] * (-errorRotation);

        // Compute the impulse P=J^T * lambda for the 3 rotation constraints of body 1
        angularImpulseBody1 = -lambdaRotation;

        // Compute the pseudo velocity of body 1
        w1 = mFixedJointComponents.mI1[i] * angularImpulseBody1;

        // Update the body position/orientation of body 1
        q1 += Quaternion(0, w1) * q1 * decimal(0.5);
        q1.normalize();

        // Compute the pseudo velocity of body 2
        w2 = mFixedJointComponents.mI2[i] * lambdaRotation;

        // Update the body position/orientation of body 2
        q2 += Quaternion(0, w2) * q2 * decimal(0.5);
        q2.normalize();
    }
}
