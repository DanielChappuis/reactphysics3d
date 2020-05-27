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
#include <reactphysics3d/systems/SolveBallAndSocketJointSystem.h>
#include <reactphysics3d/engine/PhysicsWorld.h>
#include <reactphysics3d/body/RigidBody.h>

using namespace reactphysics3d;

// Static variables definition
const decimal SolveBallAndSocketJointSystem::BETA = decimal(0.2);

// Constructor
SolveBallAndSocketJointSystem::SolveBallAndSocketJointSystem(PhysicsWorld& world, RigidBodyComponents& rigidBodyComponents,
                                                             TransformComponents& transformComponents,
                                                             JointComponents& jointComponents,
                                                             BallAndSocketJointComponents& ballAndSocketJointComponents)
              :mWorld(world), mRigidBodyComponents(rigidBodyComponents), mTransformComponents(transformComponents),
               mJointComponents(jointComponents), mBallAndSocketJointComponents(ballAndSocketJointComponents),
               mTimeStep(0), mIsWarmStartingActive(true) {

}

// Initialize before solving the constraint
void SolveBallAndSocketJointSystem::initBeforeSolve() {

    // For each joint
    for (uint32 i=0; i < mBallAndSocketJointComponents.getNbEnabledComponents(); i++) {

        const Entity jointEntity = mBallAndSocketJointComponents.mJointEntities[i];

        // Get the bodies entities
        const Entity body1Entity = mJointComponents.getBody1Entity(jointEntity);
        const Entity body2Entity = mJointComponents.getBody2Entity(jointEntity);

        assert(!mRigidBodyComponents.getIsEntityDisabled(body1Entity));
        assert(!mRigidBodyComponents.getIsEntityDisabled(body2Entity));

        // Get the inertia tensor of bodies
        mBallAndSocketJointComponents.mI1[i] = RigidBody::getWorldInertiaTensorInverse(mWorld, body1Entity);
        mBallAndSocketJointComponents.mI2[i] = RigidBody::getWorldInertiaTensorInverse(mWorld, body2Entity);
    }

    // For each joint
    for (uint32 i=0; i < mBallAndSocketJointComponents.getNbEnabledComponents(); i++) {

        const Entity jointEntity = mBallAndSocketJointComponents.mJointEntities[i];

        // Get the bodies entities
        const Entity body1Entity = mJointComponents.getBody1Entity(jointEntity);
        const Entity body2Entity = mJointComponents.getBody2Entity(jointEntity);

        const Quaternion& orientationBody1 = mTransformComponents.getTransform(body1Entity).getOrientation();
        const Quaternion& orientationBody2 = mTransformComponents.getTransform(body2Entity).getOrientation();

        // Compute the vector from body center to the anchor point in world-space
        mBallAndSocketJointComponents.mR1World[i] = orientationBody1 * mBallAndSocketJointComponents.mLocalAnchorPointBody1[i];
        mBallAndSocketJointComponents.mR2World[i] = orientationBody2 * mBallAndSocketJointComponents.mLocalAnchorPointBody2[i];
    }

    // For each joint
    for (uint32 i=0; i < mBallAndSocketJointComponents.getNbEnabledComponents(); i++) {

        const Entity jointEntity = mBallAndSocketJointComponents.mJointEntities[i];

        // Compute the corresponding skew-symmetric matrices
        const Vector3& r1World = mBallAndSocketJointComponents.mR1World[i];
        const Vector3& r2World = mBallAndSocketJointComponents.mR2World[i];
        Matrix3x3 skewSymmetricMatrixU1 = Matrix3x3::computeSkewSymmetricMatrixForCrossProduct(r1World);
        Matrix3x3 skewSymmetricMatrixU2 = Matrix3x3::computeSkewSymmetricMatrixForCrossProduct(r2World);

        // Get the bodies entities
        const Entity body1Entity = mJointComponents.getBody1Entity(jointEntity);
        const Entity body2Entity = mJointComponents.getBody2Entity(jointEntity);

        const uint32 componentIndexBody1 = mRigidBodyComponents.getEntityIndex(body1Entity);
        const uint32 componentIndexBody2 = mRigidBodyComponents.getEntityIndex(body2Entity);

        // Compute the matrix K=JM^-1J^t (3x3 matrix)
        const decimal body1MassInverse = mRigidBodyComponents.mInverseMasses[componentIndexBody1];
        const decimal body2MassInverse = mRigidBodyComponents.mInverseMasses[componentIndexBody2];
        const decimal inverseMassBodies =  body1MassInverse + body2MassInverse;
        const Matrix3x3& i1 = mBallAndSocketJointComponents.mI1[i];
        const Matrix3x3& i2 = mBallAndSocketJointComponents.mI2[i];
        Matrix3x3 massMatrix = Matrix3x3(inverseMassBodies, 0, 0,
                                        0, inverseMassBodies, 0,
                                        0, 0, inverseMassBodies) +
                               skewSymmetricMatrixU1 * i1 * skewSymmetricMatrixU1.getTranspose() +
                               skewSymmetricMatrixU2 * i2 * skewSymmetricMatrixU2.getTranspose();

        // Compute the inverse mass matrix K^-1
        mBallAndSocketJointComponents.mInverseMassMatrix[i].setToZero();
        if (mRigidBodyComponents.mBodyTypes[componentIndexBody1] == BodyType::DYNAMIC ||
            mRigidBodyComponents.mBodyTypes[componentIndexBody2] == BodyType::DYNAMIC) {
            mBallAndSocketJointComponents.mInverseMassMatrix[i] = massMatrix.getInverse();
        }
    }

    const decimal biasFactor = (BETA / mTimeStep);

    // For each joint
    for (uint32 i=0; i < mBallAndSocketJointComponents.getNbEnabledComponents(); i++) {

        const Entity jointEntity = mBallAndSocketJointComponents.mJointEntities[i];

        // Get the bodies entities
        const Entity body1Entity = mJointComponents.getBody1Entity(jointEntity);
        const Entity body2Entity = mJointComponents.getBody2Entity(jointEntity);

        const Vector3& r1World = mBallAndSocketJointComponents.mR1World[i];
        const Vector3& r2World = mBallAndSocketJointComponents.mR2World[i];

        const Vector3& x1 = mRigidBodyComponents.getCenterOfMassWorld(body1Entity);
        const Vector3& x2 = mRigidBodyComponents.getCenterOfMassWorld(body2Entity);

        // Compute the bias "b" of the constraint
        mBallAndSocketJointComponents.mBiasVector[i].setToZero();
        if (mJointComponents.getPositionCorrectionTechnique(jointEntity) == JointsPositionCorrectionTechnique::BAUMGARTE_JOINTS) {
            mBallAndSocketJointComponents.mBiasVector[i] = biasFactor * (x2 + r2World - x1 - r1World);
        }
    }

    // If warm-starting is not enabled
    if (!mIsWarmStartingActive) {

        // For each joint
        for (uint32 i=0; i < mBallAndSocketJointComponents.getNbEnabledComponents(); i++) {

            // Reset the accumulated impulse
            mBallAndSocketJointComponents.mImpulse[i].setToZero();
        }
    }
}

// Warm start the constraint (apply the previous impulse at the beginning of the step)
void SolveBallAndSocketJointSystem::warmstart() {

    // For each joint component
    for (uint32 i=0; i < mBallAndSocketJointComponents.getNbEnabledComponents(); i++) {

        const Entity jointEntity = mBallAndSocketJointComponents.mJointEntities[i];

        const Entity body1Entity = mJointComponents.getBody1Entity(jointEntity);
        const Entity body2Entity = mJointComponents.getBody2Entity(jointEntity);

        const uint32 componentIndexBody1 = mRigidBodyComponents.getEntityIndex(body1Entity);
        const uint32 componentIndexBody2 = mRigidBodyComponents.getEntityIndex(body2Entity);

        // Get the velocities
        Vector3& v1 = mRigidBodyComponents.mConstrainedLinearVelocities[componentIndexBody1];
        Vector3& v2 = mRigidBodyComponents.mConstrainedLinearVelocities[componentIndexBody2];
        Vector3& w1 = mRigidBodyComponents.mConstrainedAngularVelocities[componentIndexBody1];
        Vector3& w2 = mRigidBodyComponents.mConstrainedAngularVelocities[componentIndexBody2];

        const Vector3& r1World = mBallAndSocketJointComponents.mR1World[i];
        const Vector3& r2World = mBallAndSocketJointComponents.mR2World[i];

        const Matrix3x3& i1 = mBallAndSocketJointComponents.mI1[i];
        const Matrix3x3& i2 = mBallAndSocketJointComponents.mI2[i];

        // Compute the impulse P=J^T * lambda for the body 1
        const Vector3 linearImpulseBody1 = -mBallAndSocketJointComponents.mImpulse[i];
        const Vector3 angularImpulseBody1 = mBallAndSocketJointComponents.mImpulse[i].cross(r1World);

        // Apply the impulse to the body 1
        v1 += mRigidBodyComponents.mInverseMasses[componentIndexBody1] * linearImpulseBody1;
        w1 += i1 * angularImpulseBody1;

        // Compute the impulse P=J^T * lambda for the body 2
        const Vector3 angularImpulseBody2 = -mBallAndSocketJointComponents.mImpulse[i].cross(r2World);

        // Apply the impulse to the body to the body 2
        v2 += mRigidBodyComponents.mInverseMasses[componentIndexBody2] * mBallAndSocketJointComponents.mImpulse[i];
        w2 += i2 * angularImpulseBody2;
    }
}

// Solve the velocity constraint
void SolveBallAndSocketJointSystem::solveVelocityConstraint() {

    // For each joint component
    for (uint32 i=0; i < mBallAndSocketJointComponents.getNbEnabledComponents(); i++) {

        const Entity jointEntity = mBallAndSocketJointComponents.mJointEntities[i];

        const Entity body1Entity = mJointComponents.getBody1Entity(jointEntity);
        const Entity body2Entity = mJointComponents.getBody2Entity(jointEntity);

        const uint32 componentIndexBody1 = mRigidBodyComponents.getEntityIndex(body1Entity);
        const uint32 componentIndexBody2 = mRigidBodyComponents.getEntityIndex(body2Entity);

        // Get the velocities
        Vector3& v1 = mRigidBodyComponents.mConstrainedLinearVelocities[componentIndexBody1];
        Vector3& v2 = mRigidBodyComponents.mConstrainedLinearVelocities[componentIndexBody2];
        Vector3& w1 = mRigidBodyComponents.mConstrainedAngularVelocities[componentIndexBody1];
        Vector3& w2 = mRigidBodyComponents.mConstrainedAngularVelocities[componentIndexBody2];

        const Matrix3x3& i1 = mBallAndSocketJointComponents.mI1[i];
        const Matrix3x3& i2 = mBallAndSocketJointComponents.mI2[i];

        // Compute J*v
        const Vector3 Jv = v2 + w2.cross(mBallAndSocketJointComponents.mR2World[i]) - v1 - w1.cross(mBallAndSocketJointComponents.mR1World[i]);

        // Compute the Lagrange multiplier lambda
        const Vector3 deltaLambda = mBallAndSocketJointComponents.mInverseMassMatrix[i] * (-Jv - mBallAndSocketJointComponents.mBiasVector[i]);
        mBallAndSocketJointComponents.mImpulse[i] += deltaLambda;

        // Compute the impulse P=J^T * lambda for the body 1
        const Vector3 linearImpulseBody1 = -deltaLambda;
        const Vector3 angularImpulseBody1 = deltaLambda.cross(mBallAndSocketJointComponents.mR1World[i]);

        // Apply the impulse to the body 1
        v1 += mRigidBodyComponents.mInverseMasses[componentIndexBody1] * linearImpulseBody1;
        w1 += i1 * angularImpulseBody1;

        // Compute the impulse P=J^T * lambda for the body 2
        const Vector3 angularImpulseBody2 = -deltaLambda.cross(mBallAndSocketJointComponents.mR2World[i]);

        // Apply the impulse to the body 2
        v2 += mRigidBodyComponents.mInverseMasses[componentIndexBody2] * deltaLambda;
        w2 += i2 * angularImpulseBody2;
    }
}

// Solve the position constraint (for position error correction)
void SolveBallAndSocketJointSystem::solvePositionConstraint() {

    // For each joint component
    for (uint32 i=0; i < mBallAndSocketJointComponents.getNbEnabledComponents(); i++) {

        const Entity jointEntity = mBallAndSocketJointComponents.mJointEntities[i];

        // If the error position correction technique is not the non-linear-gauss-seidel, we do
        // do not execute this method
        if (mJointComponents.getPositionCorrectionTechnique(jointEntity) != JointsPositionCorrectionTechnique::NON_LINEAR_GAUSS_SEIDEL) continue;

        const Entity body1Entity = mJointComponents.getBody1Entity(jointEntity);
        const Entity body2Entity = mJointComponents.getBody2Entity(jointEntity);

        // Recompute the inverse inertia tensors
        mBallAndSocketJointComponents.mI1[i] = RigidBody::getWorldInertiaTensorInverse(mWorld, body1Entity);
        mBallAndSocketJointComponents.mI2[i] = RigidBody::getWorldInertiaTensorInverse(mWorld, body2Entity);
    }

    // For each joint component
    for (uint32 i=0; i < mBallAndSocketJointComponents.getNbEnabledComponents(); i++) {

        const Entity jointEntity = mBallAndSocketJointComponents.mJointEntities[i];

        // If the error position correction technique is not the non-linear-gauss-seidel, we do
        // do not execute this method
        if (mJointComponents.getPositionCorrectionTechnique(jointEntity) != JointsPositionCorrectionTechnique::NON_LINEAR_GAUSS_SEIDEL) continue;

        const Entity body1Entity = mJointComponents.getBody1Entity(jointEntity);
        const Entity body2Entity = mJointComponents.getBody2Entity(jointEntity);

        // Compute the vector from body center to the anchor point in world-space
        mBallAndSocketJointComponents.mR1World[i] = mRigidBodyComponents.getConstrainedOrientation(body1Entity) *
                                                    mBallAndSocketJointComponents.mLocalAnchorPointBody1[i];
        mBallAndSocketJointComponents.mR2World[i] = mRigidBodyComponents.getConstrainedOrientation(body2Entity) *
                                                    mBallAndSocketJointComponents.mLocalAnchorPointBody2[i];
    }

    // For each joint component
    for (uint32 i=0; i < mBallAndSocketJointComponents.getNbEnabledComponents(); i++) {

        const Entity jointEntity = mBallAndSocketJointComponents.mJointEntities[i];

        // If the error position correction technique is not the non-linear-gauss-seidel, we do
        // do not execute this method
        if (mJointComponents.getPositionCorrectionTechnique(jointEntity) != JointsPositionCorrectionTechnique::NON_LINEAR_GAUSS_SEIDEL) continue;

        const Entity body1Entity = mJointComponents.getBody1Entity(jointEntity);
        const Entity body2Entity = mJointComponents.getBody2Entity(jointEntity);

        const uint32 componentIndexBody1 = mRigidBodyComponents.getEntityIndex(body1Entity);
        const uint32 componentIndexBody2 = mRigidBodyComponents.getEntityIndex(body2Entity);

        const Vector3& r1World = mBallAndSocketJointComponents.mR1World[i];
        const Vector3& r2World = mBallAndSocketJointComponents.mR2World[i];

        // Compute the corresponding skew-symmetric matrices
        Matrix3x3 skewSymmetricMatrixU1 = Matrix3x3::computeSkewSymmetricMatrixForCrossProduct(r1World);
        Matrix3x3 skewSymmetricMatrixU2 = Matrix3x3::computeSkewSymmetricMatrixForCrossProduct(r2World);

        // Get the inverse mass and inverse inertia tensors of the bodies
        const decimal inverseMassBody1 = mRigidBodyComponents.mInverseMasses[componentIndexBody1];
        const decimal inverseMassBody2 = mRigidBodyComponents.mInverseMasses[componentIndexBody2];

        // Recompute the inverse mass matrix K=J^TM^-1J of of the 3 translation constraints
        decimal inverseMassBodies = inverseMassBody1 + inverseMassBody2;
        Matrix3x3 massMatrix = Matrix3x3(inverseMassBodies, 0, 0,
                                        0, inverseMassBodies, 0,
                                        0, 0, inverseMassBodies) +
                               skewSymmetricMatrixU1 * mBallAndSocketJointComponents.mI1[i] * skewSymmetricMatrixU1.getTranspose() +
                               skewSymmetricMatrixU2 * mBallAndSocketJointComponents.mI2[i] * skewSymmetricMatrixU2.getTranspose();
        mBallAndSocketJointComponents.mInverseMassMatrix[i].setToZero();
        if (mRigidBodyComponents.mBodyTypes[componentIndexBody1] == BodyType::DYNAMIC ||
            mRigidBodyComponents.mBodyTypes[componentIndexBody2] == BodyType::DYNAMIC) {
            mBallAndSocketJointComponents.mInverseMassMatrix[i] = massMatrix.getInverse();
        }
    }

    // For each joint component
    for (uint32 i=0; i < mBallAndSocketJointComponents.getNbEnabledComponents(); i++) {

        const Entity jointEntity = mBallAndSocketJointComponents.mJointEntities[i];

        // If the error position correction technique is not the non-linear-gauss-seidel, we do
        // do not execute this method
        if (mJointComponents.getPositionCorrectionTechnique(jointEntity) != JointsPositionCorrectionTechnique::NON_LINEAR_GAUSS_SEIDEL) continue;

        const Entity body1Entity = mJointComponents.getBody1Entity(jointEntity);
        const Entity body2Entity = mJointComponents.getBody2Entity(jointEntity);

        const uint32 componentIndexBody1 = mRigidBodyComponents.getEntityIndex(body1Entity);
        const uint32 componentIndexBody2 = mRigidBodyComponents.getEntityIndex(body2Entity);

        Vector3& x1 = mRigidBodyComponents.mConstrainedPositions[componentIndexBody1];
        Vector3& x2 = mRigidBodyComponents.mConstrainedPositions[componentIndexBody2];

        const Vector3& r1World = mBallAndSocketJointComponents.mR1World[i];
        const Vector3& r2World = mBallAndSocketJointComponents.mR2World[i];

        // Compute the constraint error (value of the C(x) function)
        const Vector3 constraintError = (x2 + r2World - x1 - r1World);

        // Compute the Lagrange multiplier lambda
        // TODO : Do not solve the system by computing the inverse each time and multiplying with the
        //        right-hand side vector but instead use a method to directly solve the linear system.
        const Vector3 lambda = mBallAndSocketJointComponents.mInverseMassMatrix[i] * (-constraintError);

        // Compute the impulse of body 1
        const Vector3 linearImpulseBody1 = -lambda;
        const Vector3 angularImpulseBody1 = lambda.cross(r1World);

        // Get the inverse mass and inverse inertia tensors of the bodies
        const decimal inverseMassBody1 = mRigidBodyComponents.mInverseMasses[componentIndexBody1];
        const decimal inverseMassBody2 = mRigidBodyComponents.mInverseMasses[componentIndexBody2];

        // Compute the pseudo velocity of body 1
        const Vector3 v1 = inverseMassBody1 * linearImpulseBody1;
        const Vector3 w1 = mBallAndSocketJointComponents.mI1[i] * angularImpulseBody1;

        Quaternion& q1 = mRigidBodyComponents.mConstrainedOrientations[componentIndexBody1];
        Quaternion& q2 = mRigidBodyComponents.mConstrainedOrientations[componentIndexBody2];

        // Update the body center of mass and orientation of body 1
        x1 += v1;
        q1 += Quaternion(0, w1) * q1 * decimal(0.5);
        q1.normalize();

        // Compute the impulse of body 2
        const Vector3 angularImpulseBody2 = -lambda.cross(r2World);

        // Compute the pseudo velocity of body 2
        const Vector3 v2 = inverseMassBody2 * lambda;
        const Vector3 w2 = mBallAndSocketJointComponents.mI2[i] * angularImpulseBody2;

        // Update the body position/orientation of body 2
        x2 += v2;
        q2 += Quaternion(0, w2) * q2 * decimal(0.5);
        q2.normalize();
    }
}
