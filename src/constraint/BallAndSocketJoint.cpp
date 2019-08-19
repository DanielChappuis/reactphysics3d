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
#include "BallAndSocketJoint.h"
#include "systems/ConstraintSolverSystem.h"
#include "components/RigidBodyComponents.h"
#include "engine/DynamicsWorld.h"

using namespace reactphysics3d;

// Static variables definition
const decimal BallAndSocketJoint::BETA = decimal(0.2);

// Constructor
BallAndSocketJoint::BallAndSocketJoint(Entity entity, DynamicsWorld& world, const BallAndSocketJointInfo& jointInfo)
                   : Joint(entity, world, jointInfo), mImpulse(Vector3(0, 0, 0)) {

    // Get the transforms of the two bodies
    Transform& body1Transform = mWorld.mTransformComponents.getTransform(jointInfo.body1->getEntity());
    Transform& body2Transform = mWorld.mTransformComponents.getTransform(jointInfo.body2->getEntity());

    // Compute the local-space anchor point for each body
    mLocalAnchorPointBody1 = body1Transform.getInverse() * jointInfo.anchorPointWorldSpace;
    mLocalAnchorPointBody2 = body2Transform.getInverse() * jointInfo.anchorPointWorldSpace;
}

// Initialize before solving the constraint
void BallAndSocketJoint::initBeforeSolve(const ConstraintSolverData& constraintSolverData) {

    // Get the bodies entities
    Entity body1Entity = mWorld.mJointsComponents.getBody1Entity(mEntity);
    Entity body2Entity = mWorld.mJointsComponents.getBody2Entity(mEntity);

    // TODO : Remove this and use compoents instead of pointers to bodies
    RigidBody* body1 = static_cast<RigidBody*>(mWorld.mRigidBodyComponents.getRigidBody(body1Entity));
    RigidBody* body2 = static_cast<RigidBody*>(mWorld.mRigidBodyComponents.getRigidBody(body2Entity));

    // Get the bodies center of mass and orientations
    const Vector3& x1 = constraintSolverData.rigidBodyComponents.getCenterOfMassWorld(body1Entity);
    const Vector3& x2 = constraintSolverData.rigidBodyComponents.getCenterOfMassWorld(body2Entity);
    const Quaternion& orientationBody1 = body1->getTransform().getOrientation();
    const Quaternion& orientationBody2 = body2->getTransform().getOrientation();

    // Get the inertia tensor of bodies
    mI1 = body1->getInertiaTensorInverseWorld();
    mI2 = body2->getInertiaTensorInverseWorld();

    // Compute the vector from body center to the anchor point in world-space
    mR1World = orientationBody1 * mLocalAnchorPointBody1;
    mR2World = orientationBody2 * mLocalAnchorPointBody2;

    // Compute the corresponding skew-symmetric matrices
    Matrix3x3 skewSymmetricMatrixU1= Matrix3x3::computeSkewSymmetricMatrixForCrossProduct(mR1World);
    Matrix3x3 skewSymmetricMatrixU2= Matrix3x3::computeSkewSymmetricMatrixForCrossProduct(mR2World);

    // Compute the matrix K=JM^-1J^t (3x3 matrix)
    decimal body1MassInverse = constraintSolverData.rigidBodyComponents.getMassInverse(body1->getEntity());
    decimal body2MassInverse = constraintSolverData.rigidBodyComponents.getMassInverse(body2->getEntity());
    decimal inverseMassBodies =  body1MassInverse + body2MassInverse;
    Matrix3x3 massMatrix = Matrix3x3(inverseMassBodies, 0, 0,
                                    0, inverseMassBodies, 0,
                                    0, 0, inverseMassBodies) +
                           skewSymmetricMatrixU1 * mI1 * skewSymmetricMatrixU1.getTranspose() +
                           skewSymmetricMatrixU2 * mI2 * skewSymmetricMatrixU2.getTranspose();

    // Compute the inverse mass matrix K^-1
    mInverseMassMatrix.setToZero();
    if (mWorld.mRigidBodyComponents.getBodyType(body1Entity) == BodyType::DYNAMIC ||
        mWorld.mRigidBodyComponents.getBodyType(body2Entity) == BodyType::DYNAMIC) {
        mInverseMassMatrix = massMatrix.getInverse();
    }

    // Compute the bias "b" of the constraint
    mBiasVector.setToZero();
    if (mWorld.mJointsComponents.getPositionCorrectionTechnique(mEntity) == JointsPositionCorrectionTechnique::BAUMGARTE_JOINTS) {
        decimal biasFactor = (BETA / constraintSolverData.timeStep);
        mBiasVector = biasFactor * (x2 + mR2World - x1 - mR1World);
    }

    // If warm-starting is not enabled
    if (!constraintSolverData.isWarmStartingActive) {

        // Reset the accumulated impulse
        mImpulse.setToZero();
    }
}

// Warm start the constraint (apply the previous impulse at the beginning of the step)
void BallAndSocketJoint::warmstart(const ConstraintSolverData& constraintSolverData) {

    Entity body1Entity = mWorld.mJointsComponents.getBody1Entity(mEntity);
    Entity body2Entity = mWorld.mJointsComponents.getBody2Entity(mEntity);

    uint32 dynamicsComponentIndexBody1 = constraintSolverData.rigidBodyComponents.getEntityIndex(body1Entity);
    uint32 dynamicsComponentIndexBody2 = constraintSolverData.rigidBodyComponents.getEntityIndex(body2Entity);

    // Get the velocities
    Vector3& v1 = constraintSolverData.rigidBodyComponents.mConstrainedLinearVelocities[dynamicsComponentIndexBody1];
    Vector3& v2 = constraintSolverData.rigidBodyComponents.mConstrainedLinearVelocities[dynamicsComponentIndexBody2];
    Vector3& w1 = constraintSolverData.rigidBodyComponents.mConstrainedAngularVelocities[dynamicsComponentIndexBody1];
    Vector3& w2 = constraintSolverData.rigidBodyComponents.mConstrainedAngularVelocities[dynamicsComponentIndexBody2];

    // Compute the impulse P=J^T * lambda for the body 1
    const Vector3 linearImpulseBody1 = -mImpulse;
    const Vector3 angularImpulseBody1 = mImpulse.cross(mR1World);

    // Apply the impulse to the body 1
    v1 += constraintSolverData.rigidBodyComponents.getMassInverse(body1Entity) * linearImpulseBody1;
    w1 += mI1 * angularImpulseBody1;

    // Compute the impulse P=J^T * lambda for the body 2
    const Vector3 angularImpulseBody2 = -mImpulse.cross(mR2World);

    // Apply the impulse to the body to the body 2
    v2 += constraintSolverData.rigidBodyComponents.getMassInverse(body2Entity) * mImpulse;
    w2 += mI2 * angularImpulseBody2;
}

// Solve the velocity constraint
void BallAndSocketJoint::solveVelocityConstraint(const ConstraintSolverData& constraintSolverData) {

    Entity body1Entity = mWorld.mJointsComponents.getBody1Entity(mEntity);
    Entity body2Entity = mWorld.mJointsComponents.getBody2Entity(mEntity);

    uint32 dynamicsComponentIndexBody1 = constraintSolverData.rigidBodyComponents.getEntityIndex(body1Entity);
    uint32 dynamicsComponentIndexBody2 = constraintSolverData.rigidBodyComponents.getEntityIndex(body2Entity);

    // Get the velocities
    Vector3& v1 = constraintSolverData.rigidBodyComponents.mConstrainedLinearVelocities[dynamicsComponentIndexBody1];
    Vector3& v2 = constraintSolverData.rigidBodyComponents.mConstrainedLinearVelocities[dynamicsComponentIndexBody2];
    Vector3& w1 = constraintSolverData.rigidBodyComponents.mConstrainedAngularVelocities[dynamicsComponentIndexBody1];
    Vector3& w2 = constraintSolverData.rigidBodyComponents.mConstrainedAngularVelocities[dynamicsComponentIndexBody2];

    // Compute J*v
    const Vector3 Jv = v2 + w2.cross(mR2World) - v1 - w1.cross(mR1World);

    // Compute the Lagrange multiplier lambda
    const Vector3 deltaLambda = mInverseMassMatrix * (-Jv - mBiasVector);
    mImpulse += deltaLambda;

    // Compute the impulse P=J^T * lambda for the body 1
    const Vector3 linearImpulseBody1 = -deltaLambda;
    const Vector3 angularImpulseBody1 = deltaLambda.cross(mR1World);

    // Apply the impulse to the body 1
    v1 += constraintSolverData.rigidBodyComponents.getMassInverse(body1Entity) * linearImpulseBody1;
    w1 += mI1 * angularImpulseBody1;

    // Compute the impulse P=J^T * lambda for the body 2
    const Vector3 angularImpulseBody2 = -deltaLambda.cross(mR2World);

    // Apply the impulse to the body 2
    v2 += constraintSolverData.rigidBodyComponents.getMassInverse(body2Entity) * deltaLambda;
    w2 += mI2 * angularImpulseBody2;
}

// Solve the position constraint (for position error correction)
void BallAndSocketJoint::solvePositionConstraint(const ConstraintSolverData& constraintSolverData) {

    Entity body1Entity = mWorld.mJointsComponents.getBody1Entity(mEntity);
    Entity body2Entity = mWorld.mJointsComponents.getBody2Entity(mEntity);

    // TODO : Remove this and use compoents instead of pointers to bodies
    RigidBody* body1 = static_cast<RigidBody*>(mWorld.mRigidBodyComponents.getRigidBody(body1Entity));
    RigidBody* body2 = static_cast<RigidBody*>(mWorld.mRigidBodyComponents.getRigidBody(body2Entity));

    // If the error position correction technique is not the non-linear-gauss-seidel, we do
    // do not execute this method
    if (mWorld.mJointsComponents.getPositionCorrectionTechnique(mEntity) != JointsPositionCorrectionTechnique::NON_LINEAR_GAUSS_SEIDEL) return;

    // Get the bodies center of mass and orientations
    Vector3 x1 = constraintSolverData.rigidBodyComponents.getConstrainedPosition(body1Entity);
    Vector3 x2 = constraintSolverData.rigidBodyComponents.getConstrainedPosition(body2Entity);
    Quaternion q1 = constraintSolverData.rigidBodyComponents.getConstrainedOrientation(body1Entity);
    Quaternion q2 = constraintSolverData.rigidBodyComponents.getConstrainedOrientation(body2Entity);

    // Get the inverse mass and inverse inertia tensors of the bodies
    const decimal inverseMassBody1 = constraintSolverData.rigidBodyComponents.getMassInverse(body1Entity);
    const decimal inverseMassBody2 = constraintSolverData.rigidBodyComponents.getMassInverse(body2Entity);

    // Recompute the inverse inertia tensors
    mI1 = body1->getInertiaTensorInverseWorld();
    mI2 = body2->getInertiaTensorInverseWorld();

    // Compute the vector from body center to the anchor point in world-space
    mR1World = q1 * mLocalAnchorPointBody1;
    mR2World = q2 * mLocalAnchorPointBody2;

    // Compute the corresponding skew-symmetric matrices
    Matrix3x3 skewSymmetricMatrixU1= Matrix3x3::computeSkewSymmetricMatrixForCrossProduct(mR1World);
    Matrix3x3 skewSymmetricMatrixU2= Matrix3x3::computeSkewSymmetricMatrixForCrossProduct(mR2World);

    // Recompute the inverse mass matrix K=J^TM^-1J of of the 3 translation constraints
    decimal inverseMassBodies = inverseMassBody1 + inverseMassBody2;
    Matrix3x3 massMatrix = Matrix3x3(inverseMassBodies, 0, 0,
                                    0, inverseMassBodies, 0,
                                    0, 0, inverseMassBodies) +
                           skewSymmetricMatrixU1 * mI1 * skewSymmetricMatrixU1.getTranspose() +
                           skewSymmetricMatrixU2 * mI2 * skewSymmetricMatrixU2.getTranspose();
    mInverseMassMatrix.setToZero();
    if (mWorld.mRigidBodyComponents.getBodyType(body1Entity) == BodyType::DYNAMIC ||
        mWorld.mRigidBodyComponents.getBodyType(body2Entity) == BodyType::DYNAMIC) {
        mInverseMassMatrix = massMatrix.getInverse();
    }

    // Compute the constraint error (value of the C(x) function)
    const Vector3 constraintError = (x2 + mR2World - x1 - mR1World);

    // Compute the Lagrange multiplier lambda
    // TODO : Do not solve the system by computing the inverse each time and multiplying with the
    //        right-hand side vector but instead use a method to directly solve the linear system.
    const Vector3 lambda = mInverseMassMatrix * (-constraintError);

    // Compute the impulse of body 1
    const Vector3 linearImpulseBody1 = -lambda;
    const Vector3 angularImpulseBody1 = lambda.cross(mR1World);

    // Compute the pseudo velocity of body 1
    const Vector3 v1 = inverseMassBody1 * linearImpulseBody1;
    const Vector3 w1 = mI1 * angularImpulseBody1;

    // Update the body center of mass and orientation of body 1
    x1 += v1;
    q1 += Quaternion(0, w1) * q1 * decimal(0.5);
    q1.normalize();

    // Compute the impulse of body 2
    const Vector3 angularImpulseBody2 = -lambda.cross(mR2World);

    // Compute the pseudo velocity of body 2
    const Vector3 v2 = inverseMassBody2 * lambda;
    const Vector3 w2 = mI2 * angularImpulseBody2;

    // Update the body position/orientation of body 2
    x2 += v2;
    q2 += Quaternion(0, w2) * q2 * decimal(0.5);
    q2.normalize();

    constraintSolverData.rigidBodyComponents.setConstrainedPosition(body1Entity, x1);
    constraintSolverData.rigidBodyComponents.setConstrainedPosition(body2Entity, x2);
    constraintSolverData.rigidBodyComponents.setConstrainedOrientation(body1Entity, q1);
    constraintSolverData.rigidBodyComponents.setConstrainedOrientation(body2Entity, q2);
}

