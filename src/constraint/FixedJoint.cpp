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
#include "FixedJoint.h"
#include "systems/ConstraintSolverSystem.h"
#include "components/RigidBodyComponents.h"
#include "engine/DynamicsWorld.h"

using namespace reactphysics3d;

// Static variables definition
const decimal FixedJoint::BETA = decimal(0.2);

// Constructor
FixedJoint::FixedJoint(Entity entity, DynamicsWorld &world, const FixedJointInfo& jointInfo)
           : Joint(entity, world) {

    // Compute the local-space anchor point for each body
    const Transform& transform1 = mWorld.mTransformComponents.getTransform(jointInfo.body1->getEntity());
    const Transform& transform2 = mWorld.mTransformComponents.getTransform(jointInfo.body2->getEntity());

    mWorld.mFixedJointsComponents.setLocalAnchoirPointBody1(mEntity, transform1.getInverse() * jointInfo.anchorPointWorldSpace);
    mWorld.mFixedJointsComponents.setLocalAnchoirPointBody2(mEntity, transform2.getInverse() * jointInfo.anchorPointWorldSpace);

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
    mWorld.mFixedJointsComponents.setInitOrientationDifferenceInv(mEntity, transform2.getOrientation().getInverse() * transform1.getOrientation());
}

// Initialize before solving the constraint
void FixedJoint::initBeforeSolve(const ConstraintSolverData& constraintSolverData) {

    // Get the bodies entities
    Entity body1Entity = mWorld.mJointsComponents.getBody1Entity(mEntity);
    Entity body2Entity = mWorld.mJointsComponents.getBody2Entity(mEntity);

    // TODO : Remove this and use compoents instead of pointers to bodies
    RigidBody* body1 = static_cast<RigidBody*>(mWorld.mRigidBodyComponents.getRigidBody(body1Entity));
    RigidBody* body2 = static_cast<RigidBody*>(mWorld.mRigidBodyComponents.getRigidBody(body2Entity));

    // Get the bodies positions and orientations
    const Vector3& x1 = constraintSolverData.rigidBodyComponents.getCenterOfMassWorld(body1Entity);
    const Vector3& x2 = constraintSolverData.rigidBodyComponents.getCenterOfMassWorld(body2Entity);
    const Quaternion& orientationBody1 = body1->getTransform().getOrientation();
    const Quaternion& orientationBody2 = body2->getTransform().getOrientation();

    // Get the inertia tensor of bodies
    mWorld.mFixedJointsComponents.setI1(mEntity, body1->getInertiaTensorInverseWorld());
    mWorld.mFixedJointsComponents.setI1(mEntity, body2->getInertiaTensorInverseWorld());

    const Vector3& r1World = mWorld.mFixedJointsComponents.getR1World(mEntity);
    const Vector3& r2World = mWorld.mFixedJointsComponents.getR2World(mEntity);

    const Matrix3x3& i1 = mWorld.mFixedJointsComponents.getI1(mEntity);
    const Matrix3x3& i2 = mWorld.mFixedJointsComponents.getI2(mEntity);

    // Compute the vector from body center to the anchor point in world-space
    mWorld.mFixedJointsComponents.setR1World(mEntity, orientationBody1 * mWorld.mFixedJointsComponents.getLocalAnchoirPointBody1(mEntity));
    mWorld.mFixedJointsComponents.setR2World(mEntity, orientationBody2 * mWorld.mFixedJointsComponents.getLocalAnchoirPointBody2(mEntity));

    // Compute the corresponding skew-symmetric matrices
    Matrix3x3 skewSymmetricMatrixU1= Matrix3x3::computeSkewSymmetricMatrixForCrossProduct(r1World);
    Matrix3x3 skewSymmetricMatrixU2= Matrix3x3::computeSkewSymmetricMatrixForCrossProduct(r2World);

    // Compute the matrix K=JM^-1J^t (3x3 matrix) for the 3 translation constraints
    const decimal body1MassInverse = constraintSolverData.rigidBodyComponents.getMassInverse(body1->getEntity());
    const decimal body2MassInverse = constraintSolverData.rigidBodyComponents.getMassInverse(body2->getEntity());
    const decimal inverseMassBodies = body1MassInverse + body2MassInverse;
    Matrix3x3 massMatrix = Matrix3x3(inverseMassBodies, 0, 0,
                                    0, inverseMassBodies, 0,
                                    0, 0, inverseMassBodies) +
                           skewSymmetricMatrixU1 * i1 * skewSymmetricMatrixU1.getTranspose() +
                           skewSymmetricMatrixU2 * i2 * skewSymmetricMatrixU2.getTranspose();

    // Compute the inverse mass matrix K^-1 for the 3 translation constraints
    Matrix3x3& inverseMassMatrixTranslation = mWorld.mFixedJointsComponents.getInverseMassMatrixTranslation(mEntity);
    inverseMassMatrixTranslation.setToZero();
    if (mWorld.mRigidBodyComponents.getBodyType(body1Entity) == BodyType::DYNAMIC ||
        mWorld.mRigidBodyComponents.getBodyType(body2Entity) == BodyType::DYNAMIC) {
        mWorld.mFixedJointsComponents.setInverseMassMatrixTranslation(mEntity, massMatrix.getInverse());
    }

    // Compute the bias "b" of the constraint for the 3 translation constraints
    const decimal biasFactor = (BETA / constraintSolverData.timeStep);
    Vector3& biasTranslation = mWorld.mFixedJointsComponents.getBiasTranslation(mEntity);
    biasTranslation.setToZero();
    if (mWorld.mJointsComponents.getPositionCorrectionTechnique(mEntity) == JointsPositionCorrectionTechnique::BAUMGARTE_JOINTS) {
        mWorld.mFixedJointsComponents.setBiasTranslation(mEntity, biasFactor * (x2 + r2World - x1 - r1World));
    }

    // Compute the inverse of the mass matrix K=JM^-1J^t for the 3 rotation
    // contraints (3x3 matrix)
    Matrix3x3& inverseMassMatrixRotation = mWorld.mFixedJointsComponents.getInverseMassMatrixRotation(mEntity);
    inverseMassMatrixRotation = i1 + i2;
    if (mWorld.mRigidBodyComponents.getBodyType(body1Entity) == BodyType::DYNAMIC ||
        mWorld.mRigidBodyComponents.getBodyType(body2Entity) == BodyType::DYNAMIC) {
        mWorld.mFixedJointsComponents.setInverseMassMatrixRotation(mEntity, mWorld.mFixedJointsComponents.getInverseMassMatrixRotation(mEntity).getInverse());
    }

    // Compute the bias "b" for the 3 rotation constraints
    Vector3& biasRotation = mWorld.mFixedJointsComponents.getBiasRotation(mEntity);
    biasRotation.setToZero();

    if (mWorld.mJointsComponents.getPositionCorrectionTechnique(mEntity) == JointsPositionCorrectionTechnique::BAUMGARTE_JOINTS) {
        const Quaternion qError = orientationBody2 * mWorld.mFixedJointsComponents.getInitOrientationDifferenceInv(mEntity) * orientationBody1.getInverse();
        mWorld.mFixedJointsComponents.setBiasRotation(mEntity, biasFactor * decimal(2.0) * qError.getVectorV());
    }

    // If warm-starting is not enabled
    if (!constraintSolverData.isWarmStartingActive) {

        Vector3& impulseTranslation = mWorld.mFixedJointsComponents.getImpulseTranslation(mEntity);
        Vector3& impulseRotation = mWorld.mFixedJointsComponents.getImpulseRotation(mEntity);

        // Reset the accumulated impulses
        impulseTranslation.setToZero();
        impulseRotation.setToZero();
    }
}

// Warm start the constraint (apply the previous impulse at the beginning of the step)
void FixedJoint::warmstart(const ConstraintSolverData& constraintSolverData) {

    // Get the bodies entities
    Entity body1Entity = mWorld.mJointsComponents.getBody1Entity(mEntity);
    Entity body2Entity = mWorld.mJointsComponents.getBody2Entity(mEntity);

    uint32 dynamicsComponentIndexBody1 = constraintSolverData.rigidBodyComponents.getEntityIndex(body1Entity);
    uint32 dynamicsComponentIndexBody2 = constraintSolverData.rigidBodyComponents.getEntityIndex(body2Entity);

    // Get the velocities
    Vector3& v1 = constraintSolverData.rigidBodyComponents.mConstrainedLinearVelocities[dynamicsComponentIndexBody1];
    Vector3& v2 = constraintSolverData.rigidBodyComponents.mConstrainedLinearVelocities[dynamicsComponentIndexBody2];
    Vector3& w1 = constraintSolverData.rigidBodyComponents.mConstrainedAngularVelocities[dynamicsComponentIndexBody1];
    Vector3& w2 = constraintSolverData.rigidBodyComponents.mConstrainedAngularVelocities[dynamicsComponentIndexBody2];

    // Get the inverse mass of the bodies
    const decimal inverseMassBody1 = constraintSolverData.rigidBodyComponents.getMassInverse(body1Entity);
    const decimal inverseMassBody2 = constraintSolverData.rigidBodyComponents.getMassInverse(body2Entity);

    const Vector3& impulseTranslation = mWorld.mFixedJointsComponents.getImpulseTranslation(mEntity);
    const Vector3& impulseRotation = mWorld.mFixedJointsComponents.getImpulseRotation(mEntity);

    const Vector3& r1World = mWorld.mFixedJointsComponents.getR1World(mEntity);
    const Vector3& r2World = mWorld.mFixedJointsComponents.getR2World(mEntity);

    const Matrix3x3& i1 = mWorld.mFixedJointsComponents.getI1(mEntity);
    const Matrix3x3& i2 = mWorld.mFixedJointsComponents.getI2(mEntity);

    // Compute the impulse P=J^T * lambda for the 3 translation constraints for body 1
    Vector3 linearImpulseBody1 = -impulseTranslation;
    Vector3 angularImpulseBody1 = impulseTranslation.cross(r1World);

    // Compute the impulse P=J^T * lambda for the 3 rotation constraints for body 1
    angularImpulseBody1 += -impulseRotation;

    // Apply the impulse to the body 1
    v1 += inverseMassBody1 * linearImpulseBody1;
    w1 += i1 * angularImpulseBody1;

    // Compute the impulse P=J^T * lambda for the 3 translation constraints for body 2
    Vector3 angularImpulseBody2 = -impulseTranslation.cross(r2World);

    // Compute the impulse P=J^T * lambda for the 3 rotation constraints for body 2
    angularImpulseBody2 += impulseRotation;

    // Apply the impulse to the body 2
    v2 += inverseMassBody2 * impulseTranslation;
    w2 += i2 * angularImpulseBody2;
}

// Solve the velocity constraint
void FixedJoint::solveVelocityConstraint(const ConstraintSolverData& constraintSolverData) {

    // Get the bodies entities
    Entity body1Entity = mWorld.mJointsComponents.getBody1Entity(mEntity);
    Entity body2Entity = mWorld.mJointsComponents.getBody2Entity(mEntity);

    uint32 dynamicsComponentIndexBody1 = constraintSolverData.rigidBodyComponents.getEntityIndex(body1Entity);
    uint32 dynamicsComponentIndexBody2 = constraintSolverData.rigidBodyComponents.getEntityIndex(body2Entity);

    // Get the velocities
    Vector3& v1 = constraintSolverData.rigidBodyComponents.mConstrainedLinearVelocities[dynamicsComponentIndexBody1];
    Vector3& v2 = constraintSolverData.rigidBodyComponents.mConstrainedLinearVelocities[dynamicsComponentIndexBody2];
    Vector3& w1 = constraintSolverData.rigidBodyComponents.mConstrainedAngularVelocities[dynamicsComponentIndexBody1];
    Vector3& w2 = constraintSolverData.rigidBodyComponents.mConstrainedAngularVelocities[dynamicsComponentIndexBody2];

    // Get the inverse mass of the bodies
    decimal inverseMassBody1 = constraintSolverData.rigidBodyComponents.getMassInverse(body1Entity);
    decimal inverseMassBody2 = constraintSolverData.rigidBodyComponents.getMassInverse(body2Entity);

    const Vector3& r1World = mWorld.mFixedJointsComponents.getR1World(mEntity);
    const Vector3& r2World = mWorld.mFixedJointsComponents.getR2World(mEntity);

    const Matrix3x3& i1 = mWorld.mFixedJointsComponents.getI1(mEntity);
    const Matrix3x3& i2 = mWorld.mFixedJointsComponents.getI2(mEntity);

    // --------------- Translation Constraints --------------- //

    // Compute J*v for the 3 translation constraints
    const Vector3 JvTranslation = v2 + w2.cross(r2World) - v1 - w1.cross(r1World);

    const Vector3& biasTranslation = mWorld.mFixedJointsComponents.getBiasTranslation(mEntity);
    const Matrix3x3& inverseMassMatrixTranslation = mWorld.mFixedJointsComponents.getInverseMassMatrixTranslation(mEntity);

    // Compute the Lagrange multiplier lambda
    const Vector3 deltaLambda = inverseMassMatrixTranslation * (-JvTranslation - biasTranslation);
    mWorld.mFixedJointsComponents.setImpulseTranslation(mEntity, mWorld.mFixedJointsComponents.getImpulseTranslation(mEntity) + deltaLambda);

    // Compute the impulse P=J^T * lambda for body 1
    const Vector3 linearImpulseBody1 = -deltaLambda;
    Vector3 angularImpulseBody1 = deltaLambda.cross(r1World);

    // Apply the impulse to the body 1
    v1 += inverseMassBody1 * linearImpulseBody1;
    w1 += i1 * angularImpulseBody1;

    // Compute the impulse P=J^T * lambda  for body 2
    const Vector3 angularImpulseBody2 = -deltaLambda.cross(r2World);

    // Apply the impulse to the body 2
    v2 += inverseMassBody2 * deltaLambda;
    w2 += i2 * angularImpulseBody2;

    // --------------- Rotation Constraints --------------- //

    // Compute J*v for the 3 rotation constraints
    const Vector3 JvRotation = w2 - w1;

    const Vector3& biasRotation = mWorld.mFixedJointsComponents.getBiasRotation(mEntity);
    const Matrix3x3& inverseMassMatrixRotation = mWorld.mFixedJointsComponents.getInverseMassMatrixRotation(mEntity);

    // Compute the Lagrange multiplier lambda for the 3 rotation constraints
    Vector3 deltaLambda2 = inverseMassMatrixRotation * (-JvRotation - biasRotation);
    mWorld.mFixedJointsComponents.setImpulseRotation(mEntity, mWorld.mFixedJointsComponents.getImpulseRotation(mEntity) + deltaLambda2);

    // Compute the impulse P=J^T * lambda for the 3 rotation constraints for body 1
    angularImpulseBody1 = -deltaLambda2;

    // Apply the impulse to the body 1
    w1 += i1 * angularImpulseBody1;

    // Apply the impulse to the body 2
    w2 += i2 * deltaLambda2;
}

// Solve the position constraint (for position error correction)
void FixedJoint::solvePositionConstraint(const ConstraintSolverData& constraintSolverData) {

    // Get the bodies entities
    Entity body1Entity = mWorld.mJointsComponents.getBody1Entity(mEntity);
    Entity body2Entity = mWorld.mJointsComponents.getBody2Entity(mEntity);

    // TODO : Remove this and use compoents instead of pointers to bodies
    RigidBody* body1 = static_cast<RigidBody*>(mWorld.mRigidBodyComponents.getRigidBody(body1Entity));
    RigidBody* body2 = static_cast<RigidBody*>(mWorld.mRigidBodyComponents.getRigidBody(body2Entity));

    // If the error position correction technique is not the non-linear-gauss-seidel, we do
    // do not execute this method
    if (mWorld.mJointsComponents.getPositionCorrectionTechnique(mEntity) != JointsPositionCorrectionTechnique::NON_LINEAR_GAUSS_SEIDEL) return;

    // Get the bodies positions and orientations
    Vector3 x1 = constraintSolverData.rigidBodyComponents.getConstrainedPosition(body1Entity);
    Vector3 x2 = constraintSolverData.rigidBodyComponents.getConstrainedPosition(body2Entity);
    Quaternion q1 = constraintSolverData.rigidBodyComponents.getConstrainedOrientation(body1Entity);
    Quaternion q2 = constraintSolverData.rigidBodyComponents.getConstrainedOrientation(body2Entity);

    // Get the inverse mass and inverse inertia tensors of the bodies
    decimal inverseMassBody1 = constraintSolverData.rigidBodyComponents.getMassInverse(body1Entity);
    decimal inverseMassBody2 = constraintSolverData.rigidBodyComponents.getMassInverse(body2Entity);

    const Vector3& r1World = mWorld.mFixedJointsComponents.getR1World(mEntity);
    const Vector3& r2World = mWorld.mFixedJointsComponents.getR2World(mEntity);

    const Matrix3x3& i1 = mWorld.mFixedJointsComponents.getI1(mEntity);
    const Matrix3x3& i2 = mWorld.mFixedJointsComponents.getI2(mEntity);

    // Recompute the inverse inertia tensors
    mWorld.mFixedJointsComponents.setI1(mEntity, body1->getInertiaTensorInverseWorld());
    mWorld.mFixedJointsComponents.setI2(mEntity, body2->getInertiaTensorInverseWorld());

    // Compute the vector from body center to the anchor point in world-space
    mWorld.mFixedJointsComponents.setR1World(mEntity, q1 * mWorld.mFixedJointsComponents.getLocalAnchoirPointBody1(mEntity));
    mWorld.mFixedJointsComponents.setR2World(mEntity, q2 * mWorld.mFixedJointsComponents.getLocalAnchoirPointBody2(mEntity));

    // Compute the corresponding skew-symmetric matrices
    Matrix3x3 skewSymmetricMatrixU1= Matrix3x3::computeSkewSymmetricMatrixForCrossProduct(r1World);
    Matrix3x3 skewSymmetricMatrixU2= Matrix3x3::computeSkewSymmetricMatrixForCrossProduct(r2World);

    // --------------- Translation Constraints --------------- //

    // Compute the matrix K=JM^-1J^t (3x3 matrix) for the 3 translation constraints
    decimal inverseMassBodies = inverseMassBody1 + inverseMassBody2;
    Matrix3x3 massMatrix = Matrix3x3(inverseMassBodies, 0, 0,
                                    0, inverseMassBodies, 0,
                                    0, 0, inverseMassBodies) +
                           skewSymmetricMatrixU1 * i1 * skewSymmetricMatrixU1.getTranspose() +
                           skewSymmetricMatrixU2 * i2 * skewSymmetricMatrixU2.getTranspose();
    Matrix3x3& inverseMassMatrixTranslation = mWorld.mFixedJointsComponents.getInverseMassMatrixTranslation(mEntity);
    inverseMassMatrixTranslation.setToZero();
    if (mWorld.mRigidBodyComponents.getBodyType(body1Entity) == BodyType::DYNAMIC ||
        mWorld.mRigidBodyComponents.getBodyType(body2Entity) == BodyType::DYNAMIC) {
        mWorld.mFixedJointsComponents.setInverseMassMatrixTranslation(mEntity, massMatrix.getInverse());
    }

    // Compute position error for the 3 translation constraints
    const Vector3 errorTranslation = x2 + r2World - x1 - r1World;

    // Compute the Lagrange multiplier lambda
    const Vector3 lambdaTranslation = inverseMassMatrixTranslation * (-errorTranslation);

    // Compute the impulse of body 1
    Vector3 linearImpulseBody1 = -lambdaTranslation;
    Vector3 angularImpulseBody1 = lambdaTranslation.cross(r1World);

    // Compute the pseudo velocity of body 1
    const Vector3 v1 = inverseMassBody1 * linearImpulseBody1;
    Vector3 w1 = i1 * angularImpulseBody1;

    // Update the body position/orientation of body 1
    x1 += v1;
    q1 += Quaternion(0, w1) * q1 * decimal(0.5);
    q1.normalize();

    // Compute the impulse of body 2
    Vector3 angularImpulseBody2 = -lambdaTranslation.cross(r2World);

    // Compute the pseudo velocity of body 2
    const Vector3 v2 = inverseMassBody2 * lambdaTranslation;
    Vector3 w2 = i2 * angularImpulseBody2;

    // Update the body position/orientation of body 2
    x2 += v2;
    q2 += Quaternion(0, w2) * q2 * decimal(0.5);
    q2.normalize();

    // --------------- Rotation Constraints --------------- //

    // Compute the inverse of the mass matrix K=JM^-1J^t for the 3 rotation
    // contraints (3x3 matrix)
    Matrix3x3& inverseMassMatrixRotation = mWorld.mFixedJointsComponents.getInverseMassMatrixRotation(mEntity);
    inverseMassMatrixRotation = i1 + i2;
    if (mWorld.mRigidBodyComponents.getBodyType(body1Entity) == BodyType::DYNAMIC ||
        mWorld.mRigidBodyComponents.getBodyType(body2Entity) == BodyType::DYNAMIC) {
        mWorld.mFixedJointsComponents.setInverseMassMatrixRotation(mEntity, inverseMassMatrixRotation.getInverse());
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
    Quaternion qError = q2 * mWorld.mFixedJointsComponents.getInitOrientationDifferenceInv(mEntity) * q1.getInverse();

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
    Vector3 lambdaRotation = inverseMassMatrixRotation * (-errorRotation);

    // Compute the impulse P=J^T * lambda for the 3 rotation constraints of body 1
    angularImpulseBody1 = -lambdaRotation;

    // Compute the pseudo velocity of body 1
    w1 = i1 * angularImpulseBody1;

    // Update the body position/orientation of body 1
    q1 += Quaternion(0, w1) * q1 * decimal(0.5);
    q1.normalize();

    // Compute the pseudo velocity of body 2
    w2 = i2 * lambdaRotation;

    // Update the body position/orientation of body 2
    q2 += Quaternion(0, w2) * q2 * decimal(0.5);
    q2.normalize();

    constraintSolverData.rigidBodyComponents.setConstrainedPosition(body1Entity, x1);
    constraintSolverData.rigidBodyComponents.setConstrainedPosition(body2Entity, x2);
    constraintSolverData.rigidBodyComponents.setConstrainedOrientation(body1Entity, q1);
    constraintSolverData.rigidBodyComponents.setConstrainedOrientation(body2Entity, q2);
}

// Return a string representation
std::string FixedJoint::to_string() const {
    return "FixedJoint{ localAnchorPointBody1=" + mWorld.mFixedJointsComponents.getLocalAnchoirPointBody1(mEntity).to_string() +
                        ", localAnchorPointBody2=" + mWorld.mFixedJointsComponents.getLocalAnchoirPointBody2(mEntity).to_string() +
                        ", initOrientationDifferenceInv=" + mWorld.mFixedJointsComponents.getInitOrientationDifferenceInv(mEntity).to_string() +
                        "}";
}

