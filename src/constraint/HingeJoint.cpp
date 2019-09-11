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
#include "systems/ConstraintSolverSystem.h"
#include "components/RigidBodyComponents.h"
#include "engine/DynamicsWorld.h"

using namespace reactphysics3d;

// Static variables definition
const decimal HingeJoint::BETA = decimal(0.2);

// Constructor
HingeJoint::HingeJoint(Entity entity, DynamicsWorld &world, const HingeJointInfo& jointInfo) : Joint(entity, world, jointInfo) {

    const decimal lowerLimit = mWorld.mHingeJointsComponents.getLowerLimit(mEntity);
    const decimal upperLimit = mWorld.mHingeJointsComponents.getUpperLimit(mEntity);
    assert(lowerLimit <= decimal(0) && lowerLimit >= decimal(-2.0) * PI);
    assert(upperLimit >= decimal(0) && upperLimit <= decimal(2.0) * PI);

    // Compute the local-space anchor point for each body
    Transform& transform1 = mWorld.mTransformComponents.getTransform(jointInfo.body1->getEntity());
    Transform& transform2 = mWorld.mTransformComponents.getTransform(jointInfo.body2->getEntity());
    mWorld.mHingeJointsComponents.setLocalAnchorPointBody1(mEntity, transform1.getInverse() * jointInfo.anchorPointWorldSpace);
    mWorld.mHingeJointsComponents.setLocalAnchorPointBody2(mEntity, transform2.getInverse() * jointInfo.anchorPointWorldSpace);

    // Compute the local-space hinge axis
    Vector3 hingeLocalAxisBody1 = transform1.getOrientation().getInverse() * jointInfo.rotationAxisWorld;
    Vector3 hingeLocalAxisBody2 = transform2.getOrientation().getInverse() * jointInfo.rotationAxisWorld;
    hingeLocalAxisBody1.normalize();
    hingeLocalAxisBody2.normalize();
    mWorld.mHingeJointsComponents.setHingeLocalAxisBody1(mEntity, hingeLocalAxisBody1);
    mWorld.mHingeJointsComponents.setHingeLocalAxisBody2(mEntity, hingeLocalAxisBody2);

    // Compute the inverse of the initial orientation difference between the two bodies
    Quaternion initOrientationDifferenceInv = transform2.getOrientation() *
                                    transform1.getOrientation().getInverse();
    initOrientationDifferenceInv.normalize();
    initOrientationDifferenceInv.inverse();
    mWorld.mHingeJointsComponents.setInitOrientationDifferenceInv(mEntity, initOrientationDifferenceInv);
}

// Initialize before solving the constraint
void HingeJoint::initBeforeSolve(const ConstraintSolverData& constraintSolverData) {

    // Get the bodies entities
    Entity body1Entity = mWorld.mJointsComponents.getBody1Entity(mEntity);
    Entity body2Entity = mWorld.mJointsComponents.getBody2Entity(mEntity);

    // TODO : Remove this and use compoents instead of pointers to bodies
    RigidBody* body1 = static_cast<RigidBody*>(mWorld.mRigidBodyComponents.getRigidBody(body1Entity));
    RigidBody* body2 = static_cast<RigidBody*>(mWorld.mRigidBodyComponents.getRigidBody(body2Entity));

    // Get the bodies positions and orientations
    const Vector3& x1 = constraintSolverData.rigidBodyComponents.getCenterOfMassWorld(body1Entity);
    const Vector3& x2 = constraintSolverData.rigidBodyComponents.getCenterOfMassWorld(body2Entity);
    const Quaternion& orientationBody1 = mWorld.mTransformComponents.getTransform(body1Entity).getOrientation();
    const Quaternion& orientationBody2 = mWorld.mTransformComponents.getTransform(body2Entity).getOrientation();

    // Get the inertia tensor of bodies
    mWorld.mHingeJointsComponents.setI1(mEntity, body1->getInertiaTensorInverseWorld());
    mWorld.mHingeJointsComponents.setI2(mEntity, body2->getInertiaTensorInverseWorld());

    // Compute the vector from body center to the anchor point in world-space
    mWorld.mHingeJointsComponents.setR1World(mEntity, orientationBody1 * mWorld.mHingeJointsComponents.getLocalAnchorPointBody1(mEntity));
    mWorld.mHingeJointsComponents.setR2World(mEntity, orientationBody2 * mWorld.mHingeJointsComponents.getLocalAnchorPointBody2(mEntity));

    // Compute the current angle around the hinge axis
    decimal hingeAngle = computeCurrentHingeAngle(orientationBody1, orientationBody2);

    // Check if the limit constraints are violated or not
    decimal lowerLimitError = hingeAngle - mWorld.mHingeJointsComponents.getLowerLimit(mEntity);
    decimal upperLimitError = mWorld.mHingeJointsComponents.getUpperLimit(mEntity) - hingeAngle;
    bool oldIsLowerLimitViolated = mWorld.mHingeJointsComponents.getIsLowerLimitViolated(mEntity);
    bool isLowerLimitViolated = lowerLimitError <= 0;
    mWorld.mHingeJointsComponents.setIsLowerLimitViolated(mEntity, isLowerLimitViolated);
    if (isLowerLimitViolated != oldIsLowerLimitViolated) {
        mWorld.mHingeJointsComponents.setImpulseLowerLimit(mEntity, decimal(0.0));
    }
    bool oldIsUpperLimitViolated = mWorld.mHingeJointsComponents.getIsUpperLimitViolated(mEntity);
    bool isUpperLimitViolated = upperLimitError <= 0;
    mWorld.mHingeJointsComponents.setIsUpperLimitViolated(mEntity, isUpperLimitViolated);
    if (isUpperLimitViolated != oldIsUpperLimitViolated) {
        mWorld.mHingeJointsComponents.setImpulseUpperLimit(mEntity, decimal(0.0));
    }

    // Compute vectors needed in the Jacobian
    Vector3 a1 = orientationBody1 * mWorld.mHingeJointsComponents.getHingeLocalAxisBody1(mEntity);
    Vector3 a2 = orientationBody2 * mWorld.mHingeJointsComponents.getHingeLocalAxisBody2(mEntity);
    a1.normalize();
    a2.normalize();
    mWorld.mHingeJointsComponents.setA1(mEntity, a1);
    const Vector3 b2 = a2.getOneUnitOrthogonalVector();
    const Vector3 c2 = a2.cross(b2);
    mWorld.mHingeJointsComponents.setB2CrossA1(mEntity, b2.cross(a1));
    mWorld.mHingeJointsComponents.setC2CrossA1(mEntity, c2.cross(a1));

    // Compute the corresponding skew-symmetric matrices
    Matrix3x3 skewSymmetricMatrixU1= Matrix3x3::computeSkewSymmetricMatrixForCrossProduct(mWorld.mHingeJointsComponents.getR1World(mEntity));
    Matrix3x3 skewSymmetricMatrixU2= Matrix3x3::computeSkewSymmetricMatrixForCrossProduct(mWorld.mHingeJointsComponents.getR2World(mEntity));

    // Compute the inverse mass matrix K=JM^-1J^t for the 3 translation constraints (3x3 matrix)
    decimal body1MassInverse = constraintSolverData.rigidBodyComponents.getMassInverse(body1->getEntity());
    decimal body2MassInverse = constraintSolverData.rigidBodyComponents.getMassInverse(body2->getEntity());
    decimal inverseMassBodies = body1MassInverse + body2MassInverse;
    Matrix3x3 massMatrix = Matrix3x3(inverseMassBodies, 0, 0,
                                    0, inverseMassBodies, 0,
                                    0, 0, inverseMassBodies) +
                           skewSymmetricMatrixU1 * mWorld.mHingeJointsComponents.getI1(mEntity) * skewSymmetricMatrixU1.getTranspose() +
                           skewSymmetricMatrixU2 * mWorld.mHingeJointsComponents.getI2(mEntity) * skewSymmetricMatrixU2.getTranspose();
    Matrix3x3& inverseMassMatrixTranslation = mWorld.mHingeJointsComponents.getInverseMassMatrixTranslation(mEntity);
    inverseMassMatrixTranslation.setToZero();
    if (mWorld.mRigidBodyComponents.getBodyType(body1Entity) == BodyType::DYNAMIC ||
        mWorld.mRigidBodyComponents.getBodyType(body2Entity) == BodyType::DYNAMIC) {
        mWorld.mHingeJointsComponents.setInverseMassMatrixTranslation(mEntity, massMatrix.getInverse());
    }

    // Compute the bias "b" of the translation constraints
    Vector3& bTranslation = mWorld.mHingeJointsComponents.getBiasTranslation(mEntity);
    bTranslation.setToZero();
    decimal biasFactor = (BETA / constraintSolverData.timeStep);
    if (mWorld.mJointsComponents.getPositionCorrectionTechnique(mEntity) == JointsPositionCorrectionTechnique::BAUMGARTE_JOINTS) {
        bTranslation = biasFactor * (x2 + mWorld.mHingeJointsComponents.getR2World(mEntity) - x1 - mWorld.mHingeJointsComponents.getR1World(mEntity));
        mWorld.mHingeJointsComponents.setBiasTranslation(mEntity, bTranslation);
    }

    const Matrix3x3& i1 = mWorld.mHingeJointsComponents.getI1(mEntity);
    const Matrix3x3& i2 = mWorld.mHingeJointsComponents.getI2(mEntity);
    const Vector3& b2CrossA1 = mWorld.mHingeJointsComponents.getB2CrossA1(mEntity);
    const Vector3& c2CrossA1 = mWorld.mHingeJointsComponents.getC2CrossA1(mEntity);

    // Compute the inverse mass matrix K=JM^-1J^t for the 2 rotation constraints (2x2 matrix)
    Vector3 i1B2CrossA1 = i1 * b2CrossA1;
    Vector3 i1C2CrossA1 = i1 * c2CrossA1;
    Vector3 i2B2CrossA1 = i2 * b2CrossA1;
    Vector3 i2C2CrossA1 = i2 * c2CrossA1;
    const decimal el11 = b2CrossA1.dot(i1B2CrossA1) +
                         b2CrossA1.dot(i2B2CrossA1);
    const decimal el12 = b2CrossA1.dot(i1C2CrossA1) +
                         b2CrossA1.dot(i2C2CrossA1);
    const decimal el21 = c2CrossA1.dot(i1B2CrossA1) +
                         c2CrossA1.dot(i2B2CrossA1);
    const decimal el22 = c2CrossA1.dot(i1C2CrossA1) +
                         c2CrossA1.dot(i2C2CrossA1);
    const Matrix2x2 matrixKRotation(el11, el12, el21, el22);
    Matrix2x2& inverseMassMatrixRotation = mWorld.mHingeJointsComponents.getInverseMassMatrixRotation(mEntity);
    inverseMassMatrixRotation.setToZero();
    if (mWorld.mRigidBodyComponents.getBodyType(body1Entity) == BodyType::DYNAMIC ||
        mWorld.mRigidBodyComponents.getBodyType(body2Entity) == BodyType::DYNAMIC) {
        mWorld.mHingeJointsComponents.setInverseMassMatrixRotation(mEntity, matrixKRotation.getInverse());
    }

    // Compute the bias "b" of the rotation constraints
    Vector2& biasRotation = mWorld.mHingeJointsComponents.getBiasRotation(mEntity);
    biasRotation.setToZero();
    if (mWorld.mJointsComponents.getPositionCorrectionTechnique(mEntity) == JointsPositionCorrectionTechnique::BAUMGARTE_JOINTS) {
        mWorld.mHingeJointsComponents.setBiasRotation(mEntity, biasFactor * Vector2(a1.dot(b2), a1.dot(c2)));
    }

    // If warm-starting is not enabled
    if (!constraintSolverData.isWarmStartingActive) {

        // Reset all the accumulated impulses
        Vector3& impulseTranslation = mWorld.mHingeJointsComponents.getImpulseTranslation(mEntity);
        Vector2& impulseRotation = mWorld.mHingeJointsComponents.getImpulseRotation(mEntity);
        impulseTranslation.setToZero();
        impulseRotation.setToZero();
        mWorld.mHingeJointsComponents.setImpulseLowerLimit(mEntity, decimal(0.0));
        mWorld.mHingeJointsComponents.setImpulseUpperLimit(mEntity, decimal(0.0));
        mWorld.mHingeJointsComponents.setImpulseMotor(mEntity, decimal(0.0));
    }

    // If the motor or limits are enabled
    if (mWorld.mHingeJointsComponents.getIsMotorEnabled(mEntity) ||
        (mWorld.mHingeJointsComponents.getIsLimitEnabled(mEntity) && (mWorld.mHingeJointsComponents.getIsLowerLimitViolated(mEntity) ||
                                                                      mWorld.mHingeJointsComponents.getIsUpperLimitViolated(mEntity)))) {

        // Compute the inverse of the mass matrix K=JM^-1J^t for the limits and motor (1x1 matrix)
        decimal inverseMassMatrixLimitMotor = a1.dot(i1 * a1) + a1.dot(i2 * a1);
        inverseMassMatrixLimitMotor = (inverseMassMatrixLimitMotor > decimal(0.0)) ?
                                  decimal(1.0) / inverseMassMatrixLimitMotor : decimal(0.0);
        mWorld.mHingeJointsComponents.setInverseMassMatrixLimitMotor(mEntity, inverseMassMatrixLimitMotor);

        if (mWorld.mHingeJointsComponents.getIsLimitEnabled(mEntity)) {

            // Compute the bias "b" of the lower limit constraint
            mWorld.mHingeJointsComponents.setBLowerLimit(mEntity, decimal(0.0));
            if (mWorld.mJointsComponents.getPositionCorrectionTechnique(mEntity) == JointsPositionCorrectionTechnique::BAUMGARTE_JOINTS) {
                mWorld.mHingeJointsComponents.setBLowerLimit(mEntity, biasFactor * lowerLimitError);
            }

            // Compute the bias "b" of the upper limit constraint
            mWorld.mHingeJointsComponents.setBUpperLimit(mEntity, decimal(0.0));
            if (mWorld.mJointsComponents.getPositionCorrectionTechnique(mEntity) == JointsPositionCorrectionTechnique::BAUMGARTE_JOINTS) {
                mWorld.mHingeJointsComponents.setBUpperLimit(mEntity, biasFactor * upperLimitError);
            }
        }
    }
}

// Warm start the constraint (apply the previous impulse at the beginning of the step)
void HingeJoint::warmstart(const ConstraintSolverData& constraintSolverData) {

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

    // Get the inverse mass and inverse inertia tensors of the bodies
    const decimal inverseMassBody1 = constraintSolverData.rigidBodyComponents.getMassInverse(body1Entity);
    const decimal inverseMassBody2 = constraintSolverData.rigidBodyComponents.getMassInverse(body2Entity);

    const Vector3& impulseTranslation = mWorld.mHingeJointsComponents.getImpulseTranslation(mEntity);
    const Vector2& impulseRotation = mWorld.mHingeJointsComponents.getImpulseRotation(mEntity);

    const decimal impulseLowerLimit = mWorld.mHingeJointsComponents.getImpulseLowerLimit(mEntity);
    const decimal impulseUpperLimit = mWorld.mHingeJointsComponents.getImpulseUpperLimit(mEntity);

    const Vector3& b2CrossA1 = mWorld.mHingeJointsComponents.getB2CrossA1(mEntity);
    const Vector3& a1 = mWorld.mHingeJointsComponents.getA1(mEntity);

    // Compute the impulse P=J^T * lambda for the 2 rotation constraints
    Vector3 rotationImpulse = -b2CrossA1 * impulseRotation.x - mWorld.mHingeJointsComponents.getC2CrossA1(mEntity) * impulseRotation.y;

    // Compute the impulse P=J^T * lambda for the lower and upper limits constraints
    const Vector3 limitsImpulse = (impulseUpperLimit - impulseLowerLimit) * a1;

    // Compute the impulse P=J^T * lambda for the motor constraint
    const Vector3 motorImpulse = -mWorld.mHingeJointsComponents.getImpulseMotor(mEntity) * a1;

    // Compute the impulse P=J^T * lambda for the 3 translation constraints of body 1
    Vector3 linearImpulseBody1 = -impulseTranslation;
    Vector3 angularImpulseBody1 = impulseTranslation.cross(mWorld.mHingeJointsComponents.getR1World(mEntity));

    // Compute the impulse P=J^T * lambda for the 2 rotation constraints of body 1
    angularImpulseBody1 += rotationImpulse;

    // Compute the impulse P=J^T * lambda for the lower and upper limits constraints of body 1
    angularImpulseBody1 += limitsImpulse;

    // Compute the impulse P=J^T * lambda for the motor constraint of body 1
    angularImpulseBody1 += motorImpulse;

    // Apply the impulse to the body 1
    v1 += inverseMassBody1 * linearImpulseBody1;
    w1 += mWorld.mHingeJointsComponents.getI1(mEntity) * angularImpulseBody1;

    // Compute the impulse P=J^T * lambda for the 3 translation constraints of body 2
    Vector3 angularImpulseBody2 = -impulseTranslation.cross(mWorld.mHingeJointsComponents.getR2World(mEntity));

    // Compute the impulse P=J^T * lambda for the 2 rotation constraints of body 2
    angularImpulseBody2 += -rotationImpulse;

    // Compute the impulse P=J^T * lambda for the lower and upper limits constraints of body 2
    angularImpulseBody2 += -limitsImpulse;

    // Compute the impulse P=J^T * lambda for the motor constraint of body 2
    angularImpulseBody2 += -motorImpulse;

    // Apply the impulse to the body 2
    v2 += inverseMassBody2 * impulseTranslation;
    w2 += mWorld.mHingeJointsComponents.getI2(mEntity) * angularImpulseBody2;
}

// Solve the velocity constraint
void HingeJoint::solveVelocityConstraint(const ConstraintSolverData& constraintSolverData) {

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

    // Get the inverse mass and inverse inertia tensors of the bodies
    decimal inverseMassBody1 = constraintSolverData.rigidBodyComponents.getMassInverse(body1Entity);
    decimal inverseMassBody2 = constraintSolverData.rigidBodyComponents.getMassInverse(body2Entity);

    const Matrix3x3& i1 = mWorld.mHingeJointsComponents.getI1(mEntity);
    const Matrix3x3& i2 = mWorld.mHingeJointsComponents.getI2(mEntity);

    const Vector3& r1World = mWorld.mHingeJointsComponents.getR1World(mEntity);
    const Vector3& r2World = mWorld.mHingeJointsComponents.getR2World(mEntity);

    const Vector3& b2CrossA1 = mWorld.mHingeJointsComponents.getB2CrossA1(mEntity);
    const Vector3& c2CrossA1 = mWorld.mHingeJointsComponents.getC2CrossA1(mEntity);

    const Vector3& a1 = mWorld.mHingeJointsComponents.getA1(mEntity);

    const decimal inverseMassMatrixLimitMotor = mWorld.mHingeJointsComponents.getInverseMassMatrixLimitMotor(mEntity);

    // --------------- Translation Constraints --------------- //

    // Compute J*v
    const Vector3 JvTranslation = v2 + w2.cross(r2World) - v1 - w1.cross(r1World);

    // Compute the Lagrange multiplier lambda
    const Vector3 deltaLambdaTranslation = mWorld.mHingeJointsComponents.getInverseMassMatrixTranslation(mEntity) *
                                           (-JvTranslation - mWorld.mHingeJointsComponents.getBiasTranslation(mEntity));
    mWorld.mHingeJointsComponents.setImpulseTranslation(mEntity, deltaLambdaTranslation + mWorld.mHingeJointsComponents.getImpulseTranslation(mEntity));

    // Compute the impulse P=J^T * lambda of body 1
    const Vector3 linearImpulseBody1 = -deltaLambdaTranslation;
    Vector3 angularImpulseBody1 = deltaLambdaTranslation.cross(r1World);

    // Apply the impulse to the body 1
    v1 += inverseMassBody1 * linearImpulseBody1;
    w1 += i1 * angularImpulseBody1;

    // Compute the impulse P=J^T * lambda of body 2
    Vector3 angularImpulseBody2 = -deltaLambdaTranslation.cross(r2World);

    // Apply the impulse to the body 2
    v2 += inverseMassBody2 * deltaLambdaTranslation;
    w2 += i2 * angularImpulseBody2;

    // --------------- Rotation Constraints --------------- //

    // Compute J*v for the 2 rotation constraints
    const Vector2 JvRotation(-b2CrossA1.dot(w1) + b2CrossA1.dot(w2),
                             -c2CrossA1.dot(w1) + c2CrossA1.dot(w2));

    // Compute the Lagrange multiplier lambda for the 2 rotation constraints
    Vector2 deltaLambdaRotation = mWorld.mHingeJointsComponents.getInverseMassMatrixRotation(mEntity) *
                                  (-JvRotation - mWorld.mHingeJointsComponents.getBiasRotation(mEntity));
    mWorld.mHingeJointsComponents.setImpulseRotation(mEntity, deltaLambdaRotation + mWorld.mHingeJointsComponents.getImpulseRotation(mEntity));

    // Compute the impulse P=J^T * lambda for the 2 rotation constraints of body 1
    angularImpulseBody1 = -b2CrossA1 * deltaLambdaRotation.x -
                                        c2CrossA1 * deltaLambdaRotation.y;

    // Apply the impulse to the body 1
    w1 += i1 * angularImpulseBody1;

    // Compute the impulse P=J^T * lambda for the 2 rotation constraints of body 2
    angularImpulseBody2 = b2CrossA1 * deltaLambdaRotation.x + c2CrossA1 * deltaLambdaRotation.y;

    // Apply the impulse to the body 2
    w2 += i2 * angularImpulseBody2;

    // --------------- Limits Constraints --------------- //

    if (mWorld.mHingeJointsComponents.getIsLimitEnabled(mEntity)) {

        // If the lower limit is violated
        if (mWorld.mHingeJointsComponents.getIsLowerLimitViolated(mEntity)) {

            decimal impulseLowerLimit = mWorld.mHingeJointsComponents.getImpulseLowerLimit(mEntity);

            // Compute J*v for the lower limit constraint
            const decimal JvLowerLimit = (w2 - w1).dot(a1);

            // Compute the Lagrange multiplier lambda for the lower limit constraint
            decimal deltaLambdaLower = inverseMassMatrixLimitMotor * (-JvLowerLimit -mWorld.mHingeJointsComponents.getBLowerLimit(mEntity));
            decimal lambdaTemp = impulseLowerLimit;
            impulseLowerLimit = std::max(impulseLowerLimit + deltaLambdaLower, decimal(0.0));
            deltaLambdaLower = impulseLowerLimit - lambdaTemp;
            mWorld.mHingeJointsComponents.setImpulseLowerLimit(mEntity, impulseLowerLimit);

            // Compute the impulse P=J^T * lambda for the lower limit constraint of body 1
            const Vector3 angularImpulseBody1 = -deltaLambdaLower * a1;

            // Apply the impulse to the body 1
            w1 += i1 * angularImpulseBody1;

            // Compute the impulse P=J^T * lambda for the lower limit constraint of body 2
            const Vector3 angularImpulseBody2 = deltaLambdaLower * a1;

            // Apply the impulse to the body 2
            w2 += i2 * angularImpulseBody2;
        }

        // If the upper limit is violated
        if (mWorld.mHingeJointsComponents.getIsUpperLimitViolated(mEntity)) {

            decimal impulseUpperLimit = mWorld.mHingeJointsComponents.getImpulseUpperLimit(mEntity);

            // Compute J*v for the upper limit constraint
            const decimal JvUpperLimit = -(w2 - w1).dot(a1);

            // Compute the Lagrange multiplier lambda for the upper limit constraint
            decimal deltaLambdaUpper = inverseMassMatrixLimitMotor * (-JvUpperLimit -mWorld.mHingeJointsComponents.getBUpperLimit(mEntity));
            decimal lambdaTemp = impulseUpperLimit;
            impulseUpperLimit = std::max(impulseUpperLimit + deltaLambdaUpper, decimal(0.0));
            deltaLambdaUpper = impulseUpperLimit - lambdaTemp;
            mWorld.mHingeJointsComponents.setImpulseUpperLimit(mEntity, impulseUpperLimit);

            // Compute the impulse P=J^T * lambda for the upper limit constraint of body 1
            const Vector3 angularImpulseBody1 = deltaLambdaUpper * a1;

            // Apply the impulse to the body 1
            w1 += i1 * angularImpulseBody1;

            // Compute the impulse P=J^T * lambda for the upper limit constraint of body 2
            const Vector3 angularImpulseBody2 = -deltaLambdaUpper * a1;

            // Apply the impulse to the body 2
            w2 += i2 * angularImpulseBody2;
        }
    }

    // --------------- Motor --------------- //

    // If the motor is enabled
    if (mWorld.mHingeJointsComponents.getIsMotorEnabled(mEntity)) {

        decimal impulseMotor = mWorld.mHingeJointsComponents.getImpulseMotor(mEntity);

        // Compute J*v for the motor
        const decimal JvMotor = a1.dot(w1 - w2);

        // Compute the Lagrange multiplier lambda for the motor
        const decimal maxMotorImpulse = mWorld.mHingeJointsComponents.getMaxMotorTorque(mEntity) * constraintSolverData.timeStep;
        decimal deltaLambdaMotor = mWorld.mHingeJointsComponents.getInverseMassMatrixLimitMotor(mEntity) * (-JvMotor - mWorld.mHingeJointsComponents.getMotorSpeed(mEntity));
        decimal lambdaTemp = impulseMotor;
        impulseMotor = clamp(impulseMotor + deltaLambdaMotor, -maxMotorImpulse, maxMotorImpulse);
        deltaLambdaMotor = impulseMotor - lambdaTemp;
        mWorld.mHingeJointsComponents.setImpulseMotor(mEntity, impulseMotor);

        // Compute the impulse P=J^T * lambda for the motor of body 1
        const Vector3 angularImpulseBody1 = -deltaLambdaMotor * a1;

        // Apply the impulse to the body 1
        w1 += i1 * angularImpulseBody1;

        // Compute the impulse P=J^T * lambda for the motor of body 2
        const Vector3 angularImpulseBody2 = deltaLambdaMotor * a1;

        // Apply the impulse to the body 2
        w2 += i2 * angularImpulseBody2;
    }
}

// Solve the position constraint (for position error correction)
void HingeJoint::solvePositionConstraint(const ConstraintSolverData& constraintSolverData) {

    // If the error position correction technique is not the non-linear-gauss-seidel, we do
    // do not execute this method
    if (mWorld.mJointsComponents.getPositionCorrectionTechnique(mEntity) != JointsPositionCorrectionTechnique::NON_LINEAR_GAUSS_SEIDEL) return;

    // Get the bodies entities
    Entity body1Entity = mWorld.mJointsComponents.getBody1Entity(mEntity);
    Entity body2Entity = mWorld.mJointsComponents.getBody2Entity(mEntity);

    // TODO : Remove this and use compoents instead of pointers to bodies
    RigidBody* body1 = static_cast<RigidBody*>(mWorld.mRigidBodyComponents.getRigidBody(body1Entity));
    RigidBody* body2 = static_cast<RigidBody*>(mWorld.mRigidBodyComponents.getRigidBody(body2Entity));

    const Matrix3x3& i1 = mWorld.mHingeJointsComponents.getI1(mEntity);
    const Matrix3x3& i2 = mWorld.mHingeJointsComponents.getI2(mEntity);

    const Vector3& r1World = mWorld.mHingeJointsComponents.getR1World(mEntity);
    const Vector3& r2World = mWorld.mHingeJointsComponents.getR2World(mEntity);

    Vector3& b2CrossA1 = mWorld.mHingeJointsComponents.getB2CrossA1(mEntity);
    Vector3& c2CrossA1 = mWorld.mHingeJointsComponents.getC2CrossA1(mEntity);

    Vector3& a1 = mWorld.mHingeJointsComponents.getA1(mEntity);

    // Get the bodies positions and orientations
    Vector3 x1 = constraintSolverData.rigidBodyComponents.getConstrainedPosition(body1Entity);
    Vector3 x2 = constraintSolverData.rigidBodyComponents.getConstrainedPosition(body2Entity);
    Quaternion q1 = constraintSolverData.rigidBodyComponents.getConstrainedOrientation(body1Entity);
    Quaternion q2 = constraintSolverData.rigidBodyComponents.getConstrainedOrientation(body2Entity);

    // Get the inverse mass and inverse inertia tensors of the bodies
    decimal inverseMassBody1 = constraintSolverData.rigidBodyComponents.getMassInverse(body1Entity);
    decimal inverseMassBody2 = constraintSolverData.rigidBodyComponents.getMassInverse(body2Entity);

    // Recompute the inverse inertia tensors
    mWorld.mHingeJointsComponents.setI1(mEntity, body1->getInertiaTensorInverseWorld());
    mWorld.mHingeJointsComponents.setI2(mEntity, body2->getInertiaTensorInverseWorld());

    // Compute the vector from body center to the anchor point in world-space
    mWorld.mHingeJointsComponents.setR1World(mEntity, q1 * mWorld.mHingeJointsComponents.getLocalAnchorPointBody1(mEntity));
    mWorld.mHingeJointsComponents.setR2World(mEntity, q2 * mWorld.mHingeJointsComponents.getLocalAnchorPointBody2(mEntity));

    // Compute the current angle around the hinge axis
    decimal hingeAngle = computeCurrentHingeAngle(q1, q2);

    // Check if the limit constraints are violated or not
    decimal lowerLimitError = hingeAngle - mWorld.mHingeJointsComponents.getLowerLimit(mEntity);
    decimal upperLimitError = mWorld.mHingeJointsComponents.getUpperLimit(mEntity) - hingeAngle;
    mWorld.mHingeJointsComponents.setIsLowerLimitViolated(mEntity, lowerLimitError <= 0);
    mWorld.mHingeJointsComponents.setIsUpperLimitViolated(mEntity, upperLimitError <= 0);

    // Compute vectors needed in the Jacobian
    a1 = q1 * mWorld.mHingeJointsComponents.getHingeLocalAxisBody1(mEntity);
    Vector3 a2 = q2 * mWorld.mHingeJointsComponents.getHingeLocalAxisBody2(mEntity);
    a1.normalize();
    mWorld.mHingeJointsComponents.setA1(mEntity, a1);
    a2.normalize();
    const Vector3 b2 = a2.getOneUnitOrthogonalVector();
    const Vector3 c2 = a2.cross(b2);
    b2CrossA1 = b2.cross(a1);
    mWorld.mHingeJointsComponents.setB2CrossA1(mEntity, b2CrossA1);
    c2CrossA1 = c2.cross(a1);
    mWorld.mHingeJointsComponents.setC2CrossA1(mEntity, c2CrossA1);

    // Compute the corresponding skew-symmetric matrices
    Matrix3x3 skewSymmetricMatrixU1= Matrix3x3::computeSkewSymmetricMatrixForCrossProduct(r1World);
    Matrix3x3 skewSymmetricMatrixU2= Matrix3x3::computeSkewSymmetricMatrixForCrossProduct(r2World);

    // --------------- Translation Constraints --------------- //

    // Compute the matrix K=JM^-1J^t (3x3 matrix) for the 3 translation constraints
    const decimal body1InverseMass = constraintSolverData.rigidBodyComponents.getMassInverse(body1Entity);
    const decimal body2InverseMass = constraintSolverData.rigidBodyComponents.getMassInverse(body2Entity);
    decimal inverseMassBodies = body1InverseMass + body2InverseMass;
    Matrix3x3 massMatrix = Matrix3x3(inverseMassBodies, 0, 0,
                                    0, inverseMassBodies, 0,
                                    0, 0, inverseMassBodies) +
                           skewSymmetricMatrixU1 * i1 * skewSymmetricMatrixU1.getTranspose() +
                           skewSymmetricMatrixU2 * i2 * skewSymmetricMatrixU2.getTranspose();
    Matrix3x3& inverseMassMatrixTranslation = mWorld.mHingeJointsComponents.getInverseMassMatrixTranslation(mEntity);
    inverseMassMatrixTranslation.setToZero();
    if (mWorld.mRigidBodyComponents.getBodyType(body1Entity) == BodyType::DYNAMIC ||
        mWorld.mRigidBodyComponents.getBodyType(body2Entity) == BodyType::DYNAMIC) {
        inverseMassMatrixTranslation = massMatrix.getInverse();
        mWorld.mHingeJointsComponents.setInverseMassMatrixTranslation(mEntity, inverseMassMatrixTranslation);
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

    // Compute the inverse mass matrix K=JM^-1J^t for the 2 rotation constraints (2x2 matrix)
    Vector3 I1B2CrossA1 = i1 * b2CrossA1;
    Vector3 I1C2CrossA1 = i1 * c2CrossA1;
    Vector3 I2B2CrossA1 = i2 * b2CrossA1;
    Vector3 I2C2CrossA1 = i2 * c2CrossA1;
    const decimal el11 = b2CrossA1.dot(I1B2CrossA1) +
                         b2CrossA1.dot(I2B2CrossA1);
    const decimal el12 = b2CrossA1.dot(I1C2CrossA1) +
                         b2CrossA1.dot(I2C2CrossA1);
    const decimal el21 = c2CrossA1.dot(I1B2CrossA1) +
                         c2CrossA1.dot(I2B2CrossA1);
    const decimal el22 = c2CrossA1.dot(I1C2CrossA1) +
                         c2CrossA1.dot(I2C2CrossA1);
    const Matrix2x2 matrixKRotation(el11, el12, el21, el22);
    Matrix2x2& inverseMassMatrixRotation = mWorld.mHingeJointsComponents.getInverseMassMatrixRotation(mEntity);
    inverseMassMatrixRotation.setToZero();
    if (mWorld.mRigidBodyComponents.getBodyType(body1Entity) == BodyType::DYNAMIC ||
        mWorld.mRigidBodyComponents.getBodyType(body2Entity) == BodyType::DYNAMIC) {
        mWorld.mHingeJointsComponents.setInverseMassMatrixRotation(mEntity, matrixKRotation.getInverse());
    }

    // Compute the position error for the 3 rotation constraints
    const Vector2 errorRotation = Vector2(a1.dot(b2), a1.dot(c2));

    // Compute the Lagrange multiplier lambda for the 3 rotation constraints
    Vector2 lambdaRotation = inverseMassMatrixRotation * (-errorRotation);

    // Compute the impulse P=J^T * lambda for the 3 rotation constraints of body 1
    angularImpulseBody1 = -b2CrossA1 * lambdaRotation.x - c2CrossA1 * lambdaRotation.y;

    // Compute the pseudo velocity of body 1
    w1 = i1 * angularImpulseBody1;

    // Update the body position/orientation of body 1
    q1 += Quaternion(0, w1) * q1 * decimal(0.5);
    q1.normalize();

    // Compute the impulse of body 2
    angularImpulseBody2 = b2CrossA1 * lambdaRotation.x + c2CrossA1 * lambdaRotation.y;

    // Compute the pseudo velocity of body 2
    w2 = i2 * angularImpulseBody2;

    // Update the body position/orientation of body 2
    q2 += Quaternion(0, w2) * q2 * decimal(0.5);
    q2.normalize();

    // --------------- Limits Constraints --------------- //

    if (mWorld.mHingeJointsComponents.getIsLimitEnabled(mEntity)) {

        decimal inverseMassMatrixLimitMotor = mWorld.mHingeJointsComponents.getInverseMassMatrixLimitMotor(mEntity);

        if (mWorld.mHingeJointsComponents.getIsLowerLimitViolated(mEntity) || mWorld.mHingeJointsComponents.getIsUpperLimitViolated(mEntity)) {

            // Compute the inverse of the mass matrix K=JM^-1J^t for the limits (1x1 matrix)
            inverseMassMatrixLimitMotor = a1.dot(i1 * a1) + a1.dot(i2 * a1);
            inverseMassMatrixLimitMotor = (inverseMassMatrixLimitMotor > decimal(0.0)) ?
                                      decimal(1.0) / inverseMassMatrixLimitMotor : decimal(0.0);
            mWorld.mHingeJointsComponents.setInverseMassMatrixLimitMotor(mEntity, inverseMassMatrixLimitMotor);
        }

        // If the lower limit is violated
        if (mWorld.mHingeJointsComponents.getIsLowerLimitViolated(mEntity)) {

            // Compute the Lagrange multiplier lambda for the lower limit constraint
            decimal lambdaLowerLimit = inverseMassMatrixLimitMotor * (-lowerLimitError );

            // Compute the impulse P=J^T * lambda of body 1
            const Vector3 angularImpulseBody1 = -lambdaLowerLimit * a1;

            // Compute the pseudo velocity of body 1
            const Vector3 w1 = i1 * angularImpulseBody1;

            // Update the body position/orientation of body 1
            q1 += Quaternion(0, w1) * q1 * decimal(0.5);
            q1.normalize();

            // Compute the impulse P=J^T * lambda of body 2
            const Vector3 angularImpulseBody2 = lambdaLowerLimit * a1;

            // Compute the pseudo velocity of body 2
            const Vector3 w2 = i2 * angularImpulseBody2;

            // Update the body position/orientation of body 2
            q2 += Quaternion(0, w2) * q2 * decimal(0.5);
            q2.normalize();
        }

        // If the upper limit is violated
        if (mWorld.mHingeJointsComponents.getIsUpperLimitViolated(mEntity)) {

            // Compute the Lagrange multiplier lambda for the upper limit constraint
            decimal lambdaUpperLimit = inverseMassMatrixLimitMotor * (-upperLimitError);

            // Compute the impulse P=J^T * lambda of body 1
            const Vector3 angularImpulseBody1 = lambdaUpperLimit * a1;

            // Compute the pseudo velocity of body 1
            const Vector3 w1 = i1 * angularImpulseBody1;

            // Update the body position/orientation of body 1
            q1 += Quaternion(0, w1) * q1 * decimal(0.5);
            q1.normalize();

            // Compute the impulse P=J^T * lambda of body 2
            const Vector3 angularImpulseBody2 = -lambdaUpperLimit * a1;

            // Compute the pseudo velocity of body 2
            const Vector3 w2 = i2 * angularImpulseBody2;

            // Update the body position/orientation of body 2
            q2 += Quaternion(0, w2) * q2 * decimal(0.5);
            q2.normalize();
        }
    }

    constraintSolverData.rigidBodyComponents.setConstrainedPosition(body1Entity, x1);
    constraintSolverData.rigidBodyComponents.setConstrainedPosition(body2Entity, x2);
    constraintSolverData.rigidBodyComponents.setConstrainedOrientation(body1Entity, q1);
    constraintSolverData.rigidBodyComponents.setConstrainedOrientation(body2Entity, q2);
}


// Enable/Disable the limits of the joint
/**
 * @param isLimitEnabled True if you want to enable the limits of the joint and
 *                       false otherwise
 */
void HingeJoint::enableLimit(bool isLimitEnabled) {

    if (isLimitEnabled != mWorld.mHingeJointsComponents.getIsLimitEnabled(mEntity)) {

        mWorld.mHingeJointsComponents.setIsLimitEnabled(mEntity, isLimitEnabled);

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

    mWorld.mHingeJointsComponents.setIsMotorEnabled(mEntity, isMotorEnabled);
    mWorld.mHingeJointsComponents.setImpulseMotor(mEntity, decimal(0.0));

    // Wake up the two bodies of the joint
    awakeBodies();
}

// Set the minimum angle limit
/**
 * @param lowerLimit The minimum limit angle of the joint (in radian)
 */
void HingeJoint::setMinAngleLimit(decimal lowerLimit) {

    const decimal limit = mWorld.mHingeJointsComponents.getLowerLimit(mEntity);

    assert(limit <= decimal(0.0) && limit >= decimal(-2.0) * PI);

    if (lowerLimit != limit) {

        mWorld.mHingeJointsComponents.setLowerLimit(mEntity, lowerLimit);

        // Reset the limits
        resetLimits();
    }
}

// Set the maximum angle limit
/**
 * @param upperLimit The maximum limit angle of the joint (in radian)
 */
void HingeJoint::setMaxAngleLimit(decimal upperLimit) {

    const decimal limit = mWorld.mHingeJointsComponents.getUpperLimit(mEntity);

    assert(limit >= decimal(0) && limit <= decimal(2.0) * PI);

    if (upperLimit != limit) {

        mWorld.mHingeJointsComponents.setUpperLimit(mEntity, upperLimit);

        // Reset the limits
        resetLimits();
    }
}

// Reset the limits
void HingeJoint::resetLimits() {

    // Reset the accumulated impulses for the limits
    mWorld.mHingeJointsComponents.setImpulseLowerLimit(mEntity, decimal(0.0));
    mWorld.mHingeJointsComponents.setImpulseUpperLimit(mEntity, decimal(0.0));

    // Wake up the two bodies of the joint
    awakeBodies();
}

// Set the motor speed
void HingeJoint::setMotorSpeed(decimal motorSpeed) {

    if (motorSpeed != mWorld.mHingeJointsComponents.getMotorSpeed(mEntity)) {

        mWorld.mHingeJointsComponents.setMotorSpeed(mEntity, motorSpeed);

        // Wake up the two bodies of the joint
        awakeBodies();
    }
}

// Set the maximum motor torque
/**
 * @param maxMotorTorque The maximum torque (in Newtons) of the joint motor
 */
void HingeJoint::setMaxMotorTorque(decimal maxMotorTorque) {

    const decimal torque = mWorld.mHingeJointsComponents.getMaxMotorTorque(mEntity);

    if (maxMotorTorque != torque) {

        assert(torque >= decimal(0.0));
        mWorld.mHingeJointsComponents.setMaxMotorTorque(mEntity, maxMotorTorque);

        // Wake up the two bodies of the joint
        awakeBodies();
    }
}

// Given an angle in radian, this method returns the corresponding angle in the range [-pi; pi]
decimal HingeJoint::computeNormalizedAngle(decimal angle) const {

    // Convert it into the range [-2*pi; 2*pi]
    angle = std::fmod(angle, PI_TIMES_2);

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
decimal HingeJoint::computeCorrespondingAngleNearLimits(decimal inputAngle, decimal lowerLimitAngle, decimal upperLimitAngle) const {
    if (upperLimitAngle <= lowerLimitAngle) {
        return inputAngle;
    }
    else if (inputAngle > upperLimitAngle) {
        decimal diffToUpperLimit = std::fabs(computeNormalizedAngle(inputAngle - upperLimitAngle));
        decimal diffToLowerLimit = std::fabs(computeNormalizedAngle(inputAngle - lowerLimitAngle));
        return (diffToUpperLimit > diffToLowerLimit) ? (inputAngle - PI_TIMES_2) : inputAngle;
    }
    else if (inputAngle < lowerLimitAngle) {
        decimal diffToUpperLimit = std::fabs(computeNormalizedAngle(upperLimitAngle - inputAngle));
        decimal diffToLowerLimit = std::fabs(computeNormalizedAngle(lowerLimitAngle - inputAngle));
        return (diffToUpperLimit > diffToLowerLimit) ? inputAngle : (inputAngle + PI_TIMES_2);
    }
    else {
        return inputAngle;
    }
}

// Compute the current angle around the hinge axis
decimal HingeJoint::computeCurrentHingeAngle(const Quaternion& orientationBody1, const Quaternion& orientationBody2) {

    decimal hingeAngle;

    // Compute the current orientation difference between the two bodies
    Quaternion currentOrientationDiff = orientationBody2 * orientationBody1.getInverse();
    currentOrientationDiff.normalize();

    // Compute the relative rotation considering the initial orientation difference
    Quaternion relativeRotation = currentOrientationDiff * mWorld.mHingeJointsComponents.getInitOrientationDifferenceInv(mEntity);
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
    decimal dotProduct = relativeRotation.getVectorV().dot(mWorld.mHingeJointsComponents.getA1(mEntity));

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
                                               mWorld.mHingeJointsComponents.getLowerLimit(mEntity),
                                               mWorld.mHingeJointsComponents.getUpperLimit(mEntity));
}

// Return true if the limits of the joint are enabled
/**
 * @return True if the limits of the joint are enabled and false otherwise
 */
bool HingeJoint::isLimitEnabled() const {
    return mWorld.mHingeJointsComponents.getIsLimitEnabled(mEntity);
}

// Return true if the motor of the joint is enabled
/**
 * @return True if the motor of joint is enabled and false otherwise
 */
bool HingeJoint::isMotorEnabled() const {
    return mWorld.mHingeJointsComponents.getIsMotorEnabled(mEntity);
}

// Return the minimum angle limit
/**
 * @return The minimum limit angle of the joint (in radian)
 */
decimal HingeJoint::getMinAngleLimit() const {
    return mWorld.mHingeJointsComponents.getLowerLimit(mEntity);
}

// Return the maximum angle limit
/**
 * @return The maximum limit angle of the joint (in radian)
 */
decimal HingeJoint::getMaxAngleLimit() const {
    return mWorld.mHingeJointsComponents.getUpperLimit(mEntity);
}

// Return the motor speed
/**
 * @return The current speed of the joint motor (in radian per second)
 */
 decimal HingeJoint::getMotorSpeed() const {
    return mWorld.mHingeJointsComponents.getMotorSpeed(mEntity);
}

// Return the maximum motor torque
/**
 * @return The maximum torque of the joint motor (in Newtons)
 */
 decimal HingeJoint::getMaxMotorTorque() const {
    return mWorld.mHingeJointsComponents.getMaxMotorTorque(mEntity);
}

// Return the intensity of the current torque applied for the joint motor
/**
 * @param timeStep The current time step (in seconds)
 * @return The intensity of the current torque (in Newtons) of the joint motor
 */
 decimal HingeJoint::getMotorTorque(decimal timeStep) const {
    return mWorld.mHingeJointsComponents.getImpulseMotor(mEntity) / timeStep;
}

// Return the number of bytes used by the joint
 size_t HingeJoint::getSizeInBytes() const {
    return sizeof(HingeJoint);
}

// Return a string representation
std::string HingeJoint::to_string() const {
    return "HingeJoint{ lowerLimit=" + std::to_string(mWorld.mHingeJointsComponents.getLowerLimit(mEntity)) +
            ", upperLimit=" + std::to_string(mWorld.mHingeJointsComponents.getUpperLimit(mEntity)) +
            "localAnchorPointBody1=" + mWorld.mHingeJointsComponents.getLocalAnchorPointBody1(mEntity).to_string() + ", localAnchorPointBody2=" +
            mWorld.mHingeJointsComponents.getLocalAnchorPointBody2(mEntity).to_string() + ", hingeLocalAxisBody1=" +
            mWorld.mHingeJointsComponents.getHingeLocalAxisBody1(mEntity).to_string() +
            ", hingeLocalAxisBody2=" + mWorld.mHingeJointsComponents.getHingeLocalAxisBody2(mEntity).to_string() +
            ", initOrientationDifferenceInv=" + mWorld.mHingeJointsComponents.getInitOrientationDifferenceInv(mEntity).to_string() +
            ", motorSpeed=" + std::to_string(mWorld.mHingeJointsComponents.getMotorSpeed(mEntity)) +
            ", maxMotorTorque=" + std::to_string(mWorld.mHingeJointsComponents.getMaxMotorTorque(mEntity)) + ", isLimitEnabled=" +
            (mWorld.mHingeJointsComponents.getIsLimitEnabled(mEntity) ? "true" : "false") + ", isMotorEnabled=" +
            (mWorld.mHingeJointsComponents.getIsMotorEnabled(mEntity) ? "true" : "false") + "}";
}
