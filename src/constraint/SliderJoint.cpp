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
#include "systems/ConstraintSolverSystem.h"
#include "components/RigidBodyComponents.h"
#include "engine/DynamicsWorld.h"

using namespace reactphysics3d;

// Static variables definition
const decimal SliderJoint::BETA = decimal(0.2);

// Constructor
SliderJoint::SliderJoint(Entity entity, DynamicsWorld &world, const SliderJointInfo& jointInfo)
            : Joint(entity, world) {

    assert(mWorld.mSliderJointsComponents.getUpperLimit(mEntity) >= decimal(0.0));
    assert(mWorld.mSliderJointsComponents.getLowerLimit(mEntity) <= decimal(0.0));
    assert(mWorld.mSliderJointsComponents.getMaxMotorForce(mEntity) >= decimal(0.0));

    // Compute the local-space anchor point for each body
    const Transform& transform1 = mWorld.mTransformComponents.getTransform(jointInfo.body1->getEntity());
    const Transform& transform2 = mWorld.mTransformComponents.getTransform(jointInfo.body2->getEntity());
    mWorld.mSliderJointsComponents.setLocalAnchorPointBody1(mEntity, transform1.getInverse() * jointInfo.anchorPointWorldSpace);
    mWorld.mSliderJointsComponents.setLocalAnchorPointBody2(mEntity, transform2.getInverse() * jointInfo.anchorPointWorldSpace);

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
    // TODO : Do not compute the inverse here, it has already been computed above
    mWorld.mSliderJointsComponents.setInitOrientationDifferenceInv(mEntity, transform2.getOrientation().getInverse() * transform1.getOrientation());

    // Compute the slider axis in local-space of body 1
    // TODO : Do not compute the inverse here, it has already been computed above
    Vector3 sliderAxisBody1 = transform1.getOrientation().getInverse() *
                       jointInfo.sliderAxisWorldSpace;
    sliderAxisBody1.normalize();
    mWorld.mSliderJointsComponents.setSliderAxisBody1(mEntity, sliderAxisBody1);
}

// Initialize before solving the constraint
void SliderJoint::initBeforeSolve(const ConstraintSolverData& constraintSolverData) {

    // Get the bodies entities
    const Entity body1Entity = mWorld.mJointsComponents.getBody1Entity(mEntity);
    const Entity body2Entity = mWorld.mJointsComponents.getBody2Entity(mEntity);

    // TODO : Remove this and use compoents instead of pointers to bodies
    RigidBody* body1 = static_cast<RigidBody*>(mWorld.mRigidBodyComponents.getRigidBody(body1Entity));
    RigidBody* body2 = static_cast<RigidBody*>(mWorld.mRigidBodyComponents.getRigidBody(body2Entity));

    // Get the bodies positions and orientations
    const Vector3& x1 = constraintSolverData.rigidBodyComponents.getCenterOfMassWorld(body1Entity);
    const Vector3& x2 = constraintSolverData.rigidBodyComponents.getCenterOfMassWorld(body2Entity);
    const Quaternion& orientationBody1 = mWorld.mTransformComponents.getTransform(body1Entity).getOrientation();
    const Quaternion& orientationBody2 = mWorld.mTransformComponents.getTransform(body2Entity).getOrientation();

    // Get the inertia tensor of bodies
    mWorld.mSliderJointsComponents.setI1(mEntity, body1->getInertiaTensorInverseWorld());
    mWorld.mSliderJointsComponents.setI2(mEntity, body2->getInertiaTensorInverseWorld());

    // Vector from body center to the anchor point
    mWorld.mSliderJointsComponents.setR1(mEntity, orientationBody1 * mWorld.mSliderJointsComponents.getLocalAnchorPointBody1(mEntity));
    mWorld.mSliderJointsComponents.setR2(mEntity, orientationBody2 * mWorld.mSliderJointsComponents.getLocalAnchorPointBody2(mEntity));

    const Vector3& r1 = mWorld.mSliderJointsComponents.getR1(mEntity);
    const Vector3& r2 = mWorld.mSliderJointsComponents.getR2(mEntity);

    // Compute the vector u (difference between anchor points)
    const Vector3 u = x2 + r2 - x1 - r1;

    // Compute the two orthogonal vectors to the slider axis in world-space
    Vector3 sliderAxisWorld = orientationBody1 * mWorld.mSliderJointsComponents.getSliderAxisBody1(mEntity);
    sliderAxisWorld.normalize();
    mWorld.mSliderJointsComponents.setSliderAxisWorld(mEntity, sliderAxisWorld);
    mWorld.mSliderJointsComponents.setN1(mEntity, sliderAxisWorld.getOneUnitOrthogonalVector());
    const Vector3& n1 = mWorld.mSliderJointsComponents.getN1(mEntity);
    mWorld.mSliderJointsComponents.setN2(mEntity, sliderAxisWorld.cross(n1));
    const Vector3& n2 = mWorld.mSliderJointsComponents.getN2(mEntity);

    // Check if the limit constraints are violated or not
    decimal uDotSliderAxis = u.dot(sliderAxisWorld);
    decimal lowerLimitError = uDotSliderAxis - mWorld.mSliderJointsComponents.getLowerLimit(mEntity);
    decimal upperLimitError = mWorld.mSliderJointsComponents.getUpperLimit(mEntity) - uDotSliderAxis;
    bool oldIsLowerLimitViolated = mWorld.mSliderJointsComponents.getIsLowerLimitViolated(mEntity);
    bool isLowerLimitViolated = lowerLimitError <= 0;
    mWorld.mSliderJointsComponents.setIsLowerLimitViolated(mEntity, isLowerLimitViolated);
    if (isLowerLimitViolated != oldIsLowerLimitViolated) {
        mWorld.mSliderJointsComponents.setImpulseLowerLimit(mEntity, decimal(0.0));
    }
    bool oldIsUpperLimitViolated = mWorld.mSliderJointsComponents.getIsUpperLimitViolated(mEntity);
    bool isUpperLimitViolated = upperLimitError <= 0;
    mWorld.mSliderJointsComponents.setIsUpperLimitViolated(mEntity, isUpperLimitViolated);
    if (isUpperLimitViolated != oldIsUpperLimitViolated) {
        mWorld.mSliderJointsComponents.setImpulseUpperLimit(mEntity, decimal(0.0));
    }

    // Compute the cross products used in the Jacobians
    mWorld.mSliderJointsComponents.setR2CrossN1(mEntity, r2.cross(n1));
    mWorld.mSliderJointsComponents.setR2CrossN2(mEntity, r2.cross(n2));
    mWorld.mSliderJointsComponents.setR2CrossSliderAxis(mEntity, r2.cross(sliderAxisWorld));
    const Vector3 r1PlusU = r1 + u;
    mWorld.mSliderJointsComponents.setR1PlusUCrossN1(mEntity, r1PlusU.cross(n1));
    mWorld.mSliderJointsComponents.setR1PlusUCrossN2(mEntity, r1PlusU.cross(n2));
    mWorld.mSliderJointsComponents.setR1PlusUCrossSliderAxis(mEntity, r1PlusU.cross(sliderAxisWorld));

    const Vector3& r2CrossN1 = mWorld.mSliderJointsComponents.getR2CrossN1(mEntity);
    const Vector3& r2CrossN2 = mWorld.mSliderJointsComponents.getR2CrossN2(mEntity);
    const Vector3& r1PlusUCrossN1 = mWorld.mSliderJointsComponents.getR1PlusUCrossN1(mEntity);
    const Vector3& r1PlusUCrossN2 = mWorld.mSliderJointsComponents.getR1PlusUCrossN2(mEntity);
    const Vector3& r2CrossSliderAxis = mWorld.mSliderJointsComponents.getR2CrossSliderAxis(mEntity);
    const Vector3& r1PlusUCrossSliderAxis = mWorld.mSliderJointsComponents.getR1PlusUCrossSliderAxis(mEntity);

    const Matrix3x3& i1 = mWorld.mSliderJointsComponents.getI1(mEntity);
    const Matrix3x3& i2 = mWorld.mSliderJointsComponents.getI2(mEntity);

    // Compute the inverse of the mass matrix K=JM^-1J^t for the 2 translation
    // constraints (2x2 matrix)
    const decimal body1MassInverse = constraintSolverData.rigidBodyComponents.getMassInverse(body1Entity);
    const decimal body2MassInverse = constraintSolverData.rigidBodyComponents.getMassInverse(body2Entity);
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
    Matrix2x2& inverseMassMatrixTranslation = mWorld.mSliderJointsComponents.getInverseMassMatrixTranslation(mEntity);
    inverseMassMatrixTranslation.setToZero();
    if (mWorld.mRigidBodyComponents.getBodyType(body1Entity) == BodyType::DYNAMIC ||
        mWorld.mRigidBodyComponents.getBodyType(body2Entity) == BodyType::DYNAMIC) {

        mWorld.mSliderJointsComponents.setInverseMassMatrixTranslation(mEntity, matrixKTranslation.getInverse());
    }

    // Compute the bias "b" of the translation constraint
    Vector2& biasTranslation = mWorld.mSliderJointsComponents.getBiasTranslation(mEntity);
    biasTranslation.setToZero();
    decimal biasFactor = (BETA / constraintSolverData.timeStep);
    if (mWorld.mJointsComponents.getPositionCorrectionTechnique(mEntity) == JointsPositionCorrectionTechnique::BAUMGARTE_JOINTS) {
        biasTranslation.x = u.dot(n1);
        biasTranslation.y = u.dot(n2);
        biasTranslation *= biasFactor;
        mWorld.mSliderJointsComponents.setBiasTranslation(mEntity, biasTranslation);
    }

    // Compute the inverse of the mass matrix K=JM^-1J^t for the 3 rotation
    // contraints (3x3 matrix)
    mWorld.mSliderJointsComponents.setInverseMassMatrixRotation(mEntity, i1 + i2);
    if (mWorld.mRigidBodyComponents.getBodyType(body1Entity) == BodyType::DYNAMIC ||
        mWorld.mRigidBodyComponents.getBodyType(body2Entity) == BodyType::DYNAMIC) {

        mWorld.mSliderJointsComponents.setInverseMassMatrixRotation(mEntity, mWorld.mSliderJointsComponents.getInverseMassMatrixRotation(mEntity).getInverse());
    }

    // Compute the bias "b" of the rotation constraint
    Vector3& biasRotation = mWorld.mSliderJointsComponents.getBiasRotation(mEntity);
    biasRotation.setToZero();
    if (mWorld.mJointsComponents.getPositionCorrectionTechnique(mEntity) == JointsPositionCorrectionTechnique::BAUMGARTE_JOINTS) {
        const Quaternion qError = orientationBody2 * mWorld.mSliderJointsComponents.getInitOrientationDifferenceInv(mEntity) * orientationBody1.getInverse();
        mWorld.mSliderJointsComponents.setBiasRotation(mEntity, biasFactor * decimal(2.0) * qError.getVectorV());
    }

    // If the limits are enabled
    if (mWorld.mSliderJointsComponents.getIsLimitEnabled(mEntity) && (mWorld.mSliderJointsComponents.getIsLowerLimitViolated(mEntity) ||
                                                                      mWorld.mSliderJointsComponents.getIsUpperLimitViolated(mEntity))) {

        // Compute the inverse of the mass matrix K=JM^-1J^t for the limits (1x1 matrix)
        decimal inverseMassMatrixLimit = sumInverseMass +
                                  r1PlusUCrossSliderAxis.dot(i1 * r1PlusUCrossSliderAxis) +
                                  r2CrossSliderAxis.dot(i2 * r2CrossSliderAxis);
        inverseMassMatrixLimit = (inverseMassMatrixLimit > decimal(0.0)) ?
                                  decimal(1.0) / inverseMassMatrixLimit : decimal(0.0);
        mWorld.mSliderJointsComponents.setInverseMassMatrixLimit(mEntity, inverseMassMatrixLimit);

        // Compute the bias "b" of the lower limit constraint
        mWorld.mSliderJointsComponents.setBLowerLimit(mEntity, decimal(0.0));
        if (mWorld.mJointsComponents.getPositionCorrectionTechnique(mEntity) == JointsPositionCorrectionTechnique::BAUMGARTE_JOINTS) {
            mWorld.mSliderJointsComponents.setBLowerLimit(mEntity, biasFactor * lowerLimitError);
        }

        // Compute the bias "b" of the upper limit constraint
        mWorld.mSliderJointsComponents.setBUpperLimit(mEntity, decimal(0.0));
        if (mWorld.mJointsComponents.getPositionCorrectionTechnique(mEntity) == JointsPositionCorrectionTechnique::BAUMGARTE_JOINTS) {
            mWorld.mSliderJointsComponents.setBUpperLimit(mEntity, biasFactor * upperLimitError);
        }
    }

    // If the motor is enabled
    if (mWorld.mSliderJointsComponents.getIsMotorEnabled(mEntity)) {

        // Compute the inverse of mass matrix K=JM^-1J^t for the motor (1x1 matrix)
        decimal inverseMassMatrixMotor = sumInverseMass;
        inverseMassMatrixMotor = (inverseMassMatrixMotor > decimal(0.0)) ?
                    decimal(1.0) / inverseMassMatrixMotor : decimal(0.0);
        mWorld.mSliderJointsComponents.setInverseMassMatrixMotor(mEntity, inverseMassMatrixMotor);
    }

    // If warm-starting is not enabled
    if (!constraintSolverData.isWarmStartingActive) {

        // Reset all the accumulated impulses
        Vector2& impulseTranslation = mWorld.mSliderJointsComponents.getImpulseTranslation(mEntity);
        Vector3& impulseRotation = mWorld.mSliderJointsComponents.getImpulseRotation(mEntity);
        impulseTranslation.setToZero();
        impulseRotation.setToZero();
        mWorld.mSliderJointsComponents.setImpulseLowerLimit(mEntity, decimal(0.0));
        mWorld.mSliderJointsComponents.setImpulseUpperLimit(mEntity, decimal(0.0));
        mWorld.mSliderJointsComponents.setImpulseMotor(mEntity, decimal(0.0));
    }
}

// Warm start the constraint (apply the previous impulse at the beginning of the step)
void SliderJoint::warmstart(const ConstraintSolverData& constraintSolverData) {

    // Get the bodies entities
    const Entity body1Entity = mWorld.mJointsComponents.getBody1Entity(mEntity);
    const Entity body2Entity = mWorld.mJointsComponents.getBody2Entity(mEntity);

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

    const Vector3& n1 = mWorld.mSliderJointsComponents.getN1(mEntity);
    const Vector3& n2 = mWorld.mSliderJointsComponents.getN2(mEntity);

    // Compute the impulse P=J^T * lambda for the lower and upper limits constraints of body 1
    decimal impulseLimits = mWorld.mSliderJointsComponents.getImpulseUpperLimit(mEntity) - mWorld.mSliderJointsComponents.getImpulseLowerLimit(mEntity);
    Vector3 linearImpulseLimits = impulseLimits * mWorld.mSliderJointsComponents.getSliderAxisWorld(mEntity);

    // Compute the impulse P=J^T * lambda for the motor constraint of body 1
    Vector3 impulseMotor = mWorld.mSliderJointsComponents.getImpulseMotor(mEntity) * mWorld.mSliderJointsComponents.getSliderAxisWorld(mEntity);

    const Vector2& impulseTranslation = mWorld.mSliderJointsComponents.getImpulseTranslation(mEntity);
    const Vector3& impulseRotation = mWorld.mSliderJointsComponents.getImpulseRotation(mEntity);

    // Compute the impulse P=J^T * lambda for the 2 translation constraints of body 1
    Vector3 linearImpulseBody1 = -n1 * impulseTranslation.x - n2 * impulseTranslation.y;
    Vector3 angularImpulseBody1 = -mWorld.mSliderJointsComponents.getR1PlusUCrossN1(mEntity) * impulseTranslation.x -
            mWorld.mSliderJointsComponents.getR1PlusUCrossN2(mEntity) * impulseTranslation.y;

    // Compute the impulse P=J^T * lambda for the 3 rotation constraints of body 1
    angularImpulseBody1 += -impulseRotation;

    // Compute the impulse P=J^T * lambda for the lower and upper limits constraints of body 1
    linearImpulseBody1 += linearImpulseLimits;
    angularImpulseBody1 += impulseLimits * mWorld.mSliderJointsComponents.getR1PlusUCrossSliderAxis(mEntity);

    // Compute the impulse P=J^T * lambda for the motor constraint of body 1
    linearImpulseBody1 += impulseMotor;

    // Apply the impulse to the body 1
    v1 += inverseMassBody1 * linearImpulseBody1;
    w1 += mWorld.mSliderJointsComponents.getI1(mEntity) * angularImpulseBody1;

    // Compute the impulse P=J^T * lambda for the 2 translation constraints of body 2
    Vector3 linearImpulseBody2 = n1 * impulseTranslation.x + n2 * impulseTranslation.y;
    Vector3 angularImpulseBody2 = mWorld.mSliderJointsComponents.getR2CrossN1(mEntity) * impulseTranslation.x +
            mWorld.mSliderJointsComponents.getR2CrossN2(mEntity) * impulseTranslation.y;

    // Compute the impulse P=J^T * lambda for the 3 rotation constraints of body 2
    angularImpulseBody2 += impulseRotation;

    // Compute the impulse P=J^T * lambda for the lower and upper limits constraints of body 2
    linearImpulseBody2 += -linearImpulseLimits;
    angularImpulseBody2 += -impulseLimits * mWorld.mSliderJointsComponents.getR2CrossSliderAxis(mEntity);

    // Compute the impulse P=J^T * lambda for the motor constraint of body 2
    linearImpulseBody2 += -impulseMotor;

    // Apply the impulse to the body 2
    v2 += inverseMassBody2 * linearImpulseBody2;
    w2 += mWorld.mSliderJointsComponents.getI2(mEntity) * angularImpulseBody2;
}

// Solve the velocity constraint
void SliderJoint::solveVelocityConstraint(const ConstraintSolverData& constraintSolverData) {

    // Get the bodies entities
    const Entity body1Entity = mWorld.mJointsComponents.getBody1Entity(mEntity);
    const Entity body2Entity = mWorld.mJointsComponents.getBody2Entity(mEntity);

    uint32 dynamicsComponentIndexBody1 = constraintSolverData.rigidBodyComponents.getEntityIndex(body1Entity);
    uint32 dynamicsComponentIndexBody2 = constraintSolverData.rigidBodyComponents.getEntityIndex(body2Entity);

    // Get the velocities
    Vector3& v1 = constraintSolverData.rigidBodyComponents.mConstrainedLinearVelocities[dynamicsComponentIndexBody1];
    Vector3& v2 = constraintSolverData.rigidBodyComponents.mConstrainedLinearVelocities[dynamicsComponentIndexBody2];
    Vector3& w1 = constraintSolverData.rigidBodyComponents.mConstrainedAngularVelocities[dynamicsComponentIndexBody1];
    Vector3& w2 = constraintSolverData.rigidBodyComponents.mConstrainedAngularVelocities[dynamicsComponentIndexBody2];

    const Matrix3x3& i1 = mWorld.mSliderJointsComponents.getI1(mEntity);
    const Matrix3x3& i2 = mWorld.mSliderJointsComponents.getI2(mEntity);

    const Vector3& n1 = mWorld.mSliderJointsComponents.getN1(mEntity);
    const Vector3& n2 = mWorld.mSliderJointsComponents.getN2(mEntity);

    const Vector3& r2CrossN1 = mWorld.mSliderJointsComponents.getR2CrossN1(mEntity);
    const Vector3& r2CrossN2 = mWorld.mSliderJointsComponents.getR2CrossN2(mEntity);
    const Vector3& r1PlusUCrossN1 = mWorld.mSliderJointsComponents.getR1PlusUCrossN1(mEntity);
    const Vector3& r1PlusUCrossN2 = mWorld.mSliderJointsComponents.getR1PlusUCrossN2(mEntity);
    const Vector3& r2CrossSliderAxis = mWorld.mSliderJointsComponents.getR2CrossSliderAxis(mEntity);
    const Vector3& r1PlusUCrossSliderAxis = mWorld.mSliderJointsComponents.getR1PlusUCrossSliderAxis(mEntity);

    // Get the inverse mass and inverse inertia tensors of the bodies
    decimal inverseMassBody1 = constraintSolverData.rigidBodyComponents.getMassInverse(body1Entity);
    decimal inverseMassBody2 = constraintSolverData.rigidBodyComponents.getMassInverse(body2Entity);

    const Vector3& sliderAxisWorld = mWorld.mSliderJointsComponents.getSliderAxisWorld(mEntity);

    // --------------- Translation Constraints --------------- //

    // Compute J*v for the 2 translation constraints
    const decimal el1 = -n1.dot(v1) - w1.dot(r1PlusUCrossN1) +
                         n1.dot(v2) + w2.dot(r2CrossN1);
    const decimal el2 = -n2.dot(v1) - w1.dot(r1PlusUCrossN2) +
                         n2.dot(v2) + w2.dot(r2CrossN2);
    const Vector2 JvTranslation(el1, el2);

    // Compute the Lagrange multiplier lambda for the 2 translation constraints
    Vector2 deltaLambda = mWorld.mSliderJointsComponents.getInverseMassMatrixTranslation(mEntity) * (-JvTranslation - mWorld.mSliderJointsComponents.getBiasTranslation(mEntity));
    mWorld.mSliderJointsComponents.setImpulseTranslation(mEntity, deltaLambda + mWorld.mSliderJointsComponents.getImpulseTranslation(mEntity));

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

    // --------------- Rotation Constraints --------------- //

    // Compute J*v for the 3 rotation constraints
    const Vector3 JvRotation = w2 - w1;

    // Compute the Lagrange multiplier lambda for the 3 rotation constraints
    Vector3 deltaLambda2 = mWorld.mSliderJointsComponents.getInverseMassMatrixRotation(mEntity) *
                           (-JvRotation - mWorld.mSliderJointsComponents.getBiasRotation(mEntity));
    mWorld.mSliderJointsComponents.setImpulseRotation(mEntity, deltaLambda2 + mWorld.mSliderJointsComponents.getImpulseRotation(mEntity));

    // Compute the impulse P=J^T * lambda for the 3 rotation constraints of body 1
    angularImpulseBody1 = -deltaLambda2;

    // Apply the impulse to the body to body 1
    w1 += i1 * angularImpulseBody1;

    // Compute the impulse P=J^T * lambda for the 3 rotation constraints of body 2
    angularImpulseBody2 = deltaLambda2;

    // Apply the impulse to the body 2
    w2 += i2 * angularImpulseBody2;

    // --------------- Limits Constraints --------------- //

    if (mWorld.mSliderJointsComponents.getIsLimitEnabled(mEntity)) {

        const decimal inverseMassMatrixLimit = mWorld.mSliderJointsComponents.getInverseMassMatrixLimit(mEntity);

        // If the lower limit is violated
        if (mWorld.mSliderJointsComponents.getIsLowerLimitViolated(mEntity)) {

            // Compute J*v for the lower limit constraint
            const decimal JvLowerLimit = sliderAxisWorld.dot(v2) + r2CrossSliderAxis.dot(w2) -
                                         sliderAxisWorld.dot(v1) - r1PlusUCrossSliderAxis.dot(w1);

            // Compute the Lagrange multiplier lambda for the lower limit constraint
            decimal deltaLambdaLower = inverseMassMatrixLimit * (-JvLowerLimit - mWorld.mSliderJointsComponents.getBLowerLimit(mEntity));
            decimal lambdaTemp = mWorld.mSliderJointsComponents.getImpulseLowerLimit(mEntity);
            mWorld.mSliderJointsComponents.setImpulseLowerLimit(mEntity, std::max(mWorld.mSliderJointsComponents.getImpulseLowerLimit(mEntity) + deltaLambdaLower, decimal(0.0)));
            deltaLambdaLower = mWorld.mSliderJointsComponents.getImpulseLowerLimit(mEntity) - lambdaTemp;

            // Compute the impulse P=J^T * lambda for the lower limit constraint of body 1
            const Vector3 linearImpulseBody1 = -deltaLambdaLower * sliderAxisWorld;
            const Vector3 angularImpulseBody1 = -deltaLambdaLower * r1PlusUCrossSliderAxis;

            // Apply the impulse to the body 1
            v1 += inverseMassBody1 * linearImpulseBody1;
            w1 += i1 * angularImpulseBody1;

            // Compute the impulse P=J^T * lambda for the lower limit constraint of body 2
            const Vector3 linearImpulseBody2 = deltaLambdaLower * sliderAxisWorld;
            const Vector3 angularImpulseBody2 = deltaLambdaLower * r2CrossSliderAxis;

            // Apply the impulse to the body 2
            v2 += inverseMassBody2 * linearImpulseBody2;
            w2 += i2 * angularImpulseBody2;
        }

        // If the upper limit is violated
        if (mWorld.mSliderJointsComponents.getIsUpperLimitViolated(mEntity)) {

            // Compute J*v for the upper limit constraint
            const decimal JvUpperLimit = sliderAxisWorld.dot(v1) + r1PlusUCrossSliderAxis.dot(w1)
                                        - sliderAxisWorld.dot(v2) - r2CrossSliderAxis.dot(w2);

            // Compute the Lagrange multiplier lambda for the upper limit constraint
            decimal deltaLambdaUpper = inverseMassMatrixLimit * (-JvUpperLimit -mWorld.mSliderJointsComponents.getBUpperLimit(mEntity));
            decimal lambdaTemp = mWorld.mSliderJointsComponents.getImpulseUpperLimit(mEntity);
            mWorld.mSliderJointsComponents.setImpulseUpperLimit(mEntity, std::max(mWorld.mSliderJointsComponents.getImpulseUpperLimit(mEntity) + deltaLambdaUpper, decimal(0.0)));
            deltaLambdaUpper = mWorld.mSliderJointsComponents.getImpulseUpperLimit(mEntity) - lambdaTemp;

            // Compute the impulse P=J^T * lambda for the upper limit constraint of body 1
            const Vector3 linearImpulseBody1 = deltaLambdaUpper * sliderAxisWorld;
            const Vector3 angularImpulseBody1 = deltaLambdaUpper * r1PlusUCrossSliderAxis;

            // Apply the impulse to the body 1
            v1 += inverseMassBody1 * linearImpulseBody1;
            w1 += i1 * angularImpulseBody1;

            // Compute the impulse P=J^T * lambda for the upper limit constraint of body 2
            const Vector3 linearImpulseBody2 = -deltaLambdaUpper * sliderAxisWorld;
            const Vector3 angularImpulseBody2 = -deltaLambdaUpper * r2CrossSliderAxis;

            // Apply the impulse to the body 2
            v2 += inverseMassBody2 * linearImpulseBody2;
            w2 += i2 * angularImpulseBody2;
        }
    }

    // --------------- Motor --------------- //

    if (mWorld.mSliderJointsComponents.getIsMotorEnabled(mEntity)) {

        // Compute J*v for the motor
        const decimal JvMotor = sliderAxisWorld.dot(v1) - sliderAxisWorld.dot(v2);

        // Compute the Lagrange multiplier lambda for the motor
        const decimal maxMotorImpulse = mWorld.mSliderJointsComponents.getMaxMotorForce(mEntity) * constraintSolverData.timeStep;
        decimal deltaLambdaMotor = mWorld.mSliderJointsComponents.getInverseMassMatrixMotor(mEntity) * (-JvMotor - mWorld.mSliderJointsComponents.getMotorSpeed(mEntity));
        decimal lambdaTemp = mWorld.mSliderJointsComponents.getImpulseMotor(mEntity);
        mWorld.mSliderJointsComponents.setImpulseMotor(mEntity, clamp(mWorld.mSliderJointsComponents.getImpulseMotor(mEntity) + deltaLambdaMotor, -maxMotorImpulse, maxMotorImpulse));
        deltaLambdaMotor = mWorld.mSliderJointsComponents.getImpulseMotor(mEntity) - lambdaTemp;

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

// Solve the position constraint (for position error correction)
void SliderJoint::solvePositionConstraint(const ConstraintSolverData& constraintSolverData) {

    // If the error position correction technique is not the non-linear-gauss-seidel, we do
    // do not execute this method
    if (mWorld.mJointsComponents.getPositionCorrectionTechnique(mEntity) != JointsPositionCorrectionTechnique::NON_LINEAR_GAUSS_SEIDEL) return;

    // Get the bodies entities
    const Entity body1Entity = mWorld.mJointsComponents.getBody1Entity(mEntity);
    const Entity body2Entity = mWorld.mJointsComponents.getBody2Entity(mEntity);

    // TODO : Remove this and use compoents instead of pointers to bodies
    RigidBody* body1 = static_cast<RigidBody*>(mWorld.mRigidBodyComponents.getRigidBody(body1Entity));
    RigidBody* body2 = static_cast<RigidBody*>(mWorld.mRigidBodyComponents.getRigidBody(body2Entity));

    // Get the bodies positions and orientations
    Vector3 x1 = constraintSolverData.rigidBodyComponents.getConstrainedPosition(body1Entity);
    Vector3 x2 = constraintSolverData.rigidBodyComponents.getConstrainedPosition(body2Entity);
    Quaternion q1 = constraintSolverData.rigidBodyComponents.getConstrainedOrientation(body1Entity);
    Quaternion q2 = constraintSolverData.rigidBodyComponents.getConstrainedOrientation(body2Entity);

    // Get the inverse mass and inverse inertia tensors of the bodies
    const decimal inverseMassBody1 = constraintSolverData.rigidBodyComponents.getMassInverse(body1Entity);
    const decimal inverseMassBody2 = constraintSolverData.rigidBodyComponents.getMassInverse(body2Entity);

    // Recompute the inertia tensor of bodies
    mWorld.mSliderJointsComponents.setI1(mEntity, body1->getInertiaTensorInverseWorld());
    mWorld.mSliderJointsComponents.setI2(mEntity, body2->getInertiaTensorInverseWorld());

    // Vector from body center to the anchor point
    mWorld.mSliderJointsComponents.setR1(mEntity, q1 * mWorld.mSliderJointsComponents.getLocalAnchorPointBody1(mEntity));
    mWorld.mSliderJointsComponents.setR2(mEntity, q2 * mWorld.mSliderJointsComponents.getLocalAnchorPointBody2(mEntity));

    const Vector3& r1 = mWorld.mSliderJointsComponents.getR1(mEntity);
    const Vector3& r2 = mWorld.mSliderJointsComponents.getR2(mEntity);

    const Vector3& n1 = mWorld.mSliderJointsComponents.getN1(mEntity);
    const Vector3& n2 = mWorld.mSliderJointsComponents.getN2(mEntity);

    // Compute the vector u (difference between anchor points)
    const Vector3 u = x2 + r2 - x1 - r1;

    // Compute the two orthogonal vectors to the slider axis in world-space
    Vector3 sliderAxisWorld = q1 * mWorld.mSliderJointsComponents.getSliderAxisBody1(mEntity);
    sliderAxisWorld.normalize();
    mWorld.mSliderJointsComponents.setSliderAxisWorld(mEntity, sliderAxisWorld);
    mWorld.mSliderJointsComponents.setN1(mEntity, sliderAxisWorld.getOneUnitOrthogonalVector());
    mWorld.mSliderJointsComponents.setN2(mEntity, sliderAxisWorld.cross(n1));

    // Check if the limit constraints are violated or not
    decimal uDotSliderAxis = u.dot(sliderAxisWorld);
    decimal lowerLimitError = uDotSliderAxis - mWorld.mSliderJointsComponents.getLowerLimit(mEntity);
    decimal upperLimitError = mWorld.mSliderJointsComponents.getUpperLimit(mEntity) - uDotSliderAxis;
    mWorld.mSliderJointsComponents.setIsLowerLimitViolated(mEntity, lowerLimitError <= 0);
    mWorld.mSliderJointsComponents.setIsUpperLimitViolated(mEntity, upperLimitError <= 0);

    // Compute the cross products used in the Jacobians
    mWorld.mSliderJointsComponents.setR2CrossN1(mEntity, r2.cross(n1));
    mWorld.mSliderJointsComponents.setR2CrossN2(mEntity, r2.cross(n2));
    mWorld.mSliderJointsComponents.setR2CrossSliderAxis(mEntity, r2.cross(sliderAxisWorld));
    const Vector3 r1PlusU = r1 + u;
    mWorld.mSliderJointsComponents.setR1PlusUCrossN1(mEntity, r1PlusU.cross(n1));
    mWorld.mSliderJointsComponents.setR1PlusUCrossN2(mEntity, r1PlusU.cross(n2));
    mWorld.mSliderJointsComponents.setR1PlusUCrossSliderAxis(mEntity, r1PlusU.cross(sliderAxisWorld));

    const Vector3& r2CrossN1 = mWorld.mSliderJointsComponents.getR2CrossN1(mEntity);
    const Vector3& r2CrossN2 = mWorld.mSliderJointsComponents.getR2CrossN2(mEntity);
    const Vector3& r1PlusUCrossN1 = mWorld.mSliderJointsComponents.getR1PlusUCrossN1(mEntity);
    const Vector3& r1PlusUCrossN2 = mWorld.mSliderJointsComponents.getR1PlusUCrossN2(mEntity);
    const Vector3& r2CrossSliderAxis = mWorld.mSliderJointsComponents.getR2CrossSliderAxis(mEntity);
    const Vector3& r1PlusUCrossSliderAxis = mWorld.mSliderJointsComponents.getR1PlusUCrossSliderAxis(mEntity);

    // --------------- Translation Constraints --------------- //

    const Matrix3x3& i1 = mWorld.mSliderJointsComponents.getI1(mEntity);
    const Matrix3x3& i2 = mWorld.mSliderJointsComponents.getI2(mEntity);

    // Recompute the inverse of the mass matrix K=JM^-1J^t for the 2 translation
    // constraints (2x2 matrix)
    const decimal body1MassInverse = constraintSolverData.rigidBodyComponents.getMassInverse(body1Entity);
    const decimal body2MassInverse = constraintSolverData.rigidBodyComponents.getMassInverse(body2Entity);
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
    Matrix2x2& inverseMassMatrixTranslation = mWorld.mSliderJointsComponents.getInverseMassMatrixTranslation(mEntity);
    inverseMassMatrixTranslation.setToZero();
    if (mWorld.mRigidBodyComponents.getBodyType(body1Entity) == BodyType::DYNAMIC ||
        mWorld.mRigidBodyComponents.getBodyType(body2Entity) == BodyType::DYNAMIC) {

        mWorld.mSliderJointsComponents.setInverseMassMatrixTranslation(mEntity, matrixKTranslation.getInverse());
    }

    // Compute the position error for the 2 translation constraints
    const Vector2 translationError(u.dot(n1), u.dot(n2));

    // Compute the Lagrange multiplier lambda for the 2 translation constraints
    Vector2 lambdaTranslation = inverseMassMatrixTranslation * (-translationError);

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

    // --------------- Rotation Constraints --------------- //

    // Compute the inverse of the mass matrix K=JM^-1J^t for the 3 rotation
    // contraints (3x3 matrix)
    mWorld.mSliderJointsComponents.setInverseMassMatrixRotation(mEntity, i1 + i2);
    if (mWorld.mRigidBodyComponents.getBodyType(body1Entity) == BodyType::DYNAMIC ||
        mWorld.mRigidBodyComponents.getBodyType(body2Entity) == BodyType::DYNAMIC) {

        mWorld.mSliderJointsComponents.setInverseMassMatrixRotation(mEntity, mWorld.mSliderJointsComponents.getInverseMassMatrixRotation(mEntity).getInverse());
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
    Quaternion qError = q2 * mWorld.mSliderJointsComponents.getInitOrientationDifferenceInv(mEntity) * q1.getInverse();

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
    Vector3 lambdaRotation = mWorld.mSliderJointsComponents.getInverseMassMatrixRotation(mEntity) * (-errorRotation);

    // Compute the impulse P=J^T * lambda for the 3 rotation constraints of body 1
    angularImpulseBody1 = -lambdaRotation;

    // Apply the impulse to the body 1
    w1 = i1 * angularImpulseBody1;

    // Update the body position/orientation of body 1
    q1 += Quaternion(0, w1) * q1 * decimal(0.5);
    q1.normalize();

    // Compute the impulse P=J^T * lambda for the 3 rotation constraints of body 2
    angularImpulseBody2 = lambdaRotation;

    // Apply the impulse to the body 2
    w2 = i2 * angularImpulseBody2;

    // Update the body position/orientation of body 2
    q2 += Quaternion(0, w2) * q2 * decimal(0.5);
    q2.normalize();

    // --------------- Limits Constraints --------------- //

    if (mWorld.mSliderJointsComponents.getIsLimitEnabled(mEntity)) {

        if (mWorld.mSliderJointsComponents.getIsLowerLimitViolated(mEntity) || mWorld.mSliderJointsComponents.getIsUpperLimitViolated(mEntity)) {

            // Compute the inverse of the mass matrix K=JM^-1J^t for the limits (1x1 matrix)
            const decimal body1MassInverse = constraintSolverData.rigidBodyComponents.getMassInverse(body1Entity);
            const decimal body2MassInverse = constraintSolverData.rigidBodyComponents.getMassInverse(body2Entity);
            decimal inverseMassMatrixLimit = body1MassInverse + body2MassInverse +
                                    r1PlusUCrossSliderAxis.dot(i1 * r1PlusUCrossSliderAxis) +
                                    r2CrossSliderAxis.dot(i2 * r2CrossSliderAxis);
            inverseMassMatrixLimit = (inverseMassMatrixLimit > decimal(0.0)) ?
                                      decimal(1.0) / inverseMassMatrixLimit : decimal(0.0);
            mWorld.mSliderJointsComponents.setInverseMassMatrixLimit(mEntity, inverseMassMatrixLimit);
        }

        // If the lower limit is violated
        if (mWorld.mSliderJointsComponents.getIsLowerLimitViolated(mEntity)) {

            // Compute the Lagrange multiplier lambda for the lower limit constraint
            decimal lambdaLowerLimit = mWorld.mSliderJointsComponents.getInverseMassMatrixLimit(mEntity) * (-lowerLimitError);

            // Compute the impulse P=J^T * lambda for the lower limit constraint of body 1
            const Vector3 linearImpulseBody1 = -lambdaLowerLimit * sliderAxisWorld;
            const Vector3 angularImpulseBody1 = -lambdaLowerLimit * r1PlusUCrossSliderAxis;

            // Apply the impulse to the body 1
            const Vector3 v1 = inverseMassBody1 * linearImpulseBody1;
            const Vector3 w1 = i1 * angularImpulseBody1;

            // Update the body position/orientation of body 1
            x1 += v1;
            q1 += Quaternion(0, w1) * q1 * decimal(0.5);
            q1.normalize();

            // Compute the impulse P=J^T * lambda for the lower limit constraint of body 2
            const Vector3 linearImpulseBody2 = lambdaLowerLimit * sliderAxisWorld;
            const Vector3 angularImpulseBody2 = lambdaLowerLimit * r2CrossSliderAxis;

            // Apply the impulse to the body 2
            const Vector3 v2 = inverseMassBody2 * linearImpulseBody2;
            const Vector3 w2 = i2 * angularImpulseBody2;

            // Update the body position/orientation of body 2
            x2 += v2;
            q2 += Quaternion(0, w2) * q2 * decimal(0.5);
            q2.normalize();
        }

        // If the upper limit is violated
        if (mWorld.mSliderJointsComponents.getIsUpperLimitViolated(mEntity)) {

            // Compute the Lagrange multiplier lambda for the upper limit constraint
            decimal lambdaUpperLimit = mWorld.mSliderJointsComponents.getInverseMassMatrixLimit(mEntity) * (-upperLimitError);

            // Compute the impulse P=J^T * lambda for the upper limit constraint of body 1
            const Vector3 linearImpulseBody1 = lambdaUpperLimit * sliderAxisWorld;
            const Vector3 angularImpulseBody1 = lambdaUpperLimit * r1PlusUCrossSliderAxis;

            // Apply the impulse to the body 1
            const Vector3 v1 = inverseMassBody1 * linearImpulseBody1;
            const Vector3 w1 = i1 * angularImpulseBody1;

            // Update the body position/orientation of body 1
            x1 += v1;
            q1 += Quaternion(0, w1) * q1 * decimal(0.5);
            q1.normalize();

            // Compute the impulse P=J^T * lambda for the upper limit constraint of body 2
            const Vector3 linearImpulseBody2 = -lambdaUpperLimit * sliderAxisWorld;
            const Vector3 angularImpulseBody2 = -lambdaUpperLimit * r2CrossSliderAxis;

            // Apply the impulse to the body 2
            const Vector3 v2 = inverseMassBody2 * linearImpulseBody2;
            const Vector3 w2 = i2 * angularImpulseBody2;

            // Update the body position/orientation of body 2
            x2 += v2;
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
 * @param isLimitEnabled True if you want to enable the joint limits and false
 *                       otherwise
 */
void SliderJoint::enableLimit(bool isLimitEnabled) {

    bool isEnabled = mWorld.mSliderJointsComponents.getIsLimitEnabled(mEntity);

    if (isLimitEnabled != isEnabled) {

        mWorld.mSliderJointsComponents.setIsLimitEnabled(mEntity, isLimitEnabled);

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

    mWorld.mSliderJointsComponents.setIsMotorEnabled(mEntity, isMotorEnabled);
    mWorld.mSliderJointsComponents.setImpulseMotor(mEntity, decimal(0.0));

    // Wake up the two bodies of the joint
    awakeBodies();
}

// Return the current translation value of the joint
/**
 * @return The current translation distance of the joint (in meters)
 */
decimal SliderJoint::getTranslation() const {

    // TODO : Check if we need to compare rigid body position or center of mass here

    // Get the bodies entities
    const Entity body1Entity = mWorld.mJointsComponents.getBody1Entity(mEntity);
    const Entity body2Entity = mWorld.mJointsComponents.getBody2Entity(mEntity);

    // Get the bodies positions and orientations
    const Transform& transform1 = mWorld.mTransformComponents.getTransform(body1Entity);
    const Transform& transform2 = mWorld.mTransformComponents.getTransform(body2Entity);

    const Vector3& x1 = transform1.getPosition();
    const Vector3& x2 = transform2.getPosition();
    const Quaternion& q1 = transform1.getOrientation();
    const Quaternion& q2 = transform2.getOrientation();

    // Compute the two anchor points in world-space coordinates
    const Vector3 anchorBody1 = x1 + q1 * mWorld.mSliderJointsComponents.getLocalAnchorPointBody1(mEntity);
    const Vector3 anchorBody2 = x2 + q2 * mWorld.mSliderJointsComponents.getLocalAnchorPointBody2(mEntity);

    // Compute the vector u (difference between anchor points)
    const Vector3 u = anchorBody2 - anchorBody1;

    // Compute the slider axis in world-space
    Vector3 sliderAxisWorld = q1 * mWorld.mSliderJointsComponents.getSliderAxisBody1(mEntity);
    sliderAxisWorld.normalize();

    // Compute and return the translation value
    return u.dot(sliderAxisWorld);
}

// Set the minimum translation limit
/**
 * @param lowerLimit The minimum translation limit of the joint (in meters)
 */
void SliderJoint::setMinTranslationLimit(decimal lowerLimit) {

    assert(lowerLimit <= mWorld.mSliderJointsComponents.getUpperLimit(mEntity));

    if (lowerLimit != mWorld.mSliderJointsComponents.getLowerLimit(mEntity)) {

        mWorld.mSliderJointsComponents.setLowerLimit(mEntity, lowerLimit);

        // Reset the limits
        resetLimits();
    }
}

// Set the maximum translation limit
/**
 * @param lowerLimit The maximum translation limit of the joint (in meters)
 */
void SliderJoint::setMaxTranslationLimit(decimal upperLimit) {

    assert(mWorld.mSliderJointsComponents.getLowerLimit(mEntity) <= upperLimit);

    if (upperLimit != mWorld.mSliderJointsComponents.getUpperLimit(mEntity)) {

        mWorld.mSliderJointsComponents.setUpperLimit(mEntity, upperLimit);

        // Reset the limits
        resetLimits();
    }
}

// Reset the limits
void SliderJoint::resetLimits() {

    // Reset the accumulated impulses for the limits
    mWorld.mSliderJointsComponents.setImpulseLowerLimit(mEntity, decimal(0.0));
    mWorld.mSliderJointsComponents.setImpulseUpperLimit(mEntity, decimal(0.0));

    // Wake up the two bodies of the joint
    awakeBodies();
}

// Set the motor speed
/**
 * @param motorSpeed The speed of the joint motor (in meters per second)
 */
void SliderJoint::setMotorSpeed(decimal motorSpeed) {

    if (motorSpeed != mWorld.mSliderJointsComponents.getMotorSpeed(mEntity)) {

        mWorld.mSliderJointsComponents.setMotorSpeed(mEntity, motorSpeed);

        // Wake up the two bodies of the joint
        awakeBodies();
    }
}

// Set the maximum motor force
/**
 * @param maxMotorForce The maximum force of the joint motor (in Newton x meters)
 */
void SliderJoint::setMaxMotorForce(decimal maxMotorForce) {

    const decimal maxForce = mWorld.mSliderJointsComponents.getMaxMotorForce(mEntity);

    if (maxMotorForce != maxForce) {

        assert(maxForce >= decimal(0.0));
        mWorld.mSliderJointsComponents.setMaxMotorForce(mEntity, maxMotorForce);

        // Wake up the two bodies of the joint
        awakeBodies();
    }
}

// Return the intensity of the current force applied for the joint motor
/**
 * @param timeStep Time step (in seconds)
 * @return The current force of the joint motor (in Newton x meters)
 */
decimal SliderJoint::getMotorForce(decimal timeStep) const {
    return mWorld.mSliderJointsComponents.getImpulseMotor(mEntity) / timeStep;
}

// Return true if the limits or the joint are enabled
/**
 * @return True if the joint limits are enabled
 */
bool SliderJoint::isLimitEnabled() const {

    return mWorld.mSliderJointsComponents.getIsLimitEnabled(mEntity);
}

// Return true if the motor of the joint is enabled
/**
 * @return True if the joint motor is enabled
 */
bool SliderJoint::isMotorEnabled() const {
    return mWorld.mSliderJointsComponents.getIsMotorEnabled(mEntity);
}

// Return the minimum translation limit
/**
 * @return The minimum translation limit of the joint (in meters)
 */
decimal SliderJoint::getMinTranslationLimit() const {
    return mWorld.mSliderJointsComponents.getLowerLimit(mEntity);
}

// Return the motor speed
/**
 * @return The current motor speed of the joint (in meters per second)
 */
decimal SliderJoint::getMotorSpeed() const {
    return mWorld.mSliderJointsComponents.getMotorSpeed(mEntity);
}

// Return the maximum motor force
/**
 * @return The maximum force of the joint motor (in Newton x meters)
 */
decimal SliderJoint::getMaxMotorForce() const {
    return mWorld.mSliderJointsComponents.getMaxMotorForce(mEntity);
}

// Return the maximum translation limit
/**
 * @return The maximum translation limit of the joint (in meters)
 */
decimal SliderJoint::getMaxTranslationLimit() const {
    return mWorld.mSliderJointsComponents.getUpperLimit(mEntity);
}
// Return a string representation
std::string SliderJoint::to_string() const {
    return "SliderJoint{ lowerLimit=" + std::to_string(mWorld.mSliderJointsComponents.getLowerLimit(mEntity)) + ", upperLimit=" + std::to_string(mWorld.mSliderJointsComponents.getUpperLimit(mEntity)) +
            "localAnchorPointBody1=" + mWorld.mSliderJointsComponents.getLocalAnchorPointBody1(mEntity).to_string() + ", localAnchorPointBody2=" +
            mWorld.mSliderJointsComponents.getLocalAnchorPointBody2(mEntity).to_string() + ", sliderAxisBody1=" + mWorld.mSliderJointsComponents.getSliderAxisBody1(mEntity).to_string() +
            ", initOrientationDifferenceInv=" +
            mWorld.mSliderJointsComponents.getInitOrientationDifferenceInv(mEntity).to_string() + ", motorSpeed=" + std::to_string(getMotorSpeed()) +
            ", maxMotorForce=" + std::to_string(getMaxMotorForce()) + ", isLimitEnabled=" +
            (mWorld.mSliderJointsComponents.getIsLimitEnabled(mEntity) ? "true" : "false") + ", isMotorEnabled=" + (mWorld.mSliderJointsComponents.getIsMotorEnabled(mEntity) ? "true" : "false") + "}";
}
