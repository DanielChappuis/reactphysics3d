/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2010-2013 Daniel Chappuis                                       *
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

using namespace reactphysics3d;

// Constructor
SliderJoint::SliderJoint(const SliderJointInfo& jointInfo)
            : Constraint(jointInfo), mImpulseTranslation(0, 0), mImpulseRotation(0, 0, 0) {

    // Compute the local-space anchor point for each body
    mLocalAnchorPointBody1 = mBody1->getTransform().getInverse() * jointInfo.anchorPointWorldSpace;
    mLocalAnchorPointBody2 = mBody2->getTransform().getInverse() * jointInfo.anchorPointWorldSpace;

    // Compute the slider axis in local-space of body 1
    mSliderAxisBody1 = mBody1->getTransform().getOrientation().getInverse() *
                       jointInfo.sliderAxisWorldSpace;
    mSliderAxisBody1.normalize();
}

// Destructor
SliderJoint::~SliderJoint() {

}

// Initialize before solving the constraint
void SliderJoint::initBeforeSolve(const ConstraintSolverData& constraintSolverData) {

    // Initialize the bodies index in the velocity array
    mIndexBody1 = constraintSolverData.mapBodyToConstrainedVelocityIndex.find(mBody1)->second;
    mIndexBody2 = constraintSolverData.mapBodyToConstrainedVelocityIndex.find(mBody2)->second;

    // Get the body positions
    const Vector3& x1 = mBody1->getTransform().getPosition();
    const Vector3& x2 = mBody2->getTransform().getPosition();

    // Get the bodies positions and orientations
    const Quaternion& orientationBody1 = mBody1->getTransform().getOrientation();
    const Quaternion& orientationBody2 = mBody2->getTransform().getOrientation();

    // Get the inertia tensor of bodies
    const Matrix3x3 I1 = mBody1->getInertiaTensorInverseWorld();
    const Matrix3x3 I2 = mBody2->getInertiaTensorInverseWorld();

    // Vector from body center to the anchor point
    mR1 = orientationBody1 * mLocalAnchorPointBody1;
    mR2 = orientationBody2 * mLocalAnchorPointBody2;

    // Compute the vector u
    const Vector3 u = x2 + mR2 - x1 - mR1;

    // Compute the two orthogonal vectors to the slider axis in local-space of body 1
    Vector3 sliderAxisWorld = orientationBody1 * mSliderAxisBody1;
    sliderAxisWorld.normalize();
    mN1 = sliderAxisWorld.getOneUnitOrthogonalVector();
    mN2 = sliderAxisWorld.cross(mN1);

    // Compute the cross products used in the Jacobian
    mR2CrossN1 = mR2.cross(mN1);
    mR2CrossN2 = mR2.cross(mN2);
    const Vector3 r1PlusU = mR1 + u;
    mR1PlusUCrossN1 = (r1PlusU).cross(mN1);
    mR1PlusUCrossN2 = (r1PlusU).cross(mN2);

    // Compute the inverse of the mass matrix K=JM^-1J^t for the 2 translation
    // constraints (2x2 matrix)
    const decimal n1Dotn1 = mN1.lengthSquare();
    const decimal n2Dotn2 = mN2.lengthSquare();
    const decimal n1Dotn2 = mN1.dot(mN2);
    decimal sumInverseMass = 0.0;
    Vector3 I1R1PlusUCrossN1(0, 0, 0);
    Vector3 I1R1PlusUCrossN2(0, 0, 0);
    Vector3 I2R2CrossN1(0, 0, 0);
    Vector3 I2R2CrossN2(0, 0, 0);
    if (mBody1->getIsMotionEnabled()) {
        sumInverseMass += mBody1->getMassInverse();
        I1R1PlusUCrossN1 = I1 * mR1PlusUCrossN1;
        I1R1PlusUCrossN2 = I1 * mR1PlusUCrossN2;
    }
    if (mBody2->getIsMotionEnabled()) {
        sumInverseMass += mBody2->getMassInverse();
        I2R2CrossN1 = I2 * mR2CrossN1;
        I2R2CrossN2 = I2 * mR2CrossN2;
    }
    const decimal el11 = sumInverseMass * (n1Dotn1) + mR1PlusUCrossN1.dot(I1R1PlusUCrossN1) +
                         mR2CrossN1.dot(I2R2CrossN1);
    const decimal el12 = sumInverseMass * (n1Dotn2) + mR1PlusUCrossN1.dot(I1R1PlusUCrossN2) +
                         mR2CrossN1.dot(I2R2CrossN2);
    const decimal el21 = sumInverseMass * (n1Dotn2) + mR1PlusUCrossN2.dot(I1R1PlusUCrossN1) +
                         mR2CrossN2.dot(I2R2CrossN1);
    const decimal el22 = sumInverseMass * (n2Dotn2) + mR1PlusUCrossN2.dot(I1R1PlusUCrossN2) +
                         mR2CrossN2.dot(I2R2CrossN2);
    Matrix2x2 matrixKTranslation(el11, el12, el21, el22);
    mInverseMassMatrixTranslationConstraint.setToZero();
    if (mBody1->getIsMotionEnabled() || mBody2->getIsMotionEnabled()) {
        mInverseMassMatrixTranslationConstraint = matrixKTranslation.getInverse();
    }

    // Compute the inverse of the mass matrix K=JM^-1J^t for the 3 rotation
    // contraints (3x3 matrix)
    mInverseMassMatrixRotationConstraint.setToZero();
    if (mBody1->getIsMotionEnabled()) {
        mInverseMassMatrixRotationConstraint += I1;
    }
    if (mBody2->getIsMotionEnabled()) {
        mInverseMassMatrixRotationConstraint += I2;
    }
    if (mBody1->getIsMotionEnabled() || mBody2->getIsMotionEnabled()) {
        mInverseMassMatrixRotationConstraint = mInverseMassMatrixRotationConstraint.getInverse();
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
    const decimal inverseMassBody1 = mBody1->getMassInverse();
    const decimal inverseMassBody2 = mBody2->getMassInverse();
    const Matrix3x3 I1 = mBody1->getInertiaTensorInverseWorld();
    const Matrix3x3 I2 = mBody2->getInertiaTensorInverseWorld();

    // Compute the impulse P=J^T * lambda for the 2 translation constraints
    Vector3 linearImpulseBody1 = -mN1 * mImpulseTranslation.x - mN2 * mImpulseTranslation.y;
    Vector3 angularImpulseBody1 = -mR1PlusUCrossN1 * mImpulseTranslation.x -
                                   mR1PlusUCrossN2 * mImpulseTranslation.y;
    Vector3 linearImpulseBody2 = -linearImpulseBody1;
    Vector3 angularImpulseBody2 = mR2CrossN1 * mImpulseTranslation.x +
                                  mR2CrossN2 * mImpulseTranslation.y;

    // Compute the impulse P=J^T * lambda for the 3 rotation constraints
    angularImpulseBody1 += -mImpulseRotation;
    angularImpulseBody2 += mImpulseRotation;

    // Apply the impulse to the bodies of the joint
    if (mBody1->getIsMotionEnabled()) {
        v1 += inverseMassBody1 * linearImpulseBody1;
        w1 += I1 * angularImpulseBody1;
    }
    if (mBody2->getIsMotionEnabled()) {
        v2 += inverseMassBody2 * linearImpulseBody2;
        w2 += I2 * angularImpulseBody2;
    }
}

// Solve the velocity constraint
void SliderJoint::solveVelocityConstraint(const ConstraintSolverData& constraintSolverData) {

    // Get the body positions
    const Vector3& x1 = mBody1->getTransform().getPosition();
    const Vector3& x2 = mBody2->getTransform().getPosition();

    // Get the velocities
    Vector3& v1 = constraintSolverData.linearVelocities[mIndexBody1];
    Vector3& v2 = constraintSolverData.linearVelocities[mIndexBody2];
    Vector3& w1 = constraintSolverData.angularVelocities[mIndexBody1];
    Vector3& w2 = constraintSolverData.angularVelocities[mIndexBody2];

    // Get the inverse mass and inverse inertia tensors of the bodies
    decimal inverseMassBody1 = mBody1->getMassInverse();
    decimal inverseMassBody2 = mBody2->getMassInverse();
    const Matrix3x3 I1 = mBody1->getInertiaTensorInverseWorld();
    const Matrix3x3 I2 = mBody2->getInertiaTensorInverseWorld();

    // --------------- Translation Constraints --------------- //

    // Compute J*v for the 2 translation constraints
    const decimal el1 = -mN1.dot(v1) - w1.dot(mR1PlusUCrossN1) +
                         mN1.dot(v2) + w2.dot(mR2CrossN1);
    const decimal el2 = -mN2.dot(v1) - w1.dot(mR1PlusUCrossN2) +
                         mN2.dot(v2) + w2.dot(mR2CrossN2);
    const Vector2 JvTranslation(el1, el2);

    // Compute the bias "b" of the translation constraint
    Vector2 bTranslation(0, 0);
    decimal beta = decimal(0.2);     // TODO : Use a constant here
    decimal biasFactor = (beta / constraintSolverData.timeStep);
    if (mPositionCorrectionTechnique == BAUMGARTE_JOINTS) {
        Vector3 deltaV = x2 + mR2 - x1 - mR1;
        bTranslation.x = biasFactor * deltaV.dot(mN1);
        bTranslation.y = biasFactor * deltaV.dot(mN2);
    }

    // Compute the Lagrange multiplier lambda for the 2 translation constraints
    Vector2 deltaLambda = mInverseMassMatrixTranslationConstraint * (-JvTranslation - bTranslation);
    mImpulseTranslation += deltaLambda;

    // Compute the impulse P=J^T * lambda for the 2 translation constraints
    Vector3 linearImpulseBody1 = -mN1 * deltaLambda.x - mN2 * deltaLambda.y;
    Vector3 angularImpulseBody1 = -mR1PlusUCrossN1 * deltaLambda.x -mR1PlusUCrossN2 * deltaLambda.y;
    Vector3 linearImpulseBody2 = -linearImpulseBody1;
    Vector3 angularImpulseBody2 = mR2CrossN1 * deltaLambda.x + mR2CrossN2 * deltaLambda.y;

    // Apply the impulse to the bodies of the joint
    if (mBody1->getIsMotionEnabled()) {
        v1 += inverseMassBody1 * linearImpulseBody1;
        w1 += I1 * angularImpulseBody1;
    }
    if (mBody2->getIsMotionEnabled()) {
        v2 += inverseMassBody2 * linearImpulseBody2;
        w2 += I2 * angularImpulseBody2;
    }

    // --------------- Rotation Constraints --------------- //

    // Compute J*v for the 3 rotation constraints
    const Vector3 JvRotation = w2 - w1;

    // Compute the bias "b" of the rotation constraint
    Vector3 bRotation(0, 0, 0);
    if (mPositionCorrectionTechnique == BAUMGARTE_JOINTS) {
        Quaternion q1 = mBody1->getTransform().getOrientation();
        Quaternion q2 = mBody2->getTransform().getOrientation();
        Quaternion qDiff = q1 * q2.getInverse();
        bRotation = 2.0 * qDiff.getVectorV();
    }

    // Compute the Lagrange multiplier lambda for the 3 rotation constraints
    Vector3 deltaLambda2 = mInverseMassMatrixRotationConstraint * (-JvRotation - bRotation);
    mImpulseRotation += deltaLambda2;

    // Compute the impulse P=J^T * lambda for the 3 rotation constraints
    angularImpulseBody1 = -deltaLambda2;
    angularImpulseBody2 = deltaLambda2;

    // Apply the impulse to the bodies of the joint
    if (mBody1->getIsMotionEnabled()) {
        w1 += I1 * angularImpulseBody1;
    }
    if (mBody2->getIsMotionEnabled()) {
        w2 += I2 * angularImpulseBody2;
    }
}

// Solve the position constraint
void SliderJoint::solvePositionConstraint(const ConstraintSolverData& constraintSolverData) {

}
