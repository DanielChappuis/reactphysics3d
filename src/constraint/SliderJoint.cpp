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
SliderJoint::SliderJoint(const SliderJointInfo& jointInfo) : Constraint(jointInfo) {

    // Compute the local-space anchor point for each body
    mLocalAnchorPointBody1 = mBody1->getTransform().getInverse() * jointInfo.anchorPointWorldSpace;
    mLocalAnchorPointBody2 = mBody2->getTransform().getInverse() * jointInfo.anchorPointWorldSpace;
}

// Destructor
SliderJoint::~SliderJoint() {

}

// Initialize before solving the constraint
void SliderJoint::initBeforeSolve(const ConstraintSolverData& constraintSolverData) {

    // Initialize the bodies index in the velocity array
    mIndexBody1 = constraintSolverData.mapBodyToConstrainedVelocityIndex.find(mBody1)->second;
    mIndexBody2 = constraintSolverData.mapBodyToConstrainedVelocityIndex.find(mBody2)->second;

    // Get the bodies positions and orientations
    const Quaternion& orientationBody1 = mBody1->getTransform().getOrientation();
    const Quaternion& orientationBody2 = mBody2->getTransform().getOrientation();

    // Get the inertia tensor of bodies
    const Matrix3x3 I1 = mBody1->getInertiaTensorInverseWorld();
    const Matrix3x3 I2 = mBody2->getInertiaTensorInverseWorld();

    // Compute the vector from body center to the anchor point in world-space
    mU1World = orientationBody1 * mLocalAnchorPointBody1;
    mU2World = orientationBody2 * mLocalAnchorPointBody2;

    // Compute the two orthogonal vectors to vector mU1World in world-space
    mN1 = mU1World.getOneUnitOrthogonalVector();
    mN2 = mU1World.cross(mN1);

    // Compute the cross product used in the Jacobian
    mU1WorldCrossN1 = mN2;
    mU1WorldCrossN2 = mU1World.cross(mN2);
    mU2WorldCrossN1 = mU2World.cross(mN1);
    mU2WorldCrossN2 = mU2World.cross(mN2);

    // Compute the mass matrix K=JM^-1J^t for the 2 translation constraints (2x2 matrix)
    const decimal n1Dotn1 = mN1.lengthSquare();
    const decimal n2Dotn2 = mN2.lengthSquare();
    const decimal sumInverseMass = mBody1->getMassInverse() + mBody2->getMassInverse();

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
    Matrix3x3 inverseInertiaTensorBody1 = mBody1->getInertiaTensorInverseWorld();
    Matrix3x3 inverseInertiaTensorBody2 = mBody2->getInertiaTensorInverseWorld();

    // Compute J*v
}

// Solve the position constraint
void SliderJoint::solvePositionConstraint(const ConstraintSolverData& constraintSolverData) {

}
