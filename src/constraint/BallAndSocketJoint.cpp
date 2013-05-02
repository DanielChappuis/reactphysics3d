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
#include "BallAndSocketJoint.h"
#include "../engine/ConstraintSolver.h"

using namespace reactphysics3d;

// Constructor
BallAndSocketJoint::BallAndSocketJoint(const BallAndSocketJointInfo &jointInfo)
                   : Constraint(jointInfo), mImpulse(Vector3(0, 0, 0)) {

    // Compute the local-space anchor point for each body
    mLocalAnchorPointBody1 = mBody1->getTransform().getInverse() * jointInfo.anchorPointWorldSpace;
    mLocalAnchorPointBody1 = mBody1->getTransform().getInverse() * jointInfo.anchorPointWorldSpace;
}

// Destructor
BallAndSocketJoint::~BallAndSocketJoint() {

}

// Initialize before solving the constraint
void BallAndSocketJoint::initBeforeSolve(const ConstraintSolverData& constraintSolverData) {

    // Initialize the bodies index in the velocity array
    mIndexBody1 = constraintSolverData.mapBodyToConstrainedVelocityIndex.find(mBody1)->second;
    mIndexBody2 = constraintSolverData.mapBodyToConstrainedVelocityIndex.find(mBody2)->second;

    // Get the bodies positions and orientations
    const Vector3& x1 = mBody1->getTransform().getPosition();
    const Vector3& x2 = mBody2->getTransform().getPosition();
    const Quaternion& orientationBody1 = mBody1->getTransform().getOrientation();
    const Quaternion& orientationBody2 = mBody2->getTransform().getOrientation();

    // Get the inertia tensor of bodies
    Matrix3x3 inverseInertiaTensorBody1 = mBody1->getInertiaTensorInverseWorld();
    Matrix3x3 inverseInertiaTensorBody2 = mBody2->getInertiaTensorInverseWorld();

    // Compute the vector from body center to anchor point in local-space
    const Vector3 u1Local = mLocalAnchorPointBody1 - x1;
    const Vector3 u2Local = mLocalAnchorPointBody2 - x2;

    // Compute the vector from body center to the anchor point in world-space
    mU1World = orientationBody1 * u1Local;
    mU2World = orientationBody2 * u2Local;

    // Compute the corresponding skew-symmetric matrices
    Matrix3x3 skewSymmetricMatrixU1= Matrix3x3::computeSkewSymmetricMatrixForCrossProduct(mU1World);
    Matrix3x3 skewSymmetricMatrixU2= Matrix3x3::computeSkewSymmetricMatrixForCrossProduct(mU2World);

    // Compute the matrix JM^-1J^t
    decimal inverseMassBodies = mBody1->getMassInverse() + mBody2->getMassInverse();
    Matrix3x3 massMatrix= Matrix3x3(inverseMassBodies, 0, 0,
                                    0, inverseMassBodies, 0,
                                    0, 0, inverseMassBodies) +
                          skewSymmetricMatrixU1 * inverseInertiaTensorBody1 * skewSymmetricMatrixU1+
                          skewSymmetricMatrixU2 * inverseInertiaTensorBody2 * skewSymmetricMatrixU2;

    // Compute the inverse mass matrix K
    mInverseMassMatrix = massMatrix.getInverse();
}

// Solve the constraint
void BallAndSocketJoint::solve(const ConstraintSolverData& constraintSolverData) {

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
    Vector3 Jv = -v1 + mU1World.cross(w1) + v2 - mU2World.cross(w2);

    // Compute the bias "b" of the constraint
    decimal beta = 0.8;     // TODO : Use a constant here
    decimal biasFactor = -(beta/constraintSolverData.timeStep);
    Vector3 b = biasFactor * (x2 + mU2World - x1 - mU1World);

    // Compute the Lagrange multiplier lambda
    Vector3 deltaLambda = mInverseMassMatrix * (-Jv - b);
    mImpulse = mImpulse + deltaLambda;

    // Compute the impulse P=J^T * lambda
    Vector3 linearImpulseBody1 = -deltaLambda;
    Vector3 angularImpulseBody1 = mU1World.cross(deltaLambda);
    Vector3 linearImpulseBody2 = deltaLambda;
    Vector3 angularImpulseBody2 = -mU2World.cross(deltaLambda);

    // Apply the impulse to the bodies of the joint
    if (mBody1->getIsMotionEnabled()) {
        v1 += inverseMassBody1 * linearImpulseBody1;
        w1 += inverseInertiaTensorBody1 * angularImpulseBody1;
    }
    if (mBody2->getIsMotionEnabled()) {
        v2 += inverseMassBody2 * linearImpulseBody2;
        w2 += inverseInertiaTensorBody2 * angularImpulseBody2;
    }
}

