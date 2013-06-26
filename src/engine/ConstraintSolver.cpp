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
#include "ConstraintSolver.h"
#include "Profiler.h"

using namespace reactphysics3d;

// Constructor
ConstraintSolver::ConstraintSolver(std::set<Constraint*>& joints,
                                   std::vector<Vector3>& linearVelocities,
                                   std::vector<Vector3>& angularVelocities,
                                   std::vector<Vector3>& positions,
                                   std::vector<Quaternion>& orientations,
                                   const std::map<RigidBody*, uint>& mapBodyToVelocityIndex)
                 : mJoints(joints), mLinearVelocities(linearVelocities),
                   mAngularVelocities(angularVelocities), mPositions(positions),
                   mOrientations(orientations),
                   mMapBodyToConstrainedVelocityIndex(mapBodyToVelocityIndex),
                   mIsWarmStartingActive(true), mConstraintSolverData(linearVelocities,
                   angularVelocities, positions, orientations, mapBodyToVelocityIndex){

}

// Destructor
ConstraintSolver::~ConstraintSolver() {

}

// Initialize the constraint solver
void ConstraintSolver::initialize(decimal dt) {

    PROFILE("ConstraintSolver::initialize()");

    // Set the current time step
    mTimeStep = dt;

    // Initialize the constraint solver data used to initialize and solve the constraints
    mConstraintSolverData.timeStep = mTimeStep;
    mConstraintSolverData.isWarmStartingActive = mIsWarmStartingActive;

    // For each joint
    std::set<Constraint*>::iterator it;
    for (it = mJoints.begin(); it != mJoints.end(); ++it) {

        Constraint* joint = (*it);

        // Get the rigid bodies of the joint
        RigidBody* body1 = joint->getBody1();
        RigidBody* body2 = joint->getBody2();

        // Add the bodies to the set of constrained bodies
        mConstraintBodies.insert(body1);
        mConstraintBodies.insert(body2);

        // Initialize the constraint before solving it
        joint->initBeforeSolve(mConstraintSolverData);

        // Warm-start the constraint if warm-starting is enabled
        if (mIsWarmStartingActive) {
            joint->warmstart(mConstraintSolverData);
        }
    }
}

// Solve the velocity constraints
void ConstraintSolver::solveVelocityConstraints() {

    PROFILE("ConstraintSolver::solveVelocityConstraints()");

    // For each joint
    std::set<Constraint*>::iterator it;
    for (it = mJoints.begin(); it != mJoints.end(); ++it) {

        // Solve the constraint
        (*it)->solveVelocityConstraint(mConstraintSolverData);
    }
}

// Solve the position constraints
void ConstraintSolver::solvePositionConstraints() {

    PROFILE("ConstraintSolver::solvePositionConstraints()");

    // For each joint
    std::set<Constraint*>::iterator it;
    for (it = mJoints.begin(); it != mJoints.end(); ++it) {

        // Solve the constraint
        (*it)->solvePositionConstraint(mConstraintSolverData);
    }
}
