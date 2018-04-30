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
#include "ConstraintSolver.h"
#include "utils/Profiler.h"
#include "engine/Island.h"

using namespace reactphysics3d;

// Constructor
ConstraintSolver::ConstraintSolver() : mIsWarmStartingActive(true) {

#ifdef IS_PROFILING_ACTIVE

	mProfiler = nullptr;

#endif

}

// Initialize the constraint solver for a given island
void ConstraintSolver::initializeForIsland(decimal dt, Island* island) {

    RP3D_PROFILE("ConstraintSolver::initializeForIsland()", mProfiler);

    assert(island != nullptr);
    assert(island->getNbBodies() > 0);
    assert(island->getNbJoints() > 0);

    // Set the current time step
    mTimeStep = dt;

    // Initialize the constraint solver data used to initialize and solve the constraints
    mConstraintSolverData.timeStep = mTimeStep;
    mConstraintSolverData.isWarmStartingActive = mIsWarmStartingActive;

    // For each joint of the island
    Joint** joints = island->getJoints();
    for (uint i=0; i<island->getNbJoints(); i++) {

        // Initialize the constraint before solving it
        joints[i]->initBeforeSolve(mConstraintSolverData);

        // Warm-start the constraint if warm-starting is enabled
        if (mIsWarmStartingActive) {
            joints[i]->warmstart(mConstraintSolverData);
        }
    }
}

// Solve the velocity constraints
void ConstraintSolver::solveVelocityConstraints(Island* island) {

    RP3D_PROFILE("ConstraintSolver::solveVelocityConstraints()", mProfiler);

    assert(island != nullptr);
    assert(island->getNbJoints() > 0);

    // For each joint of the island
    Joint** joints = island->getJoints();
    for (uint i=0; i<island->getNbJoints(); i++) {

        // Solve the constraint
        joints[i]->solveVelocityConstraint(mConstraintSolverData);
    }
}

// Solve the position constraints
void ConstraintSolver::solvePositionConstraints(Island* island) {

    RP3D_PROFILE("ConstraintSolver::solvePositionConstraints()", mProfiler);

    assert(island != nullptr);
    assert(island->getNbJoints() > 0);

    // For each joint of the island
    Joint** joints = island->getJoints();
    for (uint i=0; i < island->getNbJoints(); i++) {

        // Solve the constraint
        joints[i]->solvePositionConstraint(mConstraintSolverData);
    }
}
