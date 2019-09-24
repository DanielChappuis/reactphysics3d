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
#include "systems/ConstraintSolverSystem.h"
#include "utils/Profiler.h"
#include "engine/Island.h"

using namespace reactphysics3d;

// Constructor
ConstraintSolverSystem::ConstraintSolverSystem(Islands& islands, RigidBodyComponents& rigidBodyComponents)
                 : mIsWarmStartingActive(true), mIslands(islands), mConstraintSolverData(rigidBodyComponents),
                   mSolveBallAndSocketJointSystem(rigidBodyComponents) {

#ifdef IS_PROFILING_ACTIVE

	mProfiler = nullptr;

#endif

}

// Initialize the constraint solver for a given island
void ConstraintSolverSystem::initializeForIsland(decimal dt, uint islandIndex) {

    RP3D_PROFILE("ConstraintSolverSystem::initializeForIsland()", mProfiler);

    assert(mIslands.bodyEntities[islandIndex].size() > 0);
    assert(mIslands.joints[islandIndex].size() > 0);

    // Set the current time step
    mTimeStep = dt;

    // Initialize the constraint solver data used to initialize and solve the constraints
    mConstraintSolverData.timeStep = mTimeStep;
    mConstraintSolverData.isWarmStartingActive = mIsWarmStartingActive;

    // For each joint of the island
    for (uint i=0; i<mIslands.joints[islandIndex].size(); i++) {

        // Initialize the constraint before solving it
        mIslands.joints[islandIndex][i]->initBeforeSolve(mConstraintSolverData);

        // Warm-start the constraint if warm-starting is enabled
        if (mIsWarmStartingActive) {
            mIslands.joints[islandIndex][i]->warmstart(mConstraintSolverData);
        }
    }
}

// Solve the velocity constraints
void ConstraintSolverSystem::solveVelocityConstraints(uint islandIndex) {

    RP3D_PROFILE("ConstraintSolverSystem::solveVelocityConstraints()", mProfiler);

    assert(mIslands.joints[islandIndex].size() > 0);

    // For each joint of the island
    for (uint i=0; i<mIslands.joints[islandIndex].size(); i++) {

        // Solve the constraint
        mIslands.joints[islandIndex][i]->solveVelocityConstraint(mConstraintSolverData);
    }
}

// Solve the position constraints
void ConstraintSolverSystem::solvePositionConstraints(uint islandIndex) {

    RP3D_PROFILE("ConstraintSolverSystem::solvePositionConstraints()", mProfiler);

    assert(mIslands.joints[islandIndex].size() > 0);

    // For each joint of the island
    for (uint i=0; i<mIslands.joints[islandIndex].size(); i++) {

        // Solve the constraint
        mIslands.joints[islandIndex][i]->solvePositionConstraint(mConstraintSolverData);
    }
}
