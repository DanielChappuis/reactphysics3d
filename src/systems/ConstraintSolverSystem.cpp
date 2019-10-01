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
#include "components/JointComponents.h"
#include "components/BallAndSocketJointComponents.h"
#include "utils/Profiler.h"
#include "engine/Island.h"

using namespace reactphysics3d;

// Constructor
ConstraintSolverSystem::ConstraintSolverSystem(Islands& islands, RigidBodyComponents& rigidBodyComponents,
                                               TransformComponents& transformComponents,
                                               JointComponents& jointComponents,
                                               BallAndSocketJointComponents& ballAndSocketJointComponents)
                 : mIsWarmStartingActive(true), mIslands(islands),
                   mConstraintSolverData(rigidBodyComponents, jointComponents),
                   mSolveBallAndSocketJointSystem(rigidBodyComponents, transformComponents, jointComponents, ballAndSocketJointComponents),
                   mJointComponents(jointComponents), mBallAndSocketJointComponents(ballAndSocketJointComponents){

#ifdef IS_PROFILING_ACTIVE

	mProfiler = nullptr;

#endif

}

// Initialize the constraint solver
void ConstraintSolverSystem::initialize(decimal dt) {

    RP3D_PROFILE("ConstraintSolverSystem::initialize()", mProfiler);

    // Set the current time step
    mTimeStep = dt;

    // Initialize the constraint solver data used to initialize and solve the constraints
    mConstraintSolverData.timeStep = mTimeStep;
    mConstraintSolverData.isWarmStartingActive = mIsWarmStartingActive;

    mSolveBallAndSocketJointSystem.setTimeStep(dt);
    mSolveBallAndSocketJointSystem.setIsWarmStartingActive(mIsWarmStartingActive);

    mSolveBallAndSocketJointSystem.initBeforeSolve();

    if (mIsWarmStartingActive) {
        mSolveBallAndSocketJointSystem.warmstart();
    }

    // For each joint
    for (uint i=0; i<mConstraintSolverData.jointComponents.getNbEnabledComponents(); i++) {

        // TODO : DELETE THIS
        Entity jointEntity = mConstraintSolverData.jointComponents.mJointEntities[i];
        if (mBallAndSocketJointComponents.hasComponent(jointEntity)) {
           continue;
        }

        const Entity body1 = mConstraintSolverData.jointComponents.mBody1Entities[i];
        const Entity body2 = mConstraintSolverData.jointComponents.mBody2Entities[i];
        assert(!mConstraintSolverData.rigidBodyComponents.getIsEntityDisabled(body1));
        assert(!mConstraintSolverData.rigidBodyComponents.getIsEntityDisabled(body2));

        // Initialize the constraint before solving it
        mJointComponents.mJoints[i]->initBeforeSolve(mConstraintSolverData);

        // Warm-start the constraint if warm-starting is enabled
        if (mIsWarmStartingActive) {
            mConstraintSolverData.jointComponents.mJoints[i]->warmstart(mConstraintSolverData);
        }
    }
}

// Solve the velocity constraints
void ConstraintSolverSystem::solveVelocityConstraints() {

    RP3D_PROFILE("ConstraintSolverSystem::solveVelocityConstraints()", mProfiler);

    mSolveBallAndSocketJointSystem.solveVelocityConstraint();

    // For each joint
    for (uint i=0; i<mConstraintSolverData.jointComponents.getNbEnabledComponents(); i++) {

        // TODO : DELETE THIS
        Entity jointEntity = mConstraintSolverData.jointComponents.mJointEntities[i];
        if (mBallAndSocketJointComponents.hasComponent(jointEntity)) {
           continue;
        }

        // Solve the constraint
        mConstraintSolverData.jointComponents.mJoints[i]->solveVelocityConstraint(mConstraintSolverData);
    }
}

// Solve the position constraints
void ConstraintSolverSystem::solvePositionConstraints() {

    RP3D_PROFILE("ConstraintSolverSystem::solvePositionConstraints()", mProfiler);

    mSolveBallAndSocketJointSystem.solvePositionConstraint();

    // For each joint
    for (uint i=0; i<mConstraintSolverData.jointComponents.getNbEnabledComponents(); i++) {

        // TODO : DELETE THIS
        Entity jointEntity = mConstraintSolverData.jointComponents.mJointEntities[i];
        if (mBallAndSocketJointComponents.hasComponent(jointEntity)) {
           continue;
        }

        // Solve the constraint
        mConstraintSolverData.jointComponents.mJoints[i]->solvePositionConstraint(mConstraintSolverData);
    }
}
