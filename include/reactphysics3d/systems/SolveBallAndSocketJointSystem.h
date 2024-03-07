/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2024 Daniel Chappuis                                       *
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

#ifndef REACTPHYSICS3D_SOLVE_BALL_SOCKET_JOINT_SYSTEM_H
#define REACTPHYSICS3D_SOLVE_BALL_SOCKET_JOINT_SYSTEM_H

// Libraries
#include <reactphysics3d/utils/Profiler.h>
#include <reactphysics3d/components/RigidBodyComponents.h>
#include <reactphysics3d/components/JointComponents.h>
#include <reactphysics3d/components/BallAndSocketJointComponents.h>
#include <reactphysics3d/components/TransformComponents.h>

namespace reactphysics3d {

// Forward declarations
class PhysicsWorld;

// Class SolveBallAndSocketJointSystem
/**
 * This class is responsible to solve the BallAndSocketJoint constraints
 */
class SolveBallAndSocketJointSystem {

    private :

        // -------------------- Constants -------------------- //

        // Beta value for the bias factor of position correction
        static const decimal BETA;

        // -------------------- Attributes -------------------- //

        /// Physics world
        PhysicsWorld& mWorld;

        /// Reference to the rigid body components
        RigidBodyComponents& mRigidBodyComponents;

        /// Reference to transform components
        TransformComponents& mTransformComponents;

        /// Reference to the joint components
        JointComponents& mJointComponents;

        /// Reference to the ball-and-socket joint components
        BallAndSocketJointComponents& mBallAndSocketJointComponents;

        /// Current time step of the simulation
        decimal mTimeStep;

        /// True if warm starting of the solver is active
        bool mIsWarmStartingActive;

#ifdef IS_RP3D_PROFILING_ENABLED

        /// Pointer to the profiler
        Profiler* mProfiler;
#endif

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        SolveBallAndSocketJointSystem(PhysicsWorld& world, RigidBodyComponents& rigidBodyComponents,
                                      TransformComponents& transformComponents,
                                      JointComponents& jointComponents,
                                      BallAndSocketJointComponents& ballAndSocketJointComponents);

        /// Destructor
        ~SolveBallAndSocketJointSystem() = default;

        /// Initialize before solving the constraint
        void initBeforeSolve();

        /// Warm start the constraint (apply the previous impulse at the beginning of the step)
         void warmstart();

        /// Solve the velocity constraint
        void solveVelocityConstraint();

        /// Solve the position constraint (for position error correction)
        void solvePositionConstraint();

        /// Set the time step
        void setTimeStep(decimal timeStep);

        /// Set to true to enable warm starting
        void setIsWarmStartingActive(bool isWarmStartingActive);

        /// Return the current cone angle (for the cone limit)
        static decimal computeCurrentConeHalfAngle(const Vector3& coneLimitWorldAxisBody1, const Vector3& coneLimitWorldAxisBody2);

#ifdef IS_RP3D_PROFILING_ENABLED

        /// Set the profiler
        void setProfiler(Profiler* profiler);

#endif

};

#ifdef IS_RP3D_PROFILING_ENABLED

// Set the profiler
RP3D_FORCE_INLINE void SolveBallAndSocketJointSystem::setProfiler(Profiler* profiler) {
    mProfiler = profiler;
}

#endif

// Set the time step
RP3D_FORCE_INLINE void SolveBallAndSocketJointSystem::setTimeStep(decimal timeStep) {
    assert(timeStep > decimal(0.0));
    mTimeStep = timeStep;
}

// Set to true to enable warm starting
RP3D_FORCE_INLINE void SolveBallAndSocketJointSystem::setIsWarmStartingActive(bool isWarmStartingActive) {
    mIsWarmStartingActive = isWarmStartingActive;
}

// Return the current cone angle (for the cone limit)
/**
 * @return The positive cone angle in radian in range [0, PI]
 */
RP3D_FORCE_INLINE decimal SolveBallAndSocketJointSystem::computeCurrentConeHalfAngle(const Vector3& coneLimitWorldAxisBody1,
                                                                                     const Vector3& coneLimitWorldAxisBody2) {

    return std::acos(coneLimitWorldAxisBody1.dot(coneLimitWorldAxisBody2));
}

}

#endif
