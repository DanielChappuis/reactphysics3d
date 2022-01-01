/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2022 Daniel Chappuis                                       *
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

#ifndef REACTPHYSICS3D_SOLVE_HINGE_JOINT_SYSTEM_H
#define REACTPHYSICS3D_SOLVE_HINGE_JOINT_SYSTEM_H

// Libraries
#include <reactphysics3d/utils/Profiler.h>
#include <reactphysics3d/components/RigidBodyComponents.h>
#include <reactphysics3d/components/JointComponents.h>
#include <reactphysics3d/components/HingeJointComponents.h>
#include <reactphysics3d/components/TransformComponents.h>

namespace reactphysics3d {

class PhysicsWorld;

// Class SolveHingeJointSystem
/**
 * This class is responsible to solve the BallAndSocketJoint constraints
 */
class SolveHingeJointSystem {

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

        /// Reference to the hinge joint components
        HingeJointComponents& mHingeJointComponents;

        /// Current time step of the simulation
        decimal mTimeStep;

        /// True if warm starting of the solver is active
        bool mIsWarmStartingActive;

#ifdef IS_RP3D_PROFILING_ENABLED

        /// Pointer to the profiler
        Profiler* mProfiler;
#endif

        // -------------------- Methods -------------------- //

        /// Given an angle in radian, this method returns the corresponding
        /// angle in the range [-pi; pi]
        decimal computeNormalizedAngle(decimal angle) const;

        /// Given an "inputAngle" in the range [-pi, pi], this method returns an
        /// angle (modulo 2*pi) in the range [-2*pi; 2*pi] that is closest to one of the
        /// two angle limits in arguments.
        decimal computeCorrespondingAngleNearLimits(decimal inputAngle, decimal lowerLimitAngle,
                                                    decimal upperLimitAngle) const;

        /// Compute the current angle around the hinge axis
        decimal computeCurrentHingeAngle(Entity jointEntity, const Quaternion& orientationBody1, const Quaternion& orientationBody2);

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        SolveHingeJointSystem(PhysicsWorld& world, RigidBodyComponents& rigidBodyComponents,
                              TransformComponents& transformComponents,
                              JointComponents& jointComponents,
                              HingeJointComponents& hingeJointComponents);

        /// Destructor
        ~SolveHingeJointSystem() = default;

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

#ifdef IS_RP3D_PROFILING_ENABLED

        /// Set the profiler
        void setProfiler(Profiler* profiler);

#endif

        // ---------- Friendship ----------

        friend class HingeJoint;

};

#ifdef IS_RP3D_PROFILING_ENABLED

// Set the profiler
RP3D_FORCE_INLINE void SolveHingeJointSystem::setProfiler(Profiler* profiler) {
    mProfiler = profiler;
}

#endif

// Set the time step
RP3D_FORCE_INLINE void SolveHingeJointSystem::setTimeStep(decimal timeStep) {
    assert(timeStep > decimal(0.0));
    mTimeStep = timeStep;
}

// Set to true to enable warm starting
RP3D_FORCE_INLINE void SolveHingeJointSystem::setIsWarmStartingActive(bool isWarmStartingActive) {
    mIsWarmStartingActive = isWarmStartingActive;
}


}

#endif
