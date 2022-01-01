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

#ifndef REACTPHYSICS3D_DYNAMICS_SYSTEM_H
#define REACTPHYSICS3D_DYNAMICS_SYSTEM_H

// Libraries
#include <reactphysics3d/utils/Profiler.h>
#include <reactphysics3d/components/CollisionBodyComponents.h>
#include <reactphysics3d/components/RigidBodyComponents.h>
#include <reactphysics3d/components/TransformComponents.h>
#include <reactphysics3d/components/ColliderComponents.h>

namespace reactphysics3d {

class PhysicsWorld;

// Class DynamicsSystem
/**
 * This class is responsible to compute and update the dynamics of the bodies that are simulated
 * using physics.
 */
class DynamicsSystem {

    private :

        // -------------------- Attributes -------------------- //

        /// Physics world
        PhysicsWorld& mWorld;

        /// Reference to the collision body components
        CollisionBodyComponents& mCollisionBodyComponents;

        /// Reference to the rigid body components
        RigidBodyComponents& mRigidBodyComponents;

        /// Reference to the transform components
        TransformComponents& mTransformComponents;

        /// Reference to the colliders components
        ColliderComponents& mColliderComponents;

        /// Reference to the variable to know if gravity is enabled in the world
        bool& mIsGravityEnabled;

        /// Reference to the world gravity vector
        Vector3& mGravity;

#ifdef IS_RP3D_PROFILING_ENABLED

        /// Pointer to the profiler
        Profiler* mProfiler;
#endif

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        DynamicsSystem(PhysicsWorld& world, CollisionBodyComponents& collisionBodyComponents,
                       RigidBodyComponents& rigidBodyComponents, TransformComponents& transformComponents,
                       ColliderComponents& colliderComponents, bool& isGravityEnabled, Vector3& gravity);

        /// Destructor
        ~DynamicsSystem() = default;

#ifdef IS_RP3D_PROFILING_ENABLED

        /// Set the profiler
        void setProfiler(Profiler* profiler);

#endif

        /// Integrate the positions and orientations of rigid bodies.
        void integrateRigidBodiesPositions(decimal timeStep, bool isSplitImpulseActive);

        /// Integrate the velocities of rigid bodies.
        void integrateRigidBodiesVelocities(decimal timeStep);

        /// Update the postion/orientation of the bodies
        void updateBodiesState();

        /// Reset the external force and torque applied to the bodies
        void resetBodiesForceAndTorque();

        /// Reset the split velocities of the bodies
        void resetSplitVelocities();

};

#ifdef IS_RP3D_PROFILING_ENABLED

// Set the profiler
RP3D_FORCE_INLINE void DynamicsSystem::setProfiler(Profiler* profiler) {
    mProfiler = profiler;
}

#endif

}

#endif
