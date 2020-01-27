/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2019 Daniel Chappuis                                       *
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

#ifndef REACTPHYSICS3D_RIGID_BODY_H
#define REACTPHYSICS3D_RIGID_BODY_H

// Libraries
#include <cassert>
#include "CollisionBody.h"
#include "engine/Material.h"
#include "mathematics/mathematics.h"

/// Namespace reactphysics3d
namespace reactphysics3d {

// Class declarations
struct JointListElement;
class PhysicsWorld;
class MemoryManager;
enum class BodyType;

// Class RigidBody
/**
 * This class represents a rigid body of the physics
 * engine. A rigid body is a non-deformable body that
 * has a constant mass. This class inherits from the
 * CollisionBody class.
  */
class RigidBody : public CollisionBody {

    protected :

        // -------------------- Attributes -------------------- //

        /// Inverse Local inertia tensor of the body (in local-space) set
        /// by the user with respect to the center of mass of the body
        Matrix3x3 mUserInertiaTensorLocalInverse;

        /// Material properties of the rigid body
        Material mMaterial;

        /// True if the center of mass is set by the user
        bool mIsCenterOfMassSetByUser;

        /// True if the inertia tensor is set by the user
        bool mIsInertiaTensorSetByUser;

        // -------------------- Methods -------------------- //

        /// Set the variable to know whether or not the body is sleeping
        void setIsSleeping(bool isSleeping);

        /// Update whether the current overlapping pairs where this body is involed are active or not
        void updateOverlappingPairs();

        /// Return the inverse of the inertia tensor in world coordinates.
        static const Matrix3x3 getInertiaTensorInverseWorld(PhysicsWorld& world, Entity bodyEntity);

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        RigidBody(PhysicsWorld& world, Entity entity);

        /// Destructor
        virtual ~RigidBody() override = default;

        /// Deleted copy-constructor
        RigidBody(const RigidBody& body) = delete;

        /// Deleted assignment operator
        RigidBody& operator=(const RigidBody& body) = delete;

        /// Set the current position and orientation
        virtual void setTransform(const Transform& transform) override;

        /// Return the mass of the body
        decimal getMass() const;

        /// Return the linear velocity
        Vector3 getLinearVelocity() const;

        /// Set the linear velocity of the body.
        void setLinearVelocity(const Vector3& linearVelocity);

        /// Return the angular velocity
        Vector3 getAngularVelocity() const;

        /// Set the angular velocity.
        void setAngularVelocity(const Vector3& angularVelocity);

        /// Set the local inertia tensor of the body (in body coordinates)
        void setInertiaTensorLocal(const Matrix3x3& inertiaTensorLocal);

        /// Set the inverse local inertia tensor of the body (in body coordinates)
        void setInverseInertiaTensorLocal(const Matrix3x3& inverseInertiaTensorLocal);

        /// Get the inverse local inertia tensor of the body (in body coordinates)
        const Matrix3x3& getInverseInertiaTensorLocal() const;

        /// Return the inverse of the inertia tensor in world coordinates.
        const Matrix3x3 getInertiaTensorInverseWorld() const;

        /// Set the local center of mass of the body (in local-space coordinates)
        void setCenterOfMassLocal(const Vector3& centerOfMassLocal);

        /// Set the mass of the rigid body
        void setMass(decimal mass);

        /// Return the type of the body
        BodyType getType() const;

        /// Set the type of the body
        void setType(BodyType type);

        /// Return true if the gravity needs to be applied to this rigid body
        bool isGravityEnabled() const;

        /// Set the variable to know if the gravity is applied to this rigid body
        void enableGravity(bool isEnabled);

        /// Return a reference to the material properties of the rigid body
        Material& getMaterial();

        /// Set a new material for this rigid body
        void setMaterial(const Material& material);

        /// Return the linear velocity damping factor
        decimal getLinearDamping() const;

        /// Set the linear damping factor
        void setLinearDamping(decimal linearDamping);

        /// Return the angular velocity damping factor
        decimal getAngularDamping() const;

        /// Set the angular damping factor
        void setAngularDamping(decimal angularDamping);

        /// Apply an external force to the body at its center of mass.
        void applyForceToCenterOfMass(const Vector3& force);

        /// Apply an external force to the body at a given point (in world-space coordinates).
        void applyForce(const Vector3& force, const Vector3& point);

        /// Apply an external torque to the body.
        void applyTorque(const Vector3& torque);

        /// Return whether or not the body is allowed to sleep
        bool isAllowedToSleep() const;

        /// Set whether or not the body is allowed to go to sleep
        void setIsAllowedToSleep(bool isAllowedToSleep);

        /// Return whether or not the body is sleeping
        bool isSleeping() const;

        /// Set whether or not the body is active
        virtual void setIsActive(bool isActive) override;

        /// Create a new collider and add it to the body
        // TODO : Remove the mass from this parameter so that we can correctly use inheritance here
        //        The user will then need to call Collider->setMass() to set the mass of the shape
        virtual Collider* addCollider(CollisionShape* collisionShape, const Transform& transform, decimal mass);

        /// Remove a collider from the body
        virtual void removeCollider(Collider* collider) override;

        /// Recompute the center of mass, total mass and inertia tensor of the body using all
        /// the collision shapes attached to the body.
        void recomputeMassInformation();

#ifdef IS_PROFILING_ACTIVE

		/// Set the profiler
		void setProfiler(Profiler* profiler) override;

#endif

        // -------------------- Friendship -------------------- //

        friend class PhysicsWorld;
        friend class ContactSolverSystem;
        friend class DynamicsSystem;
        friend class BallAndSocketJoint;
        friend class SliderJoint;
        friend class HingeJoint;
        friend class FixedJoint;
        friend class SolveBallAndSocketJointSystem;
        friend class SolveFixedJointSystem;
        friend class SolveHingeJointSystem;
        friend class SolveSliderJointSystem;
        friend class Joint;
};

// Return a reference to the material properties of the rigid body
/**
 * @return A reference to the material of the body
 */
inline Material& RigidBody::getMaterial() {
    return mMaterial;
}

}

 #endif
