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

#ifndef REACTPHYSICS3D_RIGID_BODY_H
#define REACTPHYSICS3D_RIGID_BODY_H

// Libraries
#include <cassert>
#include <reactphysics3d/body/Body.h>
#include <reactphysics3d/mathematics/mathematics.h>

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
 * Body class.
  */
class RigidBody : public Body {

    protected :

        // -------------------- Methods -------------------- //

        /// Awake the disabled neighbor bodies
        void awakeNeighborDisabledBodies();

        /// Remove the disabled overlapping pairs
        void enableOverlappingPairs();

        /// Disable the overlapping pairs if both bodies of the pair are disabled (sleeping or static)
        void checkForDisabledOverlappingPairs();

        /// Compute and return the local-space center of mass of the body using its colliders
        Vector3 computeCenterOfMass() const;

        /// Compute the local-space inertia tensor and total mass of the body using its colliders
        void computeMassAndInertiaTensorLocal(Vector3& inertiaTensorLocal, decimal& totalMass) const;

        /// Compute the inverse of the inertia tensor in world coordinates.
        static void computeWorldInertiaTensorInverse(const Matrix3x3& orientation, const Vector3& inverseInertiaTensorLocal, Matrix3x3& outInverseInertiaTensorWorld);

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

        /// Set the mass of the rigid body
        void setMass(decimal mass);

        /// Return the linear velocity
        Vector3 getLinearVelocity() const;

        /// Set the linear velocity of the body.
        void setLinearVelocity(const Vector3& linearVelocity);

        /// Return the angular velocity
        Vector3 getAngularVelocity() const;

        /// Set the angular velocity.
        void setAngularVelocity(const Vector3& angularVelocity);

        /// Return the local inertia tensor of the body (in body coordinates)
        const Vector3& getLocalInertiaTensor() const;

        /// Set the local inertia tensor of the body (in body coordinates)
        void setLocalInertiaTensor(const Vector3& inertiaTensorLocal);

        /// Return the center of mass of the body (in local-space coordinates)
        const Vector3& getLocalCenterOfMass() const;

        /// Set the center of mass of the body (in local-space coordinates)
        void setLocalCenterOfMass(const Vector3& centerOfMass);

        /// Compute and set the local-space center of mass of the body using its colliders
        void updateLocalCenterOfMassFromColliders();

        /// Compute and set the local-space inertia tensor of the body using its colliders
        void updateLocalInertiaTensorFromColliders();

        /// Compute and set the mass of the body using its colliders
        void updateMassFromColliders();

        /// Compute and set the center of mass, the mass and the local-space inertia tensor of the body using its colliders
        void updateMassPropertiesFromColliders();

        /// Return the type of the body
        BodyType getType() const;

        /// Set the type of the body
        void setType(BodyType type);

        /// Return true if the gravity needs to be applied to this rigid body
        bool isGravityEnabled() const;

        /// Set the variable to know if the gravity is applied to this rigid body
        void enableGravity(bool isEnabled);

        /// Set the variable to know whether or not the body is sleeping
        void setIsSleeping(bool isSleeping);

        /// Return the linear velocity damping factor
        decimal getLinearDamping() const;

        /// Set the linear damping factor
        void setLinearDamping(decimal linearDamping);

        /// Return the angular velocity damping factor
        decimal getAngularDamping() const;

        /// Set the angular damping factor
        void setAngularDamping(decimal angularDamping);

        /// Return the lock translation factor
        const Vector3& getLinearLockAxisFactor() const;

        /// Set the linear lock factor
        void setLinearLockAxisFactor(const Vector3& linearLockAxisFactor) const;

        /// Return the lock rotation factor
        const Vector3& getAngularLockAxisFactor() const;

        /// Set the lock rotation factor
        void setAngularLockAxisFactor(const Vector3& angularLockAxisFactor) const;

        /// Manually apply an external force (in local-space) to the body at its center of mass.
        void applyLocalForceAtCenterOfMass(const Vector3& force);

        /// Manually apply an external force (in world-space) to the body at its center of mass.
        void applyWorldForceAtCenterOfMass(const Vector3& force);

        /// Manually apply an external force (in local-space) to the body at a given point (in local-space).
        void applyLocalForceAtLocalPosition(const Vector3& force, const Vector3& point);

        /// Manually apply an external force (in world-space) to the body at a given point (in local-space).
        void applyWorldForceAtLocalPosition(const Vector3& force, const Vector3& point);

        /// Manually apply an external force (in local-space) to the body at a given point (in world-space).
        void applyLocalForceAtWorldPosition(const Vector3& force, const Vector3& point);

        /// Manually apply an external force (in world-space) to the body at a given point (in world-space).
        void applyWorldForceAtWorldPosition(const Vector3& force, const Vector3& point);

        /// Manually apply an external torque to the body (in world-space).
        void applyWorldTorque(const Vector3& torque);

        /// Manually apply an external torque to the body (in local-space).
        void applyLocalTorque(const Vector3& torque);

        /// Reset the manually applied force to zero
        void resetForce();

        /// Reset the manually applied torque to zero
        void resetTorque();

        /// Return the total manually applied force on the body (in world-space)
        const Vector3& getForce() const;

        /// Return the total manually applied torque on the body (in world-space)
        const Vector3& getTorque() const;

        /// Return whether or not the body is allowed to sleep
        bool isAllowedToSleep() const;

        /// Set whether or not the body is allowed to go to sleep
        void setIsAllowedToSleep(bool isAllowedToSleep);

        /// Return whether or not the body is sleeping
        bool isSleeping() const;

        /// Set whether or not the body is active
        virtual void setIsActive(bool isActive) override;

        /// Create a new collider and add it to the body
        virtual Collider* addCollider(CollisionShape* collisionShape, const Transform& transform) override;

        /// Remove a collider from the body
        virtual void removeCollider(Collider* collider) override;

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
        friend class Collider;
};

/// Compute the inverse of the inertia tensor in world coordinates.
RP3D_FORCE_INLINE void RigidBody::computeWorldInertiaTensorInverse(const Matrix3x3& orientation, const Vector3& inverseInertiaTensorLocal, Matrix3x3& outInverseInertiaTensorWorld) {

    outInverseInertiaTensorWorld[0][0] = orientation[0][0] * inverseInertiaTensorLocal.x;
    outInverseInertiaTensorWorld[0][1] = orientation[1][0] * inverseInertiaTensorLocal.x;
    outInverseInertiaTensorWorld[0][2] = orientation[2][0] * inverseInertiaTensorLocal.x;

    outInverseInertiaTensorWorld[1][0] = orientation[0][1] * inverseInertiaTensorLocal.y;
    outInverseInertiaTensorWorld[1][1] = orientation[1][1] * inverseInertiaTensorLocal.y;
    outInverseInertiaTensorWorld[1][2] = orientation[2][1] * inverseInertiaTensorLocal.y;

    outInverseInertiaTensorWorld[2][0] = orientation[0][2] * inverseInertiaTensorLocal.z;
    outInverseInertiaTensorWorld[2][1] = orientation[1][2] * inverseInertiaTensorLocal.z;
    outInverseInertiaTensorWorld[2][2] = orientation[2][2] * inverseInertiaTensorLocal.z;

    outInverseInertiaTensorWorld = orientation * outInverseInertiaTensorWorld;
}

}

 #endif
