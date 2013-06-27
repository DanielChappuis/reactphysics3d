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

#ifndef REACTPHYSICS3D_RIGID_BODY_H
#define REACTPHYSICS3D_RIGID_BODY_H

// Libraries
#include <cassert>
#include "CollisionBody.h"
#include "../mathematics/mathematics.h"

/// Namespace reactphysics3d
namespace reactphysics3d {

// Class RigidBody
/**
 * This class represents a rigid body of the physics
 * engine. A rigid body is a non-deformable body that
 * has a constant mass. This class inherits from the
 * CollisionBody class.
  */
class RigidBody : public CollisionBody {

    protected :

        // TODO : Remove the mass variable (duplicate with inverseMass)

        // -------------------- Attributes -------------------- //

        /// Mass of the body
        decimal mMass;

        /// Linear velocity of the body
        Vector3 mLinearVelocity;

        /// Angular velocity of the body
        Vector3 mAngularVelocity;

        /// Current external force on the body
        Vector3 mExternalForce;

        /// Current external torque on the body
        Vector3 mExternalTorque;

        /// Local inertia tensor of the body (in local-space)
        Matrix3x3 mInertiaTensorLocal;

        /// Inverse of the inertia tensor of the body
        Matrix3x3 mInertiaTensorLocalInverse;

        /// Inverse of the mass of the body
        decimal mMassInverse;

        /// Coefficient of restitution (between 0 and 1) where 1 is for a very bouncy body
        decimal mRestitution;

        /// Friction coefficient
        decimal mFrictionCoefficient;

        /// True if the gravity needs to be applied to this rigid body
        bool mIsGravityEnabled;

        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        RigidBody(const RigidBody& body);

        /// Private assignment operator
        RigidBody& operator=(const RigidBody& body);

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        RigidBody(const Transform& transform, decimal mass, const Matrix3x3& inertiaTensorLocal,
                  CollisionShape* collisionShape, bodyindex id);

        /// Destructor
        virtual ~RigidBody();

        /// Return the mass of the body
        decimal getMass() const;

        /// Set the mass of the body
        void setMass(decimal mass);

        /// Return the linear velocity
        Vector3 getLinearVelocity() const;

        /// Set the linear velocity of the body
        void setLinearVelocity(const Vector3& linearVelocity);

        /// Return the angular velocity
        Vector3 getAngularVelocity() const;

        /// Set the angular velocity
        void setAngularVelocity(const Vector3& angularVelocity);

        /// Set the inverse of the mass
        void setMassInverse(decimal massInverse);

        /// Return the current external force of the body
        Vector3 getExternalForce() const;

        /// Set the current external force on the body
        void setExternalForce(const Vector3& force);

        /// Return the current external torque of the body
        Vector3 getExternalTorque() const;

        /// Set the current external torque of the body
        void setExternalTorque(const Vector3& torque);

        /// Return the inverse of the mass of the body
        decimal getMassInverse() const;

        /// Return the local inertia tensor of the body (in body coordinates)
        const Matrix3x3& getInertiaTensorLocal() const;

        /// Set the local inertia tensor of the body (in body coordinates)
        void setInertiaTensorLocal(const Matrix3x3& inertiaTensorLocal);

        /// Get the inverse of the inertia tensor
        Matrix3x3 getInertiaTensorLocalInverse() const;

        /// Return the inertia tensor in world coordinates.
        Matrix3x3 getInertiaTensorWorld() const;

        /// Return the inverse of the inertia tensor in world coordinates.
        Matrix3x3 getInertiaTensorInverseWorld() const;
        
        /// Get the restitution coefficient
        decimal getRestitution() const;

        /// Set the restitution coefficient
        void setRestitution(decimal restitution);

        /// Get the friction coefficient
        decimal getFrictionCoefficient() const;

        /// Set the friction coefficient
        void setFrictionCoefficient(decimal frictionCoefficient);

        /// Return true if the gravity needs to be applied to this rigid body
        bool isGravityEnabled() const;

        /// Set the variable to know if the gravity is applied to this rigid body
        void enableGravity(bool isEnabled);
};

// Method that return the mass of the body
inline decimal RigidBody::getMass() const {
    return mMass;
}

// Method that set the mass of the body
inline void RigidBody::setMass(decimal mass) {
    mMass = mass;
}

// Return the linear velocity
inline Vector3 RigidBody::getLinearVelocity() const {
    return mLinearVelocity;
}

// Return the angular velocity of the body
inline Vector3 RigidBody::getAngularVelocity() const {
    return mAngularVelocity;
}

inline void RigidBody::setAngularVelocity(const Vector3& angularVelocity) {
     mAngularVelocity = angularVelocity;
}

// Set the inverse of the mass
inline void RigidBody::setMassInverse(decimal massInverse) {
    mMassInverse = massInverse;
}

// Get the inverse of the inertia tensor
inline Matrix3x3 RigidBody::getInertiaTensorLocalInverse() const {
    return mInertiaTensorLocalInverse;
}

// Return the external force on the body
inline Vector3 RigidBody::getExternalForce() const {
    return mExternalForce;
}

// Set the external force on the body
inline void RigidBody::setExternalForce(const Vector3& force) {
    mExternalForce = force;
}

// Return the current external torque on the body
inline Vector3 RigidBody::getExternalTorque() const {
    return mExternalTorque;
}

 // Set the current external torque on the body
inline void RigidBody::setExternalTorque(const Vector3& torque) {
    mExternalTorque = torque;
}

// Return the inverse of the mass of the body
inline decimal RigidBody::getMassInverse() const {
    return mMassInverse;
}

// Return the local inertia tensor of the body (in body coordinates)
inline const Matrix3x3& RigidBody::getInertiaTensorLocal() const {
    return mInertiaTensorLocal;
}

// Set the local inertia tensor of the body (in body coordinates)
inline void RigidBody::setInertiaTensorLocal(const Matrix3x3& inertiaTensorLocal) {
    mInertiaTensorLocal = inertiaTensorLocal;
}

// Return the inertia tensor in world coordinates.
/// The inertia tensor I_w in world coordinates is computed
/// with the local inertia tensor I_b in body coordinates
/// by I_w = R * I_b * R^T
/// where R is the rotation matrix (and R^T its transpose) of
/// the current orientation quaternion of the body
inline Matrix3x3 RigidBody::getInertiaTensorWorld() const {

    // Compute and return the inertia tensor in world coordinates
    return mTransform.getOrientation().getMatrix() * mInertiaTensorLocal *
           mTransform.getOrientation().getMatrix().getTranspose();
}

// Return the inverse of the inertia tensor in world coordinates.
/// The inertia tensor I_w in world coordinates is computed with the
/// local inverse inertia tensor I_b^-1 in body coordinates
/// by I_w = R * I_b^-1 * R^T
/// where R is the rotation matrix (and R^T its transpose) of the
/// current orientation quaternion of the body
inline Matrix3x3 RigidBody::getInertiaTensorInverseWorld() const {

    // Compute and return the inertia tensor in world coordinates
    return mTransform.getOrientation().getMatrix() * mInertiaTensorLocalInverse *
           mTransform.getOrientation().getMatrix().getTranspose();
}

// Set the linear velocity of the rigid body
inline void RigidBody::setLinearVelocity(const Vector3& linearVelocity) {

    // If the body is able to move
    if (mIsMotionEnabled) {
        // Update the linear velocity of the current body state
        mLinearVelocity = linearVelocity;
    }
}

// Get the restitution coeffficient of the rigid body
inline decimal RigidBody::getRestitution() const {
    return mRestitution;
}

// Set the restitution coefficient
inline void RigidBody::setRestitution(decimal restitution) {
    assert(restitution >= 0.0 && restitution <= 1.0);
    mRestitution = restitution;
}

// Get the friction coefficient
inline decimal RigidBody::getFrictionCoefficient() const {
    return mFrictionCoefficient;
}

// Set the friction coefficient
inline void RigidBody::setFrictionCoefficient(decimal frictionCoefficient) {
    mFrictionCoefficient = frictionCoefficient;
}

// Return true if the gravity needs to be applied to this rigid body
inline bool RigidBody::isGravityEnabled() const {
    return mIsGravityEnabled;
}

// Set the variable to know if the gravity is applied to this rigid body
inline void RigidBody::enableGravity(bool isEnabled) {
    mIsGravityEnabled = isEnabled;
}

}

 #endif
