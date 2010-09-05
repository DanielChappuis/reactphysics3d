/***************************************************************************
* Copyright (C) 2009      Daniel Chappuis                                  *
****************************************************************************
* This file is part of ReactPhysics3D.                                     *
*                                                                          *
* ReactPhysics3D is free software: you can redistribute it and/or modify   *
* it under the terms of the GNU Lesser General Public License as published *
* by the Free Software Foundation, either version 3 of the License, or     *
* (at your option) any later version.                                      *
*                                                                          *
* ReactPhysics3D is distributed in the hope that it will be useful,        *
* but WITHOUT ANY WARRANTY; without even the implied warranty of           *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the             *
* GNU Lesser General Public License for more details.                      *
*                                                                          *
* You should have received a copy of the GNU Lesser General Public License *
* along with ReactPhysics3D. If not, see <http://www.gnu.org/licenses/>.   *
***************************************************************************/

#ifndef RIGIDBODY_H
#define RIGIDBODY_H

// Libraries
#include <cassert>
#include "Body.h"
#include "../mathematics/mathematics.h"

// Namespace reactphysics3d
namespace reactphysics3d {

/*  -------------------------------------------------------------------
    Class RigidBody :
        This class represents a rigid body of the physics
        engine. A rigid body is a non-deformable body that
        has a constant mass.
    -------------------------------------------------------------------
*/
class RigidBody : public Body {
    protected :
        Vector3D position;                          // Position of the center of mass of the body
        Vector3D oldPosition;                       // Old position used to compute the interpolated position
        Quaternion orientation;                     // Orientation quaternion of the body
        Quaternion oldOrientation;                  // Old orientation used to compute the interpolated orientation
        Vector3D linearVelocity;                    // Linear velocity of the body
        Vector3D angularVelocity;                   // Angular velocity of the body
        Vector3D externalForce;                     // Current external force on the body
        Vector3D externalTorque;                    // Current external torque on the body
        Matrix3x3 inertiaTensorLocal;               // Local inertia tensor of the body (in body coordinates)
        Matrix3x3 inertiaTensorLocalInverse;        // Inverse of the inertia tensor of the body (in body coordinates)
        double massInverse;                         // Inverse of the mass of the body
        double interpolationFactor;                 // Interpolation factor used for the state interpolation
        double restitution;                         // Coefficient of restitution (between 0 and 1), 1 for a very boucing body

    public :
        RigidBody(const Vector3D& position, const Quaternion& orientation, double mass,
                  const Matrix3x3& inertiaTensorLocal, NarrowBoundingVolume* narrowBoundingVolume);     // Constructor                                                                                                         // Copy-constructor
        virtual ~RigidBody();                                                                           // Destructor

        Vector3D getPosition() const;                                           // Return the position of the body
        void setPosition(const Vector3D& position);                             // Set the position of the body
        Quaternion getOrientation() const;                                      // Return the orientation quaternion
        void setOrientation(const Quaternion& orientation);                     // Set the orientation quaternion
        Vector3D getLinearVelocity() const;                                     // Return the linear velocity
        void setLinearVelocity(const Vector3D& linearVelocity);                 // Set the linear velocity of the body
        Vector3D getAngularVelocity() const;                                    // Return the angular velocity
        void setAngularVelocity(const Vector3D& angularVelocity);               // Set the angular velocity
        void setMassInverse(double massInverse);                                // Set the inverse of the mass
        Vector3D getExternalForce() const;                                      // Return the current external force of the body
        void setExternalForce(const Vector3D& force);                           // Set the current external force on the body
        Vector3D getExternalTorque() const;                                     // Return the current external torque of the body
        void setExternalTorque(const Vector3D& torque);                         // Set the current external torque of the body
        double getMassInverse() const;                                          // Return the inverse of the mass of the body
        Matrix3x3 getInertiaTensorLocal() const;                                // Return the local inertia tensor of the body (in body coordinates)
        void setInertiaTensorLocal(const Matrix3x3& inertiaTensorLocal);        // Set the local inertia tensor of the body (in body coordinates)
        Matrix3x3 getInertiaTensorLocalInverse() const;                         // Get the inverse of the inertia tensor
        Matrix3x3 getInertiaTensorWorld() const;                                // Return the inertia tensor in world coordinates
        Matrix3x3 getInertiaTensorInverseWorld() const;                         // Return the inverse of the inertia tensor in world coordinates
        void setInterpolationFactor(double factor);                             // Set the interpolation factor of the body
        Vector3D getInterpolatedPosition() const;                               // Return the interpolated position
        Quaternion getInterpolatedOrientation() const;                          // Return the interpolated orientation
        double getRestitution() const;                                          // Get the restitution coefficient
        void setRestitution(double restitution) throw(std::invalid_argument);   // Set the restitution coefficient
        void updateOldPositionAndOrientation();                                 // Update the previous position and orientation of the body
        void update();                                                          // Update the rigid body in order to reflect a change in the body state
};

// --- Inline functions --- //

// Return the position of the body
inline Vector3D RigidBody::getPosition() const {
    return position;
}

// Set the position of the body
inline void RigidBody::setPosition(const Vector3D& position) {
    this->position = position;
}

// Return the orientation quaternion of the body
inline Quaternion RigidBody::getOrientation() const {
    return orientation;
}

// Set the orientation quaternion
inline void RigidBody::setOrientation(const Quaternion& orientation) {
    this->orientation = orientation;

    // Normalize the orientation quaternion
    orientation.getUnit();
}

// Return the linear velocity
inline Vector3D RigidBody::getLinearVelocity() const {
    return linearVelocity;
}

// Return the angular velocity of the body
inline Vector3D RigidBody::getAngularVelocity() const {
    return angularVelocity;
}

inline void RigidBody::setAngularVelocity(const Vector3D& angularVelocity) {
     this->angularVelocity = angularVelocity;
}

// Set the inverse of the mass
inline void RigidBody::setMassInverse(double massInverse) {
    this->massInverse = massInverse;
}

// Get the inverse of the inertia tensor
inline Matrix3x3 RigidBody::getInertiaTensorLocalInverse() const {
    return inertiaTensorLocalInverse;
}

// Return the external force on the body
inline Vector3D RigidBody::getExternalForce() const {
    return externalForce;
}

// Set the external force on the body
inline void RigidBody::setExternalForce(const Vector3D& force) {
    this->externalForce = force;
}

// Return the current external torque on the body
inline Vector3D RigidBody::getExternalTorque() const {
    return externalTorque;
}

 // Set the current external torque on the body
inline void RigidBody::setExternalTorque(const Vector3D& torque) {
    this->externalTorque = torque;
}

// Return the inverse of the mass of the body
inline double RigidBody::getMassInverse() const {
    return massInverse;
}

// Return the local inertia tensor of the body (in body coordinates)
inline Matrix3x3 RigidBody::getInertiaTensorLocal() const {
    return inertiaTensorLocal;
}

// Set the local inertia tensor of the body (in body coordinates)
inline void RigidBody::setInertiaTensorLocal(const Matrix3x3& inertiaTensorLocal) {
    this->inertiaTensorLocal = inertiaTensorLocal;
}

// Return the inertia tensor in world coordinates
// The inertia tensor I_w in world coordinates in computed with the local inertia tensor I_b in body coordinates
// by I_w = R * I_b * R^T
// where R is the rotation matrix (and R^T its transpose) of the current orientation quaternion of the body
inline Matrix3x3 RigidBody::getInertiaTensorWorld() const {
    // Compute and return the inertia tensor in world coordinates
    return orientation.getMatrix() * inertiaTensorLocal * orientation.getMatrix().getTranspose();
}

// Return the inverse of the inertia tensor in world coordinates
// The inertia tensor I_w in world coordinates in computed with the local inverse inertia tensor I_b^-1 in body coordinates
// by I_w = R * I_b^-1 * R^T
// where R is the rotation matrix (and R^T its transpose) of the current orientation quaternion of the body
inline Matrix3x3 RigidBody::getInertiaTensorInverseWorld() const {
    // Compute and return the inertia tensor in world coordinates
    return orientation.getMatrix() * inertiaTensorLocalInverse * orientation.getMatrix().getTranspose();
}

// Set the interpolation factor of the body
inline void RigidBody::setInterpolationFactor(double factor) {
    // Set the factor
    interpolationFactor = factor;
}

// Return the interpolated position
inline Vector3D RigidBody::getInterpolatedPosition() const {
    // Compute the interpolated position
    return oldPosition * (1-interpolationFactor) + position * interpolationFactor;
}

 // Return the interpolated orientation
inline Quaternion RigidBody::getInterpolatedOrientation() const {
    // Compute the interpolated orientation
    return Quaternion::slerp(oldOrientation, orientation, interpolationFactor);
}

// Set the linear velocity of the rigid body
inline void RigidBody::setLinearVelocity(const Vector3D& linearVelocity) {
    // If the body is able to move
    if (isMotionEnabled) {
        // Update the linear velocity of the current body state
        this->linearVelocity = linearVelocity;
    }
}

// Get the restitution coeffficient of the rigid body
inline double RigidBody::getRestitution() const {
    return restitution;
}

// Set the restitution coefficient
inline void RigidBody::setRestitution(double restitution) throw(std::invalid_argument) {
    // Check if the restitution coefficient is between 0 and 1
    if (restitution >= 0.0 && restitution <= 1.0) {
        this->restitution = restitution;
    }
    else {
        throw std::invalid_argument("Error : the restitution coefficent must be between 0 and 1");
    }
}

// Update the previous body state of the body
// This is used to compute the interpolated position and orientation of the body
inline void RigidBody::updateOldPositionAndOrientation() {
    oldPosition = position;
    oldOrientation = orientation;
}

} // End of the ReactPhyscis3D namespace

 #endif
