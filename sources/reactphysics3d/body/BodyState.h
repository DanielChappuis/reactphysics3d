/****************************************************************************
 * Copyright (C) 2009      Daniel Chappuis                                  *
 ****************************************************************************
 * This file is part of ReactPhysics3D.                                     *
 *                                                                          *
 * ReactPhysics3D is free software: you can redistribute it and/or modify   *
 * it under the terms of the GNU General Public License as published by     *
 * the Free Software Foundation, either version 3 of the License, or        *
 * (at your option) any later version.                                      *
 *                                                                          *
 * ReactPhysics3D is distributed in the hope that it will be useful,        *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of           *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the             *
 * GNU General Public License for more details.                             *
 *                                                                          *
 * You should have received a copy of the GNU General Public License        *
 * along with ReactPhysics3D. If not, see <http://www.gnu.org/licenses/>.   *
 ***************************************************************************/

#ifndef BODYSTATE_H
#define BODYSTATE_H

// Libraries
#include "../mathematics/mathematics.h"
#include "../physics/physics.h"

// Namespace reactphysics3d
namespace reactphysics3d {

/*  -------------------------------------------------------------------
    Class BodyState :
        A BodyState contains all the dynamics values of a body.
    -------------------------------------------------------------------
*/
class BodyState {
    private :
        // Primary values
        Vector3D position;                      // Position of the body
        Vector3D linearMomentum;                // Linear momentum of the body
        Quaternion orientation;                 // Orientation quaternion of the body
        Vector3D angularMomentum;               // Angular momentum of the body

        // Secondary values
        Vector3D linearVelocity;                // Linear velocity of the body
        Vector3D angularVelocity;               // Angular velocity of the body
        Quaternion spin;                        // Spin is the derivative of orientation quaternion over time.

        // Constants
        Matrix3x3 inertiaTensorInverse;         // Inverse of the inertia tensor of the body
        Kilogram massInverse;                     // Inverse of the mass of the body

    public :
        BodyState(const Vector3D& position, const Matrix3x3& inertiaTensorInverse, const Kilogram& massInverse);    // Constructor
        BodyState(const BodyState& bodyState);                                                                      // Copy-constructor
        virtual ~BodyState();                                                                                       // Destructor

        Vector3D getPosition() const;                                           // Return the position of the body
        void setPosition(const Vector3D& position);                             // Set the position of the body
        Vector3D getLinearMomentum() const;                                     // Return the linear momemtum
        void setLinearMomentum(const Vector3D& linearMomentum);                 // Set the linear momentum
        Quaternion getOrientation() const;                                      // Return the orientation quaternion
        void setOrientation(const Quaternion& orientation);                     // Set the orientation quaternion
        Vector3D getAngularMomentum() const;                                    // Return the angular momentum
        void setAngularMomentum(const Vector3D& angularMomentum);               // Set the angular momentum
        Vector3D getAngularVelocity() const;                                    // Return the angular velocity
        Quaternion getSpin() const;                                             // Return the spin of the body
        void setMassInverse(Kilogram massInverse);                              // Set the inverse of the mass
        void setInertiaTensorInverse(const Matrix3x3& inertiaTensorInverse);    // Set the inverse of the inertia tensor

        void recalculate();             // Recalculate the secondary values of the BodyState

        // Overloaded operators
        BodyState operator*(double number) const;       // Overloaded operator for the multiplication with a number
};

// --- Inlines functions --- //

// Return the position of the body
inline Vector3D BodyState::getPosition() const {
    return position;
}

// Set the position of the body
inline void BodyState::setPosition(const Vector3D& position) {
    this->position = position;
}

// Return the linear momentum of the body
inline Vector3D BodyState::getLinearMomentum() const {
    return linearMomentum;
}

// Set the linear momentum of the body
inline void BodyState::setLinearMomentum(const Vector3D& linearMomentum) {
    this->linearMomentum = linearMomentum;
}

// Return the orientation quaternion of the body
inline Quaternion BodyState::getOrientation() const {
    return orientation;
}

// Set the orientation quaternion
inline void BodyState::setOrientation(const Quaternion& orientation) {
    this->orientation = orientation;
}

// Return the angular momentum of the body
inline Vector3D BodyState::getAngularMomentum() const {
    return angularMomentum;
}

// Set the angular momentum of the body
inline void BodyState::setAngularMomentum(const Vector3D& angularMomentum) {
    this->angularMomentum = angularMomentum;
}

// Return the angular velocity of the body
inline Vector3D BodyState::getAngularVelocity() const {
    return angularVelocity;
}

// Return the spin of the body
inline Quaternion BodyState::getSpin() const {
    return spin;
}

// Set the inverse of the mass
inline void BodyState::setMassInverse(Kilogram massInverse) {
    this->massInverse = massInverse;
}

// Set the inverse of the inertia tensor
inline void BodyState::setInertiaTensorInverse(const Matrix3x3& inertiaTensorInverse) {
    this->inertiaTensorInverse = inertiaTensorInverse;
}

}

#endif
