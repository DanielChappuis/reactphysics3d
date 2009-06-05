/****************************************************************************
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

#ifndef DERIVATIVEBODYSTATE_H
#define DERIVATIVEBODYSTATE_H

// Libraries
#include "../mathematics/mathematics.h"

// Namespace ReactPhysics3D
namespace reactphysics3d {

/*  -------------------------------------------------------------------
    Class DerivativeBodyState :
        This class represents a derivative of a body state at time t.
        This class is used in the numerical integrator to compute the
        derivative of a body state at different times.
    -------------------------------------------------------------------
*/
class DerivativeBodyState {
    private :
        Vector3D linearVelocity;            // Linear velocity of the body
        Vector3D force;                     // Force applied to the body
        Vector3D torque;                    // Torque applied to the body
        Quaternion spin;                    // Quaternion spin of the body

    public :
        DerivativeBodyState(const Vector3D& linearVelocity, const Vector3D& force, const Vector3D& torque,
                            const Quaternion& spin);                                                        // Constructor
        DerivativeBodyState(const DerivativeBodyState& derivativeBodyState);                                // Copy-constructor
        virtual ~DerivativeBodyState();                                                                     // Destructor

        Vector3D getLinearVelocity() const;         // Return the linear velocity
        Vector3D getForce() const;                  // Return the force
        Vector3D getTorque() const;                 // Return the torque
        Quaternion getSpin() const;                 // Return the spin
};

// --- Inline functions --- //

// Return the linear velocity
inline Vector3D DerivativeBodyState::getLinearVelocity() const {
    return linearVelocity;
}

// Return the force
inline Vector3D DerivativeBodyState::getForce() const {
    return force;
}

// Return the torque
inline Vector3D DerivativeBodyState::getTorque() const {
    return torque;
}

// Return the spin
inline Quaternion DerivativeBodyState::getSpin() const {
    return spin;
}

}

#endif
