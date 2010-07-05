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
 #include "BodyState.h"
 #include "OBB.h"
 #include "../mathematics/mathematics.h"
 #include "../physics/physics.h"

// Namespace reactphysics3d
namespace reactphysics3d {

// TODO : Check if it is possible to put all the attributes from BodyState class in the RigidBody class (to do this, the
//        class RungeKutta have to be changed a lot

/*  -------------------------------------------------------------------
    Class RigidBody :
        This class represents a rigid body of the physics
        engine. A rigid body is a non-deformable body that
        has a constant mass.
    -------------------------------------------------------------------
*/
class RigidBody : public Body {
    protected :
        Matrix3x3 inertiaTensorLocal;               // Local inertia tensor of the body (in body coordinates)
        BodyState currentBodyState;                 // Current body state
        BodyState previousBodyState;                // Previous body state
        double interpolationFactor;                 // Interpolation factor used for the state interpolation
        double restitution;                         // Coefficient of restitution (between 0 and 1), 1 for a very boucing body
        OBB obb;                                    // Oriented bounding box that contains the rigid body

    public :
        RigidBody(const Vector3D& position, const Quaternion& orientation, const Kilogram& mass, const Matrix3x3& inertiaTensorLocal, const OBB& obb);  // Constructor
        RigidBody(const RigidBody& rigidBody);                                                                                                          // Copy-constructor
        virtual ~RigidBody();                                                                                                                           // Destructor

        Matrix3x3 getInertiaTensorLocal() const;                            // Return the local inertia tensor of the body (in body coordinates)
        void setInertiaTensorLocal(const Matrix3x3& inertiaTensorLocal);    // Set the local inertia tensor of the body (in body coordinates)
        Matrix3x3 getInertiaTensorWorld() const;                            // Return the inertia tensor in world coordinates
        Matrix3x3 getInertiaTensorInverseWorld() const;                     // Return the inverse of the inertia tensor in world coordinates
        BodyState& getCurrentBodyState();                                   // Return a reference to the current state of the body
        BodyState& getPreviousBodyState();                                  // TODO : DELETE THIS
        void setInterpolationFactor(double factor);                         // Set the interpolation factor of the body
        BodyState getInterpolatedState() const;                             // Compute and return the interpolated state
        void setLinearVelocity(const Vector3D& linearVelocity);             // Set the linear velocity of the rigid body
        double getRestitution() const;                                      // Get the restitution coefficient
        void setRestitution(double restitution);                            // Set the restitution coefficient
        void updatePreviousBodyState();                                     // Update the previous body state of the body
        const OBB* const getOBB() const;                                    // Return the oriented bounding box of the rigid body
        void update();                                                      // Update the rigid body in order to reflect a change in the body state
};

// --- Inline functions --- //

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
    return currentBodyState.getOrientation().getMatrix() * inertiaTensorLocal * currentBodyState.getOrientation().getMatrix().getTranspose();
}

// Return the inverse of the inertia tensor in world coordinates
// The inertia tensor I_w in world coordinates in computed with the local inverse inertia tensor I_b^-1 in body coordinates
// by I_w = R * I_b^-1 * R^T
// where R is the rotation matrix (and R^T its transpose) of the current orientation quaternion of the body
inline Matrix3x3 RigidBody::getInertiaTensorInverseWorld() const {
    // Compute and return the inertia tensor in world coordinates
    return currentBodyState.getOrientation().getMatrix() * currentBodyState.getInertiaTensorInverse() * currentBodyState.getOrientation().getMatrix().getTranspose();
}

// Return a reference to the current state of the body
// This way the currentBodyState could be modify outside the rigid body
inline BodyState& RigidBody::getCurrentBodyState() {
    return currentBodyState;
}

// TODO : DELETE THIS
inline BodyState& RigidBody::getPreviousBodyState() {
    return previousBodyState;
}


// Set the interpolation factor of the body
inline void RigidBody::setInterpolationFactor(double factor) {
    assert(factor >= 0 && factor <= 1);

    // Set the factor
    interpolationFactor = factor;
}

// Set the linear velocity of the rigid body
inline void RigidBody::setLinearVelocity(const Vector3D& linearVelocity) {
    // Update the linear velocity of the current body state
    currentBodyState.setLinearVelocity(linearVelocity);

    // Update the linear momentum of the current body state
    currentBodyState.setLinearMomentum(linearVelocity * (1.0/currentBodyState.getMassInverse().getValue()));
}

// Get the restitution coeffficient of the rigid body
inline double RigidBody::getRestitution() const {
    return restitution;
}

// Set the restitution coefficient
inline void RigidBody::setRestitution(double restitution) {
    this->restitution = restitution;
}

// Update the previous body state of the body
inline void RigidBody::updatePreviousBodyState() {
    // The current body state becomes the previous body state
    previousBodyState = currentBodyState;
}

// Return the oriented bounding box of the rigid body
inline const OBB* const RigidBody::getOBB() const {
    return &obb;
}

// Update the rigid body in order to reflect a change in the body state
inline void RigidBody::update() {
    // Update the orientation of the corresponding bounding volume of the rigid body
    obb.updateOrientation(currentBodyState.getPosition(), currentBodyState.getOrientation());
}

} // End of the ReactPhyscis3D namespace

 #endif
