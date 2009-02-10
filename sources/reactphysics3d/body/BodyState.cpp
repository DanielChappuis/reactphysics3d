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

 // Libraries
 #include "BodyState.h"

 // We want to use the ReactPhysics3D namespace
 using namespace reactphysics3d;

 // Constructor
 BodyState::BodyState(const Vector3D& position, const Matrix3x3& inertiaTensorInverse, const Kilogram& massInverse)
           : position(position), linearMomentum(Vector3D()), orientation(Quaternion(1,0,0, 0)), angularMomentum(Vector3D()),
             linearVelocity(Vector3D()), angularVelocity(Vector3D()), spin(Quaternion()), inertiaTensorInverse(inertiaTensorInverse),
             massInverse(massInverse) {
    // TODO : orientation will be initialized in another way
    // TODO : linearMomentum will be initialized in another way
    // TODO : angularMomentum will be initialize in another way
    // TODO : linearVelocity will be initialize in another way
    // TODO : angularVelocity will be initialize in another way
    // TODO : spin will be initialize in another way
 }

// Copy-constructor
BodyState::BodyState(const BodyState& bodyState)
          : position(bodyState.position), linearMomentum(bodyState.linearMomentum), orientation(bodyState.orientation),
            angularMomentum(bodyState.angularMomentum), linearVelocity(bodyState.linearVelocity),
            angularVelocity(bodyState.angularVelocity), spin(bodyState.spin), inertiaTensorInverse(bodyState.inertiaTensorInverse),
            massInverse(bodyState.massInverse) {
}

// Destructor
BodyState::~BodyState() {

}

// Recalculate the secondary values of the BodyState when the primary values have changed
void BodyState::recalculate() {
    // Compute the linear velocity
    linearVelocity = linearMomentum * massInverse.getValue();

    // Compute the angular velocity
    angularVelocity = inertiaTensorInverse * angularMomentum;

    // Normalize the orientation quaternion
    orientation = orientation.getUnit();

    // Compute the spin quaternion
    spin = Quaternion(0, angularVelocity.getX(), angularVelocity.getY(), angularVelocity.getZ()) * orientation * 0.5;
}

// Overloaded operator for the multiplication with a number
BodyState BodyState::operator*(double number) const {
    // TODO : Implement this method
}
