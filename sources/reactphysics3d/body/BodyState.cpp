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
           : position(position), inertiaTensorInverse(inertiaTensorInverse), massInverse(massInverse) {

 }

// Copy-constructor
BodyState::BodyState(const BodyState& bodyState)
          : position(bodyState.position), linearMomentum(bodyState.linearMomentum), linearVelocity(bodyState.linearVelocity),
            angularVelocity(bodyState.angularVelocity), spin(bodyState.spin), inertiaTensorInverse(bodyState.inertiaTensorInverse),
            massInverse(bodyState.massInverse) {

}

// Destructor
BodyState::~BodyState() {

}

// Recalculate the secondary values of the BodyState when the primary values have changed
void BodyState::recalculate() {
    // TODO : Implement this method
}

