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
#include "Quaternion.h"
#include <cassert>

// Namespaces
using namespace reactphysics3d;

// Constructor of the class
Quaternion::Quaternion()
           :x(0.0), y(0.0), z(0.0), w(0.0) {

}

// Constructor with arguments
Quaternion::Quaternion(double x, double y, double z, double w)
           :x(x), y(y), z(z), w(w) {

}

// Constructor with the component w and the vector v=(x y z)
Quaternion::Quaternion(double w, const Vector3D& v)
           :x(v.getX()), y(v.getY()), z(v.getZ()), w(w) {

}

// Copy-constructor
Quaternion::Quaternion(const Quaternion& quaternion)
           :x(quaternion.x), y(quaternion.y), z(quaternion.z), w(quaternion.w) {

}

// Destructor
Quaternion::~Quaternion() {

}

// --- Others functions --- //

// Compute the spherical linear interpolation between two quaternions.
// The t argument has to be such that 0 <= t <= 1
Quaternion slerp(const Quaternion& quaternion1, const Quaternion& quaternion2, double t) {
    //TODO : Implement this method
    assert(t >= 0 && t <= 1);
}


