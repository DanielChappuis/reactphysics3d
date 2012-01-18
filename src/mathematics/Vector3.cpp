/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2010-2012 Daniel Chappuis                                       *
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

// Libraries
#include "Vector3.h"
#include <iostream>
#include <cassert>
#include <vector>

// Namespaces
using namespace reactphysics3d;

// Constructor of the class Vector3D
Vector3::Vector3() {
    values[0] = 0.0;
    values[1] = 0.0;
    values[2] = 0.0;
}

// Constructor with arguments
Vector3::Vector3(decimal x, decimal y, decimal z) {
    values[0] = x;
    values[1] = y;
    values[2] = z;
}

// Copy-constructor
Vector3::Vector3(const Vector3& vector) {
    values[0] = vector.values[0];
    values[1] = vector.values[1];
    values[2] = vector.values[2];
}

// Destructor
Vector3::~Vector3() {

}

// Return the corresponding unit vector
Vector3 Vector3::getUnit() const {
    decimal lengthVector = length();

    assert(lengthVector != 0.0);

    // Compute and return the unit vector
    decimal lengthInv = 1.0 / lengthVector;
    return Vector3(values[0] * lengthInv, values[1] * lengthInv, values[2] * lengthInv);
}

// Return two unit orthogonal vectors of the current vector
Vector3 Vector3::getOneOrthogonalVector() const {
    assert(!this->isZero());

    // Compute a first orthogonal vector
    Vector3 vector1;
    if (!approxEqual(values[0], 0.0)) {   // If x != 0
        vector1.setY(values[0]);
        vector1.setZ((-2*values[0]*values[1]*values[2] + 2*values[0]*values[2])/(2*(values[2]*values[2] + values[0]*values[0])));
        vector1.setX((-values[0]*values[1]-values[2]*vector1.getZ())/values[0]);
    }
    else if (!approxEqual(values[1], 0.0)) { // If y != 0
        vector1.setZ(values[1]);
        vector1.setX((-2*values[0]*values[1]*values[2] + 2*values[0]*values[1])/(2*(values[1]*values[1] + values[0]*values[0])));
        vector1.setY((-values[2]*values[1]-values[0]*vector1.getX())/values[1]);
    }
    else if (!approxEqual(values[2], 0.0)) { // If z != 0
        vector1.setX(values[2]);
        vector1.setY((-2*values[0]*values[1]*values[2] + 2*values[1]*values[2])/(2*(values[2]*values[2] + values[1]*values[1])));
        vector1.setZ((-values[0]*values[2]-values[1]*vector1.getY())/values[2]);
    }

    //assert(vector1.isUnit());
    return vector1;
}
