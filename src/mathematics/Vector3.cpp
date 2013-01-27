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
    mValues[0] = 0.0;
    mValues[1] = 0.0;
    mValues[2] = 0.0;
}

// Constructor with arguments
Vector3::Vector3(decimal x, decimal y, decimal z) {
    mValues[0] = x;
    mValues[1] = y;
    mValues[2] = z;
}

// Copy-constructor
Vector3::Vector3(const Vector3& vector) {
    mValues[0] = vector.mValues[0];
    mValues[1] = vector.mValues[1];
    mValues[2] = vector.mValues[2];
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
    return Vector3(mValues[0] * lengthInv, mValues[1] * lengthInv, mValues[2] * lengthInv);
}

// Return two unit orthogonal vectors of the current vector
Vector3 Vector3::getOneOrthogonalVector() const {
    assert(!this->isZero());

    // Compute a first orthogonal vector
    Vector3 vector1;
    if (!approxEqual(mValues[0], 0.0)) {   // If x != 0
        vector1.setY(mValues[0]);
        vector1.setZ((-2*mValues[0]*mValues[1]*mValues[2] +
                      2*mValues[0]*mValues[2])/(2*(mValues[2]*mValues[2] + mValues[0]*mValues[0])));
        vector1.setX((-mValues[0]*mValues[1]-mValues[2]*vector1.getZ())/mValues[0]);
    }
    else if (!approxEqual(mValues[1], 0.0)) { // If y != 0
        vector1.setZ(mValues[1]);
        vector1.setX((-2*mValues[0]*mValues[1]*mValues[2] +
                      2*mValues[0]*mValues[1])/(2*(mValues[1]*mValues[1] + mValues[0]*mValues[0])));
        vector1.setY((-mValues[2]*mValues[1]-mValues[0]*vector1.getX())/mValues[1]);
    }
    else if (!approxEqual(mValues[2], 0.0)) { // If z != 0
        vector1.setX(mValues[2]);
        vector1.setY((-2*mValues[0]*mValues[1]*mValues[2] +
                      2*mValues[1]*mValues[2])/(2*(mValues[2]*mValues[2] + mValues[1]*mValues[1])));
        vector1.setZ((-mValues[0]*mValues[2]-mValues[1]*vector1.getY())/mValues[2]);
    }

    //assert(vector1.isUnit());
    return vector1;
}

// Return one unit orthogonal vector of the current vector
Vector3 Vector3::getOneUnitOrthogonalVector() const {
    assert(!this->isZero());

    decimal x = mValues[0];
    decimal y = mValues[1];
    decimal z = mValues[2];

    // Get the minimum element of the vector
    Vector3 vectorAbs(fabs(x), fabs(y), fabs(z));
    int minElement = vectorAbs.getMinAxis();

    if (minElement == 0) {
        return Vector3(0.0, -z, y) / sqrt(y*y + z*z);
    }
    else if (minElement == 1) {
        return Vector3(-z, 0.0, x) / sqrt(x*x + z*z);
    }
    else {
        return Vector3(-y, x, 0.0) / sqrt(x*x + y*y);
    }

}
