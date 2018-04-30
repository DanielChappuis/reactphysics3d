/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2018 Daniel Chappuis                                       *
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

// Namespaces
using namespace reactphysics3d;

// Return the corresponding unit vector
Vector3 Vector3::getUnit() const {
    decimal lengthVector = length();

    if (lengthVector < MACHINE_EPSILON) {
        return *this;
    }

    // Compute and return the unit vector
    decimal lengthInv = decimal(1.0) / lengthVector;
    return Vector3(x * lengthInv, y * lengthInv, z * lengthInv);
}

// Return one unit orthogonal vector of the current vector
Vector3 Vector3::getOneUnitOrthogonalVector() const {

    assert(length() > MACHINE_EPSILON);

    // Get the minimum element of the vector
    Vector3 vectorAbs(std::fabs(x), std::fabs(y), std::fabs(z));
    int minElement = vectorAbs.getMinAxis();

    if (minElement == 0) {
        return Vector3(0.0, -z, y) / std::sqrt(y*y + z*z);
    }
    else if (minElement == 1) {
        return Vector3(-z, 0.0, x) / std::sqrt(x*x + z*z);
    }
    else {
        return Vector3(-y, x, 0.0) / std::sqrt(x*x + y*y);
    }

}
