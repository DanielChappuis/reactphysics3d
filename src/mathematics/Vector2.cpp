/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2024 Daniel Chappuis                                       *
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
#include <reactphysics3d/mathematics/Vector2.h>

// Namespaces
using namespace reactphysics3d;

// Return the corresponding unit vector
Vector2 Vector2::getUnit() const {
    decimal lengthVector = length();

    if (lengthVector < MACHINE_EPSILON) {
        return *this;
    }

    // Compute and return the unit vector
    decimal lengthInv = decimal(1.0) / lengthVector;
    return Vector2(x * lengthInv, y * lengthInv);
}

// Return one unit orthogonal vector of the current vector
Vector2 Vector2::getOneUnitOrthogonalVector() const {

    decimal l = length();
    assert(l > MACHINE_EPSILON);

    return Vector2(-y / l, x / l);
}
