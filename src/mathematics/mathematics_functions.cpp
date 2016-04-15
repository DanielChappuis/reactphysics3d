/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2016 Daniel Chappuis                                       *
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
#include "mathematics_functions.h"
#include "Vector3.h"

using namespace reactphysics3d;

/// Compute the barycentric coordinates u, v, w of a point p inside the triangle (a, b, c)
/// This method uses the technique described in the book Real-Time collision detection by
/// Christer Ericson.
void reactphysics3d::computeBarycentricCoordinatesInTriangle(const Vector3& a, const Vector3& b, const Vector3& c,
                                             const Vector3& p, decimal& u, decimal& v, decimal& w) {
    const Vector3 v0 = b - a;
    const Vector3 v1 = c - a;
    const Vector3 v2 = p - a;

    decimal d00 = v0.dot(v0);
    decimal d01 = v0.dot(v1);
    decimal d11 = v1.dot(v1);
    decimal d20 = v2.dot(v0);
    decimal d21 = v2.dot(v1);

    decimal denom = d00 * d11 - d01 * d01;
    v = (d11 * d20 - d01 * d21) / denom;
    w = (d00 * d21 - d01 * d20) / denom;
    u = decimal(1.0) - v - w;
}

// Clamp a vector such that it is no longer than a given maximum length
Vector3 reactphysics3d::clamp(const Vector3& vector, decimal maxLength) {
    if (vector.lengthSquare() > maxLength * maxLength) {
        return vector.getUnit() * maxLength;
    }
    return vector;
}
