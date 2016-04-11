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

#ifndef REACTPHYSICS3D_MATHEMATICS_FUNCTIONS_H
#define REACTPHYSICS3D_MATHEMATICS_FUNCTIONS_H

// Libraries
#include "configuration.h"
#include "decimal.h"
#include <algorithm>
#include <cassert>
#include <cmath>

/// ReactPhysics3D namespace
namespace reactphysics3d {

struct Vector3;

// ---------- Mathematics functions ---------- //

/// Function to test if two real numbers are (almost) equal
/// We test if two numbers a and b are such that (a-b) are in [-EPSILON; EPSILON]
inline bool approxEqual(decimal a, decimal b, decimal epsilon = MACHINE_EPSILON) {
    return (std::fabs(a - b) < epsilon);
}

/// Function that returns the result of the "value" clamped by
/// two others values "lowerLimit" and "upperLimit"
inline int clamp(int value, int lowerLimit, int upperLimit) {
    assert(lowerLimit <= upperLimit);
    return std::min(std::max(value, lowerLimit), upperLimit);
}

/// Function that returns the result of the "value" clamped by
/// two others values "lowerLimit" and "upperLimit"
inline decimal clamp(decimal value, decimal lowerLimit, decimal upperLimit) {
    assert(lowerLimit <= upperLimit);
    return std::min(std::max(value, lowerLimit), upperLimit);
}

/// Return the minimum value among three values
inline decimal min3(decimal a, decimal b, decimal c) {
    return std::min(std::min(a, b), c);
}

/// Return the maximum value among three values
inline decimal max3(decimal a, decimal b, decimal c) {
    return std::max(std::max(a, b), c);
}

/// Return true if two values have the same sign
inline bool sameSign(decimal a, decimal b) {
    return a * b >= decimal(0.0);
}

/// Clamp a vector such that it is no longer than a given maximum length
Vector3 clamp(const Vector3& vector, decimal maxLength);

/// Compute the barycentric coordinates u, v, w of a point p inside the triangle (a, b, c)
void computeBarycentricCoordinatesInTriangle(const Vector3& a, const Vector3& b, const Vector3& c,
                                             const Vector3& p, decimal& u, decimal& v, decimal& w);

}

#endif
