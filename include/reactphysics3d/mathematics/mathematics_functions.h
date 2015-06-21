/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2015 Daniel Chappuis                                       *
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
#include <algorithm>
#include "reactphysics3d/configuration.h"
#include "reactphysics3d/decimal.h"

/// ReactPhysics3D Namespace
namespace reactphysics3d {

// ---------- Mathematics functions ---------- //

/// Function to test if two real numbers are (almost) equal
/// We test if two numbers a and b are such that (a-b) are in [-EPSILON; EPSILON]
inline bool approxEqual(decimal a, decimal b, decimal epsilon = MACHINE_EPSILON) {

    decimal difference = a - b;
    return (difference < epsilon && difference > -epsilon);
}

/// Function that returns the result of the "value" clamped by
/// two others values "lowerLimit" and "upperLimit"
inline decimal clamp(decimal value, decimal lowerLimit, decimal upperLimit) {
    assert(lowerLimit <= upperLimit);
    return std::min(std::max(value, lowerLimit), upperLimit);
}

}



#endif
