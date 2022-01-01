/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2022 Daniel Chappuis                                       *
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
#include <reactphysics3d/mathematics/Matrix2x2.h>

using namespace reactphysics3d;

// Return the inverse matrix
Matrix2x2 Matrix2x2::getInverse(decimal determinant) const {

    // Check if the determinant is equal to zero
    assert(std::abs(determinant) > MACHINE_EPSILON);

    decimal invDeterminant = decimal(1.0) / determinant;

    Matrix2x2 tempMatrix(mRows[1][1], -mRows[0][1], -mRows[1][0], mRows[0][0]);

    // Return the inverse matrix
    return (invDeterminant * tempMatrix);
}
