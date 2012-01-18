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
#include <iostream>
#include "Matrix3x3.h"

// Namespaces
using namespace reactphysics3d;

// Constructor of the class Matrix3x3
Matrix3x3::Matrix3x3() {
    // Initialize all values in the matrix to zero
    setAllValues(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
}

// Constructor
Matrix3x3::Matrix3x3(decimal value) {
    setAllValues(value, value, value, value, value, value, value, value, value);
}

// Constructor with arguments
Matrix3x3::Matrix3x3(decimal a1, decimal a2, decimal a3, decimal b1, decimal b2, decimal b3, decimal c1, decimal c2, decimal c3) {
    // Initialize the matrix with the values
    setAllValues(a1, a2, a3, b1, b2, b3, c1, c2, c3);
}

// Destructor
Matrix3x3::~Matrix3x3() {

}

// Return the inverse matrix
Matrix3x3 Matrix3x3::getInverse() const {
    // Compute the determinant of the matrix
    decimal determinant = getDeterminant();

    // Check if the determinant is equal to zero
    assert(determinant != 0.0);
    decimal invDeterminant = 1.0 / determinant;
    Matrix3x3 tempMatrix;

    // Compute the inverse of the matrix
    tempMatrix.setAllValues((array[1][1]*array[2][2]-array[2][1]*array[1][2]), -(array[1][0]*array[2][2]-array[2][0]*array[1][2]), (array[1][0]*array[2][1]-array[2][0]*array[1][1]),
                            -(array[0][1]*array[2][2]-array[2][1]*array[0][2]), (array[0][0]*array[2][2]-array[2][0]*array[0][2]), -(array[0][0]*array[2][1]-array[2][0]*array[0][1]),
                            (array[0][1]*array[1][2]-array[0][2]*array[1][1]), -(array[0][0]*array[1][2]-array[1][0]*array[0][2]), (array[0][0]*array[1][1]-array[0][1]*array[1][0]));

    // Return the inverse matrix
    return (invDeterminant * tempMatrix.getTranspose());
}



