/****************************************************************************
 * Copyright (C) 2009      Daniel Chappuis                                  *
 ****************************************************************************
 * This file is part of ReactPhysics3D.                                     *
 *                                                                          *
 * ReactPhysics3D is free software: you can redistribute it and/or modify   *
 * it under the terms of the GNU Lesser General Public License as published *
 * by the Free Software Foundation, either version 3 of the License, or     *
 * (at your option) any later version.                                      *
 *                                                                          *
 * ReactPhysics3D is distributed in the hope that it will be useful,        *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of           *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the             *
 * GNU Lesser General Public License for more details.                      *
 *                                                                          *
 * You should have received a copy of the GNU Lesser General Public License *
 * along with ReactPhysics3D. If not, see <http://www.gnu.org/licenses/>.   *
 ***************************************************************************/

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

// Constructor with arguments
Matrix3x3::Matrix3x3(double a1, double a2, double a3, double b1, double b2, double b3, double c1, double c2, double c3) {
    // Initialize the matrix with the values
    setAllValues(a1, a2, a3, b1, b2, b3, c1, c2, c3);
}

// Copy-constructor
// TODO : Test if this copy-constructor is correct (check if the the copy matrix use
//        the same memory place for its array)
Matrix3x3::Matrix3x3(const Matrix3x3& matrix2) {
    // Copy the values in the matrix
    setAllValues(matrix2.array[0][0], matrix2.array[0][1], matrix2.array[0][2],
                 matrix2.array[1][0], matrix2.array[1][1], matrix2.array[1][2],
                 matrix2.array[2][0], matrix2.array[2][1], matrix2.array[2][2]);
}

// Destructor
Matrix3x3::~Matrix3x3() {

}

// Return the inverse matrix
Matrix3x3 Matrix3x3::getInverse() const throw(MathematicsException) {
    // Compute the determinant of the matrix
    double determinant = getDeterminant();

    // Check if the determinant is equal to zero
    if (determinant != 0) {
        double invDeterminant = 1.0 / determinant;
        Matrix3x3 tempMatrix;

        // Compute the inverse of the matrix
        tempMatrix.setAllValues((array[1][1]*array[2][2]-array[2][1]*array[1][2]), -(array[1][0]*array[2][2]-array[2][0]*array[1][2]), (array[1][0]*array[2][1]-array[2][0]*array[1][1]),
                                -(array[0][1]*array[2][2]-array[2][1]*array[0][2]), (array[0][0]*array[2][2]-array[2][0]*array[0][2]), -(array[0][0]*array[2][1]-array[2][0]*array[0][1]),
                                (array[0][1]*array[1][2]-array[0][2]*array[1][1]), -(array[0][0]*array[1][2]-array[1][0]*array[0][2]), (array[0][0]*array[1][1]-array[0][1]*array[1][0]));

        // Return the inverse matrix
        return (invDeterminant * tempMatrix.getTranspose());
    }
    else {
        // Throw an exception because the inverse of the matrix doesn't exist if the determinant is equal to zero
        throw MathematicsException("MathematicsException : Impossible to compute the inverse of the matrix because the determinant is equal to zero");
    }
}

// Return the 3x3 identity matrix
Matrix3x3 Matrix3x3::identity() {
    // Return the isdentity matrix
    return Matrix3x3(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
}

// Overloaded operator for addition
Matrix3x3 Matrix3x3::operator+(const Matrix3x3& matrix2) const {
    // Return the sum matrix
    return Matrix3x3(array[0][0] + matrix2.array[0][0], array[0][1] + matrix2.array[0][1], array[0][2] + matrix2.array[0][2],
                     array[1][0] + matrix2.array[1][0], array[1][1] + matrix2.array[1][1], array[1][2] + matrix2.array[1][2],
                     array[2][0] + matrix2.array[2][0], array[2][1] + matrix2.array[2][1], array[2][2] + matrix2.array[2][2]);
}

// Overloaded operator for substraction
Matrix3x3 Matrix3x3::operator-(const Matrix3x3& matrix2) const {
    // Return the substraction matrix
    return Matrix3x3(array[0][0] - matrix2.array[0][0], array[0][1] - matrix2.array[0][1], array[0][2] - matrix2.array[0][2],
                     array[1][0] - matrix2.array[1][0], array[1][1] - matrix2.array[1][1], array[1][2] - matrix2.array[1][2],
                     array[2][0] - matrix2.array[2][0], array[2][1] - matrix2.array[2][1], array[2][2] - matrix2.array[2][2]);
}

// Overloaded operator for multiplication with a number
Matrix3x3 Matrix3x3::operator*(double nb) const {
    // Return multiplied matrix
    return Matrix3x3(array[0][0] * nb, array[0][1] * nb, array[0][2] * nb,
                     array[1][0] * nb, array[1][1] * nb, array[1][2] * nb,
                     array[2][0] * nb, array[2][1] * nb, array[2][2] * nb);
}

// Overloaded operator for multiplication with a matrix
Matrix3x3 Matrix3x3::operator*(const Matrix3x3& matrix2) const {
    // Compute and return the multiplication of the matrices
    return Matrix3x3(array[0][0]*matrix2.array[0][0] + array[0][1]*matrix2.array[1][0] + array[0][2]*matrix2.array[2][0],
                     array[0][0]*matrix2.array[0][1] + array[0][1]*matrix2.array[1][1] + array[0][2]*matrix2.array[2][1],
                     array[0][0]*matrix2.array[0][2] + array[0][1]*matrix2.array[1][2] + array[0][2]*matrix2.array[2][2],
                     array[1][0]*matrix2.array[0][0] + array[1][1]*matrix2.array[1][0] + array[1][2]*matrix2.array[2][0],
                     array[1][0]*matrix2.array[0][1] + array[1][1]*matrix2.array[1][1] + array[1][2]*matrix2.array[2][1],
                     array[1][0]*matrix2.array[0][2] + array[1][1]*matrix2.array[1][2] + array[1][2]*matrix2.array[2][2],
                     array[2][0]*matrix2.array[0][0] + array[2][1]*matrix2.array[1][0] + array[2][2]*matrix2.array[2][0],
                     array[2][0]*matrix2.array[0][1] + array[2][1]*matrix2.array[1][1] + array[2][2]*matrix2.array[2][1],
                     array[2][0]*matrix2.array[0][2] + array[2][1]*matrix2.array[1][2] + array[2][2]*matrix2.array[2][2]);
}

// Overloaded operator for assignment
Matrix3x3& Matrix3x3::operator=(const Matrix3x3& matrix2) {
    // Check for self-assignment
    if (this != &matrix2) {
        setAllValues(matrix2.array[0][0], matrix2.array[0][1], matrix2.array[0][2],
                 matrix2.array[1][0], matrix2.array[1][1], matrix2.array[1][2],
                 matrix2.array[2][0], matrix2.array[2][1], matrix2.array[2][2]);
    }

    // Return a reference to the matrix
    return *this;
}



