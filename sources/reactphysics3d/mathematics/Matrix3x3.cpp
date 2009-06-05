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
Matrix3x3::Matrix3x3(const Matrix3x3& matrix2) {
    // Copy the values in the matrix
    setAllValues(matrix2.array[0][0], matrix2.array[0][1], matrix2.array[0][2],
                 matrix2.array[1][0], matrix2.array[1][1], matrix2.array[1][2],
                 matrix2.array[2][0], matrix2.array[2][1], matrix2.array[2][2]);
}

// Create a Matrix3x3 from a quaternion (the quaternion can be non-unit)
Matrix3x3::Matrix3x3(const Quaternion& quaternion) {
    double x = quaternion.getX();
    double y = quaternion.getY();
    double z = quaternion.getZ();
    double w = quaternion.getW();

    double nQ = x*x + y*y + z*z + w*w;
    double s = 0.0;

    if (nQ > 0.0) {
        s = 2.0/nQ;
    }

    // Computations used for optimization (less multiplications)
    double xs = x*s;
    double ys = y*s;
    double zs = z*s;
    double wxs = w*xs;
    double wys = w*ys;
    double wzs = w*zs;
    double xxs = x*xs;
    double xys = x*ys;
    double xzs = x*zs;
    double yys = y*ys;
    double yzs = y*zs;
    double zzs = z*zs;

    // Create the matrix corresponding to the quaternion
    setAllValues(1.0-yys-zzs, xys-wzs, xzs + wys,
                 xys + wzs, 1.0-xxs-zzs, yzs-wxs,
                 xzs-wys, yzs + wxs, 1.0-xxs-yys);
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

// Return the quaternion corresponding to the matrix (it returns a unit quaternion)
Quaternion Matrix3x3::getQuaternion() const {

    // Get the trace of the matrix
    double trace = getTrace();

    double r;
    double s;

    if (trace < 0.0) {
        if (array[1][1] > array[0][0]) {
            if(array[2][2] > array[1][1]) {
                r = sqrt(array[2][2] - array[0][0] - array[1][1] + 1.0);
                s = 0.5 / r;
                return Quaternion((array[2][0] + array[0][2])*s, (array[1][2] + array[2][1])*s, 0.5*r, (array[1][0] - array[0][1])*s);
            }
            else {
                r = sqrt(array[1][1] - array[2][2] - array[0][0] + 1.0);
                s = 0.5 / r;
                return Quaternion((array[0][1] + array[1][0])*s, 0.5 * r, (array[1][2] + array[2][1])*s, (array[0][2] - array[2][0])*s);
            }
        }
        else if (array[2][2] > array[0][0]) {
            r = sqrt(array[2][2] - array[0][0] - array[1][1] + 1.0);
            s = 0.5 / r;
            return Quaternion((array[2][0] + array[0][2])*s, (array[1][2] + array[2][1])*s, 0.5 * r, (array[1][0] - array[0][1])*s);
        }
        else {
            r = sqrt(array[0][0] - array[1][1] - array[2][2] + 1.0);
            s = 0.5 / r;
            return Quaternion(0.5 * r, (array[0][1] + array[1][0])*s, (array[2][0] - array[0][2])*s, (array[2][1] - array[1][2])*s);
        }
    }
    else {
        r = sqrt(trace + 1.0);
        s = 0.5/r;
        return Quaternion((array[2][1]-array[1][2])*s, (array[0][2]-array[2][0])*s, (array[1][0]-array[0][1])*s, 0.5 * r);
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



