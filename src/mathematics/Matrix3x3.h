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


#ifndef MATRIX3X3_H
#define MATRIX3X3_H

// Libraries
#include <cassert>
#include "Vector3.h"

// ReactPhysics3D namespace
namespace reactphysics3d {


/*  -------------------------------------------------------------------
    Class Matrix3x3 :
        This class represents a 3x3 matrix.
    -------------------------------------------------------------------
*/
class Matrix3x3 {
    private :
        decimal array[3][3];                                                                    // Array with the values of the matrix

    public :
        Matrix3x3();                                                                            // Constructor
        Matrix3x3(decimal value);                                                               // Constructor
        Matrix3x3(decimal a1, decimal a2, decimal a3, decimal b1, decimal b2, decimal b3,
                  decimal c1, decimal c2, decimal c3);                                          // Constructor
        virtual ~Matrix3x3();                                                                   // Destructor

        decimal getValue(int i, int j) const;                                                   // Get a value in the matrix
        void setValue(int i, int j, decimal value);                                             // Set a value in the matrix
        void setAllValues(decimal a1, decimal a2, decimal a3, decimal b1, decimal b2, decimal b3,
                  decimal c1, decimal c2, decimal c3);                                          // Set all the values in the matrix
        Vector3 getColumn(int i) const;                                                         // Return a column
        Matrix3x3 getTranspose() const;                                                         // Return the transpose matrix
        decimal getDeterminant() const;                                                         // Return the determinant of the matrix
        decimal getTrace() const;                                                               // Return the trace of the matrix
        Matrix3x3 getInverse() const;                                                           // Return the inverse matrix
        Matrix3x3 getAbsoluteMatrix() const;                                                    // Return the matrix with absolute values
        void setToIdentity();                                                                   // Set the matrix to the identity matrix
        static Matrix3x3 identity();                                                            // Return the 3x3 identity matrix

        // --- Overloaded operators --- //
        friend Matrix3x3 operator+(const Matrix3x3& matrix1, const Matrix3x3& matrix2);         // Overloaded operator for addition
        friend Matrix3x3 operator-(const Matrix3x3& matrix1, const Matrix3x3& matrix2);         // Overloaded operator for substraction
        friend Matrix3x3 operator-(const Matrix3x3& matrix);                                    // Overloaded operator for the negative of the matrix
        friend Matrix3x3 operator*(decimal nb, const Matrix3x3& matrix);                        // Overloaded operator for multiplication with a number
        friend Matrix3x3 operator*(const Matrix3x3& matrix, decimal nb);                        // Overloaded operator for multiplication with a matrix
        friend Matrix3x3 operator*(const Matrix3x3& matrix1, const Matrix3x3& matrix2);         // Overloaded operator for matrix multiplication
        friend Vector3 operator*(const Matrix3x3& matrix, const Vector3& vector);               // Overloaded operator for multiplication with a vector

        bool operator==(const Matrix3x3& matrix) const;                                         // Overloaded operator for equality condition
        bool operator!= (const Matrix3x3& matrix) const;                                        // Overloaded operator for the is different condition
        Matrix3x3& operator+=(const Matrix3x3& matrix);                                         // Overloaded operator for addition with assignment
        Matrix3x3& operator-=(const Matrix3x3& matrix);                                         // Overloaded operator for substraction with assignment
        Matrix3x3& operator*=(decimal nb);                                                      // Overloaded operator for multiplication with a number with assignment
};


// Method to get a value in the matrix (inline)
inline decimal Matrix3x3::getValue(int i, int j) const {
    assert(i>=0 && i<3 && j>=0 && j<3);
    return array[i][j];
}

// Method to set a value in the matrix (inline)
inline void Matrix3x3::setValue(int i, int j, decimal value) {
    assert(i>=0 && i<3 && j>=0 && j<3);
    array[i][j] = value;
}

// Method to set all the values in the matrix
inline void Matrix3x3::setAllValues(decimal a1, decimal a2, decimal a3, decimal b1, decimal b2, decimal b3,
                                    decimal c1, decimal c2, decimal c3) {
    array[0][0] = a1; array[0][1] = a2; array[0][2] = a3;
    array[1][0] = b1; array[1][1] = b2; array[1][2] = b3;
    array[2][0] = c1; array[2][1] = c2; array[2][2] = c3;
}

// Return a column
inline Vector3 Matrix3x3::getColumn(int i) const {
    assert(i>= 0 && i<3);
    return Vector3(array[0][i], array[1][i], array[2][i]);
}

// Return the transpose matrix
inline Matrix3x3 Matrix3x3::getTranspose() const {
    // Return the transpose matrix
    return Matrix3x3(array[0][0], array[1][0], array[2][0],
                     array[0][1], array[1][1], array[2][1],
                     array[0][2], array[1][2], array[2][2]);
}

// Return the determinant of the matrix
inline decimal Matrix3x3::getDeterminant() const {
    // Compute and return the determinant of the matrix
    return (array[0][0]*(array[1][1]*array[2][2]-array[2][1]*array[1][2]) - array[0][1]*(array[1][0]*array[2][2]-array[2][0]*array[1][2]) +
            array[0][2]*(array[1][0]*array[2][1]-array[2][0]*array[1][1]));
}

// Return the trace of the matrix
inline decimal Matrix3x3::getTrace() const {
    // Compute and return the trace
    return (array[0][0] + array[1][1] + array[2][2]);
}

// Set the matrix to the identity matrix
inline void Matrix3x3::setToIdentity() {
    array[0][0] = 1.0; array[0][1] = 0.0; array[0][2] = 0.0;
    array[1][0] = 0.0; array[1][1] = 1.0; array[1][2] = 0.0;
    array[2][0] = 0.0; array[2][1] = 0.0; array[2][2] = 1.0;
}

// Return the 3x3 identity matrix
inline Matrix3x3 Matrix3x3::identity() {
    // Return the isdentity matrix
    return Matrix3x3(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
}

// Return the matrix with absolute values
inline Matrix3x3 Matrix3x3::getAbsoluteMatrix() const {
    return Matrix3x3(fabs(array[0][0]), fabs(array[0][1]), fabs(array[0][2]),
                     fabs(array[1][0]), fabs(array[1][1]), fabs(array[1][2]),
                     fabs(array[2][0]), fabs(array[2][1]), fabs(array[2][2]));
}

// Overloaded operator for addition
inline Matrix3x3 operator+(const Matrix3x3& matrix1, const Matrix3x3& matrix2) {
    return Matrix3x3(matrix1.array[0][0] + matrix2.array[0][0], matrix1.array[0][1] + matrix2.array[0][1], matrix1.array[0][2] + matrix2.array[0][2],
                     matrix1.array[1][0] + matrix2.array[1][0], matrix1.array[1][1] + matrix2.array[1][1], matrix1.array[1][2] + matrix2.array[1][2],
                     matrix1.array[2][0] + matrix2.array[2][0], matrix1.array[2][1] + matrix2.array[2][1], matrix1.array[2][2] + matrix2.array[2][2]);
}

// Overloaded operator for substraction
inline Matrix3x3 operator-(const Matrix3x3& matrix1, const Matrix3x3& matrix2) {
    return Matrix3x3(matrix1.array[0][0] - matrix2.array[0][0], matrix1.array[0][1] - matrix2.array[0][1], matrix1.array[0][2] - matrix2.array[0][2],
                     matrix1.array[1][0] - matrix2.array[1][0], matrix1.array[1][1] - matrix2.array[1][1], matrix1.array[1][2] - matrix2.array[1][2],
                     matrix1.array[2][0] - matrix2.array[2][0], matrix1.array[2][1] - matrix2.array[2][1], matrix1.array[2][2] - matrix2.array[2][2]);
}

// Overloaded operator for the negative of the matrix
inline Matrix3x3 operator-(const Matrix3x3& matrix) {
    return Matrix3x3(-matrix.array[0][0], -matrix.array[0][1], -matrix.array[0][2],
                     -matrix.array[1][0], -matrix.array[1][1], -matrix.array[1][2],
                     -matrix.array[2][0], -matrix.array[2][1], -matrix.array[2][2]);
}

// Overloaded operator for multiplication with a number
inline Matrix3x3 operator*(decimal nb, const Matrix3x3& matrix) {
    return Matrix3x3(matrix.array[0][0] * nb, matrix.array[0][1] * nb, matrix.array[0][2] * nb,
                     matrix.array[1][0] * nb, matrix.array[1][1] * nb, matrix.array[1][2] * nb,
                     matrix.array[2][0] * nb, matrix.array[2][1] * nb, matrix.array[2][2] * nb);
}

// Overloaded operator for multiplication with a matrix
inline Matrix3x3 operator*(const Matrix3x3& matrix, decimal nb) {
    return nb * matrix;
}

// Overloaded operator for matrix multiplication
inline Matrix3x3 operator*(const Matrix3x3& matrix1, const Matrix3x3& matrix2) {
    return Matrix3x3(matrix1.array[0][0]*matrix2.array[0][0] + matrix1.array[0][1]*matrix2.array[1][0] + matrix1.array[0][2]*matrix2.array[2][0],
                     matrix1.array[0][0]*matrix2.array[0][1] + matrix1.array[0][1]*matrix2.array[1][1] + matrix1.array[0][2]*matrix2.array[2][1],
                     matrix1.array[0][0]*matrix2.array[0][2] + matrix1.array[0][1]*matrix2.array[1][2] + matrix1.array[0][2]*matrix2.array[2][2],
                     matrix1.array[1][0]*matrix2.array[0][0] + matrix1.array[1][1]*matrix2.array[1][0] + matrix1.array[1][2]*matrix2.array[2][0],
                     matrix1.array[1][0]*matrix2.array[0][1] + matrix1.array[1][1]*matrix2.array[1][1] + matrix1.array[1][2]*matrix2.array[2][1],
                     matrix1.array[1][0]*matrix2.array[0][2] + matrix1.array[1][1]*matrix2.array[1][2] + matrix1.array[1][2]*matrix2.array[2][2],
                     matrix1.array[2][0]*matrix2.array[0][0] + matrix1.array[2][1]*matrix2.array[1][0] + matrix1.array[2][2]*matrix2.array[2][0],
                     matrix1.array[2][0]*matrix2.array[0][1] + matrix1.array[2][1]*matrix2.array[1][1] + matrix1.array[2][2]*matrix2.array[2][1],
                     matrix1.array[2][0]*matrix2.array[0][2] + matrix1.array[2][1]*matrix2.array[1][2] + matrix1.array[2][2]*matrix2.array[2][2]);
}

// Overloaded operator for multiplication with a vector
inline Vector3 operator*(const Matrix3x3& matrix, const Vector3& vector) {
    return Vector3(matrix.array[0][0]*vector.getX() + matrix.array[0][1]*vector.getY() + matrix.array[0][2]*vector.getZ(),
                   matrix.array[1][0]*vector.getX() + matrix.array[1][1]*vector.getY() + matrix.array[1][2]*vector.getZ(),
                   matrix.array[2][0]*vector.getX() + matrix.array[2][1]*vector.getY() + matrix.array[2][2]*vector.getZ());
}

// Overloaded operator for equality condition
inline bool Matrix3x3::operator==(const Matrix3x3& matrix) const {
    return (array[0][0] == matrix.array[0][0] && array[0][1] == matrix.array[0][1] && array[0][2] == matrix.array[0][2] &&
            array[1][0] == matrix.array[1][0] && array[1][1] == matrix.array[1][1] && array[1][2] == matrix.array[1][2] &&
            array[2][0] == matrix.array[2][0] && array[2][1] == matrix.array[2][1] && array[2][2] == matrix.array[2][2]);
}

// Overloaded operator for the is different condition
inline bool Matrix3x3::operator!= (const Matrix3x3& matrix) const {
    return !(*this == matrix);
}

// Overloaded operator for addition with assignment
inline Matrix3x3& Matrix3x3::operator+=(const Matrix3x3& matrix) {
   array[0][0] += matrix.array[0][0]; array[0][1] += matrix.array[0][1]; array[0][2] += matrix.array[0][2];
   array[1][0] += matrix.array[1][0]; array[1][1] += matrix.array[1][1]; array[1][2] += matrix.array[1][2];
   array[2][0] += matrix.array[2][0]; array[2][1] += matrix.array[2][1]; array[2][2] += matrix.array[2][2];
   return *this;
}

// Overloaded operator for substraction with assignment
inline Matrix3x3& Matrix3x3::operator-=(const Matrix3x3& matrix) {
   array[0][0] -= matrix.array[0][0]; array[0][1] -= matrix.array[0][1]; array[0][2] -= matrix.array[0][2];
   array[1][0] -= matrix.array[1][0]; array[1][1] -= matrix.array[1][1]; array[1][2] -= matrix.array[1][2];
   array[2][0] -= matrix.array[2][0]; array[2][1] -= matrix.array[2][1]; array[2][2] -= matrix.array[2][2];
   return *this;
}

// Overloaded operator for multiplication with a number with assignment
inline Matrix3x3& Matrix3x3::operator*=(decimal nb) {
   array[0][0] *= nb; array[0][1] *= nb; array[0][2] *= nb;
   array[1][0] *= nb; array[1][1] *= nb; array[1][2] *= nb;
   array[2][0] *= nb; array[2][1] *= nb; array[2][2] *= nb;
   return *this;
}

} // End of the ReactPhysics3D namespace

#endif
