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


#ifndef REACTPHYSICS3D_MATRIX3X3_H
#define REACTPHYSICS3D_MATRIX3X3_H

// Libraries
#include <cassert>
#include "Vector3.h"

/// ReactPhysics3D namespace
namespace reactphysics3d {

// Class Matrix3x3
/**
 * This class represents a 3x3 matrix.
 */
class Matrix3x3 {

    private :

        // -------------------- Attributes -------------------- //

        /// Rows of the matrix;
        Vector3 mRows[3];

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        Matrix3x3();

        /// Constructor
        Matrix3x3(decimal value);

        /// Constructor
        Matrix3x3(decimal a1, decimal a2, decimal a3, decimal b1, decimal b2, decimal b3,
                  decimal c1, decimal c2, decimal c3);

        /// Destructor
        ~Matrix3x3() = default;

        /// Copy-constructor
        Matrix3x3(const Matrix3x3& matrix);

        /// Assignment operator
        Matrix3x3& operator=(const Matrix3x3& matrix);

        /// Set all the values in the matrix
        void setAllValues(decimal a1, decimal a2, decimal a3, decimal b1, decimal b2, decimal b3,
                  decimal c1, decimal c2, decimal c3);

        /// Set the matrix to zero
        void setToZero();

        /// Return a column
        Vector3 getColumn(int i) const;

        /// Return a row
        Vector3 getRow(int i) const;

        /// Return the transpose matrix
        Matrix3x3 getTranspose() const;

        /// Return the determinant of the matrix
        decimal getDeterminant() const;

        /// Return the trace of the matrix
        decimal getTrace() const;

        /// Return the inverse matrix
        Matrix3x3 getInverse() const;

        /// Return the matrix with absolute values
        Matrix3x3 getAbsoluteMatrix() const;

        /// Set the matrix to the identity matrix
        void setToIdentity();

        /// Return the 3x3 identity matrix
        static Matrix3x3 identity();

        /// Return the 3x3 zero matrix
        static Matrix3x3 zero();

        /// Return a skew-symmetric matrix using a given vector that can be used
        /// to compute cross product with another vector using matrix multiplication
        static Matrix3x3 computeSkewSymmetricMatrixForCrossProduct(const Vector3& vector);

        /// Overloaded operator for addition
        friend Matrix3x3 operator+(const Matrix3x3& matrix1, const Matrix3x3& matrix2);

        /// Overloaded operator for substraction
        friend Matrix3x3 operator-(const Matrix3x3& matrix1, const Matrix3x3& matrix2);

        /// Overloaded operator for the negative of the matrix
        friend Matrix3x3 operator-(const Matrix3x3& matrix);

        /// Overloaded operator for multiplication with a number
        friend Matrix3x3 operator*(decimal nb, const Matrix3x3& matrix);

        /// Overloaded operator for multiplication with a matrix
        friend Matrix3x3 operator*(const Matrix3x3& matrix, decimal nb);

        /// Overloaded operator for matrix multiplication
        friend Matrix3x3 operator*(const Matrix3x3& matrix1, const Matrix3x3& matrix2);

        /// Overloaded operator for multiplication with a vector
        friend Vector3 operator*(const Matrix3x3& matrix, const Vector3& vector);

        /// Overloaded operator for equality condition
        bool operator==(const Matrix3x3& matrix) const;

        /// Overloaded operator for the is different condition
        bool operator!= (const Matrix3x3& matrix) const;

        /// Overloaded operator for addition with assignment
        Matrix3x3& operator+=(const Matrix3x3& matrix);

        /// Overloaded operator for substraction with assignment
        Matrix3x3& operator-=(const Matrix3x3& matrix);

        /// Overloaded operator for multiplication with a number with assignment
        Matrix3x3& operator*=(decimal nb);

        /// Overloaded operator to read element of the matrix.
        const Vector3& operator[](int row) const;

        /// Overloaded operator to read/write element of the matrix.
        Vector3& operator[](int row);

        /// Return the string representation
        std::string to_string() const;
};

// Constructor of the class Matrix3x3
inline Matrix3x3::Matrix3x3() {
    // Initialize all values in the matrix to zero
    setAllValues(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
}

// Constructor
inline Matrix3x3::Matrix3x3(decimal value) {
    setAllValues(value, value, value, value, value, value, value, value, value);
}

// Constructor with arguments
inline Matrix3x3::Matrix3x3(decimal a1, decimal a2, decimal a3,
                     decimal b1, decimal b2, decimal b3,
                     decimal c1, decimal c2, decimal c3) {
    // Initialize the matrix with the values
    setAllValues(a1, a2, a3, b1, b2, b3, c1, c2, c3);
}

// Copy-constructor
inline Matrix3x3::Matrix3x3(const Matrix3x3& matrix) {
    setAllValues(matrix.mRows[0][0], matrix.mRows[0][1], matrix.mRows[0][2],
                 matrix.mRows[1][0], matrix.mRows[1][1], matrix.mRows[1][2],
                 matrix.mRows[2][0], matrix.mRows[2][1], matrix.mRows[2][2]);
}

// Method to set all the values in the matrix
inline void Matrix3x3::setAllValues(decimal a1, decimal a2, decimal a3,
                                    decimal b1, decimal b2, decimal b3,
                                    decimal c1, decimal c2, decimal c3) {
    mRows[0][0] = a1; mRows[0][1] = a2; mRows[0][2] = a3;
    mRows[1][0] = b1; mRows[1][1] = b2; mRows[1][2] = b3;
    mRows[2][0] = c1; mRows[2][1] = c2; mRows[2][2] = c3;
}

// Set the matrix to zero
inline void Matrix3x3::setToZero() {
    mRows[0].setToZero();
    mRows[1].setToZero();
    mRows[2].setToZero();
}

// Return a column
inline Vector3 Matrix3x3::getColumn(int i) const {
    assert(i>= 0 && i<3);
    return Vector3(mRows[0][i], mRows[1][i], mRows[2][i]);
}

// Return a row
inline Vector3 Matrix3x3::getRow(int i) const {
    assert(i>= 0 && i<3);
    return mRows[i];
}

// Return the transpose matrix
inline Matrix3x3 Matrix3x3::getTranspose() const {

    // Return the transpose matrix
    return Matrix3x3(mRows[0][0], mRows[1][0], mRows[2][0],
                     mRows[0][1], mRows[1][1], mRows[2][1],
                     mRows[0][2], mRows[1][2], mRows[2][2]);
}

// Return the determinant of the matrix
inline decimal Matrix3x3::getDeterminant() const {

    // Compute and return the determinant of the matrix
    return (mRows[0][0]*(mRows[1][1]*mRows[2][2]-mRows[2][1]*mRows[1][2]) -
            mRows[0][1]*(mRows[1][0]*mRows[2][2]-mRows[2][0]*mRows[1][2]) +
            mRows[0][2]*(mRows[1][0]*mRows[2][1]-mRows[2][0]*mRows[1][1]));
}

// Return the trace of the matrix
inline decimal Matrix3x3::getTrace() const {

    // Compute and return the trace
    return (mRows[0][0] + mRows[1][1] + mRows[2][2]);
}

// Set the matrix to the identity matrix
inline void Matrix3x3::setToIdentity() {
    mRows[0][0] = 1.0; mRows[0][1] = 0.0; mRows[0][2] = 0.0;
    mRows[1][0] = 0.0; mRows[1][1] = 1.0; mRows[1][2] = 0.0;
    mRows[2][0] = 0.0; mRows[2][1] = 0.0; mRows[2][2] = 1.0;
}

// Return the 3x3 identity matrix
inline Matrix3x3 Matrix3x3::identity() {
    return Matrix3x3(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
}

// Return the 3x3 zero matrix
inline Matrix3x3 Matrix3x3::zero() {
    return Matrix3x3(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
}

// Return a skew-symmetric matrix using a given vector that can be used
// to compute cross product with another vector using matrix multiplication
inline Matrix3x3 Matrix3x3::computeSkewSymmetricMatrixForCrossProduct(const Vector3& vector) {
    return Matrix3x3(0, -vector.z, vector.y, vector.z, 0, -vector.x, -vector.y, vector.x, 0);
}

// Return the matrix with absolute values
inline Matrix3x3 Matrix3x3::getAbsoluteMatrix() const {
    return Matrix3x3(std::fabs(mRows[0][0]), std::fabs(mRows[0][1]), std::fabs(mRows[0][2]),
                     std::fabs(mRows[1][0]), std::fabs(mRows[1][1]), std::fabs(mRows[1][2]),
                     std::fabs(mRows[2][0]), std::fabs(mRows[2][1]), std::fabs(mRows[2][2]));
}

// Overloaded operator for addition
inline Matrix3x3 operator+(const Matrix3x3& matrix1, const Matrix3x3& matrix2) {
    return Matrix3x3(matrix1.mRows[0][0] + matrix2.mRows[0][0], matrix1.mRows[0][1] +
                     matrix2.mRows[0][1], matrix1.mRows[0][2] + matrix2.mRows[0][2],
                     matrix1.mRows[1][0] + matrix2.mRows[1][0], matrix1.mRows[1][1] +
                     matrix2.mRows[1][1], matrix1.mRows[1][2] + matrix2.mRows[1][2],
                     matrix1.mRows[2][0] + matrix2.mRows[2][0], matrix1.mRows[2][1] +
                     matrix2.mRows[2][1], matrix1.mRows[2][2] + matrix2.mRows[2][2]);
}

// Overloaded operator for substraction
inline Matrix3x3 operator-(const Matrix3x3& matrix1, const Matrix3x3& matrix2) {
    return Matrix3x3(matrix1.mRows[0][0] - matrix2.mRows[0][0], matrix1.mRows[0][1] -
                     matrix2.mRows[0][1], matrix1.mRows[0][2] - matrix2.mRows[0][2],
                     matrix1.mRows[1][0] - matrix2.mRows[1][0], matrix1.mRows[1][1] -
                     matrix2.mRows[1][1], matrix1.mRows[1][2] - matrix2.mRows[1][2],
                     matrix1.mRows[2][0] - matrix2.mRows[2][0], matrix1.mRows[2][1] -
                     matrix2.mRows[2][1], matrix1.mRows[2][2] - matrix2.mRows[2][2]);
}

// Overloaded operator for the negative of the matrix
inline Matrix3x3 operator-(const Matrix3x3& matrix) {
    return Matrix3x3(-matrix.mRows[0][0], -matrix.mRows[0][1], -matrix.mRows[0][2],
                     -matrix.mRows[1][0], -matrix.mRows[1][1], -matrix.mRows[1][2],
                     -matrix.mRows[2][0], -matrix.mRows[2][1], -matrix.mRows[2][2]);
}

// Overloaded operator for multiplication with a number
inline Matrix3x3 operator*(decimal nb, const Matrix3x3& matrix) {
    return Matrix3x3(matrix.mRows[0][0] * nb, matrix.mRows[0][1] * nb, matrix.mRows[0][2] * nb,
                     matrix.mRows[1][0] * nb, matrix.mRows[1][1] * nb, matrix.mRows[1][2] * nb,
                     matrix.mRows[2][0] * nb, matrix.mRows[2][1] * nb, matrix.mRows[2][2] * nb);
}

// Overloaded operator for multiplication with a matrix
inline Matrix3x3 operator*(const Matrix3x3& matrix, decimal nb) {
    return nb * matrix;
}

// Overloaded operator for matrix multiplication
inline Matrix3x3 operator*(const Matrix3x3& matrix1, const Matrix3x3& matrix2) {
    return Matrix3x3(matrix1.mRows[0][0]*matrix2.mRows[0][0] + matrix1.mRows[0][1] *
                     matrix2.mRows[1][0] + matrix1.mRows[0][2]*matrix2.mRows[2][0],
                     matrix1.mRows[0][0]*matrix2.mRows[0][1] + matrix1.mRows[0][1] *
                     matrix2.mRows[1][1] + matrix1.mRows[0][2]*matrix2.mRows[2][1],
                     matrix1.mRows[0][0]*matrix2.mRows[0][2] + matrix1.mRows[0][1] *
                     matrix2.mRows[1][2] + matrix1.mRows[0][2]*matrix2.mRows[2][2],
                     matrix1.mRows[1][0]*matrix2.mRows[0][0] + matrix1.mRows[1][1] *
                     matrix2.mRows[1][0] + matrix1.mRows[1][2]*matrix2.mRows[2][0],
                     matrix1.mRows[1][0]*matrix2.mRows[0][1] + matrix1.mRows[1][1] *
                     matrix2.mRows[1][1] + matrix1.mRows[1][2]*matrix2.mRows[2][1],
                     matrix1.mRows[1][0]*matrix2.mRows[0][2] + matrix1.mRows[1][1] *
                     matrix2.mRows[1][2] + matrix1.mRows[1][2]*matrix2.mRows[2][2],
                     matrix1.mRows[2][0]*matrix2.mRows[0][0] + matrix1.mRows[2][1] *
                     matrix2.mRows[1][0] + matrix1.mRows[2][2]*matrix2.mRows[2][0],
                     matrix1.mRows[2][0]*matrix2.mRows[0][1] + matrix1.mRows[2][1] *
                     matrix2.mRows[1][1] + matrix1.mRows[2][2]*matrix2.mRows[2][1],
                     matrix1.mRows[2][0]*matrix2.mRows[0][2] + matrix1.mRows[2][1] *
                     matrix2.mRows[1][2] + matrix1.mRows[2][2]*matrix2.mRows[2][2]);
}

// Overloaded operator for multiplication with a vector
inline Vector3 operator*(const Matrix3x3& matrix, const Vector3& vector) {
    return Vector3(matrix.mRows[0][0]*vector.x + matrix.mRows[0][1]*vector.y +
                   matrix.mRows[0][2]*vector.z,
                   matrix.mRows[1][0]*vector.x + matrix.mRows[1][1]*vector.y +
                   matrix.mRows[1][2]*vector.z,
                   matrix.mRows[2][0]*vector.x + matrix.mRows[2][1]*vector.y +
                   matrix.mRows[2][2]*vector.z);
}

// Overloaded operator for equality condition
inline bool Matrix3x3::operator==(const Matrix3x3& matrix) const {
    return (mRows[0][0] == matrix.mRows[0][0] && mRows[0][1] == matrix.mRows[0][1] &&
            mRows[0][2] == matrix.mRows[0][2] &&
            mRows[1][0] == matrix.mRows[1][0] && mRows[1][1] == matrix.mRows[1][1] &&
            mRows[1][2] == matrix.mRows[1][2] &&
            mRows[2][0] == matrix.mRows[2][0] && mRows[2][1] == matrix.mRows[2][1] &&
            mRows[2][2] == matrix.mRows[2][2]);
}

// Overloaded operator for the is different condition
inline bool Matrix3x3::operator!= (const Matrix3x3& matrix) const {
    return !(*this == matrix);
}

// Overloaded operator for addition with assignment
inline Matrix3x3& Matrix3x3::operator+=(const Matrix3x3& matrix) {
   mRows[0][0] += matrix.mRows[0][0]; mRows[0][1] += matrix.mRows[0][1];
   mRows[0][2] += matrix.mRows[0][2]; mRows[1][0] += matrix.mRows[1][0];
   mRows[1][1] += matrix.mRows[1][1]; mRows[1][2] += matrix.mRows[1][2];
   mRows[2][0] += matrix.mRows[2][0]; mRows[2][1] += matrix.mRows[2][1];
   mRows[2][2] += matrix.mRows[2][2];
   return *this;
}

// Overloaded operator for substraction with assignment
inline Matrix3x3& Matrix3x3::operator-=(const Matrix3x3& matrix) {
   mRows[0][0] -= matrix.mRows[0][0]; mRows[0][1] -= matrix.mRows[0][1];
   mRows[0][2] -= matrix.mRows[0][2]; mRows[1][0] -= matrix.mRows[1][0];
   mRows[1][1] -= matrix.mRows[1][1]; mRows[1][2] -= matrix.mRows[1][2];
   mRows[2][0] -= matrix.mRows[2][0]; mRows[2][1] -= matrix.mRows[2][1];
   mRows[2][2] -= matrix.mRows[2][2];
   return *this;
}

// Overloaded operator for multiplication with a number with assignment
inline Matrix3x3& Matrix3x3::operator*=(decimal nb) {
   mRows[0][0] *= nb; mRows[0][1] *= nb; mRows[0][2] *= nb;
   mRows[1][0] *= nb; mRows[1][1] *= nb; mRows[1][2] *= nb;
   mRows[2][0] *= nb; mRows[2][1] *= nb; mRows[2][2] *= nb;
   return *this;
}

// Overloaded operator to return a row of the matrix.
/// This operator is also used to access a matrix value using the syntax
/// matrix[row][col].
inline const Vector3& Matrix3x3::operator[](int row) const {
    return mRows[row];
}

// Overloaded operator to return a row of the matrix.
/// This operator is also used to access a matrix value using the syntax
/// matrix[row][col].
inline Vector3& Matrix3x3::operator[](int row) {
    return mRows[row];
}

// Get the string representation
inline std::string Matrix3x3::to_string() const {
    return "Matrix3x3(" + std::to_string(mRows[0][0]) + "," + std::to_string(mRows[0][1]) + "," + std::to_string(mRows[0][2]) + "," +
           std::to_string(mRows[1][0]) + "," + std::to_string(mRows[1][1]) + "," + std::to_string(mRows[1][2]) + "," +
           std::to_string(mRows[2][0]) + "," + std::to_string(mRows[2][1]) + "," + std::to_string(mRows[2][2]) + ")";
}

}

#endif
