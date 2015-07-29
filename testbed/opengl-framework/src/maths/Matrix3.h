/********************************************************************************
* OpenGL-Framework                                                              *
* Copyright (c) 2013 Daniel Chappuis                                            *
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

#ifndef MATRIX3_H
#define MATRIX3_H

// Libraries
#include <cassert>
#include <limits>
#include "Vector3.h"

namespace openglframework {

// Class Matrix4
// This class represents a 4x4 matrix
class Matrix3 {

    private :

        // -------------------- Attributes -------------------- //

        // Elements of the matrix
        float m[3][3];

    public :

        // Constructor
        Matrix3() {
            setToNull();
        }

        // Constructor
        Matrix3(float a1, float a2,
                float a3, float b1, float b2, float b3,
                float c1, float c2, float c3) {
            setAllValues(a1, a2, a3, b1, b2, b3, c1, c2, c3);
        }

        // Constructor
        Matrix3(float n[3][3]) {
            m[0][0]=n[0][0]; m[0][1]=n[0][1]; m[0][2]=n[0][2];
            m[1][0]=n[1][0]; m[1][1]=n[1][1]; m[1][2]=n[1][2];
            m[2][0]=n[2][0]; m[2][1]=n[2][1]; m[2][2]=n[2][2];
        }

        // Constructor
        Matrix3(const Vector3& a1, const Vector3& a2, const Vector3& a3) {
            m[0][0] = a1.x; m[0][1] = a2.x; m[0][2] = a3.x;
            m[1][0] = a1.y; m[1][1] = a2.y; m[1][2] = a3.y;
            m[2][0] = a1.z; m[2][1] = a2.z; m[2][2] = a3.z;
        }

        // Constructor
        Matrix3(const Matrix3& matrix) {
            setAllValues(matrix.m[0][0], matrix.m[0][1], matrix.m[0][2],
                         matrix.m[1][0], matrix.m[1][1], matrix.m[1][2],
                         matrix.m[2][0], matrix.m[2][1], matrix.m[2][2]);
        }

        // Method to get a value in the matrix
        float getValue(int i, int j) const {
            assert(i>=0 && i<3 && j>=0 && j<3);
            return m[i][j];
        }

        // Method to set a value in the matrix
        void setValue(int i, int j, float value) {
            assert(i>=0 && i<3 && j>=0 && j<3);
            m[i][j] = value;
        }

        // Method to set all the values in the matrix
        void setAllValues(float a1, float a2, float a3, float b1, float b2, float b3,
                                          float c1, float c2, float c3) {
            m[0][0] = a1; m[0][1] = a2; m[0][2] = a3;
            m[1][0] = b1; m[1][1] = b2; m[1][2] = b3;
            m[2][0] = c1; m[2][1] = c2; m[2][2] = c3;
        }

        // Return a column
        Vector3 getColumn(int i) const {
            assert(i>= 0 && i<3);
            return Vector3(m[0][i], m[1][i], m[2][i]);
        }

        // Return the transpose matrix
        Matrix3 getTranspose() const {
            // Return the transpose matrix
            return Matrix3(m[0][0], m[1][0], m[2][0],
                           m[0][1], m[1][1], m[2][1],
                           m[0][2], m[1][2], m[2][2]);
        }

        // Return the determinant of the matrix
        float getDeterminant() const {
            // Compute and return the determinant of the matrix
            return (m[0][0]*(m[1][1]*m[2][2]-m[2][1]*m[1][2]) - m[0][1]*(m[1][0]*m[2][2]-m[2][0]*m[1][2]) +
                    m[0][2]*(m[1][0]*m[2][1]-m[2][0]*m[1][1]));
        }

        // Return the trace of the matrix
        float getTrace() const {
            // Compute and return the trace
            return (m[0][0] + m[1][1] + m[2][2]);
        }

        void setToNull() {
          m[0][0] = 0.0; m[0][1] = 0.0; m[0][2] = 0.0;
          m[1][0] = 0.0; m[1][1] = 0.0; m[1][2] = 0.0;
          m[2][0] = 0.0; m[2][1] = 0.0; m[2][2] = 0.0;
        }

        bool isNull() const {
          Matrix3 zero;
          return *this == zero;
        }

        // Set the matrix to the identity matrix
        void setToIdentity() {
            m[0][0] = 1.0; m[0][1] = 0.0; m[0][2] = 0.0;
            m[1][0] = 0.0; m[1][1] = 1.0; m[1][2] = 0.0;
            m[2][0] = 0.0; m[2][1] = 0.0; m[2][2] = 1.0;
        }

        bool isIdentity() const {
          Matrix3 I;
          I.setToIdentity();
          return ( *this == I );
        }

        // Return the inverse matrix
        Matrix3 getInverse() const {

            // Compute the determinant of the matrix
            float determinant = getDeterminant();

            // Check if the determinant is equal to zero
            assert(determinant > std::numeric_limits<float>::epsilon());

            float invDeterminant = 1.0f / determinant;
            Matrix3 tempMatrix((m[1][1]*m[2][2]-m[2][1]*m[1][2]), -(m[0][1]*m[2][2]-m[2][1]*m[0][2]), (m[0][1]*m[1][2]-m[0][2]*m[1][1]),
                               -(m[1][0]*m[2][2]-m[2][0]*m[1][2]), (m[0][0]*m[2][2]-m[2][0]*m[0][2]), -(m[0][0]*m[1][2]-m[1][0]*m[0][2]),
                               (m[1][0]*m[2][1]-m[2][0]*m[1][1]), -(m[0][0]*m[2][1]-m[2][0]*m[0][1]), (m[0][0]*m[1][1]-m[0][1]*m[1][0]));

            // Return the inverse matrix
            return (tempMatrix * invDeterminant);
        }

        // Display the matrix
        void print() const {
            for (int i=0; i<3; i++) {
                for (int j=0; j<3; j++) {
                    std::cout << m[i][j] << " ";
                }
                std::cout << std::endl;
            }
        }

        // Overloaded operator =
        Matrix3& operator=(const Matrix3& matrix) {
            if (&matrix != this) {
                setAllValues(matrix.m[0][0], matrix.m[0][1], matrix.m[0][2],
                             matrix.m[1][0], matrix.m[1][1], matrix.m[1][2],
                             matrix.m[2][0], matrix.m[2][1], matrix.m[2][2]);
            }
            return *this;
        }


        // Overloaded operator for addition
        Matrix3 operator+(const Matrix3& matrix2) {
            return Matrix3(m[0][0] + matrix2.m[0][0], m[0][1] + matrix2.m[0][1], m[0][2] + matrix2.m[0][2],
                           m[1][0] + matrix2.m[1][0], m[1][1] + matrix2.m[1][1], m[1][2] + matrix2.m[1][2],
                           m[2][0] + matrix2.m[2][0], m[2][1] + matrix2.m[2][1], m[2][2] + matrix2.m[2][2]);
        }

        // Overloaded operator for substraction
        Matrix3 operator-(const Matrix3& matrix2) {
            return Matrix3(m[0][0] - matrix2.m[0][0], m[0][1] - matrix2.m[0][1], m[0][2] - matrix2.m[0][2],
                           m[1][0] - matrix2.m[1][0], m[1][1] - matrix2.m[1][1], m[1][2] - matrix2.m[1][2],
                           m[2][0] - matrix2.m[2][0], m[2][1] - matrix2.m[2][1], m[2][2] - matrix2.m[2][2]);
        }

        // Overloaded operator for the negative of the matrix
        Matrix3 operator-() {
            return Matrix3(-m[0][0], -m[0][1], -m[0][2],
                           -m[1][0], -m[1][1], -m[1][2],
                           -m[2][0], -m[2][1], -m[2][2]);
        }

        // Overloaded operator for multiplication with a number
        Matrix3 operator*(float nb) {
            return Matrix3(m[0][0] * nb, m[0][1] * nb, m[0][2] * nb,
                           m[1][0] * nb, m[1][1] * nb, m[1][2] * nb,
                           m[2][0] * nb, m[2][1] * nb, m[2][2] * nb);
        }

        // Overloaded operator for matrix multiplication
        Matrix3 operator*(const Matrix3& matrix2) {
            return Matrix3(m[0][0]*matrix2.m[0][0] + m[0][1]*matrix2.m[1][0] + m[0][2]*matrix2.m[2][0],
                           m[0][0]*matrix2.m[0][1] + m[0][1]*matrix2.m[1][1] + m[0][2]*matrix2.m[2][1],
                           m[0][0]*matrix2.m[0][2] + m[0][1]*matrix2.m[1][2] + m[0][2]*matrix2.m[2][2],
                           m[1][0]*matrix2.m[0][0] + m[1][1]*matrix2.m[1][0] + m[1][2]*matrix2.m[2][0],
                           m[1][0]*matrix2.m[0][1] + m[1][1]*matrix2.m[1][1] + m[1][2]*matrix2.m[2][1],
                           m[1][0]*matrix2.m[0][2] + m[1][1]*matrix2.m[1][2] + m[1][2]*matrix2.m[2][2],
                           m[2][0]*matrix2.m[0][0] + m[2][1]*matrix2.m[1][0] + m[2][2]*matrix2.m[2][0],
                           m[2][0]*matrix2.m[0][1] + m[2][1]*matrix2.m[1][1] + m[2][2]*matrix2.m[2][1],
                           m[2][0]*matrix2.m[0][2] + m[2][1]*matrix2.m[1][2] + m[2][2]*matrix2.m[2][2]);
        }

        // Overloaded operator for multiplication with a vector
        Vector3 operator*(const Vector3& vector) {
            return Vector3(m[0][0]*vector.x + m[0][1]*vector.y + m[0][2]*vector.z,
                           m[1][0]*vector.x + m[1][1]*vector.y + m[1][2]*vector.z,
                           m[2][0]*vector.x + m[2][1]*vector.y + m[2][2]*vector.z);
        }

        // Overloaded operator for equality condition
        bool operator==(const Matrix3& matrix) const {
            return (m[0][0] == matrix.m[0][0] && m[0][1] == matrix.m[0][1] && m[0][2] == matrix.m[0][2] &&
                    m[1][0] == matrix.m[1][0] && m[1][1] == matrix.m[1][1] && m[1][2] == matrix.m[1][2] &&
                    m[2][0] == matrix.m[2][0] && m[2][1] == matrix.m[2][1] && m[2][2] == matrix.m[2][2]);
        }

        // Overloaded operator for the is different condition
        bool operator!= (const Matrix3& matrix) const {
            return !(*this == matrix);
        }

        // Overloaded operator for addition with assignment
        Matrix3& operator+=(const Matrix3& matrix) {
            m[0][0] += matrix.m[0][0]; m[0][1] += matrix.m[0][1]; m[0][2] += matrix.m[0][2];
            m[1][0] += matrix.m[1][0]; m[1][1] += matrix.m[1][1]; m[1][2] += matrix.m[1][2];
            m[2][0] += matrix.m[2][0]; m[2][1] += matrix.m[2][1]; m[2][2] += matrix.m[2][2];
            return *this;
        }

        // Overloaded operator for substraction with assignment
        Matrix3& operator-=(const Matrix3& matrix) {
            m[0][0] -= matrix.m[0][0]; m[0][1] -= matrix.m[0][1]; m[0][2] -= matrix.m[0][2];
            m[1][0] -= matrix.m[1][0]; m[1][1] -= matrix.m[1][1]; m[1][2] -= matrix.m[1][2];
            m[2][0] -= matrix.m[2][0]; m[2][1] -= matrix.m[2][1]; m[2][2] -= matrix.m[2][2];
            return *this;
        }

        // Overloaded operator for multiplication with a number with assignment
        Matrix3& operator*=(float nb) {
            m[0][0] *= nb; m[0][1] *= nb; m[0][2] *= nb;
            m[1][0] *= nb; m[1][1] *= nb; m[1][2] *= nb;
            m[2][0] *= nb; m[2][1] *= nb; m[2][2] *= nb;
            return *this;
        }

        // Return the identity matrix
        static Matrix3 identity() {
            return Matrix3(1, 0, 0,
                           0, 1, 0,
                           0, 0, 1);
        }
};

}

#endif
