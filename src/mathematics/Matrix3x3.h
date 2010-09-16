/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2010 Daniel Chappuis                                            *
*********************************************************************************
*                                                                               *
* Permission is hereby granted, free of charge, to any person obtaining a copy  *
* of this software and associated documentation files (the "Software"), to deal *
* in the Software without restriction, including without limitation the rights  *
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell     *
* copies of the Software, and to permit persons to whom the Software is         *
* furnished to do so, subject to the following conditions:                      *
*                                                                               *
* The above copyright notice and this permission notice shall be included in    *
* all copies or substantial portions of the Software.                           *
*                                                                               *
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR    *
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,      *
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE   *
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER        *
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, *
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN     *
* THE SOFTWARE.                                                                 *
********************************************************************************/


#ifndef MATRIX3X3_H
#define MATRIX3X3_H

// Libraries
#include <cassert>
#include "Vector3D.h"

// ReactPhysics3D namespace
namespace reactphysics3d {


/*  -------------------------------------------------------------------
    Class Matrix3x3 :
        This class represents a 3x3 matrix.
    -------------------------------------------------------------------
*/
class Matrix3x3 {
    private :
        double array[3][3];                                                                     // Array with the values of the matrix

    public :
        Matrix3x3();                                                                            // Constructor
        Matrix3x3(double value);                                                                // Constructor
        Matrix3x3(double a1, double a2, double a3, double b1, double b2, double b3,
                  double c1, double c2, double c3);                                             // Constructor
        virtual ~Matrix3x3();                                                                   // Destructor

        double getValue(int i, int j) const;                                                    // Get a value in the matrix
        void setValue(int i, int j, double value);                                              // Set a value in the matrix
        void setAllValues(double a1, double a2, double a3, double b1, double b2, double b3,
                  double c1, double c2, double c3);                                             // Set all the values in the matrix
        Matrix3x3 getTranspose() const;                                                         // Return the transpose matrix
        double getDeterminant() const;                                                          // Return the determinant of the matrix
        double getTrace() const;                                                                // Return the trace of the matrix
        Matrix3x3 getInverse() const;                                                           // Return the inverse matrix
        static Matrix3x3 identity();                                                            // Return the 3x3 identity matrix

        // --- Overloaded operators --- //
        Matrix3x3 operator+(const Matrix3x3& matrix2) const;        // Overloaded operator for addition
        Matrix3x3 operator-(const Matrix3x3& matrix2) const ;       // Overloaded operator for substraction
        Matrix3x3 operator*(double nb) const;                       // Overloaded operator for multiplication with a number
        Matrix3x3 operator*(const Matrix3x3& matrix2) const;        // Overloaded operator for multiplication with a matrix
        Vector3D operator*(const Vector3D& vector3d) const;         // Overloaded operator for multiplication with a vector
        Matrix3x3& operator=(const Matrix3x3& matrix2);             // Overloaded operator for assignment
        bool operator==(const Matrix3x3& matrix2) const;            // Overloaded operator for equality condition
};


// Method to get a value in the matrix (inline)
inline double Matrix3x3::getValue(int i, int j) const {
    assert(i>=0 && i<3 && j>=0 && j<3);
    return array[i][j];
}

// Method to set a value in the matrix (inline)
inline void Matrix3x3::setValue(int i, int j, double value) {
    assert(i>=0 && i<3 && j>=0 && j<3);
    array[i][j] = value;
}

// Method to set all the values in the matrix
inline void Matrix3x3::setAllValues(double a1, double a2, double a3, double b1, double b2, double b3,
                  double c1, double c2, double c3) {
    array[0][0] = a1; array[0][1] = a2; array[0][2] = a3;
    array[1][0] = b1; array[1][1] = b2; array[1][2] = b3;
    array[2][0] = c1; array[2][1] = c2; array[2][2] = c3;
}

// Return the transpose matrix
inline Matrix3x3 Matrix3x3::getTranspose() const {
    // Return the transpose matrix
    return Matrix3x3(array[0][0], array[1][0], array[2][0],
                     array[0][1], array[1][1], array[2][1],
                     array[0][2], array[1][2], array[2][2]);
}

// Return the determinant of the matrix
inline double Matrix3x3::getDeterminant() const {
    // Compute and return the determinant of the matrix
    return (array[0][0]*(array[1][1]*array[2][2]-array[2][1]*array[1][2]) - array[0][1]*(array[1][0]*array[2][2]-array[2][0]*array[1][2]) +
            array[0][2]*(array[1][0]*array[2][1]-array[2][0]*array[1][1]));
}

// Return the trace of the matrix
inline double Matrix3x3::getTrace() const {
    // Compute and return the trace
    return (array[0][0] + array[1][1] + array[2][2]);
}

// Overloaded operator for multiplication between a number and a Matrix3x3 (inline)
inline Matrix3x3 operator*(double number, const Matrix3x3& matrix) {
    // Return the multiplied matrix
    return matrix * number;
}

// Overloaded operator for multiplication with a vector
inline Vector3D Matrix3x3::operator*(const Vector3D& vector3d) const {
    // Compute and return the result
    return Vector3D(array[0][0]*vector3d.getX() + array[0][1]*vector3d.getY() + array[0][2]*vector3d.getZ(),
                    array[1][0]*vector3d.getX() + array[1][1]*vector3d.getY() + array[1][2]*vector3d.getZ(),
                    array[2][0]*vector3d.getX() + array[2][1]*vector3d.getY() + array[2][2]*vector3d.getZ());
}

// Overloaded operator for equality condition
inline bool Matrix3x3::operator==(const Matrix3x3& matrix2) const {
    return (array[0][0] == matrix2.array[0][0] && array[0][1] == matrix2.array[0][1] && array[0][2] == matrix2.array[0][2] &&
            array[1][0] == matrix2.array[1][0] && array[1][1] == matrix2.array[1][1] && array[1][2] == matrix2.array[1][2] &&
            array[2][0] == matrix2.array[2][0] && array[2][1] == matrix2.array[2][1] && array[2][2] == matrix2.array[2][2]);
}

// Return the 3x3 identity matrix
inline Matrix3x3 Matrix3x3::identity() {
    // Return the isdentity matrix
    return Matrix3x3(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
}

// Overloaded operator for addition
inline Matrix3x3 Matrix3x3::operator+(const Matrix3x3& matrix2) const {
    // Return the sum matrix
    return Matrix3x3(array[0][0] + matrix2.array[0][0], array[0][1] + matrix2.array[0][1], array[0][2] + matrix2.array[0][2],
                     array[1][0] + matrix2.array[1][0], array[1][1] + matrix2.array[1][1], array[1][2] + matrix2.array[1][2],
                     array[2][0] + matrix2.array[2][0], array[2][1] + matrix2.array[2][1], array[2][2] + matrix2.array[2][2]);
}

// Overloaded operator for substraction
inline Matrix3x3 Matrix3x3::operator-(const Matrix3x3& matrix2) const {
    // Return the substraction matrix
    return Matrix3x3(array[0][0] - matrix2.array[0][0], array[0][1] - matrix2.array[0][1], array[0][2] - matrix2.array[0][2],
                     array[1][0] - matrix2.array[1][0], array[1][1] - matrix2.array[1][1], array[1][2] - matrix2.array[1][2],
                     array[2][0] - matrix2.array[2][0], array[2][1] - matrix2.array[2][1], array[2][2] - matrix2.array[2][2]);
}

// Overloaded operator for multiplication with a number
inline Matrix3x3 Matrix3x3::operator*(double nb) const {
    // Return multiplied matrix
    return Matrix3x3(array[0][0] * nb, array[0][1] * nb, array[0][2] * nb,
                     array[1][0] * nb, array[1][1] * nb, array[1][2] * nb,
                     array[2][0] * nb, array[2][1] * nb, array[2][2] * nb);
}

} // End of the ReactPhysics3D namespace

#endif
