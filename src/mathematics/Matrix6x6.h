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


#ifndef MATRIX6X6_H
#define MATRIX6X6_H

// Libraries
#include <cassert>
#include "Vector6D.h"

// ReactPhysics3D namespace
namespace reactphysics3d {

/*  -------------------------------------------------------------------
    Class Matrix6x6 :
        This class represents a 6x6 matrix.
    -------------------------------------------------------------------
*/
class Matrix6x6 {
    private :
        double array[6][6];                                     // Array with the values of the matrix

    public :
        Matrix6x6();                                            // Constructor
        Matrix6x6(double value);                                // Constructor
        virtual ~Matrix6x6();                                   // Destructor

        double getValue(int i, int j) const;                    // Get a value in the matrix
        void setValue(int i, int j, double value);              // Set a value in the matrix
        void initWithValue(double value);                       // Init all the values with a unique value
        Matrix6x6 getTranspose() const;                         // Return the transpose matrix
        static Matrix6x6 identity();                            // Return the identity matrix

        // --- Overloaded operators --- //
        Matrix6x6 operator+(const Matrix6x6& matrix2) const;    // Overloaded operator for addition
        Matrix6x6 operator-(const Matrix6x6& matrix2) const;    // Overloaded operator for substraction
        Matrix6x6 operator*(double nb) const;                   // Overloaded operator for multiplication with a number
        Vector6D operator*(const Vector6D& vector6d) const;     // Overloaded operator for multiplication with a Vector6D
};


// Get a value in the matrix
inline double Matrix6x6::getValue(int i, int j) const {
    assert(i>=0 && i<6 && j>=0 && j<6);
    return array[i][j];
}

// Set a value in the matrix
inline void Matrix6x6::setValue(int i, int j, double value) {
    array[i][j] = value;
}

// Init all the values with a unique value
inline void Matrix6x6::initWithValue(double value) {
    array[0][0] = value; array[0][1] = value; array[0][2] = value;
    array[0][3] = value; array[0][4] = value; array[0][5] = value;
    array[1][0] = value; array[1][1] = value; array[1][2] = value;
    array[1][3] = value; array[1][4] = value; array[1][5] = value;
    array[2][0] = value; array[2][1] = value; array[2][2] = value;
    array[2][3] = value; array[2][4] = value; array[2][5] = value;
    array[3][0] = value; array[3][1] = value; array[3][2] = value;
    array[3][3] = value; array[3][4] = value; array[3][5] = value;
    array[4][0] = value; array[4][1] = value; array[4][2] = value;
    array[4][3] = value; array[4][4] = value; array[4][5] = value;
    array[5][0] = value; array[5][1] = value; array[5][2] = value;
    array[5][3] = value; array[5][4] = value; array[5][5] = value;
}

// Return the transpose matrix
inline Matrix6x6 Matrix6x6::getTranspose() const {
    Matrix6x6 transpose;

    transpose.array[0][0] = array[0][0]; transpose.array[0][1] = array[1][0]; transpose.array[0][2] = array[2][0];
    transpose.array[0][3] = array[3][0]; transpose.array[0][4] = array[4][0]; transpose.array[0][5] = array[5][0];
    transpose.array[1][0] = array[0][1]; transpose.array[1][1] = array[1][1]; transpose.array[1][2] = array[2][1];
    transpose.array[1][3] = array[3][1]; transpose.array[1][4] = array[4][1]; transpose.array[1][5] = array[5][1];
    transpose.array[2][0] = array[0][2]; transpose.array[2][1] = array[1][2]; transpose.array[2][2] = array[2][2];
    transpose.array[2][3] = array[3][2]; transpose.array[2][4] = array[4][2]; transpose.array[2][5] = array[5][2];
    transpose.array[3][0] = array[0][3]; transpose.array[3][1] = array[1][3]; transpose.array[3][2] = array[2][3];
    transpose.array[3][3] = array[3][3]; transpose.array[3][4] = array[4][3]; transpose.array[3][5] = array[5][3];
    transpose.array[4][0] = array[0][4]; transpose.array[4][1] = array[1][4]; transpose.array[4][2] = array[2][4];
    transpose.array[4][3] = array[3][4]; transpose.array[4][4] = array[4][4]; transpose.array[4][5] = array[5][4];
    transpose.array[5][0] = array[0][5]; transpose.array[5][1] = array[1][5]; transpose.array[5][2] = array[2][5];
    transpose.array[5][3] = array[3][5]; transpose.array[5][4] = array[4][5]; transpose.array[5][5] = array[5][5];
    return transpose;
}

// Return the identity matrix
inline Matrix6x6 Matrix6x6::identity() {
    Matrix6x6 identity(0.0);
    identity.array[0][0] = 1.0; identity.array[1][1] = 1.0; identity.array[2][2] = 1.0;
    identity.array[3][3] = 1.0; identity.array[4][4] = 1.0; identity.array[5][5] = 1.0;
    return identity;
}

// Overloaded operator for multiplication between a number and a Matrix6x6
inline Matrix6x6 operator*(double value, const Matrix6x6& matrix) {
    return matrix * value;
}

// Overloaded operator for multiplication with a Vector6D
inline Vector6D Matrix6x6::operator*(const Vector6D& vector6d) const {
    Vector6D result;
    result.setValue(0, array[0][0] * vector6d.getValue(0) + array[0][1] * vector6d.getValue(1) + array[0][2] * vector6d.getValue(2) +
                       array[0][3] * vector6d.getValue(3) + array[0][4] * vector6d.getValue(4) + array[0][5] * vector6d.getValue(5));
    result.setValue(1, array[1][0] * vector6d.getValue(0) + array[1][1] * vector6d.getValue(1) + array[1][2] * vector6d.getValue(2) +
                       array[1][3] * vector6d.getValue(3) + array[1][4] * vector6d.getValue(4) + array[1][5] * vector6d.getValue(5));
    result.setValue(2, array[2][0] * vector6d.getValue(0) + array[2][1] * vector6d.getValue(1) + array[2][2] * vector6d.getValue(2) +
                       array[2][3] * vector6d.getValue(3) + array[2][4] * vector6d.getValue(4) + array[2][5] * vector6d.getValue(5));
    result.setValue(3, array[3][0] * vector6d.getValue(0) + array[3][1] * vector6d.getValue(1) + array[3][2] * vector6d.getValue(2) +
                       array[3][3] * vector6d.getValue(3) + array[3][4] * vector6d.getValue(4) + array[3][5] * vector6d.getValue(5));
    result.setValue(4, array[4][0] * vector6d.getValue(0) + array[4][1] * vector6d.getValue(1) + array[4][2] * vector6d.getValue(2) +
                       array[4][3] * vector6d.getValue(3) + array[4][4] * vector6d.getValue(4) + array[4][5] * vector6d.getValue(5));
    result.setValue(5, array[5][0] * vector6d.getValue(0) + array[5][1] * vector6d.getValue(1) + array[5][2] * vector6d.getValue(2) +
                       array[5][3] * vector6d.getValue(3) + array[5][4] * vector6d.getValue(4) + array[5][5] * vector6d.getValue(5));
    return result;
}

// Overloaded operator for substraction
inline Matrix6x6 Matrix6x6::operator-(const Matrix6x6& matrix) const {
    Matrix6x6 result;
    result.array[0][0] = array[0][0] - matrix.array[0][0]; result.array[0][1] = array[0][1] - matrix.array[0][1]; result.array[0][2] = array[0][2] - matrix.array[0][2];
    result.array[0][3] = array[0][3] - matrix.array[0][3]; result.array[0][4] = array[0][4] - matrix.array[0][4]; result.array[0][5] = array[0][5] - matrix.array[0][5];
    result.array[1][0] = array[1][0] - matrix.array[1][0]; result.array[1][1] = array[1][1] - matrix.array[1][1]; result.array[1][2] = array[1][2] - matrix.array[1][2];
    result.array[1][3] = array[1][3] - matrix.array[1][3]; result.array[1][4] = array[1][4] - matrix.array[1][4]; result.array[1][5] = array[1][5] - matrix.array[1][5];
    result.array[2][0] = array[2][0] - matrix.array[2][0]; result.array[2][1] = array[2][1] - matrix.array[2][1]; result.array[2][2] = array[2][2] - matrix.array[2][2];
    result.array[2][3] = array[2][3] - matrix.array[2][3]; result.array[2][4] = array[2][4] - matrix.array[2][4]; result.array[2][5] = array[2][5] - matrix.array[2][5];
    result.array[3][0] = array[3][0] - matrix.array[3][0]; result.array[3][1] = array[3][1] - matrix.array[3][1]; result.array[3][2] = array[3][2] - matrix.array[3][2];
    result.array[3][3] = array[3][3] - matrix.array[3][3]; result.array[3][4] = array[3][4] - matrix.array[3][4]; result.array[3][5] = array[3][5] - matrix.array[3][5];
    result.array[4][0] = array[4][0] - matrix.array[4][0]; result.array[4][1] = array[4][1] - matrix.array[4][1]; result.array[4][2] = array[4][2] - matrix.array[4][2];
    result.array[4][3] = array[4][3] - matrix.array[4][3]; result.array[4][4] = array[4][4] - matrix.array[4][4]; result.array[4][5] = array[4][5] - matrix.array[4][5];
    result.array[5][0] = array[5][0] - matrix.array[5][0]; result.array[5][1] = array[5][1] - matrix.array[5][1]; result.array[5][2] = array[5][2] - matrix.array[5][2];
    result.array[5][3] = array[5][3] - matrix.array[5][3]; result.array[5][4] = array[5][4] - matrix.array[5][4]; result.array[5][5] = array[5][5] - matrix.array[5][5];
    return result;
}

// Overloaded operator for addition
inline Matrix6x6 Matrix6x6::operator+(const Matrix6x6& matrix) const {
    Matrix6x6 result;
    result.array[0][0] = array[0][0] + matrix.array[0][0]; result.array[0][1] = array[0][1] + matrix.array[0][1]; result.array[0][2] = array[0][2] + matrix.array[0][2];
    result.array[0][3] = array[0][3] + matrix.array[0][3]; result.array[0][4] = array[0][4] + matrix.array[0][4]; result.array[0][5] = array[0][5] + matrix.array[0][5];
    result.array[1][0] = array[1][0] + matrix.array[1][0]; result.array[1][1] = array[1][1] + matrix.array[1][1]; result.array[1][2] = array[1][2] + matrix.array[1][2];
    result.array[1][3] = array[1][3] + matrix.array[1][3]; result.array[1][4] = array[1][4] + matrix.array[1][4]; result.array[1][5] = array[1][5] + matrix.array[1][5];
    result.array[2][0] = array[2][0] + matrix.array[2][0]; result.array[2][1] = array[2][1] + matrix.array[2][1]; result.array[2][2] = array[2][2] + matrix.array[2][2];
    result.array[2][3] = array[2][3] + matrix.array[2][3]; result.array[2][4] = array[2][4] + matrix.array[2][4]; result.array[2][5] = array[2][5] + matrix.array[2][5];
    result.array[3][0] = array[3][0] + matrix.array[3][0]; result.array[3][1] = array[3][1] + matrix.array[3][1]; result.array[3][2] = array[3][2] + matrix.array[3][2];
    result.array[3][3] = array[3][3] + matrix.array[3][3]; result.array[3][4] = array[3][4] + matrix.array[3][4]; result.array[3][5] = array[3][5] + matrix.array[3][5];
    result.array[4][0] = array[4][0] + matrix.array[4][0]; result.array[4][1] = array[4][1] + matrix.array[4][1]; result.array[4][2] = array[4][2] + matrix.array[4][2];
    result.array[4][3] = array[4][3] + matrix.array[4][3]; result.array[4][4] = array[4][4] + matrix.array[4][4]; result.array[4][5] = array[4][5] + matrix.array[4][5];
    result.array[5][0] = array[5][0] + matrix.array[5][0]; result.array[5][1] = array[5][1] + matrix.array[5][1]; result.array[5][2] = array[5][2] + matrix.array[5][2];
    result.array[5][3] = array[5][3] + matrix.array[5][3]; result.array[5][4] = array[5][4] + matrix.array[5][4]; result.array[5][5] = array[5][5] + matrix.array[5][5];
    return result;
}

// Overloaded operator for multiplication with a number
inline Matrix6x6 Matrix6x6::operator*(double value) const {
    Matrix6x6 result;
    result.array[0][0] = value * array[0][0]; result.array[0][1] = value * array[0][1]; result.array[0][2] = value * array[0][2];
    result.array[0][3] = value * array[0][3]; result.array[0][4] = value * array[0][4]; result.array[0][5] = value * array[0][5];
    result.array[1][0] = value * array[1][0]; result.array[1][1] = value * array[1][1]; result.array[1][2] = value * array[1][2];
    result.array[1][3] = value * array[1][3]; result.array[1][4] = value * array[1][4]; result.array[1][5] = value * array[1][5];
    result.array[2][0] = value * array[2][0]; result.array[2][1] = value * array[2][1]; result.array[2][2] = value * array[2][2];
    result.array[2][3] = value * array[2][3]; result.array[2][4] = value * array[2][4]; result.array[2][5] = value * array[2][5];
    result.array[3][0] = value * array[3][0]; result.array[3][1] = value * array[3][1]; result.array[3][2] = value * array[3][2];
    result.array[3][3] = value * array[3][3]; result.array[3][4] = value * array[3][4]; result.array[3][5] = value * array[3][5];
    result.array[4][0] = value * array[4][0]; result.array[4][1] = value * array[4][1]; result.array[4][2] = value * array[4][2];
    result.array[4][3] = value * array[4][3]; result.array[4][4] = value * array[4][4]; result.array[4][5] = value * array[4][5];
    result.array[5][0] = value * array[5][0]; result.array[5][1] = value * array[5][1]; result.array[5][2] = value * array[5][2];
    result.array[5][3] = value * array[5][3]; result.array[5][4] = value * array[5][4]; result.array[5][5] = value * array[5][5];
    return result;
}

} // End of the ReactPhysics3D namespace

#endif
