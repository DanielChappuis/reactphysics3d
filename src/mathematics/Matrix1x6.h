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


#ifndef MATRIX1X6_H
#define MATRIX1X6_H

// Libraries
#include <cassert>

#include "Matrix6x6.h"
#include "Vector6D.h"

// ReactPhysics3D namespace
namespace reactphysics3d {

/*  -------------------------------------------------------------------
    Class Matrix1x6 :
        This class represents a 1x6 matrix.
    -------------------------------------------------------------------
*/
class Matrix1x6 {
    private :
        double array[6];                                        // Array with the values of the matrix

    public :
        Matrix1x6();                                            // Constructor
        Matrix1x6(double value);                                // Constructor
        Matrix1x6(double v1, double v2, double v3, double v4,
                  double v5, double v6);                        // Constructor
        virtual ~Matrix1x6();                                   // Destructor

        double getValue(int i) const;                           // Get a value in the matrix
        void setValue(int i, double value);                     // Set a value in the matrix
        void initWithValue(double value);                       // Init all the matrix value with the same value
        Vector6D getTranspose() const;                          // Return the transpose matrix

        // --- Overloaded operators --- //
        Matrix1x6 operator+(const Matrix1x6& matrix2) const;    // Overloaded operator for addition
        Matrix1x6 operator-(const Matrix1x6& matrix2) const ;   // Overloaded operator for substraction
        Matrix1x6 operator*(double nb) const;                   // Overloaded operator for multiplication with a number
        Matrix1x6 operator*(const Matrix6x6& matrix) const;     // Overloaded operator for multiplication with a Matrix6x6
        double operator*(const Vector6D& vector6d) const;       // Overloaded operator for multiplication with a Vector6D
};


// Get a value in the matrix
inline double Matrix1x6::getValue(int i) const {
    assert(i>=0 && i<6);
    return array[i];
}

// Set a value in the matrix
inline void Matrix1x6::setValue(int i, double value) {
    assert(i>=0 && i<6);
    array[i] = value;
}

// Init all the matrix value with the same value
inline void Matrix1x6::initWithValue(double value) {
    array[0] = value; array[1] = value; array[2] = value;
    array[3] = value; array[4] = value; array[5] = value;
}

// Return the transpose matrix
inline Vector6D Matrix1x6::getTranspose() const {
    return Vector6D(array[0], array[1], array[2], array[3], array[4], array[5]);
}

// Overloaded operator for multiplication between a number and a Matrix6x6
inline Matrix1x6 operator*(double value, const Matrix1x6& matrix) {
    return matrix * value;
}

// Overloaded operator for multiplication with a Vector6D
inline double Matrix1x6::operator*(const Vector6D& vector) const {
    return (array[0] * vector.getValue(0) + array[1] * vector.getValue(1) + array[2] * vector.getValue(2) +
            array[3] * vector.getValue(3) + array[4] * vector.getValue(4) + array[5] * vector.getValue(5));
}

// Overloaded operator for substraction
inline Matrix1x6 Matrix1x6::operator-(const Matrix1x6& matrix) const {
    return (array[0] - matrix.array[0], array[1] - matrix.array[1], array[2] - matrix.array[2],
            array[3] - matrix.array[3], array[4] - matrix.array[4], array[5] - matrix.array[5]);
}

// Overloaded operator for addition
inline Matrix1x6 Matrix1x6::operator+(const Matrix1x6& matrix) const {
    return (array[0] + matrix.array[0], array[1] + matrix.array[1], array[2] + matrix.array[2],
            array[3] + matrix.array[3], array[4] + matrix.array[4], array[5] + matrix.array[5]);
}

// Overloaded operator for multiplication with a number
inline Matrix1x6 Matrix1x6::operator*(double value) const {
    return (array[0] * value, array[1] * value, array[2] * value,
            array[3] * value, array[4] * value, array[5] * value);
}

// Overloaded operator for multiplication with a 6x6 matrix
inline Matrix1x6 Matrix1x6::operator*(const Matrix6x6& m) const {
    return Matrix1x6(array[0] * m.getValue(0,0) + array[1] * m.getValue(1,0) + array[2] * m.getValue(2,0) +
                     array[3] * m.getValue(3,0) + array[4] * m.getValue(4,0) + array[5] * m.getValue(5,0),
                     array[0] * m.getValue(0,1) + array[1] * m.getValue(1,1) + array[2] * m.getValue(2,1) +
                     array[3] * m.getValue(3,1) + array[4] * m.getValue(4,1) + array[5] * m.getValue(5,1),
                     array[0] * m.getValue(0,2) + array[1] * m.getValue(1,2) + array[2] * m.getValue(2,2) +
                     array[3] * m.getValue(3,2) + array[4] * m.getValue(4,2) + array[5] * m.getValue(5,2),
                     array[0] * m.getValue(0,3) + array[1] * m.getValue(1,3) + array[2] * m.getValue(2,3) +
                     array[3] * m.getValue(3,3) + array[4] * m.getValue(4,3) + array[5] * m.getValue(5,3),
                     array[0] * m.getValue(0,4) + array[1] * m.getValue(1,4) + array[2] * m.getValue(2,4) +
                     array[3] * m.getValue(3,4) + array[4] * m.getValue(4,4) + array[5] * m.getValue(5,4),
                     array[0] * m.getValue(0,5) + array[1] * m.getValue(1,5) + array[2] * m.getValue(2,5) +
                     array[3] * m.getValue(3,5) + array[4] * m.getValue(4,5) + array[5] * m.getValue(5,5));
}

} // End of the ReactPhysics3D namespace

#endif
