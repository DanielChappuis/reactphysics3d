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

#ifndef VECTOR6D_H
#define VECTOR6D_H

// Libraries
#include <cmath>
#include <cassert>
#include "mathematics_functions.h"


// ReactPhysics3D namespace
namespace reactphysics3d {

/*  -------------------------------------------------------------------
    Class Vector6D :
        This class represents 6 dimensionnal vector in space.
    -------------------------------------------------------------------
*/
class Vector6D {
    private :
        double array[6];

    public :
        Vector6D();                                             // Constructor
        Vector6D(double value);                                 // Constructor
        Vector6D(double v1, double v2, double v3,
                 double v4, double v5, double v6);              // Constructor
        virtual ~Vector6D();                                    // Destructor
        double getValue(int index) const;                       // Get a component of the vector
        void setValue(int index, double value);                 // Set a component of the vector
        void setAllValues(double v1, double v2, double v3,
                          double v4, double v5, double v6);     // Set all the values of the vector
        void initWithValue(double value);                       // Init all the values with the same value
        
        // --- Overloaded operators --- //
        Vector6D operator+(const Vector6D& vector) const;     // Overloaded operator for addition
        Vector6D operator-(const Vector6D& vector) const ;    // Overloaded operator for substraction
        Vector6D operator*(double number) const;              // Overloaded operator for multiplication with a number
};


// Get a component of the vector
inline double Vector6D::getValue(int index) const {
    assert(index >= 0 && index < 6);
    return array[index];
}

// Set a component of the vector
inline void Vector6D::setValue(int index, double value) {
    assert(index >= 0 && index < 6);
    array[index] = value;
}

// Set all the values of the vector (inline)
inline void Vector6D::setAllValues(double v1, double v2, double v3, double v4, double v5, double v6) {
    array[0] = v1; array[1] = v2; array[2] = v3; array[3] = v4; array[4] = v5; array[5] = v6;
}

// Init the all the values with the same value
inline void Vector6D::initWithValue(double value) {
    array[0] = value; array[1] = value; array[2] = value; array[3] = value; array[4] = value; array[5] = value;
}

// Overloaded operator for multiplication between a number and a Vector3D (inline)
inline Vector6D operator*(double number, const Vector6D& vector) {
    return vector * number;
}

// Overloaded operator for addition
inline Vector6D Vector6D::operator+(const Vector6D& vector) const {
    return Vector6D(array[0] + vector.array[0], array[1] + vector.array[1], array[2] + vector.array[2],
                    array[3] + vector.array[3], array[4] + vector.array[4], array[5] + vector.array[5]);
}

// Overloaded operator for substraction
inline Vector6D Vector6D::operator-(const Vector6D& vector) const {
    return Vector6D(array[0] - vector.array[0], array[1] - vector.array[1], array[2] - vector.array[2],
                    array[3] - vector.array[3], array[4] - vector.array[4], array[5] - vector.array[5]);
}

// Overloaded operator for multiplication with a number
inline Vector6D Vector6D::operator*(double value) const {
    return Vector6D(array[0] * value, array[1] * value, array[2] * value,
                    array[3] * value, array[4] * value, array[5] * value);
}

} // End of the ReactPhysics3D namespace

#endif
