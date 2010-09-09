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

// Libraries
#include "Vector3D.h"
#include <iostream>
#include <cassert>
#include <vector>

// Namespaces
using namespace reactphysics3d;

// Constructor of the class Vector3D
Vector3D::Vector3D()
         :x(0.0), y(0.0), z(0.0) {

}

// Constructor with arguments
Vector3D::Vector3D(double x, double y, double z)
         :x(x), y(y), z(z) {

}

// Copy-constructor
Vector3D::Vector3D(const Vector3D& vector)
         :x(vector.x), y(vector.y), z(vector.z) {

}

// Destructor
Vector3D::~Vector3D() {

}

// Return the corresponding unit vector
Vector3D Vector3D::getUnit() const throw(MathematicsException) {
    double lengthVector = length();

    // Check if the length is equal to zero
    if (lengthVector != 0) {
        // Compute and return the unit vector
        double lengthInv = 1.0 / lengthVector;
        return Vector3D(x * lengthInv, y * lengthInv, z*lengthInv);
    }
    else {
        // Throw an exception because the length of the vector is zero
        throw MathematicsException("MathematicsException : Impossible to compute the unit vector because the length of the vector is zero");
    }
}

// Return two unit orthogonal vectors of the current vector
Vector3D Vector3D::getOneOrthogonalVector() const {
    assert(!this->isZero());
    Vector3D unitVector = this->getUnit();

    // Compute a first orthogonal vector
    Vector3D vector1;
    if (!approxEqual(x, 0.0)) {   // If x != 0
        vector1.setY(x);
        vector1.setZ((-2*x*y*z + 2*x*z)/(2*(z*z + x*x)));
        vector1.setX((-x*y-z*vector1.getZ())/x);
    }
    else if (!approxEqual(y, 0.0)) { // If y != 0
        vector1.setZ(y);
        vector1.setX((-2*x*y*z + 2*x*y)/(2*(y*y + x*x)));
        vector1.setY((-z*y-x*vector1.getX())/y);
    }
    else if (!approxEqual(z, 0.0)) { // If z != 0
        vector1.setX(z);
        vector1.setY((-2*x*y*z + 2*y*z)/(2*(z*z + y*y)));
        vector1.setZ((-x*z-y*vector1.getY())/z);
    }

    assert(vector1.isUnit());
    return vector1;
}

// Overloaded operator for addition
Vector3D Vector3D::operator+(const Vector3D& vector) const {
    // Compute and return the sum of the two vectors
    return Vector3D(x + vector.x, y + vector.y, z + vector.z);
}

// Overloaded operator for substraction
Vector3D Vector3D::operator-(const Vector3D& vector) const {
    // Compute and return the substraction of the two vectors
    return Vector3D(x - vector.x, y - vector.y, z - vector.z);
}

// Overloaded operator for multiplication with a number
Vector3D Vector3D::operator*(double number) const {
    // Compute and return the result
    return Vector3D(x * number, y * number, z * number);
}

// Overloaded operator for the assignement to a Vector
Vector3D& Vector3D::operator=(const Vector3D& vector) {
    // Check for self-assignment
    if (this != &vector) {
        // Copy the vector
        x = vector.x;
        y = vector.y;
        z = vector.z;
    }

    // Return a reference to the vector
    return *this;
}
