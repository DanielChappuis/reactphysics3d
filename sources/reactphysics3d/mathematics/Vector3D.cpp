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
// TODO : Test this method
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
