/****************************************************************************
 * Copyright (C) 2009      Daniel Chappuis                                  *
 ****************************************************************************
 * This file is part of ReactPhysics3D.                                     *
 *                                                                          *
 * ReactPhysics3D is free software: you can redistribute it and/or modify   *
 * it under the terms of the GNU General Public License as published by     *
 * the Free Software Foundation, either version 3 of the License, or        *
 * (at your option) any later version.                                      *
 *                                                                          *
 * ReactPhysics3D is distributed in the hope that it will be useful,        *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of           *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the             *
 * GNU General Public License for more details.                             *
 *                                                                          *
 * You should have received a copy of the GNU General Public License        *
 * along with ReactPhysics3D. If not, see <http://www.gnu.org/licenses/>.   *
 ***************************************************************************/

// Libraries
#include "Vector3D.h"
#include <iostream>

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
