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

#ifndef VECTOR3D_H
#define VECTOR3D_H

// Libraries
#include <cmath>
#include "exceptions.h"
#include "mathematics_functions.h"


// ReactPhysics3D namespace
namespace reactphysics3d {

/*  -------------------------------------------------------------------
    Class Vector3D :
        This classrepresents 3 dimensionnal vector in space.
    -------------------------------------------------------------------
*/
class Vector3D {
    private :
        double x;                                               // X component of the vector
        double y;                                               // Y component of the vector
        double z;                                               // Z component of the vector

    public :
        Vector3D();                                             // Constructor of the class Vector3D
        Vector3D(double x, double y, double z);                 // Constructor with arguments
        Vector3D(const Vector3D& vector);                       // Copy-constructor
        virtual ~Vector3D();                                    // Destructor
        double getX() const;                                    // Get the x component of the vector
        double getY() const;                                    // Get the y component of the vector
        double getZ() const;                                    // Get the z component of the vector
        void setX(double x);                                    // Set the x component of the vector
        void setY(double y);                                    // Set the y component of the vector
        void setZ(double z);                                    // Set the z component of the vector
        void setAllValues(double x, double y, double z);        // Set all the values of the vector
        double length() const;                                  // Return the lenght of the vector
        Vector3D getUnit() const throw(MathematicsException);   // Return the corresponding unit vector
        bool isUnit() const;                                    // Return true if the vector is unit and false otherwise
        bool isZero() const;                                    // Return true if the current vector is the zero vector
        Vector3D getOpposite() const;                           // Return the vector in the opposite direction
        Vector3D getOneOrthogonalVector() const;               // Return one unit orthogonal vectors of the current vector
        double scalarProduct(const Vector3D& vector) const;     // Scalar product of two vectors
        Vector3D crossProduct(const Vector3D& vector) const;    // Cross product of two vectors
        bool isParallelWith(const Vector3D& vector) const;      // Return true if two vectors are parallel

        // --- Overloaded operators --- //
        Vector3D operator+(const Vector3D& vector) const;     // Overloaded operator for addition
        Vector3D operator-(const Vector3D& vector) const ;    // Overloaded operator for substraction
        Vector3D operator*(double number) const;              // Overloaded operator for multiplication with a number
        Vector3D& operator=(const Vector3D& vector);          // Overloaded operator for the assignement to a Vector
        bool operator==(const Vector3D& vector) const;        // Overloaded operator for the equality condition
};

// Get the x component of the vector (inline)
inline double Vector3D::getX() const {
    return x;
}

// Get the y component of the vector (inline)
inline double Vector3D::getY() const {
    return y;
}

// Get the z component of the vector (inline)
inline double Vector3D::getZ() const {
    return z;
}

// Set the x component of the vector (inline)
inline void Vector3D::setX(double x) {
    this->x = x;
}

// Set the y component of the vector (inline)
inline void Vector3D::setY(double y) {
    this->y = y;
}

// Set the z component of the vector (inline)
inline void Vector3D::setZ(double z) {
    this->z = z;
}

// Set all the values of the vector (inline)
inline void Vector3D::setAllValues(double x, double y, double z) {
    this->x = x;
    this->y = y;
    this->z = z;
}

// Return the length of the vector (inline)
inline double Vector3D::length() const {
    // Compute and return the length of the vector
    return sqrt(x*x + y*y + z*z);
}

// Return the vector in the opposite direction
// TODO : Test this function
inline Vector3D Vector3D::getOpposite() const {
    return (Vector3D(0.0, 0.0, 0.0) - *this);
}

// Scalar product of two vectors (inline)
inline double Vector3D::scalarProduct(const Vector3D& vector) const {
    // Compute and return the result of the scalar product
    return (x * vector.x + y * vector.y + z * vector.z);
}

// Cross product of two vectors (inline)
inline Vector3D Vector3D::crossProduct(const Vector3D& vector) const {
    // Compute and return the cross product
    return Vector3D(y * vector.z - z * vector.y, z * vector.x - x * vector.z , x * vector.y - y * vector.x);
}

// Return true if two vectors are parallel
inline bool Vector3D::isParallelWith(const Vector3D& vector) const {
    double scalarProd = this->scalarProduct(vector);
    return approxEqual(std::abs(scalarProd), length() * vector.length());
}

// Return true if the vector is unit and false otherwise
inline bool Vector3D::isUnit() const {
    return approxEqual(x*x+y*y+z*z, 1.0);
}

// Return true if the vector is the zero vector
inline bool Vector3D::isZero() const {
    return approxEqual(x*x+y*y+z*z, 0.0);
}

// Overloaded operator for multiplication between a number and a Vector3D (inline)
inline Vector3D operator * (double number, const Vector3D& vector) {
    // Compute and return the result vector
    return vector * number;
}

// Overloaded operator for the equality condition
inline bool Vector3D::operator == (const Vector3D& vector) const {
    return (x == vector.x && y == vector.y && z == vector.z);
}

} // End of the ReactPhysics3D namespace

#endif
