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

#ifndef QUATERNION_H
#define QUATERNION_H

// Libraries
#include <cmath>
#include "Vector3D.h"
#include "exceptions.h"

// ReactPhysics3D namespace
namespace reactphysics3d {

/*  -------------------------------------------------------------------
    Class Quaternion :
        This class represents a quaternion. We use the notation :
        q = (x*i, y*j, z*k, w) to represent a quaternion.
    -------------------------------------------------------------------
*/
class Quaternion
{
    private :
        double x;                       // Component x of the quaternion
        double y;                       // Component y of the quaternion
        double z;                       // Component z of the quaternion
        double w;                       // Component w of the quaternion

    public :
        Quaternion();                                                         // Constructor
        Quaternion(double x, double y, double z, double w);                   // Constructor with arguments
        Quaternion(double w, const Vector3D& v);                              // Constructor with the component w and the vector v=(x y z)
        Quaternion(const Quaternion& quaternion);                             // Copy-constructor
        ~Quaternion();                                                        // Destructor
        double getX() const;                                                  // Return the component x of the quaternion
        double getY() const;                                                  // Return the component y of the quaternion
        double getZ() const;                                                  // Return the component z of the quaternion
        double getW() const;                                                  // Return the component w of the quaternion
        void setX(double x);                                                  // Set the value x
        void setY(double y);                                                  // Set the value y
        void setZ(double z);                                                  // Set the value z
        void setW(double w);                                                  // Set the value w
        Vector3D vectorV() const;                                             // Return the vector v=(x y z) of the quaternion
        double length() const;                                                // Return the length of the quaternion
        Quaternion getUnit() const throw (MathematicsException);              // Return the unit quaternion
        Quaternion getConjugate() const;                                      // Return the conjugate quaternion
        Quaternion getInverse() const throw (MathematicsException);           // Return the inverse of the quaternion
        double scalarProduct(const Quaternion& quaternion) const;             // Scalar product between two quaternions
        void getRotationAngleAxis(double& angle, Vector3D& axis) const;       // Compute the rotation angle (in radians) and the axis
        static Quaternion slerp(const Quaternion& quaternion1,
                                const Quaternion& quaternion2, double t);     // Compute the spherical linear interpolation between two quaternions

        // --- Overloaded operators --- //
        Quaternion operator+(const Quaternion& quaternion) const;             // Overloaded operator for the addition
        Quaternion operator-(const Quaternion& quaternion) const;             // Overloaded operator for the substraction
        Quaternion operator*(double nb) const;                                // Overloaded operator for the multiplication with a constant
        Quaternion operator*(const Quaternion& quaternion) const;             // Overloaded operator for the multiplication
        Quaternion& operator=(const Quaternion& quaternion);                  // Overloaded operator for assignment
        bool operator==(const Quaternion& quaternion) const;                  // Overloaded operator for equality condition
};

// --- Inline functions --- //

// Get the value x (inline)
inline double Quaternion::getX() const {
    return x;
}

// Get the value y (inline)
inline double Quaternion::getY() const {
    return y;
}

// Get the value z (inline)
inline double Quaternion::getZ() const {
    return z;
}

// Get the value w (inline)
inline double Quaternion::getW() const {
    return w;
}

// Set the value x (inline)
inline void Quaternion::setX(double x) {
    this->x = x;
}

// Set the value y (inline)
inline void Quaternion::setY(double y) {
    this->y = y;
}

// Set the value z (inline)
inline void Quaternion::setZ(double z) {
    this->z = z;
}

// Set the value w (inline)
inline void Quaternion::setW(double w) {
    this->w = w;
}

// Return the vector v=(x y z) of the quaternion
inline Vector3D Quaternion::vectorV() const {
    // Return the vector v
    return Vector3D(x, y, z);
}

// Return the length of the quaternion (inline)
inline double Quaternion::length() const {
    return sqrt(x*x + y*y + z*z + w*w);
}

// Return the unit quaternion
inline Quaternion Quaternion::getUnit() const throw(MathematicsException) {
    double lengthQuaternion = length();

    // Check if the length is not equal to zero
    if (lengthQuaternion != 0.0) {

        // Compute and return the unit quaternion
        return Quaternion(x/lengthQuaternion, y/lengthQuaternion, z/lengthQuaternion, w/lengthQuaternion);
    }
    else {
        // Throw an exception because it's impossible to compute a unit quaternion if its length is equal to zero
        throw MathematicsException("MathematicsException : Impossible to compute the unit quaternion if the length of the quaternion is zero");
    }
}

// Return the conjugate of the quaternion (inline)
inline Quaternion Quaternion::getConjugate() const {
    return Quaternion(-x, -y, -z, w);
}

// Return the inverse of the quaternion (inline)
inline Quaternion Quaternion::getInverse() const throw(MathematicsException) {
    double lengthQuaternion = length();
    lengthQuaternion = lengthQuaternion * lengthQuaternion;

    // Check if the length is not equal to zero
    if (lengthQuaternion != 0.0) {

        // Compute and return the inverse quaternion
        return Quaternion(-x/lengthQuaternion, -y/lengthQuaternion, -z/lengthQuaternion, w/lengthQuaternion);
    }
    else {
        // Throw an exception because the inverse cannot be computed
        throw MathematicsException("MathematicsException : Impossible to compute the inverse of the quaternion because it's length is zero");
    }
}

// Scalar product between two quaternions
// TODO : Test for this method
inline double Quaternion::scalarProduct(const Quaternion& quaternion) const {
    return (x*quaternion.x + y*quaternion.y + z*quaternion.z + w*quaternion.w);
}

// Overloaded operator for the addition of two quaternions
inline Quaternion Quaternion::operator+(const Quaternion& quaternion) const {
    // Return the result quaternion
    return Quaternion(x + quaternion.x, y + quaternion.y, z + quaternion.z, w + quaternion.w);
}

// Overloaded operator for the substraction of two quaternions
inline Quaternion Quaternion::operator-(const Quaternion& quaternion) const {
    // Return the result of the substraction
    return Quaternion(x-quaternion.x, y - quaternion.y, z - quaternion.z, w - quaternion.w);
}

// Overloaded operator for the multiplication with a constant
inline Quaternion Quaternion::operator*(double nb) const {
    // Return the result
    return Quaternion(nb*x, nb*y, nb*z, nb*w);
}

// Overloaded operator for the multiplication of two quaternions
inline Quaternion Quaternion::operator*(const Quaternion& quaternion) const {
    // Return the result of the multiplication
    return Quaternion(w*quaternion.w - vectorV().scalarProduct(quaternion.vectorV()), w*quaternion.vectorV()+quaternion.w*vectorV() + vectorV().crossProduct(quaternion.vectorV()));
}

// Overloaded operator for the assignment
inline Quaternion& Quaternion::operator=(const Quaternion& quaternion) {
    // Check for self-assignment
    if (this != &quaternion) {
        x = quaternion.x;
        y = quaternion.y;
        z = quaternion.z;
        w = quaternion.w;
    }

    // Return this quaternion
    return *this;
}

// Overloaded operator for equality condition
inline bool Quaternion::operator==(const Quaternion& quaternion) const {
    return (x == quaternion.x && y == quaternion.y && z == quaternion.z && w == quaternion.w);
}

} // End of the ReactPhysics3D namespace

#endif
