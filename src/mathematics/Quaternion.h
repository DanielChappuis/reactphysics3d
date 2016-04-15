/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2016 Daniel Chappuis                                       *
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

#ifndef REACTPHYSICS3D_QUATERNION_H
#define REACTPHYSICS3D_QUATERNION_H

// Libraries
#include <cmath>
#include "Vector3.h"
#include "Matrix3x3.h"
#include "decimal.h"

/// ReactPhysics3D namespace
namespace reactphysics3d {

// Class Quaternion
/**
 * This class represents a quaternion. We use the notation :
 * q = (x*i, y*j, z*k, w) to represent a quaternion.
 */
struct Quaternion {

    public :

        // -------------------- Attributes -------------------- //

        /// Component x
        decimal x;

        /// Component y
        decimal y;

        /// Component z
        decimal z;

        /// Component w
        decimal w;

        // -------------------- Methods -------------------- //

        /// Constructor
        Quaternion();

        /// Constructor with arguments
        Quaternion(decimal newX, decimal newY, decimal newZ, decimal newW);

        /// Constructor with the component w and the vector v=(x y z)
        Quaternion(decimal newW, const Vector3& v);

        /// Constructor which convert Euler angles (in radians) to a quaternion
        Quaternion(decimal angleX, decimal angleY, decimal angleZ);

        /// Constructor which convert Euler angles (in radians) to a quaternion
        Quaternion(const Vector3& eulerAngles);

        /// Copy-constructor
        Quaternion(const Quaternion& quaternion);

        /// Create a unit quaternion from a rotation matrix
        Quaternion(const Matrix3x3& matrix);

        /// Destructor
        ~Quaternion();

        /// Set all the values
        void setAllValues(decimal newX, decimal newY, decimal newZ, decimal newW);

        /// Set the quaternion to zero
        void setToZero();

        /// Set to the identity quaternion
        void setToIdentity();

        /// Return the vector v=(x y z) of the quaternion
        Vector3 getVectorV() const;

        /// Return the length of the quaternion
        decimal length() const;

        /// Return the square of the length of the quaternion
        decimal lengthSquare() const;

        /// Normalize the quaternion
        void normalize();

        /// Inverse the quaternion
        void inverse();

        /// Return the unit quaternion
        Quaternion getUnit() const;

        /// Return the conjugate quaternion
        Quaternion getConjugate() const;

        /// Return the inverse of the quaternion
        Quaternion getInverse() const;

        /// Return the orientation matrix corresponding to this quaternion
        Matrix3x3 getMatrix() const;

        /// Return the identity quaternion
        static Quaternion identity();

        /// Dot product between two quaternions
        decimal dot(const Quaternion& quaternion) const;

        /// Compute the rotation angle (in radians) and the rotation axis
        void getRotationAngleAxis(decimal& angle, Vector3& axis) const;

        /// Compute the spherical linear interpolation between two quaternions
        static Quaternion slerp(const Quaternion& quaternion1, const Quaternion& quaternion2,
                                decimal t);

        /// Overloaded operator for the addition
        Quaternion operator+(const Quaternion& quaternion) const;

        /// Overloaded operator for the substraction
        Quaternion operator-(const Quaternion& quaternion) const;

        /// Overloaded operator for addition with assignment
        Quaternion& operator+=(const Quaternion& quaternion);

        /// Overloaded operator for substraction with assignment
        Quaternion& operator-=(const Quaternion& quaternion);

        /// Overloaded operator for the multiplication with a constant
        Quaternion operator*(decimal nb) const;

        /// Overloaded operator for the multiplication
        Quaternion operator*(const Quaternion& quaternion) const;

        /// Overloaded operator for the multiplication with a vector
        Vector3 operator*(const Vector3& point) const;

        /// Overloaded operator for assignment
        Quaternion& operator=(const Quaternion& quaternion);

        /// Overloaded operator for equality condition
        bool operator==(const Quaternion& quaternion) const;

    private:

        /// Initialize the quaternion using Euler angles
        void initWithEulerAngles(decimal angleX, decimal angleY, decimal angleZ);
};

/// Set all the values
inline void Quaternion::setAllValues(decimal newX, decimal newY, decimal newZ, decimal newW) {
    x = newX;
    y = newY;
    z = newZ;
    w = newW;
}

/// Set the quaternion to zero
inline void Quaternion::setToZero() {
    x = 0;
    y = 0;
    z = 0;
    w = 0;
}

// Set to the identity quaternion
inline void Quaternion::setToIdentity() {
    x = 0;
    y = 0;
    z = 0;
    w = 1;
}

// Return the vector v=(x y z) of the quaternion
inline Vector3 Quaternion::getVectorV() const {

    // Return the vector v
    return Vector3(x, y, z);
}

// Return the length of the quaternion (inline)
inline decimal Quaternion::length() const {
    return sqrt(x*x + y*y + z*z + w*w);
}

// Return the square of the length of the quaternion
inline decimal Quaternion::lengthSquare() const {
    return x*x + y*y + z*z + w*w;
}

// Normalize the quaternion
inline void Quaternion::normalize() {

    decimal l = length();

    // Check if the length is not equal to zero
    assert (l > MACHINE_EPSILON);

    x /= l;
    y /= l;
    z /= l;
    w /= l;
}

// Inverse the quaternion
inline void Quaternion::inverse() {

    // Get the square length of the quaternion
    decimal lengthSquareQuaternion = lengthSquare();

    assert (lengthSquareQuaternion > MACHINE_EPSILON);

    // Compute and return the inverse quaternion
    x /= -lengthSquareQuaternion;
    y /= -lengthSquareQuaternion;
    z /= -lengthSquareQuaternion;
    w /= lengthSquareQuaternion;
}

// Return the unit quaternion
inline Quaternion Quaternion::getUnit() const {
    decimal lengthQuaternion = length();

    // Check if the length is not equal to zero
    assert (lengthQuaternion > MACHINE_EPSILON);

    // Compute and return the unit quaternion
    return Quaternion(x / lengthQuaternion, y / lengthQuaternion,
                      z / lengthQuaternion, w / lengthQuaternion);
}

// Return the identity quaternion
inline Quaternion Quaternion::identity() {
    return Quaternion(0.0, 0.0, 0.0, 1.0);
}

// Return the conjugate of the quaternion (inline)
inline Quaternion Quaternion::getConjugate() const {
    return Quaternion(-x, -y, -z, w);
}

// Return the inverse of the quaternion (inline)
inline Quaternion Quaternion::getInverse() const {

    decimal lengthSquareQuaternion = lengthSquare();

    assert (lengthSquareQuaternion > MACHINE_EPSILON);

    // Compute and return the inverse quaternion
    return Quaternion(-x / lengthSquareQuaternion, -y / lengthSquareQuaternion,
                      -z / lengthSquareQuaternion, w / lengthSquareQuaternion);
}

// Scalar product between two quaternions
inline decimal Quaternion::dot(const Quaternion& quaternion) const {
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
    return Quaternion(x - quaternion.x, y - quaternion.y, z - quaternion.z, w - quaternion.w);
}

// Overloaded operator for addition with assignment
inline Quaternion& Quaternion::operator+=(const Quaternion& quaternion) {
    x += quaternion.x;
    y += quaternion.y;
    z += quaternion.z;
    w += quaternion.w;
    return *this;
}

// Overloaded operator for substraction with assignment
inline Quaternion& Quaternion::operator-=(const Quaternion& quaternion) {
    x -= quaternion.x;
    y -= quaternion.y;
    z -= quaternion.z;
    w -= quaternion.w;
    return *this;
}

// Overloaded operator for the multiplication with a constant
inline Quaternion Quaternion::operator*(decimal nb) const {
    return Quaternion(nb * x, nb * y, nb * z, nb * w);
}

// Overloaded operator for the multiplication of two quaternions
inline Quaternion Quaternion::operator*(const Quaternion& quaternion) const {
    return Quaternion(w * quaternion.w - getVectorV().dot(quaternion.getVectorV()),
                      w * quaternion.getVectorV() + quaternion.w * getVectorV() +
                      getVectorV().cross(quaternion.getVectorV()));
}

// Overloaded operator for the multiplication with a vector.
/// This methods rotates a point given the rotation of a quaternion.
inline Vector3 Quaternion::operator*(const Vector3& point) const {
    Quaternion p(point.x, point.y, point.z, 0.0);
    return (((*this) * p) * getConjugate()).getVectorV();
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
    return (x == quaternion.x && y == quaternion.y &&
            z == quaternion.z && w == quaternion.w);
}

}

#endif
