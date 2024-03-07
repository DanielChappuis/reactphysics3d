/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2024 Daniel Chappuis                                       *
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

#ifndef REACTPHYSICS3D_VECTOR3_H
#define REACTPHYSICS3D_VECTOR3_H

// Libraries
#include <cassert>
#include <cmath>
#include <algorithm>
#include <reactphysics3d/decimal.h>
#include <reactphysics3d/mathematics/mathematics_common.h>
#include <reactphysics3d/configuration.h>

/// ReactPhysics3D namespace
namespace reactphysics3d {

// Struct Vector3
/**
 * This class represents a 3D vector.
 */
struct Vector3 {

    public:

        // -------------------- Attributes -------------------- //

        /// Component x
        decimal x;

        /// Component y
        decimal y;

        /// Component z
        decimal z;

        // -------------------- Methods -------------------- //

        /// Constructor of the struct Vector3
        Vector3();

        /// Constructor with arguments
        Vector3(decimal newX, decimal newY, decimal newZ);

        /// Set all the values of the vector
        void setAllValues(decimal newX, decimal newY, decimal newZ);

        /// Set the vector to zero
        void setToZero();

        /// Return the length of the vector
        decimal length() const;

        /// Return the square of the length of the vector
        decimal lengthSquare() const;

        /// Return the corresponding unit vector
        Vector3 getUnit() const;

        /// Return one unit orthogonal vector of the current vector
        Vector3 getOneUnitOrthogonalVector() const;

        /// Return true if the vector is unit and false otherwise
        bool isUnit() const;

        /// Return true if the values are not NAN OR INF
        bool isFinite() const;

        /// Return true if the current vector is the zero vector
        bool isZero() const;

        /// Dot product of two vectors
        decimal dot(const Vector3& vector) const;

        /// Cross product of two vectors
        Vector3 cross(const Vector3& vector) const;

        /// Normalize the vector
        void normalize();

        /// Return the corresponding absolute value vector
        Vector3 getAbsoluteVector() const;

        /// Return the axis with the minimal value
        int getMinAxis() const;

        /// Return the axis with the maximal value
        int getMaxAxis() const;

        /// Return the minimum value among the three components of a vector
        decimal getMinValue() const;

        /// Return the maximum value among the three components of a vector
        decimal getMaxValue() const;

        /// Overloaded operator for the equality condition
        bool operator== (const Vector3& vector) const;

        /// Overloaded operator for the is different condition
        bool operator!= (const Vector3& vector) const;

        /// Overloaded operator for addition with assignment
        Vector3& operator+=(const Vector3& vector);

        /// Overloaded operator for substraction with assignment
        Vector3& operator-=(const Vector3& vector);

        /// Overloaded operator for multiplication with a number with assignment
        Vector3& operator*=(decimal number);

        /// Overloaded operator for division by a number with assignment
        Vector3& operator/=(decimal number);

        /// Overloaded operator for value access
        decimal& operator[] (int index);

        /// Overloaded operator for value access
        const decimal& operator[] (int index) const;

        /// Overloaded less than operator for ordering to be used inside std::set for instance
        bool operator<(const Vector3& vector) const;

        /// Get the string representation
        std::string to_string() const;

        /// Return a vector taking the minimum components of two vectors
        static Vector3 min(const Vector3& vector1, const Vector3& vector2);

        /// Return a vector taking the maximum components of two vectors
        static Vector3 max(const Vector3& vector1, const Vector3& vector2);

        /// Return the zero vector
        static Vector3 zero();

        /// Function to test if two vectors are (almost) equal
        static bool approxEqual(const Vector3& vec1, const Vector3& vec2, decimal epsilon = MACHINE_EPSILON);

        // -------------------- Friends -------------------- //

        friend Vector3 operator+(const Vector3& vector1, const Vector3& vector2);
        friend Vector3 operator-(const Vector3& vector1, const Vector3& vector2);
        friend Vector3 operator-(const Vector3& vector);
        friend Vector3 operator*(const Vector3& vector, decimal number);
        friend Vector3 operator*(decimal number, const Vector3& vector);
        friend Vector3 operator*(const Vector3& vector1, const Vector3& vector2);
        friend Vector3 operator/(const Vector3& vector, decimal number);
        friend Vector3 operator/(const Vector3& vector1, const Vector3& vector2);
};

// Constructor of the struct Vector3
RP3D_FORCE_INLINE Vector3::Vector3() : x(0.0), y(0.0), z(0.0) {

}

// Constructor with arguments
RP3D_FORCE_INLINE Vector3::Vector3(decimal newX, decimal newY, decimal newZ) : x(newX), y(newY), z(newZ) {

}

// Set the vector to zero
RP3D_FORCE_INLINE void Vector3::setToZero() {
    x = 0;
    y = 0;
    z = 0;
}

// Set all the values of the vector
RP3D_FORCE_INLINE void Vector3::setAllValues(decimal newX, decimal newY, decimal newZ) {
    x = newX;
    y = newY;
    z = newZ;
}

// Return the length of the vector
RP3D_FORCE_INLINE decimal Vector3::length() const {
    return std::sqrt(x*x + y*y + z*z);
}

// Return the square of the length of the vector
RP3D_FORCE_INLINE decimal Vector3::lengthSquare() const {
    return x*x + y*y + z*z;
}

// Scalar product of two vectors (RP3D_FORCE_INLINE)
RP3D_FORCE_INLINE decimal Vector3::dot(const Vector3& vector) const {
    return (x*vector.x + y*vector.y + z*vector.z);
}

// Cross product of two vectors (RP3D_FORCE_INLINE)
RP3D_FORCE_INLINE Vector3 Vector3::cross(const Vector3& vector) const {
    return Vector3(y * vector.z - z * vector.y,
                   z * vector.x - x * vector.z,
                   x * vector.y - y * vector.x);
}

// Normalize the vector
RP3D_FORCE_INLINE void Vector3::normalize() {
    decimal l = length();
    if (l < MACHINE_EPSILON) {
        return;
    }
    x /= l;
    y /= l;
    z /= l;
}

// Return the corresponding absolute value vector
RP3D_FORCE_INLINE Vector3 Vector3::getAbsoluteVector() const {
    return Vector3(std::abs(x), std::abs(y), std::abs(z));
}

// Return the axis with the minimal value
RP3D_FORCE_INLINE int Vector3::getMinAxis() const {
    return (x < y ? (x < z ? 0 : 2) : (y < z ? 1 : 2));
}

// Return the axis with the maximal value
RP3D_FORCE_INLINE int Vector3::getMaxAxis() const {
    return (x < y ? (y < z ? 2 : 1) : (x < z ? 2 : 0));
}

// Return true if the vector is unit and false otherwise
RP3D_FORCE_INLINE bool Vector3::isUnit() const {
    return reactphysics3d::approxEqual(lengthSquare(), decimal(1.0));
}

// Return true if the values are not NAN OR INF
RP3D_FORCE_INLINE bool Vector3::isFinite() const {
    return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}

// Return true if the vector is the zero vector
RP3D_FORCE_INLINE bool Vector3::isZero() const {
    return reactphysics3d::approxEqual(lengthSquare(), decimal(0.0));
}

// Overloaded operator for the equality condition
RP3D_FORCE_INLINE bool Vector3::operator== (const Vector3& vector) const {
    return (x == vector.x && y == vector.y && z == vector.z);
}

// Overloaded operator for the is different condition
RP3D_FORCE_INLINE bool Vector3::operator!= (const Vector3& vector) const {
    return !(*this == vector);
}

// Overloaded operator for addition with assignment
RP3D_FORCE_INLINE Vector3& Vector3::operator+=(const Vector3& vector) {
    x += vector.x;
    y += vector.y;
    z += vector.z;
    return *this;
}

// Overloaded operator for substraction with assignment
RP3D_FORCE_INLINE Vector3& Vector3::operator-=(const Vector3& vector) {
    x -= vector.x;
    y -= vector.y;
    z -= vector.z;
    return *this;
}

// Overloaded operator for multiplication with a number with assignment
RP3D_FORCE_INLINE Vector3& Vector3::operator*=(decimal number) {
    x *= number;
    y *= number;
    z *= number;
    return *this;
}

// Overloaded operator for division by a number with assignment
RP3D_FORCE_INLINE Vector3& Vector3::operator/=(decimal number) {
    assert(number > std::numeric_limits<decimal>::epsilon());
    x /= number;
    y /= number;
    z /= number;
    return *this;
}

// Overloaded operator for value access
RP3D_FORCE_INLINE decimal& Vector3::operator[] (int index) {
    return (&x)[index];
}

// Overloaded operator for value access
RP3D_FORCE_INLINE const decimal& Vector3::operator[] (int index) const {
    return (&x)[index];
}

// Overloaded operator for addition
RP3D_FORCE_INLINE Vector3 operator+(const Vector3& vector1, const Vector3& vector2) {
    return Vector3(vector1.x + vector2.x, vector1.y + vector2.y, vector1.z + vector2.z);
}

// Overloaded operator for substraction
RP3D_FORCE_INLINE Vector3 operator-(const Vector3& vector1, const Vector3& vector2) {
    return Vector3(vector1.x - vector2.x, vector1.y - vector2.y, vector1.z - vector2.z);
}

// Overloaded operator for the negative of a vector
RP3D_FORCE_INLINE Vector3 operator-(const Vector3& vector) {
    return Vector3(-vector.x, -vector.y, -vector.z);
}

// Overloaded operator for multiplication with a number
RP3D_FORCE_INLINE Vector3 operator*(const Vector3& vector, decimal number) {
    return Vector3(number * vector.x, number * vector.y, number * vector.z);
}

// Overloaded operator for division by a number
RP3D_FORCE_INLINE Vector3 operator/(const Vector3& vector, decimal number) {
    assert(number > MACHINE_EPSILON);
    return Vector3(vector.x / number, vector.y / number, vector.z / number);
}

// Overload operator for division between two vectors
RP3D_FORCE_INLINE Vector3 operator/(const Vector3& vector1, const Vector3& vector2) {
    assert(vector2.x > MACHINE_EPSILON);
    assert(vector2.y > MACHINE_EPSILON);
    assert(vector2.z > MACHINE_EPSILON);
    return Vector3(vector1.x / vector2.x, vector1.y / vector2.y, vector1.z / vector2.z);
}

// Overloaded operator for multiplication with a number
RP3D_FORCE_INLINE Vector3 operator*(decimal number, const Vector3& vector) {
    return vector * number;
}

// Overload operator for multiplication between two vectors
RP3D_FORCE_INLINE Vector3 operator*(const Vector3& vector1, const Vector3& vector2) {
    return Vector3(vector1.x * vector2.x, vector1.y * vector2.y, vector1.z * vector2.z);
}

// Overloaded less than operator for ordering to be used inside std::set for instance
RP3D_FORCE_INLINE bool Vector3::operator<(const Vector3& vector) const {
    return (x == vector.x ? (y == vector.y ? z < vector.z : y < vector.y) : x < vector.x);
}

// Return a vector taking the minimum components of two vectors
RP3D_FORCE_INLINE Vector3 Vector3::min(const Vector3& vector1, const Vector3& vector2) {
    return Vector3(std::min(vector1.x, vector2.x),
                   std::min(vector1.y, vector2.y),
                   std::min(vector1.z, vector2.z));
}

// Return a vector taking the maximum components of two vectors
RP3D_FORCE_INLINE Vector3 Vector3::max(const Vector3& vector1, const Vector3& vector2) {
    return Vector3(std::max(vector1.x, vector2.x),
                   std::max(vector1.y, vector2.y),
                   std::max(vector1.z, vector2.z));
}

// Return the minimum value among the three components of a vector
RP3D_FORCE_INLINE decimal Vector3::getMinValue() const {
    return std::min(std::min(x, y), z);
}

// Return the maximum value among the three components of a vector
RP3D_FORCE_INLINE decimal Vector3::getMaxValue() const {
    return std::max(std::max(x, y), z);
}

// Get the string representation
RP3D_FORCE_INLINE std::string Vector3::to_string() const {
    return "Vector3(" + std::to_string(x) + "," + std::to_string(y) + "," + std::to_string(z) + ")";
}

// Return the zero vector
RP3D_FORCE_INLINE Vector3 Vector3::zero() {
    return Vector3(0, 0, 0);
}

// Function to test if two vectors are (almost) equal
RP3D_FORCE_INLINE bool Vector3::approxEqual(const Vector3& vec1, const Vector3& vec2, decimal epsilon) {
    return reactphysics3d::approxEqual(vec1.x, vec2.x, epsilon) && reactphysics3d::approxEqual(vec1.y, vec2.y, epsilon) &&
           reactphysics3d::approxEqual(vec1.z, vec2.z, epsilon);
}

}

#endif
