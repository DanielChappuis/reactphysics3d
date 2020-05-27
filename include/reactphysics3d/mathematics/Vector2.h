/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2020 Daniel Chappuis                                       *
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

#ifndef REACTPHYSICS3D_VECTOR2_H
#define REACTPHYSICS3D_VECTOR2_H

// Libraries
#include <cassert>
#include <reactphysics3d/mathematics/mathematics_functions.h>
#include <reactphysics3d/decimal.h>

/// ReactPhysics3D namespace
namespace reactphysics3d {

// Class Vector2
/**
 * This class represents a 2D vector.
 */
struct Vector2 {

    public:

        // -------------------- Attributes -------------------- //

        /// Component x
        decimal x;

        /// Component y
        decimal y;

        // -------------------- Methods -------------------- //

        /// Constructor of the struct Vector2
        Vector2();

        /// Constructor with arguments
        Vector2(decimal newX, decimal newY);

        /// Copy-constructor
        Vector2(const Vector2& vector);

        /// Destructor
        ~Vector2() = default;

        /// Set all the values of the vector
        void setAllValues(decimal newX, decimal newY);

        /// Set the vector to zero
        void setToZero();

        /// Return the length of the vector
        decimal length() const;

        /// Return the square of the length of the vector
        decimal lengthSquare() const;

        /// Return the corresponding unit vector
        Vector2 getUnit() const;

        /// Return one unit orthogonal vector of the current vector
        Vector2 getOneUnitOrthogonalVector() const;

        /// Return true if the vector is unit and false otherwise
        bool isUnit() const;

        /// Return true if the values are not NAN OR INF
        bool isFinite() const;

        /// Return true if the current vector is the zero vector
        bool isZero() const;

        /// Dot product of two vectors
        decimal dot(const Vector2& vector) const;

        /// Normalize the vector
        void normalize();

        /// Return the corresponding absolute value vector
        Vector2 getAbsoluteVector() const;

        /// Return the axis with the minimal value
        int getMinAxis() const;

        /// Return the axis with the maximal value
        int getMaxAxis() const;

        /// Overloaded operator for the equality condition
        bool operator== (const Vector2& vector) const;

        /// Overloaded operator for the is different condition
        bool operator!= (const Vector2& vector) const;

        /// Overloaded operator for addition with assignment
        Vector2& operator+=(const Vector2& vector);

        /// Overloaded operator for substraction with assignment
        Vector2& operator-=(const Vector2& vector);

        /// Overloaded operator for multiplication with a number with assignment
        Vector2& operator*=(decimal number);

        /// Overloaded operator for division by a number with assignment
        Vector2& operator/=(decimal number);

        /// Overloaded operator for value access
        decimal& operator[] (int index);

        /// Overloaded operator for value access
        const decimal& operator[] (int index) const;

        /// Overloaded operator
        Vector2& operator=(const Vector2& vector);

        /// Overloaded less than operator for ordering to be used inside std::set for instance
        bool operator<(const Vector2& vector) const;

        /// Return the string representation
        std::string to_string() const;

        /// Return a vector taking the minimum components of two vectors
        static Vector2 min(const Vector2& vector1, const Vector2& vector2);

        /// Return a vector taking the maximum components of two vectors
        static Vector2 max(const Vector2& vector1, const Vector2& vector2);

        /// Return the zero vector
        static Vector2 zero();

        // -------------------- Friends -------------------- //

        friend Vector2 operator+(const Vector2& vector1, const Vector2& vector2);
        friend Vector2 operator-(const Vector2& vector1, const Vector2& vector2);
        friend Vector2 operator-(const Vector2& vector);
        friend Vector2 operator*(const Vector2& vector, decimal number);
        friend Vector2 operator*(decimal number, const Vector2& vector);
        friend Vector2 operator*(const Vector2& vector1, const Vector2& vector2);
        friend Vector2 operator/(const Vector2& vector, decimal number);
        friend Vector2 operator/(const Vector2& vector1, const Vector2& vector2);
};

// Constructor
inline Vector2::Vector2() : x(0.0), y(0.0) {

}

// Constructor with arguments
inline Vector2::Vector2(decimal newX, decimal newY) : x(newX), y(newY) {

}

// Copy-constructor
inline Vector2::Vector2(const Vector2& vector) : x(vector.x), y(vector.y) {

}


// Set the vector to zero
inline void Vector2::setToZero() {
    x = 0;
    y = 0;
}

// Set all the values of the vector
inline void Vector2::setAllValues(decimal newX, decimal newY) {
    x = newX;
    y = newY;
}

// Return the length of the vector
inline decimal Vector2::length() const {
    return std::sqrt(x*x + y*y);
}

// Return the square of the length of the vector
inline decimal Vector2::lengthSquare() const {
    return x*x + y*y;
}

// Scalar product of two vectors (inline)
inline decimal Vector2::dot(const Vector2& vector) const {
    return (x*vector.x + y*vector.y);
}

// Normalize the vector
inline void Vector2::normalize() {
    decimal l = length();
    if (l < MACHINE_EPSILON) {
        return;
    }
    x /= l;
    y /= l;
}

// Return the corresponding absolute value vector
inline Vector2 Vector2::getAbsoluteVector() const {
    return Vector2(std::abs(x), std::abs(y));
}

// Return the axis with the minimal value
inline int Vector2::getMinAxis() const {
    return (x < y ? 0 : 1);
}

// Return the axis with the maximal value
inline int Vector2::getMaxAxis() const {
    return (x < y ? 1 : 0);
}

// Return true if the vector is unit and false otherwise
inline bool Vector2::isUnit() const {
    return approxEqual(lengthSquare(), 1.0);
}

// Return true if the values are not NAN OR INF
inline bool Vector2::isFinite() const {
    return std::isfinite(x) && std::isfinite(y);
}

// Return true if the vector is the zero vector
inline bool Vector2::isZero() const {
    return approxEqual(lengthSquare(), 0.0);
}

// Overloaded operator for the equality condition
inline bool Vector2::operator== (const Vector2& vector) const {
    return (x == vector.x && y == vector.y);
}

// Overloaded operator for the is different condition
inline bool Vector2::operator!= (const Vector2& vector) const {
    return !(*this == vector);
}

// Overloaded operator for addition with assignment
inline Vector2& Vector2::operator+=(const Vector2& vector) {
    x += vector.x;
    y += vector.y;
    return *this;
}

// Overloaded operator for substraction with assignment
inline Vector2& Vector2::operator-=(const Vector2& vector) {
    x -= vector.x;
    y -= vector.y;
    return *this;
}

// Overloaded operator for multiplication with a number with assignment
inline Vector2& Vector2::operator*=(decimal number) {
    x *= number;
    y *= number;
    return *this;
}

// Overloaded operator for division by a number with assignment
inline Vector2& Vector2::operator/=(decimal number) {
    assert(number > std::numeric_limits<decimal>::epsilon());
    x /= number;
    y /= number;
    return *this;
}

// Overloaded operator for value access
inline decimal& Vector2::operator[] (int index) {
    return (&x)[index];
}

// Overloaded operator for value access
inline const decimal& Vector2::operator[] (int index) const {
    return (&x)[index];
}

// Overloaded operator for addition
inline Vector2 operator+(const Vector2& vector1, const Vector2& vector2) {
    return Vector2(vector1.x + vector2.x, vector1.y + vector2.y);
}

// Overloaded operator for substraction
inline Vector2 operator-(const Vector2& vector1, const Vector2& vector2) {
    return Vector2(vector1.x - vector2.x, vector1.y - vector2.y);
}

// Overloaded operator for the negative of a vector
inline Vector2 operator-(const Vector2& vector) {
    return Vector2(-vector.x, -vector.y);
}

// Overloaded operator for multiplication with a number
inline Vector2 operator*(const Vector2& vector, decimal number) {
    return Vector2(number * vector.x, number * vector.y);
}

// Overloaded operator for multiplication of two vectors
inline Vector2 operator*(const Vector2& vector1, const Vector2& vector2) {
    return Vector2(vector1.x * vector2.x, vector1.y * vector2.y);
}

// Overloaded operator for division by a number
inline Vector2 operator/(const Vector2& vector, decimal number) {
    assert(number > MACHINE_EPSILON);
    return Vector2(vector.x / number, vector.y / number);
}

// Overload operator for division between two vectors
inline Vector2 operator/(const Vector2& vector1, const Vector2& vector2) {
    assert(vector2.x > MACHINE_EPSILON);
    assert(vector2.y > MACHINE_EPSILON);
    return Vector2(vector1.x / vector2.x, vector1.y / vector2.y);
}

// Overloaded operator for multiplication with a number
inline Vector2 operator*(decimal number, const Vector2& vector) {
    return vector * number;
}

// Assignment operator
inline Vector2& Vector2::operator=(const Vector2& vector) {
    if (&vector != this) {
        x = vector.x;
        y = vector.y;
    }
    return *this;
}

// Overloaded less than operator for ordering to be used inside std::set for instance
inline bool Vector2::operator<(const Vector2& vector) const {
    return (x == vector.x ? y < vector.y : x < vector.x);
}

// Return a vector taking the minimum components of two vectors
inline Vector2 Vector2::min(const Vector2& vector1, const Vector2& vector2) {
    return Vector2(std::min(vector1.x, vector2.x),
                   std::min(vector1.y, vector2.y));
}

// Return a vector taking the maximum components of two vectors
inline Vector2 Vector2::max(const Vector2& vector1, const Vector2& vector2) {
    return Vector2(std::max(vector1.x, vector2.x),
                   std::max(vector1.y, vector2.y));
}

// Get the string representation
inline std::string Vector2::to_string() const {
    return "Vector2(" + std::to_string(x) + "," + std::to_string(y) + ")";
}

// Return the zero vector
inline Vector2 Vector2::zero() {
    return Vector2(0, 0);
}

}

#endif
