/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2010-2012 Daniel Chappuis                                       *
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

#ifndef VECTOR3_H
#define VECTOR3_H

// Libraries
#include <cmath>
#include <cassert>
#include "mathematics_functions.h"
#include "../decimal.h"


// ReactPhysics3D namespace
namespace reactphysics3d {

/*  -------------------------------------------------------------------
    Class Vector3 :
        This class represents 3D vector in space.
    -------------------------------------------------------------------
*/
class Vector3 {

    private :

        // -------------------- Attributes -------------------- //

        // Values of the 3D vector
        decimal mValues[3];

    public :

        // -------------------- Methods -------------------- //

        // Constructor of the class Vector3D
        Vector3();

        // Constructor with arguments
        Vector3(decimal x, decimal y, decimal z);

        // Copy-constructor
        Vector3(const Vector3& vector);

        // Destructor
        ~Vector3();

        // Get the x component of the vector
        decimal getX() const;

        // Get the y component of the vector
        decimal getY() const;

        // Get the z component of the vector
        decimal getZ() const;

        // Set the x component of the vector
        void setX(decimal x);

        // Set the y component of the vector
        void setY(decimal y);

        // Set the z component of the vector
        void setZ(decimal z);

        // Set all the values of the vector
        void setAllValues(decimal x, decimal y, decimal z);

        // Return the lenght of the vector
        decimal length() const;

        // Return the square of the length of the vector
        decimal lengthSquare() const;

        // Return the corresponding unit vector
        Vector3 getUnit() const;

        // Return one unit orthogonal vector of the current vector
        Vector3 getOneUnitOrthogonalVector() const;

        // Return true if the vector is unit and false otherwise
        bool isUnit() const;

        // Return true if the current vector is the zero vector
        bool isZero() const;

        // Return one unit orthogonal vectors of the current vector
        Vector3 getOneOrthogonalVector() const;

        // Dot product of two vectors
        decimal dot(const Vector3& vector) const;

        // Cross product of two vectors
        Vector3 cross(const Vector3& vector) const;

        // Normalize the vector
        void normalize();

        // Return the corresponding absolute value vector
        Vector3 getAbsoluteVector() const;

        // Return the axis with the minimal value
        int getMinAxis() const;

        // Return the axis with the maximal value
        int getMaxAxis() const;

        // Return true if two vectors are parallel
        bool isParallelWith(const Vector3& vector) const;

        // Overloaded operator for the equality condition
        bool operator== (const Vector3& vector) const;

        // Overloaded operator for the is different condition
        bool operator!= (const Vector3& vector) const;

        // Overloaded operator for addition with assignment
        Vector3& operator+=(const Vector3& vector);

        // Overloaded operator for substraction with assignment
        Vector3& operator-=(const Vector3& vector);

        // Overloaded operator for multiplication with a number with assignment
        Vector3& operator*=(decimal number);

        // Overloaded operator for division by a number with assignment
        Vector3& operator/=(decimal number);

        // Overloaded operator for value access
        decimal& operator[] (int index);

        // Overloaded operator for value access
        const decimal& operator[] (int index) const;

        // Overloaded operator
        Vector3& operator=(const Vector3& vector);

        // -------------------- Friends -------------------- //

        friend Vector3 operator+(const Vector3& vector1, const Vector3& vector2);
        friend Vector3 operator-(const Vector3& vector1, const Vector3& vector2);
        friend Vector3 operator-(const Vector3& vector);
        friend Vector3 operator*(const Vector3& vector, decimal number);
        friend Vector3 operator*(decimal number, const Vector3& vector);
        friend Vector3 operator/(const Vector3& vector, decimal number);
};

// Get the x component of the vector
inline decimal Vector3::getX() const {
    return mValues[0];
}

// Get the y component of the vector
inline decimal Vector3::getY() const {
    return mValues[1];
}

// Get the z component of the vector
inline decimal Vector3::getZ() const {
    return mValues[2];
}

// Set the x component of the vector
inline void Vector3::setX(decimal x) {
    this->mValues[0] = x;
}

// Set the y component of the vector
inline void Vector3::setY(decimal y) {
    this->mValues[1] = y;
}

// Set the z component of the vector
inline void Vector3::setZ(decimal z) {
    this->mValues[2] = z;
}

// Set all the values of the vector (inline)
inline void Vector3::setAllValues(decimal x, decimal y, decimal z) {
    mValues[0]= x;
    mValues[1] = y;
    mValues[2] = z;
}

// Return the length of the vector (inline)
inline decimal Vector3::length() const {
    // Compute and return the length of the vector
    return sqrt(mValues[0]*mValues[0] + mValues[1]*mValues[1] + mValues[2]*mValues[2]);
}

// Return the square of the length of the vector
inline decimal Vector3::lengthSquare() const {
    return mValues[0]*mValues[0] + mValues[1]*mValues[1] + mValues[2]*mValues[2];
}

// Scalar product of two vectors (inline)
inline decimal Vector3::dot(const Vector3& vector) const {
    // Compute and return the result of the scalar product
    return (mValues[0] * vector.mValues[0] + mValues[1] * vector.mValues[1] + mValues[2] * vector.mValues[2]);
}

// Cross product of two vectors (inline)
inline Vector3 Vector3::cross(const Vector3& vector) const {
    // Compute and return the cross product
    return Vector3(mValues[1] * vector.mValues[2] - mValues[2] * vector.mValues[1],
                   mValues[2] * vector.mValues[0] - mValues[0] * vector.mValues[2],
                   mValues[0] * vector.mValues[1] - mValues[1] * vector.mValues[0]);
}

// Normalize the vector
inline void Vector3::normalize() {
    decimal l = length();
    assert(l != 0.0);
    mValues[0] /= l;
    mValues[1] /= l;
    mValues[2] /= l;
}

// Return the corresponding absolute value vector
inline Vector3 Vector3::getAbsoluteVector() const {
    return Vector3(std::abs(mValues[0]), std::abs(mValues[1]), std::abs(mValues[2]));
}       

// Return true if two vectors are parallel
inline bool Vector3::isParallelWith(const Vector3& vector) const {
    decimal scalarProd = this->dot(vector);
    return approxEqual(std::abs(scalarProd), length() * vector.length());
}    


// Return the axis with the minimal value
inline int Vector3::getMinAxis() const {
    return (mValues[0] < mValues[1] ? (mValues[0] < mValues[2] ? 0 : 2) : (mValues[1] < mValues[2] ? 1 : 2));
}

// Return the axis with the maximal value
inline int Vector3::getMaxAxis() const {
    return (mValues[0] < mValues[1] ? (mValues[1] < mValues[2] ? 2 : 1) : (mValues[0] < mValues[2] ? 2 : 0));
}

// Return true if the vector is unit and false otherwise
inline bool Vector3::isUnit() const {
    return approxEqual(mValues[0] * mValues[0] + mValues[1] * mValues[1] + mValues[2] * mValues[2], 1.0);
}

// Return true if the vector is the zero vector
inline bool Vector3::isZero() const {
    return approxEqual(mValues[0] * mValues[0] + mValues[1] * mValues[1] + mValues[2] * mValues[2], 0.0);
}

// Overloaded operator for the equality condition
inline bool Vector3::operator== (const Vector3& vector) const {
    return (mValues[0] == vector.mValues[0] && mValues[1] == vector.mValues[1] && mValues[2] == vector.mValues[2]);
}

// Overloaded operator for the is different condition
inline bool Vector3::operator!= (const Vector3& vector) const {
    return !(*this == vector);
}

// Overloaded operator for addition with assignment
inline Vector3& Vector3::operator+=(const Vector3& vector) {
    mValues[0] += vector.mValues[0];
    mValues[1] += vector.mValues[1];
    mValues[2] += vector.mValues[2];
    return *this;
}

// Overloaded operator for substraction with assignment
inline Vector3& Vector3::operator-=(const Vector3& vector) {
    mValues[0] -= vector.mValues[0];
    mValues[1] -= vector.mValues[1];
    mValues[2] -= vector.mValues[2];
    return *this;
}

// Overloaded operator for multiplication with a number with assignment
inline Vector3& Vector3::operator*=(decimal number) {
    mValues[0] *= number;
    mValues[1] *= number;
    mValues[2] *= number;
    return *this;
}

// Overloaded operator for division by a number with assignment
inline Vector3& Vector3::operator/=(decimal number) {
    mValues[0] /= number;
    mValues[1] /= number;
    mValues[2] /= number;
    return *this;
}

// Overloaded operator for value access
inline decimal& Vector3::operator[] (int index) {
    return mValues[index];
}

// Overloaded operator for value access
inline const decimal& Vector3::operator[] (int index) const {
    return mValues[index];
}

// Overloaded operator for addition
inline Vector3 operator+(const Vector3& vector1, const Vector3& vector2) {
    return Vector3(vector1.mValues[0] + vector2.mValues[0], vector1.mValues[1] + vector2.mValues[1], vector1.mValues[2] + vector2.mValues[2]);
}

// Overloaded operator for substraction
inline Vector3 operator-(const Vector3& vector1, const Vector3& vector2) {
    return Vector3(vector1.mValues[0] - vector2.mValues[0], vector1.mValues[1] - vector2.mValues[1], vector1.mValues[2] - vector2.mValues[2]);
}

// Overloaded operator for the negative of a vector
inline Vector3 operator-(const Vector3& vector) {
    return Vector3(-vector.mValues[0], -vector.mValues[1], -vector.mValues[2]);
}

// Overloaded operator for multiplication with a number
inline Vector3 operator*(const Vector3& vector, decimal number) {
    return Vector3(number * vector.mValues[0], number * vector.mValues[1], number * vector.mValues[2]);
}

// Overloaded operator for division by a number
inline Vector3 operator/(const Vector3& vector, decimal number) {
    return Vector3(vector.mValues[0] / number, vector.mValues[1] / number, vector.mValues[2] / number);
}

// Overloaded operator for multiplication with a number
inline Vector3 operator*(decimal number, const Vector3& vector) {
    return vector * number;
}

// Assignment operator
inline Vector3& Vector3::operator=(const Vector3& vector) {
    if (&vector != this) {
        mValues[0] = vector.mValues[0];
        mValues[1] = vector.mValues[1];
        mValues[2] = vector.mValues[2];
    }
    return *this;
}

} // End of the ReactPhysics3D namespace

#endif
