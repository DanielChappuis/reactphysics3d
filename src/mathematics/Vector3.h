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
#include "mathematics_functions.h"


// ReactPhysics3D namespace
namespace reactphysics3d {

/*  -------------------------------------------------------------------
    Class Vector3 :
        This classrepresents 3 dimensionnal vector in space.
    -------------------------------------------------------------------
*/
class Vector3 {
    private :
        double values[3];                                       // Values of the 3D vector

    public :
        Vector3();                                              // Constructor of the class Vector3D
        Vector3(double x, double y, double z);                  // Constructor with arguments
        Vector3(const Vector3& vector);                         // Copy-constructor
        virtual ~Vector3();                                     // Destructor
        double getX() const;                                    // Get the x component of the vector
        double getY() const;                                    // Get the y component of the vector
        double getZ() const;                                    // Get the z component of the vector
        void setX(double x);                                    // Set the x component of the vector
        void setY(double y);                                    // Set the y component of the vector
        void setZ(double z);                                    // Set the z component of the vector
        void setAllValues(double x, double y, double z);        // Set all the values of the vector
        double length() const;                                  // Return the lenght of the vector
        double lengthSquare() const;                            // Return the square of the length of the vector
        Vector3 getUnit() const;                                // Return the corresponding unit vector
        bool isUnit() const;                                    // Return true if the vector is unit and false otherwise
        bool isZero() const;                                    // Return true if the current vector is the zero vector
        Vector3 getOneOrthogonalVector() const;                 // Return one unit orthogonal vectors of the current vector
        double dot(const Vector3& vector) const;                // Dot product of two vectors
        Vector3 cross(const Vector3& vector) const;             // Cross product of two vectors
        Vector3 getAbsoluteVector() const;                      // Return the corresponding absolute value vector
        int getMinAxis() const;                                 // Return the axis with the minimal value
        int getMaxAxis() const;                                 // Return the axis with the maximal value
        bool isParallelWith(const Vector3& vector) const;       // Return true if two vectors are parallel
        
        // --- Overloaded operators --- //
        bool operator== (const Vector3& vector) const;          // Overloaded operator for the equality condition
        bool operator!= (const Vector3& vector) const;          // Overloaded operator for the is different condition
        Vector3& operator+=(const Vector3& vector);             // Overloaded operator for addition with assignment
        Vector3& operator-=(const Vector3& vector);             // Overloaded operator for substraction with assignment
        Vector3& operator*=(double number);                     // Overloaded operator for multiplication with a number with assignment
        double& operator[] (int index);                         // Overloaded operator for value access
        const double& operator[] (int index) const;             // Overloaded operator for value access
        
        // Friend functions
        friend Vector3 operator+(const Vector3& vector1, const Vector3& vector2);
        friend Vector3 operator-(const Vector3& vector1, const Vector3& vector2);
        friend Vector3 operator-(const Vector3& vector);
        friend Vector3 operator*(const Vector3& vector, double number);
        friend Vector3 operator*(double number, const Vector3& vector);
};

// Get the x component of the vector
inline double Vector3::getX() const {
    return values[0];
}

// Get the y component of the vector
inline double Vector3::getY() const {
    return values[1];
}

// Get the z component of the vector
inline double Vector3::getZ() const {
    return values[2];
}

// Set the x component of the vector
inline void Vector3::setX(double x) {
    this->values[0] = x;
}

// Set the y component of the vector
inline void Vector3::setY(double y) {
    this->values[1] = y;
}

// Set the z component of the vector
inline void Vector3::setZ(double z) {
    this->values[2] = z;
}

// Set all the values of the vector (inline)
inline void Vector3::setAllValues(double x, double y, double z) {
    values[0]= x;
    values[1] = y;
    values[2] = z;
}

// Return the length of the vector (inline)
inline double Vector3::length() const {
    // Compute and return the length of the vector
    return sqrt(values[0]*values[0] + values[1]*values[1] + values[2]*values[2]);
}

// Return the square of the length of the vector
inline double Vector3::lengthSquare() const {
    return values[0]*values[0] + values[1]*values[1] + values[2]*values[2];
}

// Scalar product of two vectors (inline)
inline double Vector3::dot(const Vector3& vector) const {
    // Compute and return the result of the scalar product
    return (values[0] * vector.values[0] + values[1] * vector.values[1] + values[2] * vector.values[2]);
}

// Cross product of two vectors (inline)
inline Vector3 Vector3::cross(const Vector3& vector) const {
    // Compute and return the cross product
    return Vector3(values[1] * vector.values[2] - values[2] * vector.values[1],
                   values[2] * vector.values[0] - values[0] * vector.values[2],
                   values[0] * vector.values[1] - values[1] * vector.values[0]);
}

// Return the corresponding absolute value vector
inline Vector3 Vector3::getAbsoluteVector() const {
    return Vector3(std::abs(values[0]), std::abs(values[1]), std::abs(values[2]));
}       

// Return true if two vectors are parallel
inline bool Vector3::isParallelWith(const Vector3& vector) const {
    double scalarProd = this->dot(vector);
    return approxEqual(std::abs(scalarProd), length() * vector.length());
}    


// Return the axis with the minimal value
inline int Vector3::getMinAxis() const {
    return (values[0] < values[1] ? (values[0] < values[2] ? 0 : 2) : (values[1] < values[2] ? 1 : 2));
}

// Return the axis with the maximal value
inline int Vector3::getMaxAxis() const {
    return (values[0] < values[1] ? (values[1] < values[2] ? 2 : 1) : (values[0] < values[2] ? 2 : 0));
}

// Return true if the vector is unit and false otherwise
inline bool Vector3::isUnit() const {
    return approxEqual(values[0] * values[0] + values[1] * values[1] + values[2] * values[2], 1.0);
}

// Return true if the vector is the zero vector
inline bool Vector3::isZero() const {
    return approxEqual(values[0] * values[0] + values[1] * values[1] + values[2] * values[2], 0.0);
}

// Overloaded operator for the equality condition
inline bool Vector3::operator== (const Vector3& vector) const {
    return (values[0] == vector.values[0] && values[1] == vector.values[1] && values[2] == vector.values[2]);
}

// Overloaded operator for the is different condition
inline bool Vector3::operator!= (const Vector3& vector) const {
    return !(*this == vector);
}

// Overloaded operator for addition with assignment
inline Vector3& Vector3::operator+=(const Vector3& vector) {
    values[0] += vector.values[0];
    values[1] += vector.values[1];
    values[2] += vector.values[2];
    return *this;
}

// Overloaded operator for substraction with assignment
inline Vector3& Vector3::operator-=(const Vector3& vector) {
    values[0] -= vector.values[0];
    values[1] -= vector.values[1];
    values[2] -= vector.values[2];
    return *this;
}

// Overloaded operator for multiplication with a number with assignment
inline Vector3& Vector3::operator*=(double number) {
    values[0] *= number;
    values[1] *= number;
    values[2] *= number;
    return *this;
}

// Overloaded operator for value access
inline double& Vector3::operator[] (int index) {
    return values[index];
}

// Overloaded operator for value access
inline const double& Vector3::operator[] (int index) const {
    return values[index];
}

// Overloaded operator for addition
inline Vector3 operator+(const Vector3& vector1, const Vector3& vector2) {
    return Vector3(vector1.values[0] + vector2.values[0], vector1.values[1] + vector2.values[1], vector1.values[2] + vector2.values[2]);
}

// Overloaded operator for substraction
inline Vector3 operator-(const Vector3& vector1, const Vector3& vector2) {
    return Vector3(vector1.values[0] - vector2.values[0], vector1.values[1] - vector2.values[1], vector1.values[2] - vector2.values[2]);
}

// Overloaded operator for the negative of a vector
inline Vector3 operator-(const Vector3& vector) {
    return Vector3(-vector.values[0], -vector.values[1], -vector.values[2]);
}

// Overloaded operator for multiplication with a number
inline Vector3 operator*(const Vector3& vector, double number) {
    return Vector3(number * vector.values[0], number * vector.values[1], number * vector.values[2]);
}

// Overloaded operator for multiplication with a number
inline Vector3 operator*(double number, const Vector3& vector) {
    return vector * number;
}

} // End of the ReactPhysics3D namespace

#endif
