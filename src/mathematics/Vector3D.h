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

#ifndef VECTOR3D_H
#define VECTOR3D_H

// Libraries
#include <cmath>
#include "exceptions.h"
#include "mathematics_functions.h"

// TODO : Remove the methods getX(), getY(), getZ(), setX(), setY(), setZ() and replace the attributes
//        x, y and z by an array values[3]


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
        double getValue(int index) const throw(std::invalid_argument);      // Get a component of the vector
        void setValue(int index, double value) throw(std::invalid_argument); // Set a component of the vector
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
        Vector3D getOneOrthogonalVector() const;                // Return one unit orthogonal vectors of the current vector
        double dot(const Vector3D& vector) const;               // Dot product of two vectors
        Vector3D cross(const Vector3D& vector) const;           // Cross product of two vectors
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

// Get a component of the vector
inline double Vector3D::getValue(int index) const throw(std::invalid_argument) {
    switch(index) {
        case 0:     return x;
        case 1:     return y;
        case 2:     return z;
        default:    // Throw an exception because of the wrong argument
                    throw std::invalid_argument("The argument is outside the bounds of the Vector3D");
    }
}

// Set a component of the vector
inline void Vector3D::setValue(int index, double value) throw(std::invalid_argument) {
    switch(index) {
        case 0:     x = value;
        case 1:     y = value;
        case 2:     z = value;
        default:    // Throw an exception because of the wrong argument
                    throw std::invalid_argument("The argument is outside the bounds of the Vector3D");
    }
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
inline Vector3D Vector3D::getOpposite() const {
    return (Vector3D(0.0, 0.0, 0.0) - *this);
}

// Scalar product of two vectors (inline)
inline double Vector3D::dot(const Vector3D& vector) const {
    // Compute and return the result of the scalar product
    return (x * vector.x + y * vector.y + z * vector.z);
}

// Cross product of two vectors (inline)
inline Vector3D Vector3D::cross(const Vector3D& vector) const {
    // Compute and return the cross product
    return Vector3D(y * vector.z - z * vector.y, z * vector.x - x * vector.z , x * vector.y - y * vector.x);
}

// Return true if two vectors are parallel
inline bool Vector3D::isParallelWith(const Vector3D& vector) const {
    double scalarProd = this->dot(vector);
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
