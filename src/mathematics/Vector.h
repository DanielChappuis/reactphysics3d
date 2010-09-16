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

#ifndef VECTOR_H
#define VECTOR_H

// Libraries
#include "Vector3D.h"
#include "../constants.h"
#include "mathematics_functions.h"
#include "exceptions.h"
#include <cmath>
#include <cassert>
#include <iostream>


// ReactPhysics3D namespace
namespace reactphysics3d {

/*  -------------------------------------------------------------------
    Class Vector :
        This class represents a Vector.
    -------------------------------------------------------------------
*/
class Vector {
    private :
        double* tab;                    // Array of the vector's components
        uint nbComponent;               // number of components in the vector

    public :
        Vector();                                                                                   // Constructor without argument
        Vector(int n) throw(std::invalid_argument);                                                 // Constructor of the class Vector
        Vector(const Vector& vector);                                                               // Copy-constructor of the class Vector 
        Vector(const Vector3D& vector3d);                                                           // Conversion from Vector3D to Vector
        virtual ~Vector();                                                                          // Destructor of the class Vector
        double getValue(int n) const throw(std::invalid_argument);                                  // Get a component of the vector
        void setValue(int n, double value) throw(std::invalid_argument);                            // Set the value of a component of the vector
        int getNbComponent() const;                                                                 // Get the number of components in the vector
        void initWithValue(double value);                                                           // Init all the elements with a given value
        double length() const;                                                                      // Get the length of the vector
        Vector getUnit() const throw(MathematicsException);                                         // Return the corresponding unit vector
        double scalarProduct(const Vector& vector) const throw(MathematicsException);               // Scalar product of two vectors
        Vector crossProduct(const Vector& vector) const throw(MathematicsException);                // Cross product of two vectors (in 3D only)
        void fillInSubVector(uint index, const Vector& subVector);                                  // Replace a part of the current vector with another sub-vector
        Vector getSubVector(uint index, uint nbElements) const throw(std::invalid_argument);        // Return a sub-vector of the current vector
        void setVector(const Vector& vector);
        bool isUnit() const;                                                                        // Return true if the vector is unit and false otherwise
        void changeSize(uint newSize);

        // --- Overloaded operators --- //
        Vector operator+(const Vector& vector) const throw(MathematicsException);           // Overloaded operator for addition
        Vector operator-(const Vector& vector) const throw(MathematicsException);           // Overloaded operator for substraction
        Vector operator*(double number) const;                                              // Overloaded operator for multiplication with a number
        Vector& operator=(const Vector& vector) throw(MathematicsException);                // Overloaded operator for the assignement to a Vector
        bool operator==(const Vector& vector) const throw(MathematicsException);            // Overloaded operator for the equality condition
};


// ------ Definition of inlines functions ------ //

// Method to get the value of a component of the vector (inline)
inline double Vector::getValue(int n) const throw(std::invalid_argument) {
    // Check the argument
    if (n>=0 && n<nbComponent) {
        // Return the value of the component
        return tab[n];
    }
    else {
        // Throw an exception because of the wrong argument
        throw std::invalid_argument("The argument is outside the bounds of the Vector");
    }
}

// Method to set the value of a component of the vector
inline void Vector::setValue(int n, double value) throw(std::invalid_argument) {
    // Check the argument
    if (n >= 0 && n<nbComponent) {
        // Set the value
        tab[n] = value;
    }
    else {
        // Throw an exception because of the wrong argument
        throw std::invalid_argument("Exception : The argument is outside the bounds of the Vector");
    }
}

// Method to get the number of components in the vector  (inline)
inline int Vector::getNbComponent() const {
    // Return the number of components in the vector
    return nbComponent;
}

// Method to get the length of the vector
inline double Vector::length() const {
    // Compute the length of the vector
    double sum = 0.0;
    for(int i=0; i<nbComponent; ++i) {
        sum = sum + tab[i] * tab[i];
    }

    // Return the length of the vector
    return sqrt(sum);
}

// Init all the elements with a given value
inline void Vector::initWithValue(double value) {
    for (uint i=0; i<nbComponent; i++) {
        tab[i] = value;
    }
}

// Replace a part of the current vector with another sub-vector.
// The argument "rowIndex" is the row index where the subVector starts.
inline void Vector::fillInSubVector(uint rowIndex, const Vector& subVector) {
    assert(nbComponent-rowIndex >= subVector.nbComponent);

    // For each value of the sub-vector
    for (uint i=0; i<subVector.nbComponent; ++i) {
        tab[rowIndex + i] = subVector.getValue(i);
    }
}

// Return a sub-vector of the current vector
inline Vector Vector::getSubVector(uint index, uint nbElements) const throw(std::invalid_argument) {
    // Check if the arguments are valid
    if (index < 0 || index+nbElements > nbComponent) {
        throw std::invalid_argument("Error : arguments are out of bounds");
    }

    // Compute the sub-vector
    Vector subVector(nbElements);
    for (uint i=0, j=index; i<nbElements; i++, j++) {
        subVector.tab[i] = tab[j];
    }

    // Return the sub-vector
    return subVector;
}

// Return true if the vector is unit and false otherwise
inline bool Vector::isUnit() const {
    return approxEqual(1.0, length());
}

// Overloaded operator for multiplication between a number and a Vector (inline)
inline Vector operator*(double number, const Vector& vector) {
    // Compute and return the result
    return vector * number;
}

} // End of the ReactPhysics3D namespace

#endif
