/****************************************************************************
 * Copyright (C) 2008      Daniel Chappuis                                  *
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


#ifndef VECTOR_H
#define VECTOR_H

// Libraries
#include "exceptions.h"

#include <cmath>
#include <iostream>

/*  -------------------------------------------------------------------
    Class Vector :
        This class represents a Vector.
    -------------------------------------------------------------------
*/
class Vector {
    private :
        double* tab;                                                                // Array of the vector's components
        int nbComponent;                                                            // number of components in the vector

    public :
        Vector(int n) throw(std::invalid_argument);                                 // Constructor of the class Vector
        Vector(const Vector& vector);                                               // Copy-constructor of the class Vector
        virtual ~Vector();                                                          // Destructor of the class Vector
        double getValue(int n) const throw(std::invalid_argument);                  // Get a component of the vector
        void setValue(int n, double value) throw(std::invalid_argument);            // Set the value of a component of the vector
        int getNbComponent() const;                                                 // Get the number of components in the vector
        double length() const;                                                      // Get the length of the vector
        Vector getUnit() const throw(VectorException);                              // Return the corresponding unit vector
        double scalarProduct(const Vector& vector) const throw(VectorException);    // Scalar product of two vectors
        Vector crossProduct(const Vector& vector) const throw(VectorException);     // Cross product of two vectors (in 3D only)

        void display() const;                                                       // TO DELETE

        // --- Overloaded operators --- //
        Vector operator + (const Vector& vector) const throw(VectorException);      // Overloaded operator for addition
        Vector operator - (const Vector& vector) const throw(VectorException);      // Overloaded operator for substraction
        Vector operator * (double number) const;                                    // Overloaded operator for multiplication with a number
        Vector& operator = (const Vector& vector) throw(VectorException);           // Overloaded operator for the assignement to a Vector
        bool operator == (const Vector& vector) const throw(VectorException);       // Overloaded operator for the equality condition
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

// Overloaded operator for multiplication between a number and a Vector (inline)
inline Vector operator * (double number, const Vector& vector) {
    // Compute and return the result
    return vector * number;
}


#endif
