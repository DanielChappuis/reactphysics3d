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

// Libraries
#include "Vector.h"

// Constructor of the class Vector
Vector::Vector(int n) throw(std::invalid_argument) {
    // Check the argument
    if (n > 0) {
         // Create the array that contains the values of the vector
         nbComponent = n;
         tab = new double[nbComponent];


        // Fill the array with zero's value
        for(int i=0; i<nbComponent; ++i) {
            tab[i] = 0.0;
        }
    }
    else {
        // Throw an exception because of the wrong argument
        throw std::invalid_argument("Exception : The size of the vector has to be positive !");
    }
}

// Copy-constructor of the class Vector
Vector::Vector(const Vector& vector) {
    nbComponent = vector.nbComponent;
    tab = new double[nbComponent];

    // Fill the array with the value of the vector
    for (int i=0; i<nbComponent; ++i) {
        tab[i] = vector.tab[i];
    }
}

// Destructor of the class Vector
Vector::~Vector() {
    // Erase the array with the values of the vector
    delete [] tab;
}

// Return the corresponding unit vector
Vector Vector::getUnit() const throw(VectorException) {
    double lengthVector = length();

    // Check if the length of the vector is equal to zero
    if (lengthVector!= 0) {
        double lengthInv = 1.0 / lengthVector;
        Vector unitVector(nbComponent);

        // Compute the unit vector
        for(int i=0; i<nbComponent; ++i) {
            unitVector.setValue(i, getValue(i) * lengthInv);
        }

        // Return the unit vector
        return unitVector;

    }
    else {
        // Throw an exception because the length of the vector is zero
        throw VectorException("Exception : Impossible to compute the unit vector because the length of the vector is zero");
    }
}

// Method to compute the scalar product of two vectors
double Vector::scalarProduct(const Vector& vector) const throw(VectorException) {
    // Check the sizes of the two vectors
    if (nbComponent == vector.nbComponent) {
        double result = 0.0;

        // Compute the scalar product
        for (int i=0; i<nbComponent; ++i) {
            result = result + vector.tab[i] * tab[i];
        }

        // Return the result of the scalar product
        return result;
    }
    else {
        // Throw an exception because the two vectors haven't the same size
        throw VectorException("Exception : Impossible to compute the scalar product because the vectors haven't the same size");
    }
}

// Method to compute the cross product of two vectors
Vector Vector::crossProduct(const Vector& vector) const throw(VectorException) {
    // Check if the vectors have 3 components
    if (nbComponent == 3 && vector.nbComponent == 3) {
        Vector result(3);

        // Compute the cross product
        result.tab[0] = tab[1] * vector.tab[2] - tab[2] * vector.tab[1];
        result.tab[1] = tab[2] * vector.tab[0] - tab[0] * vector.tab[2];
        result.tab[2] = tab[0] * vector.tab[1] - tab[1] * vector.tab[0];

        // Return the result of the cross product
        return result;
    }
    else {
        // Throw an exception because the vectors haven't three components
        throw VectorException("Exception : Impossible to compute the cross product because the vectors haven't 3 components");
    }
}

// TO DELETE
void Vector::display() const {
    for (int i=0; i<nbComponent; ++i) {
        std::cout << tab[i] << std::endl;
    }
}

// Overloaded operator for addition
Vector Vector::operator + (const Vector& vector) const throw(VectorException) {
    // Check the size of the two vectors
    if (nbComponent == vector.nbComponent) {
        Vector sum(nbComponent);

        // Compute the sum of the two vectors
        for (int i=0; i<nbComponent; ++i) {
            sum.setValue(i, vector.tab[i] + tab[i]);
        }

        // Return the sum vector
        return sum;
    }
    else {
        // Throw an exception because the sizes of the two vectors aren't the same
        throw VectorException("Exception : Impossible two sum the two vectors because the sizes aren't the same !");
    }
}

// Overloaded operator for substraction
Vector Vector::operator - (const Vector& vector) const throw(VectorException) {
    // Check the size of the two vectors
    if (nbComponent == vector.nbComponent) {
        Vector substraction(nbComponent);

        // Compute the subraction of the two vectors
        for (int i=0; i<nbComponent; ++i) {
            substraction.setValue(i, tab[i] - vector.tab[i]);
        }

        // Return the subraction vector
        return substraction;
    }
    else {
        // Throw an exception because the sizes of the two vectors aren't the same
        throw VectorException("Exception : Impossible two substract the two vectors because the sizes aren't the same !");
    }
}

// Overloaded operator for multiplication with a number
Vector Vector::operator * (double number) const {
    Vector result(nbComponent);

    // Compute the multiplication
    for (int i=0; i<nbComponent; ++i) {
        result.setValue(i, number * tab[i]);
    }

    // Return the result vector
    return result;
}

// Overloaded operator for assigment to a Vector
Vector& Vector::operator = (const Vector& vector) throw(VectorException) {
    // Check the size of the vectors
    if (nbComponent == vector.nbComponent) {
        // Check for self-assignment
        if (this != &vector) {
            for (int i=0; i<nbComponent; ++i) {
                tab[i] = vector.tab[i];
            }
        }

        // Return a reference to the vector
        return *this;
    }
    else {
        // Throw an exception because the sizes of the vectors aren't the same
        throw VectorException("Exception : The assigment to a Vector is impossible because the size of the vectors aren't the same");
    }
}

// Overloaded operator for the equality condition
bool Vector::operator == (const Vector& vector) const throw(VectorException) {
    // Check if the sizes of the vectors are compatible
    if (nbComponent == vector.nbComponent) {
        for (int i=0; i<nbComponent; ++i) {
            if (tab[i] != vector.tab[i]) {
                return false;
            }
        }

        return true;
    }
    else {
        // Throw an exception because the sizes of the vectors aren't the same
        throw VectorException("Exception : Impossible to check if the vectors are equal because they don't have the same size");
    }
}

