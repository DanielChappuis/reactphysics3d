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

// Libraries
#include "Vector.h"

// Namespaces
using namespace reactphysics3d;

// Constructor without argument
Vector::Vector()
       :nbComponent(0) {
    tab = 0;
}

// Constructor of the class Vector
Vector::Vector(int n) throw(std::invalid_argument) {
    // Check the argument
    if (n > 0) {
         // Create the array that contains the values of the vector
         nbComponent = n;
         tab = new double[nbComponent];

         // TODO : Remove this initialization
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

// Conversion from Vector3D to Vector
Vector::Vector(const Vector3D& vector3d) {
    nbComponent = 3;
    tab = new double[3];

    tab[0] = vector3d.getX();
    tab[1] = vector3d.getY();
    tab[2] = vector3d.getZ();
}

// Destructor of the class Vector
Vector::~Vector() {
    // Erase the array with the values of the vector
    delete [] tab;
}

// Return the corresponding unit vector
Vector Vector::getUnit() const throw(MathematicsException) {
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
        throw MathematicsException("MathematicsException : Impossible to compute the unit vector because the length of the vector is zero");
    }
}

void Vector::setVector(const Vector& vector) {
    assert(nbComponent == vector.nbComponent);

    for (int i=0; i<nbComponent; i++) {
        tab[i] = vector.tab[i];
    }
}

// Method to compute the scalar product of two vectors
double Vector::scalarProduct(const Vector& vector) const throw(MathematicsException) {
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
        throw MathematicsException("MathematicsException : Impossible to compute the scalar product because the vectors haven't the same size");
    }
}

// Method to compute the cross product of two vectors
Vector Vector::crossProduct(const Vector& vector) const throw(MathematicsException) {
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
        throw MathematicsException("MathematicsException : Impossible to compute the cross product because the vectors haven't 3 components");
    }
}

void Vector::changeSize(uint newSize) {
    if (tab) {
         delete[] tab;
    }

    nbComponent = newSize;
    tab = new double[nbComponent];

    // Fill the array with the value of the vector
    for (int i=0; i<nbComponent; ++i) {
	tab[i] = 0.0;
    }
}

// Overloaded operator for addition
Vector Vector::operator+(const Vector& vector) const throw(MathematicsException) {
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
        throw MathematicsException("MathematicsException : Impossible two sum the two vectors because the sizes aren't the same !");
    }
}

// Overloaded operator for substraction
Vector Vector::operator-(const Vector& vector) const throw(MathematicsException) {
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
        throw MathematicsException("MathematicsException : Impossible two substract the two vectors because the sizes aren't the same !");
    }
}

// Overloaded operator for multiplication with a number
Vector Vector::operator*(double number) const {
    Vector result(nbComponent);

    // Compute the multiplication
    for (int i=0; i<nbComponent; ++i) {
        result.setValue(i, number * tab[i]);
    }

    // Return the result vector
    return result;
}

// Overloaded operator for assigment to a Vector
Vector& Vector::operator=(const Vector& vector) throw(MathematicsException) {

    // Check for self-assignment
    if (this == &vector) {
        return *this;
    }

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
        throw MathematicsException("MathematicsException : The assigment to a Vector is impossible because the size of the vectors aren't the same");
    }
}

// Overloaded operator for the equality condition
bool Vector::operator==(const Vector& vector) const throw(MathematicsException) {
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
        throw MathematicsException("MathematicsException : Impossible to check if the vectors are equal because they don't have the same size");
    }
}

