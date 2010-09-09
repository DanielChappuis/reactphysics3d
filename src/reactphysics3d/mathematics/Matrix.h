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

#ifndef MATRIX_H
#define MATRIX_H

// Libraries
#include "exceptions.h"
#include "Matrix3x3.h"
#include "Vector.h"
#include <stdexcept>
#include <iostream>

// TODO : Replace the "int" by "unsigned int"

// ReactPhysics3D namespace
namespace reactphysics3d {

/*  -------------------------------------------------------------------
    Class Matrix :
        This class represents a matrix.
    -------------------------------------------------------------------
*/
class Matrix {
    private :
        unsigned int nbRow;                                                             // Number of row in the matrix
        unsigned int nbColumn;                                                                                          // Number of colum in the matrix
        double** array;                                                                                                 // Dynamic array that contains the values of the matrix

    public :
        Matrix();                                                                                                       // Constructor without argument
        Matrix(int nbRow, int nbColum) throw(std::invalid_argument);                                                    // Constructor of the class Matrix
        Matrix(const Matrix& matrix);                                                                                   // Copy constructor of the class Matrix
        Matrix(const Matrix3x3& matrix);                                                                                // Conversion from Matrix3x3
        Matrix(const Vector& vector);                                                                                   // Conversion from Vector to Matrix
        virtual ~Matrix();                                                                                              // Destructor of the class Matrix
        double getValue(int i, int j) const throw(std::invalid_argument);                                               // Return a value in the matrix
        void setValue(int i, int j, double value) throw(std::invalid_argument);                                         // Set a value in the matrix
        int getNbRow() const;                                                                                           // Return the number of row of the matrix
        int getNbColumn() const;                                                                                        // Return the number of column of the matrix
        Matrix getCofactor(int i, int j) const throw(std::invalid_argument);                                            // Return the cofactor matrix by removing row i and column j
        Matrix getTranspose() const;                                                                                    // Return the transposed matrixs
        Matrix getInverse() const throw(MathematicsException);                                                          // Return the inverse of the matrix if there exists
        double getDeterminant() const throw(MathematicsException);                                                      // Return the determinant of the matrix
        double getTrace() const throw(MathematicsException);                                                            // Return the trace of a square matrix
        Matrix getSubMatrix(unsigned int i, unsigned int j,
                            unsigned int nbRows, unsigned int nbColumns) const throw(std::invalid_argument);            // Return a sub matrix of size of the current matrix
        Vector getVector() const;
        static Matrix identity(int dimension) throw(std::invalid_argument);                                             // Return the identity matrix I of the given dimension
        void fillInSubMatrix(unsigned int i, unsigned int j, const Matrix& subMatrix);                                  // Fill in a sub-matrix of the current matrix with another matrix
        void initWithValue(double value);                                                                               // Initialize all the matrix with the given value
        void display() const;                                                                                           // TO DELETE
        void changeSize(uint nbRows, uint nbColumns);

        // --- Overloaded operators --- //
        Matrix operator+(const Matrix& matrix2) const throw(MathematicsException);                                      // Overloaded operator for addition
        Matrix operator-(const Matrix& matrix2) const throw(MathematicsException);                                      // Overloaded operator for substraction
        Matrix operator*(double nb) const;                                                                              // Overloaded operator for multiplication with a number
        Matrix operator*(const Matrix& matrix2) const throw(MathematicsException);                                      // Overloaded operator for multiplication with a matrix
        Matrix operator*(const Vector& vector) const throw(MathematicsException);                                       // Overloaded operator for multiplication with a vector
        Matrix& operator=(const Matrix& matrix2) throw(MathematicsException);                                           // Overloaded operator for assignment
        bool operator==(const Matrix& matrix2) const throw(MathematicsException);                                       // Overloaded operator for equality condition
};

// Function to get a value in the matrix (inline)
inline double Matrix::getValue(int i, int j) const throw(std::invalid_argument) {
    if (0 <= i && i < nbRow && 0 <= j && j < nbColumn) {
        // get the value in the matrix
        return array[i][j];
    }
    else {
        // We Throw an out_of_range exception
        throw std::invalid_argument("Exception : The index i or j is outside the matrix size !");
    }
}

// Function to set a value in the matrix (inline)
inline void Matrix::setValue(int i, int j, double value) throw(std::invalid_argument) {
    if (0 <= i && i < nbRow && 0 <= j && j < nbColumn) {
        // Set the value in the matrix
        this->array[i][j] = value;
    }
    else {
        // We Throw an out_of_range exception
        throw std::invalid_argument("Exception : The index i or j is outside the matrix size !");
    }
}

// Function that return the number of row of the matrix (inline)
inline int Matrix::getNbRow() const {
    return nbRow;
}

// Function that return the number of colum of the matrix (inline)
inline int Matrix::getNbColumn() const {
    return nbColumn;
}

// Overloaded operator for multiplication between a number and a Matrix (inline)
inline Matrix operator*(double number, const Matrix& matrix) {

    // Return the result matrix
    return matrix * number;
}

} // End of the ReactPhysics3D namespace

#endif
