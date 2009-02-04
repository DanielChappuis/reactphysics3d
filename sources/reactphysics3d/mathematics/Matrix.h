/****************************************************************************
 * Copyright (C) 2009      Daniel Chappuis                                  *
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

#ifndef MATRIX_H
#define MATRIX_H

// Libraries
#include "exceptions.h"
#include <stdexcept>
#include <iostream>

// ReactPhysics3D namespace
namespace reactphysics3d {

/*  -------------------------------------------------------------------
    Class Matrix :
        This class represents a matrix.
    -------------------------------------------------------------------
*/
class Matrix {
    private :
        int nbRow;                                                                      // Number of row in the matrix
        int nbColumn;                                                                   // Number of colum in the matrix
        double** array;                                                                 // Dynamic array that contains the values of the matrix

    public :
        Matrix(int nbRow, int nbColum) throw(std::invalid_argument);                    // Constructor of the class Matrix
        Matrix(const Matrix& matrix);                                                   // Copy constructor of the class Matrix
        virtual ~Matrix();                                                              // Destructor of the class Matrix
        double getValue(int i, int j) const throw(std::invalid_argument);               // Return a value in the matrix
        void setValue(int i, int j, double value) throw(std::invalid_argument);         // Set a value in the matrix
        int getNbRow() const;                                                           // Return the number of row of the matrix
        int getNbColumn() const;                                                        // Return the number of column of the matrix
        Matrix getCofactor(int i, int j) const throw(std::invalid_argument);            // Return the cofactor matrix by removing row i and column j
        Matrix getTranspose() const;                                                    // Return the transposed matrixs
        Matrix getInverse() const throw(MathematicsException);                          // Return the inverse of the matrix if there exists
        double getDeterminant() const throw(MathematicsException);                      // Return the determinant of the matrix
        double getTrace() const throw(MathematicsException);                            // Return the trace of a square matrix
        static Matrix identity(int dimension) throw(std::invalid_argument);             // Return the identity matrix I of the given dimension

        void display() const;                                                           // TO DELETE

        // --- Overloaded operators --- //
        Matrix operator+(const Matrix& matrix2) const throw(MathematicsException);      // Overloaded operator for addition
        Matrix operator-(const Matrix& matrix2) const throw(MathematicsException);      // Overloaded operator for substraction
        Matrix operator*(double nb) const;                                              // Overloaded operator for multiplication with a number
        Matrix operator*(const Matrix& matrix2) const throw(MathematicsException);      // Overloaded operator for multiplication with a matrix
        Matrix& operator=(const Matrix& matrix2) throw(MathematicsException);           // Overloaded operator for assignment
        bool operator==(const Matrix& matrix2) const throw(MathematicsException);       // Overloaded operator for equality condition
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
