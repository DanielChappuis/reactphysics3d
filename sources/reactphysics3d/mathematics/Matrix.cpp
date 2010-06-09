/****************************************************************************
 * Copyright (C) 2009      Daniel Chappuis                                  *
 ****************************************************************************
 * This file is part of ReactPhysics3D.                                     *
 *                                                                          *
 * ReactPhysics3D is free software: you can redistribute it and/or modify   *
 * it under the terms of the GNU Lesser General Public License as published *
 * by the Free Software Foundation, either version 3 of the License, or     *
 * (at your option) any later version.                                      *
 *                                                                          *
 * ReactPhysics3D is distributed in the hope that it will be useful,        *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of           *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the             *
 * GNU Lesser General Public License for more details.                      *
 *                                                                          *
 * You should have received a copy of the GNU Lesser General Public License *
 * along with ReactPhysics3D. If not, see <http://www.gnu.org/licenses/>.   *
 ***************************************************************************/

// Libraries
#include <cassert>
#include "Matrix.h"

// Namespaces
using namespace reactphysics3d;

// Constructor without argument
Matrix::Matrix()
       :nbRow(0), nbColumn(0) {
    array = 0;
}

// Constructor of the class Matrix
Matrix::Matrix(int nbRow, int nbColumn) throw(std::invalid_argument)
       :nbRow(nbRow),nbColumn(nbColumn) {
    // Check the arguments
    if (nbRow>0 && nbColumn>0) {

        // Create the two dimensional dynamic array
        array = new double*[nbRow];

        assert(array != 0);     // Array pointer musn't be null

        for(int i=0; i<nbRow; ++i) {
            array[i] = new double[nbColumn];
        }

        // TODO : This initialization loop must be removed carefully
        // Fill the matrix with zero's
        for (int i=0; i<nbRow; ++i) {
            for(int j=0; j<nbColumn; ++j) {
                setValue(i,j, 0.0);
            }
        }
    }
    else {
        // Throw an exception
        throw std::invalid_argument("Exception : The size of the matrix has to be positive !");
    }
}

// Copy-constructor of the class Matrix
Matrix::Matrix(const Matrix& matrix)
       :nbRow(matrix.nbRow), nbColumn(matrix.nbColumn) {

    // Create the two dimensional dynamic array
    array = new double*[nbRow];

    assert(array != 0);     // Array pointer musn't be null

    for(int i=0; i<nbRow; ++i) {
        array[i] = new double[nbColumn];
    }

    // Copy the matrix
    for (int i=0; i<nbRow; ++i) {
        for(int j=0; j<nbColumn; ++j) {
            setValue(i,j, matrix.getValue(i,j));
        }
    }
}

// Conversion from Matrix3x3
Matrix::Matrix(const Matrix3x3& matrix)
    :nbRow(3), nbColumn(3) {

    // Create the two dimensional dynamic array
    array = new double*[nbRow];

    assert(array != 0);     // Array pointer musn't be null

    for(int i=0; i<nbRow; ++i) {
        array[i] = new double[nbColumn];
    }

    // Copy the matrix
    for (int i=0; i<nbRow; ++i) {
        for(int j=0; j<nbColumn; ++j) {
            setValue(i,j, matrix.getValue(i,j));
        }
    }
}


// Conversion from Vector to Matrix
Matrix::Matrix(const Vector& vector) {
        // Create the two dimensional dynamic array
        array = new double*[vector.getNbComponent()];
        assert(array != 0);

        for(int i=0; i<nbRow; ++i) {
            array[i] = new double[1];
            array[i][0] = vector.getValue(i);
        }
}

// Destructor of the class Matrix
Matrix::~Matrix() {
    // Destruction of the dynamic array
    for(int i=0; i<nbRow; ++i) {
        delete array[i];
    }
    delete this->array;
}

void Matrix::changeSize(uint newNbRows, uint newNbColumns) {
    if (array) {
        // Destruction of the dynamic array
        for(int i=0; i<nbRow; ++i) {
            delete array[i];
        }
        delete this->array;
    }

    // Create the two dimensional dynamic array
    array = new double*[newNbRows];

    assert(array != 0);     // Array pointer musn't be null

    for(int i=0; i<newNbRows; ++i) {
        array[i] = new double[newNbColumns];
    }

    nbColumn = newNbColumns;
    nbRow = newNbRows;
}


// Function that return the cofactor matrix by removing row i and column j
Matrix Matrix::getCofactor(int i, int j) const throw(std::invalid_argument) {
    // If i and j are in the matrix
    if (0<= i && i < nbRow && 0<= j && j<nbColumn) {
        // Create the cofactor matrix
        Matrix cofactor(nbRow-1,nbColumn-1);

        int u=0;                // Row coordinate in the cofactor matrix
        int v=0;                // Column coordinate in the cofactor matrix

        // For every element in the matrix
        for (int s=0; s<nbColumn; ++s) {
            for(int r=0; r<nbRow; ++r) {
                // If the element is not in row i or in column j
                if (r!=i && s!=j) {
                    // Add the element in the cofactor matrix
                    cofactor.setValue(u,v, getValue(r,s));
                    ++u;
                    if (u==cofactor.nbRow) {
                        u = 0;
                        ++v;
                    }
                }
            }
        }

        // Return the cofactor matrix
        return cofactor;
    }
    else {
        // We Throw an out_of_range exception
        throw std::invalid_argument("Exception : The index i or j is outside the matrix size !");
    }
}

// This function return the transposed matrix
Matrix Matrix::getTranspose() const {
    // Create the new matrix
    Matrix transposedMatrix(nbColumn, nbRow);

    // Transposition of the matrix
    for (int i=0; i<nbRow; ++i) {
        for(int j=0; j<nbColumn; ++j) {
            transposedMatrix.setValue(j,i, array[i][j]);
        }
    }

    // Return the transposed matrix
    return transposedMatrix;
}


// Function that return the inverse of the matrix if there exists
Matrix Matrix::getInverse() const throw(MathematicsException) {
    // Check if the matrix is a square-matrix
    if (nbRow==nbColumn) {
        // Compute the determinant of the matrix
        double determinant = getDeterminant();

        // Check if the matrix is invertible
        if (determinant != 0.0) {
            // Create a temp matrix
            Matrix tempMatrix(nbRow, nbColumn);

            double k=1.0;

            // Compute the inverse matrix
            for(int i=0; i<nbRow; ++i) {
                for(int j=0; j<nbColumn; ++j) {
                    if ( (i+j) % 2 == 0) {
                        k=1.0;
                    }
                    else {
                        k=-1.0;
                    }

                    tempMatrix.setValue(i,j, k * getCofactor(i,j).getDeterminant());
                }
            }

            // Create the inverse matrix
            Matrix inverseMatrix = tempMatrix.getTranspose() *  (1.0 / determinant);

            // Return the inverse matrix
            return inverseMatrix;
        }
        else {
            // We throw a MathematicsException
            throw MathematicsException("MathematicsException : Inverse of the matrix can't be computed because the determinant is zero !");
        }
    }
    else {
           // We throw an Matrix Exception
            throw MathematicsException("MathematicsException : Inverse can't be computed for a non-square matrix !");
    }
}

// Function that return the determinant of the matrix
double Matrix::getDeterminant() const throw(MathematicsException) {
    // If the matrix is a square matrix
    if (nbRow == nbColumn) {
        if(nbRow == 1) {
            return getValue(0,0);
        }
        else if (nbRow == 2) {
            return (getValue(0,0) * getValue(1,1) - getValue(1,0) * getValue(0,1));
        }
        else {
            double determinant = 0.0;
            double k=1.0;

            // For every element in the first row
            for(int j=0; j<nbColumn; ++j) {
                determinant = determinant + k * getValue(0,j) * getCofactor(0,j).getDeterminant();

                if (k==1.0) {
                    k=-1.0;
                }
                else {
                    k=1.0;
                }
            }

            // Return the determinant value
            return determinant;
        }
    }
    else {
        // Throw a MathematicsException
        throw MathematicsException("MathematicsException : The determinant of a non-square matrix isn't computable !");
    }
}

// Return the trace of the matrix
double Matrix::getTrace() const throw(MathematicsException) {

    // Check if the matrix is a square-matrix
    if (nbRow == nbColumn) {
        double sum = 0.0;

        // Compute the trace of the matrix
        for(int i=0; i<nbRow; ++i) {
            sum = sum + array[i][i];
        }

        // Return the trace
        return sum;
    }
    else {
        // We throw an exception because the matrix is non-square
        throw MathematicsException("MathematicsException : Impossible to compute the trace for a non-square matrix");
    }
}

// Return a sub matrix of size of the current matrix
Matrix Matrix::getSubMatrix(unsigned int i, unsigned int j,
                                   unsigned int sizeRows, unsigned int sizeColumns) const throw(std::invalid_argument) {
    // Check the arguments
    if (i<0 || j<0 || i+sizeRows > nbRow || j+sizeColumns > nbColumn) {
        // Throw an exception
        throw std::invalid_argument("Error : The arguments are out of matrix bounds");
    }

    // Compute the sub-matrix
    Matrix resultMatrix(sizeRows, sizeColumns);
    for (unsigned int k=0; k<sizeRows; k++) {
        for (unsigned int l=0; l<sizeColumns; l++) {
            resultMatrix.array[k][l] = array[i+k][j+l];
        }
    }

    // Return the sub-matrix
    return resultMatrix;
}

// Static function that return a identity matrix of size nxn
Matrix Matrix::identity(int dimension) throw(std::invalid_argument) {
    // Argument verification
    if (dimension > 0) {
        // Create a new matrix
        Matrix identityMatrix(dimension,dimension);

        // Fill in the identity matrix
        for(int i=0; i<dimension; ++i) {
            for(int j=0; j<dimension; ++j) {
                if (i==j) {
                    identityMatrix.setValue(i, j, 1.0);
                }
                else {
                    identityMatrix.setValue(i, j, 0.0);
                }
            }
        }

        // Return the identity matrix
        return identityMatrix;
    }
    else {
        // Throw an exception
        throw std::invalid_argument("Exception : The argument of identity() has to be positive !");
    }
}

// TODO : Test this method
// Fill in a sub-matrix of the current matrix with another matrix.
// This method replaces the sub-matrix with the upper-left index (i,j) in the current matrix by another
// "subMatrix" matrix.
void Matrix::fillInSubMatrix(unsigned int rowIndex, unsigned int colIndex, const Matrix& subMatrix) {
    assert(nbRow-rowIndex >= subMatrix.nbColumn);
    assert(nbColumn-colIndex >= subMatrix.nbColumn);

    // Fill in the sub-matrix
    for (unsigned int i=0; i<subMatrix.nbRow; ++i) {
        for (unsigned int j=0; j<subMatrix.nbColumn; ++j) {
            array[rowIndex + i][colIndex + j] = subMatrix.array[i][j];
        }
    }
}

// Initialize all the matrix with the given value
void Matrix::initWithValue(double value) {
    for (unsigned int i=0; i<nbRow; ++i) {
        for(unsigned int j=0; j<nbColumn; ++j) {
            array[i][j] = value;
        }
    }
}


 Vector Matrix::getVector() const {
     assert(nbColumn == 1);
     Vector vec(nbRow);
     for (int i=0; i<nbRow; i++) {
         vec.setValue(i, array[i][0]);
     }
     return vec;
 }

// Definition of the operator + for the sum of two matrices with references
Matrix Matrix::operator+(const Matrix& matrix2) const throw(MathematicsException) {
    if (nbRow == matrix2.nbRow && nbColumn == matrix2.nbColumn) {
        // Create a new matrix
        Matrix sumMatrix(nbRow,nbColumn);

        // Sum the two matrices
        for(int i=0; i<nbRow; ++i) {
            for(int j=0; j<nbColumn; ++j) {
                sumMatrix.setValue(i, j, this->getValue(i,j) + matrix2.getValue(i,j));
            }
        }

        // Return the sum matrix
        return sumMatrix;
    }
    else {
        // We throw an MathematicsException
        throw MathematicsException("MathematicsException : Addition of the matrices isn't possible beacause the size of the matrices aren't the same");
    }
}

// Definition of the operator - for the substraction of two matrices with references
Matrix Matrix::operator-(const Matrix& matrix2) const throw(MathematicsException) {
    if (nbRow == matrix2.nbRow && nbColumn == matrix2.nbColumn) {
        // Create a new matrix
        Matrix sumMatrix(nbRow, nbColumn);

        // Substract the two matrices
        for(int i=0; i<nbRow; ++i) {
            for(int j=0; j<this->nbColumn; ++j) {
                sumMatrix.setValue(i, j, this->getValue(i,j) - matrix2.getValue(i,j));
            }
        }

        // Return the sum matrix
        return sumMatrix;
    }
    else {
        // We throw a MathematicsException
        throw MathematicsException("MathematicsException : Substraction of the matrices isn't possible beacause the size of the matrices aren't the same");
    }
}

// Overloaded operator * for the multiplication of the matrix with a number
Matrix Matrix::operator*(double nb) const {
    // Creation of the result matrix
    Matrix result(nbRow,nbColumn);

    // Multiplication of the matrix with the number
    for(int i=0; i<nbRow; ++i) {
        for(int j=0; j<nbColumn; ++j) {
            result.setValue(i,j, getValue(i,j) * nb);
        }
    }

    // Return the result matrix
    return result;
}

// Overloaded operator for multiplication with a matrix
Matrix Matrix::operator*(const Matrix& matrix2) const throw(MathematicsException) {
    // Check the sizes of the matrices
    if (nbColumn == matrix2.nbRow) {
        // Compute the result of the multiplication
        Matrix result(nbRow, matrix2.nbColumn);
        double sum;

        for(int i=0; i<nbRow; ++i) {
            for(int j=0; j<matrix2.nbColumn; ++j) {
                sum = 0.0;
                for(int k=0; k<nbColumn; ++k) {
                    sum = sum + array[i][k] * matrix2.array[k][j];
                }
                result.array[i][j] = sum;
            }
        }

        // Return the result matrix
        return result;
    }
    else {
        // Throw an exception because the multiplication is impossible
        throw MathematicsException("MathematicsException : The sizes of the matrices aren't compatible for the multiplication");
    }
}

// Overloaded operator for multiplication with a vector
Matrix Matrix::operator*(const Vector& vector) const throw(MathematicsException) {
    // Check the sizes of the matrices
    if (nbColumn == vector.getNbComponent()) {
        Matrix result(nbRow, 1);
        for (int i=0; i<nbRow; i++) {
            double sum = 0.0;
            for (int j=0; j<nbColumn; j++) {
                sum += array[i][j] * vector.getValue(j);
            }
            result.array[i][0] = sum;
        }

        return result;
    }
    else {
        // Throw an exception because the multiplication is impossible
        throw MathematicsException("MathematicsException : The sizes of the matrix and the vector aren't compatible for the multiplication");
    }
}

// Overloaded operator = for the assignment
Matrix& Matrix::operator=(const Matrix& matrix2) throw(MathematicsException) {

    // Check for self-assignement
    if(this == &matrix2) {
        return *this;
    }

    // Check the size of the matrix
    if (nbRow==matrix2.nbRow && nbColumn==matrix2.nbColumn) {
        // Check for self-assignment
        if (this != &matrix2) {
            for(int i=0; i<nbRow; ++i) {
                for(int j=0; j<nbColumn; ++j) {
                    this->setValue(i,j, matrix2.getValue(i,j));
                }
            }
        }

        // Return a reference to the matrix
        return *this;
    }
    else {
        // Throw a MathematicsException
        throw MathematicsException("MathematicsException : Assignment impossible because the size of the matrices aren't the same !");
    }
}

// Overloaded operator for equality condition
bool Matrix::operator==(const Matrix& matrix2) const throw(MathematicsException) {
    // Check if the matrices dimensions are compatible
    if (nbRow == matrix2.nbRow && nbColumn == matrix2.nbColumn) {
        for (int i=0; i<nbRow; ++i) {
            for(int j=0; j<nbColumn; ++j) {
                if (array[i][j] != matrix2.array[i][j]) {
                    return false;
                }
            }
        }

        return true;
    }
    else {
        // Throw an exception because the matrices dimensions aren't the same
        throw MathematicsException("MathematicsException : Impossible to check if the matrices are equal because they don't have the same dimension");
    }
}

// TO DELETE, THIS IS JUST FOR TESTING MATRICES
void Matrix::display() const {
    for(int i=0; i<nbRow; ++i) {
        for(int j=0; j<nbColumn; ++j) {
            std::cout << array[i][j] << "  ";
        }

        std::cout << std::endl;
    }
}
