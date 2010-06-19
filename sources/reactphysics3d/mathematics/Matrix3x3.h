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


#ifndef MATRIX3X3_H
#define MATRIX3X3_H

// Libraries
#include "exceptions.h"
#include "Vector3D.h"

// ReactPhysics3D namespace
namespace reactphysics3d {


/*  -------------------------------------------------------------------
    Class Matrix3x3 :
        This class represents a 3x3 matrix.
    -------------------------------------------------------------------
*/
class Matrix3x3 {
    private :
        double array[3][3];                                                                     // Array with the values of the matrix

    public :
        Matrix3x3();                                                                            // Constructor of the class Matrix3x3
        Matrix3x3(double a1, double a2, double a3, double b1, double b2, double b3,
                  double c1, double c2, double c3);                                             // Constructor with arguments
        Matrix3x3(const Matrix3x3& matrix);                                                     // Copy-constructor
        virtual ~Matrix3x3();                                                                   // Destructor

        double getValue(int i, int j) const throw(std::invalid_argument);                       // Get a value in the matrix
        void setValue(int i, int j, double value) throw(std::invalid_argument);                 // Set a value in the matrix
        void setAllValues(double a1, double a2, double a3, double b1, double b2, double b3,
                  double c1, double c2, double c3);                                             // Set all the values in the matrix
        Matrix3x3 getTranspose() const;                                                         // Return the transpose matrix
        double getDeterminant() const;                                                          // Return the determinant of the matrix
        double getTrace() const;                                                                // Return the trace of the matrix
        Matrix3x3 getInverse() const throw(MathematicsException);                               // Return the inverse matrix
        static Matrix3x3 identity();                                                            // Return the 3x3 identity matrix

        // --- Overloaded operators --- //
        Matrix3x3 operator+(const Matrix3x3& matrix2) const;        // Overloaded operator for addition
        Matrix3x3 operator-(const Matrix3x3& matrix2) const ;       // Overloaded operator for substraction
        Matrix3x3 operator*(double nb) const;                       // Overloaded operator for multiplication with a number
        Matrix3x3 operator*(const Matrix3x3& matrix2) const;        // Overloaded operator for multiplication with a matrix
        Vector3D operator*(const Vector3D& vector3d) const;         // Overloaded operator for multiplication with a vector
        Matrix3x3& operator=(const Matrix3x3& matrix2);             // Overloaded operator for assignment
        bool operator==(const Matrix3x3& matrix2) const;            // Overloaded operator for equality condition
};


// Method to get a value in the matrix (inline)
inline double Matrix3x3::getValue(int i, int j) const throw(std::invalid_argument) {
    // Check the argument
    if (i>=0 && i<3 && j>=0 && j<3) {
        // Return the value
        return array[i][j];
    }
    else {
        // Throw an exception because of the wrong argument
        throw std::invalid_argument("Exception : The argument isn't in the bounds of the 3x3 matrix");
    }
}

// Method to set a value in the matrix (inline)
inline void Matrix3x3::setValue(int i, int j, double value) throw(std::invalid_argument) {
    // Check the argument
    if (i>=0 && i<3 && j>=0 && j<3) {
       // Set the value
       array[i][j] = value;
    }
    else {
        // Throw an exception because of the wrong argument
        throw std::invalid_argument("Exception : The argument isn't in the bounds of the 3x3 matrix");
    }
}

// Method to set all the values in the matrix
inline void Matrix3x3::setAllValues(double a1, double a2, double a3, double b1, double b2, double b3,
                  double c1, double c2, double c3) {
    // Set all the values of the matrix
    array[0][0] = a1;
    array[0][1] = a2;
    array[0][2] = a3;

    array[1][0] = b1;
    array[1][1] = b2;
    array[1][2] = b3;

    array[2][0] = c1;
    array[2][1] = c2;
    array[2][2] = c3;
}

// Return the transpose matrix
inline Matrix3x3 Matrix3x3::getTranspose() const {
    // Return the transpose matrix
    return Matrix3x3(array[0][0], array[1][0], array[2][0],
                     array[0][1], array[1][1], array[2][1],
                     array[0][2], array[1][2], array[2][2]);
}

// Return the determinant of the matrix
inline double Matrix3x3::getDeterminant() const {
    // Compute and return the determinant of the matrix
    return (array[0][0]*(array[1][1]*array[2][2]-array[2][1]*array[1][2]) - array[0][1]*(array[1][0]*array[2][2]-array[2][0]*array[1][2]) +
            array[0][2]*(array[1][0]*array[2][1]-array[2][0]*array[1][1]));
}

// Return the trace of the matrix
inline double Matrix3x3::getTrace() const {
    // Compute and return the trace
    return (array[0][0] + array[1][1] + array[2][2]);
}

// Overloaded operator for multiplication between a number and a Matrix3x3 (inline)
inline Matrix3x3 operator*(double number, const Matrix3x3& matrix) {
    // Return the multiplied matrix
    return matrix * number;
}

// Overloaded operator for multiplication with a vector
inline Vector3D Matrix3x3::operator*(const Vector3D& vector3d) const {
    // Compute and return the result
    return Vector3D(array[0][0]*vector3d.getX() + array[0][1]*vector3d.getY() + array[0][2]*vector3d.getZ(),
                    array[1][0]*vector3d.getX() + array[1][1]*vector3d.getY() + array[1][2]*vector3d.getZ(),
                    array[2][0]*vector3d.getX() + array[2][1]*vector3d.getY() + array[2][2]*vector3d.getZ());
}

// Overloaded operator for equality condition
inline bool Matrix3x3::operator==(const Matrix3x3& matrix2) const {
    return (array[0][0] == matrix2.array[0][0] && array[0][1] == matrix2.array[0][1] && array[0][2] == matrix2.array[0][2] &&
            array[1][0] == matrix2.array[1][0] && array[1][1] == matrix2.array[1][1] && array[1][2] == matrix2.array[1][2] &&
            array[2][0] == matrix2.array[2][0] && array[2][1] == matrix2.array[2][1] && array[2][2] == matrix2.array[2][2]);
}

} // End of the ReactPhysics3D namespace

#endif
