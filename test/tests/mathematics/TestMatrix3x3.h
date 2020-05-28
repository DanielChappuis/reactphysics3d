/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2016 Daniel Chappuis                                       *
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

#ifndef TEST_MATRIX3X3_H
#define TEST_MATRIX3X3_H

// Libraries
#include "Test.h"
#include <reactphysics3d/mathematics/Matrix3x3.h>

/// Reactphysics3D namespace
namespace reactphysics3d {

// Class TestMatrix3x3
/**
 * Unit test for the Matrix3x3 class
 */
class TestMatrix3x3 : public Test {

    private :

        // ---------- Atributes ---------- //

        /// Identity transform
        Matrix3x3 mIdentity;

        /// First example matrix
        Matrix3x3 mMatrix1;

    public :

        // ---------- Methods ---------- //

        /// Constructor
        TestMatrix3x3(const std::string& name)
            : Test(name), mIdentity(Matrix3x3::identity()),
              mMatrix1(2, 24, 4, 5, -6, 234, -15, 11, 66) {


        }

        /// Run the tests
        void run() {
            testConstructors();
            testGetSet();
            testIdentity();
            testZero();
            testOthersMethods();
            testOperators();
        }

        /// Test the constructors
        void testConstructors() {

            Matrix3x3 test1(5.0);
            Matrix3x3 test2(2, 3, 4, 5, 6, 7, 8, 9, 10);
            Matrix3x3 test3(mMatrix1);
            rp3d_test(test1[0][0] == 5);
            rp3d_test(test1[0][1] == 5);
            rp3d_test(test1[0][2] == 5);
            rp3d_test(test1[1][0] == 5);
            rp3d_test(test1[1][1] == 5);
            rp3d_test(test1[1][2] == 5);
            rp3d_test(test1[2][0] == 5);
            rp3d_test(test1[2][1] == 5);
            rp3d_test(test1[2][2] == 5);

            rp3d_test(test2[0][0] == 2);
            rp3d_test(test2[0][1] == 3);
            rp3d_test(test2[0][2] == 4);
            rp3d_test(test2[1][0] == 5);
            rp3d_test(test2[1][1] == 6);
            rp3d_test(test2[1][2] == 7);
            rp3d_test(test2[2][0] == 8);
            rp3d_test(test2[2][1] == 9);
            rp3d_test(test2[2][2] == 10);

            rp3d_test(test3 == mMatrix1);
        }

        /// Test the getter and setter methods
        void testGetSet() {

            // Test method to set all the values
            Matrix3x3 test2;
            test2.setAllValues(2, 24, 4, 5, -6, 234, -15, 11, 66);
            rp3d_test(test2 == mMatrix1);

            // Test method to set to zero
            test2.setToZero();
            rp3d_test(test2 == Matrix3x3(0, 0, 0, 0, 0, 0, 0, 0, 0));

            // Test method that returns a column
            Vector3 column1 = mMatrix1.getColumn(0);
            Vector3 column2 = mMatrix1.getColumn(1);
            Vector3 column3 = mMatrix1.getColumn(2);
            rp3d_test(column1 == Vector3(2, 5, -15));
            rp3d_test(column2 == Vector3(24, -6, 11));
            rp3d_test(column3 == Vector3(4, 234, 66));

            // Test method that returns a row
            Vector3 row1 = mMatrix1.getRow(0);
            Vector3 row2 = mMatrix1.getRow(1);
            Vector3 row3 = mMatrix1.getRow(2);
            rp3d_test(row1 == Vector3(2, 24, 4));
            rp3d_test(row2 == Vector3(5, -6, 234));
            rp3d_test(row3 == Vector3(-15, 11, 66));
        }

        /// Test the identity methods
        void testIdentity() {

            Matrix3x3 identity = Matrix3x3::identity();
            Matrix3x3 test1;
            test1.setToIdentity();

            rp3d_test(identity[0][0] == 1);
            rp3d_test(identity[0][1] == 0);
            rp3d_test(identity[0][2] == 0);
            rp3d_test(identity[1][0] == 0);
            rp3d_test(identity[1][1] == 1);
            rp3d_test(identity[1][2] == 0);
            rp3d_test(identity[2][0] == 0);
            rp3d_test(identity[2][1] == 0);
            rp3d_test(identity[2][2] == 1);

            rp3d_test(test1 == Matrix3x3::identity());
        }

        /// Test the zero method
        void testZero() {

            Matrix3x3 zero = Matrix3x3::zero();

            rp3d_test(zero[0][0] == 0);
            rp3d_test(zero[0][1] == 0);
            rp3d_test(zero[0][2] == 0);
            rp3d_test(zero[1][0] == 0);
            rp3d_test(zero[1][1] == 0);
            rp3d_test(zero[1][2] == 0);
            rp3d_test(zero[2][0] == 0);
            rp3d_test(zero[2][1] == 0);
            rp3d_test(zero[2][2] == 0);
        }

        /// Test others methods
        void testOthersMethods() {

            // Test transpose
            Matrix3x3 transpose = mMatrix1.getTranspose();
            rp3d_test(transpose == Matrix3x3(2, 5, -15, 24, -6, 11, 4, 234, 66));

            // Test trace
            rp3d_test(mMatrix1.getTrace() == 62);
            rp3d_test(Matrix3x3::identity().getTrace() == 3);

            // Test determinant
            Matrix3x3 matrix(-24, 64, 253, -35, 52, 72, 21, -35, -363);
            rp3d_test(mMatrix1.getDeterminant() == -98240);
            rp3d_test(matrix.getDeterminant() == -290159);
            rp3d_test(mIdentity.getDeterminant() == 1);

            // Test inverse
            Matrix3x3 inverseMatrix = matrix.getInverse();
            rp3d_test(approxEqual(inverseMatrix[0][0], decimal(0.056369), decimal(10e-6)));
            rp3d_test(approxEqual(inverseMatrix[0][1], decimal(-0.049549), decimal(10e-6)));
            rp3d_test(approxEqual(inverseMatrix[0][2], decimal(0.029460), decimal(10e-6)));
            rp3d_test(approxEqual(inverseMatrix[1][0], decimal(0.038575), decimal(10e-6)));
            rp3d_test(approxEqual(inverseMatrix[1][1], decimal(-0.011714), decimal(10e-6)));
            rp3d_test(approxEqual(inverseMatrix[1][2], decimal(0.024562), decimal(10e-6)));
            rp3d_test(approxEqual(inverseMatrix[2][0], decimal(-0.000458), decimal(10e-6)));
            rp3d_test(approxEqual(inverseMatrix[2][1], decimal(-0.001737), decimal(10e-6)));
            rp3d_test(approxEqual(inverseMatrix[2][2], decimal(-0.003419), decimal(10e-6)));
            Matrix3x3 inverseMatrix1 = mMatrix1.getInverse();
            rp3d_test(approxEqual(inverseMatrix1[0][0], decimal(0.030232), decimal(10e-6)));
            rp3d_test(approxEqual(inverseMatrix1[0][1], decimal(0.015676), decimal(10e-6)));
            rp3d_test(approxEqual(inverseMatrix1[0][2], decimal(-0.057410), decimal(10e-6)));
            rp3d_test(approxEqual(inverseMatrix1[1][0], decimal(0.039088), decimal(10e-6)));
            rp3d_test(approxEqual(inverseMatrix1[1][1], decimal(-0.001954), decimal(10e-6)));
            rp3d_test(approxEqual(inverseMatrix1[1][2], decimal(0.004560), decimal(10e-6)));
            rp3d_test(approxEqual(inverseMatrix1[2][0], decimal(0.000356), decimal(10e-6)));
            rp3d_test(approxEqual(inverseMatrix1[2][1], decimal(0.003888), decimal(10e-6)));
            rp3d_test(approxEqual(inverseMatrix1[2][2], decimal(0.001344), decimal(10e-6)));

            // Test absolute matrix
            Matrix3x3 matrix2(-2, -3, -4, -5, -6, -7, -8, -9, -10);
            rp3d_test(matrix.getAbsoluteMatrix() == Matrix3x3(24, 64, 253, 35, 52, 72, 21, 35, 363));
            Matrix3x3 absoluteMatrix = matrix2.getAbsoluteMatrix();
            rp3d_test(absoluteMatrix == Matrix3x3(2, 3, 4, 5, 6, 7, 8, 9, 10));

            // Test method that computes skew-symmetric matrix for cross product
            Vector3 vector1(3, -5, 6);
            Vector3 vector2(73, 42, 26);
            Matrix3x3 skewMatrix = Matrix3x3::computeSkewSymmetricMatrixForCrossProduct(vector1);
            rp3d_test(skewMatrix == Matrix3x3(0, -6, -5, 6, 0, -3, 5, 3, 0));
            Vector3 crossProduct1 = vector1.cross(vector2);
            Vector3 crossProduct2 = skewMatrix * vector2;
            rp3d_test(crossProduct1 == crossProduct2);
        }

        /// Test the operators
        void testOperators() {

            // Test addition
            Matrix3x3 matrix1(2, 3, 4, 5, 6, 7, 8, 9, 10);
            Matrix3x3 matrix2(-2, 3, -5, 10, 4, 7, 2, 5, 8);
            Matrix3x3 addition1 = matrix1 + matrix2;
            Matrix3x3 addition2(matrix1);
            addition2 += matrix2;
            rp3d_test(addition1 == Matrix3x3(0, 6, -1, 15, 10, 14, 10, 14, 18));
            rp3d_test(addition2 == Matrix3x3(0, 6, -1, 15, 10, 14, 10, 14, 18));

            // Test substraction
            Matrix3x3 substraction1 = matrix1 - matrix2;
            Matrix3x3 substraction2(matrix1);
            substraction2 -= matrix2;
            rp3d_test(substraction1 == Matrix3x3(4, 0, 9, -5, 2, 0, 6, 4, 2));
            rp3d_test(substraction2 == Matrix3x3(4, 0, 9, -5, 2, 0, 6, 4, 2));

            // Test negative operator
            Matrix3x3 negative = -matrix1;
            rp3d_test(negative == Matrix3x3(-2, -3, -4, -5, -6, -7, -8, -9, -10));

            // Test multiplication with a number
            Matrix3x3 multiplication1 = 3 * matrix1;
            Matrix3x3 multiplication2 = matrix1 * 3;
            Matrix3x3 multiplication3(matrix1);
            multiplication3 *= 3;
            rp3d_test(multiplication1 == Matrix3x3(6, 9, 12, 15, 18, 21, 24, 27, 30));
            rp3d_test(multiplication2 == Matrix3x3(6, 9, 12, 15, 18, 21, 24, 27, 30));
            rp3d_test(multiplication3 == Matrix3x3(6, 9, 12, 15, 18, 21, 24, 27, 30));

            // Test multiplication with a matrix
            Matrix3x3 multiplication4 = matrix1 * matrix2;
            Matrix3x3 multiplication5 = matrix2 * matrix1;
            rp3d_test(multiplication4 == Matrix3x3(34, 38, 43, 64, 74, 73, 94, 110, 103));
            rp3d_test(multiplication5 == Matrix3x3(-29, -33, -37, 96, 117, 138, 93, 108, 123));

            // Test multiplication with a vector
            Vector3 vector1(3, -32, 59);
            Vector3 vector2(-31, -422, 34);
            Vector3 test1 = matrix1 * vector1;
            Vector3 test2 = matrix2 * vector2;
            rp3d_test(test1 == Vector3(146, 236, 326));
            rp3d_test(test2 == Vector3(-1374, -1760, -1900));

            // Test equality operators
            rp3d_test(Matrix3x3(34, 38, 43, 64, 74, 73, 94, 110, 103) ==
                 Matrix3x3(34, 38, 43, 64, 74, 73, 94, 110, 103));
            rp3d_test(Matrix3x3(34, 64, 43, 7, -1, 73, 94, 110, 103) !=
                 Matrix3x3(34, 38, 43, 64, 74, 73, 94, 110, 103));

            // Test operator to read a value
            rp3d_test(mMatrix1[0][0] == 2);
            rp3d_test(mMatrix1[0][1] == 24);
            rp3d_test(mMatrix1[0][2] == 4);
            rp3d_test(mMatrix1[1][0] == 5);
            rp3d_test(mMatrix1[1][1] == -6);
            rp3d_test(mMatrix1[1][2] == 234);
            rp3d_test(mMatrix1[2][0] == -15);
            rp3d_test(mMatrix1[2][1] == 11);
            rp3d_test(mMatrix1[2][2] == 66);

            // Test operator to set a value
            Matrix3x3 test3;
            test3[0][0] = 2;
            test3[0][1] = 24;
            test3[0][2] = 4;
            test3[1][0] = 5;
            test3[1][1] = -6;
            test3[1][2] = 234;
            test3[2][0] = -15;
            test3[2][1] = 11;
            test3[2][2] = 66;
            rp3d_test(test3 == mMatrix1);
        }

 };

}

#endif
