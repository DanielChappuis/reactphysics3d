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

#ifndef TEST_MATRIX2X2_H
#define TEST_MATRIX2X2_H

// Libraries
#include "Test.h"
#include <reactphysics3d/mathematics/Matrix2x2.h>

/// Reactphysics3D namespace
namespace reactphysics3d {

// Class TestMatrix2x2
/**
 * Unit test for the Matrix2x2 class
 */
class TestMatrix2x2 : public Test {

    private :

        // ---------- Atributes ---------- //

        /// Identity transform
        Matrix2x2 mIdentity;

        /// First example matrix
        Matrix2x2 mMatrix1;

    public :

        // ---------- Methods ---------- //

        /// Constructor
        TestMatrix2x2(const std::string& name)
               : Test(name), mIdentity(Matrix2x2::identity()), mMatrix1(2, 24, -4, 5) {

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

            Matrix2x2 test1(5.0);
            Matrix2x2 test2(2, 3, 4, 5);
            Matrix2x2 test3(mMatrix1);

            rp3d_test(test1[0][0] == 5);
            rp3d_test(test1[0][1] == 5);
            rp3d_test(test1[1][0] == 5);
            rp3d_test(test1[1][1] == 5);

            rp3d_test(test2[0][0] == 2);
            rp3d_test(test2[0][1] == 3);
            rp3d_test(test2[1][0] == 4);
            rp3d_test(test2[1][1] == 5);

            rp3d_test(test3 == mMatrix1);
        }

        /// Test the getter and setter methods
        void testGetSet() {

            // Test method to set all the values
            Matrix2x2 test2;
            test2.setAllValues(2, 24, -4, 5);
            rp3d_test(test2 == mMatrix1);

            // Test method to set to zero
            test2.setToZero();
            rp3d_test(test2 == Matrix2x2(0, 0, 0, 0));

            // Test method that returns a column
            Vector2 column1 = mMatrix1.getColumn(0);
            Vector2 column2 = mMatrix1.getColumn(1);
            rp3d_test(column1 == Vector2(2, -4));
            rp3d_test(column2 == Vector2(24, 5));

            // Test method that returns a row
            Vector2 row1 = mMatrix1.getRow(0);
            Vector2 row2 = mMatrix1.getRow(1);
            rp3d_test(row1 == Vector2(2, 24));
            rp3d_test(row2 == Vector2(-4, 5));
        }

        /// Test the identity methods
        void testIdentity() {

            Matrix2x2 identity = Matrix2x2::identity();
            Matrix2x2 test1;
            test1.setToIdentity();

            rp3d_test(identity[0][0] == 1);
            rp3d_test(identity[0][1] == 0);
            rp3d_test(identity[1][0] == 0);
            rp3d_test(identity[1][1] == 1);

            rp3d_test(test1 == Matrix2x2::identity());
        }

        /// Test the zero method
        void testZero() {

            Matrix2x2 zero = Matrix2x2::zero();

            rp3d_test(zero[0][0] == 0);
            rp3d_test(zero[0][1] == 0);
            rp3d_test(zero[1][0] == 0);
            rp3d_test(zero[1][1] == 0);
        }

        /// Test others methods
        void testOthersMethods() {

            // Test transpose
            Matrix2x2 transpose = mMatrix1.getTranspose();
            rp3d_test(transpose == Matrix2x2(2, -4, 24, 5));

            // Test trace
            rp3d_test(mMatrix1.getTrace() ==7);
            rp3d_test(Matrix2x2::identity().getTrace() == 2);

            // Test determinant
            Matrix2x2 matrix(-24, 64, 253, -35);
            rp3d_test(mMatrix1.getDeterminant() == 106);
            rp3d_test(matrix.getDeterminant() == -15352);
            rp3d_test(mIdentity.getDeterminant() == 1);

            // Test inverse
            Matrix2x2 matrix2(1, 2, 3, 4);
            Matrix2x2 inverseMatrix = matrix2.getInverse();
            rp3d_test(approxEqual(inverseMatrix[0][0], decimal(-2), decimal(10e-6)));
            rp3d_test(approxEqual(inverseMatrix[0][1], decimal(1), decimal(10e-6)));
            rp3d_test(approxEqual(inverseMatrix[1][0], decimal(1.5), decimal(10e-6)));
            rp3d_test(approxEqual(inverseMatrix[1][1], decimal(-0.5), decimal(10e-6)));
            Matrix2x2 inverseMatrix1 = mMatrix1.getInverse();
            rp3d_test(approxEqual(inverseMatrix1[0][0], decimal(0.047169811), decimal(10e-6)));
            rp3d_test(approxEqual(inverseMatrix1[0][1], decimal(-0.226415094), decimal(10e-6)));
            rp3d_test(approxEqual(inverseMatrix1[1][0], decimal(0.037735849), decimal(10e-6)));
            rp3d_test(approxEqual(inverseMatrix1[1][1], decimal(0.018867925), decimal(10e-6)));

            // Test absolute matrix
            Matrix2x2 matrix3(-2, -3, -4, -5);
            rp3d_test(matrix.getAbsoluteMatrix() == Matrix2x2(24, 64, 253, 35));
            Matrix2x2 absoluteMatrix = matrix3.getAbsoluteMatrix();
            rp3d_test(absoluteMatrix == Matrix2x2(2, 3, 4, 5));
        }

        /// Test the operators
        void testOperators() {

            // Test addition
            Matrix2x2 matrix1(2, 3, 4, 5);
            Matrix2x2 matrix2(-2, 3, -5, 10);
            Matrix2x2 addition1 = matrix1 + matrix2;
            Matrix2x2 addition2(matrix1);
            addition2 += matrix2;
            rp3d_test(addition1 == Matrix2x2(0, 6, -1, 15));
            rp3d_test(addition2 == Matrix2x2(0, 6, -1, 15));

            // Test substraction
            Matrix2x2 substraction1 = matrix1 - matrix2;
            Matrix2x2 substraction2(matrix1);
            substraction2 -= matrix2;
            rp3d_test(substraction1 == Matrix2x2(4, 0, 9, -5));
            rp3d_test(substraction2 == Matrix2x2(4, 0, 9, -5));

            // Test negative operator
            Matrix2x2 negative = -matrix1;
            rp3d_test(negative == Matrix2x2(-2, -3, -4, -5));

            // Test multiplication with a number
            Matrix2x2 multiplication1 = 3 * matrix1;
            Matrix2x2 multiplication2 = matrix1 * 3;
            Matrix2x2 multiplication3(matrix1);
            multiplication3 *= 3;
            rp3d_test(multiplication1 == Matrix2x2(6, 9, 12, 15));
            rp3d_test(multiplication2 == Matrix2x2(6, 9, 12, 15));
            rp3d_test(multiplication3 == Matrix2x2(6, 9, 12, 15));

            // Test multiplication with a matrix
            Matrix2x2 multiplication4 = matrix1 * matrix2;
            Matrix2x2 multiplication5 = matrix2 * matrix1;
            rp3d_test(multiplication4 == Matrix2x2(-19, 36, -33, 62));
            rp3d_test(multiplication5 == Matrix2x2(8, 9, 30, 35));

            // Test multiplication with a vector
            Vector2 vector1(3, -32);
            Vector2 vector2(-31, -422);
            Vector2 test1 = matrix1 * vector1;
            Vector2 test2 = matrix2 * vector2;
            rp3d_test(test1 == Vector2(-90, -148));
            rp3d_test(test2 == Vector2(-1204, -4065));

            // Test equality operators
            rp3d_test(Matrix2x2(34, 38, 43, 64) ==
                 Matrix2x2(34, 38, 43, 64));
            rp3d_test(Matrix2x2(34, 64, 43, 7) !=
                 Matrix2x2(34, 38, 43, 64));

            // Test operator to read a value
            rp3d_test(mMatrix1[0][0] == 2);
            rp3d_test(mMatrix1[0][1] == 24);
            rp3d_test(mMatrix1[1][0] == -4);
            rp3d_test(mMatrix1[1][1] == 5);

            // Test operator to set a value
            Matrix2x2 test3;
            test3[0][0] = 2;
            test3[0][1] = 24;
            test3[1][0] = -4;
            test3[1][1] = 5;
            rp3d_test(test3 == mMatrix1);
        }

 };

}

#endif
