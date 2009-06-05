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

#ifndef MATRIXTEST_H
#define MATRIXTEST_H

// Libraries
#include "../TestSuite/Test.h"
#include "../../mathematics/Matrix.h"
#include <stdexcept>

// Namespaces
using namespace reactphysics3d;

// Class MatrixTest
class MatrixTest : public TestSuite::Test {
   private :
        Matrix matrix1;
        Matrix matrix2;
        Matrix matrix3;
   public :

        // Constructor
        MatrixTest() : matrix1(2,3), matrix2(2,3), matrix3(3,2) {
            matrix1.setValue(0, 0, 4);
            matrix1.setValue(0, 1, 5);
            matrix1.setValue(0, 2, 7);
            matrix1.setValue(1, 0, 2);
            matrix1.setValue(1, 1, 3);
            matrix1.setValue(1, 2, -4);

            matrix2.setValue(0, 0, -12);
            matrix2.setValue(0, 1, 3);
            matrix2.setValue(0, 2, 16);
            matrix2.setValue(1, 0, -7);
            matrix2.setValue(1, 1, 4);
            matrix2.setValue(1, 2, 6);

            matrix3.setValue(0, 0, -4);
            matrix3.setValue(0, 1, -2);
            matrix3.setValue(1, 0, 7);
            matrix3.setValue(1, 1, 9);
            matrix3.setValue(2, 0, 18);
            matrix3.setValue(2, 1, 33);
        }

        // Run method of the Test
        void run() {
            testConstructors();
            testGetValue();
            testSetValue();
            testGetNbRow();
            testGetNbColumn();
            testGetCofactor();
            testGetTranspose();
            testGetInverse();
            testGetDeterminant();
            testGetTrace();
            testIdentityMatrix();
            testOperatorAddition();
            testOperatorSubstraction();
            testOperatorMultiplicationWithConstant();
            testOperatorMultiplicationWithMatrix();
            testOperatorAssignment();
            testOperatorEquality();
        }

        // Test the constructors
        void testConstructors() {

            // Try a valid constructor call
            try {
                // Constructor
                Matrix matrix(4,6);                 // This shouldn't throw an exception
                test_(matrix.getNbRow() == 4);
                test_(matrix.getNbColumn() == 6);
                test_(matrix.getValue(0,0) == 0);
                succeed_();                         // Succeed if no exceptions have been thrown
            }
            catch(std::invalid_argument& ex) {
                fail_("Valid constructor call throws an exception");    // Failed if an exception has been thrown
            }

            // Try an invalid constructor call
            try {
                // Constructor
                Matrix matrix(-2,6);                                               // This should throw an exception
                fail_("Invalid constructor call undetected (argument -2) ");       // Failed if no exception have been thrown
            }
            catch(std::invalid_argument& ex) {
                succeed_();                         // Succeed if an exception has been thrown
            }

            // Try an invalid constructor call
            try {
                // Constructor
                Matrix matrix(3,0);                                              // This should throw an exception
                fail_("Invalid constructor call undetected (argument 0)");       // Failed if no exception have been thrown
            }
            catch(std::invalid_argument& ex) {
                succeed_();                         // Succeed if an exception has been thrown
            }

            // Copy-constructor
            Matrix matrix4 = matrix1;
            test_(matrix4.getNbRow() == 2);
            test_(matrix4.getNbColumn() == 3);
            test_(matrix4.getValue(0, 0) == 4);
            test_(matrix4.getValue(0, 1) == 5);
            test_(matrix4.getValue(0, 2) == 7);
            test_(matrix4.getValue(1, 0) == 2);
            test_(matrix4.getValue(1, 1) == 3);
            test_(matrix4.getValue(1, 2) == -4);
        }

        // Test getValue()
        void testGetValue() {
            // Try a valid getValue()
            try {
                test_(matrix1.getValue(0, 0) == 4);                     // This shouldn't throw an exception
                test_(matrix1.getValue(0, 2) == 7);                     // This shouldn't throw an exception
                test_(matrix1.getValue(1, 0) == 2);                     // This shouldn't throw an exception
                test_(matrix1.getValue(1, 2) == -4);                    // This shouldn't throw an exception
                test_(matrix1.getValue(1, 1) == 3);                     // This shouldn't throw an exception
                succeed_();                                             // Succeed if no exceptions have been thrown
            }
            catch(std::invalid_argument& ex) {
                fail_("Valid getValue() call throws an exception");     // Failed if an exception has been thrown
            }

            // Try an invalid getValue() call
            try {
                matrix1.getValue(-1, 0);                     // This should throw an exception
                fail_("Invalid getValue() call undetected");
            }
            catch(std::invalid_argument& ex) {
                succeed_();                         // Succeed if an exception has been thrown
            }

            // Try an invalid getValue() call
            try {
                matrix1.getValue(0, 3);                     // This should throw an exception
                fail_("Invalid getValue() call undetected");
            }
            catch(std::invalid_argument& ex) {
                succeed_();                         // Succeed if an exception has been thrown
            }
        }

        // Test setValue()
        void testSetValue() {

            Matrix matrix(2,3);

            // Try a valid setValue()
            try {
                matrix.setValue(0, 0, 18);              // This shouldn't throw an exception
                matrix.setValue(0, 2, -6);              // This shouldn't throw an exception
                matrix.setValue(1, 0, -44);             // This shouldn't throw an exception
                matrix.setValue(1, 2, 21);              // This shouldn't throw an exception
                matrix.setValue(1, 1, 5);               // This shouldn't throw an exception
                test_(matrix.getValue(0, 0) == 18);
                test_(matrix.getValue(0, 2) == -6);
                test_(matrix.getValue(1, 0) == -44);
                test_(matrix.getValue(1, 2) == 21);
                test_(matrix.getValue(1, 1) == 5);
                succeed_();                                             // Succeed if no exceptions have been thrown
            }
            catch(std::invalid_argument& ex) {
                fail_("Valid setValue() call throws an exception");     // Failed if an exception has been thrown
            }

            // Try an invalid setValue() call
            try {
                matrix.setValue(-1, 0, 42);                     // This should throw an exception
                fail_("Invalid setValue() call undetected");    // Failed if no exception have been thrown
            }
            catch(std::invalid_argument& ex) {
                succeed_();                                     // Succeed if an exception has been thrown
            }

            // Try an invalid setValue() call
            try {
                matrix1.setValue(0, 3, 53);                     // This should throw an exception
                fail_("Invalid setValue() call undetected");    // Failed if no exception have been thrown
            }
            catch(std::invalid_argument& ex) {
                succeed_();                                     // Succeed if an exception has been thrown
            }
        }

        // Test getNbRow()
        void testGetNbRow() {
            test_(matrix1.getNbRow() == 2);
            test_(matrix3.getNbRow() == 3);
        }

        // Test getNbColumn()
        void testGetNbColumn() {
            test_(matrix1.getNbColumn() == 3);
            test_(matrix3.getNbColumn() == 2);
        }

        // Test getCofactor()
        void testGetCofactor() {

            // Try a valid getCofactor()
            try {
                Matrix matrix = matrix1.getCofactor(0,1);              // This shouldn't throw an exception

                test_(matrix.getNbRow() == 1);
                test_(matrix.getNbColumn() == 2);
                test_(matrix.getValue(0, 0) == 2);
                test_(matrix.getValue(0, 1) == -4);

                Matrix matrix4 = matrix1.getCofactor(1,2);              // This shouldn't throw an exception

                test_(matrix4.getNbRow() == 1);
                test_(matrix4.getNbColumn() == 2);
                test_(matrix4.getValue(0, 0) == 4);
                test_(matrix4.getValue(0, 1) == 5);
                succeed_();                                             // Succeed if no exceptions have been thrown
            }
            catch(std::invalid_argument& ex) {
                fail_("Valid getCofactor() call throws an exception");  // Failed if an exception has been thrown
            }

            // Try an invalid getCofactor() call
            try {
                matrix1.getCofactor(-1,0);                          // This should throw an exception
                fail_("Invalid getCofactor() call undetected");     // Failed if no exception have been thrown
            }
            catch(std::invalid_argument& ex) {
                succeed_();                                         // Succeed if an exception has been thrown
            }

            // Try an invalid getCofactor() call
            try {
                matrix1.getCofactor(0,3);                           // This should throw an exception
                fail_("Invalid getCofactor() call undetected");     // Failed if no exception have been thrown
            }
            catch(std::invalid_argument& ex) {
                succeed_();                                         // Succeed if an exception has been thrown
            }
        }

        // Test getTranspose()
        void testGetTranspose() {
            // Get the transpose of matrix1
            Matrix matrix = matrix1.getTranspose();

            // Test the transpose matrix
            test_(matrix.getNbRow() == 3);
            test_(matrix.getNbColumn() == 2);
            test_(matrix.getValue(0, 0) == 4);
            test_(matrix.getValue(0, 1) == 2);
            test_(matrix.getValue(1, 0) == 5);
            test_(matrix.getValue(1, 1) == 3);
            test_(matrix.getValue(2, 0) == 7);
            test_(matrix.getValue(2, 1) == -4);
        }

        // Test getInverse()
        void testGetInverse() {

            // Construct a 3x3 matrix
            Matrix matrix(3,3);
            matrix.setValue(0, 0, 0);
            matrix.setValue(0, 1, 1);
            matrix.setValue(0, 2, 2);
            matrix.setValue(1, 0, 1);
            matrix.setValue(1, 1, 0);
            matrix.setValue(1, 2, 3);
            matrix.setValue(2, 0, 4);
            matrix.setValue(2, 1, -3);
            matrix.setValue(2, 2, 8);

            // Try to inverse a invertible matrix
            try {
                Matrix result = matrix.getInverse();                // This shouldn't thrown an exception
                test_(result.getValue(0, 0) == -4.5);
                test_(result.getValue(0, 1) == 7);
                test_(result.getValue(0, 2) == -3.0/2.0);
                test_(result.getValue(1, 0) == -2);
                test_(result.getValue(1, 1) == 4);
                test_(result.getValue(1, 2) == -1);
                test_(result.getValue(2, 0) == 3.0/2.0);
                test_(result.getValue(2, 1) == -2);
                test_(result.getValue(2, 2) == 1.0/2.0);
                succeed_();                                             // Succeed if no exceptions have been thrown
            }
            catch(MathematicsException& ex) {
                fail_("Valid getInverse() call throws an exception");  // Failed if an exception has been thrown
            }

            // Try to inverse a non-square matrix
            try {
                matrix1.getInverse();                                               // This should throw an exception
                fail_("Invalid getInverse() call undetected (non-square matrix)");  // Failed if no exception have been thrown
            }
            catch(MathematicsException& ex) {
                succeed_();                                                         // Succeed if an exception has been thrown
            }

            // Try to inverse a square non-invertible matrix (determinant equal to zero)
            try {
                Matrix matrix4(2,2);
                matrix4.setValue(0, 0, 3);
                matrix4.setValue(0, 1, 2);
                matrix4.setValue(1, 0, 3);
                matrix4.setValue(1, 1, 2);
                matrix4.getInverse();                                                   // This should throw an exception
                fail_("Invalid getInverse() call undetected (non-invertible matrix)");  // Failed if no exception have been thrown
            }
            catch(MathematicsException& ex) {
                succeed_();                           // Succeed if an exception has been thrown
            }
        }

        // Test getDeterminant()
        void testGetDeterminant() {

             // Try to compute the determinant of a square matrix
            try {
                Matrix matrix(2,2);
                test_(matrix.getDeterminant() == 0);                    // This shouldn't throw an exception
                matrix.setValue(0,0, 4);
                matrix.setValue(0,1, -9);
                matrix.setValue(1,0, 0);
                matrix.setValue(1,1, 5);
                test_(matrix.getDeterminant() == 20);                    // This shouldn't throw an exception
                matrix.setValue(0,0, 6);
                matrix.setValue(0,1, -9);
                matrix.setValue(1,0, -4);
                matrix.setValue(1,1, 6);
                test_(matrix.getDeterminant() == 0);                    // This shouldn't throw an exception
                succeed_();                                             // Succeed if no exceptions have been thrown
            }
            catch(MathematicsException& ex) {
                fail_("Valid getDeterminant() call throws an exception");  // Failed if an exception has been thrown
            }

            // Try to compute the determinant of a non-square matrix
            try {
                Matrix matrix5(2,8);
                matrix5.setValue(0, 2, 3);
                matrix5.setValue(1, 1, 2);
                matrix5.getDeterminant();                                               // This should throw an exception
                fail_("getDeterminant() call with a non-square matrix undetected");     // Failed if no exception have been thrown
            }
            catch(MathematicsException& ex) {
                succeed_();                           // Succeed if an exception has been thrown
            }
        }

        // Test getTrace()
        void testGetTrace() {

            // Try to compute the trace of a square matrix
            try {
                // Construct a 3x3 matrix
                Matrix matrix(3,3);
                matrix.setValue(0, 0, -2);
                matrix.setValue(0, 1, 1);
                matrix.setValue(0, 2, 2);
                matrix.setValue(1, 0, 1);
                matrix.setValue(1, 1, 5);
                matrix.setValue(1, 2, 3);
                matrix.setValue(2, 0, 4);
                matrix.setValue(2, 1, -3);
                matrix.setValue(2, 2, 8);
                test_(matrix.getTrace() == 11);                         // This shouldn't throw an exception
                succeed_();                                             // Succeed if no exceptions have been thrown
            }
            catch(MathematicsException& ex) {
                fail_("Valid getTrace() call throws an exception");  // Failed if an exception has been thrown
            }

            // Try to compute the trace of a non-square matrix
            try {
                Matrix matrix5(2,8);
                matrix5.getTrace();                                               // This should throw an exception
                fail_("getTrace() call with a non-square matrix undetected");     // Failed if no exception have been thrown
            }
            catch(MathematicsException& ex) {
                succeed_();                           // Succeed if an exception has been thrown
            }
        }

        // Test identityMatrix()
        void testIdentityMatrix() {


            // Try to compute a valid identity matrix
            try {
                Matrix matrix = Matrix::identity(2);                     // This shouldn't throw an exception
                test_(matrix.getNbRow() == 2);
                test_(matrix.getNbColumn() == 2);
                test_(matrix.getValue(0, 0) == 1);
                test_(matrix.getValue(0, 1) == 0);
                test_(matrix.getValue(1, 0) == 0);
                test_(matrix.getValue(1, 1) == 1);
                succeed_();                                             // Succeed if no exceptions have been thrown
            }
            catch(std::invalid_argument& ex) {
                fail_("Valid identity() call throws an exception");     // Failed if an exception has been thrown
            }

            // Try to compute an invalid identity matrix
            try {
                Matrix matrix5 = Matrix::identity(0);                           // This should throw an exception
                fail_("Invalid identity() call (argument 0) undetected");       // Failed if no exception have been thrown
            }
            catch(std::invalid_argument& ex) {
                succeed_();                           // Succeed if an exception has been thrown
            }

            // Try to compute an invalid identity matrix
            try {
                Matrix matrix5 = Matrix::identity(-1);                          // This should throw an exception
                fail_("Invalid identity() call (argument -1) undetected");      // Failed if no exception have been thrown
            }
            catch(std::invalid_argument& ex) {
                succeed_();                           // Succeed if an exception has been thrown
            }
        }

        // Test operator+()
        void testOperatorAddition() {

            // Try to compute a valid addition
            try {
                Matrix result = matrix1 + matrix2;                      // This shouldn't throw an exception
                test_(result.getNbRow() == 2);
                test_(result.getNbColumn() == 3);
                test_(result.getValue(0,0) == -8);
                test_(result.getValue(0,1) == 8);
                test_(result.getValue(0,2) == 23);
                test_(result.getValue(1,0) == -5);
                test_(result.getValue(1,1) == 7);
                test_(result.getValue(1,2) == 2);
                succeed_();                                             // Succeed if no exceptions have been thrown
            }
            catch(MathematicsException& ex) {
                fail_("Valid matrix addition throws an exception");     // Failed if an exception has been thrown
            }

            // Try to compute an invalid addition
            try {
                Matrix matrix5(2,4);
                matrix1 + matrix5;                                      // This should throw an exception
                fail_("Invalid matrix addition undetected");            // Failed if no exception have been thrown
            }
            catch(MathematicsException& ex) {
                succeed_();                                             // Succeed if an exception has been thrown
            }

            // Try to compute an invalid addition
            try {
                Matrix matrix5(1,3);
                matrix1 + matrix5;                                      // This should throw an exception
                fail_("Invalid matrix addition undetected");            // Failed if no exception have been thrown
            }
            catch(MathematicsException& ex) {
                succeed_();                                             // Succeed if an exception has been thrown
            }
        }

        // Test operator-()
        void testOperatorSubstraction() {

            // Try to compute a valid substraction
            try {
                Matrix result = matrix1 - matrix2;                      // This shouldn't throw an exception
                test_(result.getNbRow() == 2);
                test_(result.getNbColumn() == 3);
                test_(result.getValue(0,0) == 16);
                test_(result.getValue(0,1) == 2);
                test_(result.getValue(0,2) == -9);
                test_(result.getValue(1,0) == 9);
                test_(result.getValue(1,1) == -1);
                test_(result.getValue(1,2) == -10);
                succeed_();                                                 // Succeed if no exceptions have been thrown
            }
            catch(MathematicsException& ex) {
                fail_("Valid matrix substraction throws an exception");     // Failed if an exception has been thrown
            }

            // Try to compute an invalid substraction
            try {
                Matrix matrix5(2,4);
                matrix1 - matrix5;                                      // This should throw an exception
                fail_("Invalid matrix substraction undetected");        // Failed if no exception have been thrown
            }
            catch(MathematicsException& ex) {
                succeed_();                                             // Succeed if an exception has been thrown
            }

            // Try to compute an invalid substraction
            try {
                Matrix matrix5(1,3);
                matrix1 - matrix5;                                      // This should throw an exception
                fail_("Invalid matrix substraction undetected");        // Failed if no exception have been thrown
            }
            catch(MathematicsException& ex) {
                succeed_();                                             // Succeed if an exception has been thrown
            }
        }

        // Test operator* (multiplication with a constant number)
        void testOperatorMultiplicationWithConstant() {
            Matrix matrix = matrix1 * 2;
            test_(matrix.getNbRow() == 2);
            test_(matrix.getNbColumn() == 3);
            test_(matrix.getValue(0,0) == 8);
            test_(matrix.getValue(0,1) == 10);
            test_(matrix.getValue(0,2) == 14);
            test_(matrix.getValue(1,0) == 4);
            test_(matrix.getValue(1,1) == 6);
            test_(matrix.getValue(1,2) == -8);
        }

        // Test operator* (multiplication with matrix)
        void testOperatorMultiplicationWithMatrix() {

            // Try to compute a valid multiplication
            try {
                Matrix result = matrix1 * matrix3;                          // This shouldn't throw an exception
                test_(result.getNbRow() == 2);
                test_(result.getNbColumn() == 2);
                test_(result.getValue(0,0) == 145);
                test_(result.getValue(0,1) == 268);
                test_(result.getValue(1,0) == -59);
                test_(result.getValue(1,1) == -109);
                succeed_();                                                 // Succeed if no exceptions have been thrown
            }
            catch(MathematicsException& ex) {
                fail_("Valid matrix multiplication throws an exception");     // Failed if an exception has been thrown
            }

            // Try to compute an invalid multiplication
            try {
                Matrix matrix5(1,3);
                matrix1 * matrix5;                                      // This should throw an exception
                fail_("Invalid matrix substraction undetected");        // Failed if no exception have been thrown
            }
            catch(MathematicsException& ex) {
                succeed_();                                             // Succeed if an exception has been thrown
            }

            // Try to compute an invalid multiplication
            try {
                Matrix matrix5(2,2);
                matrix1 * matrix5;                                      // This should throw an exception
                fail_("Invalid matrix substraction undetected");        // Failed if no exception have been thrown
            }
            catch(MathematicsException& ex) {
                succeed_();                                             // Succeed if an exception has been thrown
            }
        }

        // Test operator=()
        void testOperatorAssignment() {

            // Try to compute a valid assignment
            try {
                Matrix matrix(2,3);
                matrix = matrix1;                                           // This shouldn't throw an exception
                test_(matrix.getValue(0, 0) == 4);
                test_(matrix.getValue(0, 2) == 7);
                test_(matrix.getValue(1, 0) == 2);
                test_(matrix.getValue(1, 2) == -4);
                test_(matrix.getValue(1, 1) == 3);
                succeed_();                                                 // Succeed if no exceptions have been thrown
            }
            catch(MathematicsException& ex) {
                fail_("Valid matrix assignment throws an exception");       // Failed if an exception has been thrown
            }

            // Try to compute an invalid assignment
            try {
                Matrix matrix(2,2);
                matrix = matrix1;                                       // This should throw an exception
                fail_("Invalid matrix assignment undetected");          // Failed if no exception have been thrown
            }
            catch(MathematicsException& ex) {
                succeed_();                                             // Succeed if an exception has been thrown
            }
        }

        // Test operator==()
        void testOperatorEquality() {


            // Try to test a valid equality
            try {
                Matrix matrix(2,3);
                matrix.setValue(0, 0, 4);
                matrix.setValue(0, 1, 5);
                matrix.setValue(0, 2, 7);
                matrix.setValue(1, 0, 2);
                matrix.setValue(1, 1, 3);
                matrix.setValue(1, 2, -4);
                test_(matrix == matrix1);                                   // This shouldn't throw an exception
                test_(matrix1 == matrix);                                   // This shouldn't throw an exception
                test_(matrix == matrix);                                    // This shouldn't throw an exception
                matrix.setValue(1,1, 5);
                test_(!(matrix == matrix1));                                // This shouldn't throw an exception
                test_(!(matrix1 == matrix));                                // This shouldn't throw an exception
                succeed_();                                                 // Succeed if no exceptions have been thrown
            }
            catch(MathematicsException& ex) {
                fail_("Valid matrix equality test throws an exception");    // Failed if an exception has been thrown
            }

            // Try to test a invalid equality
            try {
                Matrix matrix(2,2);
                matrix == matrix1;                                      // This should throw an exception
                fail_("Invalid matrix assignment undetected");          // Failed if no exception have been thrown
            }
            catch(MathematicsException& ex) {
                succeed_();                                             // Succeed if an exception has been thrown
            }
        }



};

#endif

