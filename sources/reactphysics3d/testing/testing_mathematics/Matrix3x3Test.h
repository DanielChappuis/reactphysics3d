
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

#ifndef MATRIX3DTEST_H
#define MATRIX3DTEST_H

// Libraries
#include "../TestSuite/Test.h"
#include "../../mathematics/Matrix3x3.h"
#include <stdexcept>
#include <iostream>
#include <cmath>

// Namespaces
using namespace reactphysics3d;

// Class MatrixTest
class Matrix3x3Test : public TestSuite::Test {
   private :
        Matrix3x3 matrix1;
        Matrix3x3 matrix2;
   public :

        // Constructor
        Matrix3x3Test() {
            matrix1.setAllValues(3, 7, -5, 13, -1, 2, 6, 5, 9);
            matrix2.setAllValues(-13, 8, 2, 5, -25, 11, -7, 6, 21);
        }

        // Run method of the Test
        void run() {
            testConstructors();
            testGetValue();
            testSetValue();
            testSetAllValues() ;
            testGetTranspose();
            testGetInverse();
            testGetDeterminant();
            testGetTrace();
            testGetQuaternion();
            testIdentityMatrix();
            testOperatorAddition();
            testOperatorSubstraction();
            testOperatorMultiplicationWithConstant();
            testOperatorMultiplicationWithMatrix();
            testOperatorMultiplicationWithVector();
            testOperatorAssignment();
            testOperatorEquality();
        }

        // Test the constructors
        void testConstructors() {

            // Constructor without argument
            Matrix3x3 matrix;                 // This shouldn't throw an exception
            test_(matrix.getValue(0,0) == 0);
            test_(matrix.getValue(1,2) == 0);
            test_(matrix.getValue(0,2) == 0);

            // Constructor with arguments
            Matrix3x3 matrix3(1, 2, 3, 4, 5, 6, 7, 8, 9);
            test_(matrix3.getValue(0,0) == 1);
            test_(matrix3.getValue(0,1) == 2);
            test_(matrix3.getValue(0,2) == 3);
            test_(matrix3.getValue(1,0) == 4);
            test_(matrix3.getValue(1,1) == 5);
            test_(matrix3.getValue(1,2) == 6);
            test_(matrix3.getValue(2,0) == 7);
            test_(matrix3.getValue(2,1) == 8);
            test_(matrix3.getValue(2,2) == 9);

            // Copy-constructor
            Matrix3x3 matrix4 = matrix3;
            test_(matrix4.getValue(0,0) == 1);
            test_(matrix4.getValue(0,1) == 2);
            test_(matrix4.getValue(0,2) == 3);
            test_(matrix4.getValue(1,0) == 4);
            test_(matrix4.getValue(1,1) == 5);
            test_(matrix4.getValue(1,2) == 6);
            test_(matrix4.getValue(2,0) == 7);
            test_(matrix4.getValue(2,1) == 8);
            test_(matrix4.getValue(2,2) == 9);

            // Conversion-constructor (Quaternion --> Matrix3x3)

            // Rotation matrix of a rotation of 180 degrees around x axis
            Matrix3x3 rotation1(1, 0, 0, 0, -1, 0, 0, 0, -1);
            Quaternion quaternion1(1, 0, 0, 0);
            Matrix3x3 matrix5(quaternion1);             // Convert the quaternion into a matrix
            test_(matrix5 == rotation1);                // Check if the matrix result and the rotation matrix are the same

            // Rotation matrix of a rotation of 180 degrees around y axis
            Matrix3x3 rotation2(-1, 0, 0, 0, 1, 0, 0, 0, -1);
            Quaternion quaternion2(0, 1, 0, 0);
            Matrix3x3 matrix6(quaternion2);             // Convert the quaternion into a matrix
            test_(matrix6 == rotation2);                // Check if the matrix result and the rotation matrix are the same

            // Rotation matrix of a rotation of 180 degrees around z axis
            Matrix3x3 rotation3(-1, 0, 0, 0, -1, 0, 0, 0, 1);
            Quaternion quaternion3(0, 0, 1, 0);
            Matrix3x3 matrix7(quaternion3);             // Convert the quaternion into a matrix
            test_(matrix7 == rotation3);                // Check if the matrix result and the rotation matrix are the same
        }

        // Test getValue()
        void testGetValue() {
            // Try a valid getValue()
            try {
                test_(matrix1.getValue(0, 0) == 3);                    // This shouldn't throw an exception
                test_(matrix1.getValue(1, 0) == 13);                   // This shouldn't throw an exception
                test_(matrix1.getValue(1, 2) == 2);                    // This shouldn't throw an exception
                test_(matrix1.getValue(1, 1) == -1);                   // This shouldn't throw an exception
                succeed_();                                            // Succeed if no exceptions have been thrown
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

            Matrix3x3 matrix;

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

        // Test setAllValues()
        void testSetAllValues() {
            Matrix3x3 matrix;
            matrix.setAllValues(1,2,3,4,5,6,7,8,9);
            test_(matrix.getValue(0,0) == 1);
            test_(matrix.getValue(0,1) == 2);
            test_(matrix.getValue(0,2) == 3);
            test_(matrix.getValue(1,0) == 4);
            test_(matrix.getValue(1,1) == 5);
            test_(matrix.getValue(1,2) == 6);
            test_(matrix.getValue(2,0) == 7);
            test_(matrix.getValue(2,1) == 8);
            test_(matrix.getValue(2,2) == 9);
        }

        // Test getTranspose()
        void testGetTranspose() {
            // Get the transpose of matrix1
            Matrix3x3 matrix = matrix1.getTranspose();

            // Test the transpose matrix
            test_(matrix.getValue(0, 0) == 3);
            test_(matrix.getValue(0, 1) == 13);
            test_(matrix.getValue(0, 2) == 6);
            test_(matrix.getValue(1, 0) == 7);
            test_(matrix.getValue(1, 1) == -1);
            test_(matrix.getValue(1, 2) == 5);
            test_(matrix.getValue(2, 0) == -5);
            test_(matrix.getValue(2, 1) == 2);
            test_(matrix.getValue(2, 2) == 9);
        }

        // Test getInverse()
        void testGetInverse() {

            // Construct a 3x3 matrix
            Matrix3x3 matrix(0, 1, 2, 1, 0, 3, 4, -3, 8);

            // Try to inverse a invertible matrix
            try {
                Matrix3x3 result = matrix.getInverse();                // This shouldn't thrown an exception
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

            // Try to inverse a square non-invertible matrix (determinant equal to zero)
            try {
                Matrix3x3 matrix4;
                matrix4.getInverse();                                                   // This should throw an exception
                fail_("Invalid getInverse() call undetected (non-invertible matrix)");  // Failed if no exception have been thrown
            }
            catch(MathematicsException& ex) {
                succeed_();                           // Succeed if an exception has been thrown
            }
        }

        // Test getDeterminant()
        void testGetDeterminant() {
            Matrix3x3 matrix;
            test_(matrix.getDeterminant() == 0);
            test_(matrix1.getDeterminant() == -1147);
            test_(matrix2.getDeterminant() == 5937);
        }

        // Test getTrace()
        void testGetTrace() {
            Matrix3x3 matrix;
            test_(matrix.getTrace() == 0);
            test_(matrix1.getTrace() == 11);
            test_(matrix2.getTrace() == -17);
        }

        // Test getQuaternion()
        void testGetQuaternion() {
            // Rotation matrix of a rotation of 180 degrees around x axis
            Matrix3x3 rotation1(1, 0, 0, 0, -1, 0, 0, 0, -1);

            // Convert the matrix into a quaternion
            Quaternion quaternion1 = rotation1.getQuaternion();
            test_(quaternion1.getX() == 1);
            test_(quaternion1.getY() == 0);
            test_(quaternion1.getZ() == 0);
            test_(quaternion1.getW() == 0);

            // Rotation matrix of a rotation of 180 degrees around y axis
            Matrix3x3 rotation2(-1, 0, 0, 0, 1, 0, 0, 0, -1);

            // Convert the matrix into a quaternion
            Quaternion quaternion2 = rotation2.getQuaternion();
            test_(quaternion2.getX() == 0);
            test_(quaternion2.getY() == 1);
            test_(quaternion2.getZ() == 0);
            test_(quaternion2.getW() == 0);

            // Rotation matrix of a rotation of 180 degrees around z axis
            Matrix3x3 rotation3(-1, 0, 0, 0, -1, 0, 0, 0, 1);

            // Convert the matrix into a quaternion
            Quaternion quaternion3 = rotation3.getQuaternion();
            test_(quaternion3.getX() == 0);
            test_(quaternion3.getY() == 0);
            test_(quaternion3.getZ() == 1);
            test_(quaternion3.getW() == 0);
        }

        // Test identityMatrix()
        void testIdentityMatrix() {
            Matrix3x3 matrix = Matrix3x3::identity();
            test_(matrix.getValue(0, 0) == 1);
            test_(matrix.getValue(0, 1) == 0);
            test_(matrix.getValue(0, 2) == 0);
            test_(matrix.getValue(1, 0) == 0);
            test_(matrix.getValue(1, 1) == 1);
            test_(matrix.getValue(1, 2) == 0);
            test_(matrix.getValue(2, 0) == 0);
            test_(matrix.getValue(2, 1) == 0);
            test_(matrix.getValue(2, 2) == 1);
        }

        // Test operator+()
        void testOperatorAddition() {
            Matrix3x3 result = matrix1 + matrix2;
            test_(result.getValue(0,0) == -10);
            test_(result.getValue(0,1) == 15);
            test_(result.getValue(0,2) == -3);
            test_(result.getValue(1,0) == 18);
            test_(result.getValue(1,1) == -26);
            test_(result.getValue(1,2) == 13);
            test_(result.getValue(2,0) == -1);
            test_(result.getValue(2,1) == 11);
            test_(result.getValue(2,2) == 30);
        }

        // Test operator-()
        void testOperatorSubstraction() {
            Matrix3x3 result = matrix1 - matrix2;
            test_(result.getValue(0,0) == 16);
            test_(result.getValue(0,1) == -1);
            test_(result.getValue(0,2) == -7);
            test_(result.getValue(1,0) == 8);
            test_(result.getValue(1,1) == 24);
            test_(result.getValue(1,2) == -9);
            test_(result.getValue(2,0) == 13);
            test_(result.getValue(2,1) == -1);
            test_(result.getValue(2,2) == -12);
        }

        // Test operator* (multiplication with a constant number)
        void testOperatorMultiplicationWithConstant() {
            Matrix3x3 result = matrix1 * 2;
            test_(result.getValue(0,0) == 6);
            test_(result.getValue(0,1) == 14);
            test_(result.getValue(0,2) == -10);
            test_(result.getValue(1,0) == 26);
            test_(result.getValue(1,1) == -2);
            test_(result.getValue(1,2) == 4);
            test_(result.getValue(2,0) == 12);
            test_(result.getValue(2,1) == 10);
            test_(result.getValue(2,2) == 18);
        }

        // Test operator* (multiplication with matrix)
        void testOperatorMultiplicationWithMatrix() {
            Matrix3x3 result = matrix1 * matrix2;
            test_(result.getValue(0,0) == 31);
            test_(result.getValue(0,1) == -181);
            test_(result.getValue(0,2) == -22);
            test_(result.getValue(1,0) == -188);
            test_(result.getValue(1,1) == 141);
            test_(result.getValue(1,2) == 57);
            test_(result.getValue(2,0) == -116);
            test_(result.getValue(2,1) == -23);
            test_(result.getValue(2,2) == 256);
        }

        void testOperatorMultiplicationWithVector() {
            Vector3D vector(4,7,3);
            Vector3D result = matrix1 * vector;
            test_(result.getX() == 46);
            test_(result.getY() == 51);
            test_(result.getZ() == 86);
        }

        // Test operator=()
        void testOperatorAssignment() {
            Matrix3x3 matrix;
            matrix = matrix1;
            test_(matrix.getValue(0,0) == 3);
            test_(matrix.getValue(0,1) == 7);
            test_(matrix.getValue(0,2) == -5);
            test_(matrix.getValue(1,0) == 13);
            test_(matrix.getValue(1,1) == -1);
            test_(matrix.getValue(1,2) == 2);
            test_(matrix.getValue(2,0) == 6);
            test_(matrix.getValue(2,1) == 5);
            test_(matrix.getValue(2,2) == 9);
        }

        // Test operator==()
        void testOperatorEquality() {
            Matrix3x3 matrix(3, 7, -5, 13, -1, 2, 6, 5, 9);
            test_(matrix == matrix1);
            test_(matrix1 == matrix);
            test_(matrix == matrix);
            matrix.setValue(1,1, 100);
            test_(!(matrix == matrix1));
            test_(!(matrix1 == matrix));
        }
};

#endif
