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

#ifndef TEST_QUATERNION_H
#define TEST_QUATERNION_H

// Libraries
#include "Test.h"
#include <reactphysics3d/mathematics/Quaternion.h>

/// Reactphysics3D namespace
namespace reactphysics3d {

// Class TestQuaternion
/**
 * Unit test for the Quaternion class
 */
class TestQuaternion : public Test {

    private :

        // ---------- Atributes ---------- //

        /// Identity Quaternion
        Quaternion mIdentity;

        /// First test quaternion
        Quaternion mQuaternion1;

    public :

        // ---------- Methods ---------- //

        /// Constructor
        TestQuaternion(const std::string& name) : Test(name), mIdentity(Quaternion::identity()) {

            decimal sinA = sin(decimal(PI/8.0));
            decimal cosA = cos(decimal(PI/8.0));
            Vector3 vector(2, 3, 4);
            vector.normalize();
            mQuaternion1 = Quaternion(vector.x * sinA, vector.y * sinA, vector.z * sinA, cosA);
            mQuaternion1.normalize();
        }

        /// Run the tests
        void run() {
            testConstructors();
            testUnitLengthNormalize();
            testOthersMethods();
            testOperators();
        }

        /// Test the constructors
        void testConstructors() {

            Quaternion quaternion1(mQuaternion1);
            rp3d_test(mQuaternion1 == quaternion1);

            Quaternion quaternion2(4, 5, 6, 7);
            rp3d_test(quaternion2 == Quaternion(4, 5, 6, 7));

            Quaternion quaternion3(8, Vector3(3, 5, 2));
            rp3d_test(quaternion3 == Quaternion(3, 5, 2, 8));

            Quaternion quaternion4(mQuaternion1.getMatrix());
            rp3d_test(approxEqual(quaternion4.x, mQuaternion1.x));
            rp3d_test(approxEqual(quaternion4.y, mQuaternion1.y));
            rp3d_test(approxEqual(quaternion4.z, mQuaternion1.z));
            rp3d_test(approxEqual(quaternion4.w, mQuaternion1.w));

            // Test conversion from Euler angles to quaternion

            const decimal PI_OVER_2 = PI * decimal(0.5);
            const decimal PI_OVER_4 = PI_OVER_2 * decimal(0.5);
            Quaternion quaternion5 = Quaternion::fromEulerAngles(PI_OVER_2, 0, 0);
            Quaternion quaternionTest5(std::sin(PI_OVER_4), 0, 0, std::cos(PI_OVER_4));
            quaternionTest5.normalize();
            rp3d_test(approxEqual(quaternion5.x, quaternionTest5.x));
            rp3d_test(approxEqual(quaternion5.y, quaternionTest5.y));
            rp3d_test(approxEqual(quaternion5.z, quaternionTest5.z));
            rp3d_test(approxEqual(quaternion5.w, quaternionTest5.w));

            Quaternion quaternion6 = Quaternion::fromEulerAngles(0, PI_OVER_2, 0);
            Quaternion quaternionTest6(0, std::sin(PI_OVER_4), 0, std::cos(PI_OVER_4));
            quaternionTest6.normalize();
            rp3d_test(approxEqual(quaternion6.x, quaternionTest6.x));
            rp3d_test(approxEqual(quaternion6.y, quaternionTest6.y));
            rp3d_test(approxEqual(quaternion6.z, quaternionTest6.z));
            rp3d_test(approxEqual(quaternion6.w, quaternionTest6.w));

            Quaternion quaternion7 = Quaternion::fromEulerAngles(Vector3(0, 0, PI_OVER_2));
            Quaternion quaternionTest7(0, 0, std::sin(PI_OVER_4), std::cos(PI_OVER_4));
            quaternionTest7.normalize();
            rp3d_test(approxEqual(quaternion7.x, quaternionTest7.x));
            rp3d_test(approxEqual(quaternion7.y, quaternionTest7.y));
            rp3d_test(approxEqual(quaternion7.z, quaternionTest7.z));
            rp3d_test(approxEqual(quaternion7.w, quaternionTest7.w));
        }

        /// Test unit, length, normalize methods
        void testUnitLengthNormalize() {

            // Test method that returns the length
            Quaternion quaternion(2, 3, -4, 5);
            rp3d_test(approxEqual(quaternion.length(), std::sqrt(decimal(54.0))));

            // Test method that returns a unit quaternion
            rp3d_test(approxEqual(quaternion.getUnit().length(), 1.0));

            // Test the normalization method
            Quaternion quaternion2(4, 5, 6, 7);
            quaternion2.normalize();
            rp3d_test(approxEqual(quaternion2.length(), 1.0));
        }

        /// Test others methods
        void testOthersMethods() {

            // Test the method to set the values
            Quaternion quaternion;
            quaternion.setAllValues(1, 2, 3, 4);
            rp3d_test(quaternion == Quaternion(1, 2, 3, 4));

            // Test the method to set the quaternion to zero
            quaternion.setToZero();
            rp3d_test(quaternion == Quaternion(0, 0, 0, 0));

            // Tes the methods to get or set to identity
            Quaternion identity1(1, 2, 3, 4);
            identity1.setToIdentity();
            rp3d_test(identity1 == Quaternion(0, 0, 0, 1));
            rp3d_test(Quaternion::identity() == Quaternion(0, 0, 0, 1));

            // Test the method to get the vector (x, y, z)
            Vector3 v = mQuaternion1.getVectorV();
            rp3d_test(v.x == mQuaternion1.x);
            rp3d_test(v.y == mQuaternion1.y);
            rp3d_test(v.z == mQuaternion1.z);

            // Test the conjugate method
            Quaternion conjugate = mQuaternion1.getConjugate();
            rp3d_test(conjugate.x == -mQuaternion1.x);
            rp3d_test(conjugate.y == -mQuaternion1.y);
            rp3d_test(conjugate.z == -mQuaternion1.z);
            rp3d_test(conjugate.w == mQuaternion1.w);

            // Test the inverse methods
            Quaternion inverse1 = mQuaternion1.getInverse();
            Quaternion inverse2(mQuaternion1);
            inverse2.inverse();
            rp3d_test(inverse2 == inverse1);
            Quaternion product = mQuaternion1 * inverse1;
            rp3d_test(approxEqual(product.x, mIdentity.x, decimal(10e-6)));
            rp3d_test(approxEqual(product.y, mIdentity.y, decimal(10e-6)));
            rp3d_test(approxEqual(product.z, mIdentity.z, decimal(10e-6)));
            rp3d_test(approxEqual(product.w, mIdentity.w, decimal(10e-6)));

            // Test the dot product
            Quaternion quaternion1(2, 3, 4, 5);
            Quaternion quaternion2(6, 7, 8, 9);
            decimal dotProduct = quaternion1.dot(quaternion2);
            rp3d_test(dotProduct == 110);

            // Test the method that returns the rotation angle and axis
            Vector3 axis;
            decimal angle;
            Vector3 originalAxis = Vector3(2, 3, 4).getUnit();
            mQuaternion1.getRotationAngleAxis(angle, axis);
            rp3d_test(approxEqual(axis.x, originalAxis.x));
            rp3d_test(approxEqual(angle, decimal(PI/4.0), decimal(10e-6)));

            // Test the method that returns the corresponding matrix
            Matrix3x3 matrix = mQuaternion1.getMatrix();
            Vector3 vector(56, -2, 82);
            Vector3 vector1 = matrix * vector;
            Vector3 vector2 = mQuaternion1 * vector;
            rp3d_test(approxEqual(vector1.x, vector2.x, decimal(10e-6)));
            rp3d_test(approxEqual(vector1.y, vector2.y, decimal(10e-6)));
            rp3d_test(approxEqual(vector1.z, vector2.z, decimal(10e-6)));

            // Test slerp method
            Quaternion quatStart = quaternion1.getUnit();
            Quaternion quatEnd = quaternion2.getUnit();
            Quaternion test1 = Quaternion::slerp(quatStart, quatEnd, 0.0);
            Quaternion test2 = Quaternion::slerp(quatStart, quatEnd, 1.0);
            rp3d_test(test1 == quatStart);
            rp3d_test(test2 == quatEnd);
            decimal sinA = sin(decimal(PI/4.0));
            decimal cosA = cos(decimal(PI/4.0));
            Quaternion quat(sinA, 0, 0, cosA);
            Quaternion test3 = Quaternion::slerp(mIdentity, quat, decimal(0.5));
            rp3d_test(approxEqual(test3.x, sin(decimal(PI/8.0))));
            rp3d_test(approxEqual(test3.y, 0.0));
            rp3d_test(approxEqual(test3.z, 0.0));
            rp3d_test(approxEqual(test3.w, cos(decimal(PI/8.0)), decimal(10e-6)));
        }

        /// Test overloaded operators
        void testOperators() {

            // Test addition
            Quaternion quat1(4, 5, 2, 10);
            Quaternion quat2(-2, 7, 8, 3);
            Quaternion test1 = quat1 + quat2;
            Quaternion test11(-6, 52, 2, 8);
            test11 += quat1;
            rp3d_test(test1 == Quaternion(2, 12, 10, 13));
            rp3d_test(test11 == Quaternion(-2, 57, 4, 18));

            // Test substraction
            Quaternion test2 = quat1 - quat2;
            Quaternion test22(-73, 62, 25, 9);
            test22 -= quat1;
            rp3d_test(test2 == Quaternion(6, -2, -6, 7));
            rp3d_test(test22 == Quaternion(-77, 57, 23, -1));

            // Test multiplication with a number
            Quaternion test3 = quat1 * 3.0;
            rp3d_test(test3 == Quaternion(12, 15, 6, 30));

            // Test multiplication between two quaternions
            Quaternion test4 = quat1 * quat2;
            Quaternion test5 = mQuaternion1 * mIdentity;
            rp3d_test(test4 == Quaternion(18, 49, 124, -13));
            rp3d_test(test5 == mQuaternion1);

            // Test multiplication between a quaternion and a point
            Vector3 point(5, -24, 563);
            Vector3 vector1 = mIdentity * point;
            Vector3 vector2 = mQuaternion1 * point;
            Vector3 testVector2 = mQuaternion1.getMatrix() * point;
            rp3d_test(vector1 == point);
            rp3d_test(approxEqual(vector2.x, testVector2.x, decimal(10e-5)));
            rp3d_test(approxEqual(vector2.y, testVector2.y, decimal(10e-5)));
            rp3d_test(approxEqual(vector2.z, testVector2.z, decimal(10e-5)));

            // Test assignment operator
            Quaternion quaternion;
            quaternion = mQuaternion1;
            rp3d_test(quaternion == mQuaternion1);

            // Test equality operator
            rp3d_test(mQuaternion1 == mQuaternion1);
        }
 };

}

#endif
