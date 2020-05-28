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

#ifndef TEST_VECTOR2_H
#define TEST_VECTOR2_H

// Libraries
#include "Test.h"
#include <reactphysics3d/mathematics/Vector2.h>

/// Reactphysics3D namespace
namespace reactphysics3d {

// Class TestVector2
/**
 * Unit test for the Vector2 class
 */
class TestVector2 : public Test {

    private :

        // ---------- Atributes ---------- //

        /// Zero vector
        Vector2 mVectorZero;

        // Vector (3, 4)
        Vector2 mVector34;

    public :

        // ---------- Methods ---------- //

        /// Constructor
        TestVector2(const std::string& name) : Test(name), mVectorZero(0, 0), mVector34(3, 4) {}

        /// Run the tests
        void run() {
            testConstructors();
            testLengthMethods();
            testDotProduct();
            testOthersMethods();
            testOperators();
        }

        /// Test the constructors, getter and setter
        void testConstructors() {

            // Test constructor
            rp3d_test(mVectorZero.x == 0.0);
            rp3d_test(mVectorZero.y == 0.0);
            rp3d_test(mVector34.x == 3.0);
            rp3d_test(mVector34.y == 4.0);

            // Test copy-constructor
            Vector2 newVector(mVector34);
            rp3d_test(newVector.x == 3.0);
            rp3d_test(newVector.y == 4.0);

            // Test method to set values
            Vector2 newVector2;
            newVector2.setAllValues(decimal(6.1), decimal(7.2));
            rp3d_test(approxEqual(newVector2.x, decimal(6.1)));
            rp3d_test(approxEqual(newVector2.y, decimal(7.2)));

            // Test method to set to zero
            newVector2.setToZero();
            rp3d_test(newVector2 == Vector2(0, 0));
        }

        /// Test the length, unit vector and normalize methods
        void testLengthMethods() {

            // Test length methods
            rp3d_test(mVectorZero.length() == 0.0);
            rp3d_test(mVectorZero.lengthSquare() == 0.0);
            rp3d_test(Vector2(1, 0).length() == 1.0);
            rp3d_test(Vector2(0, 1).length() == 1.0);
            rp3d_test(mVector34.lengthSquare() == 25.0);

            // Test unit vector methods
            rp3d_test(Vector2(1, 0).isUnit());
            rp3d_test(Vector2(0, 1).isUnit());
            rp3d_test(!mVector34.isUnit());
            rp3d_test(Vector2(5, 0).getUnit() == Vector2(1, 0));
            rp3d_test(Vector2(0, 5).getUnit() == Vector2(0, 1));

            rp3d_test(!mVector34.isZero());
            rp3d_test(mVectorZero.isZero());

            // Test normalization method
            Vector2 mVector10(1, 0);
            Vector2 mVector01(0, 1);
            Vector2 mVector50(5, 0);
            Vector2 mVector05(0, 5);
            mVector10.normalize();
            mVector01.normalize();
            mVector50.normalize();
            mVector05.normalize();
            rp3d_test(mVector10 == Vector2(1, 0));
            rp3d_test(mVector01 == Vector2(0, 1));
            rp3d_test(mVector50 == Vector2(1, 0));
            rp3d_test(mVector05 == Vector2(0, 1));
        }

        /// Test the dot product
        void testDotProduct() {

            // Test the dot product
            rp3d_test(Vector2(5, 0).dot(Vector2(0, 8)) == 0);
            rp3d_test(Vector2(5, 8).dot(Vector2(0, 0)) == 0);
            rp3d_test(Vector2(12, 45).dot(Vector2(0, 0)) == 0);
            rp3d_test(Vector2(5, 7).dot(Vector2(5, 7)) == 74);
            rp3d_test(Vector2(3, 6).dot(Vector2(-3, -6)) == -45);
            rp3d_test(Vector2(2, 3).dot(Vector2(-7, 4)) == -2);
            rp3d_test(Vector2(4, 3).dot(Vector2(8, 9)) == 59);
        }

        /// Test others methods
        void testOthersMethods() {

            // Test the method that returns the absolute vector
            rp3d_test(Vector2(4, 5).getAbsoluteVector() == Vector2(4, 5));
            rp3d_test(Vector2(-7, -24).getAbsoluteVector() == Vector2(7, 24));

            // Test the method that returns the minimal element
            rp3d_test(Vector2(6, 35).getMinAxis() == 0);
            rp3d_test(Vector2(564, 45).getMinAxis() == 1);
            rp3d_test(Vector2(98, 23).getMinAxis() == 1);
            rp3d_test(Vector2(-53, -25).getMinAxis() == 0);

            // Test the method that returns the maximal element
            rp3d_test(Vector2(6, 35).getMaxAxis() == 1);
            rp3d_test(Vector2(7, 537).getMaxAxis() == 1);
            rp3d_test(Vector2(98, 23).getMaxAxis() == 0);
            rp3d_test(Vector2(-53, -25).getMaxAxis() == 1);

            // Test the methot that return a max/min vector
            Vector2 vec1(-5, 4);
            Vector2 vec2(-8, 6);
            rp3d_test(Vector2::min(vec1, vec2) == Vector2(-8, 4));
            rp3d_test(Vector2::max(vec1, vec2) == Vector2(-5, 6));
        }

        /// Test the operators
        void testOperators() {

            // Test the [] operator
            rp3d_test(mVector34[0] == 3);
            rp3d_test(mVector34[1] == 4);

            // Assignment operator
            Vector2 newVector(6, 4);
            newVector = Vector2(7, 8);
            rp3d_test(newVector == Vector2(7, 8));

            // Equality, inequality operators
            rp3d_test(Vector2(5, 7) == Vector2(5, 7));
            rp3d_test(Vector2(63, 64) != Vector2(63, 84));
            rp3d_test(Vector2(63, 64) != Vector2(12, 64));

            // Addition, substraction
            Vector2 vector1(6, 33);
            Vector2 vector2(7, 68);
            rp3d_test(Vector2(63, 24) + Vector2(3, 4) == Vector2(66, 28));
            rp3d_test(Vector2(63, 24) - Vector2(3, 4) == Vector2(60, 20));
            vector1 += Vector2(5, 10);
            vector2 -= Vector2(10, 21);
            rp3d_test(vector1 == Vector2(11, 43));
            rp3d_test(vector2 == Vector2(-3, 47));

            // Multiplication, division
            Vector2 vector3(6, 33);
            Vector2 vector4(15, 60);
            rp3d_test(Vector2(63, 24) * 3 == Vector2(189, 72));
            rp3d_test(3 * Vector2(63, 24) == Vector2(189, 72));
            rp3d_test(Vector2(14, 8) / 2 == Vector2(7, 4));
            vector3 *= 10;
            vector4 /= 3;
            rp3d_test(vector3 == Vector2(60, 330));
            rp3d_test(vector4 == Vector2(5, 20));
            Vector2 vector5(21, 80);
            Vector2 vector6(7, 10);
            Vector2 vector7 = vector5 * vector6;
            rp3d_test(vector7 == Vector2(147, 800));
            Vector2 vector8 = vector5 / vector6;
            rp3d_test(approxEqual(vector8.x, 3));
            rp3d_test(approxEqual(vector8.y, 8));

            // Negative operator
            Vector2 vector9(-34, 5);
            Vector2 negative = -vector9;
            rp3d_test(negative == Vector2(34, -5));
        }
 };

}

#endif
