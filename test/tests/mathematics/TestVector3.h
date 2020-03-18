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

#ifndef TEST_VECTOR3_H
#define TEST_VECTOR3_H

// Libraries
#include "Test.h"
#include <reactphysics3d/mathematics/Vector3.h>

/// Reactphysics3D namespace
namespace reactphysics3d {

// Class TestVector3
/**
 * Unit test for the Vector3 class
 */
class TestVector3 : public Test {

    private :

        // ---------- Atributes ---------- //

        /// Zero vector
        Vector3 mVectorZero;

        // Vector (3, 4, 5)
        Vector3 mVector345;

    public :

        // ---------- Methods ---------- //

        /// Constructor
        TestVector3(const std::string& name): Test(name),mVectorZero(0, 0, 0),mVector345(3, 4, 5) {}

        /// Run the tests
        void run() {
            testConstructors();
            testLengthMethods();
            testDotCrossProducts();
            testOthersMethods();
            testOperators();
        }

        /// Test the constructors, getter and setter
        void testConstructors() {

            // Test constructor
            rp3d_test(mVectorZero.x == 0.0);
            rp3d_test(mVectorZero.y == 0.0);
            rp3d_test(mVectorZero.z == 0.0);
            rp3d_test(mVector345.x == 3.0);
            rp3d_test(mVector345.y == 4.0);
            rp3d_test(mVector345.z == 5.0);

            // Test copy-constructor
            Vector3 newVector(mVector345);
            rp3d_test(newVector.x == 3.0);
            rp3d_test(newVector.y == 4.0);
            rp3d_test(newVector.z == 5.0);

            // Test method to set values
            Vector3 newVector2;
            newVector2.setAllValues(decimal(6.1), decimal(7.2), decimal(8.6));
            rp3d_test(approxEqual(newVector2.x, decimal(6.1)));
            rp3d_test(approxEqual(newVector2.y, decimal(7.2)));
            rp3d_test(approxEqual(newVector2.z, decimal(8.6)));

            // Test method to set to zero
            newVector2.setToZero();
            rp3d_test(newVector2 == Vector3(0, 0, 0));
        }

        /// Test the length, unit vector and normalize methods
        void testLengthMethods() {

            // Test length methods
            rp3d_test(mVectorZero.length() == 0.0);
            rp3d_test(mVectorZero.lengthSquare() == 0.0);
            rp3d_test(Vector3(1, 0, 0).length() == 1.0);
            rp3d_test(Vector3(0, 1, 0).length() == 1.0);
            rp3d_test(Vector3(0, 0, 1).length() == 1.0);
            rp3d_test(mVector345.lengthSquare() == 50.0);

            // Test unit vector methods
            rp3d_test(Vector3(1, 0, 0).isUnit());
            rp3d_test(Vector3(0, 1, 0).isUnit());
            rp3d_test(Vector3(0, 0, 1).isUnit());
            rp3d_test(!mVector345.isUnit());
            rp3d_test(Vector3(5, 0, 0).getUnit() == Vector3(1, 0, 0));
            rp3d_test(Vector3(0, 5, 0).getUnit() == Vector3(0, 1, 0));
            rp3d_test(Vector3(0, 0, 5).getUnit() == Vector3(0, 0, 1));

            rp3d_test(!mVector345.isZero());
            rp3d_test(mVectorZero.isZero());

            // Test normalization method
            Vector3 mVector100(1, 0, 0);
            Vector3 mVector010(0, 1, 0);
            Vector3 mVector001(0, 0, 1);
            Vector3 mVector500(5, 0, 0);
            Vector3 mVector050(0, 5, 0);
            Vector3 mVector005(0, 0, 5);
            mVector100.normalize();
            mVector010.normalize();
            mVector001.normalize();
            mVector500.normalize();
            mVector050.normalize();
            mVector005.normalize();
            rp3d_test(mVector100 == Vector3(1, 0, 0));
            rp3d_test(mVector010 == Vector3(0, 1, 0));
            rp3d_test(mVector001 == Vector3(0, 0, 1));
            rp3d_test(mVector500 == Vector3(1, 0, 0));
            rp3d_test(mVector050 == Vector3(0, 1, 0));
            rp3d_test(mVector005 == Vector3(0, 0, 1));
        }

        /// Test the dot and cross products
        void testDotCrossProducts() {

            // Test the dot product
            rp3d_test(Vector3(5, 0, 0).dot(Vector3(0, 8, 0)) == 0);
            rp3d_test(Vector3(5, 8, 0).dot(Vector3(0, 0, 6)) == 0);
            rp3d_test(Vector3(12, 45, 83).dot(Vector3(0, 0, 0)) == 0);
            rp3d_test(Vector3(5, 7, 8).dot(Vector3(5, 7, 8)) == 138);
            rp3d_test(Vector3(3, 6, 78).dot(Vector3(-3, -6, -78)) == -6129);
            rp3d_test(Vector3(2, 3, 5).dot(Vector3(2, 3, 5)) == 38);
            rp3d_test(Vector3(4, 3, 2).dot(Vector3(8, 9, 10)) == 79);

            // Test the cross product
            rp3d_test(Vector3(0, 0, 0).cross(Vector3(0, 0, 0)) == Vector3(0, 0, 0));
            rp3d_test(Vector3(6, 7, 2).cross(Vector3(6, 7, 2)) == Vector3(0, 0, 0));
            rp3d_test(Vector3(1, 0, 0).cross(Vector3(0, 1, 0)) == Vector3(0, 0, 1));
            rp3d_test(Vector3(0, 1, 0).cross(Vector3(0, 0, 1)) == Vector3(1, 0, 0));
            rp3d_test(Vector3(0, 0, 1).cross(Vector3(0, 1, 0)) == Vector3(-1, 0, 0));
            rp3d_test(Vector3(4, 7, 24).cross(Vector3(8, 13, 11)) == Vector3(-235, 148, -4));
            rp3d_test(Vector3(-4, 42, -2).cross(Vector3(35, 7, -21)) == Vector3(-868, -154, -1498));
        }

        /// Test others methods
        void testOthersMethods() {

            // Test the method that returns the absolute vector
            rp3d_test(Vector3(4, 5, 6).getAbsoluteVector() == Vector3(4, 5, 6));
            rp3d_test(Vector3(-7, -24, -12).getAbsoluteVector() == Vector3(7, 24, 12));

            // Test the method that returns the minimal element
            rp3d_test(Vector3(6, 35, 82).getMinAxis() == 0);
            rp3d_test(Vector3(564, 45, 532).getMinAxis() == 1);
            rp3d_test(Vector3(98, 23, 3).getMinAxis() == 2);
            rp3d_test(Vector3(-53, -25, -63).getMinAxis() == 2);

            // Test the method that returns the maximal element
            rp3d_test(Vector3(6, 35, 82).getMaxAxis() == 2);
            rp3d_test(Vector3(7, 533, 36).getMaxAxis() == 1);
            rp3d_test(Vector3(98, 23, 3).getMaxAxis() == 0);
            rp3d_test(Vector3(-53, -25, -63).getMaxAxis() == 1);

            // Test the methot that return a max/min vector
            Vector3 vec1(-5, 4, 2);
            Vector3 vec2(-8, 6, -1);
            rp3d_test(Vector3::min(vec1, vec2) == Vector3(-8, 4, -1));
            rp3d_test(Vector3::max(vec1, vec2) == Vector3(-5, 6, 2));
        }

        /// Test the operators
        void testOperators() {

            // Test the [] operator
            rp3d_test(mVector345[0] == 3);
            rp3d_test(mVector345[1] == 4);
            rp3d_test(mVector345[2] == 5);

            // Assignment operator
            Vector3 newVector(6, 4, 2);
            newVector = Vector3(7, 8, 9);
            rp3d_test(newVector == Vector3(7, 8, 9));

            // Equality, inequality operators
            rp3d_test(Vector3(5, 7, 3) == Vector3(5, 7, 3));
            rp3d_test(Vector3(63, 64, 24) != Vector3(63, 64, 5));
            rp3d_test(Vector3(63, 64, 24) != Vector3(12, 64, 24));
            rp3d_test(Vector3(63, 64, 24) != Vector3(63, 8, 24));

            // Addition, substraction
            Vector3 vector1(6, 33, 62);
            Vector3 vector2(7, 68, 35);
            rp3d_test(Vector3(63, 24, 5) + Vector3(3, 4, 2) == Vector3(66, 28, 7));
            rp3d_test(Vector3(63, 24, 5) - Vector3(3, 4, 2) == Vector3(60, 20, 3));
            vector1 += Vector3(5, 10, 12);
            vector2 -= Vector3(10, 21, 5);
            rp3d_test(vector1 == Vector3(11, 43, 74));
            rp3d_test(vector2 == Vector3(-3, 47, 30));

            // Multiplication, division
            Vector3 vector3(6, 33, 62);
            Vector3 vector4(15, 60, 33);
            rp3d_test(Vector3(63, 24, 5) * 3 == Vector3(189, 72, 15));
            rp3d_test(3 * Vector3(63, 24, 5) == Vector3(189, 72, 15));
            rp3d_test(Vector3(14, 8, 50) / 2 == Vector3(7, 4, 25));
            vector3 *= 10;
            vector4 /= 3;
            rp3d_test(vector3 == Vector3(60, 330, 620));
            rp3d_test(vector4 == Vector3(5, 20, 11));
            Vector3 vector5(21, 80, 45);
            Vector3 vector6(7, 10, 3);
            Vector3 vector7 = vector5 * vector6;
            rp3d_test(vector7 == Vector3(147, 800, 135));
            Vector3 vector8 = vector5 / vector6;
            rp3d_test(approxEqual(vector8.x, 3));
            rp3d_test(approxEqual(vector8.y, 8));
            rp3d_test(approxEqual(vector8.z, 15));

            // Negative operator
            Vector3 vector9(-34, 5, 422);
            Vector3 negative = -vector9;
            rp3d_test(negative == Vector3(34, -5, -422));
        }
 };

}

#endif
