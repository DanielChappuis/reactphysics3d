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

#ifndef VECTOR3DTEST_H
#define VECTOR3DTEST_H

// Libraries
#include "../TestSuite/Test.h"
#include "../../mathematics/Vector3D.h"
#include <stdexcept>

// Namespaces
using namespace reactphysics3d;

// Class Vector3DTest
class Vector3DTest : public TestSuite::Test {
   private :
        Vector3D vector1;
        Vector3D vector2;
   public :

        // Constructor
        Vector3DTest() : vector1(1,2,3), vector2(-3,5,7) {

        }

        // Run method of the Test
        void run() {
            testConstructors();
            testGetX();
            testGetY();
            testGetZ();
            testSetX();
            testSetY();
            testSetZ();
            testSetAllValues();
            testLength();
            testGetUnit();
            testScalarProduct();
            testCrossProduct();
            testOperatorAddition();
            testOperatorSubstraction();
            testOperatorConstantMultiplications();
            testOperatorAssignment();
        }

        // Test the constructors
        void testConstructors() {

            // Constructor without arguments
            Vector3D vector;
            test_(vector.getX() == 0);
            test_(vector.getY() == 0);
            test_(vector.getZ() == 0);

            // Constructor with arguments
            Vector3D vector3(4, -67, 21);
            test_(vector3.getX() == 4);
            test_(vector3.getY() == -67);
            test_(vector3.getZ() == 21);

            // Copy-constructor
            Vector3D vector4 = vector1;
            test_(vector4.getX() == 1);
            test_(vector4.getY() == 2);
            test_(vector4.getZ() == 3);
        }

        // Test getX()
        void testGetX() {
            test_(vector1.getX() == 1);
        }

        // Test getY()
        void testGetY() {
            test_(vector1.getY() == 2);
        }

        // Test getZ()
        void testGetZ() {
            test_(vector1.getZ() == 3);
        }

        // Test setX()
        void testSetX() {
            Vector3D vector(5, 6, 7);
            vector.setX(8);
            test_(vector.getX() == 8);
            test_(vector.getY() == 6);
            test_(vector.getZ() == 7);
        }

        // Test setY()
        void testSetY() {
            Vector3D vector(5, 6, 7);
            vector.setY(8);
            test_(vector.getX() == 5);
            test_(vector.getY() == 8);
            test_(vector.getZ() == 7);
        }

        // Test setZ()
        void testSetZ() {
            Vector3D vector(5, 6, 7);
            vector.setZ(8);
            test_(vector.getX() == 5);
            test_(vector.getY() == 6);
            test_(vector.getZ() == 8);
        }


        // Test setAllValues()
        void testSetAllValues() {
            Vector3D vector(5, 6, 7);
            vector1.setAllValues(4,3,9);
            test_(vector1.getX() == 4);
            test_(vector1.getY() == 3);
            test_(vector1.getZ() == 9);
        }

        // Test length()
        void testLength() {
            Vector3D vector3;
            test_(vector3.length() == 0);
            vector3.setAllValues(3, 4, 0);
            test_(vector3.length() == 5);
            vector3.setAllValues(0, -3, 4);
            test_(vector3.length() == 5);
        }

        // Test getUnit()
        void testGetUnit() {

            Vector3D vector3(-23, 0, 0);
            test_(vector3.getUnit().length() == 1);
            test_(vector3.getUnit().getX() == -1);
            test_(vector3.getUnit().getY() == 0);
            test_(vector3.getUnit().getZ() == 0);

            vector3.setAllValues(0, 6, 0);
            test_(vector3.getUnit().length() == 1);
            test_(vector3.getUnit().getX() == 0);
            test_(vector3.getUnit().getY() == 1);
            test_(vector3.getUnit().getZ() == 0);

            vector3.setAllValues(0, 0, 13);
            test_(vector3.getUnit().length() == 1);
            test_(vector3.getUnit().getX() == 0);
            test_(vector3.getUnit().getY() == 0);
            test_(vector3.getUnit().getZ() == 1);

            vector3.setAllValues(0, 0, 0);                                             // Vector of length equal to zero
            try {
                vector3.getUnit();                                                      // This should throw an exception
                fail_("getUnit() with a vector of length equals to zero undetected");   // The test failed if no exception have been thrown
            }
            catch(MathematicsException& ex) {
                succeed_();                                                             // The test succeed if an exception has been thrown
            }
        }

        // Test scalarProduct()
        void testScalarProduct() {
            Vector3D vector3(2, -3, 5);
            Vector3D vector4(7, 4, 6);

            // Test the scalar product result
            test_(vector3.scalarProduct(vector4) == 32);
        }

        // Test crossProduct()
        void testCrossProduct() {
            Vector3D vector3(4, -5, 2);
            Vector3D vector4(3, 2, 6);

            // Compute the cross product
            Vector3D result = vector3.crossProduct(vector4);

            // Test the result
            test_(result.getX() == -34);
            test_(result.getY() == -18);
            test_(result.getZ() == 23);
        }

        // Test operator+()
        void testOperatorAddition() {
            Vector3D vector3(4, -5, 2);
            Vector3D vector4(3, 2, 6);

            // Compute the sum
            Vector3D result = vector3 + vector4;

            // Test the result
            test_(result.getX() == 7);
            test_(result.getY() == -3);
            test_(result.getZ() == 8);
        }

        // Test operator-()
        void testOperatorSubstraction() {
            Vector3D vector3(4, -5, 2);
            Vector3D vector4(3, 2, 6);

            // Compute the substraction
            Vector3D result = vector3 - vector4;

            // Test the result
            test_(result.getX() == 1);
            test_(result.getY() == -7);
            test_(result.getZ() == -4);
        }

        // Test operator*() (with a constant number)
        void testOperatorConstantMultiplications() {
            Vector3D vector3(4, -5, 2);

            // Compute the multiplication
            Vector3D result = vector3 * 5;

            // Test the result
            test_(result.getX() == 20);
            test_(result.getY() == -25);
            test_(result.getZ() == 10);
        }

        // Test operator=()
        void testOperatorAssignment() {
            Vector3D vector3(4, -5, 2);

            // Assignment
            Vector3D result;
            result = vector3;

            // Test the result
            test_(result.getX() == 4);
            test_(result.getY() == -5);
            test_(result.getZ() == 2);
        }

        // Test operator==()
        void testOperatorEquality() {
          Vector3D vector3(4, -5, 2);
            Vector3D vector4(4, -5, 2);
            Vector3D vector5(3, -5, -2);

            // Test the equality
            test_(vector3 == vector4);
            test_(vector4 == vector3);
            test_(vector3 == vector3);
            test_(!(vector3 == vector5));
            test_(!(vector5 == vector3));
        }
};

#endif

