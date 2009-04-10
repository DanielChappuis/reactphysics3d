
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

#ifndef QUATERNIONTEST_H
#define QUATERNIONTEST_H

// Libraries
#include "../TestSuite/Test.h"
#include "../../mathematics/Quaternion.h"
#include "../../mathematics/constants.h"
#include <stdexcept>
#include <iostream>

// Namespaces
using namespace reactphysics3d;

// Class MatrixTest
class QuaternionTest : public TestSuite::Test {
   private :
        Quaternion quaternion1;
        Quaternion quaternion2;
   public :

        // Constructor
        QuaternionTest() : quaternion1(2,3,4,5), quaternion2(6,7,8,9) {

        }

        // Run method of the Test
        void run() {
            testConstructors();
            testVectorV();
            testLength();
            testGetUnit();
            testGetConjugate();
            testGetInverse();
            testScalarProduct();
            testgetRotationAngleAxis();
            testSlerp();
            testOperatorAddition();
            testOperatorSubstraction();
            testOperatorMultiplicationWithConstant();
            testOperatorMultiplicationWithQuaternion();
            testOperatorAssignment();
            testOperatorEquality();
        }

        // Test the constructors
        void testConstructors() {
            // Constructor without argument
            Quaternion quaternion;
            test_(quaternion.getX() == 0);
            test_(quaternion.getY() == 0);
            test_(quaternion.getZ() == 0);
            test_(quaternion.getW() == 0);

            // Constructor with argument
            Quaternion quaternion3(1,2,3,4);
            test_(quaternion3.getX() == 1);
            test_(quaternion3.getY() == 2);
            test_(quaternion3.getZ() == 3);
            test_(quaternion3.getW() == 4);

            // Constructor with vector
            Vector3D vector(2,3,4);
            Quaternion quaternion4(5, vector);
            test_(quaternion4.getX() == 2);
            test_(quaternion4.getY() == 3);
            test_(quaternion4.getZ() == 4);
            test_(quaternion4.getW() == 5);

            // Copy-constructor
            Quaternion quaternion5 = quaternion3;
            test_(quaternion5.getX() == 1);
            test_(quaternion5.getY() == 2);
            test_(quaternion5.getZ() == 3);
            test_(quaternion5.getW() == 4);
        }

        // Test getX()
        void testGetX() {
            test_(quaternion1.getX() == 2);
            test_(quaternion2.getX() == 6);
        }

        // Test getY()
        void testGetY() {
            test_(quaternion1.getY() == 3);
            test_(quaternion2.getY() == 7);
        }

        // Test getZ()
        void testGetZ() {
            test_(quaternion1.getZ() == 4);
            test_(quaternion2.getZ() == 8);
        }

        // Test getW()
        void testGetW() {
            test_(quaternion1.getW() == 5);
            test_(quaternion2.getW() == 9);
        }

        // Test setX()
        void testSetX() {
            Quaternion quaternion;
            quaternion.setX(3);
            test_(quaternion.getX() == 3);
            test_(quaternion.getY() == 0);
            test_(quaternion.getZ() == 0);
            test_(quaternion.getW() == 0);
        }

        // Test setY()
        void testSetY() {
            Quaternion quaternion;
            quaternion.setY(3);
            test_(quaternion.getX() == 0);
            test_(quaternion.getY() == 3);
            test_(quaternion.getZ() == 0);
            test_(quaternion.getW() == 0);
        }

        // Test setZ()
        void testSetZ() {
            Quaternion quaternion;
            quaternion.setZ(3);
            test_(quaternion.getX() == 0);
            test_(quaternion.getY() == 0);
            test_(quaternion.getZ() == 3);
            test_(quaternion.getW() == 0);
        }

        // Test setW()
        void testSetW() {
            Quaternion quaternion;
            quaternion.setW(3);
            test_(quaternion.getX() == 0);
            test_(quaternion.getY() == 0);
            test_(quaternion.getZ() == 0);
            test_(quaternion.getW() == 3);
        }

        // Test vectorV()
        void testVectorV() {
            Vector3D vector1(2,3,4);
            Vector3D vector2(6,7,8);

            Vector3D vectorTest1 = quaternion1.vectorV();
            Vector3D vectorTest2 = quaternion2.vectorV();
            test_(vectorTest1 == vector1);
            test_(vectorTest2 == vector2);
        }

        // Test length()
        void testLength() {
            Quaternion quaternion;
            test_(quaternion.length() == 0);

            Quaternion quaternion3(3, 4, 0, 0);
            test_(quaternion3.length() == 5);

            Quaternion quaternion4(0, 4, 3, 0);
            test_(quaternion4.length() == 5);

            Quaternion quaternion5(0, 0, 3, 4);
            test_(quaternion5.length() == 5);
        }

        // Test getUnit()
        void testGetUnit() {
            // Try to compute a valid unit quaternion
            try {
                Quaternion quaternion(3, 4, 0, 0);
                Quaternion unit = quaternion.getUnit();                 // This shouldn't throw an exception
                test_(unit.getX() == 3.0/5.0);
                test_(unit.getY() == 4.0/5.0);
                test_(unit.getZ() == 0);
                test_(unit.getW() == 0);

                Quaternion quaternion3(0, 0, 4, 3);
                Quaternion unit2 = quaternion3.getUnit();                // This shouldn't throw an exception
                test_(unit2.getX() == 0);
                test_(unit2.getY() == 0);
                test_(unit2.getZ() == 4.0/5.0);
                test_(unit2.getW() == 3.0/5.0);
                succeed_();                                             // Succeed if no exception have been thrown
            }
            catch(MathematicsException& ex) {
                fail_("Valid getUnit() call throw an exception");       // Failed if an exception has been thrown
            }

            // Try to compute an invalid unit quaternion
            try {
                Quaternion quaternion(0, 0, 0, 0);
                quaternion.getUnit();                                   // This should throw an exception
                fail_("Invalid getUnit() call undetected");             // Failed if no exception have been thrown
            }
            catch(MathematicsException& ex) {
                succeed_();                                             // Succeed if an exception has been thrown
            }
        }

        // Test getConjugate()
        void testGetConjugate() {
            Quaternion conjugate1 = quaternion1.getConjugate();
            test_(conjugate1.getX() == -2);
            test_(conjugate1.getY() == -3);
            test_(conjugate1.getZ() == -4);
            test_(conjugate1.getW() == 5);

            Quaternion conjugate2 = quaternion2.getConjugate();
            test_(conjugate2.getX() == -6);
            test_(conjugate2.getY() == -7);
            test_(conjugate2.getZ() == -8);
            test_(conjugate2.getW() == 9);
        }

        // Test getInverse()
        void testGetInverse() {
            // Try to compute a valid inverse quaternion
            try {
                Quaternion quaternion(3, 4, 0, 0);
                Quaternion inverse = quaternion.getInverse();          // This shouldn't throw an exception
                test_(inverse.getX() == -3.0/25.0);
                test_(inverse.getY() == -4.0/25.0);
                test_(inverse.getZ() == 0);
                test_(inverse.getW() == 0);

                Quaternion quaternion3(0, 0, 4, 3);
                Quaternion inverse2 = quaternion3.getInverse();         // This shouldn't throw an exception
                test_(inverse2.getX() == 0);
                test_(inverse2.getY() == 0);
                test_(inverse2.getZ() == -4.0/25.0);
                test_(inverse2.getW() == 3.0/25.0);
                succeed_();                                             // Succeed if no exception have been thrown
            }
            catch(MathematicsException& ex) {
                fail_("Valid getInverse() call throw an exception");       // Failed if an exception has been thrown
            }

            // Try to compute an invalid unit quaternion
            try {
                Quaternion quaternion(0, 0, 0, 0);
                quaternion.getInverse();                                // This should throw an exception
                fail_("Invalid getInverse() call undetected");          // Failed if no exception have been thrown
            }
            catch(MathematicsException& ex) {
                succeed_();                                             // Succeed if an exception has been thrown
            }
        }

        // Test the scalarProduct() method
        void testScalarProduct() {
            double result = quaternion1.scalarProduct(quaternion2);
            test_(result == 110.0);
        }

        // Test the getRotationAngleAxis() method
        void testgetRotationAngleAxis() {
            Quaternion quaternion(1.0, 2.0, 3.0, 0.0);
            double invAxisLength = 1.0/sqrt(1.0 + 2.0*2.0 + 3.0*3.0);
            double angle;
            Vector3D axis;

            quaternion.getRotationAngleAxis(angle, axis);
            test_(equal(angle, PI));
            test_(equal(axis.getX(), 1.0*invAxisLength));
            test_(equal(axis.getY(), 2.0*invAxisLength));
            test_(equal(axis.getZ(), 3.0*invAxisLength));
        }

        // Test the slerp() method
        void testSlerp() {
            // TODO : Test the Quaternion::slerp() method
        }

        // Test operator+()
        void testOperatorAddition() {
            Quaternion result = quaternion1 + quaternion2;
            test_(result.getX() == 8);
            test_(result.getY() == 10);
            test_(result.getZ() == 12);
            test_(result.getW() == 14);
        }

        // Test operator-()
        void testOperatorSubstraction() {
            Quaternion result = quaternion1 - quaternion2;
            test_(result.getX() == -4);
            test_(result.getY() == -4);
            test_(result.getZ() == -4);
            test_(result.getW() == -4);
        }

        // Test operator* (multiplication with a constant number)
        void testOperatorMultiplicationWithConstant() {
            Quaternion result = quaternion1 * 3;
            test_(result.getX() == 6);
            test_(result.getY() == 9);
            test_(result.getZ() == 12);
            test_(result.getW() == 15);
        }

        // Test operator* (multiplication with quaternion)
        void testOperatorMultiplicationWithQuaternion() {
            Quaternion result = quaternion1 * quaternion2;
            test_(result.getX() == 44);
            test_(result.getY() == 70);
            test_(result.getZ() == 72);
            test_(result.getW() == -20);
        }

        // Test operator=()
        void testOperatorAssignment() {
            Quaternion quaternion;
            quaternion = quaternion1;
            test_(quaternion.getX() == 2);
            test_(quaternion.getY() == 3);
            test_(quaternion.getZ() == 4);
            test_(quaternion.getW() == 5);
        }

        // Test operator==()
        void testOperatorEquality() {
            Quaternion quaternion(2,3,4,5);
            test_(quaternion == quaternion1);
            test_(quaternion1 == quaternion);
            test_(!(quaternion2 == quaternion1));
            test_(!(quaternion1 == quaternion2));
        }
};

#endif
