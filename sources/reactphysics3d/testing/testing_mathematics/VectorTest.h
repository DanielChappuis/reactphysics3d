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

#ifndef VECTORTEST_H
#define VECTORTEST_H

// Libraries
#include "../TestSuite/Test.h"
#include "../../mathematics/Vector.h"
#include <stdexcept>

// Namespaces
using namespace reactphysics3d;

// Class VectorTest
class VectorTest : public TestSuite::Test {
   private :
        Vector vect2;       // Vector of dimension 2
        Vector vect4;       // Vector of dimension 4
   public :

        // Constructor
        VectorTest() : vect2(2), vect4(4) {
            vect2.setValue(0, 3);
            vect2.setValue(1, 5);

            vect4.setValue(0, 1);
            vect4.setValue(1, -5);
            vect4.setValue(2, 10);
            vect4.setValue(3, 3);
        }

        // Run method of the Test
        void run() {
            testConstructors();
            testGetValue();
            testSetValue();
            testGetNbComponent();
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

            // Try a valid constructor call
            try {
                // Constructor
                Vector vector(3);                                       // This shouldn't throw an exception
                test_(vector.getValue(0) == 0);
                test_(vector.getValue(1) == 0);
                test_(vector.getValue(2) == 0);
                succeed_();                                             // Test succeed if no exception have been thrown
            }
            catch(std::invalid_argument& ex) {
                fail_("Valid constructor call throws an exception");    // Test failed if an exception has been thrown
            }

             // Try a invalid constructor call
            try {
                // Constructor
                Vector vector3(-1);                                             // This should throw an exception
                fail_("Invalid constructor (argument -1) call undetected");    // Test failed if no exception have been thrown
            }
            catch(std::invalid_argument& ex) {
                succeed_();                                      // Test succeed if an exception has been thrown
            }

            // Try a invalid constructor call
            try {
                // Constructor
                Vector vector4(0);                                           // This should throw an exception
                fail_("Invalid constructor call (argument 0) undetected");  // Test failed if no exception have been thrown
            }
            catch(std::invalid_argument& ex) {
                succeed_();                                      // Test succeed if an exception has been thrown
            }

            // Copy-Constructor
            Vector vector5 = vect2;
            test_(vector5.getValue(0) == 3);
            test_(vector5.getValue(1) == 5);
        }

        // Test getValue()
        void testGetValue() {
            test_(vect2.getValue(0) == 3);
            test_(vect2.getValue(1) == 5);

            // Try to get an invalid value
            try {
                vect2.getValue(-1);                         // This should throw an exception
                fail_("Invalid getValue(-1) undetected");   // The test failed if no exception have been thrown
            }
            catch(std::invalid_argument& ex) {
               succeed_();                                  // The test succeed if an exception has been thrown
            }

            // Try to get an invalid value
            try {
                vect2.getValue(2);                          // This should throw an exception
                fail_("Invalid getValue(2) undetected");    // The test failed if no exception have been thrown
            }
            catch(std::invalid_argument& ex) {
               succeed_();                                  // The test succeed if an exception has been thrown
            }
        }

        // Test setValue()
        void testSetValue() {
            Vector vector(5);

            // Try to set valid values
            try {
                vector.setValue(0, 5);
                vector.setValue(3, -1);
                vector.setValue(4, 14);
                test_(vector.getValue(0) == 5);
                test_(vector.getValue(3) == -1);
                test_(vector.getValue(4) == 14);
                succeed_();                                     // The test succeed if an exception have been thrown
            }
            catch(std::invalid_argument& ex) {
               fail_("Valid setValue() throws an exception");   // The failed if an exception has been thrown
            }

            // Try to set an invalid value
            try {
                vector.setValue(-1, 4);                         // This should throw an exception
                fail_("Invalid setValue(-1,4) undetected");     // The test failed if no exception have been thrown
            }
            catch(std::invalid_argument& ex) {
               succeed_();                                      // The test succeed if an exception has been thrown
            }

            // Try to set an invalid value
            try {
                vector.setValue(5, 2);                          // This should throw an exception
                fail_("Invalid setValue(5,2) undetected");      // The test failed if no exception have been thrown
            }
            catch(std::invalid_argument& ex) {
               succeed_();                                      // The test succeed if an exception has been thrown
            }
        }

        // Test getNbComponent()
        void testGetNbComponent() {
            test_(vect2.getNbComponent() == 2);
            test_(vect4.getNbComponent() == 4);
        }

        // Test length()
        void testLength() {
            Vector vector1(2);
            test_(vector1.length() == 0);
            vector1.setValue(0, 4);
            test_(vector1.length() == 4);
            vector1.setValue(1, -3);
            test_(vector1.length() == 5);
        }

        // Test getUnit()
        void testGetUnit() {
            Vector vector1(3);
            vector1.setValue(0, 3);
            test_(vector1.getUnit().length() == 1);
            test_(vector1.getUnit().getValue(0) == 1);
            test_(vector1.getUnit().getValue(1) == 0);
            test_(vector1.getUnit().getValue(2) == 0);

            Vector vector2(8);
            vector2.setValue(2, 54);
            test_(vector2.getUnit().length() == 1);
            test_(vector2.getUnit().getValue(0) == 0);
            test_(vector2.getUnit().getValue(1) == 0);
            test_(vector2.getUnit().getValue(2) == 1);

            Vector vector3(7);                                                          // Vector of length equal to zero
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
            Vector vector1(2);
            vector1.setValue(0, 4);
            vector1.setValue(1, -5);

            Vector vector2(2);
            vector2.setValue(0, 3);
            vector2.setValue(1, 2);

            // Try to compute a valid scalar product
            try {
                test_(vector1.scalarProduct(vector2) == 2);
                succeed_();                                         // The test succeed if no exception have been thrown
            }
            catch(MathematicsException& ex) {
                fail_("scalarProduct() thrown an exception during a valid scalar product computation");     // The test failed if an exception has been thrown
            }

            // Try to compute a invalid scalar product
            Vector vector3(5);
            try {
                vector1.scalarProduct(vector3);                                 // This should throw an exception
                fail_("Invalid dimensions in scalarProduct() undetected");      // The test failed if no exception have been thrown
            }
            catch(MathematicsException& ex) {
                succeed_();                                                     // The test succeed if an exception has been thrown
            }
        }

        // Test crossProduct()
        void testCrossProduct() {
            Vector vector1(3);
            vector1.setValue(0, 4);
            vector1.setValue(1, -5);
            vector1.setValue(2, 2);

            Vector vector2(3);
            vector2.setValue(0, 3);
            vector2.setValue(1, 2);
            vector2.setValue(2, 6);

            // Try to compute a valid scalar product
            try {
                Vector result = vector1.crossProduct(vector2);
                test_(result.getValue(0) == -34);
                test_(result.getValue(1) == -18);
                test_(result.getValue(2) == 23);
                succeed_();                                         // The test succeed if no exception have been thrown
            }
            catch(MathematicsException& ex) {
                fail_("crossProduct() thrown an exception during a valid cross product computation");     // The test failed if an exception has been thrown
            }

            // Try to compute a invalid cross product
            Vector vector3(5);
            try {
                vector1.crossProduct(vector3);                                 // This should throw an exception
                fail_("Invalid dimensions in crossProduct() undetected");      // The test failed if no exception have been thrown
            }
            catch(MathematicsException& ex) {
                succeed_();                                                     // The test succeed if an exception has been thrown
            }
        }

        // Test operator+()
        void testOperatorAddition() {
            Vector vector1(4);
            vector1.setValue(0, 3);
            vector1.setValue(3, -2);

            Vector vector2(4);
            vector2.setValue(0, 9);
            vector2.setValue(2, 6);
            vector2.setValue(3, 9);

            // Try to compute a valid sum (two vectors with the same dimensions)
            try {
                // Compute the sum
                Vector sum = vector1 + vector2;                            // This shouldn't throw an exception

                test_(sum.getValue(0) == 12);
                test_(sum.getValue(1) == 0);
                test_(sum.getValue(2) == 6);
                test_(sum.getValue(3) == 7);
                succeed_();                                                 // The test succeed if no exception have been thrown
            }
            catch(MathematicsException& ex) {
                fail_("A valid sum of two vectors throws an excception");   // The test failed if an exception has been thrown
            }

            // Try to compute an invalid sum (vectors with different dimensions)
            Vector vector3(3);
            Vector vector4(5);
            try {
                // Compute the sum
                Vector sum = vector3 + vector4;                         // This should throw an exception
                fail_("An invalid sum of two vectors undetected");      // The test failed if no exception have been thrown
            }
            catch(MathematicsException& ex) {
                succeed_();                                             // The test succeed if an exception has been thrown
            }
        }

        // Test operator-()
        void testOperatorSubstraction() {
            Vector vector1(4);
            vector1.setValue(0, 3);
            vector1.setValue(3, -2);

            Vector vector2(4);
            vector2.setValue(0, 9);
            vector2.setValue(2, 6);
            vector2.setValue(3, 9);

            // Try to compute a valid subtraction (two vectors with the same dimensions)
            try {
                // Compute the substraction
                Vector sub = vector1 - vector2;                            // This shouldn't throw an exception

                test_(sub.getValue(0) == -6);
                test_(sub.getValue(1) == 0);
                test_(sub.getValue(2) == -6);
                test_(sub.getValue(3) == -11);
                succeed_();                                                 // The test succeed if no exception have been thrown
            }
            catch(MathematicsException& ex) {
                fail_("A valid subtraction of two vectors throws an excception");   // The test failed if an exception has been thrown
            }

            // Try to compute an invalid substraction (vectors with different dimensions)
            Vector vector3(3);
            Vector vector4(5);
            try {
                // Compute the substraction
                Vector sub = vector3 - vector4;                                 // This should throw an exception
                fail_("An invalid substraction of two vectors undetected");     // The test failed if no exception have been thrown
            }
            catch(MathematicsException& ex) {
                succeed_();                                                     // The test succeed if an exception has been thrown
            }
        }

        // Test operator*() (with a constant number)
        void testOperatorConstantMultiplications() {
            Vector vector(4);
            vector.setValue(0, 3);
            vector.setValue(3, -2);

            // Compute the multiplication
            Vector sum = vector * 3.0;
            test_(sum.getValue(0) == 9);
            test_(sum.getValue(1) == 0);
            test_(sum.getValue(2) == 0);
            test_(sum.getValue(3) == -6);
        }

        // Test operator=()
        void testOperatorAssignment() {
            Vector vector1(2);
            vector1.setValue(0, 4);
            vector1.setValue(1, 7);

            Vector vector2(8);


            // Try to compute a valid assignment (two vectors with the same dimensions)
            try {
                Vector vector(2);
                vector = vector1;                            // This shouldn't throw an exception
                test_(vector == vector1);

                vector = vector;                             // This shouldn't throw an exception
                succeed_();                                  // The test succeed if no exception have been thrown
            }
            catch(MathematicsException& ex) {
                fail_("A valid vector assignment throws an excception");   // The test failed if an exception has been thrown
            }

            // Try to compute an invalid assignment (vectors with different dimensions)
            try {
                Vector vector3(2);
                vector3 = vector2;                                          // This should throw an exception
                fail_("An invalid vector assignment undetected");           // The test failed if no exception have been thrown
            }
            catch(MathematicsException& ex) {
                succeed_();                                                 // The test succeed if an exception has been thrown
            }

            // Try to compute an invalid assignment (vectors with different dimensions)
            try {
                Vector vector3(2);
                vector2 = vector3;                                          // This should throw an exception
                fail_("An invalid vector assignment undetected");           // The test failed if no exception have been thrown
            }
            catch(MathematicsException& ex) {
                succeed_();                                                 // The test succeed if an exception has been thrown
            }
        }

        // Test operator==()
        void testOperatorEquality() {
            Vector vector1(2);
            vector1.setValue(0, 4);
            vector1.setValue(1, 7);

            Vector vector2(2);
            vector2.setValue(0, 4);
            vector2.setValue(1, 7);

            Vector vector3(2);
            vector3.setValue(0, 5);
            vector3.setValue(1, 7);

            Vector vector4(8);

            // Try to test a valid equality (two vectors with the same dimensions)
            try {
                test_(vector1 == vector2);                    // This shouldn't throw an exception
                test_(vector2 == vector1);                    // This shouldn't throw an exception
                test_(vector1 == vector1);                    // This shouldn't throw an exception
                test_(!(vector1 == vector3));                 // This shouldn't throw an exception
                test_(!(vector3 == vector1));                 // This shouldn't throw an exception
                succeed_();                                   // The test succeed if no exception have been thrown
            }
            catch(MathematicsException& ex) {
                fail_("A valid vector equality test throws an excception");   // The test failed if an exception has been thrown
            }

            // Try to test an invalid equality (vectors with different dimensions)
            try {
                vector4 == vector1;                                           // This should throw an exception
                fail_("An invalid equality test of two vectors undetected");  // The test failed if no exception have been thrown
            }
            catch(MathematicsException& ex) {
                succeed_();                                                   // The test succeed if an exception has been thrown
            }
        }
};

#endif
