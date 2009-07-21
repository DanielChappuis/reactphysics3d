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

#ifndef TIMETEST_H
#define TIMETEST_H

// Libraries
#include "../TestSuite/Test.h"
#include "../../physics/Time.h"
#include <stdexcept>

// Namespaces
using namespace reactphysics3d;

// Class TimeTest
class TimeTest : public TestSuite::Test {
   private :
        Time time1;
        Time time2;
   public :

        // Constructor
        TimeTest() : time1(5.0), time2(10.0) {

        }

        // Run method of the Test
        void run() {
           testConstructors();
           testGetValue();
           testSetValue();
           testOperatorAddition();
           testOperatorSubstraction();
           testOperatorMultiplication();
        }

        // Test the constructors
        void testConstructors() {
            // Try valid constructors calls
            try {
                Time time(30.4);                    // This should'n throw an exception
                Time time3(time);                   // This should'n throw an exception
                Time time4(0.0);                    // This should'n throw an exception
                Time time5;
                test_(time.getValue() == 30.4);
                test_(time3.getValue() == 30.4);
                test_(time4.getValue() == 0.0);
                test_(time5.getValue() == 0.0);
                succeed_();
            }
            catch(std::invalid_argument& ex) {
                fail_("Valid constructor call throws an exception");
            }

            // Try an invalid constructor call
            try{
                Time time4(-0.1);                   // This should throw an exception
                fail_("Invalid constructors calls didn't throw an exception");
            }
            catch(std::invalid_argument& ex) {
                succeed_();
            }

            // Try an invalid constructor call
            try{
                Time time4(-10.);                   // This should throw an exception
                fail_("Invalid constructors calls didn't throw an exception");
            }
            catch(std::invalid_argument& ex) {
                succeed_();
            }
        }

        // Test the method getValue()
        void testGetValue() {
            test_(time1.getValue() == 5.0);
            test_(time2.getValue() == 10.0);
        }

        // Test the method setValue()
        void testSetValue() {
            Time time(10.0);

            // Try a valid setValue() call
            try {
                time.setValue(0.0);                 // This should'n throw an exception
                test_(time.getValue() == 0.0);
                time.setValue(43.0);                // This should'n throw an exception
                test_(time.getValue() == 43.0);
                succeed_();
            }
            catch(std::invalid_argument& ex) {
                fail_("Valid setValue() call throw an exception");
            }

            // Try an invalid setValue() call
            try {
                time.setValue(-0.1);                // This should throw an exception
                fail_("Invalid setValue() call didn't throw an exception");
            }
            catch(std::invalid_argument& ex) {
                succeed_();
            }

            // Try an invalid setValue() call
            try {
                time.setValue(-40.0);                // This should throw an exception
                fail_("Invalid setValue() call didn't throw an exception");
            }
            catch(std::invalid_argument& ex) {
                succeed_();
            }
        }

        // Test the overloaded addition operator
        void testOperatorAddition() {
            Time result;
            result = time1 + time2;
            test_(result.getValue() == 15.0);
        }

        // Test the overloaded substraction operator
        void testOperatorSubstraction() {
            Time result;

            // Try a valid substraction
            try {

                result = time2 - time1;             // This should'n throw an exception
                test_(result.getValue() == 5.0);
                succeed_();
            }
            catch(std::invalid_argument& ex) {
                fail_("Valid call to substraction operator throw an exception");
            }

            // try an invalid substraction
            try {
                result = time1 - time2;
                fail_("Invalid call to substraction didn't throw an exception");
            }
            catch(std::invalid_argument& ex) {
                succeed_();
            }
        }

        // Test the overloaded multiplication operator
        void testOperatorMultiplication() {
             Time result;

            // Try a valid substraction
            try {
                result = time1 * 3.0;                 // This should'n throw an exception
                test_(result.getValue() == 15.0);
                succeed_();
            }
            catch(std::invalid_argument& ex) {
                fail_("Valid call to multiplication operator throw an exception");
            }

            // try an invalid substraction
            try {
                result = time1 * (-3.0);
                fail_("Invalid call to multiplication didn't throw an exception");
            }
            catch(std::invalid_argument& ex) {
                succeed_();
            }
        }

};

#endif
