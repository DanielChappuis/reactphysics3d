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

#ifndef KILOGRAMTEST_H
#define KILOGRAMTEST_H

// Libraries
#include "../TestSuite/Test.h"
#include "../../physics/Kilogram.h"
#include <stdexcept>

// Namespaces
using namespace reactphysics3d;

// Class KilogramTest
class KilogramTest : public TestSuite::Test {
   private :
        Kilogram mass1;
        Kilogram mass2;
   public :

        // Constructor
        KilogramTest() : mass1(5.0), mass2(10.0) {

        }

        // Run method of the Test
        void run() {
           testConstructors();
           testGetValue();
           testSetValue();
        }

        // Test the constructors
        void testConstructors() {
            // Try valid constructors calls
            try {
                Kilogram mass(30.4);                    // This should'n throw an exception
                Kilogram mass3(mass);                   // This should'n throw an exception
                Kilogram mass4(0.0);                    // This should'n throw an exception
                Kilogram mass5;
                test_(mass.getValue() == 30.4);
                test_(mass3.getValue() == 30.4);
                test_(mass4.getValue() == 0.0);
                test_(mass5.getValue() == 0.0);
                succeed_();
            }
            catch(std::invalid_argument& ex) {
                fail_("Valid constructor call throws an exception");
            }

            // Try an invalid constructor call
            try{
                Kilogram mass4(-0.1);                   // This should throw an exception
                fail_("Invalid constructors calls didn't throw an exception");
            }
            catch(std::invalid_argument& ex) {
                succeed_();
            }

            // Try an invalid constructor call
            try{
                Kilogram mass4(-10.);                   // This should throw an exception
                fail_("Invalid constructors calls didn't throw an exception");
            }
            catch(std::invalid_argument& ex) {
                succeed_();
            }
        }

        // Test the method getValue()
        void testGetValue() {
            test_(mass1.getValue() == 5.0);
            test_(mass2.getValue() == 10.0);
        }

        // Test the method setValue()
        void testSetValue() {
            Kilogram mass(10.0);

            // Try a valid setValue() call
            try {
                mass.setValue(0.0);                 // This should'n throw an exception
                test_(mass.getValue() == 0.0);
                mass.setValue(43.0);                // This should'n throw an exception
                test_(mass.getValue() == 43.0);
                succeed_();
            }
            catch(std::invalid_argument& ex) {
                fail_("Valid setValue() call throw an exception");
            }

            // Try an invalid setValue() call
            try {
                mass.setValue(-0.1);                // This should throw an exception
                fail_("Invalid setValue() call didn't throw an exception");
            }
            catch(std::invalid_argument& ex) {
                succeed_();
            }

            // Try an invalid setValue() call
            try {
                mass.setValue(-40.0);                // This should throw an exception
                fail_("Invalid setValue() call didn't throw an exception");
            }
            catch(std::invalid_argument& ex) {
                succeed_();
            }
        }

};

#endif
