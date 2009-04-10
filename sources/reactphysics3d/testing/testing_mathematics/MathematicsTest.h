
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

#ifndef MATHEMATICSTEST_H
#define MATHEMATICSTEST_H

// Libraries
#include "../TestSuite/Test.h"
#include "../../mathematics/mathematics.h"
#include <stdexcept>
#include <iostream>
#include <cmath>

// Namespaces
using namespace reactphysics3d;

// Class MathematicsTest
class MathematicsTest : public TestSuite::Test {
   private :

   public :

        // Constructor
        MathematicsTest() {

        }

        // Run method of the Test
        void run() {
            testEqual();
        }

        // Test the equal() method
        void testEqual() {
            double number1 = 19.13417;
            double number2 = 19.13417 + EPSILON/2.0;
            double number3 = 19.13417 + 2*EPSILON;

            test_(equal(number1, number2));
            test_(equal(number2, number1));
            test_(!equal(number1, number3));
            test_(!equal(number3, number1));
        }

};

#endif
