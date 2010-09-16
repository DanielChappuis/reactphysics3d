/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2010 Daniel Chappuis                                            *
*********************************************************************************
*                                                                               *
* Permission is hereby granted, free of charge, to any person obtaining a copy  *
* of this software and associated documentation files (the "Software"), to deal *
* in the Software without restriction, including without limitation the rights  *
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell     *
* copies of the Software, and to permit persons to whom the Software is         *
* furnished to do so, subject to the following conditions:                      *
*                                                                               *
* The above copyright notice and this permission notice shall be included in    *
* all copies or substantial portions of the Software.                           *
*                                                                               *
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR    *
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,      *
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE   *
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER        *
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, *
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN     *
* THE SOFTWARE.                                                                 *
********************************************************************************/

#ifndef CONSTANTS_H
#define	CONSTANTS_H

// Libraries
#include <limits>

// Type definitions
typedef unsigned int uint;

// Mathematical constants
const double EPSILON = 0.00001;                                             // Epsilon value to avoid numerical errors
const double ONE_MINUS_EPSILON = 0.99999;                                   // 1 - espilon
const double INFINITY_CONST = std::numeric_limits<double>::infinity();      // Infinity constant
const double PI = 3.14159265;                                               // Pi constant

// Contact constants
const double FRICTION_COEFFICIENT = 0.4;    // Friction coefficient
const double PENETRATION_FACTOR = 0.2;      // Penetration factor (between 0 and 1) which specify the importance of the
                                            // penetration depth in order to calculate the correct impulse for the contact

// Constraint solver constants
const uint MAX_LCP_ITERATIONS = 10;         // Maximum number of iterations when solving a LCP problem
const double AV_COUNTER_LIMIT = 500;        // Maximum number value of the avBodiesCounter or avConstraintsCounter
const double AV_PERCENT_TO_FREE = 0.5;      // We will free the memory if the current nb of bodies (or constraints) is
                                            // less than AV_PERCENT_TO_FREE * bodiesCapacity (or constraintsCapacity). This
                                            // is used to avoid to keep to much memory for a long time if the system doesn't
                                            // need that memory. This value is between 0.0 and 1.0


#endif

