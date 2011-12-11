/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2010-2012 Daniel Chappuis                                       *
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

#ifndef CONSTANTS_H
#define	CONSTANTS_H

// Libraries
#include <limits>
#include <cfloat>

// Type definitions
typedef unsigned int uint;
typedef long unsigned int luint;

// Mathematical constants
const double EPSILON = 1.0e-10;                                             // Epsilon value
const double MACHINE_EPSILON = DBL_EPSILON;                                 // Machine epsilon
const double EPSILON_TEST = 0.00001;                                        // TODO : Try not to use this value
const double ONE_MINUS_EPSILON_TEST = 0.99999;                              // TODO : Try not to use this value
const double INFINITY_CONST = std::numeric_limits<double>::infinity();      // Infinity constant
const double PI = 3.14159265;                                               // Pi constant

// Physics Engine constants
const double DEFAULT_TIMESTEP = 1.0 / 60.0;             // Default internal constant timestep in seconds

// GJK Algorithm parameters
const double OBJECT_MARGIN = 0.04;                      // Object margin for collision detection in cm

// Contact constants
const double FRICTION_COEFFICIENT = 0.4;                // Friction coefficient
const double DEFAULT_PENETRATION_FACTOR = 5.0;          // Penetration factor (between 0 and 1) which specify the importance of the
                                                        // penetration depth in order to calculate the correct impulse for the contact
const double PERSISTENT_CONTACT_DIST_THRESHOLD = 0.02;  // Distance threshold for two contact points for a valid persistent contact

const int NB_MAX_BODIES = 100000;                       // Maximum number of bodies
const int NB_MAX_CONTACTS = 100000;                     // Maximum number of contacts (for memory pool allocation)
const int NB_MAX_CONSTRAINTS = 100000;                  // Maximum number of constraints
const int NB_MAX_COLLISION_PAIRS = 10000;               // Maximum number of collision pairs of bodies (for memory pool allocation)

// Constraint solver constants
const uint DEFAULT_LCP_ITERATIONS = 15;                 // Number of iterations when solving a LCP problem
const double AV_COUNTER_LIMIT = 500;                    // Maximum number value of the avBodiesCounter or avConstraintsCounter
const double AV_PERCENT_TO_FREE = 0.5;                  // We will free the memory if the current nb of bodies (or constraints) is
                                                        // less than AV_PERCENT_TO_FREE * bodiesCapacity (or constraintsCapacity). This
                                                        // is used to avoid to keep to much memory for a long time if the system doesn't
                                                        // need that memory. This value is between 0.0 and 1.0


#endif

