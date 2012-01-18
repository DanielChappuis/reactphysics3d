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

#ifndef CONFIGURATION_H
#define	CONFIGURATION_H

// Libraries
#include <limits>
#include <cfloat>
#include "decimal.h"

// Windows platform
#if defined(WIN32) || defined(_WIN32) || defined(_WIN64) || defined(__WIN32__) || defined(__WINDOWS__)
    #define WINDOWS_OS
#elif defined(__APPLE__)     // Apple platform
    #define APPLE_OS
#elif defined(__linux__) || defined(linux) || defined(__linux)    // Linux platform
    #define LINUX_OS
#endif

// Type definitions
typedef unsigned int uint;
typedef long unsigned int luint;

// Mathematical constants
const reactphysics3d::decimal DECIMAL_MIN = std::numeric_limits<reactphysics3d::decimal>::min();                // Minimun decimal value
const reactphysics3d::decimal DECIMAL_MAX = std::numeric_limits<reactphysics3d::decimal>::max();               // Maximum decimal value
const reactphysics3d::decimal MACHINE_EPSILON = std::numeric_limits<reactphysics3d::decimal>::epsilon();        // Machine epsilon
const reactphysics3d::decimal DECIMAL_INFINITY = std::numeric_limits<reactphysics3d::decimal>::infinity();      // Infinity
const reactphysics3d::decimal PI = 3.14159265;                                                                  // Pi constant

// Physics Engine constants
const reactphysics3d::decimal DEFAULT_TIMESTEP = 1.0 / 60.0;             // Default internal constant timestep in seconds

// GJK Algorithm parameters
const reactphysics3d::decimal OBJECT_MARGIN = 0.04;                      // Object margin for collision detection in cm

// Contact constants
const reactphysics3d::decimal FRICTION_COEFFICIENT = 0.4;                // Friction coefficient
const reactphysics3d::decimal PERSISTENT_CONTACT_DIST_THRESHOLD = 0.02;  // Distance threshold for two contact points for a valid persistent contact

// Constraint solver constants
const int NB_MAX_BODIES = 100000;             // Maximum number of bodies
const int NB_MAX_CONTACTS = 100000;           // Maximum number of contacts (for memory pool allocation)
const int NB_MAX_CONSTRAINTS = 100000;        // Maximum number of constraints
const int NB_MAX_COLLISION_PAIRS = 10000;     // Maximum number of collision pairs of bodies (for memory pool allocation)

// Constraint solver constants
const uint DEFAULT_LCP_ITERATIONS = 15;                                            // Number of iterations when solving a LCP problem
const uint DEFAULT_LCP_ITERATIONS_ERROR_CORRECTION = 5;                            // Number of iterations when solving a LCP problem for error correction
const bool ERROR_CORRECTION_PROJECTION_ENABLED = true;                             // True if the error correction projection (first order world) is active in the constraint solver
const reactphysics3d::decimal PENETRATION_DEPTH_THRESHOLD_ERROR_CORRECTION = 0.20; // Contacts with penetration depth (in meters) larger that this use error correction with projection


#endif
