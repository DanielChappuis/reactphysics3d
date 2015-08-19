/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2015 Daniel Chappuis                                       *
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

// Libraries
#include "DefaultCollisionDispatch.h"

using namespace reactphysics3d;

// Constructor
DefaultCollisionDispatch::DefaultCollisionDispatch() {

}

// Destructor
DefaultCollisionDispatch::~DefaultCollisionDispatch() {

}

/// Initialize the collision dispatch configuration
void DefaultCollisionDispatch::init(MemoryAllocator* memoryAllocator) {

    // Initialize the collision algorithms
    mSphereVsSphereAlgorithm.init(memoryAllocator);
    mGJKAlgorithm.init(memoryAllocator);
}

// Select and return the narrow-phase collision detection algorithm to
// use between two types of collision shapes.
NarrowPhaseAlgorithm* DefaultCollisionDispatch::selectAlgorithm(int shape1Type,
                                                                int shape2Type) {

    // Sphere vs Sphere algorithm
    if (shape1Type == SPHERE && shape2Type == SPHERE) {
        return &mSphereVsSphereAlgorithm;
    }
    else {   // GJK algorithm
        return &mGJKAlgorithm;
    }
}
