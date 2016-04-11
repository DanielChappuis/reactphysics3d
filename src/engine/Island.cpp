/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2016 Daniel Chappuis                                       *
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
#include "Island.h"

using namespace reactphysics3d;

// Constructor
Island::Island(uint nbMaxBodies, uint nbMaxContactManifolds, uint nbMaxJoints,
               MemoryAllocator& memoryAllocator)
       : mBodies(NULL), mContactManifolds(NULL), mJoints(NULL), mNbBodies(0),
         mNbContactManifolds(0), mNbJoints(0), mMemoryAllocator(memoryAllocator) {

    // Allocate memory for the arrays
    mNbAllocatedBytesBodies = sizeof(RigidBody*) * nbMaxBodies;
    mBodies = (RigidBody**) mMemoryAllocator.allocate(mNbAllocatedBytesBodies);
    mNbAllocatedBytesContactManifolds = sizeof(ContactManifold*) * nbMaxContactManifolds;
    mContactManifolds = (ContactManifold**) mMemoryAllocator.allocate(
                                                                mNbAllocatedBytesContactManifolds);
    mNbAllocatedBytesJoints = sizeof(Joint*) * nbMaxJoints;
    mJoints = (Joint**) mMemoryAllocator.allocate(mNbAllocatedBytesJoints);
}

// Destructor
Island::~Island() {

    // Release the memory of the arrays
    mMemoryAllocator.release(mBodies, mNbAllocatedBytesBodies);
    mMemoryAllocator.release(mContactManifolds, mNbAllocatedBytesContactManifolds);
    mMemoryAllocator.release(mJoints, mNbAllocatedBytesJoints);
}
