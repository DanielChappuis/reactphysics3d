/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2022 Daniel Chappuis                                       *
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
#include <reactphysics3d/engine/Island.h>
#include <reactphysics3d/memory/MemoryManager.h>

using namespace reactphysics3d;

// Constructor
Island::Island(uint32 nbMaxBodies, uint32 nbMaxContactManifolds, MemoryManager& memoryManager)
       : mBodies(nullptr), mContactManifolds(nullptr), mNbBodies(0), mNbContactManifolds(0) {

    // Allocate memory for the arrays on the single frame allocator
    mBodies = static_cast<RigidBody**>(memoryManager.allocate(MemoryManager::AllocationType::Frame,
                                                              sizeof(RigidBody*) * nbMaxBodies));
    mContactManifolds = static_cast<ContactManifold**>(memoryManager.allocate(MemoryManager::AllocationType::Frame,
                                                                              sizeof(ContactManifold*) * nbMaxContactManifolds));
}

// Destructor
Island::~Island() {
    // This destructor is never called because memory is allocated on the
    // single frame allocator
}
