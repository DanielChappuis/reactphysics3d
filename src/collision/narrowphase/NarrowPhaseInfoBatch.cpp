/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2024 Daniel Chappuis                                       *
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
#include <reactphysics3d/collision/narrowphase/NarrowPhaseInfoBatch.h>
#include <reactphysics3d/collision/ContactPointInfo.h>
#include <reactphysics3d/collision/shapes/TriangleShape.h>
#include <reactphysics3d/engine/OverlappingPairs.h>
#include <iostream>

using namespace reactphysics3d;

// TriangleShape allocated size
const size_t NarrowPhaseInfoBatch::mTriangleShapeAllocatedSize = std::ceil(sizeof(TriangleShape) / float(GLOBAL_ALIGNMENT)) * GLOBAL_ALIGNMENT;

// Constructor
NarrowPhaseInfoBatch::NarrowPhaseInfoBatch(OverlappingPairs& overlappingPairs, MemoryAllocator& allocator)
                     : mMemoryAllocator(allocator), mOverlappingPairs(overlappingPairs), narrowPhaseInfos(allocator){

}

// Destructor
NarrowPhaseInfoBatch::~NarrowPhaseInfoBatch() {
    clear();
}

// Initialize the containers using cached capacity
void NarrowPhaseInfoBatch::reserveMemory() {

    narrowPhaseInfos.reserve(mCachedCapacity);
}

// Clear all the objects in the batch
void NarrowPhaseInfoBatch::clear() {

    const uint32 nbNarrowPhaseInfos = static_cast<uint32>(narrowPhaseInfos.size());
    for (uint32 i=0; i < nbNarrowPhaseInfos; i++) {

        assert(narrowPhaseInfos[i].nbContactPoints == 0);

        // TODO OPTI : Better manage this

        // Release the memory of the TriangleShape (this memory was allocated in the
        // MiddlePhaseTriangleCallback::testTriangle() method)
        if (narrowPhaseInfos[i].collisionShape1->getName() == CollisionShapeName::TRIANGLE) {
            narrowPhaseInfos[i].collisionShape1->~CollisionShape();
            narrowPhaseInfos[i].collisionShapeAllocator->release(narrowPhaseInfos[i].collisionShape1, mTriangleShapeAllocatedSize);
        }
        if (narrowPhaseInfos[i].collisionShape2->getName() == CollisionShapeName::TRIANGLE) {
            narrowPhaseInfos[i].collisionShape2->~CollisionShape();
            narrowPhaseInfos[i].collisionShapeAllocator->release(narrowPhaseInfos[i].collisionShape2, mTriangleShapeAllocatedSize);
        }
    }

    // Note that we clear the following containers and we release their allocated memory. Therefore,
    // if the memory allocator is a single frame allocator, the memory is deallocated and will be
    // allocated in the next frame at a possibly different location in memory (remember that the
    // location of the allocated memory of a single frame allocator might change between two frames)

    mCachedCapacity = static_cast<uint32>(narrowPhaseInfos.capacity());

    narrowPhaseInfos.clear(true);
}
