/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2020 Daniel Chappuis                                       *
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
#include <reactphysics3d/collision/narrowphase/CapsuleVsCapsuleNarrowPhaseInfoBatch.h>
#include <reactphysics3d/collision/shapes/CapsuleShape.h>

using namespace reactphysics3d;

// Constructor
CapsuleVsCapsuleNarrowPhaseInfoBatch::CapsuleVsCapsuleNarrowPhaseInfoBatch(MemoryAllocator& allocator,
                                                                           OverlappingPairs& overlappingPairs)
      : NarrowPhaseInfoBatch(allocator, overlappingPairs), capsule1Radiuses(allocator), capsule2Radiuses(allocator),
        capsule1Heights(allocator), capsule2Heights(allocator) {

}

// Add shapes to be tested during narrow-phase collision detection into the batch
void CapsuleVsCapsuleNarrowPhaseInfoBatch::addNarrowPhaseInfo(uint64 pairId, uint64 pairIndex, Entity collider1, Entity collider2, CollisionShape* shape1, CollisionShape* shape2,
                                                            const Transform& shape1Transform, const Transform& shape2Transform, bool needToReportContacts, MemoryAllocator &shapeAllocator) {

    NarrowPhaseInfoBatch::addNarrowPhaseInfo(pairId, pairIndex, collider1, collider2, shape1, shape2, shape1Transform,
                                             shape2Transform, needToReportContacts, shapeAllocator);

    assert(shape1->getType() == CollisionShapeType::CAPSULE);
    assert(shape2->getType() == CollisionShapeType::CAPSULE);

    const CapsuleShape* capsule1 = static_cast<const CapsuleShape*>(shape1);
    const CapsuleShape* capsule2 = static_cast<const CapsuleShape*>(shape2);

    capsule1Radiuses.add(capsule1->getRadius());
    capsule2Radiuses.add(capsule2->getRadius());
    capsule1Heights.add(capsule1->getHeight());
    capsule2Heights.add(capsule2->getHeight());
}

// Initialize the containers using cached capacity
void CapsuleVsCapsuleNarrowPhaseInfoBatch::reserveMemory() {

    NarrowPhaseInfoBatch::reserveMemory();

    capsule1Radiuses.reserve(mCachedCapacity);
    capsule2Radiuses.reserve(mCachedCapacity);
    capsule1Heights.reserve(mCachedCapacity);
    capsule2Heights.reserve(mCachedCapacity);
}

// Clear all the objects in the batch
void CapsuleVsCapsuleNarrowPhaseInfoBatch::clear() {

    // Note that we clear the following containers and we release their allocated memory. Therefore,
    // if the memory allocator is a single frame allocator, the memory is deallocated and will be
    // allocated in the next frame at a possibly different location in memory (remember that the
    // location of the allocated memory of a single frame allocator might change between two frames)

    NarrowPhaseInfoBatch::clear();

    capsule1Radiuses.clear(true);
    capsule2Radiuses.clear(true);
    capsule1Heights.clear(true);
    capsule2Heights.clear(true);
}
