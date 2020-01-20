/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2018 Daniel Chappuis                                       *
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
#include "SphereVsSphereNarrowPhaseInfoBatch.h"
#include "collision/shapes/SphereShape.h"

using namespace reactphysics3d;

// Constructor
SphereVsSphereNarrowPhaseInfoBatch::SphereVsSphereNarrowPhaseInfoBatch(MemoryAllocator& allocator, OverlappingPairs& overlappingPairs)
      : NarrowPhaseInfoBatch(allocator, overlappingPairs), sphere1Radiuses(allocator), sphere2Radiuses(allocator) {

}

// Add shapes to be tested during narrow-phase collision detection into the batch
void SphereVsSphereNarrowPhaseInfoBatch::addNarrowPhaseInfo(uint64 pairId, uint64 pairIndex, Entity collider1, Entity collider2, CollisionShape* shape1, CollisionShape* shape2,
                                                            const Transform& shape1Transform, const Transform& shape2Transform) {

    assert(shape1->getType() == CollisionShapeType::SPHERE);
    assert(shape2->getType() == CollisionShapeType::SPHERE);

    const SphereShape* sphere1 = static_cast<const SphereShape*>(shape1);
    const SphereShape* sphere2 = static_cast<const SphereShape*>(shape2);

    sphere1Radiuses.add(sphere1->getRadius());
    sphere2Radiuses.add(sphere2->getRadius());
    colliderEntities1.add(collider1);
    colliderEntities2.add(collider2);
    shape1ToWorldTransforms.add(shape1Transform);
    shape2ToWorldTransforms.add(shape2Transform);
    overlappingPairIds.add(pairId);
    contactPoints.add(List<ContactPointInfo*>(mMemoryAllocator));
    isColliding.add(false);

    // Add a collision info for the two collision shapes into the overlapping pair (if not present yet)
    LastFrameCollisionInfo* lastFrameInfo = mOverlappingPairs.addLastFrameInfoIfNecessary(pairIndex, shape1->getId(), shape2->getId());
    lastFrameCollisionInfos.add(lastFrameInfo);
}

// Initialize the containers using cached capacity
void SphereVsSphereNarrowPhaseInfoBatch::reserveMemory() {

    overlappingPairIds.reserve(mCachedCapacity);
    colliderEntities1.reserve(mCachedCapacity);
    colliderEntities2.reserve(mCachedCapacity);
    shape1ToWorldTransforms.reserve(mCachedCapacity);
    shape2ToWorldTransforms.reserve(mCachedCapacity);
    lastFrameCollisionInfos.reserve(mCachedCapacity);
    isColliding.reserve(mCachedCapacity);
    contactPoints.reserve(mCachedCapacity);

    sphere1Radiuses.reserve(mCachedCapacity);
    sphere2Radiuses.reserve(mCachedCapacity);
}

// Clear all the objects in the batch
void SphereVsSphereNarrowPhaseInfoBatch::clear() {

    // Note that we clear the following containers and we release their allocated memory. Therefore,
    // if the memory allocator is a single frame allocator, the memory is deallocated and will be
    // allocated in the next frame at a possibly different location in memory (remember that the
    // location of the allocated memory of a single frame allocator might change between two frames)

    mCachedCapacity = overlappingPairIds.size();

    overlappingPairIds.clear(true);
    colliderEntities1.clear(true);
    colliderEntities2.clear(true);
    shape1ToWorldTransforms.clear(true);
    shape2ToWorldTransforms.clear(true);
    lastFrameCollisionInfos.clear(true);
    isColliding.clear(true);
    contactPoints.clear(true);

    sphere1Radiuses.clear(true);
    sphere2Radiuses.clear(true);
}

