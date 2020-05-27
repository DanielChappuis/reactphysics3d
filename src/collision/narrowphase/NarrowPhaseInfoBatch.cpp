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
#include <reactphysics3d/collision/narrowphase/NarrowPhaseInfoBatch.h>
#include <reactphysics3d/collision/ContactPointInfo.h>
#include <reactphysics3d/collision/shapes/TriangleShape.h>
#include <reactphysics3d/engine/OverlappingPairs.h>
#include <iostream>

using namespace reactphysics3d;

// Constructor
NarrowPhaseInfoBatch::NarrowPhaseInfoBatch(MemoryAllocator& allocator, OverlappingPairs& overlappingPairs)
      : mMemoryAllocator(allocator), mOverlappingPairs(overlappingPairs), overlappingPairIds(allocator),
        colliderEntities1(allocator), colliderEntities2(allocator), collisionShapes1(allocator), collisionShapes2(allocator),
        shape1ToWorldTransforms(allocator), shape2ToWorldTransforms(allocator), reportContacts(allocator),
        isColliding(allocator), contactPoints(allocator), collisionShapeAllocators(allocator),
        lastFrameCollisionInfos(allocator) {

}

// Destructor
NarrowPhaseInfoBatch::~NarrowPhaseInfoBatch() {
    clear();
}

// Add shapes to be tested during narrow-phase collision detection into the batch
void NarrowPhaseInfoBatch::addNarrowPhaseInfo(uint64 pairId, uint64 pairIndex, Entity collider1, Entity collider2, CollisionShape* shape1, CollisionShape* shape2,
                                              const Transform& shape1Transform, const Transform& shape2Transform, bool needToReportContacts,
                                              MemoryAllocator& shapeAllocator) {

    overlappingPairIds.add(pairId);
    colliderEntities1.add(collider1);
    colliderEntities2.add(collider2);
    collisionShapes1.add(shape1);
    collisionShapes2.add(shape2);
    shape1ToWorldTransforms.add(shape1Transform);
    shape2ToWorldTransforms.add(shape2Transform);
    reportContacts.add(needToReportContacts);
    collisionShapeAllocators.add(&shapeAllocator);
    contactPoints.add(List<ContactPointInfo*>(mMemoryAllocator));
    isColliding.add(false);

    // Add a collision info for the two collision shapes into the overlapping pair (if not present yet)
    LastFrameCollisionInfo* lastFrameInfo = mOverlappingPairs.addLastFrameInfoIfNecessary(pairIndex, shape1->getId(), shape2->getId());
    lastFrameCollisionInfos.add(lastFrameInfo);
}

// Add a new contact point
void NarrowPhaseInfoBatch::addContactPoint(uint index, const Vector3& contactNormal, decimal penDepth,
                     const Vector3& localPt1, const Vector3& localPt2) {

    assert(reportContacts[index]);
    assert(penDepth > decimal(0.0));

    // Get the memory allocator
    MemoryAllocator& allocator = mOverlappingPairs.getTemporaryAllocator();

    // Create the contact point info
    ContactPointInfo* contactPointInfo = new (allocator.allocate(sizeof(ContactPointInfo)))
            ContactPointInfo(contactNormal, penDepth, localPt1, localPt2);

    // Add it into the list of contact points
    contactPoints[index].add(contactPointInfo);
}

// Reset the remaining contact points
void NarrowPhaseInfoBatch::resetContactPoints(uint index) {

    // Get the memory allocator
    MemoryAllocator& allocator = mOverlappingPairs.getTemporaryAllocator();

    // For each remaining contact point info
    for (uint i=0; i < contactPoints[index].size(); i++) {

        ContactPointInfo* contactPoint = contactPoints[index][i];

        // Call the destructor
        contactPoint->~ContactPointInfo();

        // Delete the current element
        allocator.release(contactPoint, sizeof(ContactPointInfo));
    }

    contactPoints[index].clear();
}

// Initialize the containers using cached capacity
void NarrowPhaseInfoBatch::reserveMemory() {

    overlappingPairIds.reserve(mCachedCapacity);
    colliderEntities1.reserve(mCachedCapacity);
    colliderEntities2.reserve(mCachedCapacity);
    collisionShapes1.reserve(mCachedCapacity);
    collisionShapes2.reserve(mCachedCapacity);
    shape1ToWorldTransforms.reserve(mCachedCapacity);
    shape2ToWorldTransforms.reserve(mCachedCapacity);
    reportContacts.reserve(mCachedCapacity);
    collisionShapeAllocators.reserve(mCachedCapacity);
    lastFrameCollisionInfos.reserve(mCachedCapacity);
    isColliding.reserve(mCachedCapacity);
    contactPoints.reserve(mCachedCapacity);
}

// Clear all the objects in the batch
void NarrowPhaseInfoBatch::clear() {

    for (uint i=0; i < overlappingPairIds.size(); i++) {

        assert(contactPoints[i].size() == 0);

        // Release the memory of the TriangleShape (this memory was allocated in the
        // MiddlePhaseTriangleCallback::testTriangle() method)
        if (collisionShapes1.size() > 0 && collisionShapes1[i]->getName() == CollisionShapeName::TRIANGLE) {
            collisionShapes1[i]->~CollisionShape();
            collisionShapeAllocators[i]->release(collisionShapes1[i], sizeof(TriangleShape));
        }
        if (collisionShapes2.size() > 0 && collisionShapes2[i]->getName() == CollisionShapeName::TRIANGLE) {
            collisionShapes2[i]->~CollisionShape();
            collisionShapeAllocators[i]->release(collisionShapes2[i], sizeof(TriangleShape));
        }
    }

    // Note that we clear the following containers and we release their allocated memory. Therefore,
    // if the memory allocator is a single frame allocator, the memory is deallocated and will be
    // allocated in the next frame at a possibly different location in memory (remember that the
    // location of the allocated memory of a single frame allocator might change between two frames)

    mCachedCapacity = overlappingPairIds.size();

    overlappingPairIds.clear(true);
    colliderEntities1.clear(true);
    colliderEntities2.clear(true);
    collisionShapes1.clear(true);
    collisionShapes2.clear(true);
    shape1ToWorldTransforms.clear(true);
    shape2ToWorldTransforms.clear(true);
    reportContacts.clear(true);
    collisionShapeAllocators.clear(true);
    lastFrameCollisionInfos.clear(true);
    isColliding.clear(true);
    contactPoints.clear(true);
}
