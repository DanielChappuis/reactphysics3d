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
NarrowPhaseInfoBatch::NarrowPhaseInfoBatch(OverlappingPairs& overlappingPairs, MemoryAllocator& allocator)
                     : mMemoryAllocator(allocator), mOverlappingPairs(overlappingPairs), narrowPhaseInfos(allocator){

}

// Destructor
NarrowPhaseInfoBatch::~NarrowPhaseInfoBatch() {
    clear();
}

// Add shapes to be tested during narrow-phase collision detection into the batch
void NarrowPhaseInfoBatch::addNarrowPhaseInfo(uint64 pairId, uint64 pairIndex, Entity collider1, Entity collider2, CollisionShape* shape1,
                                              CollisionShape* shape2, const Transform& shape1Transform, const Transform& shape2Transform,
                                              bool needToReportContacts, MemoryAllocator& shapeAllocator) {

    // Add a collision info for the two collision shapes into the overlapping pair (if not present yet)
    // TODO OPTI : Can we better manage this
    LastFrameCollisionInfo* lastFrameInfo = mOverlappingPairs.addLastFrameInfoIfNecessary(pairIndex, shape1->getId(), shape2->getId());

    // Create a meta data object
    narrowPhaseInfos.emplace(pairId, collider1, collider2, lastFrameInfo, shapeAllocator, shape1Transform, shape2Transform, shape1, shape2, needToReportContacts);
}

// Add a new contact point
void NarrowPhaseInfoBatch::addContactPoint(uint index, const Vector3& contactNormal, decimal penDepth, const Vector3& localPt1, const Vector3& localPt2) {

    assert(penDepth > decimal(0.0));

    if (narrowPhaseInfos[index].nbContactPoints < NB_MAX_CONTACT_POINTS_IN_NARROWPHASE_INFO) {

        assert(contactNormal.length() > 0.8f);

        // Add it into the array of contact points
        narrowPhaseInfos[index].contactPoints[narrowPhaseInfos[index].nbContactPoints].normal = contactNormal;
        narrowPhaseInfos[index].contactPoints[narrowPhaseInfos[index].nbContactPoints].penetrationDepth = penDepth;
        narrowPhaseInfos[index].contactPoints[narrowPhaseInfos[index].nbContactPoints].localPoint1 = localPt1;
        narrowPhaseInfos[index].contactPoints[narrowPhaseInfos[index].nbContactPoints].localPoint2 = localPt2;
        narrowPhaseInfos[index].nbContactPoints++;
    }
}

// Reset the remaining contact points
void NarrowPhaseInfoBatch::resetContactPoints(uint index) {

    narrowPhaseInfos[index].nbContactPoints = 0;
}

// Initialize the containers using cached capacity
void NarrowPhaseInfoBatch::reserveMemory() {

    narrowPhaseInfos.reserve(mCachedCapacity);
}

// Clear all the objects in the batch
void NarrowPhaseInfoBatch::clear() {

    // TODO OPTI : Better manage this
    for (uint i=0; i < narrowPhaseInfos.size(); i++) {

        assert(narrowPhaseOutputInfos[i].nbContactPoints == 0);

        // Release the memory of the TriangleShape (this memory was allocated in the
        // MiddlePhaseTriangleCallback::testTriangle() method)
        if (narrowPhaseInfos[i].collisionShape1->getName() == CollisionShapeName::TRIANGLE) {
            narrowPhaseInfos[i].collisionShape1->~CollisionShape();
            narrowPhaseInfos[i].collisionShapeAllocator->release(narrowPhaseInfos[i].collisionShape1, sizeof(TriangleShape));
        }
        if (narrowPhaseInfos[i].collisionShape2->getName() == CollisionShapeName::TRIANGLE) {
            narrowPhaseInfos[i].collisionShape2->~CollisionShape();
            narrowPhaseInfos[i].collisionShapeAllocator->release(narrowPhaseInfos[i].collisionShape2, sizeof(TriangleShape));
        }
    }

    // Note that we clear the following containers and we release their allocated memory. Therefore,
    // if the memory allocator is a single frame allocator, the memory is deallocated and will be
    // allocated in the next frame at a possibly different location in memory (remember that the
    // location of the allocated memory of a single frame allocator might change between two frames)

    mCachedCapacity = narrowPhaseInfos.size();

    narrowPhaseInfos.clear(true);
}
