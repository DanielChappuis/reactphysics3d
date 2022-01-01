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
#include <cassert>
#include <reactphysics3d/engine/OverlappingPairs.h>
#include <reactphysics3d/containers/containers_common.h>
#include <reactphysics3d/collision/ContactPointInfo.h>
#include <reactphysics3d/collision/narrowphase/NarrowPhaseAlgorithm.h>
#include <reactphysics3d/collision/narrowphase/CollisionDispatch.h>
#include <reactphysics3d/memory/MemoryManager.h>

using namespace reactphysics3d;

// Constructor
OverlappingPairs::OverlappingPairs(MemoryManager& memoryManager, ColliderComponents& colliderComponents,
                                   CollisionBodyComponents& collisionBodyComponents, RigidBodyComponents& rigidBodyComponents, Set<bodypair> &noCollisionPairs, CollisionDispatch &collisionDispatch)
                : mPoolAllocator(memoryManager.getPoolAllocator()), mHeapAllocator(memoryManager.getHeapAllocator()), mConvexPairs(memoryManager.getHeapAllocator()),
                  mConcavePairs(memoryManager.getHeapAllocator()), mMapConvexPairIdToPairIndex(memoryManager.getHeapAllocator()), mMapConcavePairIdToPairIndex(memoryManager.getHeapAllocator()),
                  mColliderComponents(colliderComponents), mCollisionBodyComponents(collisionBodyComponents),
                  mRigidBodyComponents(rigidBodyComponents), mNoCollisionPairs(noCollisionPairs), mCollisionDispatch(collisionDispatch) {
    
}

// Destructor
OverlappingPairs::~OverlappingPairs() {

    // Destroy the convex pairs
    while (mConvexPairs.size() > 0) {

        removePair(mConvexPairs.size() - 1, true);
    }

    // Destroy the concave pairs
    while (mConcavePairs.size() > 0) {

        removePair(mConcavePairs.size() - 1, false);
    }
}

// Remove a component at a given index
void OverlappingPairs::removePair(uint64 pairId) {

    assert(mMapConvexPairIdToPairIndex.containsKey(pairId) || mMapConcavePairIdToPairIndex.containsKey(pairId));

    auto it = mMapConvexPairIdToPairIndex.find(pairId);
    if (it != mMapConvexPairIdToPairIndex.end()) {
        removePair(it->second, true);
    }
    else {
        removePair(mMapConcavePairIdToPairIndex[pairId], false);
    }
}

// Remove a component at a given index
void OverlappingPairs::removePair(uint64 pairIndex, bool isConvexVsConvex) {

    RP3D_PROFILE("OverlappingPairs::removePair()", mProfiler);

    if (isConvexVsConvex) {

        const uint64 nbConvexPairs = mConvexPairs.size();

        assert(pairIndex < nbConvexPairs);

        // Remove the involved overlapping pair from the two colliders
        assert(mColliderComponents.getOverlappingPairs(mConvexPairs[pairIndex].collider1).find(mConvexPairs[pairIndex].pairID) != mColliderComponents.getOverlappingPairs(mConvexPairs[pairIndex].collider1).end());
        assert(mColliderComponents.getOverlappingPairs(mConvexPairs[pairIndex].collider2).find(mConvexPairs[pairIndex].pairID) != mColliderComponents.getOverlappingPairs(mConvexPairs[pairIndex].collider2).end());
        mColliderComponents.getOverlappingPairs(mConvexPairs[pairIndex].collider1).remove(mConvexPairs[pairIndex].pairID);
        mColliderComponents.getOverlappingPairs(mConvexPairs[pairIndex].collider2).remove(mConvexPairs[pairIndex].pairID);

        assert(mMapConvexPairIdToPairIndex[mConvexPairs[pairIndex].pairID] == pairIndex);
        mMapConvexPairIdToPairIndex.remove(mConvexPairs[pairIndex].pairID);

        // Change the mapping between the pairId and the index in the convex pairs array if we swap the last item with the one to remove
        if (mConvexPairs.size() > 1 && pairIndex < (nbConvexPairs - 1)) {

            mMapConvexPairIdToPairIndex[mConvexPairs[nbConvexPairs - 1].pairID] = pairIndex;
        }

        // We want to keep the arrays tightly packed. Therefore, when a pair is removed,
        // we replace it with the last element of the array.
        mConvexPairs.removeAtAndReplaceByLast(pairIndex);
    }
    else {

        const uint64 nbConcavePairs = mConcavePairs.size();

        assert(pairIndex < nbConcavePairs);

        // Remove the involved overlapping pair to the two colliders
        assert(mColliderComponents.getOverlappingPairs(mConcavePairs[pairIndex].collider1).find(mConcavePairs[pairIndex].pairID) != mColliderComponents.getOverlappingPairs(mConcavePairs[pairIndex].collider1).end());
        assert(mColliderComponents.getOverlappingPairs(mConcavePairs[pairIndex].collider2).find(mConcavePairs[pairIndex].pairID) != mColliderComponents.getOverlappingPairs(mConcavePairs[pairIndex].collider2).end());
        mColliderComponents.getOverlappingPairs(mConcavePairs[pairIndex].collider1).remove(mConcavePairs[pairIndex].pairID);
        mColliderComponents.getOverlappingPairs(mConcavePairs[pairIndex].collider2).remove(mConcavePairs[pairIndex].pairID);

        assert(mMapConcavePairIdToPairIndex[mConcavePairs[pairIndex].pairID] == pairIndex);
        mMapConcavePairIdToPairIndex.remove(mConcavePairs[pairIndex].pairID);

        // Destroy all the LastFrameCollisionInfo objects
        mConcavePairs[pairIndex].destroyLastFrameCollisionInfos();

        // Change the mapping between the pairId and the index in the convex pairs array if we swap the last item with the one to remove
        if (mConcavePairs.size() > 1 && pairIndex < (nbConcavePairs - 1)) {

            mMapConcavePairIdToPairIndex[mConcavePairs[nbConcavePairs - 1].pairID] = pairIndex;
        }

        // We want to keep the arrays tightly packed. Therefore, when a pair is removed,
        // we replace it with the last element of the array.
        mConcavePairs.removeAtAndReplaceByLast(pairIndex);
    }
}

// Add an overlapping pair
uint64 OverlappingPairs::addPair(uint32 collider1Index, uint32 collider2Index, bool isConvexVsConvex) {

    RP3D_PROFILE("OverlappingPairs::addPair()", mProfiler);

    assert(mColliderComponents.mBroadPhaseIds[collider1Index] >= 0 && mColliderComponents.mBroadPhaseIds[collider2Index] >= 0);

    const CollisionShape* collisionShape1 = mColliderComponents.mCollisionShapes[collider1Index];
    const CollisionShape* collisionShape2 = mColliderComponents.mCollisionShapes[collider2Index];

    const Entity collider1Entity = mColliderComponents.mCollidersEntities[collider1Index];
    const Entity collider2Entity = mColliderComponents.mCollidersEntities[collider2Index];

    const uint32 broadPhase1Id = static_cast<uint32>(mColliderComponents.mBroadPhaseIds[collider1Index]);
    const uint32 broadPhase2Id = static_cast<uint32>(mColliderComponents.mBroadPhaseIds[collider2Index]);

    // Compute a unique id for the overlapping pair
    const uint64 pairId = pairNumbers(std::max(broadPhase1Id, broadPhase2Id), std::min(broadPhase1Id, broadPhase2Id));

    // Select the narrow phase algorithm to use according to the two collision shapes
    if (isConvexVsConvex) {

        assert(!mMapConvexPairIdToPairIndex.containsKey(pairId));
        NarrowPhaseAlgorithmType algorithmType = mCollisionDispatch.selectNarrowPhaseAlgorithm(collisionShape1->getType(), collisionShape2->getType());

        // Map the entity with the new component lookup index
        mMapConvexPairIdToPairIndex.add(Pair<uint64, uint64>(pairId, mConvexPairs.size()));

        // Create and add a new convex pair
        mConvexPairs.emplace(pairId, broadPhase1Id, broadPhase2Id, collider1Entity, collider2Entity, algorithmType);
    }
    else {

        const bool isShape1Convex = collisionShape1->isConvex();

        assert(!mMapConcavePairIdToPairIndex.containsKey(pairId));
        NarrowPhaseAlgorithmType algorithmType = mCollisionDispatch.selectNarrowPhaseAlgorithm(isShape1Convex ? collisionShape1->getType() : collisionShape2->getType(),
                                                                      CollisionShapeType::CONVEX_POLYHEDRON);
        // Map the entity with the new component lookup index
        mMapConcavePairIdToPairIndex.add(Pair<uint64, uint64>(pairId, mConcavePairs.size()));

        // Create and add a new concave pair
        mConcavePairs.emplace(pairId, broadPhase1Id, broadPhase2Id, collider1Entity, collider2Entity, algorithmType,
                              isShape1Convex, mPoolAllocator, mHeapAllocator);
    }

    // Add the involved overlapping pair to the two colliders
    assert(mColliderComponents.mOverlappingPairs[collider1Index].find(pairId) == mColliderComponents.mOverlappingPairs[collider1Index].end());
    assert(mColliderComponents.mOverlappingPairs[collider2Index].find(pairId) == mColliderComponents.mOverlappingPairs[collider2Index].end());
    mColliderComponents.mOverlappingPairs[collider1Index].add(pairId);
    mColliderComponents.mOverlappingPairs[collider2Index].add(pairId);

    return pairId;
}

// Delete all the obsolete last frame collision info
void OverlappingPairs::clearObsoleteLastFrameCollisionInfos() {

    RP3D_PROFILE("OverlappingPairs::clearObsoleteLastFrameCollisionInfos()", mProfiler);

    // For each concave overlapping pair
    const uint64 nbConcavePairs = mConcavePairs.size();
    for (uint64 i=0; i < nbConcavePairs; i++) {

        mConcavePairs[i].clearObsoleteLastFrameInfos();
    }
}

// Set the collidingInPreviousFrame value with the collidinginCurrentFrame value for each pair
void OverlappingPairs::updateCollidingInPreviousFrame() {

    RP3D_PROFILE("OverlappingPairs::updateCollidingInPreviousFrame()", mProfiler);

    // For each convex overlapping pair
    const uint64 nbConvexPairs = mConvexPairs.size();
    for (uint64 i=0; i < nbConvexPairs; i++) {

        mConvexPairs[i].collidingInPreviousFrame = mConvexPairs[i].collidingInCurrentFrame;
    }

    // For each concave overlapping pair
    const uint64 nbConcavePairs = mConcavePairs.size();
    for (uint64 i=0; i < nbConcavePairs; i++) {

        mConcavePairs[i].collidingInPreviousFrame = mConcavePairs[i].collidingInCurrentFrame;
    }
}
