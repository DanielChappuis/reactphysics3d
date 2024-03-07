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
                                   BodyComponents& bodyComponents, RigidBodyComponents& rigidBodyComponents, Set<bodypair> &noCollisionPairs, CollisionDispatch &collisionDispatch)
                : mPoolAllocator(memoryManager.getPoolAllocator()), mHeapAllocator(memoryManager.getHeapAllocator()), mConvexPairs(memoryManager.getHeapAllocator()),
                  mConcavePairs(memoryManager.getHeapAllocator()), mDisabledConvexPairs(memoryManager.getHeapAllocator()), mDisabledConcavePairs(memoryManager.getHeapAllocator()), mMapConvexPairIdToPairIndex(memoryManager.getHeapAllocator()), mMapConcavePairIdToPairIndex(memoryManager.getHeapAllocator()),
                  mMapDisabledConvexPairIdToPairIndex(memoryManager.getHeapAllocator()), mMapDisabledConcavePairIdToPairIndex(memoryManager.getHeapAllocator()),
                  mColliderComponents(colliderComponents), mBodyComponents(bodyComponents),
                  mRigidBodyComponents(rigidBodyComponents), mNoCollisionPairs(noCollisionPairs), mCollisionDispatch(collisionDispatch) {
    
}

// Destructor
OverlappingPairs::~OverlappingPairs() {

    // Destroy the convex pairs
    while (mConvexPairs.size() > 0) {

        removeConvexPairWithIndex(mConvexPairs.size() - 1, true);
    }

    // Destroy the concave pairs
    while (mConcavePairs.size() > 0) {

        removeConcavePairWithIndex(mConcavePairs.size() - 1, true);
    }

    // Destroy the disabled convex pairs
    while (mDisabledConvexPairs.size() > 0) {

        removeDisabledConvexPairWithIndex(mDisabledConvexPairs.size() - 1, true);
    }

    // Destroy the disabled concave pairs
    while (mDisabledConcavePairs.size() > 0) {

        removeDisabledConcavePairWithIndex(mDisabledConcavePairs.size() - 1, true);
    }
}

// Remove an overlapping pair
void OverlappingPairs::removePair(uint64 pairId) {

    RP3D_PROFILE("OverlappingPairs::removePair()", mProfiler);

    assert(mMapConvexPairIdToPairIndex.containsKey(pairId) || mMapConcavePairIdToPairIndex.containsKey(pairId) ||
           mMapDisabledConvexPairIdToPairIndex.containsKey(pairId) || mMapDisabledConcavePairIdToPairIndex.containsKey(pairId));

    auto it = mMapConvexPairIdToPairIndex.find(pairId);
    if (it != mMapConvexPairIdToPairIndex.end()) {
        removeConvexPairWithIndex(it->second, true);
        return;
    }

    auto it2 = mMapConcavePairIdToPairIndex.find(pairId);
    if (it2 != mMapConcavePairIdToPairIndex.end()) {
        removeConcavePairWithIndex(it2->second, true);
        return;
    }

    auto it3 = mMapDisabledConvexPairIdToPairIndex.find(pairId);
    if (it3 != mMapDisabledConvexPairIdToPairIndex.end()) {
        removeDisabledConvexPairWithIndex(it3->second, true);
        return;
    }

    auto it4 = mMapDisabledConcavePairIdToPairIndex.find(pairId);
    if (it4 != mMapDisabledConcavePairIdToPairIndex.end()) {
        removeDisabledConcavePairWithIndex(it4->second, true);
        return;
    }
}

// Remove an disabled convex overlapping pair
void OverlappingPairs::removeDisabledConvexPairWithIndex(uint64 pairIndex, bool removeFromColliders) {

    RP3D_PROFILE("OverlappingPairs::removeDisabledConvexPairWithIndex()", mProfiler);

    // Remove the involved overlapping pair from the two colliders
    assert(mColliderComponents.getOverlappingPairs(mDisabledConvexPairs[pairIndex].collider1).find(mDisabledConvexPairs[pairIndex].pairID) != mColliderComponents.getOverlappingPairs(mDisabledConvexPairs[pairIndex].collider1).end());
    assert(mColliderComponents.getOverlappingPairs(mDisabledConvexPairs[pairIndex].collider2).find(mDisabledConvexPairs[pairIndex].pairID) != mColliderComponents.getOverlappingPairs(mDisabledConvexPairs[pairIndex].collider2).end());
    if (removeFromColliders) {

        mColliderComponents.getOverlappingPairs(mDisabledConvexPairs[pairIndex].collider1).remove(mDisabledConvexPairs[pairIndex].pairID);
        mColliderComponents.getOverlappingPairs(mDisabledConvexPairs[pairIndex].collider2).remove(mDisabledConvexPairs[pairIndex].pairID);
    }

    assert(mMapDisabledConvexPairIdToPairIndex[mDisabledConvexPairs[pairIndex].pairID] == pairIndex);
    mMapDisabledConvexPairIdToPairIndex.remove(mDisabledConvexPairs[pairIndex].pairID);

    const uint64 nbDisabledPairs = mDisabledConvexPairs.size();

    // Change the mapping between the pairId and the index in the disabled pairs array if we swap the last item with the one to remove
    if (mDisabledConvexPairs.size() > 1 && pairIndex < (nbDisabledPairs - 1)) {

        mMapDisabledConvexPairIdToPairIndex[mDisabledConvexPairs[nbDisabledPairs - 1].pairID] = pairIndex;
    }

    // We want to keep the arrays tightly packed. Therefore, when a pair is removed,
    // we replace it with the last element of the array.
    mDisabledConvexPairs.removeAtAndReplaceByLast(pairIndex);
}

// Remove an disabled concave overlapping pair
void OverlappingPairs::removeDisabledConcavePairWithIndex(uint64 pairIndex, bool removeFromColliders) {

    RP3D_PROFILE("OverlappingPairs::removeDisabledConcavePairWithIndex()", mProfiler);

    // Remove the involved overlapping pair from the two colliders
    assert(mColliderComponents.getOverlappingPairs(mDisabledConcavePairs[pairIndex].collider1).find(mDisabledConcavePairs[pairIndex].pairID) != mColliderComponents.getOverlappingPairs(mDisabledConcavePairs[pairIndex].collider1).end());
    assert(mColliderComponents.getOverlappingPairs(mDisabledConcavePairs[pairIndex].collider2).find(mDisabledConcavePairs[pairIndex].pairID) != mColliderComponents.getOverlappingPairs(mDisabledConcavePairs[pairIndex].collider2).end());
    if (removeFromColliders) {

        mColliderComponents.getOverlappingPairs(mDisabledConcavePairs[pairIndex].collider1).remove(mDisabledConcavePairs[pairIndex].pairID);
        mColliderComponents.getOverlappingPairs(mDisabledConcavePairs[pairIndex].collider2).remove(mDisabledConcavePairs[pairIndex].pairID);
    }

    assert(mMapDisabledConcavePairIdToPairIndex[mDisabledConcavePairs[pairIndex].pairID] == pairIndex);
    mMapDisabledConcavePairIdToPairIndex.remove(mDisabledConcavePairs[pairIndex].pairID);

    const uint64 nbDisabledPairs = mDisabledConcavePairs.size();

    // Change the mapping between the pairId and the index in the disabled pairs array if we swap the last item with the one to remove
    if (mDisabledConcavePairs.size() > 1 && pairIndex < (nbDisabledPairs - 1)) {

        mMapDisabledConcavePairIdToPairIndex[mDisabledConcavePairs[nbDisabledPairs - 1].pairID] = pairIndex;
    }

    // We want to keep the arrays tightly packed. Therefore, when a pair is removed,
    // we replace it with the last element of the array.
    mDisabledConcavePairs.removeAtAndReplaceByLast(pairIndex);
}

// Remove a convex pair at a given index
void OverlappingPairs::removeConvexPairWithIndex(uint64 pairIndex, bool removeFromColliders) {

    RP3D_PROFILE("OverlappingPairs::removeConvexPairWithIndex()", mProfiler);

    const uint64 nbConvexPairs = mConvexPairs.size();

    assert(pairIndex < nbConvexPairs);

    assert(mColliderComponents.getOverlappingPairs(mConvexPairs[pairIndex].collider1).find(mConvexPairs[pairIndex].pairID) != mColliderComponents.getOverlappingPairs(mConvexPairs[pairIndex].collider1).end());
    assert(mColliderComponents.getOverlappingPairs(mConvexPairs[pairIndex].collider2).find(mConvexPairs[pairIndex].pairID) != mColliderComponents.getOverlappingPairs(mConvexPairs[pairIndex].collider2).end());

    if (removeFromColliders) {

        // Remove the involved overlapping pair from the two colliders
        mColliderComponents.getOverlappingPairs(mConvexPairs[pairIndex].collider1).remove(mConvexPairs[pairIndex].pairID);
        mColliderComponents.getOverlappingPairs(mConvexPairs[pairIndex].collider2).remove(mConvexPairs[pairIndex].pairID);
    }

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

// Remove a concave pair at a given index
void OverlappingPairs::removeConcavePairWithIndex(uint64 pairIndex, bool removeFromColliders) {

    RP3D_PROFILE("OverlappingPairs::removeConcavePairWithIndex()", mProfiler);

    const uint64 nbConcavePairs = mConcavePairs.size();

    assert(pairIndex < nbConcavePairs);

    assert(mColliderComponents.getOverlappingPairs(mConcavePairs[pairIndex].collider1).find(mConcavePairs[pairIndex].pairID) != mColliderComponents.getOverlappingPairs(mConcavePairs[pairIndex].collider1).end());
    assert(mColliderComponents.getOverlappingPairs(mConcavePairs[pairIndex].collider2).find(mConcavePairs[pairIndex].pairID) != mColliderComponents.getOverlappingPairs(mConcavePairs[pairIndex].collider2).end());

    if (removeFromColliders) {

        // Remove the involved overlapping pair from the two colliders
        mColliderComponents.getOverlappingPairs(mConcavePairs[pairIndex].collider1).remove(mConcavePairs[pairIndex].pairID);
        mColliderComponents.getOverlappingPairs(mConcavePairs[pairIndex].collider2).remove(mConcavePairs[pairIndex].pairID);
    }

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

// Enable an overlapping pair (because at least one body of the pair is awaken or not static anymore)
void OverlappingPairs::enablePair(uint64 pairId) {

    RP3D_PROFILE("OverlappingPairs::enablePair()", mProfiler);

    assert(isPairDisabled(pairId));
    assert(mMapConvexPairIdToPairIndex.find(pairId) == mMapConvexPairIdToPairIndex.end() &&
           mMapConcavePairIdToPairIndex.find(pairId) == mMapConcavePairIdToPairIndex.end());

    // Get the existing overlapping pair from the convex/concave array
    auto it = mMapDisabledConvexPairIdToPairIndex.find(pairId);
    if (it != mMapDisabledConvexPairIdToPairIndex.end()) {

        // Enable the pair
        enableConvexPairWithIndex(it->second);
    }
    else {
        it = mMapDisabledConcavePairIdToPairIndex.find(pairId);
        if (it != mMapDisabledConcavePairIdToPairIndex.end()) {

            // Enable the pair
            enableConcavePairWithIndex(it->second);
        }
        else {
            // Should not happen
            assert(false);
            return;
        }
    }
}

// Disable an overlapping pair (because both bodies of the pair are disabled)
void OverlappingPairs::disablePair(uint64 pairId) {

    RP3D_PROFILE("OverlappingPairs::disablePair()", mProfiler);

    assert(!isPairDisabled(pairId));
    assert(mMapConvexPairIdToPairIndex.find(pairId) != mMapConvexPairIdToPairIndex.end() ||
           mMapConcavePairIdToPairIndex.find(pairId) != mMapConcavePairIdToPairIndex.end());

    // Get the existing overlapping pair from the convex/concave array
    auto it = mMapConvexPairIdToPairIndex.find(pairId);
    if (it != mMapConvexPairIdToPairIndex.end()) {

        // Disable the pair
        disableConvexPairWithIndex(it->second);
    }
    else {
        it = mMapConcavePairIdToPairIndex.find(pairId);
        if (it != mMapConcavePairIdToPairIndex.end()) {

            // Disable the pair
            disableConcavePairWithIndex(it->second);
        }
        else {
            // Should not happen
            assert(false);
            return;
        }
    }
}

// Enable a convex overlapping pair
void OverlappingPairs::enableConvexPairWithIndex(uint64 pairIndex) {

    RP3D_PROFILE("OverlappingPairs::enableConvexPairWithIndex()", mProfiler);

    ConvexOverlappingPair* pair = &(mDisabledConvexPairs[static_cast<uint32>(pairIndex)]);

    assert(!pair->isEnabled);

    const uint64 newPairIndex = mConvexPairs.size();

    // Map the entity with the new pair lookup index
    mMapConvexPairIdToPairIndex.add(Pair<uint64, uint64>(pair->pairID, newPairIndex));

    // Create a new pair to be added into the array of pairs
    mConvexPairs.emplace(pair->pairID, pair->broadPhaseId1, pair->broadPhaseId2, pair->collider1, pair->collider2,
                            pair->narrowPhaseAlgorithmType, true);
    mConvexPairs[newPairIndex].collidingInCurrentFrame = pair->collidingInCurrentFrame;
    mConvexPairs[newPairIndex].collidingInPreviousFrame = pair->collidingInPreviousFrame;

    // Remove the previous overlapping pair from the convex array
    removeDisabledConvexPairWithIndex(pairIndex, false);
}

// Disable a convex overlapping pair (because both bodies of the pair are disabled)
void OverlappingPairs::disableConvexPairWithIndex(uint64 pairIndex) {

    RP3D_PROFILE("OverlappingPairs::disableConvexPairWithIndex()", mProfiler);

    ConvexOverlappingPair* pair = &(mConvexPairs[static_cast<uint32>(pairIndex)]);

    assert(pair->isEnabled);

    const uint64 newPairIndex = mDisabledConvexPairs.size();

    // Map the entity with the new pair lookup index
    mMapDisabledConvexPairIdToPairIndex.add(Pair<uint64, uint64>(pair->pairID, newPairIndex));

    // Create a new pair to be added into the array of disable pairs
    mDisabledConvexPairs.emplace(pair->pairID, pair->broadPhaseId1, pair->broadPhaseId2, pair->collider1, pair->collider2,
                            pair->narrowPhaseAlgorithmType, false);
    mDisabledConvexPairs[newPairIndex].collidingInCurrentFrame = pair->collidingInCurrentFrame;
    mDisabledConvexPairs[newPairIndex].collidingInPreviousFrame = pair->collidingInPreviousFrame;

    // Remove the previous overlapping pair from the convex array
    removeConvexPairWithIndex(pairIndex, false);
}

// Enable a concave overlapping pair
void OverlappingPairs::enableConcavePairWithIndex(uint64 pairIndex) {

    RP3D_PROFILE("OverlappingPairs::enableConcavePairWithIndex()", mProfiler);

    ConcaveOverlappingPair* pair = &(mDisabledConcavePairs[static_cast<uint32>(pairIndex)]);

    assert(!pair->isEnabled);

    const uint64 newPairIndex = mConcavePairs.size();

    // Map the entity with the new pair lookup index
    mMapConcavePairIdToPairIndex.add(Pair<uint64, uint64>(pair->pairID, newPairIndex));

    // Create a new pair to be added into the array of disable pairs
    mConcavePairs.emplace(pair->pairID, pair->broadPhaseId1, pair->broadPhaseId2, pair->collider1, pair->collider2,
                            pair->narrowPhaseAlgorithmType, pair->isShape1Convex, mPoolAllocator, mHeapAllocator, true);
    mConcavePairs[newPairIndex].collidingInCurrentFrame = pair->collidingInCurrentFrame;
    mConcavePairs[newPairIndex].collidingInPreviousFrame = pair->collidingInPreviousFrame;

    // Remove the previous overlapping pair from the concave array
    removeDisabledConcavePairWithIndex(pairIndex, false);
}

// Disable a concave overlapping pair (because both bodies of the pair are disabled)
void OverlappingPairs::disableConcavePairWithIndex(uint64 pairIndex) {

    RP3D_PROFILE("OverlappingPairs::disableConcavePairWithIndex()", mProfiler);

    ConcaveOverlappingPair* pair = &(mConcavePairs[static_cast<uint32>(pairIndex)]);

    assert(pair->isEnabled);

    const uint64 newPairIndex = mDisabledConcavePairs.size();

    // Map the entity with the new pair lookup index
    mMapDisabledConcavePairIdToPairIndex.add(Pair<uint64, uint64>(pair->pairID, newPairIndex));

    // Create a new pair to be added into the array of disable pairs
    mDisabledConcavePairs.emplace(pair->pairID, pair->broadPhaseId1, pair->broadPhaseId2, pair->collider1, pair->collider2,
                            pair->narrowPhaseAlgorithmType, pair->isShape1Convex, mPoolAllocator, mHeapAllocator, false, false);
    mDisabledConcavePairs[newPairIndex].collidingInCurrentFrame = pair->collidingInCurrentFrame;
    mDisabledConcavePairs[newPairIndex].collidingInPreviousFrame = pair->collidingInPreviousFrame;

    // Remove the previous overlapping pair from the concave array
    removeConcavePairWithIndex(pairIndex, false);
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
        mConvexPairs.emplace(pairId, broadPhase1Id, broadPhase2Id, collider1Entity, collider2Entity, algorithmType, true);
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
                              isShape1Convex, mPoolAllocator, mHeapAllocator, true);
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
