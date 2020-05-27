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
#include <cassert>
#include <reactphysics3d/engine/OverlappingPairs.h>
#include <reactphysics3d/containers/containers_common.h>
#include <reactphysics3d/collision/ContactPointInfo.h>
#include <reactphysics3d/collision/narrowphase/NarrowPhaseAlgorithm.h>
#include <reactphysics3d/collision/narrowphase/CollisionDispatch.h>

using namespace reactphysics3d;

// Constructor
OverlappingPairs::OverlappingPairs(MemoryAllocator& persistentMemoryAllocator, MemoryAllocator& temporaryMemoryAllocator, ColliderComponents &colliderComponents,
                                   CollisionBodyComponents& collisionBodyComponents, RigidBodyComponents& rigidBodyComponents, Set<bodypair> &noCollisionPairs, CollisionDispatch &collisionDispatch)
                : mPersistentAllocator(persistentMemoryAllocator), mTempMemoryAllocator(temporaryMemoryAllocator),
                  mNbPairs(0), mConcavePairsStartIndex(0), mPairDataSize(sizeof(uint64) + sizeof(int32) + sizeof(int32) + sizeof(Entity) +
                                                                         sizeof(Entity) + sizeof(Map<uint64, LastFrameCollisionInfo*>) +
                                                                         sizeof(bool) + sizeof(bool) + sizeof(NarrowPhaseAlgorithmType) +
                                                                         sizeof(bool) + sizeof(bool) + sizeof(bool)),
                  mNbAllocatedPairs(0), mBuffer(nullptr),
                  mMapPairIdToPairIndex(persistentMemoryAllocator),
                  mColliderComponents(colliderComponents), mCollisionBodyComponents(collisionBodyComponents),
                  mRigidBodyComponents(rigidBodyComponents), mNoCollisionPairs(noCollisionPairs), mCollisionDispatch(collisionDispatch) {
    
    // Allocate memory for the components data
    allocate(INIT_NB_ALLOCATED_PAIRS);
}

// Destructor
OverlappingPairs::~OverlappingPairs() {

    // If there are allocated pairs
    if (mNbAllocatedPairs > 0) {

        // Destroy all the remaining pairs
        for (uint32 i = 0; i < mNbPairs; i++) {

            // Remove all the remaining last frame collision info
            for (auto it = mLastFrameCollisionInfos[i].begin(); it != mLastFrameCollisionInfos[i].end(); ++it) {

                // Call the constructor
                it->second->~LastFrameCollisionInfo();

                // Release memory
                mPersistentAllocator.release(it->second, sizeof(LastFrameCollisionInfo));
            }

            // Remove the involved overlapping pair to the two colliders
            assert(mColliderComponents.getOverlappingPairs(mColliders1[i]).find(mPairIds[i]) != mColliderComponents.getOverlappingPairs(mColliders1[i]).end());
            assert(mColliderComponents.getOverlappingPairs(mColliders2[i]).find(mPairIds[i]) != mColliderComponents.getOverlappingPairs(mColliders2[i]).end());
            mColliderComponents.getOverlappingPairs(mColliders1[i]).remove(mPairIds[i]);
            mColliderComponents.getOverlappingPairs(mColliders2[i]).remove(mPairIds[i]);

            destroyPair(i);
        }

        // Size for the data of a single pair (in bytes)
        const size_t totalSizeBytes = mNbAllocatedPairs * mPairDataSize;

        // Release the allocated memory
        mPersistentAllocator.release(mBuffer, totalSizeBytes);
    }
}

// Compute the index where we need to insert the new pair
uint64 OverlappingPairs::prepareAddPair(bool isConvexVsConvex) {

    // If we need to allocate more components
    if (mNbPairs == mNbAllocatedPairs) {
        allocate(mNbAllocatedPairs * 2);
    }

    uint64 index;

    // If the pair to add is not convex vs convex or there are no concave pairs yet
    if (!isConvexVsConvex) {

        // Add the component at the end of the array
        index = mNbPairs;
    }
    // If the pair to add is convex vs convex
    else {

        // If there already are convex vs concave pairs
        if (mConcavePairsStartIndex != mNbPairs) {

            // Move the first convex vs concave pair to the end of the array
            movePairToIndex(mConcavePairsStartIndex, mNbPairs);
        }

        index = mConcavePairsStartIndex;

        mConcavePairsStartIndex++;
    }

    return index;
}

// Remove a component at a given index
void OverlappingPairs::removePair(uint64 pairId) {

    RP3D_PROFILE("OverlappingPairs::removePair()", mProfiler);

    assert(mMapPairIdToPairIndex.containsKey(pairId));

    uint64 index = mMapPairIdToPairIndex[pairId];
    assert(index < mNbPairs);

    // We want to keep the arrays tightly packed. Therefore, when a pair is removed,
    // we replace it with the last element of the array. But we need to make sure that convex
    // and concave pairs stay grouped together.

    // Remove all the remaining last frame collision info
    for (auto it = mLastFrameCollisionInfos[index].begin(); it != mLastFrameCollisionInfos[index].end(); ++it) {

        // Call the constructor
        it->second->~LastFrameCollisionInfo();

        // Release memory
        mPersistentAllocator.release(it->second, sizeof(LastFrameCollisionInfo));
    }

    // Remove the involved overlapping pair to the two colliders
    assert(mColliderComponents.getOverlappingPairs(mColliders1[index]).find(pairId) != mColliderComponents.getOverlappingPairs(mColliders1[index]).end());
    assert(mColliderComponents.getOverlappingPairs(mColliders2[index]).find(pairId) != mColliderComponents.getOverlappingPairs(mColliders2[index]).end());
    mColliderComponents.getOverlappingPairs(mColliders1[index]).remove(pairId);
    mColliderComponents.getOverlappingPairs(mColliders2[index]).remove(pairId);

    // Destroy the pair
    destroyPair(index);

    // If the pair to remove is convex vs concave
    if (index >= mConcavePairsStartIndex) {

        // If the pair is not the last one
        if (index != mNbPairs - 1) {

            // We replace it by the last convex vs concave pair
            movePairToIndex(mNbPairs - 1, index);
        }
    }
    else {   // If the pair to remove is convex vs convex

        // If it not the last convex vs convex pair
        if (index != mConcavePairsStartIndex - 1) {

            // We replace it by the last convex vs convex pair
            movePairToIndex(mConcavePairsStartIndex - 1, index);
        }

        // If there are convex vs concave pairs at the end
        if (mConcavePairsStartIndex != mNbPairs) {

            // We replace the last convex vs convex pair by the last convex vs concave pair
            movePairToIndex(mNbPairs - 1, mConcavePairsStartIndex - 1);
        }

        mConcavePairsStartIndex--;
    }

    mNbPairs--;

    assert(mConcavePairsStartIndex <= mNbPairs);
    assert(mNbPairs == static_cast<uint32>(mMapPairIdToPairIndex.size()));
}

// Allocate memory for a given number of pairs
void OverlappingPairs::allocate(uint64 nbPairsToAllocate) {

    assert(nbPairsToAllocate > mNbAllocatedPairs);

    // Size for the data of a single component (in bytes)
    const size_t totalSizeBytes = nbPairsToAllocate * mPairDataSize;

    // Allocate memory
    void* newBuffer = mPersistentAllocator.allocate(totalSizeBytes);
    assert(newBuffer != nullptr);

    // New pointers to components data
    uint64* newPairIds = static_cast<uint64*>(newBuffer);
    int32* newPairBroadPhaseId1 = reinterpret_cast<int32*>(newPairIds + nbPairsToAllocate);
    int32* newPairBroadPhaseId2 = reinterpret_cast<int32*>(newPairBroadPhaseId1 + nbPairsToAllocate);
    Entity* newColliders1 = reinterpret_cast<Entity*>(newPairBroadPhaseId2 + nbPairsToAllocate);
    Entity* newColliders2 = reinterpret_cast<Entity*>(newColliders1 + nbPairsToAllocate);
    Map<uint64, LastFrameCollisionInfo*>* newLastFrameCollisionInfos = reinterpret_cast<Map<uint64, LastFrameCollisionInfo*>*>(newColliders2 + nbPairsToAllocate);
    bool* newNeedToTestOverlap = reinterpret_cast<bool*>(newLastFrameCollisionInfos + nbPairsToAllocate);
    bool* newIsActive = reinterpret_cast<bool*>(newNeedToTestOverlap + nbPairsToAllocate);
    NarrowPhaseAlgorithmType* newNarrowPhaseAlgorithmType = reinterpret_cast<NarrowPhaseAlgorithmType*>(newIsActive + nbPairsToAllocate);
    bool* newIsShape1Convex = reinterpret_cast<bool*>(newNarrowPhaseAlgorithmType + nbPairsToAllocate);
    bool* wereCollidingInPreviousFrame = reinterpret_cast<bool*>(newIsShape1Convex + nbPairsToAllocate);
    bool* areCollidingInCurrentFrame = reinterpret_cast<bool*>(wereCollidingInPreviousFrame + nbPairsToAllocate);

    // If there was already pairs before
    if (mNbPairs > 0) {

        // Copy component data from the previous buffer to the new one
        memcpy(newPairIds, mPairIds, mNbPairs * sizeof(uint64));
        memcpy(newPairBroadPhaseId1, mPairBroadPhaseId1, mNbPairs * sizeof(int32));
        memcpy(newPairBroadPhaseId2, mPairBroadPhaseId2, mNbPairs * sizeof(int32));
        memcpy(newColliders1, mColliders1, mNbPairs * sizeof(Entity));
        memcpy(newColliders2, mColliders2, mNbPairs * sizeof(Entity));
        memcpy(newLastFrameCollisionInfos, mLastFrameCollisionInfos, mNbPairs * sizeof(Map<uint64, LastFrameCollisionInfo*>));
        memcpy(newNeedToTestOverlap, mNeedToTestOverlap, mNbPairs * sizeof(bool));
        memcpy(newIsActive, mIsActive, mNbPairs * sizeof(bool));
        memcpy(newNarrowPhaseAlgorithmType, mNarrowPhaseAlgorithmType, mNbPairs * sizeof(NarrowPhaseAlgorithmType));
        memcpy(newIsShape1Convex, mIsShape1Convex, mNbPairs * sizeof(bool));
        memcpy(wereCollidingInPreviousFrame, mCollidingInPreviousFrame, mNbPairs * sizeof(bool));
        memcpy(areCollidingInCurrentFrame, mCollidingInCurrentFrame, mNbPairs * sizeof(bool));

        // Deallocate previous memory
        mPersistentAllocator.release(mBuffer, mNbAllocatedPairs * mPairDataSize);
    }

    mBuffer = newBuffer;
    mPairIds = newPairIds;
    mPairBroadPhaseId1 = newPairBroadPhaseId1;
    mPairBroadPhaseId2 = newPairBroadPhaseId2;
    mColliders1 = newColliders1;
    mColliders2 = newColliders2;
    mLastFrameCollisionInfos = newLastFrameCollisionInfos;
    mNeedToTestOverlap = newNeedToTestOverlap;
    mIsActive = newIsActive;
    mNarrowPhaseAlgorithmType = newNarrowPhaseAlgorithmType;
    mIsShape1Convex = newIsShape1Convex;
    mCollidingInPreviousFrame = wereCollidingInPreviousFrame;
    mCollidingInCurrentFrame = areCollidingInCurrentFrame;

    mNbAllocatedPairs = nbPairsToAllocate;
}

// Add an overlapping pair
uint64 OverlappingPairs::addPair(Collider* shape1, Collider* shape2) {

    RP3D_PROFILE("OverlappingPairs::addPair()", mProfiler);

    const Entity collider1 = shape1->getEntity();
    const Entity collider2 = shape2->getEntity();

    const uint collider1Index = mColliderComponents.getEntityIndex(collider1);
    const uint collider2Index = mColliderComponents.getEntityIndex(collider2);

    const CollisionShape* collisionShape1 = mColliderComponents.mCollisionShapes[collider1Index];
    const CollisionShape* collisionShape2 = mColliderComponents.mCollisionShapes[collider2Index];

    const bool isShape1Convex = collisionShape1->isConvex();
    const bool isShape2Convex = collisionShape2->isConvex();
    const bool isConvexVsConvex = isShape1Convex && isShape2Convex;

    // Prepare to add new pair (allocate memory if necessary and compute insertion index)
    uint64 index = prepareAddPair(isConvexVsConvex);

    const uint32 broadPhase1Id = static_cast<uint32>(shape1->getBroadPhaseId());
    const uint32 broadPhase2Id = static_cast<uint32>(shape2->getBroadPhaseId());

    // Compute a unique id for the overlapping pair
    const uint64 pairId = pairNumbers(std::max(broadPhase1Id, broadPhase2Id), std::min(broadPhase1Id, broadPhase2Id));

    assert(!mMapPairIdToPairIndex.containsKey(pairId));

    // Select the narrow phase algorithm to use according to the two collision shapes
    NarrowPhaseAlgorithmType algorithmType;
    if (isConvexVsConvex) {

        algorithmType = mCollisionDispatch.selectNarrowPhaseAlgorithm(collisionShape1->getType(), collisionShape2->getType());
    }
    else {

        algorithmType = mCollisionDispatch.selectNarrowPhaseAlgorithm(isShape1Convex ? collisionShape1->getType() : collisionShape2->getType(),
                                                                      CollisionShapeType::CONVEX_POLYHEDRON);
    }

    // Insert the new component data
    new (mPairIds + index) uint64(pairId);
    new (mPairBroadPhaseId1 + index) int32(shape1->getBroadPhaseId());
    new (mPairBroadPhaseId2 + index) int32(shape2->getBroadPhaseId());
    new (mColliders1 + index) Entity(shape1->getEntity());
    new (mColliders2 + index) Entity(shape2->getEntity());
    new (mLastFrameCollisionInfos + index) Map<uint64, LastFrameCollisionInfo*>(mPersistentAllocator);
    new (mNeedToTestOverlap + index) bool(false);
    new (mIsActive + index) bool(true);
    new (mNarrowPhaseAlgorithmType + index) NarrowPhaseAlgorithmType(algorithmType);
    new (mIsShape1Convex + index) bool(isShape1Convex);
    new (mCollidingInPreviousFrame + index) bool(false);
    new (mCollidingInCurrentFrame + index) bool(false);

    // Map the entity with the new component lookup index
    mMapPairIdToPairIndex.add(Pair<uint64, uint64>(pairId, index));

    // Add the involved overlapping pair to the two colliders
    assert(mColliderComponents.mOverlappingPairs[collider1Index].find(pairId) == mColliderComponents.mOverlappingPairs[collider1Index].end());
    assert(mColliderComponents.mOverlappingPairs[collider2Index].find(pairId) == mColliderComponents.mOverlappingPairs[collider2Index].end());
    mColliderComponents.mOverlappingPairs[collider1Index].add(pairId);
    mColliderComponents.mOverlappingPairs[collider2Index].add(pairId);

    mNbPairs++;

    assert(mConcavePairsStartIndex <= mNbPairs);
    assert(mNbPairs == static_cast<uint64>(mMapPairIdToPairIndex.size()));

    updateOverlappingPairIsActive(pairId);

    return pairId;
}

// Move a pair from a source to a destination index in the pairs array
// The destination location must contain a constructed object
void OverlappingPairs::movePairToIndex(uint64 srcIndex, uint64 destIndex) {

    const uint64 pairId = mPairIds[srcIndex];

    // Copy the data of the source pair to the destination location
    mPairIds[destIndex] = mPairIds[srcIndex];
    mPairBroadPhaseId1[destIndex] = mPairBroadPhaseId1[srcIndex];
    mPairBroadPhaseId2[destIndex] = mPairBroadPhaseId2[srcIndex];
    new (mColliders1 + destIndex) Entity(mColliders1[srcIndex]);
    new (mColliders2 + destIndex) Entity(mColliders2[srcIndex]);
    new (mLastFrameCollisionInfos + destIndex) Map<uint64, LastFrameCollisionInfo*>(mLastFrameCollisionInfos[srcIndex]);
    mNeedToTestOverlap[destIndex] = mNeedToTestOverlap[srcIndex];
    mIsActive[destIndex] = mIsActive[srcIndex];
    new (mNarrowPhaseAlgorithmType + destIndex) NarrowPhaseAlgorithmType(mNarrowPhaseAlgorithmType[srcIndex]);
    mIsShape1Convex[destIndex] = mIsShape1Convex[srcIndex];
    mCollidingInPreviousFrame[destIndex] = mCollidingInPreviousFrame[srcIndex];
    mCollidingInCurrentFrame[destIndex] = mCollidingInCurrentFrame[srcIndex];

    // Destroy the source pair
    destroyPair(srcIndex);

    assert(!mMapPairIdToPairIndex.containsKey(pairId));

    // Update the pairId to pair index mapping
    mMapPairIdToPairIndex.add(Pair<uint64, uint64>(pairId, destIndex));

    assert(mMapPairIdToPairIndex[mPairIds[destIndex]] == destIndex);
}

// Swap two pairs in the array
void OverlappingPairs::swapPairs(uint64 index1, uint64 index2) {

    // Copy pair 1 data
    uint64 pairId = mPairIds[index1];
    int32 pairBroadPhaseId1 = mPairBroadPhaseId1[index1];
    int32 pairBroadPhaseId2 = mPairBroadPhaseId2[index1];
    Entity collider1 = mColliders1[index1];
    Entity collider2 = mColliders2[index1];
    Map<uint64, LastFrameCollisionInfo*> lastFrameCollisionInfo(mLastFrameCollisionInfos[index1]);
    bool needTestOverlap = mNeedToTestOverlap[index1];
    bool isActive = mIsActive[index1];
    NarrowPhaseAlgorithmType narrowPhaseAlgorithmType = mNarrowPhaseAlgorithmType[index1];
    bool isShape1Convex = mIsShape1Convex[index1];
    bool wereCollidingInPreviousFrame = mCollidingInPreviousFrame[index1];
    bool areCollidingInCurrentFrame = mCollidingInCurrentFrame[index1];

    // Destroy pair 1
    destroyPair(index1);

    movePairToIndex(index2, index1);

    // Reconstruct pair 1 at pair 2 location
    mPairIds[index2] = pairId;
    mPairBroadPhaseId1[index2] = pairBroadPhaseId1;
    mPairBroadPhaseId2[index2] = pairBroadPhaseId2;
    new (mColliders1 + index2) Entity(collider1);
    new (mColliders2 + index2) Entity(collider2);
    new (mLastFrameCollisionInfos + index2) Map<uint64, LastFrameCollisionInfo*>(lastFrameCollisionInfo);
    mNeedToTestOverlap[index2] = needTestOverlap;
    mIsActive[index2] = isActive;
    new (mNarrowPhaseAlgorithmType + index2) NarrowPhaseAlgorithmType(narrowPhaseAlgorithmType);
    mIsShape1Convex[index2] = isShape1Convex;
    mCollidingInPreviousFrame[index2] = wereCollidingInPreviousFrame;
    mCollidingInCurrentFrame[index2] = areCollidingInCurrentFrame;

    // Update the pairID to pair index mapping
    mMapPairIdToPairIndex.add(Pair<uint64, uint64>(pairId, index2));

    assert(mMapPairIdToPairIndex[mPairIds[index1]] == index1);
    assert(mMapPairIdToPairIndex[mPairIds[index2]] == index2);
    assert(mNbPairs == static_cast<uint64>(mMapPairIdToPairIndex.size()));
}

// Destroy a pair at a given index
void OverlappingPairs::destroyPair(uint64 index) {

    assert(index < mNbPairs);

    assert(mMapPairIdToPairIndex[mPairIds[index]] == index);

    mMapPairIdToPairIndex.remove(mPairIds[index]);

    mColliders1[index].~Entity();
    mColliders2[index].~Entity();
    mLastFrameCollisionInfos[index].~Map<uint64, LastFrameCollisionInfo*>();
    mNarrowPhaseAlgorithmType[index].~NarrowPhaseAlgorithmType();
}

// Update whether a given overlapping pair is active or not
void OverlappingPairs::updateOverlappingPairIsActive(uint64 pairId) {

    assert(mMapPairIdToPairIndex.containsKey(pairId));

    const uint64 pairIndex = mMapPairIdToPairIndex[pairId];

    const Entity collider1 = mColliders1[pairIndex];
    const Entity collider2 = mColliders2[pairIndex];

    const Entity body1 = mColliderComponents.getBody(collider1);
    const Entity body2 = mColliderComponents.getBody(collider2);

    const bool isBody1Enabled = !mCollisionBodyComponents.getIsEntityDisabled(body1);
    const bool isBody2Enabled = !mCollisionBodyComponents.getIsEntityDisabled(body2);
    const bool isBody1Static = mRigidBodyComponents.hasComponent(body1) &&
                               mRigidBodyComponents.getBodyType(body1) == BodyType::STATIC;
    const bool isBody2Static = mRigidBodyComponents.hasComponent(body2) &&
                               mRigidBodyComponents.getBodyType(body2) == BodyType::STATIC;

    const bool isBody1Active = isBody1Enabled && !isBody1Static;
    const bool isBody2Active = isBody2Enabled && !isBody2Static;

    // Check if the bodies are in the set of bodies that cannot collide between each other
    bodypair bodiesIndex = OverlappingPairs::computeBodiesIndexPair(body1, body2);
    bool bodiesCanCollide = !mNoCollisionPairs.contains(bodiesIndex);

    mIsActive[pairIndex] = bodiesCanCollide && (isBody1Active || isBody2Active);
}

// Add a new last frame collision info if it does not exist for the given shapes already
LastFrameCollisionInfo* OverlappingPairs::addLastFrameInfoIfNecessary(uint64 pairIndex, uint32 shapeId1, uint32 shapeId2) {

    RP3D_PROFILE("OverlappingPairs::addLastFrameInfoIfNecessary()", mProfiler);

    assert(pairIndex < mNbPairs);

    uint32 maxShapeId = shapeId1;
    uint32 minShapeId = shapeId2;
    if (shapeId1 < shapeId2) {
       maxShapeId = shapeId2;
       minShapeId = shapeId1;
    }

    // Try to get the corresponding last frame collision info
    const uint64 shapesId = pairNumbers(maxShapeId, minShapeId);

    // If there is no collision info for those two shapes already
    auto it = mLastFrameCollisionInfos[pairIndex].find(shapesId);
    if (it == mLastFrameCollisionInfos[pairIndex].end()) {

        // Create a new collision info
        LastFrameCollisionInfo* collisionInfo = new (mPersistentAllocator.allocate(sizeof(LastFrameCollisionInfo)))
                                                LastFrameCollisionInfo();

        // Add it into the map of collision infos
        mLastFrameCollisionInfos[pairIndex].add(Pair<uint64, LastFrameCollisionInfo*>(shapesId, collisionInfo));

        return collisionInfo;
    }
    else {

       // The existing collision info is not obsolete
       it->second->isObsolete = false;

       return it->second;
    }
}

// Delete all the obsolete last frame collision info
void OverlappingPairs::clearObsoleteLastFrameCollisionInfos() {

    RP3D_PROFILE("OverlappingPairs::clearObsoleteLastFrameCollisionInfos()", mProfiler);

    // For each overlapping pair
    for (uint64 i=0; i < mNbPairs; i++) {

        // For each collision info
        for (auto it = mLastFrameCollisionInfos[i].begin(); it != mLastFrameCollisionInfos[i].end(); ) {

            // If the collision info is obsolete
            if (it->second->isObsolete) {

                // Delete it
                it->second->~LastFrameCollisionInfo();
                mPersistentAllocator.release(it->second, sizeof(LastFrameCollisionInfo));

                it = mLastFrameCollisionInfos[i].remove(it);
            }
            else {  // If the collision info is not obsolete

                // Do not delete it but mark it as obsolete
                it->second->isObsolete = true;

                ++it;
            }
        }
    }
}

// Set the collidingInPreviousFrame value with the collidinginCurrentFrame value for each pair
void OverlappingPairs::updateCollidingInPreviousFrame() {

    // For each overlapping pair
    for (uint64 i=0; i < mNbPairs; i++) {

        mCollidingInPreviousFrame[i] = mCollidingInCurrentFrame[i];
    }
}
