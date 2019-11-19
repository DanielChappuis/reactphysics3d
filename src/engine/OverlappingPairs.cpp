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
#include <cassert>
#include "OverlappingPairs.h"
#include "containers/containers_common.h"
#include "collision/ContactPointInfo.h"
#include "collision/narrowphase/NarrowPhaseAlgorithm.h"
#include "collision/narrowphase/CollisionDispatch.h"

using namespace reactphysics3d;

// Constructor
OverlappingPairs::OverlappingPairs(MemoryAllocator& persistentMemoryAllocator, MemoryAllocator& temporaryMemoryAllocator, ProxyShapeComponents& proxyShapeComponents,
                                   CollisionBodyComponents& collisionBodyComponents, RigidBodyComponents& rigidBodyComponents, Set<bodypair> &noCollisionPairs, CollisionDispatch &collisionDispatch)
                : mPersistentAllocator(persistentMemoryAllocator), mTempMemoryAllocator(temporaryMemoryAllocator),
                  mNbPairs(0), mConcavePairsStartIndex(0), mPairDataSize(sizeof(uint64) + sizeof(int32) + sizeof(int32) + sizeof(Entity) +
                                                                         sizeof(Entity) + sizeof(Map<uint64, LastFrameCollisionInfo*>) +
                                                                         sizeof(bool) + sizeof(bool) + sizeof(NarrowPhaseAlgorithmType) +
                                                                         sizeof(bool)),
                  mNbAllocatedPairs(0), mBuffer(nullptr),
                  mMapPairIdToPairIndex(persistentMemoryAllocator),
                  mProxyShapeComponents(proxyShapeComponents), mCollisionBodyComponents(collisionBodyComponents),
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

            // Remove the involved overlapping pair to the two proxy-shapes
            assert(mProxyShapeComponents.getOverlappingPairs(mProxyShapes1[i]).find(mPairIds[i]) != mProxyShapeComponents.getOverlappingPairs(mProxyShapes1[i]).end());
            assert(mProxyShapeComponents.getOverlappingPairs(mProxyShapes2[i]).find(mPairIds[i]) != mProxyShapeComponents.getOverlappingPairs(mProxyShapes2[i]).end());
            mProxyShapeComponents.getOverlappingPairs(mProxyShapes1[i]).remove(mPairIds[i]);
            mProxyShapeComponents.getOverlappingPairs(mProxyShapes2[i]).remove(mPairIds[i]);

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

    // Remove the involved overlapping pair to the two proxy-shapes
    assert(mProxyShapeComponents.getOverlappingPairs(mProxyShapes1[index]).find(pairId) != mProxyShapeComponents.getOverlappingPairs(mProxyShapes1[index]).end());
    assert(mProxyShapeComponents.getOverlappingPairs(mProxyShapes2[index]).find(pairId) != mProxyShapeComponents.getOverlappingPairs(mProxyShapes2[index]).end());
    mProxyShapeComponents.getOverlappingPairs(mProxyShapes1[index]).remove(pairId);
    mProxyShapeComponents.getOverlappingPairs(mProxyShapes2[index]).remove(pairId);

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
    Entity* newProxyShapes1 = reinterpret_cast<Entity*>(newPairBroadPhaseId2 + nbPairsToAllocate);
    Entity* newProxyShapes2 = reinterpret_cast<Entity*>(newProxyShapes1 + nbPairsToAllocate);
    Map<uint64, LastFrameCollisionInfo*>* newLastFrameCollisionInfos = reinterpret_cast<Map<uint64, LastFrameCollisionInfo*>*>(newProxyShapes2 + nbPairsToAllocate);
    bool* newNeedToTestOverlap = reinterpret_cast<bool*>(newLastFrameCollisionInfos + nbPairsToAllocate);
    bool* newIsActive = reinterpret_cast<bool*>(newNeedToTestOverlap + nbPairsToAllocate);
    NarrowPhaseAlgorithmType* newNarrowPhaseAlgorithmType = reinterpret_cast<NarrowPhaseAlgorithmType*>(newIsActive + nbPairsToAllocate);
    bool* newIsShape1Convex = reinterpret_cast<bool*>(newNarrowPhaseAlgorithmType + nbPairsToAllocate);

    // If there was already pairs before
    if (mNbPairs > 0) {

        // Copy component data from the previous buffer to the new one
        memcpy(newPairIds, mPairIds, mNbPairs * sizeof(uint64));
        memcpy(newPairBroadPhaseId1, mPairBroadPhaseId1, mNbPairs * sizeof(int32));
        memcpy(newPairBroadPhaseId2, mPairBroadPhaseId2, mNbPairs * sizeof(int32));
        memcpy(newProxyShapes1, mProxyShapes1, mNbPairs * sizeof(Entity));
        memcpy(newProxyShapes2, mProxyShapes2, mNbPairs * sizeof(Entity));
        memcpy(newLastFrameCollisionInfos, mLastFrameCollisionInfos, mNbPairs * sizeof(Map<uint64, LastFrameCollisionInfo*>));
        memcpy(newNeedToTestOverlap, mNeedToTestOverlap, mNbPairs * sizeof(bool));
        memcpy(newIsActive, mIsActive, mNbPairs * sizeof(bool));
        memcpy(newNarrowPhaseAlgorithmType, mNarrowPhaseAlgorithmType, mNbPairs * sizeof(NarrowPhaseAlgorithmType));
        memcpy(newIsShape1Convex, mIsShape1Convex, mNbPairs * sizeof(bool));

        // Deallocate previous memory
        mPersistentAllocator.release(mBuffer, mNbAllocatedPairs * mPairDataSize);
    }

    mBuffer = newBuffer;
    mPairIds = newPairIds;
    mPairBroadPhaseId1 = newPairBroadPhaseId1;
    mPairBroadPhaseId2 = newPairBroadPhaseId2;
    mProxyShapes1 = newProxyShapes1;
    mProxyShapes2 = newProxyShapes2;
    mLastFrameCollisionInfos = newLastFrameCollisionInfos;
    mNeedToTestOverlap = newNeedToTestOverlap;
    mIsActive = newIsActive;
    mNarrowPhaseAlgorithmType = newNarrowPhaseAlgorithmType;
    mIsShape1Convex = newIsShape1Convex;

    mNbAllocatedPairs = nbPairsToAllocate;
}

// Add an overlapping pair
uint64 OverlappingPairs::addPair(ProxyShape* shape1, ProxyShape* shape2) {

    RP3D_PROFILE("OverlappingPairs::addPair()", mProfiler);

    const CollisionShape* collisionShape1 = mProxyShapeComponents.getCollisionShape(shape1->getEntity());
    const CollisionShape* collisionShape2 = mProxyShapeComponents.getCollisionShape(shape2->getEntity());

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
    new (mProxyShapes1 + index) Entity(shape1->getEntity());
    new (mProxyShapes2 + index) Entity(shape2->getEntity());
    new (mLastFrameCollisionInfos + index) Map<uint64, LastFrameCollisionInfo*>(mPersistentAllocator);
    new (mNeedToTestOverlap + index) bool(false);
    new (mIsActive + index) bool(true);
    new (mNarrowPhaseAlgorithmType + index) NarrowPhaseAlgorithmType(algorithmType);
    new (mIsShape1Convex + index) bool(isShape1Convex);

    // Map the entity with the new component lookup index
    mMapPairIdToPairIndex.add(Pair<uint64, uint64>(pairId, index));

    // Add the involved overlapping pair to the two proxy-shapes
    assert(mProxyShapeComponents.getOverlappingPairs(shape1->getEntity()).find(pairId) == mProxyShapeComponents.getOverlappingPairs(shape1->getEntity()).end());
    assert(mProxyShapeComponents.getOverlappingPairs(shape2->getEntity()).find(pairId) == mProxyShapeComponents.getOverlappingPairs(shape2->getEntity()).end());
    mProxyShapeComponents.getOverlappingPairs(shape1->getEntity()).add(pairId);
    mProxyShapeComponents.getOverlappingPairs(shape2->getEntity()).add(pairId);

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
    new (mProxyShapes1 + destIndex) Entity(mProxyShapes1[srcIndex]);
    new (mProxyShapes2 + destIndex) Entity(mProxyShapes2[srcIndex]);
    new (mLastFrameCollisionInfos + destIndex) Map<uint64, LastFrameCollisionInfo*>(mLastFrameCollisionInfos[srcIndex]);
    mNeedToTestOverlap[destIndex] = mNeedToTestOverlap[srcIndex];
    mIsActive[destIndex] = mIsActive[srcIndex];
    new (mNarrowPhaseAlgorithmType + destIndex) NarrowPhaseAlgorithmType(mNarrowPhaseAlgorithmType[srcIndex]);
    mIsShape1Convex[destIndex] = mIsShape1Convex[srcIndex];

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
    Entity proxyShape1 = mProxyShapes1[index1];
    Entity proxyShape2 = mProxyShapes2[index1];
    Map<uint64, LastFrameCollisionInfo*> lastFrameCollisionInfo(mLastFrameCollisionInfos[index1]);
    bool needTestOverlap = mNeedToTestOverlap[index1];
    bool isActive = mIsActive[index1];
    NarrowPhaseAlgorithmType narrowPhaseAlgorithmType = mNarrowPhaseAlgorithmType[index1];
    bool isShape1Convex = mIsShape1Convex[index1];

    // Destroy pair 1
    destroyPair(index1);

    movePairToIndex(index2, index1);

    // Reconstruct pair 1 at pair 2 location
    mPairIds[index2] = pairId;
    mPairBroadPhaseId1[index2] = pairBroadPhaseId1;
    mPairBroadPhaseId2[index2] = pairBroadPhaseId2;
    new (mProxyShapes1 + index2) Entity(proxyShape1);
    new (mProxyShapes2 + index2) Entity(proxyShape2);
    new (mLastFrameCollisionInfos + index2) Map<uint64, LastFrameCollisionInfo*>(lastFrameCollisionInfo);
    mNeedToTestOverlap[index2] = needTestOverlap;
    mIsActive[index2] = isActive;
    new (mNarrowPhaseAlgorithmType + index2) NarrowPhaseAlgorithmType(narrowPhaseAlgorithmType);
    mIsShape1Convex[index2] = isShape1Convex;

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

    mProxyShapes1[index].~Entity();
    mProxyShapes2[index].~Entity();
    mLastFrameCollisionInfos[index].~Map<uint64, LastFrameCollisionInfo*>();
    mNarrowPhaseAlgorithmType[index].~NarrowPhaseAlgorithmType();
}

// Update whether a given overlapping pair is active or not
void OverlappingPairs::updateOverlappingPairIsActive(uint64 pairId) {

    assert(mMapPairIdToPairIndex.containsKey(pairId));

    const uint64 pairIndex = mMapPairIdToPairIndex[pairId];

    const Entity proxyShape1 = mProxyShapes1[pairIndex];
    const Entity proxyShape2 = mProxyShapes2[pairIndex];

    const Entity body1 = mProxyShapeComponents.getBody(proxyShape1);
    const Entity body2 = mProxyShapeComponents.getBody(proxyShape2);

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

    // Try to get the corresponding last frame collision info
    const uint64 shapesId = pairNumbers(std::max(shapeId1, shapeId2), std::min(shapeId1, shapeId2));

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

    // For each convex vs convex overlapping pair
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
