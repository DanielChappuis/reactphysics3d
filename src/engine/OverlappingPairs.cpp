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

using namespace reactphysics3d;

// Constructor
OverlappingPairs::OverlappingPairs(MemoryAllocator& persistentMemoryAllocator, MemoryAllocator& temporaryMemoryAllocator, ProxyShapeComponents& proxyShapeComponents)
                : mPersistentAllocator(persistentMemoryAllocator), mTempMemoryAllocator(temporaryMemoryAllocator),
                  mMapPairIdToPairIndex(persistentMemoryAllocator),
                  mConvexPairIds(mPersistentAllocator), mConvexProxyShapes1(mPersistentAllocator), mConvexProxyShapes2(mPersistentAllocator),
                  mConcavePairIds(mPersistentAllocator), mConcaveProxyShapes1(mPersistentAllocator), mConcaveProxyShapes2(mPersistentAllocator),
                  mConvexLastFrameCollisionInfos(mPersistentAllocator), mConcaveLastFrameCollisionInfos(mPersistentAllocator),
                  mConvexNeedToTestOverlap(mPersistentAllocator), mConcaveNeedToTestOverlap(mPersistentAllocator),
                  mProxyShapeComponents(proxyShapeComponents) {
    
}         

// Destructor
OverlappingPairs::~OverlappingPairs() {

}

/// Add an overlapping pair
uint64 OverlappingPairs::addPair(ProxyShape* shape1, ProxyShape* shape2) {

    // TODO : Maybe use entities in parameters

    const CollisionShape* collisionShape1 = mProxyShapeComponents.getCollisionShape(shape1->getEntity());
    const CollisionShape* collisionShape2 = mProxyShapeComponents.getCollisionShape(shape2->getEntity());

    const uint32 shape1Id = static_cast<uint32>(shape1->getBroadPhaseId());
    const uint32 shape2Id = static_cast<uint32>(shape2->getBroadPhaseId());

    // Compute a unique id for the overlapping pair
    const uint64 pairId = pairNumbers(std::max(shape1Id, shape2Id), std::min(shape1Id, shape2Id));

    // If both shapes are convex
    if (collisionShape1->isConvex() && collisionShape2->isConvex()) {

        const uint nbConvexPairs = static_cast<uint>(mConvexPairIds.size());

        mConvexPairIds.add(pairId);
        mConvexProxyShapes1.add(shape1->getEntity());
        mConvexProxyShapes2.add(shape2->getEntity());
        mConvexNeedToTestOverlap.add(false);

        // TODO: Make sure we use the correct allocator here
        mConvexLastFrameCollisionInfos.add(Map<ShapeIdPair, LastFrameCollisionInfo*>(mPersistentAllocator));

        // Add a mapping to the index in the internal arrays
        mMapPairIdToPairIndex.add(Pair<uint64, PairLocation>(pairId, PairLocation(true, nbConvexPairs)));
    }
    else {

        const uint nbConcavePairs = static_cast<uint>(mConcavePairIds.size());

        mConcavePairIds.add(pairId);
        mConcaveProxyShapes1.add(shape1->getEntity());
        mConcaveProxyShapes2.add(shape2->getEntity());
        mConcaveNeedToTestOverlap.add(true);

        // TODO: Make sure we use the correct allocator here
        mConcaveLastFrameCollisionInfos.add(Map<ShapeIdPair, LastFrameCollisionInfo*>(mPersistentAllocator));

        // Add a mapping to the index in the internal arrays
        mMapPairIdToPairIndex.add(Pair<uint64, PairLocation>(pairId, PairLocation(false, nbConcavePairs)));
    }

    return pairId;
}

// Remove an overlapping pair
uint64 OverlappingPairs::removePair(uint64 pairId) {

    RP3D_PROFILE("OverlappingPairs::removePair()", mProfiler);

    assert(mMapPairIdToPairIndex.containsKey(pairId));

    const PairLocation& pairLocation = mMapPairIdToPairIndex[pairId];

    if (pairLocation.isConvexVsConvex) {

        assert(pairLocation.pairIndex < mConvexPairIds.size());

        const uint64 lastPairId = mConvexPairIds[mConvexPairIds.size() - 1];
        const PairLocation lastPairLocation = mMapPairIdToPairIndex[lastPairId];

        // Remap the last pair location
        if (pairLocation.pairIndex < mConvexPairIds.size() - 1) {
            mMapPairIdToPairIndex[lastPairId] = PairLocation(lastPairLocation.isConvexVsConvex, pairLocation.pairIndex);
        }

        // Remove the pair (the pair is replaced by the last one of the lists)
        mConvexPairIds.removeAtAndReplaceWithLast(pairLocation.pairIndex);
        mConvexProxyShapes1.removeAtAndReplaceWithLast(pairLocation.pairIndex);
        mConvexProxyShapes2.removeAtAndReplaceWithLast(pairLocation.pairIndex);
        mConvexNeedToTestOverlap.removeAtAndReplaceWithLast(pairLocation.pairIndex);
    }
    else {

        assert(pairLocation.pairIndex < mConcavePairIds.size());

        const uint64 lastPairId = mConcavePairIds[mConcavePairIds.size() - 1];
        const PairLocation lastPairLocation = mMapPairIdToPairIndex[lastPairId];

        // Remap the last pair location
        if (pairLocation.pairIndex < mConcavePairIds.size() - 1) {
            mMapPairIdToPairIndex[lastPairId] = PairLocation(lastPairLocation.isConvexVsConvex, pairLocation.pairIndex);
        }

        // Remove the pair (the pair is replaced by the last one of the lists)
        mConcavePairIds.removeAtAndReplaceWithLast(pairLocation.pairIndex);
        mConcaveProxyShapes1.removeAtAndReplaceWithLast(pairLocation.pairIndex);
        mConcaveProxyShapes2.removeAtAndReplaceWithLast(pairLocation.pairIndex);
        mConcaveNeedToTestOverlap.removeAtAndReplaceWithLast(pairLocation.pairIndex);
    }

    mMapPairIdToPairIndex.remove(pairId);

    List<Map<ShapeIdPair, LastFrameCollisionInfo*>>& lastFrameCollisionInfos = pairLocation.isConvexVsConvex ?
                    mConvexLastFrameCollisionInfos : mConcaveLastFrameCollisionInfos;

    // Remove all the remaining last frame collision info
    for (auto it = lastFrameCollisionInfos[pairLocation.pairIndex].begin(); it != lastFrameCollisionInfos[pairLocation.pairIndex].end(); ++it) {

        // Call the constructor
        it->second->~LastFrameCollisionInfo();

        // Release memory
        mPersistentAllocator.release(it->second, sizeof(LastFrameCollisionInfo));
    }

    lastFrameCollisionInfos.removeAtAndReplaceWithLast(pairLocation.pairIndex);
}

// Try to find a pair with a given id, return true if the pair is found and the corresponding PairLocation
bool OverlappingPairs::findPair(uint64 pairId, PairLocation& pairLocation) {

    auto it = mMapPairIdToPairIndex.find(pairId);
    if (it != mMapPairIdToPairIndex.end()) {
        pairLocation = it->second;
        return true;
    }

    return false;
}

// Add a new last frame collision info if it does not exist for the given shapes already
LastFrameCollisionInfo* OverlappingPairs::addLastFrameInfoIfNecessary(uint64 pairId, uint shapeId1, uint shapeId2) {

    RP3D_PROFILE("OverlappingPairs::addLastFrameInfoIfNecessary()", mProfiler);

    assert(mMapPairIdToPairIndex.containsKey(pairId));

    PairLocation& pairLocation = mMapPairIdToPairIndex[pairId];
    List<Map<ShapeIdPair, LastFrameCollisionInfo*>>& lastFrameCollisionInfos = pairLocation.isConvexVsConvex ?
                    mConvexLastFrameCollisionInfos : mConcaveLastFrameCollisionInfos;

    // Try to get the corresponding last frame collision info
    const ShapeIdPair shapeIdPair(shapeId1, shapeId2);
    // TODO : Remove test
    auto it = lastFrameCollisionInfos[pairLocation.pairIndex].find(shapeIdPair);

    // If there is no collision info for those two shapes already
    if (it == lastFrameCollisionInfos[pairLocation.pairIndex].end()) {

        // Create a new collision info
        LastFrameCollisionInfo* collisionInfo = new (mPersistentAllocator.allocate(sizeof(LastFrameCollisionInfo)))
                                                LastFrameCollisionInfo();

        // Add it into the map of collision infos
        lastFrameCollisionInfos[pairLocation.pairIndex].add(Pair<ShapeIdPair, LastFrameCollisionInfo*>(shapeIdPair, collisionInfo));

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
    for (uint p=0; p < mConvexLastFrameCollisionInfos.size(); p++) {

        // For each collision info
        for (auto it = mConvexLastFrameCollisionInfos[p].begin(); it != mConvexLastFrameCollisionInfos[p].end(); ) {

            // If the collision info is obsolete
            if (it->second->isObsolete) {

                // Delete it
                it->second->~LastFrameCollisionInfo();
                mPersistentAllocator.release(it->second, sizeof(LastFrameCollisionInfo));

                it = mConvexLastFrameCollisionInfos[p].remove(it);
            }
            else {  // If the collision info is not obsolete

                // Do not delete it but mark it as obsolete
                it->second->isObsolete = true;

                ++it;
            }
        }
    }

    // For each convex vs concave overlapping pair
    for (uint p=0; p < mConcaveLastFrameCollisionInfos.size(); p++) {

        // For each collision info
        for (auto it = mConcaveLastFrameCollisionInfos[p].begin(); it != mConcaveLastFrameCollisionInfos[p].end(); ) {

            // If the collision info is obsolete
            if (it->second->isObsolete) {

                // Delete it
                it->second->~LastFrameCollisionInfo();
                mPersistentAllocator.release(it->second, sizeof(LastFrameCollisionInfo));

                it = mConcaveLastFrameCollisionInfos[p].remove(it);
            }
            else {  // If the collision info is not obsolete

                // Do not delete it but mark it as obsolete
                it->second->isObsolete = true;

                ++it;
            }
        }
    }
}
