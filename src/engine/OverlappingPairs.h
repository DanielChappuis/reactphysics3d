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

#ifndef REACTPHYSICS3D_OVERLAPPING_PAIR_H
#define	REACTPHYSICS3D_OVERLAPPING_PAIR_H

// Libraries
#include "collision/ProxyShape.h"
#include "containers/Map.h"
#include "containers/Pair.h"
#include "containers/containers_common.h"
#include "utils/Profiler.h"
#include "components/ProxyShapeComponents.h"
#include <cstddef>

/// ReactPhysics3D namespace
namespace reactphysics3d {

// Declarations
struct NarrowPhaseInfoBatch;
class CollisionShape;

// Structure LastFrameCollisionInfo
/**
 * This structure contains collision info about the last frame.
 * This is used for temporal coherence between frames.
 */
struct LastFrameCollisionInfo {

    /// True if we have information about the previous frame
    bool isValid;

    /// True if the frame info is obsolete (the collision shape are not overlapping in middle phase)
    bool isObsolete;

    /// True if the two shapes were colliding in the previous frame
    bool wasColliding;

    /// True if we were using GJK algorithm to check for collision in the previous frame
    bool wasUsingGJK;

    /// True if we were using SAT algorithm to check for collision in the previous frame
    bool wasUsingSAT;

    // ----- GJK Algorithm -----

    /// Previous separating axis
    Vector3 gjkSeparatingAxis;

    // SAT Algorithm
    bool satIsAxisFacePolyhedron1;
    bool satIsAxisFacePolyhedron2;
    uint satMinAxisFaceIndex;
    uint satMinEdge1Index;
    uint satMinEdge2Index;

    /// Constructor
    LastFrameCollisionInfo() {

        isValid = false;
        isObsolete = false;
        wasColliding = false;
        wasUsingSAT = false;
        wasUsingGJK = false;

        gjkSeparatingAxis = Vector3(0, 1, 0);
    }
};

// Class OverlappingPairs
/**
 * This class contains pairs of two proxy collision shapes that are overlapping
 * during the broad-phase collision detection. A pair is created when
 * the two proxy collision shapes start to overlap and is destroyed when they do not
 * overlap anymore. Each contains a contact manifold that
 * store all the contact points between the two bodies.
 */
class OverlappingPairs {

    public:

        // TODO : Try to use a pairing function like pairNumbers() here
        using ShapeIdPair = Pair<uint, uint>;

    private:

        /// Structure PairLocation
        struct PairLocation {

            /// True if the pair is a convex vs convex overlap
            bool isConvexVsConvex;

            /// Index of the overlapping pair in the internal arrays
            uint32 pairIndex;

            /// Constructor
            PairLocation() :isConvexVsConvex(true), pairIndex(0) {

            }

            /// Constructor
            PairLocation(bool isConvexVsConvex, uint32 index)
                :isConvexVsConvex(isConvexVsConvex), pairIndex(index) {

            }
        };

        // -------------------- Attributes -------------------- //

        /// Persistent memory allocator
        MemoryAllocator& mPersistentAllocator;

        /// Memory allocator used to allocated memory for the ContactManifoldInfo and ContactPointInfo
        MemoryAllocator& mTempMemoryAllocator;

        /// Map a pair id to its local location
        Map<uint64, PairLocation> mMapPairIdToPairIndex;

        /// Ids of the convex vs convex pairs
        // TODO : Check if we need this array
        List<uint64> mConvexPairIds;

        /// List of Entity of the first proxy-shape of the convex vs convex pairs
        List<Entity> mConvexProxyShapes1;

        /// List of Entity of the second proxy-shape of the convex vs convex pairs
        List<Entity> mConvexProxyShapes2;

        /// Ids of the convex vs concave pairs
        // TODO : Check if we need this array
        List<uint64> mConcavePairIds;

        /// List of Entity of the first proxy-shape of the convex vs concave pairs
        List<Entity> mConcaveProxyShapes1;

        /// List of Entity of the second proxy-shape of the convex vs concave pairs
        List<Entity> mConcaveProxyShapes2;

        /// Temporal coherence collision data for each overlapping collision shapes of this pair.
        /// Temporal coherence data store collision information about the last frame.
        /// If two convex shapes overlap, we have a single collision data but if one shape is concave,
        /// we might have collision data for several overlapping triangles. The key in the map is the
        /// shape Ids of the two collision shapes.
        List<Map<ShapeIdPair, LastFrameCollisionInfo*>> mConvexLastFrameCollisionInfos;

        /// Temporal coherence collision data for each overlapping collision shapes of this pair.
        /// Temporal coherence data store collision information about the last frame.
        /// If two convex shapes overlap, we have a single collision data but if one shape is concave,
        /// we might have collision data for several overlapping triangles. The key in the map is the
        /// shape Ids of the two collision shapes.
        List<Map<ShapeIdPair, LastFrameCollisionInfo*>> mConcaveLastFrameCollisionInfos;

        /// True if we need to test if the convex vs convex overlapping pairs of shapes still overlap
        List<bool> mConvexNeedToTestOverlap;

        /// True if we need to test if the convex vs convex overlapping pairs of shapes still overlap
        List<bool> mConcaveNeedToTestOverlap;

        /// Reference to the proxy-shapes components
        ProxyShapeComponents& mProxyShapeComponents;

#ifdef IS_PROFILING_ACTIVE

        /// Pointer to the profiler
        Profiler* mProfiler;

#endif

    public:

        // -------------------- Methods -------------------- //

        /// Constructor
        OverlappingPairs(MemoryAllocator& persistentMemoryAllocator, MemoryAllocator& temporaryMemoryAllocator,
                         ProxyShapeComponents& proxyShapeComponents);

        /// Destructor
        ~OverlappingPairs();

        /// Deleted copy-constructor
        OverlappingPairs(const OverlappingPairs& pair) = delete;

        /// Deleted assignment operator
        OverlappingPairs& operator=(const OverlappingPairs& pair) = delete;

        /// Add an overlapping pair
        uint64 addPair(ProxyShape* shape1, ProxyShape* shape2);

        /// Remove an overlapping pair
        uint64 removePair(uint64 pairId);

        /// Try to find a pair with a given id, return true if the pair is found and the corresponding PairLocation
        bool findPair(uint64 pairId, PairLocation& pairLocation);

        /// Return the entity of the first proxy-shape
        Entity getProxyShape1(uint64 pairId) const;

        /// Return the entity of the second proxy-shape
        Entity getProxyShape2(uint64 pairId) const;

        /// Return the last frame collision info
        LastFrameCollisionInfo* getLastFrameCollisionInfo(uint64, ShapeIdPair& shapeIds);

        /// Return a reference to the temporary memory allocator
        MemoryAllocator& getTemporaryAllocator();

        /// Add a new last frame collision info if it does not exist for the given shapes already
        LastFrameCollisionInfo* addLastFrameInfoIfNecessary(uint64 pairId, uint shapeId1, uint shapeId2);

        /// Delete all the obsolete last frame collision info
        void clearObsoleteLastFrameCollisionInfos();

        /// Return the pair of bodies index of the pair
        static bodypair computeBodiesIndexPair(Entity body1Entity, Entity body2Entity);

#ifdef IS_PROFILING_ACTIVE

        /// Set the profiler
        void setProfiler(Profiler* profiler);

#endif

        // -------------------- Friendship -------------------- //

        friend class DynamicsWorld;
        friend class CollisionDetectionSystem;
};

// Return the entity of the first proxy-shape
inline Entity OverlappingPairs::getProxyShape1(uint64 pairId) const {
    assert(mMapPairIdToPairIndex.containsKey(pairId));
    const PairLocation& pairLocation = mMapPairIdToPairIndex[pairId];
    const List<Entity> proxyShapes1 = pairLocation.isConvexVsConvex ? mConvexProxyShapes1 : mConcaveProxyShapes1;
    return proxyShapes1[pairLocation.pairIndex];
}

// Return the entity of the second proxy-shape
inline Entity OverlappingPairs::getProxyShape2(uint64 pairId) const {
    assert(mMapPairIdToPairIndex.containsKey(pairId));
    const PairLocation& pairLocation = mMapPairIdToPairIndex[pairId];
    const List<Entity> proxyShapes2 = pairLocation.isConvexVsConvex ? mConvexProxyShapes2 : mConcaveProxyShapes2;
    return proxyShapes2[pairLocation.pairIndex];
}

// Return the last frame collision info for a given shape id or nullptr if none is found
inline LastFrameCollisionInfo* OverlappingPairs::getLastFrameCollisionInfo(uint64 pairId, ShapeIdPair& shapeIds) {

    assert(mMapPairIdToPairIndex.containsKey(pairId));
    const PairLocation& pairLocation = mMapPairIdToPairIndex[pairId];
    const List<Map<ShapeIdPair, LastFrameCollisionInfo*>>& lastFrameCollisionInfos = pairLocation.isConvexVsConvex ?
                                                            mConvexLastFrameCollisionInfos : mConcaveLastFrameCollisionInfos;

    Map<ShapeIdPair, LastFrameCollisionInfo*>::Iterator it = lastFrameCollisionInfos[pairLocation.pairIndex].find(shapeIds);
    if (it != lastFrameCollisionInfos[pairLocation.pairIndex].end()) {
        return it->second;
    }

    return nullptr;
}

// Return the pair of bodies index
inline bodypair OverlappingPairs::computeBodiesIndexPair(Entity body1Entity, Entity body2Entity) {

    // Construct the pair of body index
    bodypair indexPair = body1Entity.id < body2Entity.id ?
                                 bodypair(body1Entity, body2Entity) :
                                 bodypair(body2Entity, body1Entity);
    assert(indexPair.first != indexPair.second);
    return indexPair;
}

// Return a reference to the temporary memory allocator
inline MemoryAllocator& OverlappingPairs::getTemporaryAllocator() {
    return mTempMemoryAllocator;
}

#ifdef IS_PROFILING_ACTIVE

// Set the profiler
inline void OverlappingPairs::setProfiler(Profiler* profiler) {
    mProfiler = profiler;
}

#endif
}

#endif

