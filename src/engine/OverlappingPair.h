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

// Class OverlappingPair
/**
 * This class represents a pair of two proxy collision shapes that are overlapping
 * during the broad-phase collision detection. It is created when
 * the two proxy collision shapes start to overlap and is destroyed when they do not
 * overlap anymore. This class contains a contact manifold that
 * store all the contact points between the two bodies.
 */
class OverlappingPair {

    public:

        using OverlappingPairId = Pair<uint, uint>;
        using ShapeIdPair = Pair<uint, uint>;

    private:

        // -------------------- Attributes -------------------- //

        /// Pair ID
        OverlappingPairId mPairID;

        /// Entity of the first proxy-shape of the overlapping pair
        Entity mProxyShape1;

        /// Entity of the second proxy-shape of the overlapping pair
        Entity mProxyShape2;

        /// Persistent memory allocator
        MemoryAllocator& mPersistentAllocator;

        /// Memory allocator used to allocated memory for the ContactManifoldInfo and ContactPointInfo
        MemoryAllocator& mTempMemoryAllocator;

        /// Temporal coherence collision data for each overlapping collision shapes of this pair.
        /// Temporal coherence data store collision information about the last frame.
        /// If two convex shapes overlap, we have a single collision data but if one shape is concave,
        /// we might have collision data for several overlapping triangles. The key in the map is the
        /// shape Ids of the two collision shapes.
        Map<ShapeIdPair, LastFrameCollisionInfo*> mLastFrameCollisionInfos;

        /// World settings
        const WorldSettings& mWorldSettings;

        /// True if we need to test if the overlapping pair of shapes still overlap
        bool mNeedToTestOverlap;

#ifdef IS_PROFILING_ACTIVE

        /// Pointer to the profiler
        Profiler* mProfiler;

#endif

    public:

        // -------------------- Methods -------------------- //

        /// Constructor
        OverlappingPair(ProxyShape* shape1, ProxyShape* shape2,  MemoryAllocator& persistentMemoryAllocator,
                        MemoryAllocator& temporaryMemoryAllocator, const WorldSettings& worldSettings);

        /// Destructor
        ~OverlappingPair();

        /// Deleted copy-constructor
        OverlappingPair(const OverlappingPair& pair) = delete;

        /// Deleted assignment operator
        OverlappingPair& operator=(const OverlappingPair& pair) = delete;

        /// Return the Id of the pair
        OverlappingPairId getId() const;

        /// Return the entity of the first proxy-shape
        Entity getProxyShape1() const;

        /// Return the entity of the second proxy-shape
        Entity getProxyShape2() const;

        /// Return the last frame collision info
        LastFrameCollisionInfo* getLastFrameCollisionInfo(ShapeIdPair& shapeIds);

        /// Return a reference to the temporary memory allocator
        MemoryAllocator& getTemporaryAllocator();

        /// Return true if we need to test if the overlapping pair of shapes still overlap
        bool needToTestOverlap() const;

        /// Set to true if we need to test if the overlapping pair of shapes still overlap
        void setNeedToTestOverlap(bool needToTestOverlap);

        /// Add a new last frame collision info if it does not exist for the given shapes already
        LastFrameCollisionInfo* addLastFrameInfoIfNecessary(uint shapeId1, uint shapeId2);

        /// Return the last frame collision info for a given pair of shape ids
        LastFrameCollisionInfo* getLastFrameCollisionInfo(uint shapeId1, uint shapeId2) const;

        /// Delete all the obsolete last frame collision info
        void clearObsoleteLastFrameCollisionInfos();

        /// Return the pair of bodies index
        static OverlappingPairId computeID(int shape1BroadPhaseId, int shape2BroadPhaseId);

        /// Return the pair of bodies index of the pair
        static bodypair computeBodiesIndexPair(Entity body1Entity, Entity body2Entity);

#ifdef IS_PROFILING_ACTIVE

        /// Set the profiler
        void setProfiler(Profiler* profiler);

#endif

        // -------------------- Friendship -------------------- //

        friend class DynamicsWorld;
};

// Return the Id of the pair
inline OverlappingPair::OverlappingPairId OverlappingPair::getId() const {
    return mPairID;
}

// Return the entity of the first proxy-shape
inline Entity OverlappingPair::getProxyShape1() const {
    return mProxyShape1;
}

// Return the entity of the second proxy-shape
inline Entity OverlappingPair::getProxyShape2() const {
    return mProxyShape2;
}                

// Return the last frame collision info for a given shape id or nullptr if none is found
inline LastFrameCollisionInfo* OverlappingPair::getLastFrameCollisionInfo(ShapeIdPair& shapeIds) {
    Map<ShapeIdPair, LastFrameCollisionInfo*>::Iterator it = mLastFrameCollisionInfos.find(shapeIds);
    if (it != mLastFrameCollisionInfos.end()) {
        return it->second;
    }

    return nullptr;
}

// Return the pair of bodies index
inline OverlappingPair::OverlappingPairId OverlappingPair::computeID(int shape1BroadPhaseId, int shape2BroadPhaseId) {
    assert(shape1BroadPhaseId >= 0 && shape2BroadPhaseId >= 0);

    // Construct the pair of body index
    OverlappingPairId pairID = shape1BroadPhaseId < shape2BroadPhaseId ?
                             OverlappingPairId(shape1BroadPhaseId, shape2BroadPhaseId) :
                             OverlappingPairId(shape2BroadPhaseId, shape1BroadPhaseId);
    assert(pairID.first != pairID.second);
    return pairID;
}

// Return the pair of bodies index
inline bodypair OverlappingPair::computeBodiesIndexPair(Entity body1Entity, Entity body2Entity) {

    // Construct the pair of body index
    bodypair indexPair = body1Entity.id < body2Entity.id ?
                                 bodypair(body1Entity, body2Entity) :
                                 bodypair(body2Entity, body1Entity);
    assert(indexPair.first != indexPair.second);
    return indexPair;
}

// Return a reference to the temporary memory allocator
inline MemoryAllocator& OverlappingPair::getTemporaryAllocator() {
    return mTempMemoryAllocator;
}

// Return true if we need to test if the overlapping pair of shapes still overlap
inline bool OverlappingPair::needToTestOverlap() const {
   return mNeedToTestOverlap;
}

// Set to true if we need to test if the overlapping pair of shapes still overlap
inline void OverlappingPair::setNeedToTestOverlap(bool needToTestOverlap)  {
   mNeedToTestOverlap = needToTestOverlap;
}

// Return the last frame collision info for a given pair of shape ids
inline LastFrameCollisionInfo* OverlappingPair::getLastFrameCollisionInfo(uint shapeId1, uint shapeId2) const {
    return mLastFrameCollisionInfos[ShapeIdPair(shapeId1, shapeId2)];
}

#ifdef IS_PROFILING_ACTIVE

// Set the profiler
inline void OverlappingPair::setProfiler(Profiler* profiler) {
    mProfiler = profiler;
}

#endif
}

#endif

