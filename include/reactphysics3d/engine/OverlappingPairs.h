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

#ifndef REACTPHYSICS3D_OVERLAPPING_PAIR_H
#define	REACTPHYSICS3D_OVERLAPPING_PAIR_H

// Libraries
#include <reactphysics3d/collision/Collider.h>
#include <reactphysics3d/containers/Map.h>
#include <reactphysics3d/containers/Pair.h>
#include <reactphysics3d/containers/Set.h>
#include <reactphysics3d/containers/containers_common.h>
#include <reactphysics3d/utils/Profiler.h>
#include <reactphysics3d/components/ColliderComponents.h>
#include <reactphysics3d/components/CollisionBodyComponents.h>
#include <reactphysics3d/components/RigidBodyComponents.h>
#include <cstddef>

/// ReactPhysics3D namespace
namespace reactphysics3d {

// Declarations
struct NarrowPhaseInfoBatch;
enum class NarrowPhaseAlgorithmType;
class CollisionShape;
class CollisionDispatch;

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
 * This class contains pairs of two colliders that are overlapping
 * during the broad-phase collision detection. A pair is created when
 * the two colliders start to overlap and is destroyed when they do not
 * overlap anymore. Each contains a contact manifold that
 * store all the contact points between the two bodies.
 */
class OverlappingPairs {

    private:

        // -------------------- Constants -------------------- //


        /// Number of pairs to allocated at the beginning
        const uint32 INIT_NB_ALLOCATED_PAIRS = 10;

        // -------------------- Attributes -------------------- //

        /// Persistent memory allocator
        MemoryAllocator& mPersistentAllocator;

        /// Memory allocator used to allocated memory for the ContactManifoldInfo and ContactPointInfo
        // TODO : Do we need to keep this ?
        MemoryAllocator& mTempMemoryAllocator;

        /// Current number of components
        uint64 mNbPairs;

        /// Index in the array of the first convex vs concave pair
        uint64 mConcavePairsStartIndex;

        /// Size (in bytes) of a single pair
        size_t mPairDataSize;

        /// Number of allocated pairs
        uint64 mNbAllocatedPairs;

        /// Allocated memory for all the data of the pairs
        void* mBuffer;

        /// Map a pair id to the internal array index
        Map<uint64, uint64> mMapPairIdToPairIndex;

        /// Ids of the convex vs convex pairs
        uint64* mPairIds;

        /// Array with the broad-phase Ids of the first shape
        int32* mPairBroadPhaseId1;

        /// Array with the broad-phase Ids of the second shape
        int32* mPairBroadPhaseId2;

        /// Array of Entity of the first collider of the convex vs convex pairs
        Entity* mColliders1;

        /// Array of Entity of the second collider of the convex vs convex pairs
        Entity* mColliders2;

        /// Temporal coherence collision data for each overlapping collision shapes of this pair.
        /// Temporal coherence data store collision information about the last frame.
        /// If two convex shapes overlap, we have a single collision data but if one shape is concave,
        /// we might have collision data for several overlapping triangles. The key in the map is the
        /// shape Ids of the two collision shapes.
        Map<uint64, LastFrameCollisionInfo*>* mLastFrameCollisionInfos;

        /// True if we need to test if the convex vs convex overlapping pairs of shapes still overlap
        bool* mNeedToTestOverlap;

        /// True if the overlapping pair is active (at least one body of the pair is active and not static)
        bool* mIsActive;

        /// Array with the pointer to the narrow-phase algorithm for each overlapping pair
        NarrowPhaseAlgorithmType* mNarrowPhaseAlgorithmType;

        /// True if the first shape of the pair is convex
        bool* mIsShape1Convex;

        /// True if the colliders of the overlapping pair were colliding in the previous frame
        bool* mCollidingInPreviousFrame;

        /// True if the colliders of the overlapping pair are colliding in the current frame
        bool* mCollidingInCurrentFrame;

        /// Reference to the colliders components
        ColliderComponents& mColliderComponents;

        /// Reference to the collision body components
        CollisionBodyComponents& mCollisionBodyComponents;

        /// Reference to the rigid bodies components
        RigidBodyComponents& mRigidBodyComponents;

        /// Reference to the set of bodies that cannot collide with each others
        Set<bodypair>& mNoCollisionPairs;

        /// Reference to the collision dispatch
        CollisionDispatch& mCollisionDispatch;

#ifdef IS_RP3D_PROFILING_ENABLED

        /// Pointer to the profiler
        Profiler* mProfiler;

#endif

        // -------------------- Methods -------------------- //

        /// Allocate memory for a given number of pairs
        void allocate(uint64 nbPairsToAllocate);

        /// Compute the index where we need to insert the new pair
        uint64 prepareAddPair(bool isConvexVsConvex);

        /// Destroy a pair at a given index
        void destroyPair(uint64 index);

        // Move a pair from a source to a destination index in the pairs array
        void movePairToIndex(uint64 srcIndex, uint64 destIndex);

        /// Swap two pairs in the array
        void swapPairs(uint64 index1, uint64 index2);

    public:

        // -------------------- Methods -------------------- //

        /// Constructor
        OverlappingPairs(MemoryAllocator& persistentMemoryAllocator, MemoryAllocator& temporaryMemoryAllocator,
                         ColliderComponents& colliderComponents, CollisionBodyComponents& collisionBodyComponents,
                         RigidBodyComponents& rigidBodyComponents, Set<bodypair>& noCollisionPairs,
                         CollisionDispatch& collisionDispatch);

        /// Destructor
        ~OverlappingPairs();

        /// Deleted copy-constructor
        OverlappingPairs(const OverlappingPairs& pair) = delete;

        /// Deleted assignment operator
        OverlappingPairs& operator=(const OverlappingPairs& pair) = delete;

        /// Add an overlapping pair
        uint64 addPair(Collider* shape1, Collider* shape2);

        /// Remove a component at a given index
        void removePair(uint64 pairId);

        /// Return the number of pairs
        uint64 getNbPairs() const;

        /// Return the number of convex vs convex pairs
        uint64 getNbConvexVsConvexPairs() const;

        /// Return the number of convex vs concave pairs
        uint64 getNbConvexVsConcavePairs() const;

        /// Return the starting index of the convex vs concave pairs
        uint64 getConvexVsConcavePairsStartIndex() const;

        /// Return the entity of the first collider
        Entity getCollider1(uint64 pairId) const;

        /// Return the entity of the second collider
        Entity getCollider2(uint64 pairId) const;

        /// Notify if a given pair is active or not
        void setIsPairActive(uint64 pairId, bool isActive);

        /// Return the index of a given overlapping pair in the internal array
        uint64 getPairIndex(uint64 pairId) const;

        /// Return the last frame collision info
        LastFrameCollisionInfo* getLastFrameCollisionInfo(uint64, uint64 shapesId);

        /// Return a reference to the temporary memory allocator
        MemoryAllocator& getTemporaryAllocator();

        /// Add a new last frame collision info if it does not exist for the given shapes already
        LastFrameCollisionInfo* addLastFrameInfoIfNecessary(uint64 pairIndex, uint32 shapeId1, uint32 shapeId2);

        /// Update whether a given overlapping pair is active or not
        void updateOverlappingPairIsActive(uint64 pairId);

        /// Delete all the obsolete last frame collision info
        void clearObsoleteLastFrameCollisionInfos();

        /// Set the collidingInPreviousFrame value with the collidinginCurrentFrame value for each pair
        void updateCollidingInPreviousFrame();

        /// Return the pair of bodies index of the pair
        static bodypair computeBodiesIndexPair(Entity body1Entity, Entity body2Entity);

        /// Set if we need to test a given pair for overlap
        void setNeedToTestOverlap(uint64 pairId, bool needToTestOverlap);

        /// Return true if the two colliders of the pair were already colliding the previous frame
        bool getCollidingInPreviousFrame(uint64 pairId) const;

        /// Set to true if the two colliders of the pair were already colliding the previous frame
        void setCollidingInPreviousFrame(uint64 pairId, bool wereCollidingInPreviousFrame);

#ifdef IS_RP3D_PROFILING_ENABLED

        /// Set the profiler
        void setProfiler(Profiler* profiler);

#endif

        // -------------------- Friendship -------------------- //

        friend class PhysicsWorld;
        friend class CollisionDetectionSystem;
};

// Return the entity of the first collider
inline Entity OverlappingPairs::getCollider1(uint64 pairId) const {
    assert(mMapPairIdToPairIndex.containsKey(pairId));
    assert(mMapPairIdToPairIndex[pairId] < mNbPairs);
    return mColliders1[mMapPairIdToPairIndex[pairId]];
}

// Return the entity of the second collider
inline Entity OverlappingPairs::getCollider2(uint64 pairId) const {
    assert(mMapPairIdToPairIndex.containsKey(pairId));
    assert(mMapPairIdToPairIndex[pairId] < mNbPairs);
    return mColliders2[mMapPairIdToPairIndex[pairId]];
}

// Notify if a given pair is active or not
inline void OverlappingPairs::setIsPairActive(uint64 pairId, bool isActive) {

    assert(mMapPairIdToPairIndex.containsKey(pairId));
    assert(mMapPairIdToPairIndex[pairId] < mNbPairs);
    mIsActive[mMapPairIdToPairIndex[pairId]] = isActive;
}

// Return the index of a given overlapping pair in the internal array
inline uint64 OverlappingPairs::getPairIndex(uint64 pairId) const {
    assert(mMapPairIdToPairIndex.containsKey(pairId));
    return mMapPairIdToPairIndex[pairId];
}

// Return the last frame collision info for a given shape id or nullptr if none is found
inline LastFrameCollisionInfo* OverlappingPairs::getLastFrameCollisionInfo(uint64 pairId, uint64 shapesId) {

    assert(mMapPairIdToPairIndex.containsKey(pairId));
    const uint64 index = mMapPairIdToPairIndex[pairId];
    assert(index < mNbPairs);

    Map<uint64, LastFrameCollisionInfo*>::Iterator it = mLastFrameCollisionInfos[index].find(shapesId);
    if (it != mLastFrameCollisionInfos[index].end()) {
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

// Return the number of pairs
inline uint64 OverlappingPairs::getNbPairs() const {
    return mNbPairs;
}

// Return the number of convex vs convex pairs
inline uint64 OverlappingPairs::getNbConvexVsConvexPairs() const {
   return mConcavePairsStartIndex;
}

// Return the number of convex vs concave pairs
inline uint64 OverlappingPairs::getNbConvexVsConcavePairs() const {
   return mNbPairs - mConcavePairsStartIndex;
}

// Return the starting index of the convex vs concave pairs
inline uint64 OverlappingPairs::getConvexVsConcavePairsStartIndex() const {
   return mConcavePairsStartIndex;
}

// Return a reference to the temporary memory allocator
inline MemoryAllocator& OverlappingPairs::getTemporaryAllocator() {
    return mTempMemoryAllocator;
}

// Set if we need to test a given pair for overlap
inline void OverlappingPairs::setNeedToTestOverlap(uint64 pairId, bool needToTestOverlap) {
    assert(mMapPairIdToPairIndex.containsKey(pairId));
    mNeedToTestOverlap[mMapPairIdToPairIndex[pairId]] = needToTestOverlap;
}

// Return true if the two colliders of the pair were already colliding the previous frame
inline bool OverlappingPairs::getCollidingInPreviousFrame(uint64 pairId) const {
    assert(mMapPairIdToPairIndex.containsKey(pairId));
    return mCollidingInPreviousFrame[mMapPairIdToPairIndex[pairId]];
}

// Set to true if the two colliders of the pair were already colliding the previous frame
inline void OverlappingPairs::setCollidingInPreviousFrame(uint64 pairId, bool wereCollidingInPreviousFrame) {
    assert(mMapPairIdToPairIndex.containsKey(pairId));
    mCollidingInPreviousFrame[mMapPairIdToPairIndex[pairId]] = wereCollidingInPreviousFrame;
}

#ifdef IS_RP3D_PROFILING_ENABLED

// Set the profiler
inline void OverlappingPairs::setProfiler(Profiler* profiler) {
    mProfiler = profiler;
}

#endif
}

#endif

