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

#ifndef REACTPHYSICS3D_COLLISION_DETECTION_SYSTEM_H
#define REACTPHYSICS3D_COLLISION_DETECTION_SYSTEM_H

// Libraries
#include <reactphysics3d/body/Body.h>
#include <reactphysics3d/systems/BroadPhaseSystem.h>
#include <reactphysics3d/collision/shapes/CollisionShape.h>
#include <reactphysics3d/collision/ContactPointInfo.h>
#include <reactphysics3d/constraint/ContactPoint.h>
#include <reactphysics3d/collision/ContactManifoldInfo.h>
#include <reactphysics3d/collision/ContactManifold.h>
#include <reactphysics3d/collision/ContactPair.h>
#include <reactphysics3d/engine/OverlappingPairs.h>
#include <reactphysics3d/engine/OverlappingPairs.h>
#include <reactphysics3d/collision/narrowphase/NarrowPhaseInput.h>
#include <reactphysics3d/collision/narrowphase/CollisionDispatch.h>
#include <reactphysics3d/containers/Map.h>
#include <reactphysics3d/containers/Set.h>
#include <reactphysics3d/components/ColliderComponents.h>
#include <reactphysics3d/components/TransformComponents.h>
#include <reactphysics3d/collision/HalfEdgeStructure.h>

/// ReactPhysics3D namespace
namespace reactphysics3d {

// Declarations
class PhysicsWorld;
class CollisionCallback;
class OverlapCallback;
class RaycastCallback;
class ContactPoint;
class MemoryManager;
class EventListener;
class CollisionDispatch;

// Class CollisionDetectionSystem
/**
 * This class computes the collision detection algorithms. We first
 * perform a broad-phase algorithm to know which pairs of bodies can
 * collide and then we run a narrow-phase algorithm to compute the
 * collision contacts between bodies.
 */
class CollisionDetectionSystem {

    private :

        using OverlappingPairMap = Map<Pair<uint, uint>, OverlappingPair*>;

        // -------------------- Constants -------------------- //

        /// Maximum number of contact points in a reduced contact manifold
        static const int8 MAX_CONTACT_POINTS_IN_MANIFOLD = 4;

        // -------------------- Attributes -------------------- //

        /// Memory manager
        MemoryManager& mMemoryManager;

        /// Reference the collider components
        ColliderComponents& mCollidersComponents;

        /// Reference to the rigid bodies components
        RigidBodyComponents& mRigidBodyComponents;

        /// Collision Detection Dispatch configuration
        CollisionDispatch mCollisionDispatch;

        /// Pointer to the physics world
        PhysicsWorld* mWorld;

        /// Set of pair of bodies that cannot collide between each other
        Set<bodypair> mNoCollisionPairs;

        /// Broad-phase overlapping pairs
        OverlappingPairs mOverlappingPairs;

        /// Overlapping nodes during broad-phase computation
        Array<Pair<int32, int32>> mBroadPhaseOverlappingNodes;

        /// Broad-phase system
        BroadPhaseSystem mBroadPhaseSystem;

        /// Map a broad-phase id with the corresponding entity of the collider
        Map<int, Entity> mMapBroadPhaseIdToColliderEntity;

        /// Narrow-phase collision detection input
        NarrowPhaseInput mNarrowPhaseInput;

        /// Array of the potential contact points
        Array<ContactPointInfo> mPotentialContactPoints;

        /// Array of the potential contact manifolds
        Array<ContactManifoldInfo> mPotentialContactManifolds;

        /// First array of narrow-phase pair contacts
        Array<ContactPair> mContactPairs1;

        /// Second array of narrow-phase pair contacts
        Array<ContactPair> mContactPairs2;

        /// Pointer to the array of contact pairs of the previous frame (either mContactPairs1 or mContactPairs2)
        Array<ContactPair>* mPreviousContactPairs;

        /// Pointer to the array of contact pairs of the current frame (either mContactPairs1 or mContactPairs2)
        Array<ContactPair>* mCurrentContactPairs;

        /// Array of lost contact pairs (contact pairs in contact in previous frame but not in the current one)
        Array<ContactPair> mLostContactPairs;

        /// Pointer to the map of overlappingPairId to the index of contact pair of the previous frame
        /// (either mMapPairIdToContactPairIndex1 or mMapPairIdToContactPairIndex2)
        Map<uint64, uint> mPreviousMapPairIdToContactPairIndex;

        /// First array with the contact manifolds
        Array<ContactManifold> mContactManifolds1;

        /// Second array with the contact manifolds
        Array<ContactManifold> mContactManifolds2;

        /// Pointer to the array of contact manifolds from the previous frame (either mContactManifolds1 or mContactManifolds2)
        Array<ContactManifold>* mPreviousContactManifolds;

        /// Pointer to the array of contact manifolds from the current frame (either mContactManifolds1 or mContactManifolds2)
        Array<ContactManifold>* mCurrentContactManifolds;

        /// Second array of contact points (contact points from either the current frame of the previous frame)
        Array<ContactPoint> mContactPoints1;

        /// Second array of contact points (contact points from either the current frame of the previous frame)
        Array<ContactPoint> mContactPoints2;

        /// Pointer to the contact points of the previous frame (either mContactPoints1 or mContactPoints2)
        Array<ContactPoint>* mPreviousContactPoints;

        /// Pointer to the contact points of the current frame (either mContactPoints1 or mContactPoints2)
        Array<ContactPoint>* mCurrentContactPoints;

        /// Number of potential contact manifolds in the previous frame
        uint32 mNbPreviousPotentialContactManifolds;

        /// Number of potential contact points in the previous frame
        uint32 mNbPreviousPotentialContactPoints;

        /// Reference to the half-edge structure of the triangle polyhedron
        HalfEdgeStructure& mTriangleHalfEdgeStructure;

        /// Allocated size for a triangle shape
        static const size_t mTriangleShapeAllocatedSize;

#ifdef IS_RP3D_PROFILING_ENABLED

    /// Pointer to the profiler
    Profiler* mProfiler;

#endif

        // -------------------- Methods -------------------- //

        /// Compute the broad-phase collision detection
        void computeBroadPhase();

        /// Compute the middle-phase collision detection
        void computeMiddlePhase(NarrowPhaseInput& narrowPhaseInput, bool needToReportContacts, bool isWorldQuery);

        // Compute the middle-phase collision detection
        void computeMiddlePhaseCollisionSnapshot(Array<uint64>& convexPairs, Array<uint64>& concavePairs, NarrowPhaseInput& narrowPhaseInput,
                                                 bool reportContacts);

        /// Compute the narrow-phase collision detection
        void computeNarrowPhase();

        /// Compute the narrow-phase collision detection for the testOverlap() methods.
        bool computeNarrowPhaseOverlapSnapshot(NarrowPhaseInput& narrowPhaseInput, OverlapCallback* callback);

        /// Compute the narrow-phase collision detection for the testCollision() methods
        bool computeNarrowPhaseCollisionSnapshot(NarrowPhaseInput& narrowPhaseInput, CollisionCallback& callback);

        /// Process the potential contacts after narrow-phase collision detection
        void computeOverlapSnapshotContactPairs(NarrowPhaseInput& narrowPhaseInput, Array<ContactPair>& contactPairs) const;

        /// Convert the potential contact into actual contacts
        void computeOverlapSnapshotContactPairs(NarrowPhaseInfoBatch& narrowPhaseInfoBatch, Array<ContactPair>& contactPairs,
                                         Set<uint64>& setOverlapContactPairId) const;

        /// Take an array of overlapping nodes in the broad-phase and create new overlapping pairs if necessary
        void updateOverlappingPairs(const Array<Pair<int32, int32> >& overlappingNodes);

        /// Remove pairs that are not overlapping anymore
        void removeNonOverlappingPairs();

        /// Disable an overlapping pair (because both bodies of the pair are disabled)
        void disableOverlappingPair(uint64 pairId);

        /// Remove an overlapping pair
        void removeOverlappingPair(uint64 pairId, bool notifyLostContact);

        /// Remove a convex overlapping pair at a given index
        void removeConvexOverlappingPairWithIndex(uint64 pairIndex);

        /// Remove a concave overlapping pair at a given index
        void removeConcaveOverlappingPairWithIndex(uint64 pairIndex);

        /// Add a lost contact pair (pair of colliders that are not in contact anymore)
        void addLostContactPair(OverlappingPairs::OverlappingPair& overlappingPair);

        /// Execute the narrow-phase collision detection algorithm on batches
        bool testNarrowPhaseCollision(NarrowPhaseInput& narrowPhaseInput, bool clipWithPreviousAxisIfStillColliding, MemoryAllocator& allocator);

        /// Compute the concave vs convex middle-phase algorithm for a given pair of bodies
        void computeConvexVsConcaveMiddlePhase(OverlappingPairs::ConcaveOverlappingPair& overlappingPair, MemoryAllocator& allocator,
                                               NarrowPhaseInput& narrowPhaseInput, bool reportContacts);

        /// Swap the previous and current contacts arrays
        void swapPreviousAndCurrentContacts();

        /// Convert the potential contact into actual contacts
        void processPotentialContacts(NarrowPhaseInfoBatch& narrowPhaseInfoBatch,
                                      bool updateLastFrameInfo, Array<ContactPointInfo>& potentialContactPoints,
                                      Array<ContactManifoldInfo>& potentialContactManifolds,
                                      Map<uint64, uint>& mapPairIdToContactPairIndex, Array<ContactPair>* contactPairs);

        /// Process the potential contacts after narrow-phase collision detection
        void processAllPotentialContacts(NarrowPhaseInput& narrowPhaseInput, bool updateLastFrameInfo, Array<ContactPointInfo>& potentialContactPoints,
                                         Array<ContactManifoldInfo>& potentialContactManifolds, Array<ContactPair>* contactPairs);

        /// Reduce the potential contact manifolds and contact points of the overlapping pair contacts
        void reducePotentialContactManifolds(Array<ContactPair>* contactPairs, Array<ContactManifoldInfo>& potentialContactManifolds,
                                             const Array<ContactPointInfo>& potentialContactPoints) const;

        /// Create the actual contact manifolds and contacts points (from potential contacts) for a given contact pair
        void createContacts();

        /// Add the contact pairs to the corresponding bodies
        void addContactPairsToBodies();

        /// Compute the map from contact pairs ids to contact pair for the next frame
        void computeMapPreviousContactPairs();

        /// Compute the lost contact pairs (contact pairs in contact in the previous frame but not in the current one)
        void computeLostContactPairs();

        /// Create the actual contact manifolds and contacts points for testCollision() methods
        void createSnapshotContacts(Array<ContactPair>& contactPairs, Array<ContactManifold> &contactManifolds,
                                    Array<ContactPoint>& contactPoints,
                                    Array<ContactManifoldInfo>& potentialContactManifolds,
                                    Array<ContactPointInfo>& potentialContactPoints);

        /// Initialize the current contacts with the contacts from the previous frame (for warmstarting)
        void initContactsWithPreviousOnes();

        /// Reduce the number of contact points of a potential contact manifold
        void reduceContactPoints(ContactManifoldInfo& manifold, const Transform& shape1ToWorldTransform,
                                 const Array<ContactPointInfo>& potentialContactPoints) const;

        /// Report contacts
        void reportContacts(CollisionCallback& callback, Array<ContactPair>* contactPairs,
                            Array<ContactManifold>* manifolds, Array<ContactPoint>* contactPoints, Array<ContactPair>& lostContactPairs);

        /// Report all triggers
        void reportTriggers(EventListener& eventListener, Array<ContactPair>* contactPairs, Array<ContactPair>& lostContactPairs);

        /// Report all contacts for debug rendering
        void reportDebugRenderingContacts(Array<ContactPair>* contactPairs, Array<ContactManifold>* manifolds, Array<ContactPoint>* contactPoints, Array<ContactPair>& lostContactPairs);

        /// Return the largest depth of all the contact points of a potential manifold
        decimal computePotentialManifoldLargestContactDepth(const ContactManifoldInfo& manifold,
                                                            const Array<ContactPointInfo>& potentialContactPoints) const;

        /// Process the potential contacts where one collion is a concave shape
        void processSmoothMeshContacts(OverlappingPair* pair);

        /// Filter the overlapping pairs to keep only the pairs where a given body is involved
        void filterOverlappingPairs(Entity bodyEntity, Array<uint64>& convexPairs, Array<uint64>& concavePairs) const;

        /// Filter the overlapping pairs to keep only the pairs where two given bodies are involved
        void filterOverlappingPairs(Entity body1Entity, Entity body2Entity, Array<uint64>& convexPairs, Array<uint64>& concavePairs) const;

        /// Remove an element in an array (and replace it by the last one in the array)
        void removeItemAtInArray(uint array[], uint8 index, uint8& arraySize) const;

        /// Remove the duplicated contact points in a given contact manifold
        void removeDuplicatedContactPointsInManifold(ContactManifoldInfo& manifold, const Array<ContactPointInfo>& potentialContactPoints) const;

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        CollisionDetectionSystem(PhysicsWorld* world, ColliderComponents& collidersComponents,
                           TransformComponents& transformComponents, BodyComponents& bodyComponents, RigidBodyComponents& rigidBodyComponents,
                           MemoryManager& memoryManager, HalfEdgeStructure& triangleHalfEdgeStructure);

        /// Destructor
        ~CollisionDetectionSystem() = default;

        /// Deleted copy-constructor
        CollisionDetectionSystem(const CollisionDetectionSystem& collisionDetection) = delete;

        /// Deleted assignment operator
        CollisionDetectionSystem& operator=(const CollisionDetectionSystem& collisionDetection) = delete;

        /// Set the collision dispatch configuration
        CollisionDispatch& getCollisionDispatch();

        /// Add a collider to the collision detection
        void addCollider(Collider* collider, const AABB& aabb);

        /// Remove a collider from the collision detection
        void removeCollider(Collider* collider);

        /// Update a collider (that has moved for instance)
        void updateCollider(Entity colliderEntity);

        /// Update all the enabled colliders
        void updateColliders();

        /// Add a pair of bodies that cannot collide with each other
        void addNoCollisionPair(Entity body1Entity, Entity body2Entity);

        /// Remove a pair of bodies that cannot collide with each other
        void removeNoCollisionPair(Entity body1Entity, Entity body2Entity);

        /// Ask for a collision shape to be tested again during broad-phase.
        void askForBroadPhaseCollisionCheck(Collider* collider);

        /// Notify that the overlapping pairs where a given collider is involved need to be tested for overlap
        void notifyOverlappingPairsToTestOverlap(Collider* collider);

        /// Report contacts and triggers
        void reportContactsAndTriggers();

        /// Compute the collision detection
        void computeCollisionDetection();

        /// Ray casting method
        void raycast(RaycastCallback* raycastCallback, const Ray& ray,
                     unsigned short raycastWithCategoryMaskBits) const;

        /// Return true if two bodies (collide) overlap
        bool testOverlap(Body* body1, Body* body2);

        /// Report all the bodies that overlap (collide) with the body in parameter
        void testOverlap(Body* body, OverlapCallback& callback);

        /// Report all the bodies that overlap (collide) in the world
        void testOverlap(OverlapCallback& overlapCallback);

        /// Test collision and report contacts between two bodies.
        void testCollision(Body* body1, Body* body2, CollisionCallback& callback);

        /// Test collision and report all the contacts involving the body in parameter
        void testCollision(Body* body, CollisionCallback& callback);

        /// Test collision and report contacts between each colliding bodies in the world
        void testCollision(CollisionCallback& callback);

        /// Return a reference to the memory manager
        MemoryManager& getMemoryManager() const;

        /// Return a pointer to the world
        PhysicsWorld* getWorld();

        /// Return the world event listener
        EventListener* getWorldEventListener();

#ifdef IS_RP3D_PROFILING_ENABLED

		/// Set the profiler
		void setProfiler(Profiler* profiler);

#endif

        /// Return the world-space AABB of a given collider
        const AABB getWorldAABB(const Collider* collider) const;

        // -------------------- Friendship -------------------- //

        friend class PhysicsWorld;
        friend class ConvexMeshShape;
        friend class RigidBody;
        friend class DebugRenderer;
};

// Return a reference to the collision dispatch configuration
RP3D_FORCE_INLINE CollisionDispatch& CollisionDetectionSystem::getCollisionDispatch() {
    return mCollisionDispatch;
}

// Add a body to the collision detection
RP3D_FORCE_INLINE void CollisionDetectionSystem::addCollider(Collider* collider, const AABB& aabb) {

    // Add the body to the broad-phase
    mBroadPhaseSystem.addCollider(collider, aabb);

    int broadPhaseId = mCollidersComponents.getBroadPhaseId(collider->getEntity());

    assert(!mMapBroadPhaseIdToColliderEntity.containsKey(broadPhaseId));

    // Add the mapping between the collider broad-phase id and its entity
    mMapBroadPhaseIdToColliderEntity.add(Pair<int, Entity>(broadPhaseId, collider->getEntity()));
}

// Remove a pair of bodies that cannot collide with each other
RP3D_FORCE_INLINE void CollisionDetectionSystem::removeNoCollisionPair(Entity body1Entity, Entity body2Entity) {
    mNoCollisionPairs.remove(OverlappingPairs::computeBodiesIndexPair(body1Entity, body2Entity));
}

// Ask for a collision shape to be tested again during broad-phase.
/// We simply put the shape in the array of collision shape that have moved in the
/// previous frame so that it is tested for collision again in the broad-phase.
RP3D_FORCE_INLINE void CollisionDetectionSystem::askForBroadPhaseCollisionCheck(Collider* collider) {

    if (collider->getBroadPhaseId() != -1) {
        mBroadPhaseSystem.addMovedCollider(collider->getBroadPhaseId(), collider);
    }
}

// Return a pointer to the world
RP3D_FORCE_INLINE PhysicsWorld* CollisionDetectionSystem::getWorld() {
    return mWorld;
}

// Return a reference to the memory manager
RP3D_FORCE_INLINE MemoryManager& CollisionDetectionSystem::getMemoryManager() const {
    return mMemoryManager;
}

// Update a collider (that has moved for instance)
RP3D_FORCE_INLINE void CollisionDetectionSystem::updateCollider(Entity colliderEntity) {

    // Update the collider component
    mBroadPhaseSystem.updateCollider(colliderEntity);
}

// Update all the enabled colliders
RP3D_FORCE_INLINE void CollisionDetectionSystem::updateColliders() {
    mBroadPhaseSystem.updateColliders();
}

#ifdef IS_RP3D_PROFILING_ENABLED

// Set the profiler
RP3D_FORCE_INLINE void CollisionDetectionSystem::setProfiler(Profiler* profiler) {
	mProfiler = profiler;
    mBroadPhaseSystem.setProfiler(profiler);
    mCollisionDispatch.setProfiler(profiler);
    mOverlappingPairs.setProfiler(profiler);
}

#endif

}

#endif
