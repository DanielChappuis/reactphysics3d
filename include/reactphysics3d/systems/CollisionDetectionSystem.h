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

#ifndef REACTPHYSICS3D_COLLISION_DETECTION_SYSTEM_H
#define REACTPHYSICS3D_COLLISION_DETECTION_SYSTEM_H

// Libraries
#include <reactphysics3d/body/CollisionBody.h>
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

        /// Collision Detection Dispatch configuration
        CollisionDispatch mCollisionDispatch;

        /// Pointer to the physics world
        PhysicsWorld* mWorld;

        /// Set of pair of bodies that cannot collide between each other
        Set<bodypair> mNoCollisionPairs;

        /// Broad-phase overlapping pairs
        OverlappingPairs mOverlappingPairs;

        /// Broad-phase system
        BroadPhaseSystem mBroadPhaseSystem;

        /// Map a broad-phase id with the corresponding entity of the collider
        Map<int, Entity> mMapBroadPhaseIdToColliderEntity;

        /// Narrow-phase collision detection input
        NarrowPhaseInput mNarrowPhaseInput;

        /// List of the potential contact points
        List<ContactPointInfo> mPotentialContactPoints;

        /// List of the potential contact manifolds
        List<ContactManifoldInfo> mPotentialContactManifolds;

        /// First list of narrow-phase pair contacts
        List<ContactPair> mContactPairs1;

        /// Second list of narrow-phase pair contacts
        List<ContactPair> mContactPairs2;

        /// Pointer to the list of contact pairs of the previous frame (either mContactPairs1 or mContactPairs2)
        List<ContactPair>* mPreviousContactPairs;

        /// Pointer to the list of contact pairs of the current frame (either mContactPairs1 or mContactPairs2)
        List<ContactPair>* mCurrentContactPairs;

        /// List of lost contact pairs (contact pairs in contact in previous frame but not in the current one)
        List<ContactPair> mLostContactPairs;

        /// First map of overlapping pair id to the index of the corresponding pair contact
        Map<uint64, uint> mMapPairIdToContactPairIndex1;

        /// Second map of overlapping pair id to the index of the corresponding pair contact
        Map<uint64, uint> mMapPairIdToContactPairIndex2;

        /// Pointer to the map of overlappingPairId to the index of contact pair of the previous frame
        /// (either mMapPairIdToContactPairIndex1 or mMapPairIdToContactPairIndex2)
        Map<uint64, uint>* mPreviousMapPairIdToContactPairIndex;

        /// Pointer to the map of overlappingPairId to the index of contact pair of the current frame
        /// (either mMapPairIdToContactPairIndex1 or mMapPairIdToContactPairIndex2)
        Map<uint64, uint>* mCurrentMapPairIdToContactPairIndex;

        /// First list with the contact manifolds
        List<ContactManifold> mContactManifolds1;

        /// Second list with the contact manifolds
        List<ContactManifold> mContactManifolds2;

        /// Pointer to the list of contact manifolds from the previous frame (either mContactManifolds1 or mContactManifolds2)
        List<ContactManifold>* mPreviousContactManifolds;

        /// Pointer to the list of contact manifolds from the current frame (either mContactManifolds1 or mContactManifolds2)
        List<ContactManifold>* mCurrentContactManifolds;

        /// Second list of contact points (contact points from either the current frame of the previous frame)
        List<ContactPoint> mContactPoints1;

        /// Second list of contact points (contact points from either the current frame of the previous frame)
        List<ContactPoint> mContactPoints2;

        /// Pointer to the contact points of the previous frame (either mContactPoints1 or mContactPoints2)
        List<ContactPoint>* mPreviousContactPoints;

        /// Pointer to the contact points of the current frame (either mContactPoints1 or mContactPoints2)
        List<ContactPoint>* mCurrentContactPoints;

        /// Map a body entity to the list of contact pairs in which it is involved
        Map<Entity, List<uint>> mMapBodyToContactPairs;

#ifdef IS_RP3D_PROFILING_ENABLED

    /// Pointer to the profiler
    Profiler* mProfiler;

#endif

        // -------------------- Methods -------------------- //

        /// Compute the broad-phase collision detection
        void computeBroadPhase();

        /// Compute the middle-phase collision detection
        void computeMiddlePhase(NarrowPhaseInput& narrowPhaseInput, bool needToReportContacts);

        // Compute the middle-phase collision detection
        void computeMiddlePhaseCollisionSnapshot(List<uint64>& convexPairs, List<uint64>& concavePairs, NarrowPhaseInput& narrowPhaseInput,
                                                 bool reportContacts);

        /// Compute the narrow-phase collision detection
        void computeNarrowPhase();

        /// Compute the narrow-phase collision detection for the testOverlap() methods.
        bool computeNarrowPhaseOverlapSnapshot(NarrowPhaseInput& narrowPhaseInput, OverlapCallback* callback);

        /// Compute the narrow-phase collision detection for the testCollision() methods
        bool computeNarrowPhaseCollisionSnapshot(NarrowPhaseInput& narrowPhaseInput, CollisionCallback& callback);

        /// Process the potential contacts after narrow-phase collision detection
        void computeOverlapSnapshotContactPairs(NarrowPhaseInput& narrowPhaseInput, List<ContactPair>& contactPairs) const;

        /// Convert the potential contact into actual contacts
        void computeOverlapSnapshotContactPairs(NarrowPhaseInfoBatch& narrowPhaseInfoBatch, List<ContactPair>& contactPairs,
                                         Set<uint64>& setOverlapContactPairId) const;

        /// Take a list of overlapping nodes in the broad-phase and create new overlapping pairs if necessary
        void updateOverlappingPairs(const List<Pair<int32, int32> >& overlappingNodes);

        /// Remove pairs that are not overlapping anymore
        void removeNonOverlappingPairs();

        /// Add a lost contact pair (pair of colliders that are not in contact anymore)
        void addLostContactPair(uint64 overlappingPairIndex);

        /// Execute the narrow-phase collision detection algorithm on batches
        bool testNarrowPhaseCollision(NarrowPhaseInput& narrowPhaseInput, bool clipWithPreviousAxisIfStillColliding, MemoryAllocator& allocator);

        /// Compute the concave vs convex middle-phase algorithm for a given pair of bodies
        void computeConvexVsConcaveMiddlePhase(uint64 pairIndex, MemoryAllocator& allocator,
                                               NarrowPhaseInput& narrowPhaseInput);

        /// Swap the previous and current contacts lists
        void swapPreviousAndCurrentContacts();

        /// Convert the potential contact into actual contacts
        void processPotentialContacts(NarrowPhaseInfoBatch& narrowPhaseInfoBatch,
                                      bool updateLastFrameInfo, List<ContactPointInfo>& potentialContactPoints,
                                      Map<uint64, uint>* mapPairIdToContactPairIndex,
                                      List<ContactManifoldInfo>& potentialContactManifolds, List<ContactPair>* contactPairs,
                                      Map<Entity, List<uint>>& mapBodyToContactPairs);

        /// Process the potential contacts after narrow-phase collision detection
        void processAllPotentialContacts(NarrowPhaseInput& narrowPhaseInput, bool updateLastFrameInfo, List<ContactPointInfo>& potentialContactPoints,
                                         Map<uint64, uint>* mapPairIdToContactPairIndex,
                                         List<ContactManifoldInfo>& potentialContactManifolds, List<ContactPair>* contactPairs,
                                         Map<Entity, List<uint>>& mapBodyToContactPairs);

        /// Reduce the potential contact manifolds and contact points of the overlapping pair contacts
        void reducePotentialContactManifolds(List<ContactPair>* contactPairs, List<ContactManifoldInfo>& potentialContactManifolds,
                                             const List<ContactPointInfo>& potentialContactPoints) const;

        /// Create the actual contact manifolds and contacts points (from potential contacts) for a given contact pair
        void createContacts();

        /// Compute the lost contact pairs (contact pairs in contact in the previous frame but not in the current one)
        void computeLostContactPairs();

        /// Create the actual contact manifolds and contacts points for testCollision() methods
        void createSnapshotContacts(List<ContactPair>& contactPairs, List<ContactManifold> &contactManifolds,
                                    List<ContactPoint>& contactPoints,
                                    List<ContactManifoldInfo>& potentialContactManifolds,
                                    List<ContactPointInfo>& potentialContactPoints);

        /// Initialize the current contacts with the contacts from the previous frame (for warmstarting)
        void initContactsWithPreviousOnes();

        /// Reduce the number of contact points of a potential contact manifold
        void reduceContactPoints(ContactManifoldInfo& manifold, const Transform& shape1ToWorldTransform,
                                 const List<ContactPointInfo>& potentialContactPoints) const;

        /// Report contacts
        void reportContacts(CollisionCallback& callback, List<ContactPair>* contactPairs,
                            List<ContactManifold>* manifolds, List<ContactPoint>* contactPoints, List<ContactPair>& lostContactPairs);

        /// Report all triggers
        void reportTriggers(EventListener& eventListener, List<ContactPair>* contactPairs, List<ContactPair>& lostContactPairs);

        /// Report all contacts for debug rendering
        void reportDebugRenderingContacts(List<ContactPair>* contactPairs, List<ContactManifold>* manifolds, List<ContactPoint>* contactPoints, List<ContactPair>& lostContactPairs);

        /// Return the largest depth of all the contact points of a potential manifold
        decimal computePotentialManifoldLargestContactDepth(const ContactManifoldInfo& manifold,
                                                            const List<ContactPointInfo>& potentialContactPoints) const;

        /// Process the potential contacts where one collion is a concave shape
        void processSmoothMeshContacts(OverlappingPair* pair);

        /// Filter the overlapping pairs to keep only the pairs where a given body is involved
        void filterOverlappingPairs(Entity bodyEntity, List<uint64>& convexPairs, List<uint64>& concavePairs) const;

        /// Filter the overlapping pairs to keep only the pairs where two given bodies are involved
        void filterOverlappingPairs(Entity body1Entity, Entity body2Entity, List<uint64>& convexPairs, List<uint64>& concavePairs) const;

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        CollisionDetectionSystem(PhysicsWorld* world, ColliderComponents& collidersComponents,
                           TransformComponents& transformComponents, CollisionBodyComponents& collisionBodyComponents, RigidBodyComponents& rigidBodyComponents,
                           MemoryManager& memoryManager);

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
        void updateCollider(Entity colliderEntity, decimal timeStep);

        /// Update all the enabled colliders
        void updateColliders(decimal timeStep);

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
        bool testOverlap(CollisionBody* body1, CollisionBody* body2);

        /// Report all the bodies that overlap (collide) with the body in parameter
        void testOverlap(CollisionBody* body, OverlapCallback& callback);

        /// Report all the bodies that overlap (collide) in the world
        void testOverlap(OverlapCallback& overlapCallback);

        /// Test collision and report contacts between two bodies.
        void testCollision(CollisionBody* body1, CollisionBody* body2, CollisionCallback& callback);

        /// Test collision and report all the contacts involving the body in parameter
        void testCollision(CollisionBody* body, CollisionCallback& callback);

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
inline CollisionDispatch& CollisionDetectionSystem::getCollisionDispatch() {
    return mCollisionDispatch;
}

// Add a body to the collision detection
inline void CollisionDetectionSystem::addCollider(Collider* collider, const AABB& aabb) {

    // Add the body to the broad-phase
    mBroadPhaseSystem.addCollider(collider, aabb);

    int broadPhaseId = mCollidersComponents.getBroadPhaseId(collider->getEntity());

    assert(!mMapBroadPhaseIdToColliderEntity.containsKey(broadPhaseId));

    // Add the mapping between the collider broad-phase id and its entity
    mMapBroadPhaseIdToColliderEntity.add(Pair<int, Entity>(broadPhaseId, collider->getEntity()));
}

// Add a pair of bodies that cannot collide with each other
inline void CollisionDetectionSystem::addNoCollisionPair(Entity body1Entity, Entity body2Entity) {
    mNoCollisionPairs.add(OverlappingPairs::computeBodiesIndexPair(body1Entity, body2Entity));
}

// Remove a pair of bodies that cannot collide with each other
inline void CollisionDetectionSystem::removeNoCollisionPair(Entity body1Entity, Entity body2Entity) {
    mNoCollisionPairs.remove(OverlappingPairs::computeBodiesIndexPair(body1Entity, body2Entity));
}

// Ask for a collision shape to be tested again during broad-phase.
/// We simply put the shape in the list of collision shape that have moved in the
/// previous frame so that it is tested for collision again in the broad-phase.
inline void CollisionDetectionSystem::askForBroadPhaseCollisionCheck(Collider* collider) {

    if (collider->getBroadPhaseId() != -1) {
        mBroadPhaseSystem.addMovedCollider(collider->getBroadPhaseId(), collider);
    }
}

// Return a pointer to the world
inline PhysicsWorld* CollisionDetectionSystem::getWorld() {
    return mWorld;
}

// Return a reference to the memory manager
inline MemoryManager& CollisionDetectionSystem::getMemoryManager() const {
    return mMemoryManager;
}

// Update a collider (that has moved for instance)
inline void CollisionDetectionSystem::updateCollider(Entity colliderEntity, decimal timeStep) {

    // Update the collider component
    mBroadPhaseSystem.updateCollider(colliderEntity, timeStep);
}

// Update all the enabled colliders
inline void CollisionDetectionSystem::updateColliders(decimal timeStep) {
    mBroadPhaseSystem.updateColliders(timeStep);
}

#ifdef IS_RP3D_PROFILING_ENABLED

// Set the profiler
inline void CollisionDetectionSystem::setProfiler(Profiler* profiler) {
	mProfiler = profiler;
    mBroadPhaseSystem.setProfiler(profiler);
    mCollisionDispatch.setProfiler(profiler);
    mOverlappingPairs.setProfiler(profiler);
}

#endif

}

#endif
