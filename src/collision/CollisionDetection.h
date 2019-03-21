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

#ifndef REACTPHYSICS3D_COLLISION_DETECTION_H
#define REACTPHYSICS3D_COLLISION_DETECTION_H

// Libraries
#include "body/CollisionBody.h"
#include "systems/BroadPhaseSystem.h"
#include "collision/shapes/CollisionShape.h"
#include "engine/OverlappingPair.h"
#include "collision/narrowphase/NarrowPhaseInput.h"
#include "collision/narrowphase/CollisionDispatch.h"
#include "containers/Map.h"
#include "containers/Set.h"
#include "components/ProxyShapeComponents.h"
#include "components/TransformComponents.h"

/// ReactPhysics3D namespace
namespace reactphysics3d {

// Declarations
class CollisionWorld;
class CollisionCallback;
class OverlapCallback;
class RaycastCallback;
class ContactPoint;
class MemoryManager;
class EventListener;
class CollisionDispatch;

// Class CollisionDetection
/**
 * This class computes the collision detection algorithms. We first
 * perform a broad-phase algorithm to know which pairs of bodies can
 * collide and then we run a narrow-phase algorithm to compute the
 * collision contacts between bodies.
 */
class CollisionDetection {

    private :

        using OverlappingPairMap = Map<Pair<uint, uint>, OverlappingPair*>;

        // -------------------- Attributes -------------------- //

        /// Memory manager
        MemoryManager& mMemoryManager;

        /// Reference the proxy-shape components
        ProxyShapeComponents& mProxyShapesComponents;

        /// Reference the transform components
        TransformComponents& mTransformComponents;

        /// Collision Detection Dispatch configuration
        CollisionDispatch mCollisionDispatch;

        /// Pointer to the physics world
        CollisionWorld* mWorld;

        /// Broad-phase overlapping pairs
        OverlappingPairMap mOverlappingPairs;

        /// Broad-phase system
        BroadPhaseSystem mBroadPhaseSystem;

        /// Set of pair of bodies that cannot collide between each other
        Set<bodyindexpair> mNoCollisionPairs;

        /// True if some collision shapes have been added previously
        bool mIsCollisionShapesAdded;

        /// Narrow-phase collision detection input
        NarrowPhaseInput mNarrowPhaseInput;

#ifdef IS_PROFILING_ACTIVE

		/// Pointer to the profiler
		Profiler* mProfiler;

#endif

        // -------------------- Methods -------------------- //

        /// Compute the broad-phase collision detection
        void computeBroadPhase();

        /// Compute the middle-phase collision detection
        void computeMiddlePhase();

        /// Compute the narrow-phase collision detection
        void computeNarrowPhase();

        /// Execute the narrow-phase collision detection algorithm on batches
        bool testNarrowPhaseCollision(NarrowPhaseInput& narrowPhaseInput, bool stopFirstContactFound,
                                      bool reportContacts, MemoryAllocator& allocator);

        /// Add a contact manifold to the linked list of contact manifolds of the two bodies
        /// involved in the corresponding contact.
        void addContactManifoldToBody(OverlappingPair* pair);

        /// Add all the contact manifold of colliding pairs to their bodies
        void addAllContactManifoldsToBodies();

        /// Compute the concave vs convex middle-phase algorithm for a given pair of bodies
        void computeConvexVsConcaveMiddlePhase(OverlappingPair* pair, MemoryAllocator& allocator,
                                               NarrowPhaseInput& narrowPhaseInput);

        /// Compute the middle-phase collision detection between two proxy shapes
        void computeMiddlePhaseForProxyShapes(OverlappingPair* pair, NarrowPhaseInput& outNarrowPhaseInput);

        /// Convert the potential contact into actual contacts
        void processPotentialContacts(NarrowPhaseInfoBatch& narrowPhaseInfoBatch,
                                      bool updateLastFrameInfo);

        /// Process the potential contacts after narrow-phase collision detection
        void processAllPotentialContacts(NarrowPhaseInput& narrowPhaseInput, bool updateLastFrameInfo);

        /// Clear the obsolete manifolds and contact points and reduce the number of contacts points of the remaining manifolds
        void reduceContactManifolds(const OverlappingPairMap& overlappingPairs);

        /// Report contacts for all the colliding overlapping pairs
        void reportAllContacts();

        /// Process the potential contacts where one collion is a concave shape
        void processSmoothMeshContacts(OverlappingPair* pair);

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        CollisionDetection(CollisionWorld* world, ProxyShapeComponents& proxyShapesComponents,
                           TransformComponents& transformComponents, DynamicsComponents& dynamicsComponents,
                           MemoryManager& memoryManager);

        /// Destructor
        ~CollisionDetection() = default;

        /// Deleted copy-constructor
        CollisionDetection(const CollisionDetection& collisionDetection) = delete;

        /// Deleted assignment operator
        CollisionDetection& operator=(const CollisionDetection& collisionDetection) = delete;

        /// Set the collision dispatch configuration
        CollisionDispatch& getCollisionDispatch();

        /// Add a proxy collision shape to the collision detection
        void addProxyCollisionShape(ProxyShape* proxyShape, const AABB& aabb);

        /// Remove a proxy collision shape from the collision detection
        void removeProxyCollisionShape(ProxyShape* proxyShape);

        /// Update a proxy collision shape (that has moved for instance)
        void updateProxyShape(Entity proxyShapeEntity);

        /// Update all the enabled proxy-shapes
        void updateProxyShapes();

        /// Add a pair of bodies that cannot collide with each other
        void addNoCollisionPair(CollisionBody* body1, CollisionBody* body2);

        /// Remove a pair of bodies that cannot collide with each other
        void removeNoCollisionPair(CollisionBody* body1, CollisionBody* body2);

        /// Ask for a collision shape to be tested again during broad-phase.
        void askForBroadPhaseCollisionCheck(ProxyShape* shape);

        /// Compute the collision detection
        void computeCollisionDetection();

        /// Ray casting method
        void raycast(RaycastCallback* raycastCallback, const Ray& ray,
                     unsigned short raycastWithCategoryMaskBits) const;

        /// Report all the bodies that overlap with the aabb in parameter
        void testAABBOverlap(const AABB& aabb, OverlapCallback* overlapCallback, unsigned short categoryMaskBits = 0xFFFF);

        /// Return true if two bodies overlap
        bool testOverlap(CollisionBody* body1, CollisionBody* body2);

        /// Report all the bodies that overlap with the body in parameter
        void testOverlap(CollisionBody* body, OverlapCallback* overlapCallback, unsigned short categoryMaskBits = 0xFFFF);

        /// Test and report collisions between two bodies
        void testCollision(CollisionBody* body1, CollisionBody* body2, CollisionCallback* callback);

        /// Test and report collisions between a body and all the others bodies of the world
        void testCollision(CollisionBody* body, CollisionCallback* callback, unsigned short categoryMaskBits = 0xFFFF);

        /// Test and report collisions between all shapes of the world
        void testCollision(CollisionCallback* callback);

        /// Allow the broadphase to notify the collision detection about an overlapping pair.
        void broadPhaseNotifyOverlappingPair(ProxyShape* shape1, ProxyShape* shape2);

        /// Return a reference to the memory manager
        MemoryManager& getMemoryManager() const;

        /// Return a pointer to the world
        CollisionWorld* getWorld();

        /// Return the world event listener
        EventListener* getWorldEventListener();

#ifdef IS_PROFILING_ACTIVE

		/// Set the profiler
		void setProfiler(Profiler* profiler);

#endif

        /// Return the world-space AABB of a given proxy shape
        const AABB getWorldAABB(const ProxyShape* proxyShape) const;

        // -------------------- Friendship -------------------- //

        friend class DynamicsWorld;
        friend class ConvexMeshShape;
};

// Return a reference to the collision dispatch configuration
inline CollisionDispatch& CollisionDetection::getCollisionDispatch() {
    return mCollisionDispatch;
}

// Add a body to the collision detection
inline void CollisionDetection::addProxyCollisionShape(ProxyShape* proxyShape,
                                                       const AABB& aabb) {
    
    // Add the body to the broad-phase
    mBroadPhaseSystem.addProxyCollisionShape(proxyShape, aabb);

    mIsCollisionShapesAdded = true;
}  

// Add a pair of bodies that cannot collide with each other
inline void CollisionDetection::addNoCollisionPair(CollisionBody* body1,
                                                   CollisionBody* body2) {
    mNoCollisionPairs.add(OverlappingPair::computeBodiesIndexPair(body1, body2));
}

// Remove a pair of bodies that cannot collide with each other
inline void CollisionDetection::removeNoCollisionPair(CollisionBody* body1,
                                                      CollisionBody* body2) {
    mNoCollisionPairs.remove(OverlappingPair::computeBodiesIndexPair(body1, body2));
}

// Ask for a collision shape to be tested again during broad-phase.
/// We simply put the shape in the list of collision shape that have moved in the
/// previous frame so that it is tested for collision again in the broad-phase.
inline void CollisionDetection::askForBroadPhaseCollisionCheck(ProxyShape* shape) {

    if (shape->getBroadPhaseId() != -1) {
        mBroadPhaseSystem.addMovedCollisionShape(shape->getBroadPhaseId());
    }
}

// Return a pointer to the world
inline CollisionWorld* CollisionDetection::getWorld() {
    return mWorld;
}

// Return a reference to the memory manager
inline MemoryManager& CollisionDetection::getMemoryManager() const {
    return mMemoryManager;
}

// Update a proxy collision shape (that has moved for instance)
inline void CollisionDetection::updateProxyShape(Entity proxyShapeEntity) {

    // Update the proxy-shape component
    mBroadPhaseSystem.updateProxyShape(proxyShapeEntity);
}

// Update all the enabled proxy-shapes
inline void CollisionDetection::updateProxyShapes() {
    mBroadPhaseSystem.updateProxyShapes();
}

#ifdef IS_PROFILING_ACTIVE

// Set the profiler
inline void CollisionDetection::setProfiler(Profiler* profiler) {
	mProfiler = profiler;
    mBroadPhaseSystem.setProfiler(profiler);
    mCollisionDispatch.setProfiler(profiler);
}

#endif

}

#endif
