/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2016 Daniel Chappuis                                       *
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

#ifndef REACTPHYSICS3D_COLLISION_WORLD_H
#define REACTPHYSICS3D_COLLISION_WORLD_H

// Libraries
#include <vector>
#include <set>
#include <list>
#include <algorithm>
#include "mathematics/mathematics.h"
#include "Profiler.h"
#include "body/CollisionBody.h"
#include "collision/RaycastInfo.h"
#include "OverlappingPair.h"
#include "collision/CollisionDetection.h"
#include "constraint/Joint.h"
#include "constraint/ContactPoint.h"
#include "memory/MemoryManager.h"
#include "EventListener.h"

/// Namespace reactphysics3d
namespace reactphysics3d {

// Declarations
class CollisionCallback;
class OverlapCallback;

// Class CollisionWorld
/**
 * This class represent a world where it is possible to move bodies
 * by hand and to test collision between each other. In this kind of
 * world, the bodies movement is not computed using the laws of physics.
 */
class CollisionWorld {

    protected :

        // -------------------- Attributes -------------------- //

        /// Memory manager
        MemoryManager mMemoryManager;

        /// Reference to the collision detection
        CollisionDetection mCollisionDetection;

        /// All the bodies (rigid and soft) of the world
        std::set<CollisionBody*> mBodies;

        /// Current body ID
        bodyindex mCurrentBodyID;

        /// List of free ID for rigid bodies
        std::vector<luint> mFreeBodiesIDs;

        /// Pointer to an event listener object
        EventListener* mEventListener;

#ifdef IS_PROFILING_ACTIVE

		/// Real-time hierarchical profiler
		Profiler mProfiler;
#endif

        // -------------------- Methods -------------------- //

        /// Return the next available body ID
        bodyindex computeNextAvailableBodyID();

        /// Reset all the contact manifolds linked list of each body
        void resetContactManifoldListsOfBodies();

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        CollisionWorld();

        /// Destructor
        virtual ~CollisionWorld();

        /// Deleted copy-constructor
        CollisionWorld(const CollisionWorld& world) = delete;

        /// Deleted assignment operator
        CollisionWorld& operator=(const CollisionWorld& world) = delete;

        /// Return an iterator to the beginning of the bodies of the physics world
        std::set<CollisionBody*>::iterator getBodiesBeginIterator();

        /// Return an iterator to the end of the bodies of the physics world
        std::set<CollisionBody*>::iterator getBodiesEndIterator();

        /// Create a collision body
        CollisionBody* createCollisionBody(const Transform& transform);

        /// Destroy a collision body
        void destroyCollisionBody(CollisionBody* collisionBody);

        /// Set the collision dispatch configuration
        void setCollisionDispatch(CollisionDispatch* collisionDispatch);

        /// Ray cast method
        void raycast(const Ray& ray, RaycastCallback* raycastCallback,
                     unsigned short raycastWithCategoryMaskBits = 0xFFFF) const;

        /// Test if the AABBs of two bodies overlap
        bool testAABBOverlap(const CollisionBody* body1,
                             const CollisionBody* body2) const;

        /// Report all the bodies that overlap with the AABB in parameter
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

#ifdef IS_PROFILING_ACTIVE

		/// Set the name of the profiler
		void setProfilerName(std::string name);
#endif

        /// Return the current world-space AABB of given proxy shape
        AABB getWorldAABB(const ProxyShape* proxyShape) const;

        // -------------------- Friendship -------------------- //

        friend class CollisionDetection;
        friend class CollisionBody;
        friend class RigidBody;
        friend class ConvexMeshShape;
};

// Return an iterator to the beginning of the bodies of the physics world
/**
 * @return An starting iterator to the set of bodies of the world
 */
inline std::set<CollisionBody*>::iterator CollisionWorld::getBodiesBeginIterator() {
    return mBodies.begin();
}

// Return an iterator to the end of the bodies of the physics world
/**
 * @return An ending iterator to the set of bodies of the world
 */
inline std::set<CollisionBody*>::iterator CollisionWorld::getBodiesEndIterator() {
    return mBodies.end();
}

// Set the collision dispatch configuration
/// This can be used to replace default collision detection algorithms by your
/// custom algorithm for instance.
/**
 * @param CollisionDispatch Pointer to a collision dispatch object describing
 * which collision detection algorithm to use for two given collision shapes
 */
inline void CollisionWorld::setCollisionDispatch(CollisionDispatch* collisionDispatch) {
    mCollisionDetection.setCollisionDispatch(collisionDispatch);
}

// Ray cast method
/**
 * @param ray Ray to use for raycasting
 * @param raycastCallback Pointer to the class with the callback method
 * @param raycastWithCategoryMaskBits Bits mask corresponding to the category of
 *                                    bodies to be raycasted
 */
inline void CollisionWorld::raycast(const Ray& ray,
                                    RaycastCallback* raycastCallback,
                                    unsigned short raycastWithCategoryMaskBits) const {
    mCollisionDetection.raycast(raycastCallback, ray, raycastWithCategoryMaskBits);
}

// Test and report collisions between two bodies
/**
 * @param body1 Pointer to the first body to test
 * @param body2 Pointer to the second body to test
 * @param callback Pointer to the object with the callback method
 */
inline void CollisionWorld::testCollision(CollisionBody* body1, CollisionBody* body2, CollisionCallback* callback) {
    mCollisionDetection.testCollision(body1, body2, callback);
}

// Test and report collisions between a body and all the others bodies of the world
/**
 * @param body Pointer to the body against which we need to test collision
 * @param callback Pointer to the object with the callback method to report contacts
 * @param categoryMaskBits Bits mask corresponding to the category of bodies we need to test collision with
 */
inline void CollisionWorld::testCollision(CollisionBody* body, CollisionCallback* callback, unsigned short categoryMaskBits) {
    mCollisionDetection.testCollision(body, callback, categoryMaskBits);
}

// Test and report collisions between all bodies of the world
/**
 * @param callback Pointer to the object with the callback method to report contacts
 */
inline void CollisionWorld::testCollision(CollisionCallback* callback) {
    mCollisionDetection.testCollision(callback);
}

// Report all the bodies that overlap with the body in parameter
/**
 * @param body Pointer to the collision body to test overlap with
 * @param overlapCallback Pointer to the callback class to report overlap
 * @param categoryMaskBits bits mask used to filter the bodies to test overlap with
 */
inline void CollisionWorld::testOverlap(CollisionBody* body, OverlapCallback* overlapCallback, unsigned short categoryMaskBits) {
    mCollisionDetection.testOverlap(body, overlapCallback, categoryMaskBits);
}

#ifdef IS_PROFILING_ACTIVE

// Set the name of the profiler
inline void CollisionWorld::setProfilerName(std::string name) {
	mProfiler.setName(name);
}

#endif

}

#endif
