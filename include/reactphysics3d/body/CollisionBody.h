/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2022 Daniel Chappuis                                       *
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

#ifndef REACTPHYSICS3D_COLLISION_BODY_H
#define REACTPHYSICS3D_COLLISION_BODY_H

// Libraries
#include <cassert>
#include <reactphysics3d/engine/Entity.h>
#include <reactphysics3d/collision/shapes/AABB.h>
#include <reactphysics3d/mathematics/Transform.h>
#include <reactphysics3d/configuration.h>

/// Namespace reactphysics3d
namespace reactphysics3d {

// Declarations
class Collider;
class CollisionShape;
class PhysicsWorld;
struct RaycastInfo;
class DefaultPoolAllocator;
class Profiler;
class Logger;

// Class CollisionBody
/**
 * This class represents a body that is able to collide with others
 * bodies.
 */
class CollisionBody {

    protected :

        // -------------------- Attributes -------------------- //

        /// Identifier of the entity in the ECS
        Entity mEntity;

        /// Reference to the world the body belongs to
        PhysicsWorld& mWorld;

#ifdef IS_RP3D_PROFILING_ENABLED

		/// Pointer to the profiler
		Profiler* mProfiler;

#endif

        // -------------------- Methods -------------------- //

        /// Remove all the collision shapes
        void removeAllColliders();

        /// Update the broad-phase state for this body (because it has moved for instance)
        void updateBroadPhaseState() const;

        /// Ask the broad-phase to test again the collision shapes of the body for collision
        /// (as if the body has moved).
        void askForBroadPhaseCollisionCheck() const;

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        CollisionBody(PhysicsWorld& world, Entity entity);

        /// Destructor
        virtual ~CollisionBody();

        /// Deleted copy-constructor
        CollisionBody(const CollisionBody& body) = delete;

        /// Deleted assignment operator
        CollisionBody& operator=(const CollisionBody& body) = delete;

        /// Return the corresponding entity of the body
        Entity getEntity() const;

        /// Return true if the body is active
        bool isActive() const;

        /// Return a pointer to the user data attached to this body
        void* getUserData() const;

        /// Attach user data to this body
        void setUserData(void* userData);

        /// Set whether or not the body is active
        virtual void setIsActive(bool isActive);

        /// Return the current position and orientation
        const Transform& getTransform() const;

        /// Set the current position and orientation
        virtual void setTransform(const Transform& transform);

        /// Create a new collider and add it to the body.
        virtual Collider* addCollider(CollisionShape* collisionShape, const Transform& transform);

        /// Remove a collider from the body
        virtual void removeCollider(Collider* collider);

        /// Return true if a point is inside the collision body
        bool testPointInside(const Vector3& worldPoint) const;

        /// Raycast method with feedback information
        bool raycast(const Ray& ray, RaycastInfo& raycastInfo);

        /// Test if the collision body overlaps with a given AABB
        bool testAABBOverlap(const AABB& worldAABB) const;

        /// Compute and return the AABB of the body by merging all colliders AABBs
        AABB getAABB() const;

        /// Return a const pointer to a given collider of the body
        const Collider* getCollider(uint32 colliderIndex) const;

        /// Return a pointer to a given collider of the body
        Collider* getCollider(uint32 colliderIndex);

        /// Return the number of colliders associated with this body
        uint32 getNbColliders() const;

        /// Return the world-space coordinates of a point given the local-space coordinates of the body
        Vector3 getWorldPoint(const Vector3& localPoint) const;

        /// Return the world-space vector of a vector given in local-space coordinates of the body
        Vector3 getWorldVector(const Vector3& localVector) const;

        /// Return the body local-space coordinates of a point given in the world-space coordinates
        Vector3 getLocalPoint(const Vector3& worldPoint) const;

        /// Return the body local-space coordinates of a vector given in the world-space coordinates
        Vector3 getLocalVector(const Vector3& worldVector) const;

#ifdef IS_RP3D_PROFILING_ENABLED

		/// Set the profiler
		virtual void setProfiler(Profiler* profiler);

#endif

        // -------------------- Friendship -------------------- //

        friend class PhysicsWorld;
        friend class CollisionDetectionSystem;
        friend class BroadPhaseAlgorithm;
        friend class ConvexMeshShape;
        friend class Collider;
};

/// Test if the collision body overlaps with a given AABB
/**
* @param worldAABB The AABB (in world-space coordinates) that will be used to test overlap
* @return True if the given AABB overlaps with the AABB of the collision body
*/
RP3D_FORCE_INLINE bool CollisionBody::testAABBOverlap(const AABB& worldAABB) const {
    return worldAABB.testCollision(getAABB());
}

// Return the corresponding entity of the body
/**
 * @return The entity of the body
 */
RP3D_FORCE_INLINE Entity CollisionBody::getEntity() const {
    return mEntity;
}

#ifdef IS_RP3D_PROFILING_ENABLED

// Set the profiler
RP3D_FORCE_INLINE void CollisionBody::setProfiler(Profiler* profiler) {
	mProfiler = profiler;
}

#endif

}

#endif
