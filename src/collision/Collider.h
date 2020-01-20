/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2019 Daniel Chappuis                                       *
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

#ifndef REACTPHYSICS3D_COLLIDER_H
#define REACTPHYSICS3D_COLLIDER_H

// Libraries
#include "body/CollisionBody.h"
#include "collision/shapes/CollisionShape.h"

namespace  reactphysics3d {

// Declarations
class MemoryManager;

// Class Collider
/**
 * The CollisionShape instances are supposed to be unique for memory optimization. For instance,
 * consider two rigid bodies with the same sphere collision shape. In this situation, we will have
 * a unique instance of SphereShape but we need to differentiate between the two instances during
 * the collision detection. They do not have the same position in the world and they do not
 * belong to the same rigid body. The Collider class is used for that purpose by attaching a
 * rigid body with one of its collision shape. A body can have multiple colliders (one for
 * each collision shape attached to the body).
 */
class Collider {

    protected:

        // -------------------- Attributes -------------------- //

        /// Reference to the memory manager
        MemoryManager& mMemoryManager;

        /// Identifier of the entity in the ECS
        Entity mEntity;

        /// Pointer to the parent body
        CollisionBody* mBody;

        /// Pointer to user data
        void* mUserData;

#ifdef IS_PROFILING_ACTIVE

		/// Pointer to the profiler
		Profiler* mProfiler;

#endif

#ifdef IS_LOGGING_ACTIVE

        /// Logger
        Logger* mLogger;
#endif

		// -------------------- Methods -------------------- //

		/// Return the collision shape
		CollisionShape* getCollisionShape();

    public:

        // -------------------- Methods -------------------- //

        /// Constructor
        Collider(Entity entity, CollisionBody* body, MemoryManager& memoryManager);

        /// Destructor
        virtual ~Collider();

        /// Deleted copy-constructor
        Collider(const Collider& collider) = delete;

        /// Deleted assignment operator
        Collider& operator=(const Collider& collider) = delete;

        /// Return the corresponding entity of the collider
        Entity getEntity() const;

        /// Return the collision shape
        const CollisionShape* getCollisionShape() const;

        /// Return the parent body
        CollisionBody* getBody() const;

        /// Return the mass of the collision shape
        decimal getMass() const;

        /// Return a pointer to the user data attached to this body
        void* getUserData() const;

        /// Attach user data to this body
        void setUserData(void* userData);

        /// Return the local to parent body transform
        const Transform& getLocalToBodyTransform() const;

        /// Set the local to parent body transform
        void setLocalToBodyTransform(const Transform& transform);

        /// Return the local to world transform
        const Transform getLocalToWorldTransform() const;

        /// Return the AABB of the collider in world-space
        const AABB getWorldAABB() const;

        /// Test if the collider overlaps with a given AABB
        bool testAABBOverlap(const AABB& worldAABB) const;

        /// Return true if a point is inside the collision shape
        bool testPointInside(const Vector3& worldPoint);

        /// Raycast method with feedback information
        bool raycast(const Ray& ray, RaycastInfo& raycastInfo);

        /// Return the collision bits mask
        unsigned short getCollideWithMaskBits() const;

        /// Set the collision bits mask
        void setCollideWithMaskBits(unsigned short collideWithMaskBits);

        /// Return the collision category bits
        unsigned short getCollisionCategoryBits() const;

        /// Set the collision category bits
        void setCollisionCategoryBits(unsigned short collisionCategoryBits);

        /// Return the broad-phase id
        int getBroadPhaseId() const;

#ifdef IS_PROFILING_ACTIVE

		/// Set the profiler
		void setProfiler(Profiler* profiler);

#endif

#ifdef IS_LOGGING_ACTIVE

        /// Set the logger
        void setLogger(Logger* logger);
#endif

        // -------------------- Friendship -------------------- //

        friend class OverlappingPair;
        friend class CollisionBody;
        friend class RigidBody;
        friend class BroadPhaseAlgorithm;
        friend class DynamicAABBTree;
        friend class CollisionDetectionSystem;
        friend class CollisionWorld;
        friend class DynamicsWorld;
        friend class GJKAlgorithm;
        friend class ConvexMeshShape;
		friend class ContactManifoldSet;
		friend class MiddlePhaseTriangleCallback;

};

// Return the corresponding entity of the collider
/**
 * @return The entity of the collider
 */
inline Entity Collider::getEntity() const {
    return mEntity;
}

// Return the parent body
/**
 * @return Pointer to the parent body
 */
inline CollisionBody* Collider::getBody() const {
    return mBody;
}

// Return a pointer to the user data attached to this body
/**
 * @return A pointer to the user data stored into the collider
 */
inline void* Collider::getUserData() const {
    return mUserData;
}

// Attach user data to this body
/**
 * @param userData Pointer to the user data you want to store within the collider
 */
inline void Collider::setUserData(void* userData) {
    mUserData = userData;
}

/// Test if the collider overlaps with a given AABB
/**
* @param worldAABB The AABB (in world-space coordinates) that will be used to test overlap
* @return True if the given AABB overlaps with the AABB of the collision body
*/
inline bool Collider::testAABBOverlap(const AABB& worldAABB) const {
    return worldAABB.testCollision(getWorldAABB());
}

#ifdef IS_LOGGING_ACTIVE

// Set the logger
inline void Collider::setLogger(Logger* logger) {

   mLogger = logger;
}

#endif

}

#endif
