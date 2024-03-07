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

#ifndef REACTPHYSICS3D_COLLIDER_H
#define REACTPHYSICS3D_COLLIDER_H

// Libraries
#include <reactphysics3d/body/Body.h>
#include <reactphysics3d/collision/shapes/CollisionShape.h>
#include <reactphysics3d/engine/Material.h>
#include <reactphysics3d/utils/Logger.h>

namespace  reactphysics3d {

// Declarations
class MemoryManager;

// Class Collider
/**
 * A collider has a collision shape (box, sphere, capsule, ...) and is attached to a
 * RigidBody. A body can have multiple colliders. The collider also have a mass value and a Material
 * with many physics parameters like friction or bounciness. When you create a body, you need to attach
 * at least one collider to it if you want that body to be able to collide in the physics world.
 */
class Collider {

    protected:

        // -------------------- Attributes -------------------- //

        /// Reference to the memory manager
        MemoryManager& mMemoryManager;

        /// Identifier of the entity in the ECS
        Entity mEntity;

        /// Pointer to the parent body
        Body* mBody;

        /// Pointer to user data
        void* mUserData;

#ifdef IS_RP3D_PROFILING_ENABLED

		/// Pointer to the profiler
		Profiler* mProfiler;

#endif

		// -------------------- Methods -------------------- //

        /// Notify the collider that the size of the collision shape has been
        /// changed by the user
        void setHasCollisionShapeChangedSize(bool hasCollisionShapeChangedSize);

    public:

        // -------------------- Methods -------------------- //

        /// Constructor
        Collider(Entity entity, Body* body, MemoryManager& memoryManager);

        /// Destructor
        virtual ~Collider();

        /// Deleted copy-constructor
        Collider(const Collider& collider) = delete;

        /// Deleted assignment operator
        Collider& operator=(const Collider& collider) = delete;

        /// Return the corresponding entity of the collider
        Entity getEntity() const;

        /// Return a pointer to the collision shape
        CollisionShape* getCollisionShape();

        /// Return a const pointer to the collision shape
        const CollisionShape* getCollisionShape() const;

        /// Return the parent body
        Body* getBody() const;

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

        /// Return a reference to the material properties of the collider
        Material& getMaterial();

        /// Set a new material for this collider
        void setMaterial(const Material& material);

        /// Return true if the collider is a trigger
        bool getIsTrigger() const;

        /// Set whether the collider is a trigger
        void setIsTrigger(bool isTrigger) const;

        /// Return true if the collider can generate contacts for the simulation of the associated body
        bool getIsSimulationCollider() const;

        /// Set whether the collider can generate contacts for the simulation of the associated body
        void setIsSimulationCollider(bool isSimulationCollider) const;

        /// Return true if the collider will be part of results of queries on the PhysicsWorld
        bool getIsWorldQueryCollider() const;

        /// Set whether the collider will be part of results of queries on the PhysicsWorld
        void setIsWorldQueryCollider(bool isWorldQueryCollider) const;

#ifdef IS_RP3D_PROFILING_ENABLED

		/// Set the profiler
		void setProfiler(Profiler* profiler);

#endif

        // -------------------- Friendship -------------------- //

        friend class OverlappingPair;
        friend class Body;
        friend class RigidBody;
        friend class BroadPhaseAlgorithm;
        friend class DynamicAABBTree;
        friend class CollisionDetectionSystem;
        friend class PhysicsWorld;
        friend class GJKAlgorithm;
        friend class ConvexMeshShape;
        friend class CollisionShape;
        friend class ContactManifoldSet;
		friend class MiddlePhaseTriangleCallback;

};

// Return the corresponding entity of the collider
/**
 * @return The entity of the collider
 */
RP3D_FORCE_INLINE Entity Collider::getEntity() const {
    return mEntity;
}

// Return the parent body
/**
 * @return Pointer to the parent body
 */
RP3D_FORCE_INLINE Body* Collider::getBody() const {
    return mBody;
}

// Return a pointer to the user data attached to this body
/**
 * @return A pointer to the user data stored into the collider
 */
RP3D_FORCE_INLINE void* Collider::getUserData() const {
    return mUserData;
}

// Attach user data to this body
/**
 * @param userData Pointer to the user data you want to store within the collider
 */
RP3D_FORCE_INLINE void Collider::setUserData(void* userData) {
    mUserData = userData;
}

/// Test if the collider overlaps with a given AABB
/**
* @param worldAABB The AABB (in world-space coordinates) that will be used to test overlap
* @return True if the given AABB overlaps with the AABB of the collision body
*/
RP3D_FORCE_INLINE bool Collider::testAABBOverlap(const AABB& worldAABB) const {
    return worldAABB.testCollision(getWorldAABB());
}

}

#endif
