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

#ifndef REACTPHYSICS3D_COLLIDERS_COMPONENTS_H
#define REACTPHYSICS3D_COLLIDERS_COMPONENTS_H

// Libraries
#include <reactphysics3d/mathematics/Transform.h>
#include <reactphysics3d/engine/Entity.h>
#include <reactphysics3d/containers/Map.h>
#include <reactphysics3d/collision/shapes/AABB.h>
#include <reactphysics3d/components/Components.h>

// ReactPhysics3D namespace
namespace reactphysics3d {

// Class declarations
class MemoryAllocator;
class EntityManager;
class AABB;
class CollisionShape;
class Collider;

// Class ColliderComponents
/**
 * This class represent the component of the ECS that contains data about the the colliders of the
 * different bodies. We also make sure that colliders of sleeping entities (bodies) are
 * always stored at the end of the array.
 */
class ColliderComponents : public Components {

    private:

        // -------------------- Attributes -------------------- //

        /// Array of body entity of each component
        Entity* mBodiesEntities;

        /// Array of collider entity of each component
        Entity* mCollidersEntities;

        /// Array of pointer to the colliders
        Collider** mColliders;

        /// Ids of the colliders for the broad-phase algorithm
        int32* mBroadPhaseIds;

        /// Transform from local-space of the collider to the body-space of its body
        Transform* mLocalToBodyTransforms;

        /// Pointers to the collision shapes of the colliders
        CollisionShape** mCollisionShapes;

        /// Array of bits used to define the collision category of this shape.
        /// You can set a single bit to one to define a category value for this
        /// shape. This value is one (0x0001) by default. This variable can be used
        /// together with the mCollideWithMaskBits variable so that given
        /// categories of shapes collide with each other and do not collide with
        /// other categories.
        unsigned short* mCollisionCategoryBits;

        /// Array of bits mask used to state which collision categories this shape can
        /// collide with. This value is 0xFFFF by default. It means that this
        /// collider will collide with every collision categories by default.
        unsigned short* mCollideWithMaskBits;

        /// Array with the local-to-world transforms of the colliders
        Transform* mLocalToWorldTransforms;

        /// Array with the list of involved overlapping pairs for each collider
        List<uint64>* mOverlappingPairs;

        /// True if the size of the collision shape associated with the collider
        /// has been changed by the user
        bool* mHasCollisionShapeChangedSize;

        /// True if the collider is a trigger
        bool* mIsTrigger;


        // -------------------- Methods -------------------- //

        /// Allocate memory for a given number of components
        virtual void allocate(uint32 nbComponentsToAllocate) override;

        /// Destroy a component at a given index
        virtual void destroyComponent(uint32 index) override;

        /// Move a component from a source to a destination index in the components array
        virtual void moveComponentToIndex(uint32 srcIndex, uint32 destIndex) override;

        /// Swap two components in the array
        virtual void swapComponents(uint32 index1, uint32 index2) override;

    public:

        /// Structure for the data of a collider component
        struct ColliderComponent {

            Entity bodyEntity;
            Collider* collider;
            AABB localBounds;
            const Transform& localToBodyTransform;
            CollisionShape* collisionShape;
            unsigned short collisionCategoryBits;
            unsigned short collideWithMaskBits;
            const Transform& localToWorldTransform;

            /// Constructor
            ColliderComponent(Entity bodyEntity, Collider* collider, AABB localBounds, const Transform& localToBodyTransform,
                                CollisionShape* collisionShape, unsigned short collisionCategoryBits,
                                unsigned short collideWithMaskBits, const Transform& localToWorldTransform)
                 :bodyEntity(bodyEntity), collider(collider), localBounds(localBounds), localToBodyTransform(localToBodyTransform),
                  collisionShape(collisionShape), collisionCategoryBits(collisionCategoryBits), collideWithMaskBits(collideWithMaskBits),
                  localToWorldTransform(localToWorldTransform) {

            }
        };

        // -------------------- Methods -------------------- //

        /// Constructor
        ColliderComponents(MemoryAllocator& allocator);

        /// Destructor
        virtual ~ColliderComponents() override = default;

        /// Add a component
        void addComponent(Entity colliderEntity, bool isSleeping, const ColliderComponent& component);

        /// Return the body entity of a given collider
        Entity getBody(Entity colliderEntity) const;

        /// Return a pointer to a given collider
        Collider* getCollider(Entity colliderEntity) const;

        /// Return the local-to-body transform of a collider
        const Transform& getLocalToBodyTransform(Entity colliderEntity) const;

        /// Set the local-to-body transform of a collider
        void setLocalToBodyTransform(Entity colliderEntity, const Transform& transform);

        /// Return a pointer to the collision shape of a collider
        CollisionShape* getCollisionShape(Entity colliderEntity) const;

        /// Return the broad-phase id of a given collider
        int32 getBroadPhaseId(Entity colliderEntity) const;

        /// Set the broad-phase id of a given collider
        void setBroadPhaseId(Entity colliderEntity, int32 broadPhaseId);

        /// Return the collision category bits of a given collider
        unsigned short getCollisionCategoryBits(Entity colliderEntity) const;

        /// Set the collision category bits of a given collider
        void setCollisionCategoryBits(Entity colliderEntity, unsigned short collisionCategoryBits);

        /// Return the "collide with" mask bits of a given collider
        unsigned short getCollideWithMaskBits(Entity colliderEntity) const;

        /// Set the "collide with" mask bits of a given collider
        void setCollideWithMaskBits(Entity colliderEntity, unsigned short collideWithMaskBits);

        /// Return the local-to-world transform of a collider
        const Transform& getLocalToWorldTransform(Entity colliderEntity) const;

        /// Set the local-to-world transform of a collider
        void setLocalToWorldTransform(Entity colliderEntity, const Transform& transform);

        /// Return a reference to the list of overlapping pairs for a given collider
        List<uint64>& getOverlappingPairs(Entity colliderEntity);

        /// Return true if the size of collision shape of the collider has been changed by the user
        bool getHasCollisionShapeChangedSize(Entity colliderEntity) const;

        /// Set whether the size of collision shape of the collider has been changed by the user
        void setHasCollisionShapeChangedSize(Entity colliderEntity, bool hasCollisionShapeChangedSize);

        /// Return true if a collider is a trigger
        bool getIsTrigger(Entity colliderEntity) const;

        /// Set whether a collider is a trigger
        void setIsTrigger(Entity colliderEntity, bool isTrigger);

        // -------------------- Friendship -------------------- //

        friend class BroadPhaseSystem;
        friend class CollisionDetectionSystem;
        friend class DynamicsSystem;
        friend class OverlappingPairs;
};

// Return the body entity of a given collider
inline Entity ColliderComponents::getBody(Entity colliderEntity) const {

   assert(mMapEntityToComponentIndex.containsKey(colliderEntity));

   return mBodiesEntities[mMapEntityToComponentIndex[colliderEntity]];
}

// Return a pointer to a given collider
inline Collider *ColliderComponents::getCollider(Entity colliderEntity) const {

   assert(mMapEntityToComponentIndex.containsKey(colliderEntity));

   return mColliders[mMapEntityToComponentIndex[colliderEntity]];
}

// Return the local-to-body transform of a collider
inline const Transform& ColliderComponents::getLocalToBodyTransform(Entity colliderEntity) const {

   assert(mMapEntityToComponentIndex.containsKey(colliderEntity));

   return mLocalToBodyTransforms[mMapEntityToComponentIndex[colliderEntity]];
}

// Set the local-to-body transform of a collider
inline void ColliderComponents::setLocalToBodyTransform(Entity colliderEntity, const Transform& transform) {

   assert(mMapEntityToComponentIndex.containsKey(colliderEntity));

   mLocalToBodyTransforms[mMapEntityToComponentIndex[colliderEntity]] = transform;
}

// Return a pointer to the collision shape of a collider
inline CollisionShape* ColliderComponents::getCollisionShape(Entity colliderEntity) const {

   assert(mMapEntityToComponentIndex.containsKey(colliderEntity));

    return mCollisionShapes[mMapEntityToComponentIndex[colliderEntity]];
}

// Return the broad-phase id of a given collider
inline int32 ColliderComponents::getBroadPhaseId(Entity colliderEntity) const {

    assert(mMapEntityToComponentIndex.containsKey(colliderEntity));

    return mBroadPhaseIds[mMapEntityToComponentIndex[colliderEntity]];
}

// Set the broad-phase id of a given collider
inline void ColliderComponents::setBroadPhaseId(Entity colliderEntity, int32 broadPhaseId) {

    assert(mMapEntityToComponentIndex.containsKey(colliderEntity));

    mBroadPhaseIds[mMapEntityToComponentIndex[colliderEntity]] = broadPhaseId;
}

// Return the collision category bits of a given collider
inline unsigned short ColliderComponents::getCollisionCategoryBits(Entity colliderEntity) const {

    assert(mMapEntityToComponentIndex.containsKey(colliderEntity));

    return mCollisionCategoryBits[mMapEntityToComponentIndex[colliderEntity]];
}

// Return the "collide with" mask bits of a given collider
inline unsigned short ColliderComponents::getCollideWithMaskBits(Entity colliderEntity) const {

    assert(mMapEntityToComponentIndex.containsKey(colliderEntity));

    return mCollideWithMaskBits[mMapEntityToComponentIndex[colliderEntity]];
}

// Set the collision category bits of a given collider
inline void ColliderComponents::setCollisionCategoryBits(Entity colliderEntity, unsigned short collisionCategoryBits) {

    assert(mMapEntityToComponentIndex.containsKey(colliderEntity));

    mCollisionCategoryBits[mMapEntityToComponentIndex[colliderEntity]] = collisionCategoryBits;
}

// Set the "collide with" mask bits of a given collider
inline void ColliderComponents::setCollideWithMaskBits(Entity colliderEntity, unsigned short collideWithMaskBits) {

    assert(mMapEntityToComponentIndex.containsKey(colliderEntity));

    mCollideWithMaskBits[mMapEntityToComponentIndex[colliderEntity]] = collideWithMaskBits;
}

// Return the local-to-world transform of a collider
inline const Transform& ColliderComponents::getLocalToWorldTransform(Entity colliderEntity) const {

   assert(mMapEntityToComponentIndex.containsKey(colliderEntity));

   return mLocalToWorldTransforms[mMapEntityToComponentIndex[colliderEntity]];
}

// Set the local-to-world transform of a collider
inline void ColliderComponents::setLocalToWorldTransform(Entity colliderEntity, const Transform& transform) {

   assert(mMapEntityToComponentIndex.containsKey(colliderEntity));

   mLocalToWorldTransforms[mMapEntityToComponentIndex[colliderEntity]] = transform;
}

// Return a reference to the list of overlapping pairs for a given collider
inline List<uint64>& ColliderComponents::getOverlappingPairs(Entity colliderEntity) {

    assert(mMapEntityToComponentIndex.containsKey(colliderEntity));

    return mOverlappingPairs[mMapEntityToComponentIndex[colliderEntity]];
}

// Return true if the size of collision shape of the collider has been changed by the user
inline bool ColliderComponents::getHasCollisionShapeChangedSize(Entity colliderEntity) const {

    assert(mMapEntityToComponentIndex.containsKey(colliderEntity));

    return mHasCollisionShapeChangedSize[mMapEntityToComponentIndex[colliderEntity]];
}

// Return true if the size of collision shape of the collider has been changed by the user
inline void ColliderComponents::setHasCollisionShapeChangedSize(Entity colliderEntity, bool hasCollisionShapeChangedSize) {

    assert(mMapEntityToComponentIndex.containsKey(colliderEntity));

    mHasCollisionShapeChangedSize[mMapEntityToComponentIndex[colliderEntity]] = hasCollisionShapeChangedSize;
}


// Return true if a collider is a trigger
inline bool ColliderComponents::getIsTrigger(Entity colliderEntity) const {

    assert(mMapEntityToComponentIndex.containsKey(colliderEntity));

    return mIsTrigger[mMapEntityToComponentIndex[colliderEntity]];
}

// Set whether a collider is a trigger
inline void ColliderComponents::setIsTrigger(Entity colliderEntity, bool isTrigger) {

    assert(mMapEntityToComponentIndex.containsKey(colliderEntity));

    mIsTrigger[mMapEntityToComponentIndex[colliderEntity]] = isTrigger;
}

}

#endif
