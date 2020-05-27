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

#ifndef REACTPHYSICS3D_COLLISION_BODY_COMPONENTS_H
#define REACTPHYSICS3D_COLLISION_BODY_COMPONENTS_H

// Libraries
#include <reactphysics3d/mathematics/Transform.h>
#include <reactphysics3d/engine/Entity.h>
#include <reactphysics3d/components/Components.h>
#include <reactphysics3d/containers/Map.h>

// ReactPhysics3D namespace
namespace reactphysics3d {

// Class declarations
class MemoryAllocator;
class EntityManager;
class CollisionBody;

// Class CollisionBodyComponents
/**
 * This class represent the component of the ECS that contains data about a collision body.
 * The components of the sleeping entities (bodies) are always stored at the end of the array.
 */
class CollisionBodyComponents : public Components {

    private:

        // -------------------- Attributes -------------------- //

        /// Array of body entities of each component
        Entity* mBodiesEntities;

        /// Array of pointers to the corresponding bodies
        CollisionBody** mBodies;

        /// Array with the list of colliders of each body
        List<Entity>* mColliders;

        /// Array of boolean values to know if the body is active.
        bool* mIsActive;

        /// Array of pointers that can be used to attach user data to the body
        void** mUserData;

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

        /// Structure for the data of a collision body component
        struct CollisionBodyComponent {

            CollisionBody* body;

            /// Constructor
            CollisionBodyComponent(CollisionBody* body) : body(body) {

            }
        };

        // -------------------- Methods -------------------- //

        /// Constructor
        CollisionBodyComponents(MemoryAllocator& allocator);

        /// Destructor
        virtual ~CollisionBodyComponents() override = default;

        /// Add a component
        void addComponent(Entity bodyEntity, bool isSleeping, const CollisionBodyComponent& component);

        /// Add a collider to a body component
        void addColliderToBody(Entity bodyEntity, Entity colliderEntity);

        /// Remove a collider from a body component
        void removeColliderFromBody(Entity bodyEntity, Entity colliderEntity);

        /// Return a pointer to a body
        CollisionBody* getBody(Entity bodyEntity);

        /// Return the list of colliders of a body
        const List<Entity>& getColliders(Entity bodyEntity) const;

        /// Return true if the body is active
        bool getIsActive(Entity bodyEntity) const;

        /// Set the value to know if the body is active
        void setIsActive(Entity bodyEntity, bool isActive) const;

        /// Return the user data associated with the body
        void* getUserData(Entity bodyEntity) const;

        /// Set the user data associated with the body
        void setUserData(Entity bodyEntity, void* userData) const;
};

// Add a collider to a body component
inline void CollisionBodyComponents::addColliderToBody(Entity bodyEntity, Entity colliderEntity) {

    assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

    mColliders[mMapEntityToComponentIndex[bodyEntity]].add(colliderEntity);
}

// Remove a collider from a body component
inline void CollisionBodyComponents::removeColliderFromBody(Entity bodyEntity, Entity colliderEntity) {

    assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

    mColliders[mMapEntityToComponentIndex[bodyEntity]].remove(colliderEntity);
}

// Return a pointer to a body
inline CollisionBody *CollisionBodyComponents::getBody(Entity bodyEntity) {

    assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

    return mBodies[mMapEntityToComponentIndex[bodyEntity]];
}

// Return the list of colliders of a body
inline const List<Entity>& CollisionBodyComponents::getColliders(Entity bodyEntity) const {

    assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

    return mColliders[mMapEntityToComponentIndex[bodyEntity]];
}

// Return true if the body is active
inline bool CollisionBodyComponents::getIsActive(Entity bodyEntity) const {

    assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

    return mIsActive[mMapEntityToComponentIndex[bodyEntity]];
}

// Set the value to know if the body is active
inline void CollisionBodyComponents::setIsActive(Entity bodyEntity, bool isActive) const {

    assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

    mIsActive[mMapEntityToComponentIndex[bodyEntity]] = isActive;
}

// Return the user data associated with the body
inline void* CollisionBodyComponents::getUserData(Entity bodyEntity) const {

    assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

    return mUserData[mMapEntityToComponentIndex[bodyEntity]];
}

// Set the user data associated with the body
inline void CollisionBodyComponents::setUserData(Entity bodyEntity, void* userData) const {

    assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

    mUserData[mMapEntityToComponentIndex[bodyEntity]] = userData;
}

}

#endif
