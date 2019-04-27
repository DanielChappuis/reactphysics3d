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

#ifndef REACTPHYSICS3D_DYNAMICS_COMPONENTS_H
#define REACTPHYSICS3D_DYNAMICS_COMPONENTS_H

// Libraries
#include "mathematics/Transform.h"
#include "engine/Entity.h"
#include "components/Components.h"
#include "containers/Map.h"

// ReactPhysics3D namespace
namespace reactphysics3d {

// Class declarations
class MemoryAllocator;
class EntityManager;

// Class DynamicsComponents
/**
 * This class represent the component of the ECS that contains the variables concerning dynamics
 * like velocities.
 */
class DynamicsComponents : public Components {

    private:

        // -------------------- Attributes -------------------- //

        /// Array of body entities of each component
        Entity* mBodies;

        /// Array with the linear velocity of each component
        Vector3* mLinearVelocities;

        /// Array with the angular velocity of each component
        Vector3* mAngularVelocities;

        /// Array with the boolean value to know if the body has already been added into an island
        bool* mIsAlreadyInIsland;

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

        /// Structure for the data of a transform component
        struct DynamicsComponent {

            const Vector3& linearVelocity;
            const Vector3& angularVelocity;

            /// Constructor
            DynamicsComponent(const Vector3& linearVelocity, const Vector3& angularVelocity)
                : linearVelocity(linearVelocity), angularVelocity(angularVelocity) {

            }
        };

        // -------------------- Methods -------------------- //

        /// Constructor
        DynamicsComponents(MemoryAllocator& allocator);

        /// Destructor
        virtual ~DynamicsComponents() override = default;

        /// Add a component
        void addComponent(Entity bodyEntity, bool isSleeping, const DynamicsComponent& component);

        /// Return the linear velocity of an entity
        Vector3& getLinearVelocity(Entity bodyEntity) const;

        /// Return the angular velocity of an entity
        Vector3& getAngularVelocity(Entity bodyEntity) const;

        /// Return true if the entity is already in an island
        bool getIsAlreadyInIsland(Entity bodyEntity) const;

        /// Set the linear velocity of an entity
        void setLinearVelocity(Entity bodyEntity, const Vector3& linearVelocity);

        /// Set the angular velocity of an entity
        void setAngularVelocity(Entity bodyEntity, const Vector3& angularVelocity);

        /// Set the value to know if the entity is already in an island
        bool setIsAlreadyInIsland(Entity bodyEntity, bool isAlreadyInIsland);

        // -------------------- Friendship -------------------- //

        friend class BroadPhaseSystem;
        friend class DynamicsWorld;
};

// Return the linear velocity of an entity
inline Vector3& DynamicsComponents::getLinearVelocity(Entity bodyEntity) const {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   return mLinearVelocities[mMapEntityToComponentIndex[bodyEntity]];
}

// Return the angular velocity of an entity
inline Vector3& DynamicsComponents::getAngularVelocity(Entity bodyEntity) const {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   return mAngularVelocities[mMapEntityToComponentIndex[bodyEntity]];
}

// Set the linear velocity of an entity
inline void DynamicsComponents::setLinearVelocity(Entity bodyEntity, const Vector3& linearVelocity) {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   mLinearVelocities[mMapEntityToComponentIndex[bodyEntity]] = linearVelocity;
}

// Set the angular velocity of an entity
inline void DynamicsComponents::setAngularVelocity(Entity bodyEntity, const Vector3& angularVelocity) {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   mAngularVelocities[mMapEntityToComponentIndex[bodyEntity]] = angularVelocity;
}

// Return true if the entity is already in an island
inline bool DynamicsComponents::getIsAlreadyInIsland(Entity bodyEntity) const {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   return mIsAlreadyInIsland[mMapEntityToComponentIndex[bodyEntity]];
}

/// Set the value to know if the entity is already in an island
inline bool DynamicsComponents::setIsAlreadyInIsland(Entity bodyEntity, bool isAlreadyInIsland) {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   mIsAlreadyInIsland[mMapEntityToComponentIndex[bodyEntity]] = isAlreadyInIsland;
}

}

#endif
