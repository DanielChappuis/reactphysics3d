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

#ifndef REACTPHYSICS3D_TRANSFORM_COMPONENTS_H
#define REACTPHYSICS3D_TRANSFORM_COMPONENTS_H

// Libraries
#include "mathematics/Transform.h"
#include "engine/Entity.h"
#include "containers/Map.h"

// ReactPhysics3D namespace
namespace reactphysics3d {

// Class declarations
class MemoryAllocator;
class EntityManager;

// Class TransformComponents
/**
 * This class represent the component of the ECS that contains the transforms of the
 * different entities. The position and orientation of the bodies are stored there.
 */
class TransformComponents {

    private:

        // -------------------- Constants -------------------- //

        /// Number of components to allocated at the beginning
        const uint32 INIT_ALLOCATED_COMPONENTS = 10;

        /// Number of valid entities to hit before stopping garbage collection
        const uint32 GARBAGE_COLLECTION_MAX_VALID_ENTITIES = 5;

        const size_t COMPONENT_DATA_SIZE = sizeof(Entity) + sizeof(Transform);

        // -------------------- Attributes -------------------- //

        /// Memory allocator
        MemoryAllocator& mMemoryAllocator;

        /// Current number of components
        uint32 mNbComponents;

        /// Number of allocated components
        uint32 mNbAllocatedComponents;

        /// Allocated memory for all the data of the components
        void* mBuffer;

        /// Map an entity to the index of its component in the array
        Map<Entity, uint32> mMapEntityToComponentIndex;

        /// Array of entities of each component
        Entity* mEntities;

        /// Array of transform of each component
        Transform* mTransforms;

        // -------------------- Methods -------------------- //

        /// Remove a component at a given index
        void removeComponent(uint32 index);

        /// Destroy a component at a given index
        void destroyComponent(uint32 index);

    public:

        /// Structure for the data of a transform component
        struct TransformComponent {

            const Transform& transform;

            /// Constructor
            TransformComponent(const Transform& transform) : transform(transform) {

            }
        };

        // -------------------- Methods -------------------- //

        /// Constructor
        TransformComponents(MemoryAllocator& allocator);

        /// Destructor
        ~TransformComponents();

        /// Allocate memory for a given number of components
        void allocate(uint32 nbComponentsToAllocate);

        /// Add a component
        void addComponent(Entity entity, const TransformComponent& component);

        /// Perform garbage collection to remove unused components
        void garbageCollection(const EntityManager& entityManager);

        /// Return the transform of an entity
        Transform& getTransform(Entity entity) const;

        /// Set the transform of an entity
        void setTransform(Entity entity, const Transform& transform);
};

// Return the transform of an entity
inline Transform& TransformComponents::getTransform(Entity entity) const {
    return mTransforms[mMapEntityToComponentIndex[entity]];
}

// Set the transform of an entity
inline void TransformComponents::setTransform(Entity entity, const Transform& transform) {
    mTransforms[mMapEntityToComponentIndex[entity]] = transform;
}

}

#endif
