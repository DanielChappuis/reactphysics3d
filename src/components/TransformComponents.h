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
#include "components/Components.h"
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
 * The components of the sleeping entities (bodies) are always stored at the end of the array.
 */
class TransformComponents : public Components {

    private:

        // -------------------- Constants -------------------- //

        const size_t COMPONENT_DATA_SIZE = sizeof(Entity) + sizeof(Transform);

        // -------------------- Attributes -------------------- //

        /// Index of the first component of a sleeping entity (sleeping components are stored at the end)
        uint32 mSleepingStartIndex;

        /// Array of entities of each component
        Entity* mEntities;

        /// Array of transform of each component
        Transform* mTransforms;

        // -------------------- Methods -------------------- //

        /// Remove a component at a given index
        void removeComponent(uint32 index);

        /// Destroy a component at a given index
        void destroyComponent(uint32 index);

        /// Move a component from a source to a destination index in the components array
        void moveComponentToIndex(uint32 srcIndex, uint32 destIndex);

        /// Swap two components in the array
        void swapComponents(uint32 index1, uint32 index2);

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
        virtual ~TransformComponents();

        /// Allocate memory for a given number of components
        void allocate(uint32 nbComponentsToAllocate);

        /// Add a component
        void addComponent(Entity entity, bool isSleeping, const TransformComponent& component);

        /// Remove all the components of a given entity
        void removeComponents(Entity entity);

        /// Return the transform of an entity
        Transform& getTransform(Entity entity) const;

        /// Set the transform of an entity
        void setTransform(Entity entity, const Transform& transform);

        /// Notify if a given entity is sleeping or not
        void setIsEntitySleeping(Entity entity, bool isSleeping);
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
