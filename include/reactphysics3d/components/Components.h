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

#ifndef REACTPHYSICS3D_COMPONENTS_H
#define REACTPHYSICS3D_COMPONENTS_H

// Libraries
#include <reactphysics3d/configuration.h>
#include <reactphysics3d/engine/Entity.h>
#include <reactphysics3d/containers/Map.h>

// ReactPhysics3D namespace
namespace reactphysics3d {

// Class declarations
class MemoryAllocator;
class EntityManager;

// Class Components
/**
 * This class represent the abstract class to store components of the ECS.
 */
class Components {

    protected:

        // -------------------- Constants -------------------- //


        /// Number of components to allocated at the beginning
        const uint32 INIT_NB_ALLOCATED_COMPONENTS = 10;

        // -------------------- Attributes -------------------- //

        /// Memory allocator
        MemoryAllocator& mMemoryAllocator;

        /// Current number of components
        uint32 mNbComponents;

        /// Size (in bytes) of a single component
        size_t mComponentDataSize;

        /// Number of allocated components
        uint32 mNbAllocatedComponents;

        /// Allocated memory for all the data of the components
        void* mBuffer;

        /// Map an entity to the index of its component in the array
        Map<Entity, uint32> mMapEntityToComponentIndex;

        /// Index of the first component of a disabled (sleeping or inactive) entity
        /// Disabled components are stored at the end of the components array
        uint32 mDisabledStartIndex;

        /// Compute the index where we need to insert the new component
        uint32 prepareAddComponent(bool isSleeping);

        /// Allocate memory for a given number of components
        virtual void allocate(uint32 nbComponentsToAllocate)=0;

        /// Destroy a component at a given index
        virtual void destroyComponent(uint32 index);

        /// Move a component from a source to a destination index in the components array
        virtual void moveComponentToIndex(uint32 srcIndex, uint32 destIndex)=0;

        /// Swap two components in the array
        virtual void swapComponents(uint32 index1, uint32 index2)=0;

    public:

        // -------------------- Methods -------------------- //

        /// Constructor
        Components(MemoryAllocator& allocator, size_t componentDataSize);

        /// Destructor
        virtual ~Components();

        /// Remove a component
        void removeComponent(Entity entity);

        /// Return true if an entity is disabled
        bool getIsEntityDisabled(Entity entity) const;

        /// Notify if a given entity is disabled
        void setIsEntityDisabled(Entity entity, bool isDisabled);

        /// Return true if there is a component for a given entity
        bool hasComponent(Entity entity) const;

        /// Return true if there is a component for a given entiy and if so set the entity index
        bool hasComponentGetIndex(Entity entity, uint32& entityIndex) const;

        /// Return the number of components
        uint32 getNbComponents() const;

        /// Return the number of enabled components
        uint32 getNbEnabledComponents() const;

        /// Return the index in the arrays for a given entity
        uint32 getEntityIndex(Entity entity) const;
};

// Return true if an entity is sleeping
RP3D_FORCE_INLINE bool Components::getIsEntityDisabled(Entity entity) const {
    assert(hasComponent(entity));
    return mMapEntityToComponentIndex[entity] >= mDisabledStartIndex;
}

// Return true if there is a component for a given entity
RP3D_FORCE_INLINE bool Components::hasComponent(Entity entity) const {
    return mMapEntityToComponentIndex.containsKey(entity);
}

// Return true if there is a component for a given entity and if so set the entity index
RP3D_FORCE_INLINE bool Components::hasComponentGetIndex(Entity entity, uint32& entityIndex) const {

    auto it = mMapEntityToComponentIndex.find(entity);

    if (it != mMapEntityToComponentIndex.end()) {
        entityIndex = it->second;
        return true;
    }

    return false;
}

// Return the number of components
RP3D_FORCE_INLINE uint32 Components::getNbComponents() const {
    return mNbComponents;
}

// Return the number of enabled components
RP3D_FORCE_INLINE uint32 Components::getNbEnabledComponents() const {
    return mDisabledStartIndex;
}

// Return the index in the arrays for a given entity
RP3D_FORCE_INLINE uint32 Components::getEntityIndex(Entity entity) const {
    assert(hasComponent(entity));
    return mMapEntityToComponentIndex[entity];
}
}

#endif
