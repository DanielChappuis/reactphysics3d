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

#ifndef REACTPHYSICS3D_COMPONENTS_MANAGER_H
#define REACTPHYSICS3D_COMPONENTS_MANAGER_H

// Libraries
#include <reactphysics3d/configuration.h>
#include <reactphysics3d/ecs/Entity.h>
#include <reactphysics3d/ecs/Archetype.h>
#include <reactphysics3d/containers/Map.h>

namespace reactphysics3d {

// Class ComponentsManager
/**
 * This class is responsible to manage the components
 * of the Entity Components System (ECS).
 */
class ComponentsManager {

    public:

    private:

        // ----- Attributes ----- //

        static Entity::ComponentId mCurrentComponentId;

        /// Map an archetype signature with the archetype
        Map<Entity::Signature, Archetype> mMapSignatureToArchetype;

        /// Map a component to its size
        Map<Entity::ComponentId, size_t> mMapComponentToSize;

    public:

        /// Constructor
        ComponentsManager(MemoryAllocator& allocator);

        /// Register a component type
        template<typename T>
        void registerComponent();

        const Entity::Signature& createArchetype();

        /// Return the id of a component type
        template<typename T>
        static Entity::ComponentId getComponentId();

        /// Return the size of a component type
        template<typename T>
        static size_t getComponentSize();

        /// Return the size of a component (given its id)
        size_t getComponentSize(Entity::ComponentId componentId) const;
};

// Register a component
template<typename T>
RP3D_FORCE_INLINE void ComponentsManager::registerComponent() {
    const Entity::ComponentId componentId = getComponentId<T>();

    // Add a mapping between the component and its size
    mMapComponentToSize.add(Pair<Entity::ComponentId, size_t>(componentId, getComponentSize<T>()));
}

// Return the id of a component type
template<typename T>
RP3D_FORCE_INLINE Entity::ComponentId ComponentsManager::getComponentId() {

    static Entity::ComponentId componentId = mCurrentComponentId++;
    return componentId;
}

// Return the size of a component type
template<typename T>
RP3D_FORCE_INLINE size_t ComponentsManager::getComponentSize() {
    return sizeof(T);
}

// Return the size of a component (given its id)
RP3D_FORCE_INLINE size_t ComponentsManager::getComponentSize(Entity::ComponentId componentId) const {
    return mMapComponentToSize[componentId];
}


}

#endif

