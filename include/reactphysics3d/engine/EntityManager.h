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

#ifndef REACTPHYSICS3D_ENTITY_MANAGER_H
#define REACTPHYSICS3D_ENTITY_MANAGER_H

// Libraries
#include <reactphysics3d/configuration.h>
#include <reactphysics3d/containers/List.h>
#include <reactphysics3d/containers/Deque.h>
#include <reactphysics3d/engine/Entity.h>

/// Namespace reactphysics3d
namespace reactphysics3d {

// Class EntityManager
/**
 * This class is responsible to manage the entities of the ECS.
 */
class EntityManager {

    private:

        // -------------------- Attributes -------------------- //

        /// List storing the generations of the created entities
        List<uint8> mGenerations;

        /// Deque with the indices of destroyed entities that can be reused
        Deque<uint32> mFreeIndices;

        // -------------------- Methods -------------------- //

    public:

        // -------------------- Methods -------------------- //

        /// Constructor
        EntityManager(MemoryAllocator& allocator);

        /// Create a new entity
        Entity createEntity();

        /// Destroy an entity
        void destroyEntity(Entity entity);

        /// Return true if the entity is still valid (not destroyed)
        bool isValid(Entity entity) const;
};

// Return true if the entity is still valid (not destroyed)
inline bool EntityManager::isValid(Entity entity) const {
    return mGenerations[entity.getIndex()] == entity.getGeneration();
}

}

#endif
