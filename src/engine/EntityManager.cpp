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

// Libraries
#include <reactphysics3d/engine/EntityManager.h>
#include <reactphysics3d/engine/Entity.h>

using namespace reactphysics3d;

// Constructor
EntityManager::EntityManager(MemoryAllocator& allocator)
              :mGenerations(allocator), mFreeIndices(allocator) {

}

// Create a new entity
Entity EntityManager::createEntity() {

    uint32 index;

    // If there are already enough free indices to start using them
    if (mFreeIndices.size() > Entity::MINIMUM_FREE_INDICES) {

        // Recycle an index from the free indices
        index = mFreeIndices.getFront();
        mFreeIndices.popFront();
    }
    else {

        // We start at generation 0
        mGenerations.add(0);

        // Create a new indice
        index = static_cast<uint32>(mGenerations.size()) - 1;

        assert(index < (1 << Entity::ENTITY_INDEX_BITS));
    }

    // Return a new entity
    return Entity(index, mGenerations[index]);
}

// Destroy an entity
void EntityManager::destroyEntity(Entity entity) {

    const uint32 index = entity.getIndex();

    // Increment the generation of this index
    mGenerations[index]++;

    // Add the index into the deque of free indices
    mFreeIndices.addBack(index);
}
