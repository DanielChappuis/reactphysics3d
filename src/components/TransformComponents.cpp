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

// Libraries
#include "TransformComponents.h"
#include "engine/EntityManager.h"
#include <cassert>
#include <random>

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;


// Constructor
TransformComponents::TransformComponents(MemoryAllocator& allocator)
                    :mMemoryAllocator(allocator), mNbComponents(0), mNbAllocatedComponents(0),
                     mBuffer(nullptr), mMapEntityToComponentIndex(allocator) {

    // Allocate memory for the components data
    allocate(INIT_ALLOCATED_COMPONENTS);
}

// Destructor
TransformComponents::~TransformComponents() {

    if (mNbAllocatedComponents > 0) {

        // Destroy all the remaining components
        for (uint32 i = 0; i < mNbComponents; i++) {
            destroyComponent(i);
        }

        // Size for the data of a single component (in bytes)
        const size_t totalSizeBytes = mNbAllocatedComponents * COMPONENT_DATA_SIZE;

        // Release the allocated memory
        mMemoryAllocator.release(mBuffer, totalSizeBytes);
    }
}

// Allocate memory for a given number of components
void TransformComponents::allocate(uint32 nbComponentsToAllocate) {

    assert(nbComponentsToAllocate > mNbAllocatedComponents);

    // Size for the data of a single component (in bytes)
    const size_t totalSizeBytes = nbComponentsToAllocate * COMPONENT_DATA_SIZE;

    // Allocate memory
    void* newBuffer = mMemoryAllocator.allocate(totalSizeBytes);
    assert(newBuffer != nullptr);

    // New pointers to components data
    Entity* newEntities = static_cast<Entity*>(newBuffer);
    Transform* newTransforms = reinterpret_cast<Transform*>(newEntities + mNbAllocatedComponents);

    // If there was already components before
    if (mNbAllocatedComponents > 0) {

        // Copy component data from the previous buffer to the new one
        memcpy(newTransforms, mTransforms, mNbComponents * sizeof(Transform));
        memcpy(newEntities, mEntities, mNbComponents * sizeof(Entity));

        // Deallocate previous memory
        mMemoryAllocator.release(mBuffer, mNbAllocatedComponents * COMPONENT_DATA_SIZE);
    }

    mBuffer = newBuffer;
    mEntities = newEntities;
    mTransforms = newTransforms;
    mNbAllocatedComponents = nbComponentsToAllocate;
}

// Add a component
void TransformComponents::addComponent(Entity entity, const TransformComponent& component) {

    // If we need to allocate more components
    if (mNbComponents == mNbAllocatedComponents) {
        allocate(mNbAllocatedComponents * 2);
    }

    // Map the entity with the new component lookup index
    mMapEntityToComponentIndex.add(Pair<Entity, uint32>(entity, mNbComponents));

    // Insert the new component data
    new (mEntities + mNbComponents) Entity(entity);
    new (mTransforms + mNbComponents) Transform(component.transform);

    mNbComponents++;
}

// Perform garbage collection to remove unused components
void TransformComponents::garbageCollection(const EntityManager& entityManager) {

    // TODO : Make sure we call this method each frame

    // We use lazy garbage collection. The idea is to pick random components and destroy
    // them if their corresponding entities have been destroyed. We do this until we hit
    // GARBAGE_COLLECTION_MAX_VALID_ENTITIES in a row. Therefore, it cost almost nothing
    // if there are no destroyed entities and it very quickly destroys components where there
    // are a lot of destroyed entities.

    uint32 nbHitValidEntitiesInARow = 0;

    // For random number generation
    std::random_device rd;
    std::mt19937 eng(rd());

    while (mNbComponents > 0 && nbHitValidEntitiesInARow < GARBAGE_COLLECTION_MAX_VALID_ENTITIES) {

        // Select a random index in the components array
        std::uniform_int_distribution<uint32> distr(0, mNbComponents - 1);
        uint32 i = distr(eng);

        // If the corresponding entity is valid
        if (entityManager.isValid(mEntities[i])) {
            nbHitValidEntitiesInARow++;

            continue;
        }

        nbHitValidEntitiesInARow = 0;

        // Destroy the component
        removeComponent(i);
    }
}

// Remove a component at a given index
void TransformComponents::removeComponent(uint32 index) {

    assert(index < mNbComponents);

    // We want to keep the arrays tightly packed. Therefore, when a component is removed,
    // we replace it with the last element of the array

    const uint32 lastIndex = mNbComponents - 1;

    Entity entity = mEntities[index];
    Entity lastEntity = mEntities[lastIndex];

    if (mNbComponents > 1 && index != lastIndex) {

        // Replace the data of the component to destroy by the data of the last component
        mEntities[index] = mEntities[lastIndex];
        mTransforms[index] = mTransforms[lastIndex];

        // Update the entity to component index mapping
        mMapEntityToComponentIndex[lastEntity] = index;
    }
    else {

        // Call the destructors on the component values
        destroyComponent(index);
    }

    // Update the entity to component index mapping
    mMapEntityToComponentIndex.remove(entity);

    mNbComponents--;
}

// Destroy a component at a given index
void TransformComponents::destroyComponent(uint32 index) {

    mEntities[index].~Entity();
    mTransforms[index].~Transform();
}
