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
                    :Components(allocator), mSleepingStartIndex(0){

    // Allocate memory for the components data
    allocate(INIT_ALLOCATED_COMPONENTS);
}

// Destructor
TransformComponents::~TransformComponents() {

    if (mNbAllocatedComponents > 0) {

        // Destroy all the remaining components
        for (uint32 i = 0; i < mNbComponents; i++) {

            // TODO : MAke sure we do not delete already deleted components
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
    Transform* newTransforms = reinterpret_cast<Transform*>(newEntities + nbComponentsToAllocate);

    // If there was already components before
    if (mNbComponents > 0) {

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
void TransformComponents::addComponent(Entity entity, bool isSleeping, const TransformComponent& component) {

    // If we need to allocate more components
    if (mNbComponents == mNbAllocatedComponents) {
        allocate(mNbAllocatedComponents * 2);
    }

    uint32 index;

    // If the component to add is part of a sleeping entity or there are no sleeping entity
    if (isSleeping) {

        // Add the component at the end of the array
        index = mNbComponents;

        mSleepingStartIndex = index;
    }
    // If the component to add is not part of a sleeping entity
    else {

        // If there already are sleeping components
        if (mSleepingStartIndex != mNbComponents) {

            // Move the first sleeping component to the end of the array
            moveComponentToIndex(mSleepingStartIndex, mNbComponents);
        }

        index = mSleepingStartIndex;

        mSleepingStartIndex++;
    }

    // Insert the new component data
    new (mEntities + index) Entity(entity);
    new (mTransforms + index) Transform(component.transform);

    // Map the entity with the new component lookup index
    mMapEntityToComponentIndex.add(Pair<Entity, uint32>(entity, index));

    mNbComponents++;

    assert(mSleepingStartIndex <= mNbComponents);
    assert(mNbComponents == static_cast<uint32>(mMapEntityToComponentIndex.size()));
}

// Remove a component at a given index
void TransformComponents::removeComponent(uint32 index) {

    assert(index < mNbComponents);

    // We want to keep the arrays tightly packed. Therefore, when a component is removed,
    // we replace it with the last element of the array. But we need to make sure that sleeping
    // and non-sleeping components stay grouped together.

    // Destroy the component
    destroyComponent(index);

    // If the component to remove is sleeping
    if (index >= mSleepingStartIndex) {

        // If the component is not the last one
        if (index != mNbComponents - 1) {

            // We replace it by the last sleeping component
            moveComponentToIndex(mNbComponents - 1, index);
        }
    }
    else {   // If the component to remove is not sleeping

        // If it not the last awake component
        if (index != mSleepingStartIndex - 1) {

            // We replace it by the last awake component
            moveComponentToIndex(mSleepingStartIndex - 1, index);
        }

        // If there are sleeping components at the end
        if (mSleepingStartIndex != mNbComponents) {

            // We replace the last awake component by the last sleeping component
            moveComponentToIndex(mNbComponents - 1, mSleepingStartIndex - 1);
        }

        mSleepingStartIndex--;
    }

    mNbComponents--;

    assert(mSleepingStartIndex <= mNbComponents);
    assert(mNbComponents == static_cast<uint32>(mMapEntityToComponentIndex.size()));
}

// Notify if a given entity is sleeping or not
void TransformComponents::setIsEntitySleeping(Entity entity, bool isSleeping) {

    const uint32 index = mMapEntityToComponentIndex[entity];

    // If the component was sleeping and is not sleeping anymore
    if (!isSleeping && index >= mSleepingStartIndex) {

        assert(mSleepingStartIndex < mNbComponents);

        // If the sleeping component is not the first sleeping component
        if (mSleepingStartIndex != index) {

            // Swap the first sleeping component with the one we need to wake up
            swapComponents(index, mSleepingStartIndex);
        }

        mSleepingStartIndex++;
    }
    // If the component was awake and must now go to sleep
    else if (isSleeping && index < mSleepingStartIndex) {

        assert(mSleepingStartIndex > 0);

        // If the awake component is not the only awake component
        if (index != mSleepingStartIndex - 1) {

            // Swap the last awake component with the one we need to put to sleep
            swapComponents(index, mSleepingStartIndex - 1);
        }

        mSleepingStartIndex--;
    }

    assert(mSleepingStartIndex <= mNbComponents);
    assert(mNbComponents == static_cast<uint32>(mMapEntityToComponentIndex.size()));
}

// Move a component from a source to a destination index in the components array
// The destination location must contain a constructed object
void TransformComponents::moveComponentToIndex(uint32 srcIndex, uint32 destIndex) {

    // Copy the data of the source component to the destination location
    new (mEntities + destIndex) Entity(mEntities[srcIndex]);
    new (mTransforms + destIndex) Transform(mTransforms[srcIndex]);

    const Entity entity = mEntities[srcIndex];

    // Destroy the source component
    destroyComponent(srcIndex);

    // Update the entity to component index mapping
    mMapEntityToComponentIndex.add(Pair<Entity, uint32>(entity, destIndex));

    assert(mMapEntityToComponentIndex[mEntities[destIndex]] == destIndex);
}

// Swap two components in the array
void TransformComponents::swapComponents(uint32 index1, uint32 index2) {

    // Copy component 1 data
    Entity entity1(mEntities[index1]);
    Transform transform1(mTransforms[index1]);

    // Destroy component 1
    destroyComponent(index1);

    moveComponentToIndex(index2, index1);

    // Reconstruct component 1 at component 2 location
    new (mEntities + index2) Entity(entity1);
    new (mTransforms + index2) Transform(transform1);

    // Update the entity to component index mapping
    mMapEntityToComponentIndex.add(Pair<Entity, uint32>(entity1, index2));

    assert(mMapEntityToComponentIndex[mEntities[index1]] == index1);
    assert(mMapEntityToComponentIndex[mEntities[index2]] == index2);
    assert(mNbComponents == static_cast<uint32>(mMapEntityToComponentIndex.size()));
}

// Destroy a component at a given index
void TransformComponents::destroyComponent(uint32 index) {

    assert(index < mNbComponents);
    assert(mMapEntityToComponentIndex[mEntities[index]] == index);

    mMapEntityToComponentIndex.remove(mEntities[index]);

    mEntities[index].~Entity();
    mTransforms[index].~Transform();
}

// Remove all the components of a given entity
void TransformComponents::removeComponents(Entity entity) {

    auto it = mMapEntityToComponentIndex.find(entity);

    // If there is a component for this entity
    if (it != mMapEntityToComponentIndex.end()) {

        // Remove the component
        removeComponent(it->second);
    }

   assert(!mMapEntityToComponentIndex.containsKey(entity));
}
