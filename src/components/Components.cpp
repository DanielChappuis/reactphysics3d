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
#include "Components.h"
#include <cassert>

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor
Components::Components(MemoryAllocator& allocator, size_t componentDataSize)
    : mMemoryAllocator(allocator), mNbComponents(0), mComponentDataSize(componentDataSize),
      mNbAllocatedComponents(0), mBuffer(nullptr), mMapEntityToComponentIndex(allocator),
      mSleepingStartIndex(0) {

}

Components::~Components() {

    // If there are allocated components
    if (mNbAllocatedComponents > 0) {

        // Destroy all the remaining components
        for (uint32 i = 0; i < mNbComponents; i++) {

            destroyComponent(i);
        }

        // Size for the data of a single component (in bytes)
        const size_t totalSizeBytes = mNbAllocatedComponents * mComponentDataSize;

        // Release the allocated memory
        mMemoryAllocator.release(mBuffer, totalSizeBytes);
    }
}

// Compute the index where we need to insert the new component
uint32 Components::prepareAddComponent(bool isSleeping) {

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

    return index;
}

// Destroy a component at a given index
void Components::destroyComponent(uint32 index) {

    assert(index < mNbComponents);
}

// Remove a component at a given index
void Components::removeComponent(Entity entity) {

    assert(mMapEntityToComponentIndex.containsKey(entity));

    uint index = mMapEntityToComponentIndex[entity];

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
void Components::setIsEntitySleeping(Entity entity, bool isSleeping) {

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
