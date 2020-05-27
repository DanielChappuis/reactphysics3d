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
#include <reactphysics3d/components/Components.h>
#include <cassert>

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor
Components::Components(MemoryAllocator& allocator, size_t componentDataSize)
    : mMemoryAllocator(allocator), mNbComponents(0), mComponentDataSize(componentDataSize),
      mNbAllocatedComponents(0), mBuffer(nullptr), mMapEntityToComponentIndex(allocator),
      mDisabledStartIndex(0) {

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

    // If the component to add is part of a disabled entity or there are no disabled entity
    if (isSleeping) {

        // Add the component at the end of the array
        index = mNbComponents;
    }
    // If the component to add is not part of a disabled entity
    else {

        // If there already are disabled components
        if (mDisabledStartIndex != mNbComponents) {

            // Move the first disabled component to the end of the array
            moveComponentToIndex(mDisabledStartIndex, mNbComponents);
        }

        index = mDisabledStartIndex;

        mDisabledStartIndex++;
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
    // we replace it with the last element of the array. But we need to make sure that enabled
    // and disabled components stay grouped together.

    // Destroy the component
    destroyComponent(index);

    // If the component to remove is disabled
    if (index >= mDisabledStartIndex) {

        // If the component is not the last one
        if (index != mNbComponents - 1) {

            // We replace it by the last disabled component
            moveComponentToIndex(mNbComponents - 1, index);
        }
    }
    else {   // If the component to remove is enabled

        // If it not the last enabled component
        if (index != mDisabledStartIndex - 1) {

            // We replace it by the last enabled component
            moveComponentToIndex(mDisabledStartIndex - 1, index);
        }

        // If there are disabled components at the end
        if (mDisabledStartIndex != mNbComponents) {

            // We replace the last enabled component by the last disabled component
            moveComponentToIndex(mNbComponents - 1, mDisabledStartIndex - 1);
        }

        mDisabledStartIndex--;
    }

    mNbComponents--;

    assert(mDisabledStartIndex <= mNbComponents);
    assert(mNbComponents == static_cast<uint32>(mMapEntityToComponentIndex.size()));
}

// Notify if a given entity is disabled (sleeping) or not
void Components::setIsEntityDisabled(Entity entity, bool isDisabled) {

    const uint32 index = mMapEntityToComponentIndex[entity];

    // If the component was disabled and is not disabled anymore
    if (!isDisabled && index >= mDisabledStartIndex) {

        assert(mDisabledStartIndex < mNbComponents);

        // If the disabled component is not the first disabled component
        if (mDisabledStartIndex != index) {

            // Swap the first disabled component with the one we need to enable it
            swapComponents(index, mDisabledStartIndex);
        }

        mDisabledStartIndex++;
    }
    // If the component was enabled and must now be disabled
    else if (isDisabled && index < mDisabledStartIndex) {

        assert(mDisabledStartIndex > 0);

        // If the enabled component is not the only enabled component
        if (index != mDisabledStartIndex - 1) {

            // Swap the last enabled component with the one we need to disable
            swapComponents(index, mDisabledStartIndex - 1);
        }

        mDisabledStartIndex--;
    }

    assert(mDisabledStartIndex <= mNbComponents);
    assert(mNbComponents == static_cast<uint32>(mMapEntityToComponentIndex.size()));
}
