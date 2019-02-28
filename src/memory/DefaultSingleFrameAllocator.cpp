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
#include "DefaultSingleFrameAllocator.h"
#include "MemoryManager.h"
#include <cstdlib>
#include <cassert>

using namespace reactphysics3d;

// Constructor
DefaultSingleFrameAllocator::DefaultSingleFrameAllocator()
    : mBaseMemoryAllocator(&MemoryManager::getBaseAllocator()),
      mTotalSizeBytes(INIT_SINGLE_FRAME_ALLOCATOR_NB_BYTES),
      mCurrentOffset(0), mNbFramesTooMuchAllocated(0), mNeedToAllocatedMore(false) {

    // Allocate a whole block of memory at the beginning
    mMemoryBufferStart = static_cast<char*>(mBaseMemoryAllocator->allocate(mTotalSizeBytes));
    assert(mMemoryBufferStart != nullptr);
}

// Destructor
DefaultSingleFrameAllocator::~DefaultSingleFrameAllocator() {

    // Release the memory allocated at the beginning
    mBaseMemoryAllocator->release(mMemoryBufferStart, mTotalSizeBytes);
}


// Allocate memory of a given size (in bytes) and return a pointer to the
// allocated memory.
void* DefaultSingleFrameAllocator::allocate(size_t size) {

    // Check that there is enough remaining memory in the buffer
    if (mCurrentOffset + size > mTotalSizeBytes) {

        // We need to allocate more memory next time reset() is called
        mNeedToAllocatedMore = true;

        // Return default memory allocation
        return mBaseMemoryAllocator->allocate(size);
    }

    // Next available memory location
    void* nextAvailableMemory = mMemoryBufferStart + mCurrentOffset;

    // Increment the offset
    mCurrentOffset += size;

    // Return the next available memory location
    return nextAvailableMemory;
}

// Release previously allocated memory.
void DefaultSingleFrameAllocator::release(void* pointer, size_t size) {

    // If allocated memory is not within the single frame allocation range
    char* p = static_cast<char*>(pointer);
    if (p < mMemoryBufferStart || p > mMemoryBufferStart + mTotalSizeBytes) {

        // Use default deallocation
        mBaseMemoryAllocator->release(pointer, size);
    }
}

// Reset the marker of the current allocated memory
void DefaultSingleFrameAllocator::reset() {

    // If too much memory is allocated
    if (mCurrentOffset < mTotalSizeBytes / 2) {

        mNbFramesTooMuchAllocated++;

        if (mNbFramesTooMuchAllocated > NB_FRAMES_UNTIL_SHRINK) {

            // Release the memory allocated at the beginning
            mBaseMemoryAllocator->release(mMemoryBufferStart, mTotalSizeBytes);

            // Divide the total memory to allocate by two
            mTotalSizeBytes /= 2;
            if (mTotalSizeBytes == 0) mTotalSizeBytes = 1;

            // Allocate a whole block of memory at the beginning
            mMemoryBufferStart = static_cast<char*>(mBaseMemoryAllocator->allocate(mTotalSizeBytes));
            assert(mMemoryBufferStart != nullptr);

            mNbFramesTooMuchAllocated = 0;
        }
    }
    else {
        mNbFramesTooMuchAllocated = 0;
    }

    // If we need to allocate more memory
    if (mNeedToAllocatedMore) {

        // Release the memory allocated at the beginning
        mBaseMemoryAllocator->release(mMemoryBufferStart, mTotalSizeBytes);

        // Multiply the total memory to allocate by two
        mTotalSizeBytes *= 2;

        // Allocate a whole block of memory at the beginning
        mMemoryBufferStart = static_cast<char*>(mBaseMemoryAllocator->allocate(mTotalSizeBytes));
        assert(mMemoryBufferStart != nullptr);

        mNeedToAllocatedMore = false;
        mNbFramesTooMuchAllocated = 0;
    }

    // Reset the current offset at the beginning of the block
    mCurrentOffset = 0;
}
