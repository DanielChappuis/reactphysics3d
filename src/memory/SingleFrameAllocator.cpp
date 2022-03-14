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

// Libraries
#include <reactphysics3d/memory/SingleFrameAllocator.h>
#include <reactphysics3d/memory/MemoryManager.h>
#include <cstdlib>
#include <cassert>

using namespace reactphysics3d;

// Constructor
SingleFrameAllocator::SingleFrameAllocator(MemoryAllocator& baseAllocator) : mBaseAllocator(baseAllocator),
                                           mTotalSizeBytes(INIT_SINGLE_FRAME_ALLOCATOR_NB_BYTES),
                                           mCurrentOffset(0) {

    // Allocate a whole block of memory at the beginning
    mMemoryBufferStart = static_cast<char*>(mBaseAllocator.allocate(mTotalSizeBytes));
    assert(mMemoryBufferStart != nullptr);
}

// Destructor
SingleFrameAllocator::~SingleFrameAllocator() {

    // Release the memory allocated at the beginning
    mBaseAllocator.release(mMemoryBufferStart, mTotalSizeBytes);
}

// Allocate memory of a given size (in bytes) and return a pointer to the
// allocated memory. Allocated memory must be 16-bytes aligned.
void* SingleFrameAllocator::allocate(size_t size) {

    // Lock the method with a mutex
    std::lock_guard<std::mutex> lock(mMutex);

    // Allocate a little bit more memory to make sure we can return an aligned address
    const size_t totalSize = size + GLOBAL_ALIGNMENT;

    // Check that there is enough remaining memory in the buffer
    if (mCurrentOffset + totalSize > mTotalSizeBytes) {

        const size_t previousTotalSizeBytes = mTotalSizeBytes;

        // Multiply the total memory to allocate by two
        mTotalSizeBytes *= 2;

        // Allocate a whole block of memory
        void* allocatedMemory = mBaseAllocator.allocate(mTotalSizeBytes);
        assert(allocatedMemory != nullptr);

        // Check that allocated memory is 16-bytes aligned
        assert(reinterpret_cast<uintptr_t>(allocatedMemory) % GLOBAL_ALIGNMENT == 0);

        char* newMemoryBufferStart = static_cast<char*>(allocatedMemory);

        // Copy the previous memory bloc to new new location
        memcpy(newMemoryBufferStart, mMemoryBufferStart, previousTotalSizeBytes);

        // Release the memory allocated at the beginning
        mBaseAllocator.release(mMemoryBufferStart, previousTotalSizeBytes);

        mMemoryBufferStart = newMemoryBufferStart;
    }

    // Next available memory location
    void* nextAvailableMemory = mMemoryBufferStart + mCurrentOffset;

    // Take care of alignment to make sure that we always return an address to the
    // enforce the global alignment of the library
    uintptr_t currentAdress = reinterpret_cast<uintptr_t>(nextAvailableMemory);

    // Calculate the adjustment by masking off the lower bits of the address, to determine how "misaligned" it is.
    size_t mask = GLOBAL_ALIGNMENT - 1;
    uintptr_t misalignment = currentAdress & mask;
    ptrdiff_t alignmentOffset = GLOBAL_ALIGNMENT - misalignment;

    // Compute the aligned address
    uintptr_t alignedAdress = currentAdress + alignmentOffset;
    nextAvailableMemory = reinterpret_cast<void*>(alignedAdress);

    // Increment the offset
    mCurrentOffset += totalSize;

    // Check that allocated memory is 16-bytes aligned
    assert(reinterpret_cast<uintptr_t>(nextAvailableMemory) % GLOBAL_ALIGNMENT == 0);

    // Return the next available memory location
    return nextAvailableMemory;
}

// Release previously allocated memory.
void SingleFrameAllocator::release(void* /*pointer*/, size_t /*size*/) {

}

// Reset the marker of the current allocated memory
void SingleFrameAllocator::reset() {

    // Lock the method with a mutex
    std::lock_guard<std::mutex> lock(mMutex);

    // Reset the current offset at the beginning of the block
    mCurrentOffset = 0;
}
