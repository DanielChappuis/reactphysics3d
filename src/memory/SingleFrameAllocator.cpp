/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2016 Daniel Chappuis                                       *
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
#include "SingleFrameAllocator.h"
#include <cstdlib>
#include <cassert>

using namespace reactphysics3d;

// Constructor
SingleFrameAllocator::SingleFrameAllocator(size_t totalSizeBytes)
    : mTotalSizeBytes(totalSizeBytes), mCurrentOffset(0) {

    // Allocate a whole block of memory at the beginning
    mMemoryBufferStart = static_cast<char*>(malloc(mTotalSizeBytes));
    assert(mMemoryBufferStart != nullptr);
}

// Destructor
SingleFrameAllocator::~SingleFrameAllocator() {

    // Release the memory allocated at the beginning
    free(mMemoryBufferStart);
}


// Allocate memory of a given size (in bytes) and return a pointer to the
// allocated memory.
void* SingleFrameAllocator::allocate(size_t size) {

    // Check that there is enough remaining memory in the buffer
    if (static_cast<size_t>(mCurrentOffset) + size > mTotalSizeBytes) {

        // This should never occur. If it does, you must increase the initial
        // size of memory of this allocator
        assert(false);

        // Return null
        return nullptr;
    }

    // Next available memory location
    void* nextAvailableMemory = mMemoryBufferStart + mCurrentOffset;

    // Increment the offset
    mCurrentOffset += size;

    // Return the next available memory location
    return nextAvailableMemory;
}

// Reset the marker of the current allocated memory
void SingleFrameAllocator::reset() {

    // Reset the current offset at the beginning of the block
    mCurrentOffset = 0;
}
