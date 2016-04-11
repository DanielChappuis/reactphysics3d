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
#include "MemoryAllocator.h"
#include <cstdlib>
#include <cassert>

using namespace reactphysics3d;

// Initialization of static variables
bool MemoryAllocator::isMapSizeToHeadIndexInitialized = false;
size_t MemoryAllocator::mUnitSizes[NB_HEAPS];
int MemoryAllocator::mMapSizeToHeapIndex[MAX_UNIT_SIZE + 1];

// Constructor
MemoryAllocator::MemoryAllocator() {

    // Allocate some memory to manage the blocks
    mNbAllocatedMemoryBlocks = 64;
    mNbCurrentMemoryBlocks = 0;
    const size_t sizeToAllocate = mNbAllocatedMemoryBlocks * sizeof(MemoryBlock);
    mMemoryBlocks = (MemoryBlock*) malloc(sizeToAllocate);
    memset(mMemoryBlocks, 0, sizeToAllocate);
    memset(mFreeMemoryUnits, 0, sizeof(mFreeMemoryUnits));

#ifndef NDEBUG
        mNbTimesAllocateMethodCalled = 0;
#endif

    // If the mMapSizeToHeapIndex has not been initialized yet
    if (!isMapSizeToHeadIndexInitialized) {

        // Initialize the array that contains the sizes the memory units that will
        // be allocated in each different heap
        for (int i=0; i < NB_HEAPS; i++) {
            mUnitSizes[i] = (i+1) * 8;
        }

        // Initialize the lookup table that maps the size to allocated to the
        // corresponding heap we will use for the allocation
        uint j = 0;
        mMapSizeToHeapIndex[0] = -1;    // This element should not be used
        for (uint i=1; i <= MAX_UNIT_SIZE; i++) {
            if (i <= mUnitSizes[j]) {
                mMapSizeToHeapIndex[i] = j;
            }
            else {
                j++;
                mMapSizeToHeapIndex[i] = j;
            }
        }

        isMapSizeToHeadIndexInitialized = true;
    }
}

// Destructor
MemoryAllocator::~MemoryAllocator() {

    // Release the memory allocated for each block
    for (uint i=0; i<mNbCurrentMemoryBlocks; i++) {
        free(mMemoryBlocks[i].memoryUnits);
    }

    free(mMemoryBlocks);

#ifndef NDEBUG
        // Check that the allocate() and release() methods have been called the same
        // number of times to avoid memory leaks.
        assert(mNbTimesAllocateMethodCalled == 0);
#endif
}

// Allocate memory of a given size (in bytes) and return a pointer to the
// allocated memory.
void* MemoryAllocator::allocate(size_t size) {

    // We cannot allocate zero bytes
    if (size == 0) return NULL;

#ifndef NDEBUG
        mNbTimesAllocateMethodCalled++;
#endif

    // If we need to allocate more than the maximum memory unit size
    if (size > MAX_UNIT_SIZE) {

        // Allocate memory using standard malloc() function
        return malloc(size);
    }

    // Get the index of the heap that will take care of the allocation request
    int indexHeap = mMapSizeToHeapIndex[size];
    assert(indexHeap >= 0 && indexHeap < NB_HEAPS);

    // If there still are free memory units in the corresponding heap
    if (mFreeMemoryUnits[indexHeap] != NULL) {

        // Return a pointer to the memory unit
        MemoryUnit* unit = mFreeMemoryUnits[indexHeap];
        mFreeMemoryUnits[indexHeap] = unit->nextUnit;
        return unit;
    }
    else {  // If there is no more free memory units in the corresponding heap

        // If we need to allocate more memory to containsthe blocks
        if (mNbCurrentMemoryBlocks == mNbAllocatedMemoryBlocks) {

            // Allocate more memory to contain the blocks
            MemoryBlock* currentMemoryBlocks = mMemoryBlocks;
            mNbAllocatedMemoryBlocks += 64;
            mMemoryBlocks = (MemoryBlock*) malloc(mNbAllocatedMemoryBlocks * sizeof(MemoryBlock));
            memcpy(mMemoryBlocks, currentMemoryBlocks,mNbCurrentMemoryBlocks * sizeof(MemoryBlock));
            memset(mMemoryBlocks + mNbCurrentMemoryBlocks, 0, 64 * sizeof(MemoryBlock));
            free(currentMemoryBlocks);
        }

        // Allocate a new memory blocks for the corresponding heap and divide it in many
        // memory units
        MemoryBlock* newBlock = mMemoryBlocks + mNbCurrentMemoryBlocks;
        newBlock->memoryUnits = (MemoryUnit*) malloc(BLOCK_SIZE);
        assert(newBlock->memoryUnits != NULL);
        size_t unitSize = mUnitSizes[indexHeap];
        uint nbUnits = BLOCK_SIZE / unitSize;
        assert(nbUnits * unitSize <= BLOCK_SIZE);
        for (size_t i=0; i < nbUnits - 1; i++) {
            MemoryUnit* unit = (MemoryUnit*) ((size_t)newBlock->memoryUnits + unitSize * i);
            MemoryUnit* nextUnit = (MemoryUnit*) ((size_t)newBlock->memoryUnits + unitSize * (i+1));
            unit->nextUnit = nextUnit;
        }
        MemoryUnit* lastUnit = (MemoryUnit*) ((size_t)newBlock->memoryUnits + unitSize*(nbUnits-1));
        lastUnit->nextUnit = NULL;

        // Add the new allocated block into the list of free memory units in the heap
        mFreeMemoryUnits[indexHeap] = newBlock->memoryUnits->nextUnit;
        mNbCurrentMemoryBlocks++;

        // Return the pointer to the first memory unit of the new allocated block
        return newBlock->memoryUnits;
    }
}

// Release previously allocated memory.
void MemoryAllocator::release(void* pointer, size_t size) {

    // Cannot release a 0-byte allocated memory
    if (size == 0) return;

#ifndef NDEBUG
        mNbTimesAllocateMethodCalled--;
#endif

    // If the size is larger than the maximum memory unit size
    if (size > MAX_UNIT_SIZE) {

        // Release the memory using the standard free() function
        free(pointer);
        return;
    }

    // Get the index of the heap that has handled the corresponding allocation request
    int indexHeap = mMapSizeToHeapIndex[size];
    assert(indexHeap >= 0 && indexHeap < NB_HEAPS);

    // Insert the released memory unit into the list of free memory units of the
    // corresponding heap
    MemoryUnit* releasedUnit = (MemoryUnit*) pointer;
    releasedUnit->nextUnit = mFreeMemoryUnits[indexHeap];
    mFreeMemoryUnits[indexHeap] = releasedUnit;
}
