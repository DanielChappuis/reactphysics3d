/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2011 Daniel Chappuis                                            *
*********************************************************************************
*                                                                               *
* Permission is hereby granted, free of charge, to any person obtaining a copy  *
* of this software and associated documentation files (the "Software"), to deal *
* in the Software without restriction, including without limitation the rights  *
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell     *
* copies of the Software, and to permit persons to whom the Software is         *
* furnished to do so, subject to the following conditions:                      *
*                                                                               *
* The above copyright notice and this permission notice shall be included in    *
* all copies or substantial portions of the Software.                           *
*                                                                               *
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR    *
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,      *
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE   *
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER        *
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, *
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN     *
* THE SOFTWARE.                                                                 *
********************************************************************************/

// Libraries
#include "MemoryPool.h"
#include <cassert>

using namespace reactphysics3d;

// Constructor
// Allocate a large block of memory in order to contain
// a given number of object of the template type T
template<typename T>
MemoryPool<T>::MemoryPool(uint nbObjectsToAllocate) throw(std::bad_alloc) {
    pMemoryBlock = NULL;
    pAllocatedLinkedList = NULL;
    pFreeLinkedList = NULL;

    // Compute the total memory size that need to be allocated
    memorySize = nbObjectsToAllocate * (sizeof(struct Unit) + sizeof(T));

    // Allocate the whole memory block
    pMemoryBlock = malloc(memorySize);

    // Check that the allocation didn't fail
    if (!pMemoryBlock) throw std::bad_alloc();

    // For each allocated memory unit
    for (uint i=0; i<nbObjectsToAllocate; i++) {
        // Get the adress of a memory unit
        struct Unit* currentUnit = (struct Unit*)( (char*)pMemoryBlock + i * (sizeof(T) + sizeof(struct Unit)) );

        currentUnit->pPrevious = NULL;
        currentUnit->pNext = pFreeLinkedList;

        if (pFreeLinkedList) {
            pFreeLinkedList->pPrevious = currentUnit;
        }

        pFreeLinkedList = currentUnit;
    }

}

// Destructor
// Deallocate the block of memory that has been allocated previously
template<typename T>
MemoryPool<T>::~MemoryPool() {
    // Release the whole memory block
    free(pMemoryBlock);
}

// Return a pointer to an memory allocated location to store a new object
// This method does not allocate memory because it has already been done at the
// beginning but it returns a pointer to a location in the allocated block of
// memory where a new object can be stored
template<typename T>
void* MemoryPool<T>::allocateObject() {
    // If no memory unit is available in the current allocated memory block
    if (!pFreeLinkedList) {
        // TODO : REALLOCATE MEMORY HERE
        assert(false);
    }

    assert(pFreeLinkedList);
    struct Unit* currentUnit = pFreeLinkedList;
    pFreeLinkedList = currentUnit->pNext;
    if (pFreeLinkedList) {
        pFreeLinkedList->pPrevious = NULL;
    }

    currentUnit->pNext = pAllocatedLinkedList;
    if (pAllocatedLinkedList) {
        pAllocatedLinkedList->pPrevious = currentUnit;
    }
    pAllocatedLinkedList = currentUnit;

    // Return a pointer to the allocated memory unit
    return (void*)((char*)currentUnit + sizeof(struct Unit));
}

// Tell the pool that an object doesn't need to be store in the pool anymore
// This method does not deallocate memory because it will be done only at the
// end but it informs the memory pool that an object that was stored in the heap
// does not need to be stored anymore and therefore we can use the corresponding
// location in the pool for another object
template<typename T>
void MemoryPool<T>::freeObject(void* pObjectToFree) {
    // The pointer location must be inside the memory block
    assert(pMemoryBlock<pObjectToFree && pObjectToFree<(void*)((char*)pMemoryBlock + memorySize));

    struct Unit* currentUnit = (struct Unit*)((char*)pObjectToFree - sizeof(struct Unit));
    pAllocatedLinkedList = currentUnit->pNext;
    if (pAllocatedLinkedList) {
        pAllocatedLinkedList->pPrevious = NULL;
    }

    currentUnit->pNext = pFreeLinkedList;
    if (pFreeLinkedList) {
        pFreeLinkedList->pPrevious = currentUnit;
    }
    pFreeLinkedList = currentUnit;
}
