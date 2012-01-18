/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2010-2012 Daniel Chappuis                                       *
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

#ifndef MEMORY_POOL_H
#define	MEMORY_POOL_H

// Libraries
#include "../configuration.h"
#include <cstddef>
#include <cstdlib>
#include <cassert>
#include <new>

// TODO : Check that casting is done correctly in this class using
//        C++ cast operator like reinterpret_cast<>, ...

// ReactPhysics3D namespace
namespace reactphysics3d {

/*  -------------------------------------------------------------------
    Class MemoryPool :
        This class represents a memory pool that allows us to allocate
        dynamic memory at the beginning of the application in order to
        avoid memory fragmentation and also a large number of allocation
        and deallocation.
    -------------------------------------------------------------------
*/
template<class T>
class MemoryPool {
    private:

        // Unit of memory
        struct Unit {
            struct Unit* pNext;         // Pointer to the next memory unit
            struct Unit* pPrevious;     // Pointer to the previous memory unit
        };

        void* pMemoryBlock;                     // Pointer to the whole memory block
        struct Unit* pAllocatedLinkedList;      // Pointer to the linked list of allocated memory units
        struct Unit* pFreeLinkedList;           // Pointer to the linked list of free memory units
        size_t memorySize;                      // Total allocated memory in the pool
        uint currentNbObjects;                  // Current number of objects in the pool
        const uint maxNbObjects;                // Maximum number of objects in the pool

    public:
        MemoryPool(uint nbObjectsToAllocate) throw(std::bad_alloc);     // Constructor
        ~MemoryPool();                                                  // Destructor

        uint getMaxNbObjects() const;           // Return the maximum number of objects allowed in the pool
        uint getCurrentNbObjects() const;       // Return the current number of objects in the pool
        void* allocateObject();                 // Return a pointer to an memory allocated location to store a new object
        void freeObject(void* pObjectToFree);   // Tell the pool that an object doesn't need to be store in the pool anymore
};


// Constructor
// Allocate a large block of memory in order to contain
// a given number of object of the template type T
template<class T>
MemoryPool<T>::MemoryPool(uint nbObjectsToAllocate) throw(std::bad_alloc)
              : currentNbObjects(0), maxNbObjects(nbObjectsToAllocate) {
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
template<class T>
MemoryPool<T>::~MemoryPool() {
    
    // Check if we have a memory leak
    assert(currentNbObjects == 0);
    
    // Release the whole memory block
    free(pMemoryBlock);
}

// Return a pointer to an memory allocated location to store a new object
// This method does not allocate memory because it has already been done at the
// beginning but it returns a pointer to a location in the allocated block of
// memory where a new object can be stored
template<class T>
void* MemoryPool<T>::allocateObject() {
    // If no memory unit is available in the current allocated memory block
    assert(currentNbObjects < maxNbObjects);
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
    
    currentNbObjects++;

    // Return a pointer to the allocated memory unit
    return (void*)((char*)currentUnit + sizeof(struct Unit));
}

// Tell the pool that an object doesn't need to be store in the pool anymore
// This method does not deallocate memory because it will be done only at the
// end but it informs the memory pool that an object that was stored in the heap
// does not need to be stored anymore and therefore we can use the corresponding
// location in the pool for another object
template<class T>
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
    
    currentNbObjects--;
}

// Return the maximum number of objects allowed in the pool
template<class T>
uint MemoryPool<T>::getMaxNbObjects() const {
    return maxNbObjects;
}

// Return the current number of objects in the pool
template<class T>
uint MemoryPool<T>::getCurrentNbObjects() const {
    return currentNbObjects;
}      

}

#endif

