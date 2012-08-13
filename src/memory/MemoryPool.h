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

        // MemoryUnit represents a unit of memory
        struct MemoryUnit {
            struct MemoryUnit* pNext;               // Pointer to the next memory unit
            struct MemoryUnit* pPrevious;           // Pointer to the previous memory unit
        };

        // Memory block (that contains several memory units)
        struct MemoryBlock {
            struct MemoryBlock* pNext;              // Pointer to the next memory block
        };

        static const uint NB_OBJECTS_FIRST_BLOCK;   // Number of objects allocated in the first block
        void* pBlocks;                              // Pointer to the first allocated memory block
        struct MemoryUnit* pAllocatedUnits;         // Pointer to the first allocated memory unit
        struct MemoryUnit* pFreeUnits;              // Pointer to the first free memory unit
        uint currentNbObjects;                      // Current number of objects in the pool
        uint capacity;                              // Current maximum number of objects that can be allocated in the pool
        uint nbObjectsNextBlock;                    // Number of objects to allocate in the next block
        void allocateMemory();                      // Allocate more memory (more blocks) when needed

    public:
        MemoryPool(uint capacity = 0) throw(std::bad_alloc);    // Constructor
        ~MemoryPool();                                          // Destructor

        uint getCapacity() const;               // Return the current maximum number of objects allowed in the pool
        uint getCurrentNbObjects() const;       // Return the current number of objects in the pool
        void* allocateObject();                 // Return a pointer to an memory allocated location to store a new object
        void freeObject(void* pObjectToFree);   // Tell the pool that an object doesn't need to be store in the pool anymore
};

// static member definition
template<class T> const uint MemoryPool<T>::NB_OBJECTS_FIRST_BLOCK = 100;

// Constructor
// Allocate a large block of memory in order to contain
// a given number of object of the template type T
template<class T>
MemoryPool<T>::MemoryPool(uint capacity) throw(std::bad_alloc)
              : currentNbObjects(0), capacity(capacity) {
    pBlocks = 0;
    pAllocatedUnits = 0;
    pFreeUnits = 0;
    nbObjectsNextBlock = (capacity == 0) ? NB_OBJECTS_FIRST_BLOCK : capacity;

    // Allocate the first memory block if the capacity is
    // different from zero
    if (capacity) allocateMemory();
}

// Destructor
// Deallocate the blocks of memory that have been allocated previously
template<class T>
MemoryPool<T>::~MemoryPool() {
    
    // Check if we have a memory leak
    assert(currentNbObjects == 0);
    
    // Release all the allocated memory blocks
    struct MemoryBlock* currentBlock = (struct MemoryBlock*) pBlocks;
    while(currentBlock != 0) {
        MemoryBlock* tempBlock = currentBlock->pNext;
        free(currentBlock);
        currentBlock = tempBlock;
    }
}

// Return a pointer to a memory allocated location to store a new object
// This method only allocates memory if needed and it returns a pointer
// to a location in an allocated block of memory where a new object can be stored
template<class T>
void* MemoryPool<T>::allocateObject() {

    // If there is not enough allocated memory in the pool
    if (currentNbObjects == capacity) {

        // Allocate a new memory block
        allocateMemory();
    }

    assert(currentNbObjects < capacity);
    assert(pFreeUnits);
    
    struct MemoryUnit* currentUnit = pFreeUnits;
    pFreeUnits = currentUnit->pNext;
    if (pFreeUnits) {
        pFreeUnits->pPrevious = 0;
    }

    currentUnit->pNext = pAllocatedUnits;
    if (pAllocatedUnits) {
        pAllocatedUnits->pPrevious = currentUnit;
    }
    pAllocatedUnits = currentUnit;
    
    currentNbObjects++;

    // Return a pointer to the allocated memory unit
    return (void*)((char*)currentUnit + sizeof(struct MemoryUnit));
}

// Tell the pool that an object does not need to be stored in the pool anymore
// This method does not deallocate memory because it will be done only at the
// end but it notifies the memory pool that an object that was stored in the pool
// does not need to be stored anymore and therefore we can use the corresponding
// location in the pool for another object
template<class T>
void MemoryPool<T>::freeObject(void* pObjectToFree) {

    // The pointer location must be inside the memory block
    //assert(pBlocks<pObjectToFree && pObjectToFree<(void*)((char*)pBlocks + memorySize));

    struct MemoryUnit* currentUnit = (struct MemoryUnit*)((char*)pObjectToFree - sizeof(struct MemoryUnit));
    pAllocatedUnits = currentUnit->pNext;
    if (pAllocatedUnits) {
        pAllocatedUnits->pPrevious = 0;
    }

    currentUnit->pNext = pFreeUnits;
    if (pFreeUnits) {
        pFreeUnits->pPrevious = currentUnit;
    }
    pFreeUnits = currentUnit;
    
    currentNbObjects--;
}

// Allocate more memory. This method is called when there are no
// free memory units available anymore. Therefore, we need to allocate
// a new memory block in order to be able to store several new memory units.
template<class T>
void MemoryPool<T>::allocateMemory() {

    // Compute the size of the new
    size_t sizeBlock = nbObjectsNextBlock * (sizeof(struct MemoryUnit) + sizeof(T));

    struct MemoryBlock* tempBlocks = (struct MemoryBlock*) pBlocks;

    // Allocate a new memory block
    pBlocks = malloc(sizeBlock);

    // Check that the allocation didn't fail
    if (!pBlocks) throw std::bad_alloc();

    struct MemoryBlock* block = (struct MemoryBlock*) pBlocks;
    block->pNext = tempBlocks;

    // For each allocated memory unit in the new block
    for (uint i=0; i<nbObjectsNextBlock; i++) {

        // Get the adress of a memory unit
        struct MemoryUnit* currentUnit = (struct MemoryUnit*)( (char*)pBlocks + i * (sizeof(T) + sizeof(struct MemoryUnit)) );

        currentUnit->pPrevious = 0;
        currentUnit->pNext = pFreeUnits;

        if (pFreeUnits) {
            pFreeUnits->pPrevious = currentUnit;
        }

        pFreeUnits = currentUnit;
    }

    // Update the current capacity of the memory pool
    capacity += nbObjectsNextBlock;

    // The next block will be two times the size of the last
    // allocated memory block
    nbObjectsNextBlock *= 2;
}


// Return the maximum number of objects allowed in the pool
template<class T>
uint MemoryPool<T>::getCapacity() const {
    return capacity;
}

// Return the current number of objects in the pool
template<class T>
uint MemoryPool<T>::getCurrentNbObjects() const {
    return currentNbObjects;
}      

}

#endif

