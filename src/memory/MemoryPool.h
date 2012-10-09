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

        // -------------------- Constants -------------------- //

        static const uint NB_OBJECTS_FIRST_BLOCK;   // Number of objects allocated in the first block

        // -------------------- Attributes -------------------- //

        // Pointer to the first allocated memory block
        void* mPBlocks;

        // Pointer to the first allocated memory unit
        MemoryUnit* mPAllocatedUnits;

        // Pointer to the first free memory unit
        MemoryUnit* mPFreeUnits;

        // Current number of objects in the pool
        uint mCurrentNbObjects;

        // Current maximum number of objects that can be allocated in the pool
        uint mCapacity;

        // Number of objects to allocate in the next block
        uint mNbObjectsNextBlock;

        // -------------------- Methods -------------------- //

        // Private copy-constructor
        MemoryPool(const MemoryPool& body);

        // Private assignment operator
        MemoryPool& operator=(const MemoryPool& timer);

        // Allocate more memory (more blocks) when needed
        void allocateMemory();

    public:

        // -------------------- Methods -------------------- //

        // Constructor
        MemoryPool(uint capacity = 0) throw(std::bad_alloc);

        // Destructor
        ~MemoryPool();

        // Return the current maximum number of objects allowed in the pool
        uint getCapacity() const;

        // Return the current number of objects in the pool
        uint getCurrentNbObjects() const;

        // Return a pointer to an memory allocated location to store a new object
        void* allocateObject();

        // Tell the pool that an object doesn't need to be store in the pool anymore
        void freeObject(void* pObjectToFree);
};

// static member definition
template<class T> const uint MemoryPool<T>::NB_OBJECTS_FIRST_BLOCK = 100;

// Constructor
// Allocate a large block of memory in order to contain
// a given number of object of the template type T
template<class T>
MemoryPool<T>::MemoryPool(uint capacity) throw(std::bad_alloc)
              : mCurrentNbObjects(0), mCapacity(capacity) {
    mPBlocks = 0;
    mPAllocatedUnits = 0;
    mPFreeUnits = 0;
    mNbObjectsNextBlock = (capacity == 0) ? NB_OBJECTS_FIRST_BLOCK : capacity;

    // Allocate the first memory block if the capacity is
    // different from zero
    if (capacity) allocateMemory();
}

// Destructor
// Deallocate the blocks of memory that have been allocated previously
template<class T>
MemoryPool<T>::~MemoryPool() {
    
    // Check if we have a memory leak
    assert(mCurrentNbObjects == 0);
    
    // Release all the allocated memory blocks
    MemoryBlock* currentBlock = (MemoryBlock*) mPBlocks;
    while(currentBlock) {
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
    if (mCurrentNbObjects == mCapacity) {

        // Allocate a new memory block
        allocateMemory();
    }

    assert(mCurrentNbObjects < mCapacity);
    assert(mPFreeUnits);
    
    MemoryUnit* currentUnit = mPFreeUnits;
    mPFreeUnits = currentUnit->pNext;
    if (mPFreeUnits) {
        mPFreeUnits->pPrevious = 0;
    }

    currentUnit->pNext = mPAllocatedUnits;
    if (mPAllocatedUnits) {
        mPAllocatedUnits->pPrevious = currentUnit;
    }
    mPAllocatedUnits = currentUnit;
    
    mCurrentNbObjects++;

    // Return a pointer to the allocated memory unit
    return (void*)((char*)currentUnit + sizeof(MemoryUnit));
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

    MemoryUnit* currentUnit = (MemoryUnit*)((char*)pObjectToFree - sizeof(MemoryUnit));
    mPAllocatedUnits = currentUnit->pNext;
    if (mPAllocatedUnits) {
        mPAllocatedUnits->pPrevious = 0;
    }

    currentUnit->pNext = mPFreeUnits;
    if (mPFreeUnits) {
        mPFreeUnits->pPrevious = currentUnit;
    }
    mPFreeUnits = currentUnit;
    
    mCurrentNbObjects--;
}

// Allocate more memory. This method is called when there are no
// free memory units available anymore. Therefore, we need to allocate
// a new memory block in order to be able to store several new memory units.
template<class T>
void MemoryPool<T>::allocateMemory() {

    // Compute the size of the new
    size_t sizeBlock = mNbObjectsNextBlock * (sizeof(MemoryUnit) + sizeof(T));

    MemoryBlock* tempBlocks = (MemoryBlock*) mPBlocks;

    // Allocate a new memory block
    mPBlocks = malloc(sizeBlock);

    // Check that the allocation didn't fail
    if (!mPBlocks) throw std::bad_alloc();

    MemoryBlock* block = (MemoryBlock*) mPBlocks;
    block->pNext = tempBlocks;

    // For each allocated memory unit in the new block
    for (uint i=0; i<mNbObjectsNextBlock; i++) {

        // Get the adress of a memory unit
        MemoryUnit* currentUnit = (MemoryUnit*)( (char*)mPBlocks + i *
                                                 (sizeof(T) + sizeof(MemoryUnit)) );

        currentUnit->pPrevious = 0;
        currentUnit->pNext = mPFreeUnits;

        if (mPFreeUnits) {
            mPFreeUnits->pPrevious = currentUnit;
        }

        mPFreeUnits = currentUnit;
    }

    // Update the current capacity of the memory pool
    mCapacity += mNbObjectsNextBlock;

    // The next block will be two times the size of the last
    // allocated memory block
    mNbObjectsNextBlock *= 2;
}


// Return the maximum number of objects allowed in the pool
template<class T>
uint MemoryPool<T>::getCapacity() const {
    return mCapacity;
}

// Return the current number of objects in the pool
template<class T>
uint MemoryPool<T>::getCurrentNbObjects() const {
    return mCurrentNbObjects;
}      

}

#endif

