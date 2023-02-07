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

#ifndef REACTPHYSICS3D_MEMORY_MANAGER_H
#define REACTPHYSICS3D_MEMORY_MANAGER_H

// Libraries
#include <reactphysics3d/memory/DefaultAllocator.h>
#include <reactphysics3d/memory/PoolAllocator.h>
#include <reactphysics3d/memory/HeapAllocator.h>
#include <reactphysics3d/memory/SingleFrameAllocator.h>

/// Namespace ReactPhysics3D
namespace reactphysics3d {

// Declarations
class MemoryAllocator;

// Class MemoryManager
/**
 * The memory manager is used to store the different memory allocators that are used
 * by the library. The base allocator is either the default allocator (malloc/free) of a custom
 * allocated specified by the user. The HeapAllocator is used on top of the base allocator.
 * The SingleFrameAllocator is used for memory that is allocated only during a frame and the PoolAllocator
 * is used to allocated objects of small size. Both SingleFrameAllocator and PoolAllocator will fall back to
 * HeapAllocator if an allocation request cannot be fulfilled.
 */
class MemoryManager {

    private:

       /// Default malloc/free memory allocator
       DefaultAllocator mDefaultAllocator;

       /// Pointer to the base memory allocator to use
       MemoryAllocator* mBaseAllocator;

       /// Memory heap allocator
       HeapAllocator mHeapAllocator;

       /// Memory pool allocator
       PoolAllocator mPoolAllocator;

       /// Single frame stack allocator
       SingleFrameAllocator mSingleFrameAllocator;

    public:

        /// Memory allocation types
       enum class AllocationType {
           Base, 	// Base memory allocator
           Pool,	// Memory pool allocator
           Heap,	// Memory pool allocator
           Frame,   // Single frame memory allocator
       };

       /// Constructor
       MemoryManager(MemoryAllocator* baseAllocator, size_t initAllocatedMemory = 0);

       /// Destructor
       ~MemoryManager() = default;

        /// Allocate memory of a given type
        void* allocate(AllocationType allocationType, size_t size);

        /// Release previously allocated memory.
        void release(AllocationType allocationType, void* pointer, size_t size);

        /// Return the pool allocator
        PoolAllocator& getPoolAllocator();

        /// Return the single frame stack allocator
        SingleFrameAllocator& getSingleFrameAllocator();

        /// Return the heap allocator
        HeapAllocator& getHeapAllocator();

        /// Reset the single frame allocator
        void resetFrameAllocator();
};

// Allocate memory of a given type
RP3D_FORCE_INLINE void* MemoryManager::allocate(AllocationType allocationType, size_t size) {

    void* allocatedMemory = nullptr;

    switch (allocationType) {
       case AllocationType::Base: allocatedMemory = mBaseAllocator->allocate(size); break;
       case AllocationType::Pool: allocatedMemory =  mPoolAllocator.allocate(size); break;
       case AllocationType::Heap: allocatedMemory =  mHeapAllocator.allocate(size); break;
       case AllocationType::Frame: allocatedMemory =  mSingleFrameAllocator.allocate(size); break;
    }

    assert(allocatedMemory == nullptr || reinterpret_cast<uintptr_t>(allocatedMemory) % 8 == 0);

    return allocatedMemory;
}

// Release previously allocated memory.
RP3D_FORCE_INLINE void MemoryManager::release(AllocationType allocationType, void* pointer, size_t size) {

    switch (allocationType) {
       case AllocationType::Base: mBaseAllocator->release(pointer, size); break;
       case AllocationType::Pool: mPoolAllocator.release(pointer, size); break;
       case AllocationType::Heap: mHeapAllocator.release(pointer, size); break;
       case AllocationType::Frame: mSingleFrameAllocator.release(pointer, size); break;
    }
}

// Return the pool allocator
RP3D_FORCE_INLINE PoolAllocator& MemoryManager::getPoolAllocator() {
   return mPoolAllocator;
}

// Return the single frame stack allocator
RP3D_FORCE_INLINE SingleFrameAllocator& MemoryManager::getSingleFrameAllocator() {
   return mSingleFrameAllocator;
}

// Return the heap allocator
RP3D_FORCE_INLINE HeapAllocator& MemoryManager::getHeapAllocator() {
   return mHeapAllocator;
}

// Reset the single frame allocator
RP3D_FORCE_INLINE void MemoryManager::resetFrameAllocator() {
   mSingleFrameAllocator.reset();
}

}

#endif

