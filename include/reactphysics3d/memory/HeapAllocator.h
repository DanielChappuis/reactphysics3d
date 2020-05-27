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

#ifndef REACTPHYSICS3D_HEAP_ALLOCATOR_H
#define REACTPHYSICS3D_HEAP_ALLOCATOR_H

// Libraries
#include <reactphysics3d/configuration.h>
#include <reactphysics3d/memory/MemoryAllocator.h>
#include <cassert>
#include <mutex>
#include <reactphysics3d/containers/Map.h>

/// ReactPhysics3D namespace
namespace reactphysics3d {

// Class HeapAllocator
/**
 * This class is used to efficiently allocate memory on the heap.
 * It is used to allocate memory that cannot be allocated in a single frame allocator or a pool allocator.
 */
class HeapAllocator : public MemoryAllocator {

    private :

        // -------------------- Internal Classes -------------------- //

        // Structure MemoryUnitHeader
        /**
         * Represent the header of a memory unit in the heap
         */
        struct MemoryUnitHeader {

            public :

                // -------------------- Attributes -------------------- //

                /// Size in bytes of the allocated memory unit
                size_t size;

                /// True if the memory unit is currently allocated
                bool isAllocated;

                /// Pointer to the previous memory unit
                MemoryUnitHeader* previousUnit;

                /// Pointer to the next memory unit
                MemoryUnitHeader* nextUnit;

                /// True if the next memory unit has been allocated with the same call to malloc()
                bool isNextContiguousMemory;

                // -------------------- Methods -------------------- //

                MemoryUnitHeader(size_t size, MemoryUnitHeader* previousUnit, MemoryUnitHeader* nextUnit, bool isNextContiguousMemory)
                    : size(size), isAllocated(false), previousUnit(previousUnit),
                      nextUnit(nextUnit), isNextContiguousMemory(isNextContiguousMemory) {

                    assert(size > 0);
                }

        };

        // -------------------- Constants -------------------- //

        static size_t INIT_ALLOCATED_SIZE;

        // -------------------- Attributes -------------------- //

        // Mutex
        std::mutex mMutex;

        /// Base memory allocator
        MemoryAllocator& mBaseAllocator;

        /// Allocated memory (in bytes)
        size_t mAllocatedMemory;

        /// Pointer to the first memory unit of the linked-list
        MemoryUnitHeader* mMemoryUnits;

        /// Pointer to a cached free memory unit
        MemoryUnitHeader* mCachedFreeUnit;

#ifndef NDEBUG
        /// This variable is incremented by one when the allocate() method has been
        /// called and decreased by one when the release() method has been called.
        /// This variable is used in debug mode to check that the allocate() and release()
        /// methods are called the same number of times
        int mNbTimesAllocateMethodCalled;
#endif

        // -------------------- Methods -------------------- //
        
        /// Split a memory unit in two units. One of size "size" and the second with
        /// left over space. The second unit is put into the free memory units
        void splitMemoryUnit(MemoryUnitHeader* unit, size_t size);

        // Merge two contiguous memory units that are not allocated.
        void mergeUnits(MemoryUnitHeader* unit1, MemoryUnitHeader* unit2);

        /// Reserve more memory for the allocator
        void reserve(size_t sizeToAllocate);

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        HeapAllocator(MemoryAllocator& baseAllocator, size_t initAllocatedMemory = 0);

        /// Destructor
        virtual ~HeapAllocator() override;

        /// Assignment operator
        HeapAllocator& operator=(HeapAllocator& allocator) = delete;

        /// Allocate memory of a given size (in bytes) and return a pointer to the
        /// allocated memory.
        virtual void* allocate(size_t size) override;

        /// Release previously allocated memory.
        virtual void release(void* pointer, size_t size) override;
};

}

#endif
