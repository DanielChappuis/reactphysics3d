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
#include <reactphysics3d/memory/HeapAllocator.h>
#include <reactphysics3d/memory/MemoryManager.h>
#include <cstdlib>
#include <cassert>
#include <iostream>

using namespace reactphysics3d;

size_t HeapAllocator::INIT_ALLOCATED_SIZE = 5 * 1048576;    // 5 Mb

// Constructor
HeapAllocator::HeapAllocator(MemoryAllocator& baseAllocator, size_t initAllocatedMemory)
              : mBaseAllocator(baseAllocator), mAllocatedMemory(0), mMemoryUnits(nullptr), mCachedFreeUnit(nullptr) {

#ifndef NDEBUG
        mNbTimesAllocateMethodCalled = 0;
#endif

    reserve(initAllocatedMemory == 0 ? INIT_ALLOCATED_SIZE : initAllocatedMemory);
}

// Destructor
HeapAllocator::~HeapAllocator() {

#ifndef NDEBUG
        // Check that the allocate() and release() methods have been called the same
        // number of times to avoid memory leaks.
        assert(mNbTimesAllocateMethodCalled == 0);
#endif

    // Release the memory allocated for memory unit
    MemoryUnitHeader* unit = mMemoryUnits;
    while (unit != nullptr) {

        assert(!unit->isAllocated);

        MemoryUnitHeader* nextUnit = unit->nextUnit;

        const size_t unitSize = unit->size;

        // Destroy the unit
        unit->~MemoryUnitHeader();
        mBaseAllocator.release(static_cast<void*>(unit), unitSize + sizeof(MemoryUnitHeader));

        unit = nextUnit;
    }
}

/// Split a memory unit in two units. One of size "size" and the second with
/// left over space. The second unit is put into the free memory units
void HeapAllocator::splitMemoryUnit(MemoryUnitHeader* unit, size_t size) {

    assert(size <= unit->size);
    assert(!unit->isAllocated);

    // Split the free memory unit in two memory units, one with the requested memory size
    // and a second one with the left over space
    if (size + sizeof(MemoryUnitHeader) < unit->size) {

        assert(unit->size - size > 0);

        // Create a new memory unit with left over space
        unsigned char* newUnitLocation = (reinterpret_cast<unsigned char*>(unit)) + sizeof(MemoryUnitHeader) + size;
        MemoryUnitHeader* newUnit = new (static_cast<void*>(newUnitLocation)) MemoryUnitHeader(unit->size - sizeof(MemoryUnitHeader) - size, unit, unit->nextUnit, unit->isNextContiguousMemory);
        assert(newUnit->nextUnit != newUnit);
        unit->nextUnit = newUnit;
        if (newUnit->nextUnit != nullptr) {
            newUnit->nextUnit->previousUnit = newUnit;
        }
        assert(unit->nextUnit != unit);
        unit->isNextContiguousMemory = true;
        unit->size = size;

       assert(unit->previousUnit == nullptr || unit->previousUnit->nextUnit == unit);
       assert(unit->nextUnit == nullptr || unit->nextUnit->previousUnit == unit);

       assert(newUnit->previousUnit == nullptr || newUnit->previousUnit->nextUnit == newUnit);
       assert(newUnit->nextUnit == nullptr || newUnit->nextUnit->previousUnit == newUnit);
    }
}

// Allocate memory of a given size (in bytes) and return a pointer to the
// allocated memory.
void* HeapAllocator::allocate(size_t size) {

    // Lock the method with a mutex
    std::lock_guard<std::mutex> lock(mMutex);

    assert(size > 0);

    // We cannot allocate zero bytes
    if (size == 0) return nullptr;

#ifndef NDEBUG
        mNbTimesAllocateMethodCalled++;
#endif

    MemoryUnitHeader* currentUnit = mMemoryUnits;
    assert(mMemoryUnits->previousUnit == nullptr);

    // If there is a cached free memory unit
    if (mCachedFreeUnit != nullptr) {
        assert(!mCachedFreeUnit->isAllocated);

        // If the cached free memory unit matches the request
        if (size <= mCachedFreeUnit->size) {
            currentUnit = mCachedFreeUnit;
            mCachedFreeUnit = nullptr;
        }
    }

    // For each memory unit
    while (currentUnit != nullptr) {

        // If we have found a free memory unit with size large enough for the allocation request
        if (!currentUnit->isAllocated && size <= currentUnit->size) {

            // Split the free memory unit in two memory units, one with the requested memory size
            // and a second one with the left over space
            splitMemoryUnit(currentUnit, size);

            break;
        }

        currentUnit = currentUnit->nextUnit;
    }

    // If we have not found a large enough memory unit we need to allocate more memory
    if (currentUnit == nullptr) {

        reserve((mAllocatedMemory + size) * 2);

        assert(mCachedFreeUnit != nullptr);
        assert(!mCachedFreeUnit->isAllocated);

        // The cached free memory unit is large enough at this point
        currentUnit = mCachedFreeUnit;

        assert(currentUnit->size >= size);

        splitMemoryUnit(currentUnit, size);
    }

    currentUnit->isAllocated = true;

    // Cache the next memory unit if it is not allocated
    if (currentUnit->nextUnit != nullptr && !currentUnit->nextUnit->isAllocated) {
        mCachedFreeUnit = currentUnit->nextUnit;
    }

    // Return a pointer to the memory area inside the unit
    return static_cast<void*>(reinterpret_cast<unsigned char*>(currentUnit) + sizeof(MemoryUnitHeader));
}

// Release previously allocated memory.
void HeapAllocator::release(void* pointer, size_t size) {

    // Lock the method with a mutex
    std::lock_guard<std::mutex> lock(mMutex);

    assert(size > 0);

    // Cannot release a 0-byte allocated memory
    if (size == 0) return;

#ifndef NDEBUG
        mNbTimesAllocateMethodCalled--;
#endif

    unsigned char* unitLocation = static_cast<unsigned char*>(pointer) - sizeof(MemoryUnitHeader);
    MemoryUnitHeader* unit = reinterpret_cast<MemoryUnitHeader*>(unitLocation);
    assert(unit->isAllocated);
    unit->isAllocated = false;

    MemoryUnitHeader* currentUnit = unit;

    // If the previous unit is not allocated and memory is contiguous to the current unit
    if (unit->previousUnit != nullptr && !unit->previousUnit->isAllocated && unit->previousUnit->isNextContiguousMemory) {

        currentUnit = unit->previousUnit;

        // Merge the two contiguous memory units
        mergeUnits(unit->previousUnit, unit);
    }

    // If the next unit is not allocated and memory is contiguous to the current unit
    if (currentUnit->nextUnit != nullptr && !currentUnit->nextUnit->isAllocated && currentUnit->isNextContiguousMemory) {

        // Merge the two contiguous memory units
        mergeUnits(currentUnit, currentUnit->nextUnit);
    }

    mCachedFreeUnit = currentUnit;
}

// Merge two contiguous memory units that are not allocated.
/// Memory unit 2 will be merged into memory unit 1 and memory unit 2 will be removed
void HeapAllocator::mergeUnits(MemoryUnitHeader* unit1, MemoryUnitHeader* unit2) {

   assert(unit2->previousUnit == unit1);
   assert(unit1->nextUnit == unit2);
   assert(!unit1->isAllocated);
   assert(!unit2->isAllocated);
   assert(unit1->isNextContiguousMemory);

   unit1->size += unit2->size + sizeof(MemoryUnitHeader);
   unit1->nextUnit = unit2->nextUnit;
   assert(unit1->nextUnit != unit1);
   if (unit2->nextUnit != nullptr) {
       unit2->nextUnit->previousUnit = unit1;
   }
   unit1->isNextContiguousMemory = unit2->isNextContiguousMemory;

   // Destroy unit 2
   unit2->~MemoryUnitHeader();

   assert(unit1->previousUnit == nullptr || unit1->previousUnit->nextUnit == unit1);
   assert(unit1->nextUnit == nullptr || unit1->nextUnit->previousUnit == unit1);
}

// Reserve more memory for the allocator
void HeapAllocator::reserve(size_t sizeToAllocate) {

    // Allocate memory
    void* memory = mBaseAllocator.allocate(sizeToAllocate + sizeof(MemoryUnitHeader));
    assert(memory != nullptr);

    // Create a new memory unit for the allocated memory
    MemoryUnitHeader* memoryUnit = new (memory) MemoryUnitHeader(sizeToAllocate, nullptr, mMemoryUnits, false);

    if (mMemoryUnits != nullptr) {
        mMemoryUnits->previousUnit = memoryUnit;
    }

    // Add the memory unit at the beginning of the linked-list of memory units
    mMemoryUnits = memoryUnit;

    mCachedFreeUnit = mMemoryUnits;

    mAllocatedMemory += sizeToAllocate;
}
