/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2024 Daniel Chappuis                                       *
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
              : mBaseAllocator(baseAllocator), mAllocatedMemory(0), mMemoryUnits(nullptr), mFreeUnits(nullptr) {

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
    MemoryUnitHeader* unit = mFreeUnits;
    while (unit != nullptr) {

        assert(!unit->isAllocated);

        MemoryUnitHeader* nextUnit = unit->nextFreeUnit;

        const size_t unitSize = unit->size;

        // Destroy the unit
        unit->~MemoryUnitHeader();
        const size_t sizeHeader = std::ceil(sizeof(MemoryUnitHeader) / float(GLOBAL_ALIGNMENT)) * GLOBAL_ALIGNMENT;
        mBaseAllocator.release(static_cast<void*>(unit), unitSize + sizeHeader);

        unit = nextUnit;
    }
}

/// Split a memory unit in two units. One of size "size" and the second with
/// left over space. The second unit is put into the free memory units
void HeapAllocator::splitMemoryUnit(MemoryUnitHeader* unit, size_t size) {

    assert(!unit->isAllocated);

    // If the size of the unit is large enough to be slit
    if (size + sizeof(MemoryUnitHeader) < unit->size) {

        // Create a new memory unit with left over space
        unsigned char* newUnitLocation = (reinterpret_cast<unsigned char*>(unit)) + sizeof(MemoryUnitHeader) + size;
        MemoryUnitHeader* newUnit = new (static_cast<void*>(newUnitLocation)) MemoryUnitHeader(unit->size - sizeof(MemoryUnitHeader) - size, unit, unit->nextUnit, unit, unit->nextFreeUnit, unit->isNextContiguousMemory);
        assert(newUnit->nextUnit != newUnit);
        unit->nextUnit = newUnit;
        unit->nextFreeUnit = newUnit;
        if (newUnit->nextUnit != nullptr) {
            newUnit->nextUnit->previousUnit = newUnit;
        }
        if (newUnit->nextFreeUnit != nullptr) {
            newUnit->nextFreeUnit->previousFreeUnit = newUnit;
        }

        assert(unit->nextUnit != unit);
        unit->isNextContiguousMemory = true;
        unit->size = size;

       assert(unit->previousUnit == nullptr || unit->previousUnit->nextUnit == unit);
       assert(unit->nextUnit == nullptr || unit->nextUnit->previousUnit == unit);

       assert(unit->previousFreeUnit == nullptr || unit->previousFreeUnit->nextFreeUnit == unit);
       assert(unit->nextFreeUnit == nullptr || unit->nextFreeUnit->previousFreeUnit == unit);

       assert(newUnit->previousUnit == nullptr || newUnit->previousUnit->nextUnit == newUnit);
       assert(newUnit->nextUnit == nullptr || newUnit->nextUnit->previousUnit == newUnit);

       assert(newUnit->previousFreeUnit->nextFreeUnit == newUnit);
       assert(newUnit->nextFreeUnit == nullptr || newUnit->nextFreeUnit->previousFreeUnit == newUnit);

       assert(unit->nextFreeUnit == newUnit);
       assert(newUnit->previousFreeUnit == unit);
       assert(!newUnit->isAllocated);
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

    // Allocate a little bit more memory to make sure we can return an aligned address
    const size_t totalSize = size + GLOBAL_ALIGNMENT;

#ifndef NDEBUG
        mNbTimesAllocateMethodCalled++;
#endif

    MemoryUnitHeader* currentUnit = mFreeUnits;

    // For each free memory unit
    while (currentUnit != nullptr) {

        assert(!currentUnit->isAllocated);

        // If we have found a free memory unit with size large enough for the allocation request
        if (totalSize <= currentUnit->size) {

            break;
        }

        assert(currentUnit->nextFreeUnit == nullptr || currentUnit->nextFreeUnit->previousFreeUnit == currentUnit);

        currentUnit = currentUnit->nextFreeUnit;
    }

    // If we have not found a large enough free memory unit
    if (currentUnit == nullptr) {

        // We need to allocate more memory
        reserve((mAllocatedMemory + totalSize) * 2);

        assert(mFreeUnits != nullptr);

        // The cached free memory unit is large enough at this point
        currentUnit = mFreeUnits;
    }

    // Split the free memory unit in two memory units, one with the requested memory size
    // and a second one with the left over space
    splitMemoryUnit(currentUnit, totalSize);

    assert(currentUnit->size >= totalSize);
    assert(!currentUnit->isAllocated);

    currentUnit->isAllocated = true;

    removeFromFreeUnits(currentUnit);

    // Return a pointer to the memory area inside the unit
    void* allocatedMemory = static_cast<void*>(reinterpret_cast<unsigned char*>(currentUnit) + sizeof(MemoryUnitHeader));

    // Offset the allocated address such that it is properly aligned
    allocatedMemory = computeAlignedAddress(allocatedMemory);

    // Check that allocated memory is 16-bytes aligned
    assert(reinterpret_cast<uintptr_t>(allocatedMemory) % GLOBAL_ALIGNMENT == 0);

    return allocatedMemory;
}

// Return the next aligned memory address
void* HeapAllocator::computeAlignedAddress(void* unalignedAddress) {

    // Take care of alignment to make sure that we always return an address to the
    // enforce the global alignment of the library

    // Compute the aligned address
    ptrdiff_t alignmentOffset;
    void* alignedPointer = alignAddress(unalignedAddress, GLOBAL_ALIGNMENT, alignmentOffset);

    uintptr_t alignedAddress = reinterpret_cast<uintptr_t>(alignedPointer);

    // Store the adjustment in the byte immediately preceding the adjusted address.
    // This way we can find again the original allocated memory address returned by malloc
    // when this memory unit is released.
    assert(alignmentOffset <= GLOBAL_ALIGNMENT);
    uint8* pAlignedMemory = reinterpret_cast<uint8*>(alignedAddress);
    pAlignedMemory[-1] = static_cast<uint8>(alignmentOffset);

    return alignedPointer;
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

    // Read the alignment offset in order to compute the initial allocated
    // raw address (instead of the aligned address)
    const uint8* pAlignedMemory = reinterpret_cast<const uint8*>(pointer);
    const uintptr_t alignedAddress = reinterpret_cast<uintptr_t>(pAlignedMemory);
    const ptrdiff_t alignmentOffset = static_cast<ptrdiff_t>(pAlignedMemory[-1]);
    const uintptr_t initialAddress = alignedAddress - alignmentOffset;
    void* pInitialAddress = reinterpret_cast<void*>(initialAddress);

    unsigned char* unitLocation = static_cast<unsigned char*>(pInitialAddress) - sizeof(MemoryUnitHeader);
    MemoryUnitHeader* unit = reinterpret_cast<MemoryUnitHeader*>(unitLocation);
    assert(unit->isAllocated);
    assert(unit->nextFreeUnit == nullptr);
    assert(unit->previousFreeUnit == nullptr);
    unit->isAllocated = false;

    MemoryUnitHeader* currentUnit = unit;

    // If the previous unit is not allocated and memory is contiguous to the current unit
    if (unit->previousUnit != nullptr && !unit->previousUnit->isAllocated && unit->previousUnit->isNextContiguousMemory) {

        removeFromFreeUnits(unit->previousUnit);

        currentUnit = unit->previousUnit;

        // Merge the two contiguous memory units
        mergeUnits(unit->previousUnit, unit);
    }

    // If the next unit is not allocated and memory is contiguous to the current unit
    if (currentUnit->nextUnit != nullptr && !currentUnit->nextUnit->isAllocated && currentUnit->isNextContiguousMemory) {

        removeFromFreeUnits(unit->nextUnit);

        // Merge the two contiguous memory units
        mergeUnits(currentUnit, currentUnit->nextUnit);
    }

    addToFreeUnits(currentUnit);
}

// Add the unit from the linked-list of free units
void HeapAllocator::addToFreeUnits(MemoryUnitHeader* unit) {

    if (mFreeUnits != nullptr) {
        assert(mFreeUnits->previousFreeUnit == nullptr);
        mFreeUnits->previousFreeUnit = unit;
    }
    unit->nextFreeUnit = mFreeUnits;
    mFreeUnits = unit;
}

// Remove the unit from the linked-list of free units
void HeapAllocator::removeFromFreeUnits(MemoryUnitHeader* unit) {

    if (unit->previousFreeUnit != nullptr) {
        unit->previousFreeUnit->nextFreeUnit = unit->nextFreeUnit;
    }
    if (unit->nextFreeUnit != nullptr) {
        unit->nextFreeUnit->previousFreeUnit = unit->previousFreeUnit;
    }
    if (unit == mFreeUnits) {
        mFreeUnits = unit->nextFreeUnit;
    }
    unit->nextFreeUnit = nullptr;
    unit->previousFreeUnit = nullptr;
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

    const size_t sizeHeader = std::ceil(sizeof(MemoryUnitHeader) / float(GLOBAL_ALIGNMENT)) * GLOBAL_ALIGNMENT;

    // Allocate memory
    void* memory = mBaseAllocator.allocate(sizeToAllocate + sizeHeader);
    assert(memory != nullptr);

    // Check that allocated memory is 16-bytes aligned
    assert(reinterpret_cast<uintptr_t>(memory) % GLOBAL_ALIGNMENT == 0);

    // Create a new memory unit for the allocated memory
    MemoryUnitHeader* memoryUnit = new (memory) MemoryUnitHeader(sizeToAllocate, nullptr, mMemoryUnits, nullptr, mFreeUnits, false);

    if (mFreeUnits != nullptr) {

        assert(mFreeUnits->previousFreeUnit == nullptr);
        mFreeUnits->previousFreeUnit = memoryUnit;
    }

    if (mMemoryUnits != nullptr) {
        mMemoryUnits->previousUnit = memoryUnit;
    }

    // Add the memory unit at the beginning of the linked-list of memory units
    mMemoryUnits = memoryUnit;
    mFreeUnits = mMemoryUnits;

    mAllocatedMemory += sizeToAllocate;
}
