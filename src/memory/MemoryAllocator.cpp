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
#include <reactphysics3d/memory/MemoryAllocator.h>
#include <cassert>

using namespace reactphysics3d;

/// Given a pointer to memory, this method returns the next aligned address
/**
* @param pointer Pointer to a memory location
* @param alignment Desired alignment
* @return Pointer to the next aligned memory location
*/
void* MemoryAllocator::alignAddress(void* pointer, uint8 alignment) {

    ptrdiff_t alignmentOffset;
    return alignAddress(pointer, alignment, alignmentOffset);
}

/// Given a pointer to memory, this method returns the next aligned address and also output the alignment offset
/**
* @param pointer Pointer to a memory location
* @param alignment Desired alignment
* @param outAlignmentOffset Output alignment offset needed to align the initial pointer
* @return Pointer to the next aligned memory location
*/
void* MemoryAllocator::alignAddress(void* pointer, uint8 alignment, ptrdiff_t& outAlignmentOffset) {

    // Take care of alignment to make sure that we always return an address to the
    // enforce the global alignment of the library
    const uintptr_t currentAdress = reinterpret_cast<uintptr_t>(pointer);

    // Calculate the adjustment by masking off the lower bits of the address, to determine how "misaligned" it is.
    const size_t mask = alignment - 1;
    const uintptr_t misalignment = currentAdress & mask;
    outAlignmentOffset = alignment - misalignment;
    assert(outAlignmentOffset <= alignment);

    // Compute the aligned address
    const uintptr_t alignedAdress = currentAdress + outAlignmentOffset;
    void* alignedPointer = reinterpret_cast<void*>(alignedAdress);

    // Check that allocated memory is correctly aligned
    assert(reinterpret_cast<uintptr_t>(alignedPointer) % alignment == 0);

    return alignedPointer;
}
