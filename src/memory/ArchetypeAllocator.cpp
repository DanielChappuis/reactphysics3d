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
#include <reactphysics3d/memory/ArchetypeAllocator.h>
#include <reactphysics3d/ecs/Entity.h>

using namespace reactphysics3d;

/// Constructor
ArchetypeAllocator::ArchetypeAllocator(MemoryAllocator& baseAllocator)
                   :mBaseAllocator(baseAllocator){

}

/// Destructor
ArchetypeAllocator::~ArchetypeAllocator() {

#ifndef NDEBUG
        // Check that the allocate() and release() methods have been called the same
        // number of times to avoid memory leaks.
        assert(mNbAllocatedChunks == 0);
#endif

    // Release the allocated memory units
    ChunkHeader* chunk = mFreeChunksList;
    while (chunk != nullptr) {

        ChunkHeader* nextChunk = chunk->nextChunk;

        // Destroy the chunk
        chunk->~ChunkHeader();
        mBaseAllocator.release(static_cast<void*>(chunk), CHUNK_DATA_SIZE + SIZE_CHUNK_HEADER);

        chunk = nextChunk;
    }
}

/// Allocate memory
ArchetypeAllocator::ChunkHeader* ArchetypeAllocator::allocate() {

#ifndef NDEBUG
        mNbAllocatedChunks++;
#endif

    // If there is no chunk in the free list
    if (mFreeChunksList == nullptr) {

        // We need to allocate more memory
        allocateChunk();
    }

    assert(mFreeChunksList != nullptr);

    ChunkHeader* newChunk = mFreeChunksList;

    // Remove the chunk from the free list
    mFreeChunksList = newChunk->nextChunk;
    newChunk->nextChunk = nullptr;

    return newChunk;
}

// Reserve more memory for the allocator
void ArchetypeAllocator::allocateChunk() {

    // Allocate memory
    void* memory = mBaseAllocator.allocate(CHUNK_DATA_SIZE + SIZE_CHUNK_HEADER);
    assert(memory != nullptr);

    // Check that allocated memory is 16-bytes aligned
    assert(reinterpret_cast<uintptr_t>(memory) % GLOBAL_ALIGNMENT == 0);

    // Create a new memory chunk the allocated memory
    ChunkHeader* memoryUnit = new (memory) ChunkHeader();

    memoryUnit->nextChunk = mFreeChunksList;
    mFreeChunksList = memoryUnit;
}

/// Release previously allocated memory
void ArchetypeAllocator::release(ChunkHeader* chunk) {

    assert(chunk->nextChunk == nullptr);

    // Add the chunk back to the free list
    chunk->nextChunk = mFreeChunksList;
    mFreeChunksList = chunk;

    mNbAllocatedChunks--;
}
