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

#ifndef REACTPHYSICS3D_ARCHETYPE_ALLOCATOR_H
#define REACTPHYSICS3D_ARCHETYPE_ALLOCATOR_H

// Libraries
#include <reactphysics3d/configuration.h>
#include <reactphysics3d/memory/MemoryAllocator.h>
#include <reactphysics3d/ecs/Entity.h>
#include <reactphysics3d/containers/Map.h>

/// ReactPhysics3D namespace
namespace reactphysics3d {

// Class ArchetypeAllocator
/**
 * This class is used to efficiently allocate for the components of the
 * Entity Components System (ECS). An archetype is a set of entities that have the
 * same components. We store the components of the archetypes together close in memory.
 */
class ArchetypeAllocator {

    public :

        // Header of a memory chunk
        struct ChunkHeader {

            /// Pointer to the next chunk in the linked list
            ChunkHeader* nextChunk;

            /// Constructor
            ChunkHeader() : nextChunk(nullptr) {

            }

            /// Return the pointer where the data of the chunk start
            void* getDataStart() {
                unsigned char* pointer = reinterpret_cast<unsigned char*>(this);
                return static_cast<void*>(pointer  + SIZE_CHUNK_HEADER);
            }

        };

        // -------------------- Constants -------------------- //

        /// Default size of the data in a chunk (in bytes)
        static const uint32 CHUNK_DATA_SIZE = 16 * 1024;     // 16 Kb

        /// Size of the chunk header (more space that sizeof(ChunkHeader) for alignment)
        static const size_t SIZE_CHUNK_HEADER = std::ceil(sizeof(ChunkHeader) / float(GLOBAL_ALIGNMENT)) * GLOBAL_ALIGNMENT;

        // -------------------- Attributes -------------------- //

    private:

        /// Base memory allocator
        MemoryAllocator& mBaseAllocator;

        /// Pointer to the first chunk of the linked-list of available chunks
        ChunkHeader* mFreeChunksList = nullptr;

#ifndef NDEBUG

        // Make sure that the number of allocated chunks is equal to the
        // number of release ones
        int mNbAllocatedChunks = 0;
#endif

        // -------------------- Methods -------------------- //

        /// Reserve more memory
        void allocateChunk();

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        ArchetypeAllocator(MemoryAllocator& baseAllocator);

        /// Destructor
        ~ArchetypeAllocator();

        /// Assignment operator
        ArchetypeAllocator& operator=(ArchetypeAllocator& allocator) = delete;

        /// Allocate a chunk of memory
        ChunkHeader* allocate();

        /// Release a previously allocated memory chunk
        void release(ChunkHeader* chunk);
};

}

#endif
