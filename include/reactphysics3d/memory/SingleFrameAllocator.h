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

#ifndef REACTPHYSICS3D_SINGLE_FRAME_ALLOCATOR_H
#define REACTPHYSICS3D_SINGLE_FRAME_ALLOCATOR_H

// Libraries
#include <reactphysics3d/memory/MemoryAllocator.h>
#include <reactphysics3d/configuration.h>
#include <mutex>

/// ReactPhysics3D namespace
namespace reactphysics3d {

// Class SingleFrameAllocator
/**
 * This class represent a memory allocator used to efficiently allocate
 * memory on the heap that is used during a single frame.
 */
class SingleFrameAllocator : public MemoryAllocator {

    private :

        // -------------------- Constants -------------------- //

        /// Number of frames to wait before shrinking the allocated
        /// memory if too much is allocated
        static const int NB_FRAMES_UNTIL_SHRINK = 120;

        /// Initial size (in bytes) of the single frame allocator
        size_t INIT_SINGLE_FRAME_ALLOCATOR_NB_BYTES = 1048576; // 1Mb

        // -------------------- Attributes -------------------- //

        /// Mutex
        std::mutex mMutex;

        /// Reference to the base memory allocator
        MemoryAllocator& mBaseAllocator;

        /// Total size (in bytes) of memory of the allocator
        size_t mTotalSizeBytes;

        /// Pointer to the beginning of the allocated memory block
        char* mMemoryBufferStart;

        /// Pointer to the next available memory location in the buffer
        size_t mCurrentOffset;

        /// Current number of frames since we detected too much memory
        /// is allocated
        size_t mNbFramesTooMuchAllocated;

        /// True if we need to allocate more memory in the next reset() call
        bool mNeedToAllocatedMore;

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        SingleFrameAllocator(MemoryAllocator& baseAllocator);

        /// Destructor
        virtual ~SingleFrameAllocator() override;

        /// Assignment operator
        SingleFrameAllocator& operator=(SingleFrameAllocator& allocator) = delete;

        /// Allocate memory of a given size (in bytes)
        virtual void* allocate(size_t size) override;

        /// Release previously allocated memory.
        virtual void release(void* pointer, size_t size) override;

        /// Reset the marker of the current allocated memory
        virtual void reset();
};

}

#endif
