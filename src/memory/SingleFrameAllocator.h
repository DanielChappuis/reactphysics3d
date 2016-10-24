/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2016 Daniel Chappuis                                       *
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
#include <cstring>
#include "configuration.h"

/// ReactPhysics3D namespace
namespace reactphysics3d {

// Class SingleFrameAllocator
/**
 * This class represent a memory allocator used to efficiently allocate
 * memory on the heap that is used during a single frame.
 */
class SingleFrameAllocator {

    private :

        // -------------------- Attributes -------------------- //

        /// Total size (in bytes) of memory of the allocator
        size_t mTotalSizeBytes;

        /// Pointer to the beginning of the allocated memory block
        char* mMemoryBufferStart;

        /// Pointer to the next available memory location in the buffer
        int mCurrentOffset;

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        SingleFrameAllocator(size_t totalSizeBytes);

        /// Destructor
        ~SingleFrameAllocator();

        /// Allocate memory of a given size (in bytes)
        void* allocate(size_t size);

        /// Reset the marker of the current allocated memory
        void reset();

};

}

#endif
