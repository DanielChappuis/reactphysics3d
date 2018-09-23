/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2018 Daniel Chappuis                                       *
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

#ifndef REACTPHYSICS3D_MEMORY_ALLOCATOR_H
#define REACTPHYSICS3D_MEMORY_ALLOCATOR_H

// Libraries
#include <cstring>

/// ReactPhysics3D namespace
namespace reactphysics3d {

// Class MemoryAllocator
/**
 * Abstract class with the basic interface of all the derived memory allocators
 */
class MemoryAllocator {

    public:

        /// Constructor
        MemoryAllocator() = default;

        /// Destructor
        virtual ~MemoryAllocator() = default;

        /// Assignment operator
        MemoryAllocator& operator=(MemoryAllocator& allocator) = default;

        /// Allocate memory of a given size (in bytes) and return a pointer to the
        /// allocated memory.
        virtual void* allocate(size_t size)=0;

        /// Release previously allocated memory.
        virtual void release(void* pointer, size_t size)=0;
};

/**
 * Abstract class with the basic interface of all single frames memory allocators
 */
class SingleFrameAllocator : public MemoryAllocator{
	
    public:

        /// Constructor
        SingleFrameAllocator() = default;

        /// Destructor
        virtual ~SingleFrameAllocator() override = default;

        /// Assignment operator
        SingleFrameAllocator& operator=(SingleFrameAllocator& allocator) = default;

        /// Allocate memory of a given size (in bytes) and return a pointer to the
        /// allocated memory.
        virtual void* allocate(size_t size) override =0;

        /// Release previously allocated memory.
        virtual void release(void* pointer, size_t size) override =0;
		
        /// Reset the marker of the current allocated memory
        virtual void reset()=0;
};

}

#endif
