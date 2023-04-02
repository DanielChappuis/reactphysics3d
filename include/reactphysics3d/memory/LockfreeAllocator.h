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

#ifndef REACTPHYSICS3D_LOCKFREE_ALLOCATOR_H
#define REACTPHYSICS3D_LOCKFREE_ALLOCATOR_H

// Libraries
#include <reactphysics3d/memory/MemoryAllocator.h>
#include <cstdlib>
#include <iostream>

/// ReactPhysics3D namespace
namespace reactphysics3d {

// Class DefaultAllocator
/**
 * This class implements various allocation methods that are usefull for debugging 
 * memory.
 */
class LockfreeAllocator : public MemoryAllocator {

    public:
        /// Destructor
        virtual ~LockfreeAllocator() override = default;
        
        /// Assignment operator
        LockfreeAllocator &operator=(LockfreeAllocator &allocator) = default;
        
        /// Allocate memory of a given size (in bytes) and return a pointer to the
        /// allocated memory.
        virtual void *allocate(size_t size) override;

        /// Duplicate a block of memory.
        virtual void *duplicate(const void *vmem);
        
        /// Returns the allocation size of the specified memory block, the block 
        /// has to have benn allocated by this allocator.
        virtual size_t getAllocLen(const void *vmem);
        
        /// Release previously allocated memory.
        virtual void release(void *pointer, size_t /*size*/) override;
};

} // namespace reactphysics3d

#endif
