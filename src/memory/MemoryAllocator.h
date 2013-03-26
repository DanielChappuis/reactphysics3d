/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2010-2013 Daniel Chappuis                                       *
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

#ifndef MEMORY_ALLOCATOR_H
#define MEMORY_ALLOCATOR_H

// Libraries

// Class MemoryAllocator
/**
 * This class is used to efficiently allocate memory on the heap.
 * It allows us to allocate small blocks of memory (smaller than 1024 bytes)
 * efficiently. This implementation is based on the small block allocator
 * described here : http://www.codeproject.com/useritems/Small_Block_Allocator.asp
 */
class MemoryAllocator {

    private :

        // -------------------- Attributes -------------------- //

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        MemoryAllocator();

        /// Destructor
        ~MemoryAllocator();

        /// Allocate memory of a given size (in bytes) and return a pointer to the
        /// allocated memory.
        void* allocate(uint size);

        /// Free previously allocated memory.
        void free(void* pointer);

};

#endif
