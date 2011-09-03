/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2011 Daniel Chappuis                                            *
*********************************************************************************
*                                                                               *
* Permission is hereby granted, free of charge, to any person obtaining a copy  *
* of this software and associated documentation files (the "Software"), to deal *
* in the Software without restriction, including without limitation the rights  *
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell     *
* copies of the Software, and to permit persons to whom the Software is         *
* furnished to do so, subject to the following conditions:                      *
*                                                                               *
* The above copyright notice and this permission notice shall be included in    *
* all copies or substantial portions of the Software.                           *
*                                                                               *
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR    *
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,      *
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE   *
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER        *
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, *
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN     *
* THE SOFTWARE.                                                                 *
********************************************************************************/

#ifndef MEMORY_POOL_H
#define	MEMORY_POOL_H

// Libraries
#include "../constants.h"
#include <cstddef>

// ReactPhysics3D namespace
namespace reactphysics3d {

/*  -------------------------------------------------------------------
    Class MemoryPool :
        This class represents a memory pool that allows us to allocate
        dynamic memory at the beginning of the application in order to
        avoid memory fragmentation and also a large number of allocation
        and deallocation.
    -------------------------------------------------------------------
*/
template<typename T>    // TODO : Check if we need to use a template here
class MemoryPool {
    private:

        // Unit of memory
        struct Unit {
            struct Unit* pNext;         // Pointer to the next memory unit
            struct Unit* pPrevious;     // Pointer to the previous memory unit
        };

        void* pMemoryBlock;                     // Pointer to the whole memory block
        struct Unit* pAllocatedLinkedList;      // Pointer to the linked list of allocated memory units
        struct Unit* pFreeLinkedList;           // Pointer to the linked list of free memory units
        size_t memorySize;                      // Total allocated memory in the pool

    public:
        MemoryPool(uint nbObjectsToAllocate) throw(std::bad_alloc);     // Constructor
        ~MemoryPool();                                                  // Destructor

        void* allocateObject();                     // Return a pointer to an memory allocated location to store a new object
        void freeObject(void* pObjectToFree);       // Tell the pool that an object doesn't need to be store in the pool anymore
};


}

#endif

