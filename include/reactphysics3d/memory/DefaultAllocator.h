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

#ifndef REACTPHYSICS3D_DEFAULT_ALLOCATOR_H
#define REACTPHYSICS3D_DEFAULT_ALLOCATOR_H

// Libraries
#include <reactphysics3d/memory/MemoryAllocator.h>
#include <reactphysics3d/configuration.h>
#include <cstdlib>
#include <cassert>
#include <iostream>
#include <stdlib.h>

/// ReactPhysics3D namespace
namespace reactphysics3d {

// Class DefaultAllocator
/**
 * This class represents a default memory allocator that uses standard C++ functions
 * to allocated 16-bytes aligned memory.
 *
 */
class DefaultAllocator : public MemoryAllocator {

    public:

        /// Destructor
        virtual ~DefaultAllocator() override = default;

        /// Assignment operator
        DefaultAllocator& operator=(DefaultAllocator& allocator) = default;

        /// Allocate memory of a given size (in bytes) and return a pointer to the
        /// allocated memory. The returned allocated memory must be 16 bytes aligned.
        virtual void* allocate(size_t size) override {

            assert(size % GLOBAL_ALIGNMENT == 0);

// If compiler is Visual Studio
#ifdef RP3D_PLATFORM_WINDOWS

                // Visual Studio doesn't not support standard std:aligned_alloc() method from C++ 17
                return _aligned_malloc(size, GLOBAL_ALIGNMENT);
#else

                // Return 16-bytes aligned memory
                void* address = nullptr;
                posix_memalign(&address, GLOBAL_ALIGNMENT, size);
                return address;
#endif
        }

        /// Release previously allocated memory.
        virtual void release(void* pointer, size_t /*size*/) override {

            // If compiler is Visual Studio
#ifdef RP3D_COMPILER_VISUAL_STUDIO

                // Visual Studio doesn't not support standard std:aligned_alloc() method from c++ 17
                return _aligned_free(pointer);
#else

                return std::free(pointer);
#endif
        }
};

}

#endif
