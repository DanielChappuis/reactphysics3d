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

#ifndef REACTPHYSICS3D_MEMORY_MANAGER_H
#define REACTPHYSICS3D_MEMORY_MANAGER_H

// Libraries
#include "memory/DefaultAllocator.h"
#include "memory/DefaultPoolAllocator.h"
#include "memory/MemoryAllocator.h"
#include "memory/DefaultSingleFrameAllocator.h"

/// Namespace ReactPhysics3D
namespace reactphysics3d {

// Declarations
class MemoryAllocator;

// Class MemoryManager
/**
 * The memory manager is used to store the different memory allocators that are used
 * by the library.
 */
class MemoryManager {

    private:
		
       /// Default malloc/free memory allocator
       static DefaultAllocator mDefaultAllocator;
	   
       /// Default single frame memory allocator
       static DefaultSingleFrameAllocator mDefaultSingleFrameAllocator;

       /// Default pool memory allocator
       static DefaultPoolAllocator mDefaultPoolAllocator;

       /// Pointer to the base memory allocator to use
       static MemoryAllocator* mBaseAllocator;

       /// Single frame stack allocator
       static SingleFrameAllocator* mSingleFrameAllocator;

       /// Memory pool allocator
       static MemoryAllocator* mPoolAllocator;

    public:

        /// Memory allocation types
       enum class AllocationType {
           Base, 	// Base memory allocator
           Pool,	// Memory pool allocator
           Frame,   // Single frame memory allocator
       };

       /// Constructor
       MemoryManager() = default;

       /// Destructor
       ~MemoryManager() = default;

        /// Allocate memory of a given type
        void* allocate(AllocationType allocationType, size_t size);

        /// Release previously allocated memory.
        void release(AllocationType allocationType, void* pointer, size_t size);

        /// Return the pool allocator
        MemoryAllocator& getPoolAllocator();

        /// Return the single frame stack allocator
        SingleFrameAllocator& getSingleFrameAllocator();

        /// Return the base memory allocator
        static MemoryAllocator& getBaseAllocator();
		
        /// Set the base memory allocator
        static void setBaseAllocator(MemoryAllocator* memoryAllocator);

        /// Set the single frame memory allocator
        static void setSingleFrameAllocator(SingleFrameAllocator* singleFrameAllocator);

        /// Set the pool memory allocator
        static void setPoolAllocator(MemoryAllocator* poolAllocator);

        /// Reset the single frame allocator
        void resetFrameAllocator();
};

// Allocate memory of a given type
inline void* MemoryManager::allocate(AllocationType allocationType, size_t size) {

    switch (allocationType) {
       case AllocationType::Base: return mBaseAllocator->allocate(size);
       case AllocationType::Pool: return mPoolAllocator->allocate(size);
       case AllocationType::Frame: return mSingleFrameAllocator->allocate(size);
    }

    return nullptr;
}

// Release previously allocated memory.
inline void MemoryManager::release(AllocationType allocationType, void* pointer, size_t size) {

    switch (allocationType) {
       case AllocationType::Base: mBaseAllocator->release(pointer, size); break;
       case AllocationType::Pool: mPoolAllocator->release(pointer, size); break;
       case AllocationType::Frame: mSingleFrameAllocator->release(pointer, size); break;
    }
}

// Return the pool allocator
inline MemoryAllocator& MemoryManager::getPoolAllocator() {
   return *mPoolAllocator;
}

// Return the single frame stack allocator
inline SingleFrameAllocator& MemoryManager::getSingleFrameAllocator() {
   return *mSingleFrameAllocator;
}

// Return the base memory allocator
inline MemoryAllocator& MemoryManager::getBaseAllocator() {
    return *mBaseAllocator;
}

// Set the base memory allocator
inline void MemoryManager::setBaseAllocator(MemoryAllocator* baseAllocator) {
    mBaseAllocator = baseAllocator;
}

// Set the base memory allocator
inline void MemoryManager::setSingleFrameAllocator(SingleFrameAllocator* singleFrameAllocator) {
    mSingleFrameAllocator = singleFrameAllocator;
}

// Set the pool memory allocator
inline void MemoryManager::setPoolAllocator(MemoryAllocator* poolAllocator) {
    mPoolAllocator = poolAllocator;
}

// Reset the single frame allocator
inline void MemoryManager::resetFrameAllocator() {
   mSingleFrameAllocator->reset();
}

}

#endif

