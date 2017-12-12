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

#ifndef REACTPHYSICS3D_LIST_H
#define REACTPHYSICS3D_LIST_H

// Libraries
#include "configuration.h"
#include "memory/Allocator.h"
#include <cstring>

namespace reactphysics3d {

// Class List
/**
 * This class represents a simple generic list with custom memory allocator.
  */
template<typename T>
class List {

    private:

        // -------------------- Attributes -------------------- //

        /// Pointer to the first element of the list
        T* mElements;

        /// Number of elements in the list
        uint mSize;

        /// Number of allocated elements in the list
        uint mCapacity;

        /// Memory allocator
        Allocator& mAllocator;

        // -------------------- Methods -------------------- //

        /// Allocate more memory for the elements of the list
        void allocateMemory(uint nbElementsToAllocate) {

            assert(nbElementsToAllocate > mCapacity);

            // Allocate memory for the new array
            void* newMemory = mAllocator.allocate(nbElementsToAllocate * sizeof(T));

            if (mElements != nullptr) {

                // Copy the elements to the new allocated memory location
                std::memcpy(newMemory, static_cast<void*>(mElements), mSize * sizeof(T));

                // Release the previously allocated memory
                mAllocator.release(mElements, mCapacity * sizeof(T));
            }

            mElements = static_cast<T*>(newMemory);

            mCapacity = nbElementsToAllocate;
        }

    public:

        // -------------------- Methods -------------------- //

        /// Constructor
        List(Allocator& allocator, uint capacity = 0)
            : mElements(nullptr), mSize(0), mCapacity(0), mAllocator(allocator) {

            if (capacity > 0) {

                // Allocate memory
                allocateMemory(capacity);
            }
        }

        /// Copy constructor
        List(const List<T>& list) : mElements(nullptr), mSize(0), mCapacity(0), mAllocator(list.mAllocator) {

            // All all the elements of the list to the current one
            addRange(list);
        }

        /// Destructor
        ~List() {

            // If elements have been allocated
            if (mCapacity > 0) {

                // Clear the list
                clear();

                // Release the memory allocated on the heap
                mAllocator.release(static_cast<void*>(mElements), mCapacity * sizeof(T));
            }
        }

        /// Add an element into the list
        void add(const T& element) {

            // If we need to allocate more memory
            if (mSize == mCapacity) {
                allocateMemory(mCapacity == 0 ? 1 : mCapacity * 2);
            }

            mElements[mSize] = element;
            mSize++;
        }

        /// Append another list to the current one
        void addRange(const List<T>& list) {

            // If we need to allocate more memory
            if (mSize + list.size() > mCapacity) {

                // Allocate memory
                allocateMemory(mSize + list.size());
            }

            // Add the elements of the list to the current one
            for(uint i=0; i<list.size(); i++) {
                mElements[mSize] = list[i];
                mSize++;
            }
        }

        /// Clear the list
        void clear() {

            // Call the destructor of each element
            for (uint i=0; i < mSize; i++) {
                mElements[i].~T();
            }

            mSize = 0;
        }

        /// Return the number of elments in the list
        uint size() const {
            return mSize;
        }

        /// Overloaded index operator
        T& operator[](const uint index) {
           assert(index >= 0 && index < mSize);
           return mElements[index];
        }

        /// Overloaded const index operator
        const T& operator[](const uint index) const {
           assert(index >= 0 && index < mSize);
           return mElements[index];
        }

        /// Overloaded assignment operator
        List<T>& operator=(const List<T>& list) {

            // Clear all the elements
            clear();

            // Add all the elements of the list to the current one
            addRange(list);

            return *this;
        }
};

}

#endif
