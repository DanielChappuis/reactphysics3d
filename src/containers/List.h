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

        /// Buffer for the list elements
        void* mBuffer;

        /// Number of elements in the list
        size_t mSize;

        /// Number of allocated elements in the list
        size_t mCapacity;

        /// Memory allocator
        Allocator& mAllocator;

        // -------------------- Methods -------------------- //


    public:

        // -------------------- Methods -------------------- //

        /// Constructor
        List(Allocator& allocator, size_t capacity = 0)
            : mBuffer(nullptr), mSize(0), mCapacity(0), mAllocator(allocator) {

            if (capacity > 0) {

                // Allocate memory
                reserve(capacity);
            }
        }

        /// Copy constructor
        List(const List<T>& list) : mBuffer(nullptr), mSize(0), mCapacity(0), mAllocator(list.mAllocator) {

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
                mAllocator.release(mBuffer, mCapacity * sizeof(T));
            }
        }

        /// Allocate memory for a given number of elements
        void reserve(size_t capacity) {

            if (capacity <= mCapacity) return;

            // Allocate memory for the new array
            void* newMemory = mAllocator.allocate(capacity * sizeof(T));

            if (mBuffer != nullptr) {

                // Copy the elements to the new allocated memory location
                std::memcpy(newMemory, mBuffer, mSize * sizeof(T));

                // Release the previously allocated memory
                mAllocator.release(mBuffer, mCapacity * sizeof(T));
            }

            mBuffer = newMemory;
            assert(mBuffer != nullptr);

            mCapacity = capacity;
        }

        /// Add an element into the list
        void add(const T& element) {

            // If we need to allocate more memory
            if (mSize == mCapacity) {
                reserve(mCapacity == 0 ? 1 : mCapacity * 2);
            }

            // Use the copy-constructor to construct the element
            new (static_cast<char*>(mBuffer) + mSize * sizeof(T)) T(element);

            mSize++;
        }

        /// Remove an element from the list at a given index
        void remove(uint index) {

          assert(index >= 0 && index < mSize);

          // Call the destructor
          (static_cast<T*>(mBuffer)[index]).~T();

          mSize--;

          if (index != mSize) {

              // Move the elements to fill in the empty slot
              char* dest = static_cast<char*>(mBuffer) + index * sizeof(T);
              char* src = dest + sizeof(T);
              std::memcpy(static_cast<void*>(dest), static_cast<void*>(src), (mSize - index) * sizeof(T));
          }
        }

        /// Append another list to the current one
        void addRange(const List<T>& list) {

            // If we need to allocate more memory
            if (mSize + list.size() > mCapacity) {

                // Allocate memory
                reserve(mSize + list.size());
            }

            // Add the elements of the list to the current one
            for(uint i=0; i<list.size(); i++) {

                new (static_cast<char*>(mBuffer) + mSize * sizeof(T)) T(list[i]);
                mSize++;
            }
        }

        /// Clear the list
        void clear() {

            // Call the destructor of each element
            for (uint i=0; i < mSize; i++) {
                (static_cast<T*>(mBuffer)[i]).~T();
            }

            mSize = 0;
        }

        /// Return the number of elments in the list
        size_t size() const {
            return mSize;
        }

        /// Return the capacity of the list
        size_t capacity() const {
            return mCapacity;
        }

        /// Overloaded index operator
        T& operator[](const uint index) {
           assert(index >= 0 && index < mSize);
           return (static_cast<T*>(mBuffer)[index]);
        }

        /// Overloaded const index operator
        const T& operator[](const uint index) const {
           assert(index >= 0 && index < mSize);
           return (static_cast<T*>(mBuffer)[index]);
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
