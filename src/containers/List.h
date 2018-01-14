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
#include "memory/MemoryAllocator.h"
#include <cstring>
#include <iterator>

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
        MemoryAllocator& mAllocator;

        // -------------------- Methods -------------------- //


    public:

        /// Class Iterator
        /**
         * This class represents an iterator for the List
         */
        class Iterator {

            private:

                size_t mCurrentIndex;
                T* mBuffer;
                size_t mSize;

            public:

                // Iterator traits
                using value_type = T;
                using difference_type = std::ptrdiff_t;
                using pointer = T*;
                using reference = T&;
                using iterator_category = std::bidirectional_iterator_tag;

                /// Constructor
                Iterator() = default;

                /// Constructor
                Iterator(T* buffer, size_t index, size_t size)
                     :mCurrentIndex(index), mBuffer(buffer), mSize(size) {

                }

                /// Copy constructor
                Iterator(const Iterator& it)
                     :mCurrentIndex(it.mCurrentIndex), mBuffer(it.mBuffer), mSize(it.size) {

                }

                /// Deferencable
                reference operator*() const {
                    assert(mCurrentIndex >= 0 && mCurrentIndex < mSize);
                    return mBuffer[mCurrentIndex];
                }

                /// Deferencable
                pointer operator->() const {
                    assert(mCurrentIndex >= 0 && mCurrentIndex < mSize);
                    return &(mBuffer[mCurrentIndex]);
                }

                /// Post increment (it++)
                Iterator& operator++() {
                    assert(mCurrentIndex < mSize - 1);
                    mCurrentIndex++;
                    return *this;
                }

                /// Pre increment (++it)
                Iterator operator++(int number) {
                    assert(mCurrentIndex < mSize - 1);
                    Iterator tmp = *this;
                    mCurrentIndex++;
                    return tmp;
                }

                /// Post decrement (it--)
                Iterator& operator--() {
                    assert(mCurrentIndex > 0);
                    mCurrentIndex--;
                    return *this;
                }

                /// Pre decrement (--it)
                Iterator operator--(int number) {
                    assert(mCurrentIndex > 0);
                    Iterator tmp = *this;
                    mCurrentIndex--;
                    return tmp;
                }

                /// Equality operator (it == end())
                bool operator==(const Iterator& iterator) const {
                    assert(mCurrentIndex >= 0 && mCurrentIndex <= mSize);

                    // If both iterators points to the end of the list
                    if (mCurrentIndex == mSize && iterator.mCurrentIndex == iterator.mSize) {
                        return true;
                    }

                    return &(mBuffer[mCurrentIndex]) == &(iterator.mBuffer[mCurrentIndex]);
                }

                /// Inequality operator (it != end())
                bool operator!=(const Iterator& iterator) const {
                    return !(*this == iterator);
                }
        };

        // -------------------- Methods -------------------- //

        /// Constructor
        List(MemoryAllocator& allocator, size_t capacity = 0)
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

        /// Append another list at the end of the current one
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

        /// Return the number of elements in the list
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

        /// Overloaded equality operator
        bool operator==(const List<T>& list) const {

           if (mSize != list.mSize) return false;

           T* items = static_cast<T*>(mBuffer);
            for (int i=0; i < mSize; i++) {
                if (items[i] != list[i]) {
                    return false;
                }
            }

            return true;
        }

        /// Overloaded not equal operator
        bool operator!=(const List<T>& list) const {

            return !((*this) == list);
        }

        /// Overloaded assignment operator
        List<T>& operator=(const List<T>& list) {

            if (this != &list) {

                // Clear all the elements
                clear();

                // Add all the elements of the list to the current one
                addRange(list);
            }

            return *this;
        }
};

}

#endif
