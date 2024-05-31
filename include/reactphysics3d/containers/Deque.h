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

#ifndef REACTPHYSICS3D_DEQUE_H
#define REACTPHYSICS3D_DEQUE_H

// Libraries
#include <reactphysics3d/configuration.h>
#include <reactphysics3d/memory/MemoryAllocator.h>
#include <cassert>
#include <cmath>
#include <cstring>
#include <iterator>
#include <memory>

namespace reactphysics3d {

// Class Deque
/**
 * This class represents a Deque. A Deque is a double ended queue. It is possible to
 * push and pop items at both ends of the queue. Note that in the following code, the
 * virtualIndex is the index of the items in the deque in range [0, mSize-1] where
 * 0 is always the front item and mSize-1 the back item. There is a virtual index for
 * each item that is in the deque. The allocatedIndex is the index of a allocated slot in the
 * deque in range [0, (mNbChunks * CHUNK_NB_ITEMS)-1]. A given allocatedIndex corresponds to a slot
 * in the deque that can be empty or contains an item.
 */
template<typename T>
class Deque {

    private:

        // -------------------- Attributes -------------------- //

        /// Buffer of elements
        T* mBuffer;

        /// Number of current elements in the deque
        uint64 mSize;

        /// Capacity
        uint64 mCapacity;

        /// Index in the buffer of the first item of the deque
        uint64 mFirstItemIndex;

        /// Memory allocator
        MemoryAllocator& mAllocator;

        // -------------------- Methods -------------------- //

        /// Return a reference to an item at the given virtual index in range [0; mSize-1]
        T& getItem(uint64 index) const {

            // Ensure the virtual index is valid
            assert(index < mSize);

            return mBuffer[mFirstItemIndex + index];
        }

        /// Add more chunks
        void reserve(uint64 capacity) {

            if (capacity <= mCapacity) return;

            // Make sure capacity is an integral multiple of alignment
            capacity = std::ceil(capacity / float(GLOBAL_ALIGNMENT)) * GLOBAL_ALIGNMENT;

            // Allocate memory for the new array
            void* newMemory = mAllocator.allocate(capacity * sizeof(T));
            T* destination = static_cast<T*>(newMemory);

            const uint64 newStartIndex = capacity / 2 - 1;

            if (mBuffer != nullptr) {

                if (mSize > 0) {

                    // Copy the elements to the new allocated memory location
                    std::uninitialized_copy(mBuffer + mFirstItemIndex, mBuffer + mFirstItemIndex + mSize, destination + newStartIndex);

                    // Destruct the previous items
                    for (uint64 i=0; i < mSize; i++) {
                        mBuffer[mFirstItemIndex + i].~T();
                    }
                }

                // Release the previously allocated memory
                mAllocator.release(mBuffer, mCapacity * sizeof(T));
            }

            mBuffer = destination;
            assert(mBuffer != nullptr);

            mCapacity = capacity;

            // Update the first and last chunk index
            mFirstItemIndex = newStartIndex;
        }

    public:

        /// Class Iterator
        /**
         * This class represents an iterator for the Deque
         */
        class Iterator {

            private:

                uint64 mVirtualIndex;
                const Deque<T>* mDeque;

            public:

                // Iterator traits
                using value_type = T;
                using difference_type = std::ptrdiff_t;
                using pointer = T*;
                using const_pointer = T const*;
                using reference = T&;
                using const_reference = const T&;
                using iterator_category = std::random_access_iterator_tag;

                /// Constructor
                Iterator(const Deque<T>* deque, uint64 index) : mVirtualIndex(index), mDeque(deque) {

                }

                /// Deferencable
                reference operator*() {
                    assert(mVirtualIndex < mDeque->mSize);
                    return mDeque->getItem(mVirtualIndex);
                }

                /// Const Deferencable
                const_reference operator*() const {
                    assert(mVirtualIndex < mDeque->mSize);
                    return mDeque->getItem(mVirtualIndex);
                }

                /// Deferencable
                const_pointer operator->() const {
                    assert(mVirtualIndex < mDeque->mSize);
                    return &(mDeque->getItem(mVirtualIndex));
                }

                /// Pre increment (++it)
                Iterator& operator++() {
                    assert(mVirtualIndex < mDeque->mSize);
                    mVirtualIndex++;
                    return *this;
                }

                /// Post increment (it++)
                Iterator operator++(int /*number*/) {
                    assert(mVirtualIndex < mDeque->mSize);
                    Iterator tmp = *this;
                    mVirtualIndex++;
                    return tmp;
                }

                /// Pre decrement (--it)
                Iterator& operator--() {
                    mVirtualIndex--;
                    return *this;
                }

                /// Post decrement (it--)
                Iterator operator--(int /*number*/) {
                    Iterator tmp = *this;
                    mVirtualIndex--;
                    return tmp;
                }

                /// Plus operator
                Iterator operator+(const difference_type& n) {
                    return Iterator(mDeque, mVirtualIndex + n);
                }

                /// Plus operator
                Iterator& operator+=(const difference_type& n) {
                    mVirtualIndex += n;
                    return *this;
                }

                /// Minus operator
                Iterator operator-(const difference_type& n) {
                    return Iterator(mDeque, mVirtualIndex - n);
                }

                /// Minus operator
                Iterator& operator-=(const difference_type& n) {
                    mVirtualIndex -= n;
                    return *this;
                }

                /// Difference operator
                difference_type operator-(const Iterator& iterator) const {
                   return mVirtualIndex - iterator.mVirtualIndex;
                }

                /// Comparison operator
                bool operator<(const Iterator& other) const {

                    return mVirtualIndex < other.mVirtualIndex;
                }

                /// Comparison operator
                bool operator>(const Iterator& other) const {

                    return mVirtualIndex > other.mVirtualIndex;
                }

                /// Comparison operator
                bool operator<=(const Iterator& other) const {

                    return mVirtualIndex <= other.mVirtualIndex;
                }

                /// Comparison operator
                bool operator>=(const Iterator& other) const {

                    return mVirtualIndex >= other.mVirtualIndex;
                }

                /// Equality operator (it == end())
                bool operator==(const Iterator& iterator) const {

                    return mVirtualIndex == iterator.mVirtualIndex;
                }

                /// Inequality operator (it != end())
                bool operator!=(const Iterator& iterator) const {
                    return !(*this == iterator);
                }

                /// Frienship
                friend class Deque;

        };

        // -------------------- Methods -------------------- //

        /// Constructor
        Deque(MemoryAllocator& allocator)
            : mBuffer(nullptr), mSize(0), mCapacity(0), mFirstItemIndex(0), mAllocator(allocator) {

        }

        /// Copy constructor
        Deque(const Deque<T>& deque)
            : mBuffer(nullptr), mSize(0), mCapacity(0), mFirstItemIndex(deque.mFirstItemIndex),
              mAllocator(deque.mAllocator) {

            // Allocate memory
            reserve(deque.mCapacity);

            if (deque.mSize > 0) {

                // Add the items into the deque
                for(uint64 i=0; i < deque.mSize; i++) {

                    // Construct the element at its location in the buffer
                    new (static_cast<void*>(&(mBuffer[deque.mFirstItemIndex + i]))) T(deque.mBuffer[deque.mFirstItemIndex + i]);
                }

                mSize = deque.mSize;
            }
        }

        /// Destructor
        ~Deque() {

            clear();

            if (mCapacity > 0) {

                assert(mBuffer != nullptr);

                // Release the chunks array
                mAllocator.release(mBuffer, sizeof(T) * mCapacity);
            }

            mCapacity = 0;
            mBuffer = nullptr;
        }

        /// Add an element at the end of the deque
        void addBack(const T& element) {

            // If we need to add the item in a another chunk
            if (mFirstItemIndex + mSize >= mCapacity) {

                reserve(mCapacity == 0 ? GLOBAL_ALIGNMENT : mCapacity * 2);
            }

            assert(mFirstItemIndex + mSize < mCapacity);

            // Construct the element at its location in the chunk
            new (static_cast<void*>(&(mBuffer[mFirstItemIndex + mSize]))) T(element);

            mSize++;

            assert(mFirstItemIndex + mSize <= mCapacity);
            assert(mSize <= mCapacity);
        }

        /// Add an element at the front of the deque
        void addFront(const T& element) {

            // If we need to add the item in a another chunk
            if (mFirstItemIndex == 0) {

                reserve(mCapacity == 0 ? GLOBAL_ALIGNMENT : mCapacity * 2);
            }

            assert(mFirstItemIndex > 0);

            mFirstItemIndex--;

            // Construct the element at its location in the chunk
            new (static_cast<void*>(&(mBuffer[mFirstItemIndex]))) T(element);

            mSize++;

            assert(mFirstItemIndex + mSize <= mCapacity);
            assert(mSize <= mCapacity);
        }

        /// Remove the first element of the deque
        void popFront() {

            if (mSize > 0) {

                // Call the destructor of the first element
                mBuffer[mFirstItemIndex].~T();

                mSize--;
                mFirstItemIndex++;

                if (mSize == 0) {
                    mFirstItemIndex = mCapacity / 2 - 1;
                }

                assert(mFirstItemIndex + mSize <= mCapacity);
                assert(mSize <= mCapacity);
            }
        }

        /// Remove the last element of the deque
        void popBack() {

            if (mSize > 0) {

                // Call the destructor of the last element
                mBuffer[mFirstItemIndex + mSize - 1].~T();

                mSize--;

                if (mSize == 0) {
                    mFirstItemIndex = mCapacity / 2 - 1;
                }

                assert(mFirstItemIndex + mSize <= mCapacity);
                assert(mSize <= mCapacity);
            }
        }

        /// Return a reference to the first item of the deque
        const T& getFront() const {
            assert(mSize > 0);
            return mBuffer[mFirstItemIndex];
        }

        /// Return a reference to the last item of the deque
        const T& getBack() const {
            assert(mSize > 0);
            return mBuffer[mFirstItemIndex + mSize - 1];
        }

        /// Clear the elements of the deque
        void clear() {

            if (mSize > 0) {

                // Call the destructor of every items
                for (uint64 i=0; i < mSize; i++) {
                    getItem(i).~T();
                }

                mSize = 0;
                mFirstItemIndex = mCapacity / 2 - 1;
            }
        }

        /// Return the number of elements in the deque
        uint64 size() const {
            return mSize;
        }

        /// Overloaded index operator
        T& operator[](const uint64 index) {
           assert(index < mSize);
           return getItem(index);
        }

        /// Overloaded const index operator
        const T& operator[](const uint64 index) const {
           assert(index < mSize);
           return getItem(index);
        }

        /// Overloaded equality operator
        bool operator==(const Deque<T>& deque) const {

            if (mSize != deque.mSize) return false;

            for (uint64 i=0; i < mSize; i++) {
                if (getItem(i) != deque.getItem(i)) {
                    return false;
                }
            }

            return true;
        }

        /// Overloaded not equal operator
        bool operator!=(const Deque<T>& deque) const {

            return !((*this) == deque);
        }

        /// Overloaded assignment operator
        Deque<T>& operator=(const Deque<T>& deque) {

            if (this != &deque) {

                // Clear all the elements
                clear();

                reserve(deque.mCapacity);

                if (deque.mSize > 0) {

                    // Add the items into the deque
                    for(uint64 i=0; i < deque.mSize; i++) {

                        // Construct the element at its location in the buffer
                        new (static_cast<void*>(&(mBuffer[mFirstItemIndex + i]))) T(deque.mBuffer[deque.mFirstItemIndex + i]);
                    }

                }

                mSize = deque.mSize;
            }

            assert(mFirstItemIndex + mSize <= mCapacity);
            assert(mSize <= mCapacity);

            return *this;
        }

        /// Return a begin iterator
        Iterator begin() const {
            return Iterator(this, 0);
        }

        /// Return a end iterator
        Iterator end() const {
            return Iterator(this, mSize);
        }
};

}

#endif
