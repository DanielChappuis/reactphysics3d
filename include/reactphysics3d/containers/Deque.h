/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2020 Daniel Chappuis                                       *
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

        // -------------------- Constants -------------------- //

        /// Number of items in a chunk
        const uint CHUNK_NB_ITEMS = 17;

        /// First item index in a chunk
        const uint CHUNK_FIRST_ITEM_INDEX = CHUNK_NB_ITEMS / 2;

        // -------------------- Attributes -------------------- //

        /// Array of chunks
        T** mChunks;

        /// Number of current elements in the deque
        size_t mSize;

        /// Number of chunks
        size_t mNbChunks;

        /// Index of the chunk with the first element of the deque
        size_t mFirstChunkIndex;

        /// Index of the chunk with the last element of the deque
        size_t mLastChunkIndex;

        /// Index of the first element in the first chunk
        uint8 mFirstItemIndex;

        /// Index of the last element in the last chunk
        uint8 mLastItemIndex;

        /// Memory allocator
        MemoryAllocator& mAllocator;

        // -------------------- Methods -------------------- //

        /// Return a reference to an item at the given virtual index in range [0; mSize-1]
        T& getItem(size_t virtualIndex) const {

            // If the virtual index is valid
            if (virtualIndex < mSize) {

                size_t chunkIndex = mFirstChunkIndex;
                size_t itemIndex = mFirstItemIndex;

                const size_t nbItemsFirstChunk = CHUNK_NB_ITEMS - mFirstItemIndex;
                if (virtualIndex < nbItemsFirstChunk) {
                   itemIndex += virtualIndex;
                }
                else {

                    virtualIndex -= nbItemsFirstChunk;
                    chunkIndex++;

                    chunkIndex += virtualIndex / CHUNK_NB_ITEMS;
                    itemIndex = virtualIndex % CHUNK_NB_ITEMS;
                }

                return mChunks[chunkIndex][itemIndex];
            }
            else {
                assert(false);
            }
        }

        /// Add more chunks
        void expandChunks(size_t atLeastNbChunks = 0) {

            // If it is not necessary to expand the chunks
            if (atLeastNbChunks > 0 && atLeastNbChunks <= mNbChunks) {
                return;
            }

            size_t newNbChunks = mNbChunks == 0 ? 3 : 2 * mNbChunks - 1;
            if (atLeastNbChunks > 0 && newNbChunks < atLeastNbChunks) {
                newNbChunks = size_t(atLeastNbChunks / 2) * 2 + 1;
            }
            const size_t halfNbChunksToAdd = mNbChunks == 0 ? 1 : (mNbChunks - 1) / 2;

            // Allocate memory for the new array of pointers to chunk
            void* newMemory = mAllocator.allocate(newNbChunks * sizeof(T*));
            assert(newMemory != nullptr);
            T** newChunks = static_cast<T**>(newMemory);

            // If chunks have already been allocated
            if (mNbChunks > 0) {

                // Copy the pointers to the previous chunks to the new allocated memory location
                std::uninitialized_copy(mChunks, mChunks + mNbChunks, newChunks + halfNbChunksToAdd);

                // Release the previously allocated memory
                mAllocator.release(mChunks, mNbChunks * sizeof(T*));
            }

            mChunks = newChunks;

            // If we need to allocate the first chunk (in the middle of the chunks array)
            if (mNbChunks == 0) {
                mChunks[newNbChunks / 2] = static_cast<T*>(mAllocator.allocate(sizeof(T) * CHUNK_NB_ITEMS));
                assert(mChunks[newNbChunks / 2] != nullptr);
            }

            mNbChunks = newNbChunks;

            // Allocate memory for each new chunk
            for (size_t i=0; i < halfNbChunksToAdd; i++) {

                // Allocate memory for the new chunk
                mChunks[i] = static_cast<T*>(mAllocator.allocate(sizeof(T) * CHUNK_NB_ITEMS));
                assert(mChunks[i] != nullptr);

                mChunks[mNbChunks - 1 - i] = static_cast<T*>(mAllocator.allocate(sizeof(T) * CHUNK_NB_ITEMS));
                assert(mChunks[mNbChunks - 1 -i] != nullptr);
            }

            // Update the first and last chunk index
            mFirstChunkIndex += halfNbChunksToAdd;
            mLastChunkIndex += halfNbChunksToAdd;
        }

    public:

        /// Class Iterator
        /**
         * This class represents an iterator for the Deque
         */
        class Iterator {

            private:

                size_t mVirtualIndex;
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
                Iterator() = default;

                /// Constructor
                Iterator(const Deque<T>* deque, size_t virtualIndex) : mVirtualIndex(virtualIndex), mDeque(deque) {

                }

                /// Copy constructor
                Iterator(const Iterator& it) : mVirtualIndex(it.mVirtualIndex), mDeque(it.mDeque) {

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

                /// Post increment (it++)
                Iterator& operator++() {
                    assert(mVirtualIndex < mDeque->mSize);
                    mVirtualIndex++;
                    return *this;
                }

                /// Pre increment (++it)
                Iterator operator++(int number) {
                    assert(mVirtualIndex < mDeque->mSize);
                    Iterator tmp = *this;
                    mVirtualIndex++;
                    return tmp;
                }

                /// Post decrement (it--)
                Iterator& operator--() {
                    mVirtualIndex--;
                    return *this;
                }

                /// Pre decrement (--it)
                Iterator operator--(int number) {
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
            : mChunks(nullptr), mSize(0), mNbChunks(0), mFirstChunkIndex(1),
              mLastChunkIndex(1), mFirstItemIndex(CHUNK_FIRST_ITEM_INDEX),
              mLastItemIndex(CHUNK_FIRST_ITEM_INDEX), mAllocator(allocator) {

            // Allocate memory for the chunks array
            expandChunks();
        }

        /// Copy constructor
        Deque(const Deque<T>& deque)
            : mSize(0), mNbChunks(0), mFirstChunkIndex(1),
              mLastChunkIndex(1), mFirstItemIndex(CHUNK_FIRST_ITEM_INDEX),
              mLastItemIndex(CHUNK_FIRST_ITEM_INDEX), mAllocator(deque.mAllocator) {

            // Allocate memory for the array of chunks
            expandChunks(deque.mNbChunks);

            if (deque.mSize > 0) {

                const size_t dequeHalfSize1 = std::ceil(deque.mSize / 2.0f);
                const size_t dequeHalfSize2 = deque.mSize - dequeHalfSize1;

                // Add the items into the deque
                for(size_t i=0; i < dequeHalfSize1; i++) {
                   addFront(deque[dequeHalfSize1 - 1 - i]);
                }
                for(size_t i=0; i < dequeHalfSize2; i++) {
                   addBack(deque[dequeHalfSize1 + i]);
                }
            }
        }

        /// Destructor
        ~Deque() {

            clear();

            // Release each chunk
            for (size_t i=0; i < mNbChunks; i++) {

                mAllocator.release(mChunks[i], sizeof(T) * CHUNK_NB_ITEMS);
            }

            // Release the chunks array
            mAllocator.release(mChunks, sizeof(T*) * mNbChunks);
        }

        /// Add an element at the end of the deque
        void addBack(const T& element) {

            // If we need to add the item in a another chunk
            if (mLastItemIndex == CHUNK_NB_ITEMS - 1) {

                // If we need to add more chunks
                if (mLastChunkIndex == mNbChunks - 1) {

                    // Add more chunks
                    expandChunks();
                }

                mLastItemIndex = 0;
                mLastChunkIndex++;
            }
            else if (mSize != 0) {
                mLastItemIndex++;
            }

            // Construct the element at its location in the chunk
            new (static_cast<void*>(&(mChunks[mLastChunkIndex][mLastItemIndex]))) T(element);

            mSize++;

            assert(mFirstChunkIndex >= 0 && mLastChunkIndex < mNbChunks);
            assert(mFirstItemIndex >= 0 && mFirstItemIndex < CHUNK_NB_ITEMS);
            assert(mLastItemIndex >= 0 && mLastItemIndex < CHUNK_NB_ITEMS);
            assert(mFirstChunkIndex <= mLastChunkIndex);
        }

        /// Add an element at the front of the deque
        void addFront(const T& element) {

            // If we need to add the item in another chunk
            if (mFirstItemIndex == 0) {

                // If we need to add more chunks
                if (mFirstChunkIndex == 0) {

                    // Add more chunks
                    expandChunks();
                }

                mFirstItemIndex = CHUNK_NB_ITEMS - 1;
                mFirstChunkIndex--;
            }
            else if (mSize != 0) {
                mFirstItemIndex--;
            }

            // Construct the element at its location in the chunk
            new (static_cast<void*>(&(mChunks[mFirstChunkIndex][mFirstItemIndex]))) T(element);

            mSize++;

            assert(mFirstChunkIndex >= 0 && mLastChunkIndex < mNbChunks);
            assert(mFirstItemIndex >= 0 && mFirstItemIndex < CHUNK_NB_ITEMS);
            assert(mLastItemIndex >= 0 && mLastItemIndex < CHUNK_NB_ITEMS);
            assert(mFirstChunkIndex <= mLastChunkIndex);
        }

        /// Remove the first element of the deque
        void popFront() {

            if (mSize > 0) {

                // Call the destructor of the first element
                mChunks[mFirstChunkIndex][mFirstItemIndex].~T();

                mSize--;

                if (mSize == 0) {
                    mFirstChunkIndex = mNbChunks / 2;
                    mFirstItemIndex = CHUNK_FIRST_ITEM_INDEX;
                    mLastChunkIndex = mFirstChunkIndex;
                    mLastItemIndex = CHUNK_FIRST_ITEM_INDEX;
                }
                else if (mFirstItemIndex == CHUNK_NB_ITEMS - 1){
                    mFirstChunkIndex++;
                    mFirstItemIndex = 0;
                }
                else {
                    mFirstItemIndex++;
                }

                assert(mFirstChunkIndex >= 0 && mLastChunkIndex < mNbChunks);
                assert(mFirstItemIndex >= 0 && mFirstItemIndex < CHUNK_NB_ITEMS);
                assert(mLastItemIndex >= 0 && mLastItemIndex < CHUNK_NB_ITEMS);
                assert(mFirstChunkIndex <= mLastChunkIndex);
            }
        }

        /// Remove the last element of the deque
        void popBack() {

            if (mSize > 0) {

                // Call the destructor of the last element
                mChunks[mLastChunkIndex][mLastItemIndex].~T();

                mSize--;

                if (mSize == 0) {
                    mFirstChunkIndex = mNbChunks / 2;
                    mFirstItemIndex = CHUNK_FIRST_ITEM_INDEX;
                    mLastChunkIndex = mFirstChunkIndex;
                    mLastItemIndex = CHUNK_FIRST_ITEM_INDEX;
                }
                else if (mLastItemIndex == 0){
                    mLastChunkIndex--;
                    mLastItemIndex = CHUNK_NB_ITEMS - 1;
                }
                else {
                    mLastItemIndex--;
                }

                assert(mFirstChunkIndex >= 0 && mLastChunkIndex < mNbChunks);
                assert(mFirstItemIndex >= 0 && mFirstItemIndex < CHUNK_NB_ITEMS);
                assert(mLastItemIndex >= 0 && mLastItemIndex < CHUNK_NB_ITEMS);
                assert(mFirstChunkIndex <= mLastChunkIndex);
            }
        }

        /// Return a reference to the first item of the deque
        const T& getFront() const {
            if (mSize > 0) {
                return mChunks[mFirstChunkIndex][mFirstItemIndex];
            }
            assert(false);
        }

        /// Return a reference to the last item of the deque
        const T& getBack() const {
            if (mSize > 0) {
                return mChunks[mLastChunkIndex][mLastItemIndex];
            }
            assert(false);
        }

        /// Clear the elements of the deque
        void clear() {

            if (mSize > 0) {

                // Call the destructor of every items
                for (size_t i=0; i < mSize; i++) {
                    getItem(i).~T();
                }

                mSize = 0;

                mFirstChunkIndex = mNbChunks / 2;
                mLastChunkIndex = mFirstChunkIndex;
                mFirstItemIndex = CHUNK_FIRST_ITEM_INDEX;
                mLastItemIndex = CHUNK_FIRST_ITEM_INDEX;
            }
        }

        /// Return the number of elements in the deque
        size_t size() const {
            return mSize;
        }

        /// Overloaded index operator
        T& operator[](const uint index) {
           assert(index < mSize);
           return getItem(index);
        }

        /// Overloaded const index operator
        const T& operator[](const uint index) const {
           assert(index < mSize);
           return getItem(index);
        }

        /// Overloaded equality operator
        bool operator==(const Deque<T>& deque) const {

            if (mSize != deque.mSize) return false;

            for (size_t i=0; i < mSize; i++) {
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

                if (deque.mSize > 0) {

                    // Number of used chunks
                    const size_t nbUsedChunks = deque.mLastChunkIndex - deque.mFirstChunkIndex + 1;

                    // Expand the chunk if necessary
                    expandChunks(nbUsedChunks);

                    const size_t dequeHalfSize1 = std::ceil(deque.mSize / 2.0f);
                    const size_t dequeHalfSize2 = deque.mSize - dequeHalfSize1;

                    // Add the items into the deque
                    for(size_t i=0; i < dequeHalfSize1; i++) {
                       addFront(deque[dequeHalfSize1 - 1 - i]);
                    }
                    for(size_t i=0; i < dequeHalfSize2; i++) {
                       addBack(deque[dequeHalfSize1 + i]);
                    }
                }
            }

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
