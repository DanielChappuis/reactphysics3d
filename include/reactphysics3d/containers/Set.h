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

#ifndef REACTPHYSICS3D_SET_H
#define REACTPHYSICS3D_SET_H

// Libraries
#include <reactphysics3d/memory/MemoryAllocator.h>
#include <reactphysics3d/mathematics/mathematics_functions.h>
#include <cstring>
#include <stdexcept>
#include <functional>
#include <limits>

namespace reactphysics3d {

// Class Set
/**
 * This class represents a simple generic set. This set is implemented
 * with a hash table.
  */
template<typename V, class Hash = std::hash<V>, class KeyEqual = std::equal_to<V>>
class Set {

    private:

        // -------------------- Constants -------------------- //

        /// Default load factor
        static constexpr float DEFAULT_LOAD_FACTOR = 0.75;

        /// Invalid index in the array
        static constexpr uint64 INVALID_INDEX = -1;

        // -------------------- Attributes -------------------- //

        /// Total number of allocated entries
        uint64 mNbAllocatedEntries;

        /// Number of items in the set
        uint64 mNbEntries;

        /// Number of buckets and size of the hash table (nbEntries = loadFactor * mHashSize)
        uint64 mHashSize ;

        /// Array with all the buckets
        uint64* mBuckets;

        /// Array with all the entries (nbEntries = loadFactor * mHashSize)
        V* mEntries;

        /// For each entry, index of the next entry at the same bucket
        uint64* mNextEntries;

        /// Memory allocator
        MemoryAllocator& mAllocator;

        /// Index to the fist free entry
        uint64 mFreeIndex;

        // -------------------- Methods -------------------- //

        /// Return the index of the entry with a given value or -1 if there is no entry with this value
        uint64 findEntry(const V& value) const {

            if (mHashSize > 0) {

               const size_t hashCode = Hash()(value);
               const size_t divider = mHashSize - 1;
               const uint64 bucket = static_cast<uint64>(hashCode & divider);
               auto keyEqual = KeyEqual();

               for (uint64 i = mBuckets[bucket]; i != INVALID_INDEX; i = mNextEntries[i]) {
                   if (Hash()(mEntries[i]) == hashCode && keyEqual(mEntries[i], value)) {
                       return i;
                   }
               }
            }

            return INVALID_INDEX;
        }

    public:

        /// Class Iterator
        /**
         * This class represents an iterator for the Set
         */
        class Iterator {

            private:

                /// Pointer to the set
                const Set* mSet;

                /// Index of the current bucket
                uint64 mCurrentBucketIndex;

                /// Index of the current entry
                uint64 mCurrentEntryIndex;

                /// Advance the iterator
                void advance() {

                    assert(mCurrentBucketIndex < mSet->mHashSize);
                    assert(mCurrentEntryIndex < mSet->mNbAllocatedEntries);

                    // Try the next entry
                    if (mSet->mNextEntries[mCurrentEntryIndex] != INVALID_INDEX) {
                        mCurrentEntryIndex = mSet->mNextEntries[mCurrentEntryIndex];
                        return;
                    }

                    // Try to move to the next bucket
                    mCurrentEntryIndex = 0;
                    mCurrentBucketIndex++;
                    while(mCurrentBucketIndex < mSet->mHashSize && mSet->mBuckets[mCurrentBucketIndex] == INVALID_INDEX) {
                       mCurrentBucketIndex++;
                    }
                    if (mCurrentBucketIndex < mSet->mHashSize) {
                        mCurrentEntryIndex = mSet->mBuckets[mCurrentBucketIndex];
                    }
                }

            public:

                // Iterator traits
                using value_type = V;
                using difference_type = std::ptrdiff_t;
                using pointer = V*;
                using reference = V&;
                using iterator_category = std::forward_iterator_tag;

                /// Constructor
                Iterator() = default;

                /// Constructor
                Iterator(const Set* set, uint64 bucketIndex, uint64 entryIndex)
                     :mSet(set), mCurrentBucketIndex(bucketIndex), mCurrentEntryIndex(entryIndex) {

                }

                /// Deferencable
                reference operator*() const {
                    assert(mCurrentEntryIndex < mSet->mNbAllocatedEntries);
                    assert(mCurrentEntryIndex != INVALID_INDEX);
                    return mSet->mEntries[mCurrentEntryIndex];
                }

                /// Deferencable
                pointer operator->() const {
                    assert(mCurrentEntryIndex < mSet->mNbAllocatedEntries);
                    assert(mCurrentEntryIndex != INVALID_INDEX);
                    return &(mSet->mEntries[mCurrentEntryIndex]);
                }

                /// Pre increment (++it)
                Iterator& operator++() {
                    advance();
                    return *this;
                }

                /// Post increment (it++)
                Iterator operator++(int) {
                    Iterator tmp = *this;
                    advance();
                    return tmp;
                }

                /// Equality operator (it == end())
                bool operator==(const Iterator& iterator) const {
                    return mCurrentBucketIndex == iterator.mCurrentBucketIndex && mCurrentEntryIndex == iterator.mCurrentEntryIndex && mSet == iterator.mSet;
                }

                /// Inequality operator (it != end())
                bool operator!=(const Iterator& iterator) const {
                    return !(*this == iterator);
                }
        };


        // -------------------- Methods -------------------- //

        /// Constructor
        Set(MemoryAllocator& allocator, uint64 capacity = 0)
            : mNbAllocatedEntries(0), mNbEntries(0), mHashSize(0), mBuckets(nullptr),
              mEntries(nullptr), mNextEntries(nullptr), mAllocator(allocator), mFreeIndex(INVALID_INDEX) {

            if (capacity > 0) {

               reserve(capacity);
            }
        }

        /// Copy constructor
        Set(const Set<V, Hash, KeyEqual>& set)
          :mNbAllocatedEntries(set.mNbAllocatedEntries), mNbEntries(set.mNbEntries), mHashSize(set.mHashSize),
           mBuckets(nullptr), mEntries(nullptr), mNextEntries(nullptr), mAllocator(set.mAllocator), mFreeIndex(set.mFreeIndex) {

            if (mHashSize > 0) {

                // Allocate memory for the buckets
                mBuckets = static_cast<uint64*>(mAllocator.allocate(mHashSize * sizeof(uint64)));

                // Allocate memory for the entries
                mEntries = static_cast<V*>(mAllocator.allocate(mNbAllocatedEntries * sizeof(V)));
                mNextEntries = static_cast<uint64*>(mAllocator.allocate(mNbAllocatedEntries * sizeof(uint64)));

                // Copy the buckets array
                std::memcpy(mBuckets, set.mBuckets, mHashSize * sizeof(uint64));

                // Copy the next entries indices
                std::memcpy(mNextEntries, set.mNextEntries, mNbAllocatedEntries * sizeof(uint64));

                // Copy the entries
                for (uint64 i=0; i<mHashSize; i++) {

                    uint64 entryIndex = mBuckets[i];
                    while(entryIndex != INVALID_INDEX) {

                        // Copy the entry to the new location and destroy the previous one
                        new (mEntries + entryIndex) V(set.mEntries[entryIndex]);

                        entryIndex = mNextEntries[entryIndex];
                    }
                }
            }
        }

        /// Destructor
        ~Set() {

            clear(true);
        }

        /// Allocate memory for a given number of elements
        void reserve(uint64 capacity) {

            if (capacity <= mHashSize) return;

            if (capacity < 16) capacity = 16;

            // Make sure we have a power of two size
            if (!isPowerOfTwo(capacity)) {
                capacity = nextPowerOfTwo64Bits(capacity);
            }

            assert(capacity < INVALID_INDEX);

            assert(capacity > mHashSize);

            // Allocate memory for the buckets
            uint64* newBuckets = static_cast<uint64*>(mAllocator.allocate(capacity * sizeof(uint64)));

            // Allocate memory for the entries
            uint64 nbAllocatedEntries = static_cast<uint64>(capacity * double(DEFAULT_LOAD_FACTOR));
            assert(nbAllocatedEntries > 0);

            // Make sure capacity is an integral multiple of alignment
            nbAllocatedEntries = std::ceil(nbAllocatedEntries / float(GLOBAL_ALIGNMENT)) * GLOBAL_ALIGNMENT;

            V* newEntries = static_cast<V*>(mAllocator.allocate(nbAllocatedEntries * sizeof(V)));
            uint64* newNextEntries = static_cast<uint64*>(mAllocator.allocate(nbAllocatedEntries * sizeof(uint64)));

            assert(newEntries != nullptr);
            assert(newNextEntries != nullptr);

            // Initialize the new buckets
            for (uint64 i=0; i<capacity; i++) {
                newBuckets[i] = INVALID_INDEX;
            }

            if (mNbAllocatedEntries > 0) {

                assert(mNextEntries != nullptr);

                // Copy the free nodes indices in the nextEntries array
                std::memcpy(newNextEntries, mNextEntries, mNbAllocatedEntries * sizeof(uint64));
            }

            // Recompute the buckets (hash) with the new hash size
            for (uint64 i=0; i<mHashSize; i++) {

                uint64 entryIndex = mBuckets[i];
                while(entryIndex != INVALID_INDEX) {

                    // Get the corresponding bucket
                    const size_t hashCode = Hash()(mEntries[entryIndex]);
                    const size_t divider = capacity - 1;
                    const uint64 bucketIndex = static_cast<uint64>(hashCode & divider);

                    newNextEntries[entryIndex] = newBuckets[bucketIndex];
                    newBuckets[bucketIndex] = entryIndex;

                    // Copy the entry to the new location and destroy the previous one
                    new (newEntries + entryIndex) V(mEntries[entryIndex]);
                    mEntries[entryIndex].~V();

                    entryIndex = mNextEntries[entryIndex];
                }
            }

            if (mNbAllocatedEntries > 0) {

                // Release previously allocated memory
                mAllocator.release(mBuckets, mHashSize * sizeof(uint64));
                mAllocator.release(mEntries, mNbAllocatedEntries * sizeof(V));
                mAllocator.release(mNextEntries, mNbAllocatedEntries * sizeof(uint64));
            }

            // Add the new entries to the free list
            for (uint64 i=mNbAllocatedEntries; i < nbAllocatedEntries-1; i++) {
                newNextEntries[i] = i + 1;
            }
            newNextEntries[nbAllocatedEntries - 1] = mFreeIndex;
            mFreeIndex = mNbAllocatedEntries;

            mHashSize = capacity;
            mNbAllocatedEntries = nbAllocatedEntries;
            mBuckets = newBuckets;
            mEntries = newEntries;
            mNextEntries = newNextEntries;

            assert(mFreeIndex != INVALID_INDEX);
        }

        /// Return true if the set contains a given value
        bool contains(const V& value) const {
            return findEntry(value) != INVALID_INDEX;
        }

        /// Add a value into the set.
        /// Returns true if the item has been inserted and false otherwise.
        bool add(const V& value) {

            uint64 bucket = INVALID_INDEX;

            // Compute the hash code of the value
            const size_t hashCode = Hash()(value);

            if (mHashSize > 0) {

                // Compute the corresponding bucket index
                const size_t divider = mHashSize - 1;
                bucket = static_cast<uint64>(hashCode & divider);

                auto keyEqual  = KeyEqual();

                // Check if the item is already in the set
                for (uint64 i = mBuckets[bucket]; i != INVALID_INDEX; i = mNextEntries[i]) {

                    // If there is already an item with the same value in the set
                    if (Hash()(mEntries[i]) == hashCode && keyEqual(mEntries[i], value)) {

                        return false;
                    }
                }
            }

            uint64 entryIndex;

            // If there are no more free entries to use
            if (mFreeIndex == INVALID_INDEX) {

                // Allocate more memory
                reserve(mHashSize == 0 ? 16 : mHashSize * 2);

                // Recompute the bucket index
                const size_t divider = mHashSize - 1;
                bucket = static_cast<uint64>(hashCode & divider);
            }

            assert(mNbEntries < mNbAllocatedEntries);
            assert(mFreeIndex != INVALID_INDEX);

            // Get the next free entry
            entryIndex = mFreeIndex;
            mFreeIndex = mNextEntries[entryIndex];

            mNbEntries++;

            assert(bucket != INVALID_INDEX);

            mNextEntries[entryIndex] = mBuckets[bucket];
            new (mEntries + entryIndex) V(value);
            mBuckets[bucket] = entryIndex;

            return true;
        }

        /// Remove the element pointed by some iterator
        /// This method returns an iterator pointing to the
        /// element after the one that has been removed
        Iterator remove(const Iterator& it) {

            return remove(*it);
        }

        /// Remove the element from the set with a given value
        /// This method returns an iterator pointing to the
        /// element after the one that has been removed
        Iterator remove(const V& value) {

            if (mHashSize > 0) {

                const size_t hashcode = Hash()(value);
                auto keyEqual = KeyEqual();
                const size_t divider = mHashSize - 1;
                const size_t bucket = hashcode & divider;
                uint64 last = INVALID_INDEX;
                for (uint64 i = mBuckets[bucket]; i != INVALID_INDEX; last = i, i = mNextEntries[i]) {

                    // If we have found the item
                    if (Hash()(mEntries[i]) == hashcode && keyEqual(mEntries[i], value)) {

                        if (last == INVALID_INDEX) {
                           mBuckets[bucket] = mNextEntries[i];
                        }
                        else {
                           mNextEntries[last] = mNextEntries[i];
                        }

                        uint64 nextEntryIndex = mNextEntries[i];
                        uint64 nextBucketIndex = bucket;

                        mEntries[i].~V();
                        mNextEntries[i] = mFreeIndex;
                        mFreeIndex = i;
                        mNbEntries--;

                        // Find the next entry to return an iterator
                        if (nextEntryIndex == INVALID_INDEX) {
                            nextEntryIndex = 0;
                            nextBucketIndex++;
                            while(nextBucketIndex < mHashSize && mBuckets[nextBucketIndex] == INVALID_INDEX) {
                               nextBucketIndex++;
                            }
                            if (nextBucketIndex < mHashSize) {
                                nextEntryIndex = mBuckets[nextBucketIndex];
                            }
                        }

                        // We have found the next non empty entry
                        return Iterator(this, nextBucketIndex, nextEntryIndex);
                    }
                }
            }

            return end();
        }

        /// Return an array with all the values of the set
        Array<V> toArray(MemoryAllocator& arrayAllocator) const {

            Array<V> array(arrayAllocator, size());

            for (auto it = begin(); it != end(); ++it) {
                array.add(*it);
            }

           return array;
        }

        /// Clear the set
        void clear(bool releaseMemory = false) {

            for (uint64 i=0; i<mHashSize; i++) {

                uint64 entryIndex = mBuckets[i];
                while(entryIndex != INVALID_INDEX) {

                    // Destroy the entry
                    mEntries[entryIndex].~V();

                    uint64 nextEntryIndex = mNextEntries[entryIndex];

                    // Add entry to the free list
                    mNextEntries[entryIndex] = mFreeIndex;
                    mFreeIndex = entryIndex;

                    entryIndex = nextEntryIndex;
                }

                mBuckets[i] = INVALID_INDEX;
            }

            if (releaseMemory && mNbAllocatedEntries > 0) {

                // Release previously allocated memory
                mAllocator.release(mBuckets, mHashSize * sizeof(uint64));
                mAllocator.release(mEntries, mNbAllocatedEntries * sizeof(V));
                mAllocator.release(mNextEntries, mNbAllocatedEntries * sizeof(uint64));

                mBuckets = nullptr;
                mEntries = nullptr;
                mNextEntries = nullptr;

                mNbAllocatedEntries = 0;
                mHashSize = 0;
            }

            mNbEntries = 0;
        }

        /// Return the number of elements in the set
        uint64 size() const {
            return mNbEntries;
        }

        /// Return the capacity of the set
        uint64 capacity() const {
            return mHashSize;
        }

        /// Try to find an item of the set given a key.
        /// The method returns an iterator to the found item or
        /// an iterator pointing to the end if not found
        Iterator find(const V& value) const {

            uint64 bucket;
            uint64 entry = INVALID_INDEX;

            if (mHashSize > 0) {

               const size_t hashCode = Hash()(value);
               const size_t divider = mHashSize - 1;
               bucket = static_cast<uint64>(hashCode & divider);
               auto keyEqual = KeyEqual();

               for (uint64 i = mBuckets[bucket]; i != INVALID_INDEX; i = mNextEntries[i]) {
                   if (Hash()(mEntries[i]) == hashCode && keyEqual(mEntries[i], value)) {
                       entry = i;
                       break;
                   }
               }
            }

            if (entry == INVALID_INDEX) {
                return end();
            }

            return Iterator(this, bucket, entry);
        }

        /// Overloaded equality operator
        bool operator==(const Set<V>& set) const {

            if (size() != set.size()) return false;

            for (auto it = begin(); it != end(); ++it) {
                if(!set.contains(*it)) {
                    return false;
                }
            }

            return true;
        }

        /// Overloaded not equal operator
        bool operator!=(const Set<V>& set) const {

            return !((*this) == set);
        }

        /// Overloaded assignment operator
        Set<V>& operator=(const Set<V>& set) {

            // Check for self assignment
            if (this != &set) {

                // Clear the set
                clear(true);

                mNbAllocatedEntries = set.mNbAllocatedEntries;
                mNbEntries = set.mNbEntries;
                mHashSize = set.mHashSize;
                mFreeIndex = set.mFreeIndex;

                if (mHashSize > 0) {

                    // Allocate memory for the buckets
                    mBuckets = static_cast<uint64*>(mAllocator.allocate(mHashSize * sizeof(uint64)));

                    // Allocate memory for the entries
                    mEntries = static_cast<V*>(mAllocator.allocate(mNbAllocatedEntries * sizeof(V)));
                    mNextEntries = static_cast<uint64*>(mAllocator.allocate(mNbAllocatedEntries * sizeof(uint64)));

                    // Copy the buckets array
                    std::memcpy(mBuckets, set.mBuckets, mHashSize * sizeof(uint64));

                    // Copy the next entries indices
                    std::memcpy(mNextEntries, set.mNextEntries, mNbAllocatedEntries * sizeof(uint64));

                    // Copy the entries
                    for (uint64 i=0; i<mHashSize; i++) {

                        uint64 entryIndex = mBuckets[i];
                        while(entryIndex != INVALID_INDEX) {

                            // Copy the entry to the new location and destroy the previous one
                            new (mEntries + entryIndex) V(set.mEntries[entryIndex]);

                            entryIndex = mNextEntries[entryIndex];
                        }
                    }
                }
            }

            return *this;
        }

        /// Return a begin iterator
        Iterator begin() const {

            // If the set is empty
            if (size() == 0) {

                // Return an iterator to the end
                return end();
            }

            // Find the first used entry
            uint64 bucketIndex = 0;
            while (mBuckets[bucketIndex] == INVALID_INDEX) {

                bucketIndex++;
            }

            assert(bucketIndex < mHashSize);
            assert(mBuckets[bucketIndex] != INVALID_INDEX);

            return Iterator(this, bucketIndex, mBuckets[bucketIndex]);
        }

        /// Return a end iterator
        Iterator end() const {

            return Iterator(this, mHashSize, 0);
        }

        // ---------- Friendship ---------- //

        friend class Iterator;
};

}

#endif
