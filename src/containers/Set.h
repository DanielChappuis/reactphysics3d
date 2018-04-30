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

#ifndef REACTPHYSICS3D_SET_H
#define REACTPHYSICS3D_SET_H

// Libraries
#include "memory/MemoryAllocator.h"
#include "mathematics/mathematics_functions.h"
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
template<typename V>
class Set {

    private:

        /// An entry of the set
        struct Entry {

            size_t hashCode;		// Hash code of the entry
            int next;				// Index of the next entry
            V* value;				// Pointer to the value stored in the entry

            /// Constructor
            Entry() {
                next = -1;
                value = nullptr;
            }

            /// Constructor
            Entry(size_t hashcode, int nextEntry) {
                hashCode = hashcode;
                next = nextEntry;
                value = nullptr;
            }

            /// Copy-constructor
            Entry(const Entry& entry) {
                hashCode = entry.hashCode;
                next = entry.next;
                value = entry.value;
            }

            /// Destructor
            ~Entry() {

            }

        };

        // -------------------- Constants -------------------- //

        /// Number of prime numbers in array
        static constexpr int NB_PRIMES = 70;

        /// Array of prime numbers for the size of the set
        static const int PRIMES[NB_PRIMES];

        /// Largest prime number
        static int LARGEST_PRIME;

        // -------------------- Attributes -------------------- //

        /// Current number of used entries in the set
        int mNbUsedEntries;

        /// Number of free entries among the used ones
        int mNbFreeEntries;

        /// Current capacity of the set
        int mCapacity;

        /// Array with all the buckets
        int* mBuckets;

        /// Array with all the entries
        Entry* mEntries;

        /// Memory allocator
        MemoryAllocator& mAllocator;

        /// Index to the fist free entry
        int mFreeIndex;

        // -------------------- Methods -------------------- //

        /// Initialize the set
        void initialize(int capacity) {

            // Compute the next larger prime size
            mCapacity = getPrimeSize(capacity);

            // Allocate memory for the buckets
            mBuckets = static_cast<int*>(mAllocator.allocate(mCapacity * sizeof(int)));

            // Allocate memory for the entries
            mEntries = static_cast<Entry*>(mAllocator.allocate(mCapacity * sizeof(Entry)));

            // Initialize the buckets and entries
            for (int i=0; i<mCapacity; i++) {

                mBuckets[i] = -1;

                // Construct the entry
                new (&mEntries[i]) Entry();
            }

            mNbUsedEntries = 0;
            mNbFreeEntries = 0;
            mFreeIndex = -1;
        }

        /// Expand the capacity of the set
        void expand(int newCapacity) {

            assert(newCapacity > mCapacity);
            assert(isPrimeNumber(newCapacity));

            // Allocate memory for the buckets
            int* newBuckets = static_cast<int*>(mAllocator.allocate(newCapacity * sizeof(int)));

            // Allocate memory for the entries
            Entry* newEntries = static_cast<Entry*>(mAllocator.allocate(newCapacity * sizeof(Entry)));

            // Initialize the new buckets
            for (int i=0; i<newCapacity; i++) {
                newBuckets[i] = -1;
            }

            if (mNbUsedEntries > 0) {

                // Copy the old entries to the new allocated memory location
                std::uninitialized_copy(mEntries, mEntries + mNbUsedEntries, newEntries); 

                // Destruct the old entries at previous location
                for (int i=0; i<mNbUsedEntries; i++) {
                    mEntries[i].~Entry();
                }
            }

            // Construct the new entries
            for (int i=mNbUsedEntries; i<newCapacity; i++) {

                // Construct the entry
                new (static_cast<void*>(&newEntries[i])) Entry();
            }

            // For each used entry
            for (int i=0; i<mNbUsedEntries; i++) {

                // If the entry is not free
                if (newEntries[i].value != nullptr) {

                    // Get the corresponding bucket
                    int bucket = newEntries[i].hashCode % newCapacity;

                    newEntries[i].next = newBuckets[bucket];
                    newBuckets[bucket] = i;
                }
            }

            // Release previously allocated memory
            mAllocator.release(mBuckets, mCapacity * sizeof(int));
            mAllocator.release(mEntries, mCapacity * sizeof(Entry));

            mCapacity = newCapacity;
            mBuckets = newBuckets;
            mEntries = newEntries;
        }

        /// Return the index of the entry with a given value or -1 if there is no entry with this value
        int findEntry(const V& value) const {

            if (mCapacity > 0) {

               size_t hashCode = std::hash<V>()(value);
               int bucket = hashCode % mCapacity;

               for (int i = mBuckets[bucket]; i >= 0; i = mEntries[i].next) {
                   if (mEntries[i].hashCode == hashCode && (*mEntries[i].value) == value) {
                       return i;
                   }
               }
            }

            return -1;
        }

        /// Return the prime number that is larger or equal to the number in parameter
        /// for the size of the set
        static int getPrimeSize(int number) {

            // Check if the next larger prime number is in the precomputed array of primes
            for (int i = 0; i < NB_PRIMES; i++) {
                if (PRIMES[i] >= number) return PRIMES[i];
            }

            // Manually compute the next larger prime number
            for (int i = (number | 1); i < std::numeric_limits<int>::max(); i+=2) {

                if (isPrimeNumber(i)) {
                    return i;
                }
            }

            return number;
        }

        /// Clear and reset the set
        void reset() {

            // If elements have been allocated
            if (mCapacity > 0) {

                // Clear the list
                clear();

                // Destroy the entries
                for (int i=0; i < mCapacity; i++) {
                    mEntries[i].~Entry();
                }

                mAllocator.release(mBuckets, mCapacity * sizeof(int));
                mAllocator.release(mEntries, mCapacity * sizeof(Entry));

                mNbUsedEntries = 0;
                mNbFreeEntries = 0;
                mCapacity = 0;
                mBuckets = nullptr;
                mEntries = nullptr;
                mFreeIndex = -1;
            }
        }

    public:

        /// Class Iterator
        /**
         * This class represents an iterator for the Set
         */
        class Iterator {

            private:

                /// Array of entries
                const Entry* mEntries;

                /// Capacity of the map
                int mCapacity;

                /// Number of used entries in the map
                int mNbUsedEntries;

                /// Index of the current entry
                int mCurrentEntry;

                /// Advance the iterator
                void advance() {

                    // If we are trying to move past the end
                    assert(mCurrentEntry < mNbUsedEntries);

                    for (mCurrentEntry += 1; mCurrentEntry < mNbUsedEntries; mCurrentEntry++) {

                        // If the entry is not empty
                        if (mEntries[mCurrentEntry].value != nullptr) {

                           // We have found the next non empty entry
                           return;
                        }
                    }

                    // We have not find a non empty entry, we return an iterator to the end
                    mCurrentEntry = mCapacity;
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
                Iterator(const Entry* entries, int capacity, int nbUsedEntries, int currentEntry)
                     :mEntries(entries), mCapacity(capacity), mNbUsedEntries(nbUsedEntries), mCurrentEntry(currentEntry) {

                }

                /// Copy constructor
                Iterator(const Iterator& it)
                     :mEntries(it.mEntries), mCapacity(it.mCapacity), mNbUsedEntries(it.mNbUsedEntries), mCurrentEntry(it.mCurrentEntry) {

                }

                /// Deferencable
                reference operator*() const {
                    assert(mCurrentEntry >= 0 && mCurrentEntry < mNbUsedEntries);
                    assert(mEntries[mCurrentEntry].value != nullptr);
                    return *(mEntries[mCurrentEntry].value);
                }

                /// Deferencable
                pointer operator->() const {
                    assert(mCurrentEntry >= 0 && mCurrentEntry < mNbUsedEntries);
                    assert(mEntries[mCurrentEntry].value != nullptr);
                    return mEntries[mCurrentEntry].value;
                }

                /// Post increment (it++)
                Iterator& operator++() {
                    advance();
                    return *this;
                }

                /// Pre increment (++it)
                Iterator operator++(int number) {
                    Iterator tmp = *this;
                    advance();
                    return tmp;
                }

                /// Equality operator (it == end())
                bool operator==(const Iterator& iterator) const {
                    return mCurrentEntry == iterator.mCurrentEntry && mEntries == iterator.mEntries;
                }

                /// Inequality operator (it != end())
                bool operator!=(const Iterator& iterator) const {
                    return !(*this == iterator);
                }
        };


        // -------------------- Methods -------------------- //

        /// Constructor
        Set(MemoryAllocator& allocator, size_t capacity = 0)
            : mNbUsedEntries(0), mNbFreeEntries(0), mCapacity(0), mBuckets(nullptr),
              mEntries(nullptr), mAllocator(allocator), mFreeIndex(-1) {

            // If the largest prime has not been computed yet
            if (LARGEST_PRIME == -1) {

                // Compute the largest prime number (largest map capacity)
                LARGEST_PRIME = getPrimeSize(PRIMES[NB_PRIMES - 1] + 2);
            }

            if (capacity > 0) {

               initialize(capacity);
            }
        }

        /// Copy constructor
        Set(const Set<V>& set)
          :mNbUsedEntries(set.mNbUsedEntries), mNbFreeEntries(set.mNbFreeEntries), mCapacity(set.mCapacity),
           mBuckets(nullptr), mEntries(nullptr), mAllocator(set.mAllocator), mFreeIndex(set.mFreeIndex) {

            if (mCapacity > 0) {

                // Allocate memory for the buckets
                mBuckets = static_cast<int*>(mAllocator.allocate(mCapacity * sizeof(int)));

                // Allocate memory for the entries
                mEntries = static_cast<Entry*>(mAllocator.allocate(mCapacity * sizeof(Entry)));

                // Copy the buckets
                std::uninitialized_copy(set.mBuckets, set.mBuckets + mCapacity, mBuckets);

                // Copy the entries
                for (int i=0; i < mCapacity; i++) {

                    new (&mEntries[i]) Entry(set.mEntries[i].hashCode, set.mEntries[i].next);

                    if (set.mEntries[i].value != nullptr) {
                       mEntries[i].value = static_cast<V*>(mAllocator.allocate(sizeof(V)));
                       new (mEntries[i].value) V(*(set.mEntries[i].value));
                    }
                }
            }
        }

        /// Destructor
        ~Set() {

            reset();
        }

        /// Allocate memory for a given number of elements
        void reserve(int capacity) {

           if (capacity <= mCapacity) return;

           if (capacity > LARGEST_PRIME && LARGEST_PRIME > mCapacity) {
               capacity = LARGEST_PRIME;
           }
           else {
               capacity = getPrimeSize(capacity);
           }

           expand(capacity);
        }

        /// Return true if the set contains a given value
        bool contains(const V& value) const {
            return findEntry(value) != -1;
        }

        /// Add a value into the set
        void add(const V& value) {

            if (mCapacity == 0) {
                initialize(0);
            }

            // Compute the hash code of the value
            size_t hashCode = std::hash<V>()(value);

            // Compute the corresponding bucket index
            int bucket = hashCode % mCapacity;

            // Check if the item is already in the set
            for (int i = mBuckets[bucket]; i >= 0; i = mEntries[i].next) {

                // If there is already an item with the same value in the set
                if (mEntries[i].hashCode == hashCode && (*mEntries[i].value) == value) {

                    return;
                }
            }

            size_t entryIndex;

            // If there are free entries to use
            if (mNbFreeEntries > 0) {
                assert(mFreeIndex >= 0);
                entryIndex = mFreeIndex;
                mFreeIndex = mEntries[entryIndex].next;
                mNbFreeEntries--;
            }
            else {

                // If we need to allocator more entries
                if (mNbUsedEntries == mCapacity) {

                    // Allocate more memory
                    reserve(mCapacity * 2);

                    // Recompute the bucket index
                    bucket = hashCode % mCapacity;
                }

                entryIndex = mNbUsedEntries;
                mNbUsedEntries++;
            }

            assert(mEntries[entryIndex].value == nullptr);
            mEntries[entryIndex].hashCode = hashCode;
            mEntries[entryIndex].next = mBuckets[bucket];
            mEntries[entryIndex].value = static_cast<V*>(mAllocator.allocate(sizeof(V)));
            assert(mEntries[entryIndex].value != nullptr);
            new (mEntries[entryIndex].value) V(value);
            mBuckets[bucket] = entryIndex;
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

            if (mCapacity > 0) {

                size_t hashcode = std::hash<V>()(value);
                int bucket = hashcode % mCapacity;
                int last = -1;
                for (int i = mBuckets[bucket]; i >= 0; last = i, i = mEntries[i].next) {

                    if (mEntries[i].hashCode == hashcode && (*mEntries[i].value) == value) {

                        if (last < 0 ) {
                           mBuckets[bucket] = mEntries[i].next;
                        }
                        else {
                           mEntries[last].next = mEntries[i].next;
                        }

                        // Release memory for the value if any
                        if (mEntries[i].value != nullptr) {
                            mEntries[i].value->~V();
                            mAllocator.release(mEntries[i].value, sizeof(V));
                            mEntries[i].value = nullptr;
                        }
                        assert(mEntries[i].value == nullptr);
                        mEntries[i].next = mFreeIndex;
                        mFreeIndex = i;
                        mNbFreeEntries++;

                        // Find the next valid entry to return an iterator
                        for (i += 1; i < mNbUsedEntries; i++) {

                            // If the entry is not empty
                            if (mEntries[i].value != nullptr) {

                               // We have found the next non empty entry
                               return Iterator(mEntries, mCapacity, mNbUsedEntries, i);
                            }
                        }

                        return end();
                    }
                }
            }

            return end();
        }

        /// Clear the set
        void clear() {

            if (mNbUsedEntries > 0) {

                for (int i=0; i < mCapacity; i++) {
                    mBuckets[i] = -1;
                    mEntries[i].next = -1;
                    if (mEntries[i].value != nullptr) {
                        mEntries[i].value->~V();
                        mAllocator.release(mEntries[i].value, sizeof(V));
                        mEntries[i].value = nullptr;
                    }
                }

                mFreeIndex = -1;
                mNbUsedEntries = 0;
                mNbFreeEntries = 0;
            }

            assert(size() == 0);
        }

        /// Return the number of elements in the set
        int size() const {
            return mNbUsedEntries - mNbFreeEntries;
        }

        /// Return the capacity of the set
        int capacity() const {
            return mCapacity;
        }

        /// Try to find an item of the set given a key.
        /// The method returns an iterator to the found item or
        /// an iterator pointing to the end if not found
        Iterator find(const V& value) const {

            int bucket;
            int entry = -1;

            if (mCapacity > 0) {

               size_t hashCode = std::hash<V>()(value);
               bucket = hashCode % mCapacity;

               for (int i = mBuckets[bucket]; i >= 0; i = mEntries[i].next) {
                   if (mEntries[i].hashCode == hashCode && *(mEntries[i].value) == value) {
                       entry = i;
                       break;
                   }
               }
            }

            if (entry == -1) {
                return end();
            }

            assert(mEntries[entry].value != nullptr);

            return Iterator(mEntries, mCapacity, mNbUsedEntries, entry);
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

                // Reset the set
                reset();

                if (set.mCapacity > 0) {

                    // Compute the next larger prime size
                    mCapacity = getPrimeSize(set.mCapacity);

                    // Allocate memory for the buckets
                    mBuckets = static_cast<int*>(mAllocator.allocate(mCapacity * sizeof(int)));

                    // Allocate memory for the entries
                    mEntries = static_cast<Entry*>(mAllocator.allocate(mCapacity * sizeof(Entry)));

                    // Copy the buckets
                    std::uninitialized_copy(set.mBuckets, set.mBuckets + mCapacity, mBuckets);

                    // Copy the entries
                    for (int i=0; i < mCapacity; i++) {

                        new (&mEntries[i]) Entry(set.mEntries[i].hashCode, set.mEntries[i].next);

                        if (set.mEntries[i].value != nullptr) {
                           mEntries[i].value = static_cast<V*>(mAllocator.allocate(sizeof(V)));
                           new (mEntries[i].value) V(*(set.mEntries[i].value));
                        }
                    }

                    mNbUsedEntries = set.mNbUsedEntries;
                    mNbFreeEntries = set.mNbFreeEntries;
                    mFreeIndex = set.mFreeIndex;
                }
            }

            return *this;
        }

        /// Return a begin iterator
        Iterator begin() const {

            // If the map is empty
            if (size() == 0) {

                // Return an iterator to the end
                return end();
            }

            // Find the first used entry
            int entry;
            for (entry=0; entry < mNbUsedEntries; entry++) {
                if (mEntries[entry].value != nullptr) {
                    return Iterator(mEntries, mCapacity, mNbUsedEntries, entry);
                }
            }

            assert(false);
            return end();
        }

        /// Return a end iterator
        Iterator end() const {
            return Iterator(mEntries, mCapacity, mNbUsedEntries, mCapacity);
        }
};

template<typename V>
const int Set<V>::PRIMES[NB_PRIMES] = {3, 7, 11, 17, 23, 29, 37, 47, 59, 71, 89, 107, 131, 163, 197, 239, 293, 353, 431, 521, 631, 761, 919,
                             1103, 1327, 1597, 1931, 2333, 2801, 3371, 4049, 4861, 5839, 7013, 8419, 10103, 12143, 14591,
                             17519, 21023, 25229, 30293, 36353, 43627, 52361, 62851, 75431, 90523, 108631, 130363, 156437,
                             187751, 225307, 270371, 324449, 389357, 467237, 560689, 672827, 807403, 968897, 1162687, 1395263,
                             1674319, 2009191, 2411033, 2893249, 3471899, 4166287, 4999559};

template<typename V>
int Set<V>::LARGEST_PRIME = -1;

}

#endif
