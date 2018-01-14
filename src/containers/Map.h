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

#ifndef REACTPHYSICS3D_MAP_H
#define REACTPHYSICS3D_MAP_H

// Libraries
#include "memory/MemoryAllocator.h"
#include "mathematics/mathematics_functions.h"
#include <cstring>
#include <limits>

namespace reactphysics3d {

// Class Map
/**
 * This class represents a simple generic associative map
  */
template<typename K, typename V>
class Map {

    private:

        /// An entry of the map
        struct Entry {

            size_t hashCode;			// Hash code of the entry
            int next;					// Index of the next entry
            std::pair<K, V>* keyValue;	// Pointer to the pair with key and value

            /// Constructor
            Entry() {
                next = -1;
                keyValue = nullptr;
            }

            /// Destructor
            ~Entry() {

                assert(keyValue == nullptr);
            }

        };

        // -------------------- Constants -------------------- //

        /// Number of prime numbers in array
        static constexpr int NB_PRIMES = 70;

        /// Array of prime numbers for the size of the map
        static const int PRIMES[NB_PRIMES];

        /// Largest prime number
        static int LARGEST_PRIME;

        // -------------------- Attributes -------------------- //

        /// Current number of used entries in the map
        int mNbUsedEntries;

        /// Number of free entries among the used ones
        int mNbFreeEntries;

        /// Current capacity of the map
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

        /// Initialize the map
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

        /// Expand the capacity of the map
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

            // Copy the old entries to the new allocated memory location
            std::memcpy(newEntries, mEntries, mNbUsedEntries * sizeof(Entry));

            // Construct the new entries
            for (int i=mNbUsedEntries; i<newCapacity; i++) {

                // Construct the entry
                new (static_cast<void*>(&newEntries[i])) Entry();
            }

            // For each used entry
            for (int i=0; i<mNbUsedEntries; i++) {

                // If the entry is not free
                if (newEntries[i].hashCode != -1) {

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

        /// Return the index of the entry with a given key or -1 if there is no entry with this key
        int findEntry(const K& key) const {

            if (mCapacity > 0) {

               size_t hashCode = std::hash<K>()(key);
               int bucket = hashCode % mCapacity;

               for (int i = mBuckets[bucket]; i >= 0; i = mEntries[i].next) {
                   if (mEntries[i].hashCode == hashCode && mEntries[i].keyValue->first == key) {
                       return i;
                   }
               }
            }

            return -1;
        }

        /// Return the prime number that is larger or equal to the number in parameter
        /// for the size of the map
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

        /// Clear and reset the map
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

        // -------------------- Methods -------------------- //

        /// Constructor
        Map(MemoryAllocator& allocator, size_t capacity = 0)
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
        Map(const Map<K, V>& map)
          :mNbUsedEntries(map.mNbUsedEntries), mNbFreeEntries(map.mNbFreeEntries), mCapacity(map.mCapacity),
           mAllocator(map.mAllocator), mFreeIndex(map.mFreeIndex) {

            // Allocate memory for the buckets
            mBuckets = static_cast<int*>(mAllocator.allocate(mCapacity * sizeof(int)));

            // Allocate memory for the entries
            mEntries = static_cast<Entry*>(mAllocator.allocate(mCapacity * sizeof(Entry)));

            // Copy the buckets
            std::memcpy(mBuckets, map.mBuckets, mCapacity * sizeof(int));

            // Copy the entries
            std::memcpy(mEntries, map.mEntries, mCapacity * sizeof(Entry));
        }

        /// Destructor
        ~Map() {

            reset();
        }

        /// Allocate memory for a given number of elements
        void reserve(size_t capacity) {

           if (capacity <= mCapacity) return;

           if (capacity > LARGEST_PRIME && LARGEST_PRIME > mCapacity) {
               capacity = LARGEST_PRIME;
           }
           else {
               capacity = getPrimeSize(capacity);
           }

           expand(capacity);
        }

        /// Return true if the map contains an item with the given key
        bool containsKey(const K& key) const {
            return findEntry(key) != -1;
        }

        /// Add an element into the map
        void add(const std::pair<K,V>& keyValue) {

            if (mCapacity == 0) {
                initialize(0);
            }

            // Compute the hash code of the key
            size_t hashCode = std::hash<K>()(keyValue.first);

            // Compute the corresponding bucket index
            int bucket = hashCode % mCapacity;

            // Check if the item is already in the map
            for (int i = mBuckets[bucket]; i >= 0; i = mEntries[i].next) {

                // If there is already an item with the same key in the map
                if (mEntries[i].hashCode == hashCode && mEntries[i].keyValue->first == keyValue.first) {

                    throw std::runtime_error("The key and value pair already exists in the map");
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

            assert(mEntries[entryIndex].keyValue == nullptr);
            mEntries[entryIndex].hashCode = hashCode;
            mEntries[entryIndex].next = mBuckets[bucket];
            mEntries[entryIndex].keyValue = static_cast<std::pair<K,V>*>(mAllocator.allocate(sizeof(std::pair<K,V>)));
            assert(mEntries[entryIndex].keyValue != nullptr);
            new (mEntries[entryIndex].keyValue) std::pair<K,V>(keyValue);
            mBuckets[bucket] = entryIndex;
        }

        /// Remove the element from the map with a given key
        bool remove(const K& key) {

            if (mCapacity > 0) {

                size_t hashcode = std::hash<K>()(key);
                int bucket = hashcode % mCapacity;
                int last = -1;
                for (int i = mBuckets[bucket]; i >= 0; last = i, i = mEntries[i].next) {

                    if (mEntries[i].hashCode == hashcode && mEntries[i].keyValue->first == key) {

                        if (last < 0 ) {
                           mBuckets[bucket] = mEntries[i].next;
                        }
                        else {
                           mEntries[last].next = mEntries[i].next;
                        }

                        // Release memory for the key/value pair if any
                        if (mEntries[i].keyValue != nullptr) {
                            mEntries[i].keyValue->~pair<K,V>();
                            mAllocator.release(mEntries[i].keyValue, sizeof(std::pair<K,V>));
                            mEntries[i].keyValue = nullptr;
                        }
                        mEntries[i].hashCode = -1;
                        mEntries[i].next = mFreeIndex;
                        mFreeIndex = i;
                        mNbFreeEntries++;

                        return true;
                    }
                }
            }

            return false;
        }

        /// Clear the list
        void clear() {

            if (mNbUsedEntries > 0) {

                for (int i=0; i < mCapacity; i++) {
                    mBuckets[i] = -1;
                    mEntries[i].next = -1;
                    if (mEntries[i].keyValue != nullptr) {
                        mEntries[i].keyValue->~pair<K,V>();
                        mAllocator.release(mEntries[i].keyValue, sizeof(std::pair<K,V>));
                        mEntries[i].keyValue = nullptr;
                    }
                }

                mFreeIndex = -1;
                mNbUsedEntries = 0;
                mNbFreeEntries = 0;
            }

            assert(size() == 0);
        }

        /// Return the number of elements in the map
        int size() const {
            return mNbUsedEntries - mNbFreeEntries;
        }

        /// Return the capacity of the map
        int capacity() const {
            return mCapacity;
        }

        /// Overloaded index operator
        V& operator[](const K& key) {

            int entry = -1;

            if (mCapacity > 0) {
                entry = findEntry(key);
            }

            if (entry == -1) {
                throw std::runtime_error("No item with given key has been found in the map");
            }

            assert(mEntries[entry].keyValue != nullptr);

            return mEntries[entry].keyValue->second;
        }

        /// Overloaded index operator
        const V& operator[](const K& key) const {

            int entry = -1;

            if (mCapacity > 0) {
                entry = findEntry(key);
            }

            if (entry == -1) {
                throw std::runtime_error("No item with given key has been found in the map");
            }

            return mEntries[entry];
        }

        /// Overloaded equality operator
        bool operator==(const Map<K,V>& map) const {

            // TODO : Implement this
            return false;
        }

        /// Overloaded not equal operator
        bool operator!=(const Map<K,V>& map) const {

            return !((*this) == map);
        }

        /// Overloaded assignment operator
        Map<K,V>& operator=(const Map<K, V>& map) {

            // Check for self assignment
            if (this != &map) {

                // Reset the map
                reset();

                if (map.mCapacity > 0) {

                    // Compute the next larger prime size
                    mCapacity = getPrimeSize(map.mCapacity);

                    // Allocate memory for the buckets
                    mBuckets = static_cast<int*>(mAllocator.allocate(mCapacity * sizeof(int)));

                    // Allocate memory for the entries
                    mEntries = static_cast<Entry*>(mAllocator.allocate(mCapacity * sizeof(Entry)));

                    // Copy the buckets
                    std::memcpy(mBuckets, map.mBuckets, mCapacity * sizeof(int));

                    // Copy the entries
                    std::memcpy(mEntries, map.mEntries, mCapacity * sizeof(Entry));

                    mNbUsedEntries = map.mNbUsedEntries;
                    mNbFreeEntries = map.mNbFreeEntries;
                    mFreeIndex = map.mFreeIndex;
                }
            }

            return *this;
        }
};

template<typename K, typename V>
const int Map<K,V>::PRIMES[NB_PRIMES] = {3, 7, 11, 17, 23, 29, 37, 47, 59, 71, 89, 107, 131, 163, 197, 239, 293, 353, 431, 521, 631, 761, 919,
                             1103, 1327, 1597, 1931, 2333, 2801, 3371, 4049, 4861, 5839, 7013, 8419, 10103, 12143, 14591,
                             17519, 21023, 25229, 30293, 36353, 43627, 52361, 62851, 75431, 90523, 108631, 130363, 156437,
                             187751, 225307, 270371, 324449, 389357, 467237, 560689, 672827, 807403, 968897, 1162687, 1395263,
                             1674319, 2009191, 2411033, 2893249, 3471899, 4166287, 4999559};

template<typename K, typename V>
int Map<K,V>::LARGEST_PRIME = -1;

}

#endif
