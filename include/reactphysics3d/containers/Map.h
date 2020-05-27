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

#ifndef REACTPHYSICS3D_MAP_H
#define REACTPHYSICS3D_MAP_H

// Libraries
#include <reactphysics3d/memory/MemoryAllocator.h>
#include <reactphysics3d/mathematics/mathematics_functions.h>
#include <reactphysics3d/containers/Pair.h>
#include <cstring>
#include <stdexcept>
#include <functional>
#include <limits>

namespace reactphysics3d {

// Class Map
/**
 * This class represents a simple generic associative map. This map is
 * implemented with a hash table.
  */
template<typename K, typename V, class Hash = std::hash<K>, class KeyEqual = std::equal_to<K>>
class Map {

    private:

        /// An entry of the map
        struct Entry {

            size_t hashCode;			// Hash code of the entry
            int next;					// Index of the next entry
            Pair<K, V>* keyValue;	    // Pointer to the pair with key and value

            /// Constructor
            Entry() {
                next = -1;
                keyValue = nullptr;
            }

            /// Constructor
            Entry(size_t hashcode, int nextEntry) {
                hashCode = hashcode;
                next = nextEntry;
                keyValue = nullptr;
            }

            /// Copy-constructor
            Entry(const Entry& entry) {
                hashCode = entry.hashCode;
                next = entry.next;
                keyValue = entry.keyValue;
            }

            /// Destructor
            ~Entry() {

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
            assert(mCapacity >= 0);

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

            assert(size() >= 0);
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

            if (mNbUsedEntries > 0) {

                // Copy the old entries to the new allocated memory location
                std::uninitialized_copy(mEntries, mEntries + mNbUsedEntries, newEntries);

                // Destruct the old entries in the previous location
                for (int i=0; i < mNbUsedEntries; i++) {
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
                if (newEntries[i].keyValue != nullptr) {

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

            assert(mCapacity >= 0);
        }

        /// Return the index of the entry with a given key or -1 if there is no entry with this key
        int findEntry(const K& key) const {

            if (mCapacity > 0) {

               const size_t hashCode = Hash()(key);
               int bucket = hashCode % mCapacity;
               auto keyEqual = KeyEqual();

               for (int i = mBuckets[bucket]; i >= 0; i = mEntries[i].next) {
                   if (mEntries[i].hashCode == hashCode && keyEqual(mEntries[i].keyValue->first, key)) {
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

    public:

        /// Class Iterator
        /**
         * This class represents an iterator for the Map
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
                        if (mEntries[mCurrentEntry].keyValue != nullptr) {

                           // We have found the next non empty entry
                           return;
                        }
                    }

                    // We have not find a non empty entry, we return an iterator to the end
                    mCurrentEntry = mCapacity;
                }

            public:

                // Iterator traits
                using value_type = Pair<K,V>;
                using difference_type = std::ptrdiff_t;
                using pointer = Pair<K, V>*;
                using reference = Pair<K,V>&;
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
                    assert(mEntries[mCurrentEntry].keyValue != nullptr);
                    return *(mEntries[mCurrentEntry].keyValue);
                }

                /// Deferencable
                pointer operator->() const {
                    assert(mCurrentEntry >= 0 && mCurrentEntry < mNbUsedEntries);
                    assert(mEntries[mCurrentEntry].keyValue != nullptr);
                    return mEntries[mCurrentEntry].keyValue;
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
           mBuckets(nullptr), mEntries(nullptr), mAllocator(map.mAllocator), mFreeIndex(map.mFreeIndex) {

            assert(capacity() >= 0);

            if (mCapacity > 0) {

                // Allocate memory for the buckets
                mBuckets = static_cast<int*>(mAllocator.allocate(mCapacity * sizeof(int)));

                // Allocate memory for the entries
                mEntries = static_cast<Entry*>(mAllocator.allocate(mCapacity * sizeof(Entry)));

                // Copy the buckets
                std::uninitialized_copy(map.mBuckets, map.mBuckets + mCapacity, mBuckets);

                // Copy the entries
                for (int i=0; i < mCapacity; i++) {

                    new (&mEntries[i]) Entry(map.mEntries[i].hashCode, map.mEntries[i].next);

                    if (map.mEntries[i].keyValue != nullptr) {
                       mEntries[i].keyValue = static_cast<Pair<K,V>*>(mAllocator.allocate(sizeof(Pair<K, V>)));
                       new (mEntries[i].keyValue) Pair<K,V>(*(map.mEntries[i].keyValue));
                    }
                }

            }

            assert(size() >= 0);
            assert((*this) == map);
        }

        /// Destructor
        ~Map() {

            clear(true);
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

        /// Return true if the map contains an item with the given key
        bool containsKey(const K& key) const {
            return findEntry(key) != -1;
        }

        /// Add an element into the map
        void add(const Pair<K,V>& keyValue, bool insertIfAlreadyPresent = false) {

            if (mCapacity == 0) {
                initialize(0);
            }

            // Compute the hash code of the key
            const size_t hashCode = Hash()(keyValue.first);

            // Compute the corresponding bucket index
            int bucket = hashCode % mCapacity;

            auto keyEqual = KeyEqual();

            // Check if the item is already in the map
            for (int i = mBuckets[bucket]; i >= 0; i = mEntries[i].next) {

                // If there is already an item with the same key in the map
                if (mEntries[i].hashCode == hashCode && keyEqual(mEntries[i].keyValue->first, keyValue.first)) {

                    if (insertIfAlreadyPresent) {

                        // Destruct the previous key/value
                        mEntries[i].keyValue->~Pair<K, V>();

                        // Copy construct the new key/value
                        new (mEntries[i].keyValue) Pair<K,V>(keyValue);

                        return;
                    }
                    else {
                        throw std::runtime_error("The key and value pair already exists in the map");
                    }
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

            assert(size() >= 0);
            assert(mEntries[entryIndex].keyValue == nullptr);
            mEntries[entryIndex].hashCode = hashCode;
            mEntries[entryIndex].next = mBuckets[bucket];
            mEntries[entryIndex].keyValue = static_cast<Pair<K,V>*>(mAllocator.allocate(sizeof(Pair<K,V>)));
            assert(mEntries[entryIndex].keyValue != nullptr);
            new (mEntries[entryIndex].keyValue) Pair<K,V>(keyValue);
            mBuckets[bucket] = entryIndex;
        }

        /// Remove the element pointed by some iterator
        /// This method returns an iterator pointing to the element after
        /// the one that has been removed
        Iterator remove(const Iterator& it) {

            const K& key = it->first;
            return remove(key);
        }

        /// Remove the element from the map with a given key
        /// This method returns an iterator pointing to the element after
        /// the one that has been removed
        Iterator remove(const K& key) {

            if (mCapacity > 0) {

                const size_t hashcode = Hash()(key);
                int bucket = hashcode % mCapacity;
                int last = -1;
                auto keyEqual = KeyEqual();

                for (int i = mBuckets[bucket]; i >= 0; last = i, i = mEntries[i].next) {

                    if (mEntries[i].hashCode == hashcode &&  keyEqual(mEntries[i].keyValue->first, key)) {

                        if (last < 0 ) {
                           mBuckets[bucket] = mEntries[i].next;
                        }
                        else {
                           mEntries[last].next = mEntries[i].next;
                        }

                        // Release memory for the key/value pair if any
                        if (mEntries[i].keyValue != nullptr) {
                            mEntries[i].keyValue->~Pair<K,V>();
                            mAllocator.release(mEntries[i].keyValue, sizeof(Pair<K,V>));
                            mEntries[i].keyValue = nullptr;
                        }
                        assert(mEntries[i].keyValue == nullptr);
                        mEntries[i].next = mFreeIndex;
                        mFreeIndex = i;
                        mNbFreeEntries++;

                        // Find the next entry to return the iterator
                        for (i += 1; i < mNbUsedEntries; i++) {

                            // If the entry is not empty
                            if (mEntries[i].keyValue != nullptr) {

                               // We have found the next non empty entry
                               return Iterator(mEntries, mCapacity, mNbUsedEntries, i);
                            }
                        }

                        return end();
                    }
                }
            }

            assert(size() >= 0);

            // Return the end iterator
            return end();
        }

        /// Clear the map
        void clear(bool releaseMemory = false) {

            if (mNbUsedEntries > 0) {

                // Remove the key/value pair of each entry
                for (int i=0; i < mCapacity; i++) {

                    mBuckets[i] = -1;
                    mEntries[i].next = -1;
                    if (mEntries[i].keyValue != nullptr) {
                        mEntries[i].keyValue->~Pair<K,V>();
                        mAllocator.release(mEntries[i].keyValue, sizeof(Pair<K,V>));
                        mEntries[i].keyValue = nullptr;
                    }
                }

                mFreeIndex = -1;
                mNbUsedEntries = 0;
                mNbFreeEntries = 0;

                assert(size() >= 0);
            }

            // If elements have been allocated
            if (releaseMemory && mCapacity > 0) {

                // Destroy the entries
                for (int i=0; i < mCapacity; i++) {
                    mEntries[i].~Entry();
                }

                // Release memory
                mAllocator.release(mBuckets, mCapacity * sizeof(int));
                mAllocator.release(mEntries, mCapacity * sizeof(Entry));

                mCapacity = 0;
                mBuckets = nullptr;
                mEntries = nullptr;
            }

            assert(size() == 0);
        }

        /// Return the number of elements in the map
        int size() const {
            assert(mNbUsedEntries - mNbFreeEntries >= 0);
            return mNbUsedEntries - mNbFreeEntries;
        }

        /// Return the capacity of the map
        int capacity() const {
            return mCapacity;
        }

        /// Try to find an item of the map given a key.
        /// The method returns an iterator to the found item or
        /// an iterator pointing to the end if not found
        Iterator find(const K& key) const {

            int bucket;
            int entry = -1;

            if (mCapacity > 0) {

               const size_t hashCode = Hash()(key);
               bucket = hashCode % mCapacity;

               auto keyEqual = KeyEqual();

               for (int i = mBuckets[bucket]; i >= 0; i = mEntries[i].next) {
                   if (mEntries[i].hashCode == hashCode && keyEqual(mEntries[i].keyValue->first, key)) {
                       entry = i;
                       break;
                   }
               }
            }

            if (entry == -1) {
                return end();
            }

            assert(mEntries[entry].keyValue != nullptr);

            return Iterator(mEntries, mCapacity, mNbUsedEntries, entry);
        }

        /// Overloaded index operator
        V& operator[](const K& key) {

            int entry = -1;

            if (mCapacity > 0) {
                entry = findEntry(key);
            }

            if (entry == -1) {
                assert(false);
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

            assert(mEntries[entry].keyValue != nullptr);

            return mEntries[entry].keyValue->second;
        }

        /// Overloaded equality operator
        bool operator==(const Map<K,V>& map) const {

            if (size() != map.size()) return false;

            for (auto it = begin(); it != end(); ++it) {
                auto it2 = map.find(it->first);
                if (it2 == map.end() || it2->second != it->second) {
                    return false;
                }
            }

            for (auto it = map.begin(); it != map.end(); ++it) {
                auto it2 = find(it->first);
                if (it2 == end() || it2->second != it->second) {
                    return false;
                }
            }

            return true;
        }

        /// Overloaded not equal operator
        bool operator!=(const Map<K,V>& map) const {

            return !((*this) == map);
        }

        /// Overloaded assignment operator
        Map<K,V>& operator=(const Map<K, V>& map) {

            // Check for self assignment
            if (this != &map) {

                // Clear the map
                clear(true);

                if (map.mCapacity > 0) {

                    // Compute the next larger prime size
                    mCapacity = getPrimeSize(map.mCapacity);
                    assert(mCapacity >= 0);

                    // Allocate memory for the buckets
                    mBuckets = static_cast<int*>(mAllocator.allocate(mCapacity * sizeof(int)));

                    // Allocate memory for the entries
                    mEntries = static_cast<Entry*>(mAllocator.allocate(mCapacity * sizeof(Entry)));

                    // Copy the buckets
                    std::uninitialized_copy(map.mBuckets, map.mBuckets + mCapacity, mBuckets);

                    // Copy the entries
                    for (int i=0; i < mCapacity; i++) {

                        new (&mEntries[i]) Entry(map.mEntries[i].hashCode, map.mEntries[i].next);

                        if (map.mEntries[i].keyValue != nullptr) {
                           mEntries[i].keyValue = static_cast<Pair<K,V>*>(mAllocator.allocate(sizeof(Pair<K, V>)));
                           new (mEntries[i].keyValue) Pair<K,V>(*(map.mEntries[i].keyValue));
                        }
                    }

                    mNbUsedEntries = map.mNbUsedEntries;
                    mNbFreeEntries = map.mNbFreeEntries;
                    mFreeIndex = map.mFreeIndex;
                }
            }

            assert(size() >= 0);

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
                if (mEntries[entry].keyValue != nullptr) {
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

template<typename K, typename V, class Hash, class KeyEqual>
const int Map<K,V, Hash, KeyEqual>::PRIMES[NB_PRIMES] = {3, 7, 11, 17, 23, 29, 37, 47, 59, 71, 89, 107, 131, 163, 197, 239, 293, 353, 431, 521, 631, 761, 919,
                             1103, 1327, 1597, 1931, 2333, 2801, 3371, 4049, 4861, 5839, 7013, 8419, 10103, 12143, 14591,
                             17519, 21023, 25229, 30293, 36353, 43627, 52361, 62851, 75431, 90523, 108631, 130363, 156437,
                             187751, 225307, 270371, 324449, 389357, 467237, 560689, 672827, 807403, 968897, 1162687, 1395263,
                             1674319, 2009191, 2411033, 2893249, 3471899, 4166287, 4999559};

template<typename K, typename V, class Hash, class KeyEqual>
int Map<K,V, Hash, KeyEqual>::LARGEST_PRIME = -1;

}

#endif
