/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2010-2012 Daniel Chappuis                                       *
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

#ifndef PAIR_MANAGER_H
#define PAIR_MANAGER_H

// Libraries
#include "../../body/CollisionBody.h"
#include <utility>


// Namespace ReactPhysics3D
namespace reactphysics3d {
    
// Declaration
class CollisionDetection;

// struct BodyPair
// This structure represents a pair of bodies
// during the broad-phase collision detection
struct BodyPair {

    public:
        CollisionBody* body1;                // Pointer to the first body
        CollisionBody* body2;                // Pointer to the second body

        // Return the pair of bodies index
        bodyindexpair getBodiesIndexPair() const {

            // Construct the pair of body index
           bodyindexpair indexPair = body1->getID() < body2->getID() ? std::make_pair(body1->getID(), body2->getID()) :
                                                                    std::make_pair(body2->getID(), body1->getID());
            assert(indexPair.first != indexPair.second);
            return indexPair;
        }
};

/*  --------------------------------------------------------------------
    Class PairManager :
        This class is a data-structure contains the pairs of bodies that
        are overlapping during the broad-phase collision detection.
        This class implements the pair manager described by Pierre Terdiman
        in www.codercorner.com/SAP.pdf.
    --------------------------------------------------------------------
*/
class PairManager {
    private :
        bodyindex nbElementsHashTable;                                          // Number of elements in the hash table
        uint hashMask;                                                          // Hash mask for the hash function
        bodyindex nbOverlappingPairs;                                           // Number of overlapping pairs
        bodyindex* hashTable;                                                   // Hash table that contains the offset of the first pair of the list of
                                                                                // pairs with the same hash value in the "overlappingPairs" array
        bodyindex* offsetNextPair;                                              // Array that contains for each offset, the offset of the next pair with
                                                                                // the same hash value
                                                                                // for a given same hash value
        BodyPair* overlappingPairs;                                             // Array that contains the currently active pairs
        static bodyindex INVALID_INDEX;                                         // Invalid ID
        CollisionDetection& collisionDetection;                                 // Reference to the collision detection
        
        void sortBodiesUsingID(CollisionBody*& body1, CollisionBody*& body2) const; // Sort the bodies according to their IDs (smallest ID first)
        void sortIDs(bodyindex& id1, bodyindex& id2) const;                         // Sort the IDs (smallest ID first)
        bool isDifferentPair(const BodyPair& pair1, bodyindex pair2ID1,
                             bodyindex pair2ID2) const;                         // Return true if pair1 and pair2 are the same
        uint computeHashBodies(uint id1, uint id2) const;                       // Compute the hash value of two bodies using their IDs
        int computeHash32Bits(int key) const;                                   // This method returns an hash value for a 32 bits key
        luint computeNextPowerOfTwo(luint number) const;                        // Return the next power of two
        void reallocatePairs();                                                 // Reallocate memory for more pairs
        void shrinkMemory();                                                    // Shrink the allocated memory
        bodyindex computePairOffset(const BodyPair* pair) const;                // Compute the offset of a given pair
        BodyPair* lookForAPair(bodyindex id1, bodyindex id2,
                                     luint hashValue) const;                    // Look for a pair in the set of overlapping pairs
        BodyPair* findPairWithHashValue(bodyindex id1, bodyindex id2,
                                              luint hashValue) const;           // Find a pair given two body IDs and an hash value
        void removePairWithHashValue(bodyindex id1, bodyindex id2, luint hashValue,
                                     bodyindex indexPair);                      // Remove a pair from the set of active pair
    public :
        PairManager(CollisionDetection& collisionDetection);                    // Constructor
        ~PairManager();                                                         // Destructor
        
        bodyindex getNbOverlappingPairs() const;                                // Return the number of active pairs
        BodyPair* addPair(CollisionBody* body1, CollisionBody* body2);                                                      // Add a pair of bodies in the pair manager
        bool removePair(bodyindex id1, bodyindex id2);                                                                      // Remove a pair of bodies from the pair manager
        BodyPair* findPair(bodyindex id1, bodyindex id2) const;                                                             // Find a pair given two body IDs
        BodyPair* beginOverlappingPairsPointer() const;                                                                     // Return a pointer to the first overlapping pair (used to iterate over the active pairs)
        BodyPair* endOverlappingPairsPointer() const;                                                                       // Return a pointer to the last overlapping pair (used to iterate over the active pairs)
        void registerAddedOverlappingPairCallback(void (CollisionDetection::*callbackFunction) (const BodyPair* addedActivePair));     // Register a callback function (using a function pointer) that will be called when a new overlapping pair is added in the pair manager
        void unregisterAddedOverlappingPairCallback();                                                                   // Unregister the callback function that will be called when a new active pair is added in the pair manager
        void registerRemovedOverlappingPairCallback(void (CollisionDetection::*callbackFunction) (const BodyPair* removedActivePair)); // Register a callback function (using a function pointer) that will be called when an overlapping pair is removed from the pair manager
        void unregisterRemovedOverlappingPairCallback();                                                                 // Unregister a callback function that will be called when a active pair is removed from the pair manager
};

// Return the number of overlapping pairs
inline bodyindex PairManager::getNbOverlappingPairs() const {
    return nbOverlappingPairs;
}                                         

// Compute the hash value of two bodies
inline uint PairManager::computeHashBodies(uint id1, uint id2) const {
    return computeHash32Bits(id1 | (id2 << 16));
}  

// Return true if pair1 and pair2 are the same
inline bool PairManager::isDifferentPair(const BodyPair& pair1, bodyindex pair2ID1, bodyindex pair2ID2) const {
    return (pair2ID1 != pair1.body1->getID() || pair2ID2 != pair1.body2->getID());
}

// Return the next power of two of a 32bits integer using a SWAR algorithm
inline luint PairManager::computeNextPowerOfTwo(luint number) const {
    number |= (number >> 1);
    number |= (number >> 2);
    number |= (number >> 4);
    number |= (number >> 8);
    number |= (number >> 16);
    return number+1;
}        

// Sort the bodies according to their IDs (smallest ID first)
inline void PairManager::sortBodiesUsingID(CollisionBody*& body1, CollisionBody*& body2) const {
    
    // If the ID of body1 is larger than the ID of body 2
    if (body1->getID() > body2->getID()) {
        
        // Swap the two bodies pointers
        CollisionBody* temp = body2;
        body2 = body1;
        body1 = temp;
    }
} 

// Sort the IDs (smallest ID first)
inline void PairManager::sortIDs(bodyindex &id1, bodyindex &id2) const {
    if (id1 > id2) {
        bodyindex temp = id2;
        id2 = id1;
        id1 = temp;
    }
}

// This method returns an hash value for a 32 bits key
// using Thomas Wang's hash technique.
// This hash function can be found at :
// http://www.concentric.net/~ttwang/tech/inthash.htm
inline int PairManager::computeHash32Bits(int key) const {
    key += ~(key << 15);
    key ^=  (key >> 10);
    key +=  (key << 3);
    key ^=  (key >> 6);
    key += ~(key << 11);
    key ^=  (key >> 16);
    return key;
}

// Find a pair given two body IDs
inline BodyPair* PairManager::findPair(bodyindex id1, bodyindex id2) const {
    
    // Check if the hash table has been allocated yet
    if (!hashTable) return 0;
    
    // Sort the IDs
    sortIDs(id1, id2);
    
    // Compute the hash value of the pair to find
    uint hashValue = computeHashBodies(id1, id2) & hashMask;
    
    // Look for the pair in the set of overlapping pairs
    lookForAPair(id1, id2, hashValue);
}

// Find a pair given two body IDs and an hash value
// This internal version is used to avoid computing multiple times in the 
// caller method
inline BodyPair* PairManager::findPairWithHashValue(bodyindex id1, bodyindex id2, luint hashValue) const {
    
    // Check if the hash table has been allocated yet
    if (!hashTable) return 0;
    
    // Look for the pair in the set of overlapping pairs
    return lookForAPair(id1, id2, hashValue);
}

// Try to reduce the allocated memory by the pair manager
inline void PairManager::shrinkMemory() {
    
    // Check if the allocated memory can be reduced
    const bodyindex correctNbElementsHashTable = computeNextPowerOfTwo(nbOverlappingPairs);
    if (nbElementsHashTable == correctNbElementsHashTable) return;
    
    // Reduce the allocated memory
    nbElementsHashTable = correctNbElementsHashTable;
    hashMask = nbElementsHashTable - 1;
    reallocatePairs();
}

// Compute the offset of a given pair in the array of overlapping pairs
inline bodyindex PairManager::computePairOffset(const BodyPair* pair) const {
    return ((bodyindex)((size_t(pair) - size_t(overlappingPairs))) / sizeof(BodyPair));
}

// Return a pointer to the first overlapping pair (used to iterate over the overlapping pairs) or
// returns 0 if there is no overlapping pairs.
inline BodyPair* PairManager::beginOverlappingPairsPointer() const {
    return &overlappingPairs[0];
}

// Return a pointer to the last overlapping pair (used to iterate over the overlapping pairs) or
// returns 0 if there is no overlapping pairs.
inline BodyPair* PairManager::endOverlappingPairsPointer() const {
    if (nbOverlappingPairs > 0) {
        return &overlappingPairs[nbOverlappingPairs-1];
    }
    else {
        return &overlappingPairs[0];
    }
}

} // End of reactphysics3d namespace

#endif


