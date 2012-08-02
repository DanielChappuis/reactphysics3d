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

// Libraries
#include "PairManager.h"
#include "../CollisionDetection.h"
#include <cassert>

using namespace reactphysics3d;

// Initialization of static variables
bodyindex PairManager::INVALID_INDEX = std::numeric_limits<reactphysics3d::bodyindex>::max();

// Constructor of PairManager
PairManager::PairManager(CollisionDetection& collisionDetection) : collisionDetection(collisionDetection) {
    hashTable = 0;
    overlappingPairs = 0;
    offsetNextPair = 0;
    nbOverlappingPairs = 0;
    hashMask = 0;
    nbElementsHashTable = 0;
}

// Destructor of PairManager
PairManager::~PairManager() {
    
    // Release the allocated memory
    free(offsetNextPair);
    free(overlappingPairs);
    free(hashTable);
}

// Add a pair of bodies in the pair manager and returns a pointer to
// that pair. If the pair to add does not already exist in the set of 
// overlapping pairs, it will be created and if it already exists, we only
// return a pointer to that pair.
BroadPhasePair* PairManager::addPair(Body* body1, Body* body2) {

    // Sort the bodies to have the body with smallest ID first
    sortBodiesUsingID(body1, body2);
    
    // Get the bodies IDs
    bodyindex id1 = body1->getID();
    bodyindex id2 = body2->getID();
    
    // Compute the hash value of the two bodies
    uint hashValue = computeHashBodies(id1, id2) & hashMask;
    
    // Try to find the pair in the current overlapping pairs.
    BroadPhasePair* pair = findPairWithHashValue(id1, id2, hashValue);

    // If the pair is already in the set of overlapping pairs
    if (pair) {
        // We only return a pointer to that pair
        return pair;
    }
    
    // If we need to allocate more pairs in the set of overlapping pairs
    if (nbOverlappingPairs >= nbElementsHashTable) {
        // Increase the size of the hash table (always a power of two)
        nbElementsHashTable = computeNextPowerOfTwo(nbOverlappingPairs + 1);
        
        // Compute the new hash mask with the new hash size
        hashMask = nbElementsHashTable - 1;
        
        // Reallocate more pairs
        reallocatePairs();
        
        // Compute the new hash value (using the new hash size and hash mask)
        hashValue = computeHashBodies(id1, id2) & hashMask;
    }
    
    // Create the new overlapping pair
    BroadPhasePair* newPair = &overlappingPairs[nbOverlappingPairs];
    newPair->body1 = body1;
    newPair->body2 = body2;
    
    // Put the new pair as the initial pair with this hash value
    offsetNextPair[nbOverlappingPairs] = hashTable[hashValue];
    hashTable[hashValue] = nbOverlappingPairs++;
    
    // Notify the collision detection about this new overlapping pair
    collisionDetection.broadPhaseNotifyAddedOverlappingPair(newPair);
    
    // Return a pointer to the new created pair
    return newPair;
} 

// Remove a pair of bodies from the pair manager. This method returns
// true if the pair has been found and removed.
bool PairManager::removePair(bodyindex id1, bodyindex id2) {
    
    // Sort the bodies IDs
    sortIDs(id1, id2);
    
    // Compute the hash value of the pair to remove
    const uint hashValue = computeHashBodies(id1, id2) & hashMask;
    
    // Find the pair to remove
    const BroadPhasePair* pair = findPairWithHashValue(id1, id2, hashValue);
    
    // If we have not found the pair
    if (!pair) {
        return false;
    }
    
    assert(pair->body1->getID() == id1);
    assert(pair->body2->getID() == id2);
    
    // Notify the collision detection about this removed overlapping pair
    collisionDetection.broadPhaseNotifyRemovedOverlappingPair(pair);
    
    // Remove the pair from the set of overlapping pairs
    removePairWithHashValue(id1, id2, hashValue, computePairOffset(pair));
    
    // Try to shrink the memory used by the pair manager
    shrinkMemory();
    
    return true;
}

// Internal method to remove a pair from the set of overlapping pair
void PairManager::removePairWithHashValue(bodyindex id1, bodyindex id2, luint hashValue, bodyindex indexPair) {
    
    // Get the initial offset of the pairs with
    // the corresponding hash value
    bodyindex offset = hashTable[hashValue];
    assert(offset != INVALID_INDEX);
    
    // Look for the pair in the set of overlapping pairs
    bodyindex previousPair = INVALID_INDEX;
    while(offset != indexPair) {
        previousPair = offset;
        offset = offsetNextPair[offset];
    }
  
    // If the pair was the first one with this hash
    // value in the hash table
    if (previousPair == INVALID_INDEX) {
        // Replace the pair to remove in the
        // hash table by the next one
        hashTable[hashValue] = offsetNextPair[indexPair];
    }
    else {  // If the pair was not the first one
        // Replace the pair to remove in the
        // hash table by the next one
        assert(offsetNextPair[previousPair] == indexPair);
        offsetNextPair[previousPair] = offsetNextPair[indexPair];
    }
    
    const bodyindex indexLastPair = nbOverlappingPairs - 1;
    
    // If the pair to remove is the last one in the list
    if (indexPair == indexLastPair) {
        
        // We simply decrease the number of overlapping pairs
        nbOverlappingPairs--;
    }
    else {  // If the pair to remove is in the middle of the list
        
        // Now, we want to move the last pair into the location that is
        // now free because of the pair we want to remove
        
        // Get the last pair
        const BroadPhasePair* lastPair = &overlappingPairs[indexLastPair];
        const uint lastPairHashValue = computeHashBodies(lastPair->body1->getID(), lastPair->body2->getID()) & hashMask;
        
        // Compute the initial offset of the last pair
        bodyindex offset = hashTable[lastPairHashValue];
        assert(offset != INVALID_INDEX);
        
        // Go through the pairs with the same hash value
        // and find the offset of the last pair
        bodyindex previous = INVALID_INDEX;
        while(offset != indexLastPair) {
           previous = offset;
           offset = offsetNextPair[offset];
        }
        
        // If the last pair is not the first one with this hash value
        if (previous != INVALID_INDEX) {
            
            // Remove the offset of the last pair in the "nextOffset" array
            assert(offsetNextPair[previous] == indexLastPair);
            offsetNextPair[previous] = offsetNextPair[indexLastPair];
        }
        else {  // If the last pair is the first offset with this hash value
            
            // Remove the offset of the last pair in the "nextOffset" array
            hashTable[lastPairHashValue] = offsetNextPair[indexLastPair];
        }
        
        // Replace the pair to remove by the last pair in
        // the overlapping pairs array
        overlappingPairs[indexPair] = overlappingPairs[indexLastPair];
        offsetNextPair[indexPair] = hashTable[lastPairHashValue];
        hashTable[lastPairHashValue] = indexPair;
        
        nbOverlappingPairs--;
    }
}                                

// Look for a pair in the set of overlapping pairs
BroadPhasePair* PairManager::lookForAPair(bodyindex id1, bodyindex id2, luint hashValue) const {
    
    // Look for the pair in the set of overlapping pairs
    bodyindex offset = hashTable[hashValue];
    while (offset != INVALID_INDEX && isDifferentPair(overlappingPairs[offset], id1, id2)) {
        offset = offsetNextPair[offset];
    }
    
    // If the pair has not been found in the overlapping pairs
    if (offset == INVALID_INDEX) {
        // Return null
        return 0;
    }
    
    assert(offset < nbOverlappingPairs);
    
    // The pair has been found in the set of overlapping pairs, then
    // we return a pointer to it
    return &overlappingPairs[offset];
}

// Reallocate more pairs
void PairManager::reallocatePairs() {
    
    // Reallocate the hash table and initialize it
    free(hashTable);
    hashTable = (bodyindex*) malloc(nbElementsHashTable * sizeof(bodyindex));
    assert(hashTable);
    for (bodyindex i=0; i<nbElementsHashTable; i++) {
        hashTable[i] = INVALID_INDEX;
    }
    
    // Reallocate the overlapping pairs
    BroadPhasePair* newOverlappingPairs = (BroadPhasePair*) malloc(nbElementsHashTable * sizeof(BroadPhasePair));
    bodyindex* newOffsetNextPair = (bodyindex*) malloc(nbElementsHashTable * sizeof(bodyindex));
    
    assert(newOverlappingPairs);
    assert(newOffsetNextPair);
    
    // If there is already some overlapping pairs
    if (nbOverlappingPairs) {
        // Copy the pairs to the new place
        memcpy(newOverlappingPairs, overlappingPairs, nbOverlappingPairs * sizeof(BroadPhasePair));
    }
    
    // Recompute the hash table with the new hash values
    for (bodyindex i=0; i<nbOverlappingPairs; i++) {
        const uint newHashValue = computeHashBodies(overlappingPairs[i].body1->getID(), overlappingPairs[i].body2->getID()) & hashMask;
        newOffsetNextPair[i] = hashTable[newHashValue];
        hashTable[newHashValue] = i;
    }
    
    // Delete the old pairs
    free(offsetNextPair);
    free(overlappingPairs);
    
    // Replace by the new data
    overlappingPairs = newOverlappingPairs;
    offsetNextPair = newOffsetNextPair;
}                                

