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
PairManager::PairManager(CollisionDetection& collisionDetection)
            : mCollisionDetection(collisionDetection) {
    mHashTable = NULL;
    mOverlappingPairs = NULL;
    mOffsetNextPair = NULL;
    mNbOverlappingPairs = 0;
    mHashMask = 0;
    mNbElementsHashTable = 0;
}

// Destructor of PairManager
PairManager::~PairManager() {
    
    // Release the allocated memory
    free(mOffsetNextPair);
    free(mOverlappingPairs);
    free(mHashTable);
}

// Add a pair of bodies in the pair manager and returns a pointer to
// that pair. If the pair to add does not already exist in the set of 
// overlapping pairs, it will be created and if it already exists, we only
// return a pointer to that pair.
BodyPair* PairManager::addPair(CollisionBody* body1, CollisionBody* body2) {

    // Sort the bodies to have the body with smallest ID first
    sortBodiesUsingID(body1, body2);
    
    // Get the bodies IDs
    bodyindex id1 = body1->getID();
    bodyindex id2 = body2->getID();
    
    // Compute the hash value of the two bodies
    uint hashValue = computeHashBodies(id1, id2) & mHashMask;
    
    // Try to find the pair in the current overlapping pairs.
    BodyPair* pair = findPairWithHashValue(id1, id2, hashValue);

    // If the pair is already in the set of overlapping pairs
    if (pair) {
        // We only return a pointer to that pair
        return pair;
    }
    
    // If we need to allocate more pairs in the set of overlapping pairs
    if (mNbOverlappingPairs >= mNbElementsHashTable) {
        // Increase the size of the hash table (always a power of two)
        mNbElementsHashTable = computeNextPowerOfTwo(mNbOverlappingPairs + 1);
        
        // Compute the new hash mask with the new hash size
        mHashMask = mNbElementsHashTable - 1;
        
        // Reallocate more pairs
        reallocatePairs();
        
        // Compute the new hash value (using the new hash size and hash mask)
        hashValue = computeHashBodies(id1, id2) & mHashMask;
    }
    
    // Create the new overlapping pair
    BodyPair* newPair = &mOverlappingPairs[mNbOverlappingPairs];
    newPair->body1 = body1;
    newPair->body2 = body2;
    
    // Put the new pair as the initial pair with this hash value
    mOffsetNextPair[mNbOverlappingPairs] = mHashTable[hashValue];
    mHashTable[hashValue] = mNbOverlappingPairs++;
    
    // Notify the collision detection about this new overlapping pair
    mCollisionDetection.broadPhaseNotifyAddedOverlappingPair(newPair);
    
    // Return a pointer to the new created pair
    return newPair;
} 

// Remove a pair of bodies from the pair manager. This method returns
// true if the pair has been found and removed.
bool PairManager::removePair(bodyindex id1, bodyindex id2) {
    
    // Sort the bodies IDs
    sortIDs(id1, id2);
    
    // Compute the hash value of the pair to remove
    const uint hashValue = computeHashBodies(id1, id2) & mHashMask;
    
    // Find the pair to remove
    BodyPair* pair = findPairWithHashValue(id1, id2, hashValue);
    
    // If we have not found the pair
    if (pair == NULL) {
        return false;
    }
    
    assert(pair->body1->getID() == id1);
    assert(pair->body2->getID() == id2);
    
    // Notify the collision detection about this removed overlapping pair
    mCollisionDetection.broadPhaseNotifyRemovedOverlappingPair(pair);
    
    // Remove the pair from the set of overlapping pairs
    removePairWithHashValue(id1, id2, hashValue, computePairOffset(pair));
    
    // Try to shrink the memory used by the pair manager
    shrinkMemory();
    
    return true;
}

// Internal method to remove a pair from the set of overlapping pair
void PairManager::removePairWithHashValue(bodyindex id1, bodyindex id2, luint hashValue,
                                          bodyindex indexPair) {
    
    // Get the initial offset of the pairs with
    // the corresponding hash value
    bodyindex offset = mHashTable[hashValue];
    assert(offset != INVALID_INDEX);
    
    // Look for the pair in the set of overlapping pairs
    bodyindex previousPair = INVALID_INDEX;
    while(offset != indexPair) {
        previousPair = offset;
        offset = mOffsetNextPair[offset];
    }
  
    // If the pair was the first one with this hash
    // value in the hash table
    if (previousPair == INVALID_INDEX) {
        // Replace the pair to remove in the
        // hash table by the next one
        mHashTable[hashValue] = mOffsetNextPair[indexPair];
    }
    else {  // If the pair was not the first one
        // Replace the pair to remove in the
        // hash table by the next one
        assert(mOffsetNextPair[previousPair] == indexPair);
        mOffsetNextPair[previousPair] = mOffsetNextPair[indexPair];
    }
    
    const bodyindex indexLastPair = mNbOverlappingPairs - 1;
    
    // If the pair to remove is the last one in the list
    if (indexPair == indexLastPair) {
        
        // We simply decrease the number of overlapping pairs
        mNbOverlappingPairs--;
    }
    else {  // If the pair to remove is in the middle of the list
        
        // Now, we want to move the last pair into the location that is
        // now free because of the pair we want to remove
        
        // Get the last pair
        const BodyPair* lastPair = &mOverlappingPairs[indexLastPair];
        const uint lastPairHashValue = computeHashBodies(lastPair->body1->getID(),
                                                         lastPair->body2->getID()) & mHashMask;
        
        // Compute the initial offset of the last pair
        bodyindex offset = mHashTable[lastPairHashValue];
        assert(offset != INVALID_INDEX);
        
        // Go through the pairs with the same hash value
        // and find the offset of the last pair
        bodyindex previous = INVALID_INDEX;
        while(offset != indexLastPair) {
           previous = offset;
           offset = mOffsetNextPair[offset];
        }
        
        // If the last pair is not the first one with this hash value
        if (previous != INVALID_INDEX) {
            
            // Remove the offset of the last pair in the "nextOffset" array
            assert(mOffsetNextPair[previous] == indexLastPair);
            mOffsetNextPair[previous] = mOffsetNextPair[indexLastPair];
        }
        else {  // If the last pair is the first offset with this hash value
            
            // Remove the offset of the last pair in the "nextOffset" array
            mHashTable[lastPairHashValue] = mOffsetNextPair[indexLastPair];
        }
        
        // Replace the pair to remove by the last pair in
        // the overlapping pairs array
        mOverlappingPairs[indexPair] = mOverlappingPairs[indexLastPair];
        mOffsetNextPair[indexPair] = mHashTable[lastPairHashValue];
        mHashTable[lastPairHashValue] = indexPair;
        
        mNbOverlappingPairs--;
    }
}                                

// Look for a pair in the set of overlapping pairs
BodyPair* PairManager::lookForAPair(bodyindex id1, bodyindex id2, luint hashValue) const {
    
    // Look for the pair in the set of overlapping pairs
    bodyindex offset = mHashTable[hashValue];
    while (offset != INVALID_INDEX && isDifferentPair(mOverlappingPairs[offset], id1, id2)) {
        offset = mOffsetNextPair[offset];
    }
    
    // If the pair has not been found in the overlapping pairs
    if (offset == INVALID_INDEX) {
        return NULL;
    }
    
    assert(offset < mNbOverlappingPairs);
    
    // The pair has been found in the set of overlapping pairs, then
    // we return a pointer to it
    return &mOverlappingPairs[offset];
}

// Reallocate more pairs
void PairManager::reallocatePairs() {
    
    // Reallocate the hash table and initialize it
    free(mHashTable);
    mHashTable = (bodyindex*) malloc(mNbElementsHashTable * sizeof(bodyindex));
    assert(mHashTable != NULL);
    for (bodyindex i=0; i<mNbElementsHashTable; i++) {
        mHashTable[i] = INVALID_INDEX;
    }
    
    // Reallocate the overlapping pairs
    BodyPair* newOverlappingPairs = (BodyPair*) malloc(mNbElementsHashTable * sizeof(BodyPair));
    bodyindex* newOffsetNextPair = (bodyindex*) malloc(mNbElementsHashTable * sizeof(bodyindex));
    
    assert(newOverlappingPairs != NULL);
    assert(newOffsetNextPair != NULL);
    
    // If there is already some overlapping pairs
    if (mNbOverlappingPairs) {
        // Copy the pairs to the new location
        memcpy(newOverlappingPairs, mOverlappingPairs, mNbOverlappingPairs * sizeof(BodyPair));
    }
    
    // Recompute the hash table with the new hash values
    for (bodyindex i=0; i<mNbOverlappingPairs; i++) {
        const uint newHashValue = computeHashBodies(mOverlappingPairs[i].body1->getID(),
                                                   mOverlappingPairs[i].body2->getID()) & mHashMask;
        newOffsetNextPair[i] = mHashTable[newHashValue];
        mHashTable[newHashValue] = i;
    }
    
    // Delete the old pairs
    free(mOffsetNextPair);
    free(mOverlappingPairs);
    
    // Replace by the new data
    mOverlappingPairs = newOverlappingPairs;
    mOffsetNextPair = newOffsetNextPair;
}
