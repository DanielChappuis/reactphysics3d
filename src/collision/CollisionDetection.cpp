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
#include "CollisionDetection.h"
#include "../engine/CollisionWorld.h"
#include "broadphase/SweepAndPruneAlgorithm.h"
#include "broadphase/NoBroadPhaseAlgorithm.h"
#include "../body/Body.h"
#include "../collision/shapes/BoxShape.h"
#include "../body/RigidBody.h"
#include "../configuration.h"
#include <cassert>
#include <complex>
#include <set>
#include <utility>
#include <utility>

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;
using namespace std;

// Constructor
CollisionDetection::CollisionDetection(CollisionWorld* world, MemoryAllocator& memoryAllocator)
                   : mWorld(world), mMemoryAllocator(memoryAllocator),
                     mNarrowPhaseGJKAlgorithm(memoryAllocator),
                     mNarrowPhaseSphereVsSphereAlgorithm(memoryAllocator) {

    // Create the broad-phase algorithm that will be used (Sweep and Prune with AABB)
    mBroadPhaseAlgorithm = new SweepAndPruneAlgorithm(*this);
    assert(mBroadPhaseAlgorithm != NULL);
}

// Destructor
CollisionDetection::~CollisionDetection() {
    
    // Delete the broad-phase algorithm
    delete mBroadPhaseAlgorithm;
}

// Compute the collision detection
void CollisionDetection::computeCollisionDetection() {

    PROFILE("CollisionDetection::computeCollisionDetection()");
	    
    // Compute the broad-phase collision detection
    computeBroadPhase();
    
    // Compute the narrow-phase collision detection
    computeNarrowPhase();
}

// Compute the broad-phase collision detection
void CollisionDetection::computeBroadPhase() {

    PROFILE("CollisionDetection::computeBroadPhase()");

    // Notify the broad-phase algorithm about the bodies that have moved since last frame
    for (set<CollisionBody*>::iterator it = mWorld->getBodiesBeginIterator();
         it != mWorld->getBodiesEndIterator(); it++) {

        // If the body has moved
        if ((*it)->getHasMoved()) {

            // Notify the broad-phase that the body has moved
            mBroadPhaseAlgorithm->updateObject(*it, (*it)->getAABB());
        }
    }  
}

// Compute the narrow-phase collision detection
void CollisionDetection::computeNarrowPhase() {

    PROFILE("CollisionDetection::computeNarrowPhase()");

    map<bodyindexpair, BroadPhasePair*>::iterator it;
    
    // For each possible collision pair of bodies
    for (it = mOverlappingPairs.begin(); it != mOverlappingPairs.end(); it++) {
        ContactPointInfo* contactInfo = NULL;

        BroadPhasePair* pair = (*it).second;
        assert(pair != NULL);

        CollisionBody* const body1 = pair->body1;
        CollisionBody* const body2 = pair->body2;
        
        // Update the contact cache of the overlapping pair
        mWorld->updateOverlappingPair(pair);
        
        // Select the narrow phase algorithm to use according to the two collision shapes
        NarrowPhaseAlgorithm& narrowPhaseAlgorithm = SelectNarrowPhaseAlgorithm(
                                                        body1->getCollisionShape(),
                                                        body2->getCollisionShape());
        
        // Notify the narrow-phase algorithm about the overlapping pair we are going to test
        narrowPhaseAlgorithm.setCurrentOverlappingPair(pair);
        
        // Use the narrow-phase collision detection algorithm to check
        // if there really is a collision
        if (narrowPhaseAlgorithm.testCollision(body1->getCollisionShape(), body1->getTransform(),
                                               body2->getCollisionShape(), body2->getTransform(),
                                               contactInfo)) {
            assert(contactInfo != NULL);

            // Set the bodies of the contact
            contactInfo->body1 = dynamic_cast<RigidBody*>(body1);
            contactInfo->body2 = dynamic_cast<RigidBody*>(body2);
            assert(contactInfo->body1 != NULL);
            assert(contactInfo->body2 != NULL);

            // Notify the world about the new narrow-phase contact
            mWorld->notifyNewContact(pair, contactInfo);

            // Delete and remove the contact info from the memory allocator
            contactInfo->ContactPointInfo::~ContactPointInfo();
            mMemoryAllocator.release(contactInfo, sizeof(ContactPointInfo));
        }
    }
}

// Allow the broadphase to notify the collision detection about an overlapping pair.
/// This method is called by a broad-phase collision detection algorithm
void CollisionDetection::broadPhaseNotifyAddedOverlappingPair(BodyPair* addedPair) {

    // Get the pair of body index
    bodyindexpair indexPair = addedPair->getBodiesIndexPair();

    // Create the corresponding broad-phase pair object
    BroadPhasePair* broadPhasePair = new (mMemoryAllocator.allocate(sizeof(BroadPhasePair)))
                                             BroadPhasePair(addedPair->body1, addedPair->body2);
    assert(broadPhasePair != NULL);

    // Add the pair into the set of overlapping pairs (if not there yet)
    pair<map<bodyindexpair, BroadPhasePair*>::iterator, bool> check = mOverlappingPairs.insert(
                                                                            make_pair(indexPair,
                                                                            broadPhasePair));
    assert(check.second);

    // Notify the world about the new broad-phase overlapping pair
    mWorld->notifyAddedOverlappingPair(broadPhasePair);
}

// Allow the broadphase to notify the collision detection about a removed overlapping pair
void CollisionDetection::broadPhaseNotifyRemovedOverlappingPair(BodyPair* removedPair) {

    // Get the pair of body index
    bodyindexpair indexPair = removedPair->getBodiesIndexPair();

    // Get the broad-phase pair
    BroadPhasePair* broadPhasePair = mOverlappingPairs.find(indexPair)->second;
    assert(broadPhasePair != NULL);

    // Notify the world about the removed broad-phase pair
    mWorld->notifyRemovedOverlappingPair(broadPhasePair);

    // Remove the overlapping pair from the memory allocator
    broadPhasePair->BroadPhasePair::~BroadPhasePair();
    mMemoryAllocator.release(broadPhasePair, sizeof(BroadPhasePair));
    mOverlappingPairs.erase(indexPair);
}
