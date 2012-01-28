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
#include "broadphase/SweepAndPruneAlgorithm.h"
#include "../body/Body.h"
#include "../colliders/BoxCollider.h"
#include "../body/RigidBody.h"
#include "../configuration.h"
#include <cassert>
#include <complex>
#include <set>
#include <utility>

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;
using namespace std;

// Constructor
CollisionDetection::CollisionDetection(PhysicsWorld* world)
                   : world(world), memoryPoolContacts(NB_MAX_CONTACTS), memoryPoolOverlappingPairs(NB_MAX_COLLISION_PAIRS),
                     memoryPoolContactInfos(NB_MAX_CONTACTS), narrowPhaseGJKAlgorithm(*this, memoryPoolContactInfos),
                     narrowPhaseSphereVsSphereAlgorithm(*this, memoryPoolContactInfos) {

    // Create the broad-phase algorithm that will be used (Sweep and Prune with AABB)
    broadPhaseAlgorithm = new SweepAndPruneAlgorithm(*this);
}

// Destructor
CollisionDetection::~CollisionDetection() {
    // Delete the remaining overlapping pairs
    for (map<std::pair<luint, luint>, OverlappingPair*>::iterator it=overlappingPairs.begin(); it != overlappingPairs.end(); it++) {
        // Delete the overlapping pair
        (*it).second->OverlappingPair::~OverlappingPair();
		memoryPoolOverlappingPairs.freeObject((*it).second);
    }
}

// Compute the collision detection
bool CollisionDetection::computeCollisionDetection() {
	
    world->removeAllContactConstraints();

    // Compute the broad-phase collision detection
    computeBroadPhase();

    // Compute the narrow-phase collision detection
    bool collisionExists = computeNarrowPhase();
	
    // Return true if at least one contact has been found
    return collisionExists;
}

// Compute the broad-phase collision detection
void CollisionDetection::computeBroadPhase() {

    // Notify the broad-phase algorithm about new and removed bodies in the physics world
    broadPhaseAlgorithm->notifyAddedBodies(world->getAddedRigidBodies());
    broadPhaseAlgorithm->notifyRemovedBodies(world->getRemovedRigidBodies());
    
    // Clear the set of the overlapping pairs in the current step
    currentStepOverlappingPairs.clear();

    // Execute the broad-phase collision algorithm in order to compute the overlapping pairs of bodies
    broadPhaseAlgorithm->computePossibleCollisionPairs();
    
    // At this point, the pairs in the set lastStepOverlappingPairs contains
    // only the pairs that are not overlapping anymore. Therefore, we can
    // remove them from the overlapping pairs map
    for (set<pair<luint, luint> >::iterator it = lastStepOverlappingPairs.begin(); it != lastStepOverlappingPairs.end(); it++) {
        // Remove the overlapping pair from the memory pool
		overlappingPairs[(*it)]->OverlappingPair::~OverlappingPair();
		memoryPoolOverlappingPairs.freeObject(overlappingPairs[(*it)]);
        overlappingPairs.erase(*it);
    }
    
    // The current overlapping pairs become the last step overlapping pairs
    lastStepOverlappingPairs = currentStepOverlappingPairs;
}

// Compute the narrow-phase collision detection
bool CollisionDetection::computeNarrowPhase() {
    bool collisionExists = false;
    map<std::pair<luint, luint>, OverlappingPair*>::iterator it;
    
    // For each possible collision pair of bodies
    for (it = overlappingPairs.begin(); it != overlappingPairs.end(); it++) {
        ContactInfo* contactInfo = NULL;

        Body* const body1 = (*it).second->getBody1();
        Body* const body2 = (*it).second->getBody2();
        
        // Update the contact cache of the overlapping pair
        (*it).second->update();
        
        // Select the narrow phase algorithm to use according to the two colliders
        NarrowPhaseAlgorithm& narrowPhaseAlgorithm = SelectNarrowPhaseAlgorithm(body1->getCollider(), body2->getCollider());
        
        // Notify the narrow-phase algorithm about the overlapping pair we are going to test
        narrowPhaseAlgorithm.setCurrentOverlappingPair((*it).second);
        
        // Use the narrow-phase collision detection algorithm to check if there really is a collision
        if (narrowPhaseAlgorithm.testCollision(body1->getCollider(), body1->getTransform(),
                                               body2->getCollider(), body2->getTransform(), contactInfo)) {
            assert(contactInfo);
            collisionExists = true;

            // Create a new contact
            Contact* contact = new(memoryPoolContacts.allocateObject()) Contact(body1, body2, contactInfo);
            
            // Delete and remove the contact info from the memory pool
            contactInfo->ContactInfo::~ContactInfo();
            memoryPoolContactInfos.freeObject(contactInfo);
            
            // Add the contact to the contact cache of the corresponding overlapping pair
            (*it).second->addContact(contact);
            
            // Add all the contacts in the contact cache of the two bodies
            // to the set of constraints in the physics world
            for (uint i=0; i<(*it).second->getNbContacts(); i++) {
                world->addConstraint((*it).second->getContact(i));
            }
        }
    }
    
    return collisionExists;
}


// Allow the broadphase to notify the collision detection about an overlapping pair
// This method is called by a broad-phase collision detection algorithm
void CollisionDetection::broadPhaseNotifyOverlappingPair(Body* body1, Body* body2) {
    // Construct the pair of body index
    pair<luint, luint> indexPair = body1->getID() < body2->getID() ? make_pair(body1->getID(), body2->getID()) :
                                                                make_pair(body2->getID(), body1->getID());
    assert(indexPair.first != indexPair.second);
    
    // Add the pair to the overlapping pairs of the current step
    currentStepOverlappingPairs.insert(indexPair);
    
    // Remove the pair from the set of overlapping pairs in the last step
    // if the pair of bodies were already overlapping (used to compute the 
    // set of pair that were overlapping in the last step but are not overlapping
    // in the current one anymore
    lastStepOverlappingPairs.erase(indexPair);
    
    // Add the pair into the set of overlapping pairs (if not there yet)
	OverlappingPair* newPair = new (memoryPoolOverlappingPairs.allocateObject()) OverlappingPair(body1, body2, memoryPoolContacts);
    pair<map<pair<luint, luint>, OverlappingPair*>::iterator, bool> check = overlappingPairs.insert(make_pair(indexPair, newPair));
	
	// If the overlapping pair was already in the set of overlapping pair
	if (!check.second) {
		// Delete the new pair
		newPair->OverlappingPair::~OverlappingPair();
		memoryPoolOverlappingPairs.freeObject(newPair);
	}
}
