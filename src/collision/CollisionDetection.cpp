/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2010 Daniel Chappuis                                            *
*********************************************************************************
*                                                                               *
* Permission is hereby granted, free of charge, to any person obtaining a copy  *
* of this software and associated documentation files (the "Software"), to deal *
* in the Software without restriction, including without limitation the rights  *
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell     *
* copies of the Software, and to permit persons to whom the Software is         *
* furnished to do so, subject to the following conditions:                      *
*                                                                               *
* The above copyright notice and this permission notice shall be included in    *
* all copies or substantial portions of the Software.                           *
*                                                                               *
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR    *
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,      *
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE   *
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER        *
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, *
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN     *
* THE SOFTWARE.                                                                 *
********************************************************************************/

// Libraries
#include "CollisionDetection.h"
#include "SAPAlgorithm.h"
#include "GJK/GJKAlgorithm.h"
#include "SATAlgorithm.h"
#include "../body/Body.h"
#include "../shapes/BoxShape.h"
#include "../body/RigidBody.h"
#include "../constants.h"
#include <cassert>
#include <complex>
#include <set>
#include <utility>

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;
using namespace std;

// Constructor
CollisionDetection::CollisionDetection(PhysicsWorld* world)
                   : world(world), memoryPoolContacts(NB_MAX_CONTACTS), memoryPoolOverlappingPairs(NB_MAX_COLLISION_PAIRS) {

    // Create the broad-phase algorithm that will be used (Sweep and Prune with AABB)
    broadPhaseAlgorithm = new SAPAlgorithm(*this);

    // Create the narrow-phase algorithm that will be used (Separating axis algorithm)
    narrowPhaseAlgorithm = new GJKAlgorithm(*this);
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
    broadPhaseAlgorithm->notifyAddedBodies(world->getAddedBodies());
    broadPhaseAlgorithm->notifyRemovedBodies(world->getRemovedBodies());
    
    // Clear the set of the overlapping pairs in the current step
    currentStepOverlappingPairs.clear();

    // Execute the broad-phase collision algorithm in order to compute the overlapping pairs of bodies
    broadPhaseAlgorithm->computePossibleCollisionPairs();
    
    // At this point, the pairs in the set lastStepOverlappingPairs contains
    // only the pairs that are not overlapping anymore. Therefore, we can
    // remove them from the overlapping pairs map
    for (set<pair<luint, luint> >::iterator it = lastStepOverlappingPairs.begin(); it != lastStepOverlappingPairs.end(); it++) {
        // Remove the overlapping pair from the memory pool
		overlappingPairs.at((*it))->OverlappingPair::~OverlappingPair();
		memoryPoolOverlappingPairs.freeObject(overlappingPairs.at((*it)));
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
        
        // Use the narrow-phase collision detection algorithm to check if there really are a contact
        if (narrowPhaseAlgorithm->testCollision(body1->getShape(), body1->getTransform(),
                                                body2->getShape(), body2->getTransform(), contactInfo)) {
            assert(contactInfo);
            collisionExists = true;

            // Create a new contact
            Contact* contact = new(memoryPoolContacts.allocateObject()) Contact(contactInfo);
            
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
    // Construct the pair of index
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
