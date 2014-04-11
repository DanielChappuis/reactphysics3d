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
                   : mWorld(world), mBroadPhaseAlgorithm(*this),
                     mNarrowPhaseGJKAlgorithm(memoryAllocator),
                     mNarrowPhaseSphereVsSphereAlgorithm(memoryAllocator),
                     mIsCollisionShapesAdded(false) {

}

// Destructor
CollisionDetection::~CollisionDetection() {
    
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

    // If new collision shapes have been added to bodies
    if (mIsCollisionShapesAdded) {

        // Ask the broad-phase to recompute the overlapping pairs of collision
        // shapes. This call can only add new overlapping pairs in the collision
        // detection.
        mBroadPhaseAlgorithm.computeOverlappingPairs();
    }
}

// Compute the narrow-phase collision detection
void CollisionDetection::computeNarrowPhase() {

    PROFILE("CollisionDetection::computeNarrowPhase()");
    
    // For each possible collision pair of bodies
    map<overlappingpairid, OverlappingPair*>::iterator it;
    for (it = mOverlappingPairs.begin(); it != mOverlappingPairs.end(); it++) {
        ContactPointInfo* contactInfo = NULL;

        OverlappingPair* pair = it->second;

        ProxyShape* shape1 = pair->getShape1();
        ProxyShape* shape2 = pair->getShape2();
        CollisionBody* const body1 = shape1->getBody();
        CollisionBody* const body2 = shape2->getBody();
        
        // Update the contact cache of the overlapping pair
        mWorld->updateOverlappingPair(pair);

        // Check if the two bodies are allowed to collide, otherwise, we do not test for collision
        if (body1->getType() != DYNAMIC && body2->getType() != DYNAMIC) continue;
        bodyindexpair bodiesIndex = OverlappingPair::computeBodiesIndexPair(body1, body2);
        if (mNoCollisionPairs.count(bodiesIndex) > 0) continue;

        // Check if the two bodies are sleeping, if so, we do no test collision between them
        if (body1->isSleeping() && body2->isSleeping()) continue;
        
        // Select the narrow phase algorithm to use according to the two collision shapes
        NarrowPhaseAlgorithm& narrowPhaseAlgorithm = SelectNarrowPhaseAlgorithm(
                                                        shape1->getCollisionShape(),
                                                        shape2->getCollisionShape());
        
        // Notify the narrow-phase algorithm about the overlapping pair we are going to test
        narrowPhaseAlgorithm.setCurrentOverlappingPair(pair);
        
        // Use the narrow-phase collision detection algorithm to check
        // if there really is a collision
        const Transform transform1 = body1->getTransform() * shape1->getLocalToBodyTransform();
        const Transform transform2 = body2->getTransform() * shape2->getLocalToBodyTransform();
        if (narrowPhaseAlgorithm.testCollision(shape1->getCollisionShape(), transform1,
                                               shape2->getCollisionShape(), transform2,
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
            mWorld->mMemoryAllocator.release(contactInfo, sizeof(ContactPointInfo));
        }
    }
}

// Allow the broadphase to notify the collision detection about an overlapping pair.
/// This method is called by a broad-phase collision detection algorithm
void CollisionDetection::broadPhaseNotifyOverlappingPair(ProxyShape* shape1, ProxyShape* shape2) {

    // Compute the overlapping pair ID
    overlappingpairid pairID = OverlappingPair::computeID(shape1, shape2);

    // Check if the overlapping pair already exists
    if (mOverlappingPairs.find(pairID) != mOverlappingPairs.end()) return;

    // Create the overlapping pair and add it into the set of overlapping pairs
    OverlappingPair* newPair = new (mWorld->mMemoryAllocator.allocate(sizeof(OverlappingPair)))
                              OverlappingPair(shape1, shape2, mWorld->mMemoryAllocator);
    assert(newPair != NULL);
    std::pair<map<overlappingpairid, OverlappingPair*>::iterator, bool> check =
            mOverlappingPairs.insert(make_pair(pairID, newPair));
    assert(check.second);

    /* TODO : DELETE THIS
    // Get the pair of body index
    bodyindexpair indexPair = addedPair->getBodiesIndexPair();

    // If the overlapping pair already exists, we don't do anything
    if (mOverlappingPairs.count(indexPair) > 0) return;

    // Create the corresponding broad-phase pair object
    BroadPhasePair* broadPhasePair = new (mWorld->mMemoryAllocator.allocate(sizeof(BroadPhasePair)))
                                             BroadPhasePair(addedPair->body1, addedPair->body2);
    assert(broadPhasePair != NULL);

    // Add the pair into the set of overlapping pairs (if not there yet)
    pair<map<bodyindexpair, BroadPhasePair*>::iterator, bool> check = mOverlappingPairs.insert(
                                                                            make_pair(indexPair,
                                                                            broadPhasePair));
    assert(check.second);

    // Notify the world about the new broad-phase overlapping pair
    mWorld->notifyAddedOverlappingPair(broadPhasePair);
    */
}

// Allow the broadphase to notify the collision detection about a removed overlapping pair
void CollisionDetection::removeOverlappingPair(ProxyShape* shape1, ProxyShape* shape2) {

    // Compute the overlapping pair ID
    overlappingpairid pairID = OverlappingPair::computeID(shape1, shape2);

    // If the overlapping
    std::map<overlappingpairid, OverlappingPair*>::iterator it;
    it = mOverlappingPairs.find(indexPair);
    if ()

    // Notify the world about the removed broad-phase pair
    // TODO : DELETE THIS
    //mWorld->notifyRemovedOverlappingPair(broadPhasePair);

    // Remove the overlapping pair from the memory allocator
    mOverlappingPairs.find(pairID)->second->OverlappingPair::~OverlappingPair();
    mWorld->mMemoryAllocator.release(mOverlappingPairs[indexPair], sizeof(OverlappingPair));
    mOverlappingPairs.erase(pairID);
}

// Remove a body from the collision detection
void CollisionDetection::removeProxyCollisionShape(ProxyShape* proxyShape) {

    // Remove all the overlapping pairs involving this proxy shape
    std::map<overlappingpairid, OverlappingPair*>::iterator it;
    for (it = mOverlappingPairs.begin(); it != mOverlappingPairs.end(); ) {
        if (it->second->getShape1()->getBroadPhaseID() == proxyShape->getBroadPhaseID() ||
            it->second->getShape2()->getBroadPhaseID() == proxyShape->getBroadPhaseID()) {
            std::map<overlappingpairid, OverlappingPair*>::iterator itToRemove = it;
            ++it;
            mOverlappingPairs.erase(itToRemove);
        }
        else {
            ++it;
        }
    }

    // Remove the body from the broad-phase
    mBroadPhaseAlgorithm.removeProxyCollisionShape(proxyShape);
}
