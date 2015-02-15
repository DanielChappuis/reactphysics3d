/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2015 Daniel Chappuis                                       *
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
#include "engine/CollisionWorld.h"
#include "body/Body.h"
#include "collision/shapes/BoxShape.h"
#include "body/RigidBody.h"
#include "configuration.h"
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

// Compute the collision detection
void CollisionDetection::testCollisionBetweenShapes(CollisionCallback* callback,
                                                    const std::set<uint>& shapes1,
                                                    const std::set<uint>& shapes2) {

    // Compute the broad-phase collision detection
    computeBroadPhase();

    // Delete all the contact points in the currently overlapping pairs
    clearContactPoints();

    // Compute the narrow-phase collision detection among given sets of shapes
    computeNarrowPhaseBetweenShapes(callback, shapes1, shapes2);
}

// Report collision between two sets of shapes
void CollisionDetection::reportCollisionBetweenShapes(CollisionCallback* callback,
                                                      const std::set<uint>& shapes1,
                                                      const std::set<uint>& shapes2) {

    // For each possible collision pair of bodies
    map<overlappingpairid, OverlappingPair*>::iterator it;
    for (it = mOverlappingPairs.begin(); it != mOverlappingPairs.end(); ++it) {

        OverlappingPair* pair = it->second;

        ProxyShape* shape1 = pair->getShape1();
        ProxyShape* shape2 = pair->getShape2();

        assert(shape1->mBroadPhaseID != shape2->mBroadPhaseID);

        // If both shapes1 and shapes2 sets are non-empty, we check that
        // shape1 is among on set and shape2 is among the other one
        if (!shapes1.empty() && !shapes2.empty() &&
            (shapes1.count(shape1->mBroadPhaseID) == 0 || shapes2.count(shape2->mBroadPhaseID) == 0) &&
            (shapes1.count(shape2->mBroadPhaseID) == 0 || shapes2.count(shape1->mBroadPhaseID) == 0)) {
            continue;
        }
        if (!shapes1.empty() && shapes2.empty() &&
            shapes1.count(shape1->mBroadPhaseID) == 0 && shapes1.count(shape2->mBroadPhaseID) == 0)
        {
            continue;
        }
        if (!shapes2.empty() && shapes1.empty() &&
            shapes2.count(shape1->mBroadPhaseID) == 0 && shapes2.count(shape2->mBroadPhaseID) == 0)
        {
            continue;
        }

        // For each contact manifold of the overlapping pair
        ContactManifold* manifold = pair->getContactManifold();
        for (uint i=0; i<manifold->getNbContactPoints(); i++) {

            ContactPoint* contactPoint = manifold->getContactPoint(i);

            // Create the contact info object for the contact
            ContactPointInfo* contactInfo = new (mWorld->mMemoryAllocator.allocate(sizeof(ContactPointInfo)))
                                         ContactPointInfo(manifold->getShape1(), manifold->getShape2(),
                                                          contactPoint->getNormal(),
                                                          contactPoint->getPenetrationDepth(),
                                                          contactPoint->getLocalPointOnBody1(),
                                                          contactPoint->getLocalPointOnBody2());

            // Notify the collision callback about this new contact
            if (callback != NULL) callback->notifyContact(*contactInfo);

            // Delete and remove the contact info from the memory allocator
            contactInfo->~ContactPointInfo();
            mWorld->mMemoryAllocator.release(contactInfo, sizeof(ContactPointInfo));
        }
    }
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
    for (it = mOverlappingPairs.begin(); it != mOverlappingPairs.end(); ) {
        ContactPointInfo* contactInfo = NULL;

        OverlappingPair* pair = it->second;

        ProxyShape* shape1 = pair->getShape1();
        ProxyShape* shape2 = pair->getShape2();

        assert(shape1->mBroadPhaseID != shape2->mBroadPhaseID);

        // Check if the collision filtering allows collision between the two shapes and
        // that the two shapes are still overlapping. Otherwise, we destroy the
        // overlapping pair
        if (((shape1->getCollideWithMaskBits() & shape2->getCollisionCategoryBits()) == 0 ||
             (shape1->getCollisionCategoryBits() & shape2->getCollideWithMaskBits()) == 0) ||
             !mBroadPhaseAlgorithm.testOverlappingShapes(shape1, shape2)) {

            std::map<overlappingpairid, OverlappingPair*>::iterator itToRemove = it;
            ++it;

            // TODO : Remove all the contact manifold of the overlapping pair from the contact manifolds list of the two bodies involved

            // Destroy the overlapping pair
            itToRemove->second->~OverlappingPair();
            mWorld->mMemoryAllocator.release(itToRemove->second, sizeof(OverlappingPair));
            mOverlappingPairs.erase(itToRemove);
            continue;
        }
        else {
            ++it;
        }

        CollisionBody* const body1 = shape1->getBody();
        CollisionBody* const body2 = shape2->getBody();
        
        // Update the contact cache of the overlapping pair
        pair->update();

        // Check that at least one body is awake and not static
        bool isBody1Active = !body1->isSleeping() && body1->getType() != STATIC;
        bool isBody2Active = !body2->isSleeping() && body2->getType() != STATIC;
        if (!isBody1Active && !isBody2Active) continue;

        // Check if the bodies are in the set of bodies that cannot collide between each other
        bodyindexpair bodiesIndex = OverlappingPair::computeBodiesIndexPair(body1, body2);
        if (mNoCollisionPairs.count(bodiesIndex) > 0) continue;
        
        // Select the narrow phase algorithm to use according to the two collision shapes
        NarrowPhaseAlgorithm& narrowPhaseAlgorithm = selectNarrowPhaseAlgorithm(
                                                        shape1->getCollisionShape(),
                                                        shape2->getCollisionShape());
        
        // Notify the narrow-phase algorithm about the overlapping pair we are going to test
        narrowPhaseAlgorithm.setCurrentOverlappingPair(pair);
        
        // Use the narrow-phase collision detection algorithm to check
        // if there really is a collision
        if (narrowPhaseAlgorithm.testCollision(shape1, shape2, contactInfo)) {
            assert(contactInfo != NULL);

            // If it is the first contact since the pair are overlapping
            if (pair->getNbContactPoints() == 0) {

                // Trigger a callback event
                if (mWorld->mEventListener != NULL) mWorld->mEventListener->beginContact(*contactInfo);
            }

            // Create a new contact
            createContact(pair, contactInfo);

            // Trigger a callback event for the new contact
            if (mWorld->mEventListener != NULL) mWorld->mEventListener->newContact(*contactInfo);

            // Delete and remove the contact info from the memory allocator
            contactInfo->~ContactPointInfo();
            mWorld->mMemoryAllocator.release(contactInfo, sizeof(ContactPointInfo));
        }
    }
}

// Compute the narrow-phase collision detection
void CollisionDetection::computeNarrowPhaseBetweenShapes(CollisionCallback* callback,
                                                         const std::set<uint>& shapes1,
                                                         const std::set<uint>& shapes2) {

    // For each possible collision pair of bodies
    map<overlappingpairid, OverlappingPair*>::iterator it;
    for (it = mOverlappingPairs.begin(); it != mOverlappingPairs.end(); ) {
        ContactPointInfo* contactInfo = NULL;

        OverlappingPair* pair = it->second;

        ProxyShape* shape1 = pair->getShape1();
        ProxyShape* shape2 = pair->getShape2();

        assert(shape1->mBroadPhaseID != shape2->mBroadPhaseID);

        // If both shapes1 and shapes2 sets are non-empty, we check that
        // shape1 is among on set and shape2 is among the other one
        if (!shapes1.empty() && !shapes2.empty() &&
            (shapes1.count(shape1->mBroadPhaseID) == 0 || shapes2.count(shape2->mBroadPhaseID) == 0) &&
            (shapes1.count(shape2->mBroadPhaseID) == 0 || shapes2.count(shape1->mBroadPhaseID) == 0)) {
            ++it;
            continue;
        }
        if (!shapes1.empty() && shapes2.empty() &&
            shapes1.count(shape1->mBroadPhaseID) == 0 && shapes1.count(shape2->mBroadPhaseID) == 0)
        {
            ++it;
            continue;
        }
        if (!shapes2.empty() && shapes1.empty() &&
            shapes2.count(shape1->mBroadPhaseID) == 0 && shapes2.count(shape2->mBroadPhaseID) == 0)
        {
            ++it;
            continue;
        }

        // Check if the collision filtering allows collision between the two shapes and
        // that the two shapes are still overlapping. Otherwise, we destroy the
        // overlapping pair
        if (((shape1->getCollideWithMaskBits() & shape2->getCollisionCategoryBits()) == 0 ||
             (shape1->getCollisionCategoryBits() & shape2->getCollideWithMaskBits()) == 0) ||
             !mBroadPhaseAlgorithm.testOverlappingShapes(shape1, shape2)) {

            std::map<overlappingpairid, OverlappingPair*>::iterator itToRemove = it;
            ++it;

            // TODO : Remove all the contact manifold of the overlapping pair from the contact manifolds list of the two bodies involved

            // Destroy the overlapping pair
            itToRemove->second->~OverlappingPair();
            mWorld->mMemoryAllocator.release(itToRemove->second, sizeof(OverlappingPair));
            mOverlappingPairs.erase(itToRemove);
            continue;
        }
        else {
            ++it;
        }

        CollisionBody* const body1 = shape1->getBody();
        CollisionBody* const body2 = shape2->getBody();

        // Update the contact cache of the overlapping pair
        pair->update();

        // Check if the two bodies are allowed to collide, otherwise, we do not test for collision
        if (body1->getType() != DYNAMIC && body2->getType() != DYNAMIC) continue;
        bodyindexpair bodiesIndex = OverlappingPair::computeBodiesIndexPair(body1, body2);
        if (mNoCollisionPairs.count(bodiesIndex) > 0) continue;

        // Check if the two bodies are sleeping, if so, we do no test collision between them
        if (body1->isSleeping() && body2->isSleeping()) continue;

        // Select the narrow phase algorithm to use according to the two collision shapes
        NarrowPhaseAlgorithm& narrowPhaseAlgorithm = selectNarrowPhaseAlgorithm(
                                                        shape1->getCollisionShape(),
                                                        shape2->getCollisionShape());

        // Notify the narrow-phase algorithm about the overlapping pair we are going to test
        narrowPhaseAlgorithm.setCurrentOverlappingPair(pair);

        // Use the narrow-phase collision detection algorithm to check
        // if there really is a collision
        if (narrowPhaseAlgorithm.testCollision(shape1, shape2, contactInfo)) {
            assert(contactInfo != NULL);

            // Create a new contact
            createContact(pair, contactInfo);

            // Notify the collision callback about this new contact
            if (callback != NULL) callback->notifyContact(*contactInfo);

            // Delete and remove the contact info from the memory allocator
            contactInfo->~ContactPointInfo();
            mWorld->mMemoryAllocator.release(contactInfo, sizeof(ContactPointInfo));
        }
    }
}

// Allow the broadphase to notify the collision detection about an overlapping pair.
/// This method is called by the broad-phase collision detection algorithm
void CollisionDetection::broadPhaseNotifyOverlappingPair(ProxyShape* shape1, ProxyShape* shape2) {

    assert(shape1->mBroadPhaseID != shape2->mBroadPhaseID);

    // If the two proxy collision shapes are from the same body, skip it
    if (shape1->getBody()->getID() == shape2->getBody()->getID()) return;

    // Check if the collision filtering allows collision between the two shapes
    if ((shape1->getCollideWithMaskBits() & shape2->getCollisionCategoryBits()) == 0 ||
        (shape1->getCollisionCategoryBits() & shape2->getCollideWithMaskBits()) == 0) return;

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

    // Wake up the two bodies
    shape1->getBody()->setIsSleeping(false);
    shape2->getBody()->setIsSleeping(false);
}

// Remove a body from the collision detection
void CollisionDetection::removeProxyCollisionShape(ProxyShape* proxyShape) {

    // Remove all the overlapping pairs involving this proxy shape
    std::map<overlappingpairid, OverlappingPair*>::iterator it;
    for (it = mOverlappingPairs.begin(); it != mOverlappingPairs.end(); ) {
        if (it->second->getShape1()->mBroadPhaseID == proxyShape->mBroadPhaseID||
            it->second->getShape2()->mBroadPhaseID == proxyShape->mBroadPhaseID) {
            std::map<overlappingpairid, OverlappingPair*>::iterator itToRemove = it;
            ++it;

            // TODO : Remove all the contact manifold of the overlapping pair from the contact manifolds list of the two bodies involved

            // Destroy the overlapping pair
            itToRemove->second->~OverlappingPair();
            mWorld->mMemoryAllocator.release(itToRemove->second, sizeof(OverlappingPair));
            mOverlappingPairs.erase(itToRemove);
        }
        else {
            ++it;
        }
    }

    // Remove the body from the broad-phase
    mBroadPhaseAlgorithm.removeProxyCollisionShape(proxyShape);
}

// Create a new contact
void CollisionDetection::createContact(OverlappingPair* overlappingPair,
                                       const ContactPointInfo* contactInfo) {

    // Create a new contact
    ContactPoint* contact = new (mWorld->mMemoryAllocator.allocate(sizeof(ContactPoint)))
                                ContactPoint(*contactInfo);
    assert(contact != NULL);

    // Add the contact to the contact cache of the corresponding overlapping pair
    overlappingPair->addContact(contact);

    // Add the contact manifold into the list of contact manifolds
    // of the two bodies involved in the contact
    addContactManifoldToBody(overlappingPair->getContactManifold(),
                             overlappingPair->getShape1()->getBody(),
                             overlappingPair->getShape2()->getBody());
}

// Add a contact manifold to the linked list of contact manifolds of the two bodies involved
// in the corresponding contact
void CollisionDetection::addContactManifoldToBody(ContactManifold* contactManifold,
                                                  CollisionBody* body1, CollisionBody* body2) {

    assert(contactManifold != NULL);

    // Add the contact manifold at the beginning of the linked
    // list of contact manifolds of the first body
    void* allocatedMemory1 = mWorld->mMemoryAllocator.allocate(sizeof(ContactManifoldListElement));
    ContactManifoldListElement* listElement1 = new (allocatedMemory1)
                                                  ContactManifoldListElement(contactManifold,
                                                                     body1->mContactManifoldsList);
    body1->mContactManifoldsList = listElement1;

    // Add the contact manifold at the beginning of the linked
    // list of the contact manifolds of the second body
    void* allocatedMemory2 = mWorld->mMemoryAllocator.allocate(sizeof(ContactManifoldListElement));
    ContactManifoldListElement* listElement2 = new (allocatedMemory2)
                                                  ContactManifoldListElement(contactManifold,
                                                                     body2->mContactManifoldsList);
    body2->mContactManifoldsList = listElement2;
}

// Delete all the contact points in the currently overlapping pairs
void CollisionDetection::clearContactPoints() {

    // For each overlapping pair
    std::map<overlappingpairid, OverlappingPair*>::iterator it;
    for (it = mOverlappingPairs.begin(); it != mOverlappingPairs.end(); ++it) {
        it->second->clearContactPoints();
    }
}
