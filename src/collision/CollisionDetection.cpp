/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2018 Daniel Chappuis                                       *
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
#include "collision/OverlapCallback.h"
#include "body/Body.h"
#include "collision/shapes/BoxShape.h"
#include "collision/shapes/ConcaveShape.h"
#include "body/RigidBody.h"
#include "configuration.h"
#include "collision/CollisionCallback.h"
#include "collision/MiddlePhaseTriangleCallback.h"
#include "collision/OverlapCallback.h"
#include "collision/narrowphase/NarrowPhaseInfoBatch.h"
#include "collision/ContactManifold.h"
#include "utils/Profiler.h"
#include "engine/EventListener.h"
#include "collision/RaycastInfo.h"
#include <cassert>

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;
using namespace std;


// Constructor
CollisionDetection::CollisionDetection(CollisionWorld* world, ProxyShapesComponents& proxyShapesComponents, MemoryManager& memoryManager)
                   : mMemoryManager(memoryManager), mProxyShapesComponents(proxyShapesComponents), mCollisionDispatch(mMemoryManager.getPoolAllocator()), mWorld(world),
                     mOverlappingPairs(mMemoryManager.getPoolAllocator()), mBroadPhaseSystem(*this, mProxyShapesComponents),
                     mNoCollisionPairs(mMemoryManager.getPoolAllocator()), mIsCollisionShapesAdded(false),
                     mNarrowPhaseInput(mMemoryManager.getSingleFrameAllocator()) {

#ifdef IS_PROFILING_ACTIVE

	mProfiler = nullptr;
    mCollisionDispatch.setProfiler(mProfiler);

#endif

}

// Compute the collision detection
void CollisionDetection::computeCollisionDetection() {

    RP3D_PROFILE("CollisionDetection::computeCollisionDetection()", mProfiler);
	    
    // Compute the broad-phase collision detection
    computeBroadPhase();

    // Compute the middle-phase collision detection
    computeMiddlePhase();
    
    // Compute the narrow-phase collision detection
    computeNarrowPhase();
}

// Compute the broad-phase collision detection
void CollisionDetection::computeBroadPhase() {

    RP3D_PROFILE("CollisionDetection::computeBroadPhase()", mProfiler);

    // If new collision shapes have been added to bodies
    if (mIsCollisionShapesAdded) {

        // Ask the broad-phase to recompute the overlapping pairs of collision
        // shapes. This call can only add new overlapping pairs in the collision
        // detection.
        mBroadPhaseSystem.computeOverlappingPairs(mMemoryManager);
    }
}

// Compute the middle-phase collision detection
void CollisionDetection::computeMiddlePhase() {

    RP3D_PROFILE("CollisionDetection::computeMiddlePhase()", mProfiler);

    // Reserve memory for the narrow-phase input using cached capacity from previous frame
    mNarrowPhaseInput.reserveMemory();

    // For each possible collision pair of bodies
    for (auto it = mOverlappingPairs.begin(); it != mOverlappingPairs.end(); ) {

        OverlappingPair* pair = it->second;

        // Make all the contact manifolds and contact points of the pair obsolete
        pair->makeContactsObsolete();

        // Make all the last frame collision info obsolete
        pair->makeLastFrameCollisionInfosObsolete();

        ProxyShape* shape1 = pair->getShape1();
        ProxyShape* shape2 = pair->getShape2();

        assert(shape1->getBroadPhaseId() != -1);
        assert(shape2->getBroadPhaseId() != -1);
        assert(shape1->getBroadPhaseId() != shape2->getBroadPhaseId());

        // Check if the two shapes are still overlapping. Otherwise, we destroy the
        // overlapping pair
        if (!mBroadPhaseSystem.testOverlappingShapes(shape1, shape2)) {

            // Destroy the overlapping pair
            pair->~OverlappingPair();

            mWorld->mMemoryManager.release(MemoryManager::AllocationType::Pool, pair, sizeof(OverlappingPair));
            it = mOverlappingPairs.remove(it);
            continue;
        }
        else {
            ++it;
        }

        // Check if the collision filtering allows collision between the two shapes
        if (((shape1->getCollideWithMaskBits() & shape2->getCollisionCategoryBits()) != 0 &&
             (shape1->getCollisionCategoryBits() & shape2->getCollideWithMaskBits()) != 0)) {

            CollisionBody* const body1 = shape1->getBody();
            CollisionBody* const body2 = shape2->getBody();

            // Check that at least one body is awake and not static
            bool isBody1Active = !body1->isSleeping() && body1->getType() != BodyType::STATIC;
            bool isBody2Active = !body2->isSleeping() && body2->getType() != BodyType::STATIC;
            if (!isBody1Active && !isBody2Active) continue;

            // Check if the bodies are in the set of bodies that cannot collide between each other
            bodyindexpair bodiesIndex = OverlappingPair::computeBodiesIndexPair(body1, body2);
            if (mNoCollisionPairs.contains(bodiesIndex) > 0) continue;

			bool isShape1Convex = shape1->getCollisionShape()->isConvex();
			bool isShape2Convex = shape2->getCollisionShape()->isConvex();

            // If both shapes are convex
            if (isShape1Convex && isShape2Convex) {

                // Select the narrow phase algorithm to use according to the two collision shapes
                NarrowPhaseAlgorithmType algorithmType = mCollisionDispatch.selectNarrowPhaseAlgorithm(shape1->getCollisionShape()->getType(),
                                                                                                       shape2->getCollisionShape()->getType());

                // No middle-phase is necessary, simply create a narrow phase info
                // for the narrow-phase collision detection
                mNarrowPhaseInput.addNarrowPhaseTest(pair, shape1->getCollisionShape(), shape2->getCollisionShape(),
                                                         shape1->getLocalToWorldTransform(), shape2->getLocalToWorldTransform(),
                                                         algorithmType, mMemoryManager.getSingleFrameAllocator());

            }
            // Concave vs Convex algorithm
            else if ((!isShape1Convex && isShape2Convex) || (!isShape2Convex && isShape1Convex)) {

                computeConvexVsConcaveMiddlePhase(pair, mMemoryManager.getSingleFrameAllocator(), mNarrowPhaseInput);
            }
            // Concave vs Concave shape
            else {
                // Not handled
                continue;
            }

            // Remove the obsolete last frame collision infos
            pair->clearObsoleteLastFrameCollisionInfos();
        }
    }
}

// Compute the concave vs convex middle-phase algorithm for a given pair of bodies
void CollisionDetection::computeConvexVsConcaveMiddlePhase(OverlappingPair* pair, MemoryAllocator& allocator,
                                                           NarrowPhaseInput& narrowPhaseInput) {

    ProxyShape* shape1 = pair->getShape1();
    ProxyShape* shape2 = pair->getShape2();

    ProxyShape* convexProxyShape;
    ProxyShape* concaveProxyShape;
    const ConvexShape* convexShape;
    const ConcaveShape* concaveShape;

    // Collision shape 1 is convex, collision shape 2 is concave
    if (shape1->getCollisionShape()->isConvex()) {
        convexProxyShape = shape1;
        convexShape = static_cast<const ConvexShape*>(shape1->getCollisionShape());
        concaveProxyShape = shape2;
        concaveShape = static_cast<const ConcaveShape*>(shape2->getCollisionShape());
    }
    else {  // Collision shape 2 is convex, collision shape 1 is concave
        convexProxyShape = shape2;
        convexShape = static_cast<const ConvexShape*>(shape2->getCollisionShape());
        concaveProxyShape = shape1;
        concaveShape = static_cast<const ConcaveShape*>(shape1->getCollisionShape());
    }

    // Select the narrow phase algorithm to use according to the two collision shapes
    NarrowPhaseAlgorithmType algorithmType = mCollisionDispatch.selectNarrowPhaseAlgorithm(convexShape->getType(),
                                                                                           CollisionShapeType::CONVEX_POLYHEDRON);

    assert(algorithmType != NarrowPhaseAlgorithmType::None);

    // Set the parameters of the callback object
    MiddlePhaseTriangleCallback middlePhaseCallback(pair, concaveProxyShape, convexProxyShape,
                                                    concaveShape, narrowPhaseInput, algorithmType, allocator);

#ifdef IS_PROFILING_ACTIVE

	// Set the profiler
	middlePhaseCallback.setProfiler(mProfiler);

#endif

    // Compute the convex shape AABB in the local-space of the convex shape
    const Transform convexToConcaveTransform = concaveProxyShape->getLocalToWorldTransform().getInverse() *
                                               convexProxyShape->getLocalToWorldTransform();
    AABB aabb;
    convexShape->computeAABB(aabb, convexToConcaveTransform);

    // Call the convex vs triangle callback for each triangle of the concave shape
    concaveShape->testAllTriangles(middlePhaseCallback, aabb);
}

// Execute the narrow-phase collision detection algorithm on batches
bool CollisionDetection::testNarrowPhaseCollision(NarrowPhaseInput& narrowPhaseInput, bool stopFirstContactFound,
                                                  bool reportContacts, MemoryAllocator& allocator) {

    bool contactFound = false;

    // Get the narrow-phase collision detection algorithms for each kind of collision shapes
    SphereVsSphereAlgorithm* sphereVsSphereAlgo = mCollisionDispatch.getSphereVsSphereAlgorithm();
    SphereVsCapsuleAlgorithm* sphereVsCapsuleAlgo = mCollisionDispatch.getSphereVsCapsuleAlgorithm();
    CapsuleVsCapsuleAlgorithm* capsuleVsCapsuleAlgo = mCollisionDispatch.getCapsuleVsCapsuleAlgorithm();
    SphereVsConvexPolyhedronAlgorithm* sphereVsConvexPolyAlgo = mCollisionDispatch.getSphereVsConvexPolyhedronAlgorithm();
    CapsuleVsConvexPolyhedronAlgorithm* capsuleVsConvexPolyAlgo = mCollisionDispatch.getCapsuleVsConvexPolyhedronAlgorithm();
    ConvexPolyhedronVsConvexPolyhedronAlgorithm* convexPolyVsConvexPolyAlgo = mCollisionDispatch.getConvexPolyhedronVsConvexPolyhedronAlgorithm();

    // get the narrow-phase batches to test for collision
    SphereVsSphereNarrowPhaseInfoBatch& sphereVsSphereBatch = narrowPhaseInput.getSphereVsSphereBatch();
    SphereVsCapsuleNarrowPhaseInfoBatch& sphereVsCapsuleBatch = narrowPhaseInput.getSphereVsCapsuleBatch();
    CapsuleVsCapsuleNarrowPhaseInfoBatch& capsuleVsCapsuleBatch = narrowPhaseInput.getCapsuleVsCapsuleBatch();
    NarrowPhaseInfoBatch& sphereVsConvexPolyhedronBatch = narrowPhaseInput.getSphereVsConvexPolyhedronBatch();
    NarrowPhaseInfoBatch& capsuleVsConvexPolyhedronBatch = narrowPhaseInput.getCapsuleVsConvexPolyhedronBatch();
    NarrowPhaseInfoBatch& convexPolyhedronVsConvexPolyhedronBatch = narrowPhaseInput.getConvexPolyhedronVsConvexPolyhedronBatch();

    // Compute the narrow-phase collision detection for each kind of collision shapes
    if (sphereVsSphereBatch.getNbObjects() > 0) {
        contactFound |= sphereVsSphereAlgo->testCollision(sphereVsSphereBatch, 0, sphereVsSphereBatch.getNbObjects(), reportContacts, stopFirstContactFound, allocator);
        if (stopFirstContactFound && contactFound) return true;
    }
    if (sphereVsCapsuleBatch.getNbObjects() > 0) {
        contactFound |= sphereVsCapsuleAlgo->testCollision(sphereVsCapsuleBatch, 0, sphereVsCapsuleBatch.getNbObjects(), reportContacts, stopFirstContactFound, allocator);
        if (stopFirstContactFound && contactFound) return true;
    }
    if (capsuleVsCapsuleBatch.getNbObjects() > 0) {
        contactFound |= capsuleVsCapsuleAlgo->testCollision(capsuleVsCapsuleBatch, 0, capsuleVsCapsuleBatch.getNbObjects(), reportContacts, stopFirstContactFound, allocator);
        if (stopFirstContactFound && contactFound) return true;
    }
    if (sphereVsConvexPolyhedronBatch.getNbObjects() > 0) {
        contactFound |= sphereVsConvexPolyAlgo->testCollision(sphereVsConvexPolyhedronBatch, 0, sphereVsConvexPolyhedronBatch.getNbObjects(), reportContacts, stopFirstContactFound, allocator);
        if (stopFirstContactFound && contactFound) return true;
    }
    if (capsuleVsConvexPolyhedronBatch.getNbObjects() > 0) {
        contactFound |= capsuleVsConvexPolyAlgo->testCollision(capsuleVsConvexPolyhedronBatch, 0, capsuleVsConvexPolyhedronBatch.getNbObjects(), reportContacts, stopFirstContactFound, allocator);
        if (stopFirstContactFound && contactFound) return true;
    }
    if (convexPolyhedronVsConvexPolyhedronBatch.getNbObjects() > 0) {
        contactFound |= convexPolyVsConvexPolyAlgo->testCollision(convexPolyhedronVsConvexPolyhedronBatch, 0, convexPolyhedronVsConvexPolyhedronBatch.getNbObjects(), reportContacts, stopFirstContactFound, allocator);
        if (stopFirstContactFound && contactFound) return true;
    }

    return contactFound;
}

// Process the potential contacts after narrow-phase collision detection
void CollisionDetection::processAllPotentialContacts(NarrowPhaseInput& narrowPhaseInput, bool updateLastFrameInfo) {

    // get the narrow-phase batches to test for collision
    NarrowPhaseInfoBatch& sphereVsSphereBatch = narrowPhaseInput.getSphereVsSphereBatch();
    NarrowPhaseInfoBatch& sphereVsCapsuleBatch = narrowPhaseInput.getSphereVsCapsuleBatch();
    NarrowPhaseInfoBatch& capsuleVsCapsuleBatch = narrowPhaseInput.getCapsuleVsCapsuleBatch();
    NarrowPhaseInfoBatch& sphereVsConvexPolyhedronBatch = narrowPhaseInput.getSphereVsConvexPolyhedronBatch();
    NarrowPhaseInfoBatch& capsuleVsConvexPolyhedronBatch = narrowPhaseInput.getCapsuleVsConvexPolyhedronBatch();
    NarrowPhaseInfoBatch& convexPolyhedronVsConvexPolyhedronBatch = narrowPhaseInput.getConvexPolyhedronVsConvexPolyhedronBatch();

    // Process the potential contacts
    processPotentialContacts(sphereVsSphereBatch, updateLastFrameInfo);
    processPotentialContacts(sphereVsCapsuleBatch, updateLastFrameInfo);
    processPotentialContacts(capsuleVsCapsuleBatch, updateLastFrameInfo);
    processPotentialContacts(sphereVsConvexPolyhedronBatch, updateLastFrameInfo);
    processPotentialContacts(capsuleVsConvexPolyhedronBatch, updateLastFrameInfo);
    processPotentialContacts(convexPolyhedronVsConvexPolyhedronBatch, updateLastFrameInfo);
}

// Compute the narrow-phase collision detection
void CollisionDetection::computeNarrowPhase() {

    RP3D_PROFILE("CollisionDetection::computeNarrowPhase()", mProfiler);

    MemoryAllocator& allocator = mMemoryManager.getSingleFrameAllocator();

    // Test the narrow-phase collision detection on the batches to be tested
    testNarrowPhaseCollision(mNarrowPhaseInput, false, true, allocator);

    // Process all the potential contacts after narrow-phase collision
    processAllPotentialContacts(mNarrowPhaseInput, true);

    // Reduce the number of contact points in the manifolds
    reduceContactManifolds(mOverlappingPairs);

    // Add all the contact manifolds (between colliding bodies) to the bodies
    addAllContactManifoldsToBodies();

    // Report contacts to the user
    reportAllContacts();

    // Clear the list of narrow-phase infos
    mNarrowPhaseInput.clear();
}

// Allow the broadphase to notify the collision detection about an overlapping pair.
/// This method is called by the broad-phase collision detection algorithm
void CollisionDetection::broadPhaseNotifyOverlappingPair(ProxyShape* shape1, ProxyShape* shape2) {

    assert(shape1->getBroadPhaseId() != -1);
    assert(shape2->getBroadPhaseId() != -1);
    assert(shape1->getBroadPhaseId() != shape2->getBroadPhaseId());

    // Check if the collision filtering allows collision between the two shapes
    if ((shape1->getCollideWithMaskBits() & shape2->getCollisionCategoryBits()) == 0 ||
        (shape1->getCollisionCategoryBits() & shape2->getCollideWithMaskBits()) == 0) return;

    // Compute the overlapping pair ID
    Pair<uint, uint> pairID = OverlappingPair::computeID(shape1, shape2);

    // Check if the overlapping pair already exists
    if (mOverlappingPairs.containsKey(pairID)) return;

    // Create the overlapping pair and add it into the set of overlapping pairs
    OverlappingPair* newPair = new (mMemoryManager.allocate(MemoryManager::AllocationType::Pool, sizeof(OverlappingPair)))
                              OverlappingPair(shape1, shape2, mMemoryManager.getPoolAllocator(),
                                              mMemoryManager.getSingleFrameAllocator(), mWorld->mConfig);

    assert(newPair != nullptr);

    // Add the new overlapping pair
    mOverlappingPairs.add(Pair<Pair<uint, uint>, OverlappingPair*>(pairID, newPair));

    // Wake up the two bodies
    shape1->getBody()->setIsSleeping(false);
    shape2->getBody()->setIsSleeping(false);
}

// Remove a body from the collision detection
void CollisionDetection::removeProxyCollisionShape(ProxyShape* proxyShape) {

    assert(proxyShape->getBroadPhaseId() != -1);

    // Remove all the overlapping pairs involving this proxy shape
    for (auto it = mOverlappingPairs.begin(); it != mOverlappingPairs.end(); ) {

        OverlappingPair* pair = it->second;

        if (pair->getShape1()->getBroadPhaseId() == proxyShape->getBroadPhaseId()||
            pair->getShape2()->getBroadPhaseId() == proxyShape->getBroadPhaseId()) {

            // TODO : Remove all the contact manifold of the overlapping pair from the contact manifolds list of the two bodies involved

            // Destroy the overlapping pair
            pair->~OverlappingPair();
            mWorld->mMemoryManager.release(MemoryManager::AllocationType::Pool, pair, sizeof(OverlappingPair));
            it = mOverlappingPairs.remove(it);
        }
        else {
            ++it;
        }
    }

    // Remove the body from the broad-phase
    mBroadPhaseSystem.removeProxyCollisionShape(proxyShape);
}

void CollisionDetection::addAllContactManifoldsToBodies() {

    RP3D_PROFILE("CollisionDetection::addAllContactManifoldsToBodies()", mProfiler);

    // For each overlapping pairs in contact during the narrow-phase
    for (auto it = mOverlappingPairs.begin(); it != mOverlappingPairs.end(); ++it) {

        // Add all the contact manifolds of the pair into the list of contact manifolds
        // of the two bodies involved in the contact
        addContactManifoldToBody(it->second);
    }
}

// Ray casting method
void CollisionDetection::raycast(RaycastCallback* raycastCallback,
                                        const Ray& ray,
                                        unsigned short raycastWithCategoryMaskBits) const {

    RP3D_PROFILE("CollisionDetection::raycast()", mProfiler);

    RaycastTest rayCastTest(raycastCallback);

    // Ask the broad-phase algorithm to call the testRaycastAgainstShape()
    // callback method for each proxy shape hit by the ray in the broad-phase
    mBroadPhaseSystem.raycast(ray, rayCastTest, raycastWithCategoryMaskBits);
}

// Add a contact manifold to the linked list of contact manifolds of the two bodies involved
// in the corresponding contact
void CollisionDetection::addContactManifoldToBody(OverlappingPair* pair) {

    assert(pair != nullptr);

    CollisionBody* body1 = pair->getShape1()->getBody();
    CollisionBody* body2 = pair->getShape2()->getBody();
    const ContactManifoldSet& manifoldSet = pair->getContactManifoldSet();

    // For each contact manifold in the set of manifolds in the pair
    ContactManifold* contactManifold = manifoldSet.getContactManifolds();
    while (contactManifold != nullptr) {

        assert(contactManifold->getNbContactPoints() > 0);

        // Add the contact manifold at the beginning of the linked
        // list of contact manifolds of the first body
        ContactManifoldListElement* listElement1 = new (mMemoryManager.allocate(MemoryManager::AllocationType::Pool,
                                                                                sizeof(ContactManifoldListElement)))
                                                      ContactManifoldListElement(contactManifold,
                                                                         body1->mContactManifoldsList);
        body1->mContactManifoldsList = listElement1;

        // Add the contact manifold at the beginning of the linked
        // list of the contact manifolds of the second body
        ContactManifoldListElement* listElement2 = new (mMemoryManager.allocate(MemoryManager::AllocationType::Pool,
                                                                                sizeof(ContactManifoldListElement)))
                                                      ContactManifoldListElement(contactManifold,
                                                                         body2->mContactManifoldsList);
        body2->mContactManifoldsList = listElement2;

        contactManifold = contactManifold->getNext();
    }
}

/// Convert the potential contact into actual contacts
void CollisionDetection::processPotentialContacts(NarrowPhaseInfoBatch& narrowPhaseInfoBatch, bool updateLastFrameInfo) {

    RP3D_PROFILE("CollisionDetection::processAllPotentialContacts()", mProfiler);

    // For each narrow phase info object
    for(uint i=0; i < narrowPhaseInfoBatch.getNbObjects(); i++) {

        if (updateLastFrameInfo) {
            narrowPhaseInfoBatch.lastFrameCollisionInfos[i]->wasColliding = narrowPhaseInfoBatch.isColliding[i];

            // The previous frame collision info is now valid
            narrowPhaseInfoBatch.lastFrameCollisionInfos[i]->isValid = true;
        }

        if (narrowPhaseInfoBatch.isColliding[i]) {

            assert(narrowPhaseInfoBatch.contactPoints[i].size() > 0);

            // Transfer the contact points from the narrow phase info to the overlapping pair
            narrowPhaseInfoBatch.overlappingPairs[i]->addPotentialContactPoints(narrowPhaseInfoBatch, i);

            // Remove the contacts points from the narrow phase info object.
            narrowPhaseInfoBatch.resetContactPoints(i);
        }
    }
}

// Clear the obsolete manifolds and contact points and reduce the number of contacts points of the remaining manifolds
void CollisionDetection::reduceContactManifolds(const OverlappingPairMap& overlappingPairs) {

    // For each overlapping pairs in contact during the narrow-phase
    for (auto it = overlappingPairs.begin(); it != overlappingPairs.end(); ++it) {

        OverlappingPair* pair = it->second;

        // Clear the obsolete contact manifolds and contact points
        pair->clearObsoleteManifoldsAndContactPoints();

        // Reduce the contact manifolds and contact points if there are too many of them
        pair->reduceContactManifolds();
    }

}

// Report contacts for all the colliding overlapping pairs
void CollisionDetection::reportAllContacts() {

    RP3D_PROFILE("CollisionDetection::reportAllContacts()", mProfiler);

    // For each overlapping pairs in contact during the narrow-phase
    for (auto it = mOverlappingPairs.begin(); it != mOverlappingPairs.end(); ++it) {

        OverlappingPair* pair = it->second;

        // If there is a user callback
        if (mWorld->mEventListener != nullptr && pair->hasContacts()) {

            CollisionCallback::CollisionCallbackInfo collisionInfo(pair, mMemoryManager);

            // Trigger a callback event to report the new contact to the user
             mWorld->mEventListener->newContact(collisionInfo);
        }
    }
}

// Compute the middle-phase collision detection between two proxy shapes
void CollisionDetection::computeMiddlePhaseForProxyShapes(OverlappingPair* pair, NarrowPhaseInput& outNarrowPhaseInput) {

    ProxyShape* shape1 = pair->getShape1();
    ProxyShape* shape2 = pair->getShape2();

    // -------------------------------------------------------

    const bool isShape1Convex = shape1->getCollisionShape()->isConvex();
    const bool isShape2Convex = shape2->getCollisionShape()->isConvex();

    pair->makeLastFrameCollisionInfosObsolete();

    // If both shapes are convex
    if ((isShape1Convex && isShape2Convex)) {

        // Select the narrow phase algorithm to use according to the two collision shapes
        NarrowPhaseAlgorithmType algorithmType = mCollisionDispatch.selectNarrowPhaseAlgorithm(shape1->getCollisionShape()->getType(),
                                                                                               shape2->getCollisionShape()->getType());
        // No middle-phase is necessary, simply create a narrow phase info
        // for the narrow-phase collision detection
        outNarrowPhaseInput.addNarrowPhaseTest(pair, shape1->getCollisionShape(), shape2->getCollisionShape(),
                                                   shape1->getLocalToWorldTransform(), shape2->getLocalToWorldTransform(),
                                                   algorithmType, mMemoryManager.getPoolAllocator());

    }
    // Concave vs Convex algorithm
    else if ((!isShape1Convex && isShape2Convex) || (!isShape2Convex && isShape1Convex)) {

        // Run the middle-phase collision detection algorithm to find the triangles of the concave
        // shape we need to use during the narrow-phase collision detection
        computeConvexVsConcaveMiddlePhase(pair, mMemoryManager.getPoolAllocator(), outNarrowPhaseInput);
    }

    pair->clearObsoleteLastFrameCollisionInfos();
}

// Report all the bodies that overlap with the aabb in parameter
void CollisionDetection::testAABBOverlap(const AABB& aabb, OverlapCallback* overlapCallback,
                                         unsigned short categoryMaskBits) {
    assert(overlapCallback != nullptr);

    Set<bodyindex> reportedBodies(mMemoryManager.getPoolAllocator());

    // Ask the broad-phase to get all the overlapping shapes
    LinkedList<int> overlappingNodes(mMemoryManager.getPoolAllocator());
    mBroadPhaseSystem.reportAllShapesOverlappingWithAABB(aabb, overlappingNodes);

    // For each overlaping proxy shape
    LinkedList<int>::ListElement* element = overlappingNodes.getListHead();
    while (element != nullptr) {

        // Get the overlapping proxy shape
        int broadPhaseId = element->data;
        ProxyShape* proxyShape = mBroadPhaseSystem.getProxyShapeForBroadPhaseId(broadPhaseId);

        CollisionBody* overlapBody = proxyShape->getBody();

        // If the proxy shape is from a body that we have not already reported collision
        if (reportedBodies.find(overlapBody->getId()) == reportedBodies.end()) {

            // Check if the collision filtering allows collision between the two shapes
            if ((proxyShape->getCollisionCategoryBits() & categoryMaskBits) != 0) {

                // Add the body into the set of reported bodies
                reportedBodies.add(overlapBody->getId());

                // Notify the overlap to the user
                overlapCallback->notifyOverlap(overlapBody);
            }
        }

        // Go to the next overlapping proxy shape
        element = element->next;
    }
}

// Return true if two bodies overlap
bool CollisionDetection::testOverlap(CollisionBody* body1, CollisionBody* body2) {

    NarrowPhaseInput narrowPhaseInput(mMemoryManager.getPoolAllocator());

    // For each proxy shape proxy shape of the first body
    const List<Entity>& body1ProxyShapesEntities = mWorld->mBodyComponents.getProxyShapes(body1->getEntity());
    const List<Entity>& body2ProxyShapesEntities = mWorld->mBodyComponents.getProxyShapes(body2->getEntity());
    for (uint i=0; i < body1ProxyShapesEntities.size(); i++) {

        ProxyShape* body1ProxyShape = mWorld->mProxyShapesComponents.getProxyShape(body1ProxyShapesEntities[i]);

        AABB aabb1 = body1ProxyShape->getWorldAABB();

        // For each proxy shape of the second body
        for (uint j=0; j < body2ProxyShapesEntities.size(); j++) {

            ProxyShape* body2ProxyShape = mWorld->mProxyShapesComponents.getProxyShape(body2ProxyShapesEntities[j]);

            AABB aabb2 = body2ProxyShape->getWorldAABB();

            // Test if the AABBs of the two proxy shapes overlap
            if (aabb1.testCollision(aabb2)) {

                // Create a temporary overlapping pair
                OverlappingPair pair(body1ProxyShape, body2ProxyShape, mMemoryManager.getPoolAllocator(),
                                     mMemoryManager.getPoolAllocator(), mWorld->mConfig);

                // Compute the middle-phase collision detection between the two shapes
                computeMiddlePhaseForProxyShapes(&pair, narrowPhaseInput);

            }
        }
    }

    // Test narrow-phase collision
    bool isCollisionFound = testNarrowPhaseCollision(narrowPhaseInput, true, false, mMemoryManager.getPoolAllocator());

    // No overlap has been found
    return isCollisionFound;
}

// Report all the bodies that overlap with the body in parameter
void CollisionDetection::testOverlap(CollisionBody* body, OverlapCallback* overlapCallback,
                                     unsigned short categoryMaskBits) {

    assert(overlapCallback != nullptr);

    Set<bodyindex> reportedBodies(mMemoryManager.getPoolAllocator());
    NarrowPhaseInput narrowPhaseInput(mMemoryManager.getPoolAllocator());

    // For each proxy shape proxy shape of the body
    const List<Entity>& proxyShapesEntities = mWorld->mBodyComponents.getProxyShapes(body->getEntity());
    for (uint i=0; i < proxyShapesEntities.size(); i++) {

        ProxyShape* bodyProxyShape = mWorld->mProxyShapesComponents.getProxyShape(proxyShapesEntities[i]);

        if (bodyProxyShape->getBroadPhaseId() != -1) {

            // Get the AABB of the shape
            const AABB& shapeAABB = mBroadPhaseSystem.getFatAABB(bodyProxyShape->getBroadPhaseId());

            // Ask the broad-phase to get all the overlapping shapes
            LinkedList<int> overlappingNodes(mMemoryManager.getPoolAllocator());
            mBroadPhaseSystem.reportAllShapesOverlappingWithAABB(shapeAABB, overlappingNodes);

            const bodyindex bodyId = body->getId();

            // For each overlaping proxy shape
            LinkedList<int>::ListElement* element = overlappingNodes.getListHead();
            while (element != nullptr) {

                // Get the overlapping proxy shape
                int broadPhaseId = element->data;
                ProxyShape* proxyShape = mBroadPhaseSystem.getProxyShapeForBroadPhaseId(broadPhaseId);

                // If the proxy shape is from a body that we have not already reported collision and the
                // two proxy collision shapes are not from the same body
                if (reportedBodies.find(proxyShape->getBody()->getId()) == reportedBodies.end() &&
                    proxyShape->getBody()->getId() != bodyId) {

                    // Check if the collision filtering allows collision between the two shapes
                    if ((proxyShape->getCollisionCategoryBits() & categoryMaskBits) != 0) {

                        // Create a temporary overlapping pair
                        OverlappingPair pair(bodyProxyShape, proxyShape, mMemoryManager.getPoolAllocator(),
                                             mMemoryManager.getPoolAllocator(), mWorld->mConfig);

                        // Compute the middle-phase collision detection between the two shapes
                        computeMiddlePhaseForProxyShapes(&pair, narrowPhaseInput);

                        // Test narrow-phase collision
                        if (testNarrowPhaseCollision(narrowPhaseInput, true, false, mMemoryManager.getPoolAllocator())) {

                            CollisionBody* overlapBody = proxyShape->getBody();

                            // Add the body into the set of reported bodies
                            reportedBodies.add(overlapBody->getId());

                            // Notify the overlap to the user
                            overlapCallback->notifyOverlap(overlapBody);
                        }

                        narrowPhaseInput.clear();
                    }
                }

                // Go to the next overlapping proxy shape
                element = element->next;
            }
        }
    }
}

// Test and report collisions between two bodies
void CollisionDetection::testCollision(CollisionBody* body1, CollisionBody* body2, CollisionCallback* collisionCallback) {

    assert(collisionCallback != nullptr);

    NarrowPhaseInput narrowPhaseInput(mMemoryManager.getPoolAllocator());
    OverlappingPairMap overlappingPairs(mMemoryManager.getPoolAllocator());

    // For each proxy shape proxy shape of the first body
    const List<Entity>& body1ProxyShapesEntities = mWorld->mBodyComponents.getProxyShapes(body1->getEntity());
    const List<Entity>& body2ProxyShapesEntities = mWorld->mBodyComponents.getProxyShapes(body2->getEntity());
    for (uint i=0; i < body1ProxyShapesEntities.size(); i++) {

        ProxyShape* body1ProxyShape = mWorld->mProxyShapesComponents.getProxyShape(body1ProxyShapesEntities[i]);

        AABB aabb1 = body1ProxyShape->getWorldAABB();

        // For each proxy shape of the second body
        for (uint j=0; j < body2ProxyShapesEntities.size(); j++) {

            ProxyShape* body2ProxyShape = mWorld->mProxyShapesComponents.getProxyShape(body2ProxyShapesEntities[i]);

            AABB aabb2 = body2ProxyShape->getWorldAABB();

            // Test if the AABBs of the two proxy shapes overlap
            if (aabb1.testCollision(aabb2)) {

                OverlappingPair* pair;
                const Pair<uint, uint> pairID = OverlappingPair::computeID(body1ProxyShape, body2ProxyShape);

                // Try to retrieve a corresponding copy of the overlapping pair (if it exists)
                auto itPair = overlappingPairs.find(pairID);

                // If a copy of the overlapping pair does not exist yet
                if (itPair == overlappingPairs.end()) {

                   // Create a temporary copy of the overlapping pair
                   pair = new (mMemoryManager.allocate(MemoryManager::AllocationType::Pool, sizeof(OverlappingPair)))
                                  OverlappingPair(body1ProxyShape, body2ProxyShape, mMemoryManager.getPoolAllocator(),
                                                  mMemoryManager.getPoolAllocator(), mWorld->mConfig);

                    overlappingPairs.add(Pair<Pair<uint, uint>, OverlappingPair*>(pairID, pair));
                }
                else { // If a temporary copy of this overlapping pair already exists

                    // Retrieve the existing copy of the overlapping pair
                    pair = itPair->second;
                }

                // Compute the middle-phase collision detection between the two shapes
                computeMiddlePhaseForProxyShapes(pair, narrowPhaseInput);
            }
        }
    }

    // Test narrow-phase collision
    testNarrowPhaseCollision(narrowPhaseInput, false, true, mMemoryManager.getPoolAllocator());

    // Process the potential contacts
    processAllPotentialContacts(narrowPhaseInput, false);

    // Reduce the number of contact points in the manifolds
    reduceContactManifolds(overlappingPairs);

    // For each overlapping pair
    for (auto it = overlappingPairs.begin(); it != overlappingPairs.end(); ++it) {

        OverlappingPair* pair = it->second;

        if (pair->hasContacts()) {

            // Report the contacts to the user
            CollisionCallback::CollisionCallbackInfo collisionInfo(pair, mMemoryManager);
            collisionCallback->notifyContact(collisionInfo);
        }

        // Destroy the temporary overlapping pair
        pair->~OverlappingPair();
        mMemoryManager.release(MemoryManager::AllocationType::Pool, pair, sizeof(OverlappingPair));
    }
}

// Test and report collisions between a body and all the others bodies of the world
void CollisionDetection::testCollision(CollisionBody* body, CollisionCallback* callback, unsigned short categoryMaskBits) {

    assert(callback != nullptr);

    NarrowPhaseInput narrowPhaseInput(mMemoryManager.getPoolAllocator());
    OverlappingPairMap overlappingPairs(mMemoryManager.getPoolAllocator());

    // For each proxy shape proxy shape of the body
    const List<Entity>& proxyShapesEntities = mWorld->mBodyComponents.getProxyShapes(body->getEntity());
    for (uint i=0; i < proxyShapesEntities.size(); i++) {

        ProxyShape* bodyProxyShape = mWorld->mProxyShapesComponents.getProxyShape(proxyShapesEntities[i]);

        if (bodyProxyShape->getBroadPhaseId() != -1) {

            // Get the AABB of the shape
            const AABB& shapeAABB = mBroadPhaseSystem.getFatAABB(bodyProxyShape->getBroadPhaseId());

            // Ask the broad-phase to get all the overlapping shapes
            LinkedList<int> overlappingNodes(mMemoryManager.getPoolAllocator());
            mBroadPhaseSystem.reportAllShapesOverlappingWithAABB(shapeAABB, overlappingNodes);

            const bodyindex bodyId = body->getId();

            // For each overlaping proxy shape
            LinkedList<int>::ListElement* element = overlappingNodes.getListHead();
            while (element != nullptr) {

                // Get the overlapping proxy shape
                int broadPhaseId = element->data;
                ProxyShape* proxyShape = mBroadPhaseSystem.getProxyShapeForBroadPhaseId(broadPhaseId);

                // If the two proxy collision shapes are not from the same body
                if (proxyShape->getBody()->getId() != bodyId) {

                    // Check if the collision filtering allows collision between the two shapes
                    if ((proxyShape->getCollisionCategoryBits() & categoryMaskBits) != 0) {

                        OverlappingPair* pair;
                        const Pair<uint, uint> pairID = OverlappingPair::computeID(bodyProxyShape, proxyShape);

                        // Try to retrieve a corresponding copy of the overlapping pair (if it exists)
                        auto itPair = overlappingPairs.find(pairID);

                        // If a copy of the overlapping pair does not exist yet
                        if (itPair == overlappingPairs.end()) {

                            // Create a temporary overlapping pair
                            pair = new (mMemoryManager.allocate(MemoryManager::AllocationType::Pool, sizeof(OverlappingPair)))
                                  OverlappingPair(bodyProxyShape, proxyShape, mMemoryManager.getPoolAllocator(),
                                                  mMemoryManager.getPoolAllocator(), mWorld->mConfig);

                            overlappingPairs.add(Pair<Pair<uint, uint>, OverlappingPair*>(pairID, pair));
                        }
                        else { // If a temporary copy of this overlapping pair already exists

                            // Retrieve the existing copy of the overlapping pair
                            pair = itPair->second;
                        }

                        // Compute the middle-phase collision detection between the two shapes
                        computeMiddlePhaseForProxyShapes(pair, narrowPhaseInput);
                    }
                }

                // Go to the next overlapping proxy shape
                element = element->next;
            }
        }
    }

    // Test narrow-phase collision
    testNarrowPhaseCollision(narrowPhaseInput, false, true, mMemoryManager.getPoolAllocator());

    // Process the potential contacts
    processAllPotentialContacts(narrowPhaseInput, false);

    // Reduce the number of contact points in the manifolds
    reduceContactManifolds(overlappingPairs);

    // For each overlapping pair
    for (auto it = overlappingPairs.begin(); it != overlappingPairs.end(); ++it) {

        OverlappingPair* pair = it->second;

        if (pair->hasContacts()) {

            // Report the contacts to the user
            CollisionCallback::CollisionCallbackInfo collisionInfo(pair, mMemoryManager);
            callback->notifyContact(collisionInfo);
        }

        // Destroy the temporary overlapping pair
        pair->~OverlappingPair();
        mMemoryManager.release(MemoryManager::AllocationType::Pool, pair, sizeof(OverlappingPair));
    }
}

// Test and report collisions between all shapes of the world
void CollisionDetection::testCollision(CollisionCallback* callback) {

    assert(callback != nullptr);

    // Compute the broad-phase collision detection
    computeBroadPhase();

    NarrowPhaseInput narrowPhaseInput(mMemoryManager.getPoolAllocator());
    OverlappingPairMap overlappingPairs(mMemoryManager.getPoolAllocator());

    // For each possible collision pair of bodies
    for (auto it = mOverlappingPairs.begin(); it != mOverlappingPairs.end(); ++it) {

        OverlappingPair* originalPair = it->second;

        OverlappingPair* pair;
        const Pair<uint, uint> pairID = OverlappingPair::computeID(originalPair->getShape1(), originalPair->getShape2());

        // Try to retrieve a corresponding copy of the overlapping pair (if it exists)
        auto itPair = overlappingPairs.find(pairID);

        // If a copy of the overlapping pair does not exist yet
        if (itPair == overlappingPairs.end()) {

            // Create a temporary overlapping pair
            pair = new (mMemoryManager.allocate(MemoryManager::AllocationType::Pool, sizeof(OverlappingPair)))
                          OverlappingPair(originalPair->getShape1(), originalPair->getShape2(), mMemoryManager.getPoolAllocator(),
                                          mMemoryManager.getPoolAllocator(), mWorld->mConfig);

            overlappingPairs.add(Pair<Pair<uint, uint>, OverlappingPair*>(pairID, pair));
        }
        else { // If a temporary copy of this overlapping pair already exists

            // Retrieve the existing copy of the overlapping pair
            pair = itPair->second;
        }

        ProxyShape* shape1 = pair->getShape1();
        ProxyShape* shape2 = pair->getShape2();

        // Check if the collision filtering allows collision between the two shapes and
        // that the two shapes are still overlapping.
        if (((shape1->getCollideWithMaskBits() & shape2->getCollisionCategoryBits()) != 0 &&
             (shape1->getCollisionCategoryBits() & shape2->getCollideWithMaskBits()) != 0) &&
             mBroadPhaseSystem.testOverlappingShapes(shape1, shape2)) {

            // Compute the middle-phase collision detection between the two shapes
            computeMiddlePhaseForProxyShapes(pair, narrowPhaseInput);
        }
    }

    // Test narrow-phase collision
    testNarrowPhaseCollision(narrowPhaseInput, false, true, mMemoryManager.getPoolAllocator());

    // Process the potential contacts
    processAllPotentialContacts(narrowPhaseInput, false);

    // Reduce the number of contact points in the manifolds
    reduceContactManifolds(overlappingPairs);

    // For each overlapping pair
    for (auto it = overlappingPairs.begin(); it != overlappingPairs.end(); ++it) {

        OverlappingPair* pair = it->second;

        if (pair->hasContacts()) {

            // Report the contacts to the user
            CollisionCallback::CollisionCallbackInfo collisionInfo(pair, mMemoryManager);
            callback->notifyContact(collisionInfo);
        }

        // Destroy the temporary overlapping pair
        pair->~OverlappingPair();
        mMemoryManager.release(MemoryManager::AllocationType::Pool, pair, sizeof(OverlappingPair));
    }
}

// Return the world event listener
EventListener* CollisionDetection::getWorldEventListener() {
   return mWorld->mEventListener;
}

// Return the world-space AABB of a given proxy shape
const AABB CollisionDetection::getWorldAABB(const ProxyShape* proxyShape) const {
    assert(proxyShape->getBroadPhaseId() > -1);
    return mBroadPhaseSystem.getFatAABB(proxyShape->getBroadPhaseId());
}
