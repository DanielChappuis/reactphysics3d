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
#include "collision/NarrowPhaseInfo.h"
#include "collision/ContactManifold.h"
#include "collision/ContactManifoldInfo.h"
#include "utils/Profiler.h"
#include "engine/EventListener.h"
#include "collision/RaycastInfo.h"
#include <cassert>

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;
using namespace std;


// Constructor
CollisionDetection::CollisionDetection(CollisionWorld* world, MemoryManager& memoryManager)
                   : mMemoryManager(memoryManager), mWorld(world), mNarrowPhaseInfoList(nullptr),
                     mOverlappingPairs(mMemoryManager.getPoolAllocator()), mBroadPhaseAlgorithm(*this),
                     mNoCollisionPairs(mMemoryManager.getPoolAllocator()), mIsCollisionShapesAdded(false) {

    // Set the default collision dispatch configuration
    setCollisionDispatch(&mDefaultCollisionDispatch);

    // Fill-in the collision detection matrix with algorithms
    fillInCollisionMatrix();

#ifdef IS_PROFILING_ACTIVE

	mProfiler = nullptr;

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

    // Reset the linked list of narrow-phase info
    mNarrowPhaseInfoList = nullptr;
}

// Compute the broad-phase collision detection
void CollisionDetection::computeBroadPhase() {

    RP3D_PROFILE("CollisionDetection::computeBroadPhase()", mProfiler);

    // If new collision shapes have been added to bodies
    if (mIsCollisionShapesAdded) {

        // Ask the broad-phase to recompute the overlapping pairs of collision
        // shapes. This call can only add new overlapping pairs in the collision
        // detection.
        mBroadPhaseAlgorithm.computeOverlappingPairs(mMemoryManager);
    }
}

// Compute the middle-phase collision detection
void CollisionDetection::computeMiddlePhase() {

    RP3D_PROFILE("CollisionDetection::computeMiddlePhase()", mProfiler);

    // For each possible collision pair of bodies
    Map<Pair<uint, uint>, OverlappingPair*>::Iterator it;
    for (it = mOverlappingPairs.begin(); it != mOverlappingPairs.end(); ) {

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
        if (!mBroadPhaseAlgorithm.testOverlappingShapes(shape1, shape2)) {

            // Destroy the overlapping pair
            it->second->~OverlappingPair();

            mWorld->mMemoryManager.release(MemoryManager::AllocationType::Pool, it->second, sizeof(OverlappingPair));
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

                // No middle-phase is necessary, simply create a narrow phase info
                // for the narrow-phase collision detection
                NarrowPhaseInfo* firstNarrowPhaseInfo = mNarrowPhaseInfoList;
                mNarrowPhaseInfoList = new (mMemoryManager.allocate(MemoryManager::AllocationType::Frame, sizeof(NarrowPhaseInfo)))
                                       NarrowPhaseInfo(pair, shape1->getCollisionShape(),
                                       shape2->getCollisionShape(), shape1->getLocalToWorldTransform(),
                                       shape2->getLocalToWorldTransform(), mMemoryManager.getSingleFrameAllocator());
                mNarrowPhaseInfoList->next = firstNarrowPhaseInfo;

            }
            // Concave vs Convex algorithm
            else if ((!isShape1Convex && isShape2Convex) || (!isShape2Convex && isShape1Convex)) {

                NarrowPhaseInfo* narrowPhaseInfo = nullptr;
                computeConvexVsConcaveMiddlePhase(pair, mMemoryManager.getSingleFrameAllocator(), &narrowPhaseInfo);

                // Add all the narrow-phase info object reported by the callback into the
                // list of all the narrow-phase info object
                while (narrowPhaseInfo != nullptr) {
                    NarrowPhaseInfo* next = narrowPhaseInfo->next;
                    narrowPhaseInfo->next = mNarrowPhaseInfoList;
                    mNarrowPhaseInfoList = narrowPhaseInfo;

                    narrowPhaseInfo = next;
                }
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
                                                           NarrowPhaseInfo** firstNarrowPhaseInfo) {

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

    // Set the parameters of the callback object
    MiddlePhaseTriangleCallback middlePhaseCallback(pair, concaveProxyShape, convexProxyShape,
                                                    concaveShape, allocator);

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

    // Add all the narrow-phase info object reported by the callback into the
    // list of all the narrow-phase info object
    *firstNarrowPhaseInfo = middlePhaseCallback.narrowPhaseInfoList;
}

// Compute the narrow-phase collision detection
void CollisionDetection::computeNarrowPhase() {

    RP3D_PROFILE("CollisionDetection::computeNarrowPhase()", mProfiler);

    NarrowPhaseInfo* currentNarrowPhaseInfo = mNarrowPhaseInfoList;
    while (currentNarrowPhaseInfo != nullptr) {

        // Select the narrow phase algorithm to use according to the two collision shapes
        const CollisionShapeType shape1Type = currentNarrowPhaseInfo->collisionShape1->getType();
        const CollisionShapeType shape2Type = currentNarrowPhaseInfo->collisionShape2->getType();
        NarrowPhaseAlgorithm* narrowPhaseAlgorithm = selectNarrowPhaseAlgorithm(shape1Type, shape2Type);

        // If there is no collision algorithm between those two kinds of shapes, skip it
        if (narrowPhaseAlgorithm != nullptr) {

            LastFrameCollisionInfo* lastCollisionFrameInfo = currentNarrowPhaseInfo->getLastFrameCollisionInfo();

            // Use the narrow-phase collision detection algorithm to check
            // if there really is a collision. If a collision occurs, the
            // notifyContact() callback method will be called.
            if (narrowPhaseAlgorithm->testCollision(currentNarrowPhaseInfo, true, mMemoryManager.getSingleFrameAllocator())) {

                // Add the contact points as a potential contact manifold into the pair                
                currentNarrowPhaseInfo->addContactPointsAsPotentialContactManifold();

                lastCollisionFrameInfo->wasColliding = true;
            }
            else {
                lastCollisionFrameInfo->wasColliding = false;
            }

            // The previous frame collision info is now valid
            lastCollisionFrameInfo->isValid = true;
        }

        NarrowPhaseInfo* narrowPhaseInfoToDelete = currentNarrowPhaseInfo;
        currentNarrowPhaseInfo = currentNarrowPhaseInfo->next;

        // Call the destructor
        narrowPhaseInfoToDelete->~NarrowPhaseInfo();

        // Release the allocated memory for the narrow phase info
        mMemoryManager.release(MemoryManager::AllocationType::Frame, narrowPhaseInfoToDelete, sizeof(NarrowPhaseInfo));
    }

    // Convert the potential contact into actual contacts
    processAllPotentialContacts();

    // Add all the contact manifolds (between colliding bodies) to the bodies
    addAllContactManifoldsToBodies();

    // Report contacts to the user
    reportAllContacts();
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

    mOverlappingPairs.add(Pair<Pair<uint, uint>, OverlappingPair*>(pairID, newPair));

    // Wake up the two bodies
    shape1->getBody()->setIsSleeping(false);
    shape2->getBody()->setIsSleeping(false);
}

// Remove a body from the collision detection
void CollisionDetection::removeProxyCollisionShape(ProxyShape* proxyShape) {

    assert(proxyShape->getBroadPhaseId() != -1);

    // Remove all the overlapping pairs involving this proxy shape
    Map<Pair<uint, uint>, OverlappingPair*>::Iterator it;
    for (it = mOverlappingPairs.begin(); it != mOverlappingPairs.end(); ) {
        if (it->second->getShape1()->getBroadPhaseId() == proxyShape->getBroadPhaseId()||
            it->second->getShape2()->getBroadPhaseId() == proxyShape->getBroadPhaseId()) {

            // TODO : Remove all the contact manifold of the overlapping pair from the contact manifolds list of the two bodies involved

            // Destroy the overlapping pair
            it->second->~OverlappingPair();
            mWorld->mMemoryManager.release(MemoryManager::AllocationType::Pool, it->second, sizeof(OverlappingPair));
            it = mOverlappingPairs.remove(it);
        }
        else {
            ++it;
        }
    }

    // Remove the body from the broad-phase
    mBroadPhaseAlgorithm.removeProxyCollisionShape(proxyShape);
}

void CollisionDetection::addAllContactManifoldsToBodies() {

    RP3D_PROFILE("CollisionDetection::addAllContactManifoldsToBodies()", mProfiler);

    // For each overlapping pairs in contact during the narrow-phase
    Map<Pair<uint, uint>, OverlappingPair*>::Iterator it;
    for (it = mOverlappingPairs.begin(); it != mOverlappingPairs.end(); ++it) {

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
    mBroadPhaseAlgorithm.raycast(ray, rayCastTest, raycastWithCategoryMaskBits);
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
void CollisionDetection::processAllPotentialContacts() {

    RP3D_PROFILE("CollisionDetection::processAllPotentialContacts()", mProfiler);

    // For each overlapping pairs in contact during the narrow-phase
    Map<Pair<uint, uint>, OverlappingPair*>::Iterator it;
    for (it = mOverlappingPairs.begin(); it != mOverlappingPairs.end(); ++it) {

        // Process the potential contacts of the overlapping pair
        processPotentialContacts(it->second);
    }
}

// Process the potential contact manifold of a pair to create actual contact manifold
void CollisionDetection::processPotentialContacts(OverlappingPair* pair) {

    // Reduce the number of contact points of the manifold
    pair->reducePotentialContactManifolds();

	// Add all the potential contact manifolds as actual contact manifolds to the pair
	ContactManifoldInfo* potentialManifold = pair->getPotentialContactManifolds();
	while (potentialManifold != nullptr) {

		pair->addContactManifold(potentialManifold);

		potentialManifold = potentialManifold->mNext;
	}

    // Clear the obsolete contact manifolds and contact points
    pair->clearObsoleteManifoldsAndContactPoints();

    // Reduce the contact manifolds and contact points if there are too many of them
    pair->reduceContactManifolds();

    // Reset the potential contacts of the pair
    pair->clearPotentialContactManifolds();
}

// Report contacts for all the colliding overlapping pairs
void CollisionDetection::reportAllContacts() {

    RP3D_PROFILE("CollisionDetection::reportAllContacts()", mProfiler);

    // For each overlapping pairs in contact during the narrow-phase
    Map<Pair<uint, uint>, OverlappingPair*>::Iterator it;
    for (it = mOverlappingPairs.begin(); it != mOverlappingPairs.end(); ++it) {

        // If there is a user callback
        if (mWorld->mEventListener != nullptr && it->second->hasContacts()) {

            CollisionCallback::CollisionCallbackInfo collisionInfo(it->second, mMemoryManager);

            // Trigger a callback event to report the new contact to the user
             mWorld->mEventListener->newContact(collisionInfo);
        }
    }
}

// Compute the middle-phase collision detection between two proxy shapes
NarrowPhaseInfo* CollisionDetection::computeMiddlePhaseForProxyShapes(OverlappingPair* pair) {

    ProxyShape* shape1 = pair->getShape1();
    ProxyShape* shape2 = pair->getShape2();

    // -------------------------------------------------------

    const bool isShape1Convex = shape1->getCollisionShape()->isConvex();
    const bool isShape2Convex = shape2->getCollisionShape()->isConvex();

    NarrowPhaseInfo* narrowPhaseInfo = nullptr;

    pair->makeLastFrameCollisionInfosObsolete();

    // If both shapes are convex
    if ((isShape1Convex && isShape2Convex)) {

        // No middle-phase is necessary, simply create a narrow phase info
        // for the narrow-phase collision detection
        narrowPhaseInfo = new (mMemoryManager.allocate(MemoryManager::AllocationType::Pool,
                                                       sizeof(NarrowPhaseInfo))) NarrowPhaseInfo(pair, shape1->getCollisionShape(),
                                       shape2->getCollisionShape(), shape1->getLocalToWorldTransform(),
                                       shape2->getLocalToWorldTransform(), mMemoryManager.getPoolAllocator());

    }
    // Concave vs Convex algorithm
    else if ((!isShape1Convex && isShape2Convex) || (!isShape2Convex && isShape1Convex)) {

        // Run the middle-phase collision detection algorithm to find the triangles of the concave
        // shape we need to use during the narrow-phase collision detection
        computeConvexVsConcaveMiddlePhase(pair, mMemoryManager.getPoolAllocator(), &narrowPhaseInfo);
    }

    pair->clearObsoleteLastFrameCollisionInfos();

    return narrowPhaseInfo;
}

// Report all the bodies that overlap with the aabb in parameter
void CollisionDetection::testAABBOverlap(const AABB& aabb, OverlapCallback* overlapCallback,
                                         unsigned short categoryMaskBits) {
    assert(overlapCallback != nullptr);

    Set<bodyindex> reportedBodies(mMemoryManager.getPoolAllocator());

    // Ask the broad-phase to get all the overlapping shapes
    LinkedList<int> overlappingNodes(mMemoryManager.getPoolAllocator());
    mBroadPhaseAlgorithm.reportAllShapesOverlappingWithAABB(aabb, overlappingNodes);

    // For each overlaping proxy shape
    LinkedList<int>::ListElement* element = overlappingNodes.getListHead();
    while (element != nullptr) {

        // Get the overlapping proxy shape
        int broadPhaseId = element->data;
        ProxyShape* proxyShape = mBroadPhaseAlgorithm.getProxyShapeForBroadPhaseId(broadPhaseId);

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

    // For each proxy shape proxy shape of the first body
    ProxyShape* body1ProxyShape = body1->getProxyShapesList();
    while (body1ProxyShape != nullptr) {

        AABB aabb1 = body1ProxyShape->getWorldAABB();

        // For each proxy shape of the second body
        ProxyShape* body2ProxyShape = body2->getProxyShapesList();
        while (body2ProxyShape != nullptr) {

            AABB aabb2 = body2ProxyShape->getWorldAABB();

            // Test if the AABBs of the two proxy shapes overlap
            if (aabb1.testCollision(aabb2)) {

                // Create a temporary overlapping pair
                OverlappingPair pair(body1ProxyShape, body2ProxyShape, mMemoryManager.getPoolAllocator(),
                                     mMemoryManager.getPoolAllocator(), mWorld->mConfig);

                // Compute the middle-phase collision detection between the two shapes
                NarrowPhaseInfo* narrowPhaseInfo = computeMiddlePhaseForProxyShapes(&pair);

                bool isColliding = false;

                // For each narrow-phase info object
                while (narrowPhaseInfo != nullptr) {

                    // If we have not found a collision yet
                    if (!isColliding) {

						const CollisionShapeType shape1Type = narrowPhaseInfo->collisionShape1->getType();
						const CollisionShapeType shape2Type = narrowPhaseInfo->collisionShape2->getType();

                        // Select the narrow phase algorithm to use according to the two collision shapes
                        NarrowPhaseAlgorithm* narrowPhaseAlgorithm = selectNarrowPhaseAlgorithm(shape1Type, shape2Type);

                        // If there is a collision algorithm for those two kinds of shapes
                        if (narrowPhaseAlgorithm != nullptr) {

                            // Use the narrow-phase collision detection algorithm to check
                            // if there really is a collision. If a collision occurs, the
                            // notifyContact() callback method will be called.
                            isColliding |= narrowPhaseAlgorithm->testCollision(narrowPhaseInfo, false, mMemoryManager.getPoolAllocator());
                        }
                    }

                    NarrowPhaseInfo* currentNarrowPhaseInfo = narrowPhaseInfo;
                    narrowPhaseInfo = narrowPhaseInfo->next;

                    // Call the destructor
                    currentNarrowPhaseInfo->~NarrowPhaseInfo();

                    // Release the allocated memory
                    mMemoryManager.release(MemoryManager::AllocationType::Pool, currentNarrowPhaseInfo, sizeof(NarrowPhaseInfo));
                }

                // Return if we have found a narrow-phase collision
                if (isColliding) return true;
            }

            // Go to the next proxy shape
            body2ProxyShape = body2ProxyShape->getNext();
        }

        // Go to the next proxy shape
        body1ProxyShape = body1ProxyShape->getNext();
    }

    // No overlap has been found
    return false;
}

// Report all the bodies that overlap with the body in parameter
void CollisionDetection::testOverlap(CollisionBody* body, OverlapCallback* overlapCallback,
                                     unsigned short categoryMaskBits) {

    assert(overlapCallback != nullptr);

    Set<bodyindex> reportedBodies(mMemoryManager.getPoolAllocator());

    // For each proxy shape proxy shape of the body
    ProxyShape* bodyProxyShape = body->getProxyShapesList();
    while (bodyProxyShape != nullptr) {

        if (bodyProxyShape->getBroadPhaseId() != -1) {

            // Get the AABB of the shape
            const AABB& shapeAABB = mBroadPhaseAlgorithm.getFatAABB(bodyProxyShape->getBroadPhaseId());

            // Ask the broad-phase to get all the overlapping shapes
            LinkedList<int> overlappingNodes(mMemoryManager.getPoolAllocator());
            mBroadPhaseAlgorithm.reportAllShapesOverlappingWithAABB(shapeAABB, overlappingNodes);

            const bodyindex bodyId = body->getId();

            // For each overlaping proxy shape
            LinkedList<int>::ListElement* element = overlappingNodes.getListHead();
            while (element != nullptr) {

                // Get the overlapping proxy shape
                int broadPhaseId = element->data;
                ProxyShape* proxyShape = mBroadPhaseAlgorithm.getProxyShapeForBroadPhaseId(broadPhaseId);

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
                        NarrowPhaseInfo* narrowPhaseInfo = computeMiddlePhaseForProxyShapes(&pair);

                        bool isColliding = false;

                        // For each narrow-phase info object
                        while (narrowPhaseInfo != nullptr) {

                            // If we have not found a collision yet
                            if (!isColliding) {

                                const CollisionShapeType shape1Type = narrowPhaseInfo->collisionShape1->getType();
                                const CollisionShapeType shape2Type = narrowPhaseInfo->collisionShape2->getType();

                                // Select the narrow phase algorithm to use according to the two collision shapes
                                NarrowPhaseAlgorithm* narrowPhaseAlgorithm = selectNarrowPhaseAlgorithm(shape1Type, shape2Type);

                                // If there is a collision algorithm for those two kinds of shapes
                                if (narrowPhaseAlgorithm != nullptr) {

                                    // Use the narrow-phase collision detection algorithm to check
                                    // if there really is a collision. If a collision occurs, the
                                    // notifyContact() callback method will be called.
                                    isColliding |= narrowPhaseAlgorithm->testCollision(narrowPhaseInfo, false, mMemoryManager.getPoolAllocator());
                                }
                            }

                            NarrowPhaseInfo* currentNarrowPhaseInfo = narrowPhaseInfo;
                            narrowPhaseInfo = narrowPhaseInfo->next;

                            // Call the destructor
                            currentNarrowPhaseInfo->~NarrowPhaseInfo();

                            // Release the allocated memory
                            mMemoryManager.release(MemoryManager::AllocationType::Pool, currentNarrowPhaseInfo, sizeof(NarrowPhaseInfo));
                        }

                        // Return if we have found a narrow-phase collision
                        if (isColliding) {

                            CollisionBody* overlapBody = proxyShape->getBody();

                            // Add the body into the set of reported bodies
                            reportedBodies.add(overlapBody->getId());

                            // Notify the overlap to the user
                            overlapCallback->notifyOverlap(overlapBody);
                        }
                    }
                }

                // Go to the next overlapping proxy shape
                element = element->next;
            }
        }

        // Go to the next proxy shape
        bodyProxyShape = bodyProxyShape->getNext();
    }
}

// Test and report collisions between two bodies
void CollisionDetection::testCollision(CollisionBody* body1, CollisionBody* body2, CollisionCallback* collisionCallback) {

    assert(collisionCallback != nullptr);

    // For each proxy shape proxy shape of the first body
    ProxyShape* body1ProxyShape = body1->getProxyShapesList();
    while (body1ProxyShape != nullptr) {

        AABB aabb1 = body1ProxyShape->getWorldAABB();

        // For each proxy shape of the second body
        ProxyShape* body2ProxyShape = body2->getProxyShapesList();
        while (body2ProxyShape != nullptr) {

            AABB aabb2 = body2ProxyShape->getWorldAABB();

            // Test if the AABBs of the two proxy shapes overlap
            if (aabb1.testCollision(aabb2)) {

                // Create a temporary overlapping pair
                OverlappingPair pair(body1ProxyShape, body2ProxyShape, mMemoryManager.getPoolAllocator(),
                                     mMemoryManager.getPoolAllocator(), mWorld->mConfig);

                // Compute the middle-phase collision detection between the two shapes
                NarrowPhaseInfo* narrowPhaseInfo = computeMiddlePhaseForProxyShapes(&pair);

                // For each narrow-phase info object
                while (narrowPhaseInfo != nullptr) {

					const CollisionShapeType shape1Type = narrowPhaseInfo->collisionShape1->getType();
					const CollisionShapeType shape2Type = narrowPhaseInfo->collisionShape2->getType();

                    // Select the narrow phase algorithm to use according to the two collision shapes
                    NarrowPhaseAlgorithm* narrowPhaseAlgorithm = selectNarrowPhaseAlgorithm(shape1Type, shape2Type);

                    // If there is a collision algorithm for those two kinds of shapes
                    if (narrowPhaseAlgorithm != nullptr) {

                        // Use the narrow-phase collision detection algorithm to check
                        // if there really is a collision. If a collision occurs, the
                        // notifyContact() callback method will be called.
                        if (narrowPhaseAlgorithm->testCollision(narrowPhaseInfo, true, mMemoryManager.getPoolAllocator())) {

                            // Add the contact points as a potential contact manifold into the pair
                            narrowPhaseInfo->addContactPointsAsPotentialContactManifold();
                        }
                    }

                    NarrowPhaseInfo* currentNarrowPhaseInfo = narrowPhaseInfo;
                    narrowPhaseInfo = narrowPhaseInfo->next;

                    // Call the destructor
                    currentNarrowPhaseInfo->~NarrowPhaseInfo();

                    // Release the allocated memory
                    mMemoryManager.release(MemoryManager::AllocationType::Pool, currentNarrowPhaseInfo, sizeof(NarrowPhaseInfo));
                }

                // Process the potential contacts
                processPotentialContacts(&pair);

				if (pair.hasContacts()) {

					// Report the contacts to the user
                    CollisionCallback::CollisionCallbackInfo collisionInfo(&pair, mMemoryManager);
					collisionCallback->notifyContact(collisionInfo);
				}
            }

            // Go to the next proxy shape
            body2ProxyShape = body2ProxyShape->getNext();
        }

        // Go to the next proxy shape
        body1ProxyShape = body1ProxyShape->getNext();
    }
}

// Test and report collisions between a body and all the others bodies of the world
void CollisionDetection::testCollision(CollisionBody* body, CollisionCallback* callback, unsigned short categoryMaskBits) {

    assert(callback != nullptr);

    // For each proxy shape proxy shape of the body
    ProxyShape* bodyProxyShape = body->getProxyShapesList();
    while (bodyProxyShape != nullptr) {

        if (bodyProxyShape->getBroadPhaseId() != -1) {

            // Get the AABB of the shape
            const AABB& shapeAABB = mBroadPhaseAlgorithm.getFatAABB(bodyProxyShape->getBroadPhaseId());

            // Ask the broad-phase to get all the overlapping shapes
            LinkedList<int> overlappingNodes(mMemoryManager.getPoolAllocator());
            mBroadPhaseAlgorithm.reportAllShapesOverlappingWithAABB(shapeAABB, overlappingNodes);

            const bodyindex bodyId = body->getId();

            // For each overlaping proxy shape
            LinkedList<int>::ListElement* element = overlappingNodes.getListHead();
            while (element != nullptr) {

                // Get the overlapping proxy shape
                int broadPhaseId = element->data;
                ProxyShape* proxyShape = mBroadPhaseAlgorithm.getProxyShapeForBroadPhaseId(broadPhaseId);

                // If the two proxy collision shapes are not from the same body
                if (proxyShape->getBody()->getId() != bodyId) {

                    // Check if the collision filtering allows collision between the two shapes
                    if ((proxyShape->getCollisionCategoryBits() & categoryMaskBits) != 0) {

                        // Create a temporary overlapping pair
                        OverlappingPair pair(bodyProxyShape, proxyShape, mMemoryManager.getPoolAllocator(),
                                             mMemoryManager.getPoolAllocator(), mWorld->mConfig);

                        // Compute the middle-phase collision detection between the two shapes
                        NarrowPhaseInfo* narrowPhaseInfo = computeMiddlePhaseForProxyShapes(&pair);

                        // For each narrow-phase info object
                        while (narrowPhaseInfo != nullptr) {

                            const CollisionShapeType shape1Type = narrowPhaseInfo->collisionShape1->getType();
                            const CollisionShapeType shape2Type = narrowPhaseInfo->collisionShape2->getType();

                            // Select the narrow phase algorithm to use according to the two collision shapes
                            NarrowPhaseAlgorithm* narrowPhaseAlgorithm = selectNarrowPhaseAlgorithm(shape1Type, shape2Type);

                            // If there is a collision algorithm for those two kinds of shapes
                            if (narrowPhaseAlgorithm != nullptr) {

                                // Use the narrow-phase collision detection algorithm to check
                                // if there really is a collision. If a collision occurs, the
                                // notifyContact() callback method will be called.
                                if (narrowPhaseAlgorithm->testCollision(narrowPhaseInfo, true, mMemoryManager.getPoolAllocator())) {

                                    // Add the contact points as a potential contact manifold into the pair
                                    narrowPhaseInfo->addContactPointsAsPotentialContactManifold();
                                }
                            }

                            NarrowPhaseInfo* currentNarrowPhaseInfo = narrowPhaseInfo;
                            narrowPhaseInfo = narrowPhaseInfo->next;

                            // Call the destructor
                            currentNarrowPhaseInfo->~NarrowPhaseInfo();

                            // Release the allocated memory
                            mMemoryManager.release(MemoryManager::AllocationType::Pool, currentNarrowPhaseInfo, sizeof(NarrowPhaseInfo));
                        }

                        // Process the potential contacts
                        processPotentialContacts(&pair);

                        if (pair.hasContacts()) {

                            // Report the contacts to the user
                            CollisionCallback::CollisionCallbackInfo collisionInfo(&pair, mMemoryManager);
                            callback->notifyContact(collisionInfo);
                        }
                    }
                }

                // Go to the next overlapping proxy shape
                element = element->next;
            }

            // Go to the next proxy shape
            bodyProxyShape = bodyProxyShape->getNext();
        }
    }
}

// Test and report collisions between all shapes of the world
void CollisionDetection::testCollision(CollisionCallback* callback) {

    assert(callback != nullptr);

    // Compute the broad-phase collision detection
    computeBroadPhase();

    // For each possible collision pair of bodies
    Map<Pair<uint, uint>, OverlappingPair*>::Iterator it;
    for (it = mOverlappingPairs.begin(); it != mOverlappingPairs.end(); ++it) {

        OverlappingPair* originalPair = it->second;

        // Create a new overlapping pair so that we do not work on the original one
        OverlappingPair pair(originalPair->getShape1(), originalPair->getShape2(), mMemoryManager.getPoolAllocator(),
                             mMemoryManager.getPoolAllocator(), mWorld->mConfig);

        ProxyShape* shape1 = pair.getShape1();
        ProxyShape* shape2 = pair.getShape2();

        // Check if the collision filtering allows collision between the two shapes and
        // that the two shapes are still overlapping.
        if (((shape1->getCollideWithMaskBits() & shape2->getCollisionCategoryBits()) != 0 &&
             (shape1->getCollisionCategoryBits() & shape2->getCollideWithMaskBits()) != 0) &&
             mBroadPhaseAlgorithm.testOverlappingShapes(shape1, shape2)) {

            // Compute the middle-phase collision detection between the two shapes
            NarrowPhaseInfo* narrowPhaseInfo = computeMiddlePhaseForProxyShapes(&pair);

            // For each narrow-phase info object
            while (narrowPhaseInfo != nullptr) {

                const CollisionShapeType shape1Type = narrowPhaseInfo->collisionShape1->getType();
                const CollisionShapeType shape2Type = narrowPhaseInfo->collisionShape2->getType();

                // Select the narrow phase algorithm to use according to the two collision shapes
                NarrowPhaseAlgorithm* narrowPhaseAlgorithm = selectNarrowPhaseAlgorithm(shape1Type, shape2Type);

                // If there is a collision algorithm for those two kinds of shapes
                if (narrowPhaseAlgorithm != nullptr) {

                    // Use the narrow-phase collision detection algorithm to check
                    // if there really is a collision. If a collision occurs, the
                    // notifyContact() callback method will be called.
                    if (narrowPhaseAlgorithm->testCollision(narrowPhaseInfo, true, mMemoryManager.getPoolAllocator())) {

                        // Add the contact points as a potential contact manifold into the pair
                        narrowPhaseInfo->addContactPointsAsPotentialContactManifold();
                    }
                }

                NarrowPhaseInfo* currentNarrowPhaseInfo = narrowPhaseInfo;
                narrowPhaseInfo = narrowPhaseInfo->next;

                // Call the destructor
                currentNarrowPhaseInfo->~NarrowPhaseInfo();

                // Release the allocated memory
                mMemoryManager.release(MemoryManager::AllocationType::Pool, currentNarrowPhaseInfo, sizeof(NarrowPhaseInfo));
            }

            // Process the potential contacts
            processPotentialContacts(&pair);

			if (pair.hasContacts()) {

				// Report the contacts to the user
                CollisionCallback::CollisionCallbackInfo collisionInfo(&pair, mMemoryManager);
				callback->notifyContact(collisionInfo);
			}
        }
    }
}

// Fill-in the collision detection matrix
void CollisionDetection::fillInCollisionMatrix() {

    // For each possible type of collision shape
    for (int i=0; i<NB_COLLISION_SHAPE_TYPES; i++) {
        for (int j=0; j<NB_COLLISION_SHAPE_TYPES; j++) {
            mCollisionMatrix[i][j] = mCollisionDispatch->selectAlgorithm(i, j);
        }
    }
}

// Return the world event listener
EventListener* CollisionDetection::getWorldEventListener() {
   return mWorld->mEventListener;
}

// Return the world-space AABB of a given proxy shape
const AABB CollisionDetection::getWorldAABB(const ProxyShape* proxyShape) const {
    assert(proxyShape->getBroadPhaseId() > -1);
    return mBroadPhaseAlgorithm.getFatAABB(proxyShape->getBroadPhaseId());
}
