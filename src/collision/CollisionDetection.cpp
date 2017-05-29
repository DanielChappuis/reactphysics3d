/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2016 Daniel Chappuis                                       *
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
#include <unordered_set>

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;
using namespace std;

// Constructor
CollisionDetection::CollisionDetection(CollisionWorld* world, PoolAllocator& memoryAllocator, SingleFrameAllocator& singleFrameAllocator)
                   : mMemoryAllocator(memoryAllocator), mSingleFrameAllocator(singleFrameAllocator),
                     mWorld(world), mNarrowPhaseInfoList(nullptr), mBroadPhaseAlgorithm(*this),
                     mIsCollisionShapesAdded(false) {

    // Set the default collision dispatch configuration
    setCollisionDispatch(&mDefaultCollisionDispatch);

    // Fill-in the collision detection matrix with algorithms
    fillInCollisionMatrix();
}

// Compute the collision detection
void CollisionDetection::computeCollisionDetection() {

    PROFILE("CollisionDetection::computeCollisionDetection()");
	    
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

    PROFILE("CollisionDetection::computeBroadPhase()");

    // If new collision shapes have been added to bodies
    if (mIsCollisionShapesAdded) {

        // Ask the broad-phase to recompute the overlapping pairs of collision
        // shapes. This call can only add new overlapping pairs in the collision
        // detection.
        mBroadPhaseAlgorithm.computeOverlappingPairs(mMemoryAllocator);
    }
}

// Compute the middle-phase collision detection
void CollisionDetection::computeMiddlePhase() {

    PROFILE("CollisionDetection::computeMiddlePhase()");

    // Clear the set of overlapping pairs in narrow-phase contact
    mContactOverlappingPairs.clear();

    // For each possible collision pair of bodies
    map<overlappingpairid, OverlappingPair*>::iterator it;
    for (it = mOverlappingPairs.begin(); it != mOverlappingPairs.end(); ) {

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
            mWorld->mPoolAllocator.release(itToRemove->second, sizeof(OverlappingPair));
            mOverlappingPairs.erase(itToRemove);
            continue;
        }
        else {
            ++it;
        }

        CollisionBody* const body1 = shape1->getBody();
        CollisionBody* const body2 = shape2->getBody();

        // Check that at least one body is awake and not static
        bool isBody1Active = !body1->isSleeping() && body1->getType() != BodyType::STATIC;
        bool isBody2Active = !body2->isSleeping() && body2->getType() != BodyType::STATIC;
        if (!isBody1Active && !isBody2Active) continue;

        // Check if the bodies are in the set of bodies that cannot collide between each other
        bodyindexpair bodiesIndex = OverlappingPair::computeBodiesIndexPair(body1, body2);
        if (mNoCollisionPairs.count(bodiesIndex) > 0) continue;

        const CollisionShapeType shape1Type = shape1->getCollisionShape()->getType();
        const CollisionShapeType shape2Type = shape2->getCollisionShape()->getType();

        // If both shapes are convex
        if ((CollisionShape::isConvex(shape1Type) && CollisionShape::isConvex(shape2Type))) {

            // No middle-phase is necessary, simply create a narrow phase info
            // for the narrow-phase collision detection
            NarrowPhaseInfo* firstNarrowPhaseInfo = mNarrowPhaseInfoList;
            mNarrowPhaseInfoList = new (mSingleFrameAllocator.allocate(sizeof(NarrowPhaseInfo)))
                                   NarrowPhaseInfo(pair, shape1->getCollisionShape(),
                                   shape2->getCollisionShape(), shape1->getLocalToWorldTransform(),
                                   shape2->getLocalToWorldTransform(), shape1->getCachedCollisionData(),
                                   shape2->getCachedCollisionData());
            mNarrowPhaseInfoList->next = firstNarrowPhaseInfo;

        }
        // Concave vs Convex algorithm
        else if ((!CollisionShape::isConvex(shape1Type) && CollisionShape::isConvex(shape2Type)) ||
                 (!CollisionShape::isConvex(shape2Type) && CollisionShape::isConvex(shape1Type))) {

            NarrowPhaseInfo* narrowPhaseInfo = nullptr;
            computeConvexVsConcaveMiddlePhase(pair, mSingleFrameAllocator, &narrowPhaseInfo);

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
    }
}

// Compute the concave vs convex middle-phase algorithm for a given pair of bodies
void CollisionDetection::computeConvexVsConcaveMiddlePhase(OverlappingPair* pair, Allocator& allocator,
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

    // Compute the convex shape AABB in the local-space of the convex shape
    AABB aabb;
    convexShape->computeAABB(aabb, convexProxyShape->getLocalToWorldTransform());

    // TODO : Implement smooth concave mesh collision somewhere

    // Call the convex vs triangle callback for each triangle of the concave shape
    concaveShape->testAllTriangles(middlePhaseCallback, aabb);

    // Add all the narrow-phase info object reported by the callback into the
    // list of all the narrow-phase info object
    *firstNarrowPhaseInfo = middlePhaseCallback.narrowPhaseInfoList;
}

// Compute the narrow-phase collision detection
void CollisionDetection::computeNarrowPhase() {

    PROFILE("CollisionDetection::computeNarrowPhase()");

    const NarrowPhaseInfo* currentNarrowPhaseInfo = mNarrowPhaseInfoList;
    while (currentNarrowPhaseInfo != nullptr) {

        // Select the narrow phase algorithm to use according to the two collision shapes
        const CollisionShapeType shape1Type = currentNarrowPhaseInfo->collisionShape1->getType();
        const CollisionShapeType shape2Type = currentNarrowPhaseInfo->collisionShape2->getType();
        NarrowPhaseAlgorithm* narrowPhaseAlgorithm = selectNarrowPhaseAlgorithm(shape1Type, shape2Type);

        // If there is no collision algorithm between those two kinds of shapes, skip it
        if (narrowPhaseAlgorithm != nullptr) {

            // Use the narrow-phase collision detection algorithm to check
            // if there really is a collision. If a collision occurs, the
            // notifyContact() callback method will be called.
            ContactManifoldInfo contactManifoldInfo(mSingleFrameAllocator);
            if (narrowPhaseAlgorithm->testCollision(currentNarrowPhaseInfo, contactManifoldInfo)) {

                // Reduce the number of points in the contact manifold
                contactManifoldInfo.reduce();

                // If it is the first contact since the pairs are overlapping
                if (currentNarrowPhaseInfo->overlappingPair->getNbContactPoints() == 0) {

                    // Trigger a callback event
                    if (mWorld->mEventListener != nullptr) mWorld->mEventListener->beginContact(contactManifoldInfo);
                }

                // Add the contact manifold to the corresponding overlapping pair
                currentNarrowPhaseInfo->overlappingPair->addContactManifold(contactManifoldInfo);

                // Add the overlapping pair into the set of pairs in contact during narrow-phase
                overlappingpairid pairId = OverlappingPair::computeID(currentNarrowPhaseInfo->overlappingPair->getShape1(),
                                                                      currentNarrowPhaseInfo->overlappingPair->getShape2());
                mContactOverlappingPairs[pairId] = currentNarrowPhaseInfo->overlappingPair;

                // Trigger a callback event for the new contact
                if (mWorld->mEventListener != nullptr) mWorld->mEventListener->newContact(contactManifoldInfo);

                currentNarrowPhaseInfo->overlappingPair->getLastFrameCollisionInfo().wasColliding = true;
            }
            else {
                currentNarrowPhaseInfo->overlappingPair->getLastFrameCollisionInfo().wasColliding = false;
            }

            // The previous frame collision info is now valid
            currentNarrowPhaseInfo->overlappingPair->getLastFrameCollisionInfo().isValid = true;
        }

        currentNarrowPhaseInfo = currentNarrowPhaseInfo->next;
    }

    // Add all the contact manifolds (between colliding bodies) to the bodies
    addAllContactManifoldsToBodies();
}

// Allow the broadphase to notify the collision detection about an overlapping pair.
/// This method is called by the broad-phase collision detection algorithm
void CollisionDetection::broadPhaseNotifyOverlappingPair(ProxyShape* shape1, ProxyShape* shape2) {

    assert(shape1->mBroadPhaseID != shape2->mBroadPhaseID);

    // Check if the collision filtering allows collision between the two shapes
    if ((shape1->getCollideWithMaskBits() & shape2->getCollisionCategoryBits()) == 0 ||
        (shape1->getCollisionCategoryBits() & shape2->getCollideWithMaskBits()) == 0) return;

    // Compute the overlapping pair ID
    overlappingpairid pairID = OverlappingPair::computeID(shape1, shape2);

    // Check if the overlapping pair already exists
    if (mOverlappingPairs.find(pairID) != mOverlappingPairs.end()) return;

    // Compute the maximum number of contact manifolds for this pair
    int nbMaxManifolds = CollisionShape::computeNbMaxContactManifolds(shape1->getCollisionShape()->getType(),
                                                                      shape2->getCollisionShape()->getType());

    // Create the overlapping pair and add it into the set of overlapping pairs
    OverlappingPair* newPair = new (mWorld->mPoolAllocator.allocate(sizeof(OverlappingPair)))
                              OverlappingPair(shape1, shape2, nbMaxManifolds, mWorld->mPoolAllocator);
    assert(newPair != nullptr);

#ifndef NDEBUG
    std::pair<map<overlappingpairid, OverlappingPair*>::iterator, bool> check =
#endif
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
            mWorld->mPoolAllocator.release(itToRemove->second, sizeof(OverlappingPair));
            mOverlappingPairs.erase(itToRemove);
        }
        else {
            ++it;
        }
    }

    // Remove the body from the broad-phase
    mBroadPhaseAlgorithm.removeProxyCollisionShape(proxyShape);
}

void CollisionDetection::addAllContactManifoldsToBodies() {

    // For each overlapping pairs in contact during the narrow-phase
    std::map<overlappingpairid, OverlappingPair*>::iterator it;
    for (it = mContactOverlappingPairs.begin(); it != mContactOverlappingPairs.end(); ++it) {

        // Add all the contact manifolds of the pair into the list of contact manifolds
        // of the two bodies involved in the contact
        addContactManifoldToBody(it->second);
    }
}

// Add a contact manifold to the linked list of contact manifolds of the two bodies involved
// in the corresponding contact
void CollisionDetection::addContactManifoldToBody(OverlappingPair* pair) {

    assert(pair != nullptr);

    CollisionBody* body1 = pair->getShape1()->getBody();
    CollisionBody* body2 = pair->getShape2()->getBody();
    const ContactManifoldSet& manifoldSet = pair->getContactManifoldSet();

    // For each contact manifold in the set of manifolds in the pair
    for (int i=0; i<manifoldSet.getNbContactManifolds(); i++) {

        ContactManifold* contactManifold = manifoldSet.getContactManifold(i);

        assert(contactManifold->getNbContactPoints() > 0);

        // Add the contact manifold at the beginning of the linked
        // list of contact manifolds of the first body
        void* allocatedMemory1 = mWorld->mPoolAllocator.allocate(sizeof(ContactManifoldListElement));
        ContactManifoldListElement* listElement1 = new (allocatedMemory1)
                                                      ContactManifoldListElement(contactManifold,
                                                                         body1->mContactManifoldsList);
        body1->mContactManifoldsList = listElement1;

        // Add the contact manifold at the beginning of the linked
        // list of the contact manifolds of the second body
        void* allocatedMemory2 = mWorld->mPoolAllocator.allocate(sizeof(ContactManifoldListElement));
        ContactManifoldListElement* listElement2 = new (allocatedMemory2)
                                                      ContactManifoldListElement(contactManifold,
                                                                         body2->mContactManifoldsList);
        body2->mContactManifoldsList = listElement2;
    }
}

// Delete all the contact points in the currently overlapping pairs
void CollisionDetection::clearContactPoints() {

    // For each overlapping pair
    std::map<overlappingpairid, OverlappingPair*>::iterator it;
    for (it = mOverlappingPairs.begin(); it != mOverlappingPairs.end(); ++it) {
        it->second->clearContactPoints();
    }
}

// Compute the middle-phase collision detection between two proxy shapes
NarrowPhaseInfo* CollisionDetection::computeMiddlePhaseForProxyShapes(OverlappingPair* pair) {

    ProxyShape* shape1 = pair->getShape1();
    ProxyShape* shape2 = pair->getShape2();

    // -------------------------------------------------------

    const CollisionShapeType shape1Type = shape1->getCollisionShape()->getType();
    const CollisionShapeType shape2Type = shape2->getCollisionShape()->getType();

    NarrowPhaseInfo* narrowPhaseInfo = nullptr;

    // If both shapes are convex
    if ((CollisionShape::isConvex(shape1Type) && CollisionShape::isConvex(shape2Type))) {

        // No middle-phase is necessary, simply create a narrow phase info
        // for the narrow-phase collision detection
        narrowPhaseInfo = new (mMemoryAllocator.allocate(sizeof(NarrowPhaseInfo))) NarrowPhaseInfo(pair, shape1->getCollisionShape(),
                                       shape2->getCollisionShape(), shape1->getLocalToWorldTransform(),
                                       shape2->getLocalToWorldTransform(), shape1->getCachedCollisionData(),
                                       shape2->getCachedCollisionData());

    }
    // Concave vs Convex algorithm
    else if ((!CollisionShape::isConvex(shape1Type) && CollisionShape::isConvex(shape2Type)) ||
             (!CollisionShape::isConvex(shape2Type) && CollisionShape::isConvex(shape1Type))) {

        // Run the middle-phase collision detection algorithm to find the triangles of the concave
        // shape we need to use during the narrow-phase collision detection
        computeConvexVsConcaveMiddlePhase(pair, mMemoryAllocator, &narrowPhaseInfo);
    }

    return narrowPhaseInfo;
}

// Report all the bodies that overlap with the aabb in parameter
void CollisionDetection::testAABBOverlap(const AABB& aabb, OverlapCallback* overlapCallback,
                                         unsigned short categoryMaskBits) {
    assert(overlapCallback != nullptr);

    std::unordered_set<bodyindex> reportedBodies;

    // Ask the broad-phase to get all the overlapping shapes
    LinkedList<int> overlappingNodes(mMemoryAllocator);
    mBroadPhaseAlgorithm.reportAllShapesOverlappingWithAABB(aabb, overlappingNodes);

    // For each overlaping proxy shape
    LinkedList<int>::ListElement* element = overlappingNodes.getListHead();
    while (element != nullptr) {

        // Get the overlapping proxy shape
        int broadPhaseId = element->data;
        ProxyShape* proxyShape = mBroadPhaseAlgorithm.getProxyShapeForBroadPhaseId(broadPhaseId);

        CollisionBody* overlapBody = proxyShape->getBody();

        // If the proxy shape is from a body that we have not already reported collision
        if (reportedBodies.find(overlapBody->getID()) == reportedBodies.end()) {

            // Check if the collision filtering allows collision between the two shapes
            if ((proxyShape->getCollisionCategoryBits() & categoryMaskBits) != 0) {

                // Add the body into the set of reported bodies
                reportedBodies.insert(overlapBody->getID());

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

                const CollisionShapeType shape1Type = body1ProxyShape->getCollisionShape()->getType();
                const CollisionShapeType shape2Type = body2ProxyShape->getCollisionShape()->getType();

                // Create a temporary overlapping pair
                OverlappingPair pair(body1ProxyShape, body2ProxyShape, 0, mMemoryAllocator);

                // Compute the middle-phase collision detection between the two shapes
                NarrowPhaseInfo* narrowPhaseInfo = computeMiddlePhaseForProxyShapes(&pair);

                bool isColliding = false;

                // For each narrow-phase info object
                while (narrowPhaseInfo != nullptr) {

                    // If we have not found a collision yet
                    if (!isColliding) {

                        // Select the narrow phase algorithm to use according to the two collision shapes
                        NarrowPhaseAlgorithm* narrowPhaseAlgorithm = selectNarrowPhaseAlgorithm(shape1Type, shape2Type);

                        // If there is a collision algorithm for those two kinds of shapes
                        if (narrowPhaseAlgorithm != nullptr) {

                            // Use the narrow-phase collision detection algorithm to check
                            // if there really is a collision. If a collision occurs, the
                            // notifyContact() callback method will be called.
                            ContactManifoldInfo contactManifoldInfo(mMemoryAllocator);
                            isColliding |= narrowPhaseAlgorithm->testCollision(narrowPhaseInfo, contactManifoldInfo);
                        }
                    }

                    NarrowPhaseInfo* currentNarrowPhaseInfo = narrowPhaseInfo;
                    narrowPhaseInfo = narrowPhaseInfo->next;

                    // Release the allocated memory
                    mMemoryAllocator.release(currentNarrowPhaseInfo, sizeof(NarrowPhaseInfo));
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

    std::unordered_set<bodyindex> reportedBodies;

    // For each proxy shape proxy shape of the body
    ProxyShape* bodyProxyShape = body->getProxyShapesList();
    while (bodyProxyShape != nullptr) {

        // Get the AABB of the shape
        const AABB& shapeAABB = mBroadPhaseAlgorithm.getFatAABB(bodyProxyShape->mBroadPhaseID);

        // Ask the broad-phase to get all the overlapping shapes
        LinkedList<int> overlappingNodes(mMemoryAllocator);
        mBroadPhaseAlgorithm.reportAllShapesOverlappingWithAABB(shapeAABB, overlappingNodes);

        const bodyindex bodyId = body->getID();

        // For each overlaping proxy shape
        LinkedList<int>::ListElement* element = overlappingNodes.getListHead();
        while (element != nullptr) {

            // Get the overlapping proxy shape
            int broadPhaseId = element->data;
            ProxyShape* proxyShape = mBroadPhaseAlgorithm.getProxyShapeForBroadPhaseId(broadPhaseId);

            // If the proxy shape is from a body that we have not already reported collision and the
            // two proxy collision shapes are not from the same body
            if (reportedBodies.find(proxyShape->getBody()->getID()) == reportedBodies.end() &&
                proxyShape->getBody()->getID() != bodyId) {

                // Check if the collision filtering allows collision between the two shapes
                if ((proxyShape->getCollisionCategoryBits() & categoryMaskBits) != 0) {

                    const CollisionShapeType shape1Type = bodyProxyShape->getCollisionShape()->getType();
                    const CollisionShapeType shape2Type = proxyShape->getCollisionShape()->getType();

                    // Create a temporary overlapping pair
                    OverlappingPair pair(bodyProxyShape, proxyShape, 0, mMemoryAllocator);

                    // Compute the middle-phase collision detection between the two shapes
                    NarrowPhaseInfo* narrowPhaseInfo = computeMiddlePhaseForProxyShapes(&pair);

                    bool isColliding = false;

                    // For each narrow-phase info object
                    while (narrowPhaseInfo != nullptr) {

                        // If we have not found a collision yet
                        if (!isColliding) {

                            // Select the narrow phase algorithm to use according to the two collision shapes
                            NarrowPhaseAlgorithm* narrowPhaseAlgorithm = selectNarrowPhaseAlgorithm(shape1Type, shape2Type);

                            // If there is a collision algorithm for those two kinds of shapes
                            if (narrowPhaseAlgorithm != nullptr) {

                                // Use the narrow-phase collision detection algorithm to check
                                // if there really is a collision. If a collision occurs, the
                                // notifyContact() callback method will be called.
                                ContactManifoldInfo contactManifoldInfo(mMemoryAllocator);
                                isColliding |= narrowPhaseAlgorithm->testCollision(narrowPhaseInfo, contactManifoldInfo);
                            }
                        }

                        NarrowPhaseInfo* currentNarrowPhaseInfo = narrowPhaseInfo;
                        narrowPhaseInfo = narrowPhaseInfo->next;

                        // Release the allocated memory
                        mMemoryAllocator.release(currentNarrowPhaseInfo, sizeof(NarrowPhaseInfo));
                    }

                    // Return if we have found a narrow-phase collision
                    if (isColliding) {

                        CollisionBody* overlapBody = proxyShape->getBody();

                        // Add the body into the set of reported bodies
                        reportedBodies.insert(overlapBody->getID());

                        // Notify the overlap to the user
                        overlapCallback->notifyOverlap(overlapBody);
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
                OverlappingPair pair(body1ProxyShape, body2ProxyShape, 0, mMemoryAllocator);

                // Compute the middle-phase collision detection between the two shapes
                NarrowPhaseInfo* narrowPhaseInfo = computeMiddlePhaseForProxyShapes(&pair);

                const CollisionShapeType shape1Type = body1ProxyShape->getCollisionShape()->getType();
                const CollisionShapeType shape2Type = body2ProxyShape->getCollisionShape()->getType();

                // For each narrow-phase info object
                while (narrowPhaseInfo != nullptr) {

                    // Select the narrow phase algorithm to use according to the two collision shapes
                    NarrowPhaseAlgorithm* narrowPhaseAlgorithm = selectNarrowPhaseAlgorithm(shape1Type, shape2Type);

                    // If there is a collision algorithm for those two kinds of shapes
                    if (narrowPhaseAlgorithm != nullptr) {

                        // Use the narrow-phase collision detection algorithm to check
                        // if there really is a collision. If a collision occurs, the
                        // notifyContact() callback method will be called.
                        ContactManifoldInfo contactManifoldInfo(mMemoryAllocator);
                        if (narrowPhaseAlgorithm->testCollision(narrowPhaseInfo, contactManifoldInfo)) {

                            CollisionCallback::CollisionCallbackInfo collisionInfo(contactManifoldInfo, body1, body2,
                                                                                   body1ProxyShape, body2ProxyShape);

                            // Report the contact to the user
                            collisionCallback->notifyContact(collisionInfo);
                        }
                    }

                    NarrowPhaseInfo* currentNarrowPhaseInfo = narrowPhaseInfo;
                    narrowPhaseInfo = narrowPhaseInfo->next;

                    // Release the allocated memory
                    mMemoryAllocator.release(currentNarrowPhaseInfo, sizeof(NarrowPhaseInfo));
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

        // Get the AABB of the shape
        const AABB& shapeAABB = mBroadPhaseAlgorithm.getFatAABB(bodyProxyShape->mBroadPhaseID);

        // Ask the broad-phase to get all the overlapping shapes
        LinkedList<int> overlappingNodes(mMemoryAllocator);
        mBroadPhaseAlgorithm.reportAllShapesOverlappingWithAABB(shapeAABB, overlappingNodes);

        const bodyindex bodyId = body->getID();

        // For each overlaping proxy shape
        LinkedList<int>::ListElement* element = overlappingNodes.getListHead();
        while (element != nullptr) {

            // Get the overlapping proxy shape
            int broadPhaseId = element->data;
            ProxyShape* proxyShape = mBroadPhaseAlgorithm.getProxyShapeForBroadPhaseId(broadPhaseId);

            // If the two proxy collision shapes are not from the same body
            if (proxyShape->getBody()->getID() != bodyId) {

                // Check if the collision filtering allows collision between the two shapes
                if ((proxyShape->getCollisionCategoryBits() & categoryMaskBits) != 0) {

                    const CollisionShapeType shape1Type = bodyProxyShape->getCollisionShape()->getType();
                    const CollisionShapeType shape2Type = proxyShape->getCollisionShape()->getType();

                    // Create a temporary overlapping pair
                    OverlappingPair pair(bodyProxyShape, proxyShape, 0, mMemoryAllocator);

                    // Compute the middle-phase collision detection between the two shapes
                    NarrowPhaseInfo* narrowPhaseInfo = computeMiddlePhaseForProxyShapes(&pair);

                    // For each narrow-phase info object
                    while (narrowPhaseInfo != nullptr) {

                        // Select the narrow phase algorithm to use according to the two collision shapes
                        NarrowPhaseAlgorithm* narrowPhaseAlgorithm = selectNarrowPhaseAlgorithm(shape1Type, shape2Type);

                        // If there is a collision algorithm for those two kinds of shapes
                        if (narrowPhaseAlgorithm != nullptr) {

                            // Use the narrow-phase collision detection algorithm to check
                            // if there really is a collision. If a collision occurs, the
                            // notifyContact() callback method will be called.
                            ContactManifoldInfo contactManifoldInfo(mMemoryAllocator);
                            if (narrowPhaseAlgorithm->testCollision(narrowPhaseInfo, contactManifoldInfo)) {

                                CollisionCallback::CollisionCallbackInfo collisionInfo(contactManifoldInfo, body,
                                                                                       proxyShape->getBody(), bodyProxyShape,
                                                                                       proxyShape);

                                // Report the contact to the user
                                callback->notifyContact(collisionInfo);
                            }
                        }

                        NarrowPhaseInfo* currentNarrowPhaseInfo = narrowPhaseInfo;
                        narrowPhaseInfo = narrowPhaseInfo->next;

                        // Release the allocated memory
                        mMemoryAllocator.release(currentNarrowPhaseInfo, sizeof(NarrowPhaseInfo));
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

// Test and report collisions between all shapes of the world
void CollisionDetection::testCollision(CollisionCallback* callback) {

    assert(callback != nullptr);

    // Compute the broad-phase collision detection
    computeBroadPhase();

    // For each possible collision pair of bodies
    map<overlappingpairid, OverlappingPair*>::iterator it;
    for (it = mOverlappingPairs.begin(); it != mOverlappingPairs.end(); ++it) {

        OverlappingPair* pair = it->second;

        ProxyShape* shape1 = pair->getShape1();
        ProxyShape* shape2 = pair->getShape2();

        // Check if the collision filtering allows collision between the two shapes and
        // that the two shapes are still overlapping.
        if (((shape1->getCollideWithMaskBits() & shape2->getCollisionCategoryBits()) != 0 &&
             (shape1->getCollisionCategoryBits() & shape2->getCollideWithMaskBits()) != 0) &&
             mBroadPhaseAlgorithm.testOverlappingShapes(shape1, shape2)) {

            // Compute the middle-phase collision detection between the two shapes
            NarrowPhaseInfo* narrowPhaseInfo = computeMiddlePhaseForProxyShapes(pair);

            const CollisionShapeType shape1Type = shape1->getCollisionShape()->getType();
            const CollisionShapeType shape2Type = shape2->getCollisionShape()->getType();

            // For each narrow-phase info object
            while (narrowPhaseInfo != nullptr) {

                // Select the narrow phase algorithm to use according to the two collision shapes
                NarrowPhaseAlgorithm* narrowPhaseAlgorithm = selectNarrowPhaseAlgorithm(shape1Type, shape2Type);

                // If there is a collision algorithm for those two kinds of shapes
                if (narrowPhaseAlgorithm != nullptr) {

                    // Use the narrow-phase collision detection algorithm to check
                    // if there really is a collision. If a collision occurs, the
                    // notifyContact() callback method will be called.
                    ContactManifoldInfo contactManifoldInfo(mMemoryAllocator);
                    if (narrowPhaseAlgorithm->testCollision(narrowPhaseInfo, contactManifoldInfo)) {

                        CollisionCallback::CollisionCallbackInfo collisionInfo(contactManifoldInfo, shape1->getBody(),
                                                                               shape2->getBody(), shape1, shape2);

                        // Report the contact to the user
                        callback->notifyContact(collisionInfo);
                    }

                    // The previous frame collision info is now valid
                    narrowPhaseInfo->overlappingPair->getLastFrameCollisionInfo().isValid = true;
                }

                NarrowPhaseInfo* currentNarrowPhaseInfo = narrowPhaseInfo;
                narrowPhaseInfo = narrowPhaseInfo->next;

                // Release the allocated memory
                mMemoryAllocator.release(currentNarrowPhaseInfo, sizeof(NarrowPhaseInfo));
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

/// Return a reference to the world memory allocator
PoolAllocator& CollisionDetection::getWorldMemoryAllocator() {
  return mWorld->mPoolAllocator;
}
