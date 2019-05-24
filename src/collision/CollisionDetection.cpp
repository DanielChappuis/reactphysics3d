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
#include "collision/ContactManifoldInfo.h"
#include "constraint/ContactPoint.h"
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
#include "engine/Islands.h"
#include <cassert>
#include <iostream>

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;
using namespace std;


// Constructor
CollisionDetection::CollisionDetection(CollisionWorld* world, ProxyShapeComponents& proxyShapesComponents, TransformComponents& transformComponents,
                                       DynamicsComponents& dynamicsComponents, MemoryManager& memoryManager)
                   : mMemoryManager(memoryManager), mProxyShapesComponents(proxyShapesComponents),
                     mCollisionDispatch(mMemoryManager.getPoolAllocator()), mWorld(world),
                     mOverlappingPairs(mMemoryManager.getPoolAllocator()),
                     mBroadPhaseSystem(*this, mProxyShapesComponents, transformComponents, dynamicsComponents),
                     mNoCollisionPairs(mMemoryManager.getPoolAllocator()), mMapBroadPhaseIdToProxyShapeEntity(memoryManager.getPoolAllocator()),
                     mNarrowPhaseInput(mMemoryManager.getSingleFrameAllocator()), mPotentialContactPoints(mMemoryManager.getSingleFrameAllocator()),
                     // TODO : We should probably use single frame allocator for mPotentialContactPoints, mPotentialContactManifolds,  mMapPairIdToOverlappingPairContacts
                     mPotentialContactManifolds(mMemoryManager.getSingleFrameAllocator()), mContactPairs1(mMemoryManager.getPoolAllocator()),
                     mContactPairs2(mMemoryManager.getPoolAllocator()), mPreviousContactPairs(&mContactPairs1), mCurrentContactPairs(&mContactPairs2),
                     mMapPairIdToContactPairIndex1(mMemoryManager.getPoolAllocator()), mMapPairIdToContactPairIndex2(mMemoryManager.getPoolAllocator()),
                     mPreviousMapPairIdToContactPairIndex(&mMapPairIdToContactPairIndex1), mCurrentMapPairIdToContactPairIndex(&mMapPairIdToContactPairIndex2),
                     mContactPairsIndicesOrderingForContacts(memoryManager.getSingleFrameAllocator()),
                     mContactManifolds1(mMemoryManager.getPoolAllocator()), mContactManifolds2(mMemoryManager.getPoolAllocator()),
                     mPreviousContactManifolds(&mContactManifolds1), mCurrentContactManifolds(&mContactManifolds2),
                     mContactPoints1(mMemoryManager.getPoolAllocator()),
                     mContactPoints2(mMemoryManager.getPoolAllocator()), mPreviousContactPoints(&mContactPoints1),
                     mCurrentContactPoints(&mContactPoints2), mMapBodyToContactPairs(mMemoryManager.getSingleFrameAllocator()) {

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

    // Ask the broad-phase to compute all the shapes overlapping the shapes that
    // have moved or have been added in the last frame. This call can only add new
    // overlapping pairs in the collision detection.
    List<Pair<int, int>> overlappingNodes(mMemoryManager.getPoolAllocator(), 32);
    mBroadPhaseSystem.computeOverlappingPairs(mMemoryManager, overlappingNodes);

    // Create new overlapping pairs if necessary
    updateOverlappingPairs(overlappingNodes);

    // Remove non overlapping pairs
    removeNonOverlappingPairs();
}

// Remove pairs that are not overlapping anymore
void CollisionDetection::removeNonOverlappingPairs() {

    // For each possible collision pair of bodies
    for (auto it = mOverlappingPairs.begin(); it != mOverlappingPairs.end(); ) {

        OverlappingPair* pair = it->second;

        ProxyShape* shape1 = pair->getShape1();
        ProxyShape* shape2 = pair->getShape2();

        // Check if the two shapes are still overlapping. Otherwise, we destroy the overlapping pair
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
    }
}

// Take a list of overlapping nodes in the broad-phase and create new overlapping pairs if necessary
void CollisionDetection::updateOverlappingPairs(List<Pair<int, int>>& overlappingNodes) {

    List<OverlappingPair*> newOverlappingPairs(mMemoryManager.getPoolAllocator(), overlappingNodes.size());

    // For each overlapping pair of nodes
    for (uint i=0; i < overlappingNodes.size(); i++) {

        Pair<int, int> nodePair = overlappingNodes[i];

        assert(nodePair.first != -1);
        assert(nodePair.second != -1);

        // Skip pairs with same overlapping nodes
        if (nodePair.first != nodePair.second) {

            // Get the two proxy-shapes
            Entity proxyShape1Entity = mMapBroadPhaseIdToProxyShapeEntity[nodePair.first];
            Entity proxyShape2Entity = mMapBroadPhaseIdToProxyShapeEntity[nodePair.second];

            // Get the two bodies
            Entity body1Entity = mProxyShapesComponents.getBody(proxyShape1Entity);
            Entity body2Entity = mProxyShapesComponents.getBody(proxyShape2Entity);

            // If the two proxy collision shapes are from the same body, skip it
            if (body1Entity != body2Entity) {

                // Compute the overlapping pair ID
                Pair<uint, uint> pairID = OverlappingPair::computeID(nodePair.first, nodePair.second);

                // Check if the overlapping pair already exists
                if (!mOverlappingPairs.containsKey(pairID)) {

                    unsigned short shape1CollideWithMaskBits = mProxyShapesComponents.getCollideWithMaskBits(proxyShape1Entity);
                    unsigned short shape2CollideWithMaskBits = mProxyShapesComponents.getCollideWithMaskBits(proxyShape2Entity);

                    unsigned short shape1CollisionCategoryBits = mProxyShapesComponents.getCollisionCategoryBits(proxyShape1Entity);
                    unsigned short shape2CollisionCategoryBits = mProxyShapesComponents.getCollisionCategoryBits(proxyShape2Entity);

                    // Check if the collision filtering allows collision between the two shapes
                    if ((shape1CollideWithMaskBits & shape2CollisionCategoryBits) != 0 &&
                        (shape1CollisionCategoryBits & shape2CollideWithMaskBits) != 0) {

                        ProxyShape* shape1 = mProxyShapesComponents.getProxyShape(proxyShape1Entity);
                        ProxyShape* shape2 = mProxyShapesComponents.getProxyShape(proxyShape2Entity);

                        // Create the overlapping pair and add it into the set of overlapping pairs
                        OverlappingPair* newPair = new (mMemoryManager.allocate(MemoryManager::AllocationType::Pool, sizeof(OverlappingPair)))
                                                  OverlappingPair(shape1, shape2, mMemoryManager.getPoolAllocator(),
                                                                  mMemoryManager.getSingleFrameAllocator(), mWorld->mConfig);

                        assert(newPair != nullptr);

                        // Add the new overlapping pair
                        mOverlappingPairs.add(Pair<Pair<uint, uint>, OverlappingPair*>(pairID, newPair));
                        newOverlappingPairs.add(newPair);
                    }
                }
            }
        }
    }
}

// Compute the middle-phase collision detection
void CollisionDetection::computeMiddlePhase() {

    RP3D_PROFILE("CollisionDetection::computeMiddlePhase()", mProfiler);

    // Reserve memory for the narrow-phase input using cached capacity from previous frame
    mNarrowPhaseInput.reserveMemory();

    // For each possible collision pair of bodies
    for (auto it = mOverlappingPairs.begin(); it != mOverlappingPairs.end(); ++it) {

        OverlappingPair* pair = it->second;

        // Make all the last frame collision info obsolete
        pair->makeLastFrameCollisionInfosObsolete();

        const Entity proxyShape1Entity = pair->getShape1()->getEntity();
        const Entity proxyShape2Entity = pair->getShape2()->getEntity();

        ProxyShape* shape1 = pair->getShape1();
        ProxyShape* shape2 = pair->getShape2();

        assert(mProxyShapesComponents.getBroadPhaseId(proxyShape1Entity) != -1);
        assert(mProxyShapesComponents.getBroadPhaseId(proxyShape2Entity) != -1);
        assert(mProxyShapesComponents.getBroadPhaseId(proxyShape1Entity) != mProxyShapesComponents.getBroadPhaseId(proxyShape2Entity));

        // Check if the collision filtering allows collision between the two shapes
        if ((mProxyShapesComponents.getCollideWithMaskBits(proxyShape1Entity) & mProxyShapesComponents.getCollisionCategoryBits(proxyShape2Entity)) != 0 &&
            (mProxyShapesComponents.getCollisionCategoryBits(proxyShape1Entity) & mProxyShapesComponents.getCollideWithMaskBits(proxyShape2Entity)) != 0) {

            CollisionBody* const body1 = shape1->getBody();
            CollisionBody* const body2 = shape2->getBody();

            const Entity body1Entity = body1->getEntity();
            const Entity body2Entity = body2->getEntity();

            // Check that at least one body is awake and not static
            bool isBody1Active = !mWorld->mBodyComponents.getIsEntityDisabled(body1Entity) && body1->getType() != BodyType::STATIC;
            bool isBody2Active = !mWorld->mBodyComponents.getIsEntityDisabled(body2Entity) && body2->getType() != BodyType::STATIC;
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

    assert(mCurrentContactPairs->size() == 0);
    assert(mCurrentMapPairIdToContactPairIndex->size() == 0);

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

    // Swap the previous and current contacts lists
    swapPreviousAndCurrentContacts();

    // Test the narrow-phase collision detection on the batches to be tested
    testNarrowPhaseCollision(mNarrowPhaseInput, false, true, allocator);

    // Process all the potential contacts after narrow-phase collision
    processAllPotentialContacts(mNarrowPhaseInput, true);

    // Reduce the number of contact points in the manifolds
    reducePotentialContactManifolds();

    // Report contacts to the user
    reportAllContacts();

    assert(mCurrentContactManifolds->size() == 0);
    assert(mCurrentContactPoints->size() == 0);

    mContactPairsIndicesOrderingForContacts.reserve(mCurrentContactPairs->size());

    mNarrowPhaseInput.clear();
}

// Swap the previous and current contacts lists
void CollisionDetection::swapPreviousAndCurrentContacts() {

    if (mPreviousContactPairs == &mContactPairs1) {

        mPreviousContactPairs = &mContactPairs2;
        mPreviousContactManifolds = &mContactManifolds2;
        mPreviousContactPoints = &mContactPoints2;
        mPreviousMapPairIdToContactPairIndex = &mMapPairIdToContactPairIndex2;

        mCurrentContactPairs = &mContactPairs1;
        mCurrentContactManifolds = &mContactManifolds1;
        mCurrentContactPoints = &mContactPoints1;
        mCurrentMapPairIdToContactPairIndex = &mMapPairIdToContactPairIndex1;
    }
    else {

        mPreviousContactPairs = &mContactPairs1;
        mPreviousContactManifolds = &mContactManifolds1;
        mPreviousContactPoints = &mContactPoints1;
        mPreviousMapPairIdToContactPairIndex = &mMapPairIdToContactPairIndex1;

        mCurrentContactPairs = &mContactPairs2;
        mCurrentContactManifolds = &mContactManifolds2;
        mCurrentContactPoints = &mContactPoints2;
        mCurrentMapPairIdToContactPairIndex = &mMapPairIdToContactPairIndex2;
    }
}

// Create the actual contact manifolds and contacts points
/// List of the indices of the contact pairs (in mCurrentContacPairs array) with contact pairs of
/// same islands packed together linearly and contact pairs that are not part of islands at the end.
/// This is used when we create contact manifolds and contact points so that there are also packed
/// together linearly if they are part of the same island.
void CollisionDetection::createContacts() {

    RP3D_PROFILE("CollisionDetection::createContacts()", mProfiler);

    assert(mCurrentContactPairs->size() == mContactPairsIndicesOrderingForContacts.size());

    mCurrentContactManifolds->reserve(mCurrentContactPairs->size());
    mCurrentContactPoints->reserve(mCurrentContactManifolds->size());

    // For each contact pair
    for (uint p=0; p < mContactPairsIndicesOrderingForContacts.size(); p++) {

        ContactPair& contactPair = (*mCurrentContactPairs)[mContactPairsIndicesOrderingForContacts[p]];
        assert(contactPair.potentialContactManifoldsIndices.size() > 0);

        contactPair.contactManifoldsIndex = mCurrentContactManifolds->size();
        contactPair.nbContactManifolds = contactPair.potentialContactManifoldsIndices.size();
        contactPair.contactPointsIndex = mCurrentContactPoints->size();

        // For each potential contact manifold of the pair
        for (uint m=0; m < contactPair.potentialContactManifoldsIndices.size(); m++) {

            ContactManifoldInfo& potentialManifold = mPotentialContactManifolds[contactPair.potentialContactManifoldsIndices[m]];

            // Start index and number of contact points for this manifold
            const uint contactPointsIndex = mCurrentContactPoints->size();
            const int8 nbContactPoints = static_cast<int8>(potentialManifold.potentialContactPointsIndices.size());
            contactPair.nbToTalContactPoints += nbContactPoints;

            // We create a new contact manifold
            ContactManifold contactManifold(contactPair.body1Entity, contactPair.body2Entity, contactPair.proxyShape1Entity,
                                            contactPair.proxyShape2Entity, contactPointsIndex, nbContactPoints);

            // Add the contact manifold
            mCurrentContactManifolds->add(contactManifold);

            assert(potentialManifold.potentialContactPointsIndices.size() > 0);

            // For each contact point of the manifold
            for (uint c=0; c < potentialManifold.potentialContactPointsIndices.size(); c++) {

                ContactPointInfo& potentialContactPoint = mPotentialContactPoints[potentialManifold.potentialContactPointsIndices[c]];

                // Create a new contact point
                ContactPoint contactPoint(potentialContactPoint, mWorld->mConfig);

                // Add the contact point
                mCurrentContactPoints->add(contactPoint);
            }
        }
    }

    // Initialize the current contacts with the contacts from the previous frame (for warmstarting)
    initContactsWithPreviousOnes();

    // Reset the potential contacts
    mPotentialContactPoints.clear(true);
    mPotentialContactManifolds.clear(true);
    mContactPairsIndicesOrderingForContacts.clear(true);
}

// Initialize the current contacts with the contacts from the previous frame (for warmstarting)
void CollisionDetection::initContactsWithPreviousOnes() {

    // For each contact pair of the current frame
    for (uint i=0; i < mCurrentContactPairs->size(); i++) {

        ContactPair& currentContactPair = (*mCurrentContactPairs)[i];

        // Find the corresponding contact pair in the previous frame (if any)
        auto itPrevContactPair = mPreviousMapPairIdToContactPairIndex->find(currentContactPair.pairId);

        // If we have found a corresponding contact pair in the previous frame
        if (itPrevContactPair != mPreviousMapPairIdToContactPairIndex->end()) {

            const uint previousContactPairIndex = itPrevContactPair->second;
            ContactPair& previousContactPair = (*mPreviousContactPairs)[previousContactPairIndex];

            // --------------------- Contact Manifolds --------------------- //

            const uint contactManifoldsIndex = currentContactPair.contactManifoldsIndex;
            const uint nbContactManifolds = currentContactPair.nbContactManifolds;

            // For each current contact manifold of the contact pair
            for (uint m=contactManifoldsIndex; m < contactManifoldsIndex + nbContactManifolds; m++) {

                assert(m < mCurrentContactManifolds->size());
                ContactManifold& currentContactManifold = (*mCurrentContactManifolds)[m];
                assert(currentContactManifold.mNbContactPoints > 0);
                ContactPoint& currentContactPoint = (*mCurrentContactPoints)[currentContactManifold.mContactPointsIndex];
                const Vector3& currentContactPointNormal = currentContactPoint.getNormal();

                // Find a similar contact manifold among the contact manifolds from the previous frame (for warmstarting)
                const uint previousContactManifoldIndex = previousContactPair.contactManifoldsIndex;
                const uint previousNbContactManifolds = previousContactPair.nbContactManifolds;
                for (uint p=previousContactManifoldIndex; p < previousContactManifoldIndex + previousNbContactManifolds; p++) {

                    ContactManifold& previousContactManifold = (*mPreviousContactManifolds)[p];
                    assert(previousContactManifold.mNbContactPoints > 0);
                    ContactPoint& previousContactPoint = (*mPreviousContactPoints)[previousContactManifold.mContactPointsIndex];

                    // If the previous contact manifold has a similar contact normal with the current manifold
                    if (previousContactPoint.getNormal().dot(currentContactPointNormal) >= mWorld->mConfig.cosAngleSimilarContactManifold) {

                        // Transfer data from the previous contact manifold to the current one
                        currentContactManifold.mFrictionVector1 = previousContactManifold.mFrictionVector1;
                        currentContactManifold.mFrictionVector2 = previousContactManifold.mFrictionVector2;
                        currentContactManifold.mFrictionImpulse1 = previousContactManifold.mFrictionImpulse1;
                        currentContactManifold.mFrictionImpulse2 = previousContactManifold.mFrictionImpulse2;
                        currentContactManifold.mFrictionTwistImpulse = previousContactManifold.mFrictionTwistImpulse;
                        currentContactManifold.mRollingResistanceImpulse = previousContactManifold.mRollingResistanceImpulse;

                        break;
                    }
                }
            }

            // --------------------- Contact Points --------------------- //

            const uint contactPointsIndex = currentContactPair.contactPointsIndex;
            const uint nbTotalContactPoints = currentContactPair.nbToTalContactPoints;

            // For each current contact point of the contact pair
            for (uint c=contactPointsIndex; c < contactPointsIndex + nbTotalContactPoints; c++) {

                assert(c < mCurrentContactPoints->size());
                ContactPoint& currentContactPoint = (*mCurrentContactPoints)[c];

                // Find a similar contact point among the contact points from the previous frame (for warmstarting)
                const uint previousContactPointsIndex = previousContactPair.contactPointsIndex;
                const uint previousNbContactPoints = previousContactPair.nbToTalContactPoints;
                for (uint p=previousContactPointsIndex; p < previousContactPointsIndex + previousNbContactPoints; p++) {

                    ContactPoint& previousContactPoint = (*mPreviousContactPoints)[p];

                    // If the previous contact point is very close to th current one
                    const decimal distSquare = (currentContactPoint.getLocalPointOnShape1() - previousContactPoint.getLocalPointOnShape1()).lengthSquare();
                    if (distSquare <= mWorld->mConfig.persistentContactDistanceThreshold * mWorld->mConfig.persistentContactDistanceThreshold) {

                        // Transfer data from the previous contact point to the current one
                        currentContactPoint.setPenetrationImpulse(previousContactPoint.getPenetrationImpulse());
                        currentContactPoint.setIsRestingContact(previousContactPoint.getIsRestingContact());

                        break;
                    }
                }
            }
        }
    }

    mPreviousContactPoints->clear();
    mPreviousContactManifolds->clear();
    mPreviousContactPairs->clear();
    mPreviousMapPairIdToContactPairIndex->clear();

    /*
    // TODO : DELETE THIS
    std::cout << "_______________ RECAP ACTUAL CONTACTS___________________" << std::endl;
    std::cout << "ContactPairs :" << std::endl;
    for (uint i=0; i < mCurrentContactPairs->size(); i++) {

        ContactPair& pair = (*mCurrentContactPairs)[i];
        std::cout << "  PairId : (" << pair.pairId.first << ", " << pair.pairId.second << std::endl;
        std::cout << "  Index : " << i << std::endl;
        std::cout << "  ContactManifoldsIndex : " << pair.contactManifoldsIndex << std::endl;
        std::cout << "  nbManifolds : " << pair.nbContactManifolds << std::endl;
        std::cout << "  ContactPointsIndex : " << pair.contactPointsIndex << std::endl;
        std::cout << "  nbTotalPoints : " << pair.nbToTalContactPoints << std::endl;

    }
    std::cout << "ContactManifolds :" << std::endl;
    for (uint i=0; i < mCurrentContactManifolds->size(); i++) {

        ContactManifold& manifold = (*mCurrentContactManifolds)[i];
        std::cout << "  Index : " << i << std::endl;
        std::cout << "  >>ContactPointsIndex : " << manifold.mContactPointsIndex << std::endl;
        std::cout << "  >>Nb Contact Points : " << manifold.mNbContactPoints << std::endl;
    }
    std::cout << "ContactPoints :" << std::endl;
    for (uint i=0; i < mCurrentContactPoints->size(); i++) {

        ContactPoint& contactPoint = (*mCurrentContactPoints)[i];
        std::cout << "  Index : " << i << std::endl;
        std::cout << "  Point : (" << contactPoint.getLocalPointOnShape1().x << ", " << contactPoint.getLocalPointOnShape1().y << ", " << contactPoint.getLocalPointOnShape1().z << std::endl;
    }
    std::cout << "mCurrentMapPairIdToContactPairIndex :" << std::endl;
    for (auto it = mCurrentMapPairIdToContactPairIndex->begin(); it != mCurrentMapPairIdToContactPairIndex->end(); ++it) {

        OverlappingPair::OverlappingPairId pairId = it->first;
        uint index = it->second;
        std::cout << "  PairId : " << pairId.first << ", " << pairId.second << std::endl;
        std::cout << "  ContactPair Index : " << index << std::endl;
    }
    */
}

// Remove a body from the collision detection
void CollisionDetection::removeProxyCollisionShape(ProxyShape* proxyShape) {

    assert(proxyShape->getBroadPhaseId() != -1);
    assert(mMapBroadPhaseIdToProxyShapeEntity.containsKey(proxyShape->getBroadPhaseId()));

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

    mMapBroadPhaseIdToProxyShapeEntity.remove(proxyShape->getBroadPhaseId());

    // Remove the body from the broad-phase
    mBroadPhaseSystem.removeProxyCollisionShape(proxyShape);
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

/// Convert the potential contact into actual contacts
void CollisionDetection::processPotentialContacts(NarrowPhaseInfoBatch& narrowPhaseInfoBatch, bool updateLastFrameInfo) {

    RP3D_PROFILE("CollisionDetection::processPotentialContacts()", mProfiler);

    // For each narrow phase info object
    for(uint i=0; i < narrowPhaseInfoBatch.getNbObjects(); i++) {

        if (updateLastFrameInfo) {
            narrowPhaseInfoBatch.lastFrameCollisionInfos[i]->wasColliding = narrowPhaseInfoBatch.isColliding[i];

            // The previous frame collision info is now valid
            narrowPhaseInfoBatch.lastFrameCollisionInfos[i]->isValid = true;
        }

        // Add the potential contacts
        for (uint j=0; j < narrowPhaseInfoBatch.contactPoints[i].size(); j++) {

            assert(narrowPhaseInfoBatch.isColliding[i]);

            const ContactPointInfo& contactPoint = *(narrowPhaseInfoBatch.contactPoints[i][j]);

            // Add the contact point to the list of potential contact points
            const uint contactPointIndex = static_cast<uint>(mPotentialContactPoints.size());

            // TODO : We should probably use single frame allocator here for mPotentialContactPoints
            //        If so, do not forget to call mPotentialContactPoints.clear(true) at the end of frame
            mPotentialContactPoints.add(contactPoint);

            bool similarManifoldFound = false;

            // If there is already a contact pair for this overlapping pair
            OverlappingPair::OverlappingPairId pairId = narrowPhaseInfoBatch.overlappingPairs[i]->getId();
            auto it = mCurrentMapPairIdToContactPairIndex->find(pairId);
            ContactPair* pairContact = nullptr;
            if (it != mCurrentMapPairIdToContactPairIndex->end()) {

                assert(it->first == pairId);

                const uint pairContactIndex = it->second;
                pairContact = &((*mCurrentContactPairs)[pairContactIndex]);

                assert(pairContact->potentialContactManifoldsIndices.size() > 0);

                // For each contact manifold of the overlapping pair
                for (uint m=0; m < pairContact->potentialContactManifoldsIndices.size(); m++) {

                   uint contactManifoldIndex = pairContact->potentialContactManifoldsIndices[m];

                   // Get the first contact point of the current manifold
                   assert(mPotentialContactManifolds[contactManifoldIndex].potentialContactPointsIndices.size() > 0);
                   const uint manifoldContactPointIndex = mPotentialContactManifolds[contactManifoldIndex].potentialContactPointsIndices[0];
                   const ContactPointInfo& manifoldContactPoint = mPotentialContactPoints[manifoldContactPointIndex];

                    // If we have found a corresponding manifold for the new contact point
                    // (a manifold with a similar contact normal direction)
                    if (manifoldContactPoint.normal.dot(contactPoint.normal) >= mWorld->mConfig.cosAngleSimilarContactManifold) {

                        // Add the contact point to the manifold
                        mPotentialContactManifolds[contactManifoldIndex].potentialContactPointsIndices.add(contactPointIndex);

                        similarManifoldFound = true;

                        break;
                    }
                }
            }

            // If we have not found a manifold with a similar contact normal for the contact point
            if (!similarManifoldFound) {

                // Create a new contact manifold for the overlapping pair
                // TODO : We should probably use single frame allocator here
                //        If so, do not forget to call mPotentialContactPoints.clear(true) at the end of frame
                ContactManifoldInfo contactManifoldInfo(pairId, mMemoryManager.getPoolAllocator());

                // Add the contact point to the manifold
                contactManifoldInfo.potentialContactPointsIndices.add(contactPointIndex);

                // If the overlapping pair contact does not exists yet
                if (pairContact == nullptr) {

                    Entity body1Entity = narrowPhaseInfoBatch.overlappingPairs[i]->getShape1()->getBody()->getEntity();
                    Entity body2Entity = narrowPhaseInfoBatch.overlappingPairs[i]->getShape2()->getBody()->getEntity();

                    assert(!mWorld->mBodyComponents.getIsEntityDisabled(body1Entity) || !mWorld->mBodyComponents.getIsEntityDisabled(body2Entity));

                    // TODO : We should probably use a single frame allocator here
                    const uint newContactPairIndex = mCurrentContactPairs->size();
                    ContactPair overlappingPairContact(pairId, body1Entity, body2Entity,
                                                       narrowPhaseInfoBatch.overlappingPairs[i]->getShape1()->getEntity(),
                                                       narrowPhaseInfoBatch.overlappingPairs[i]->getShape2()->getEntity(),
                                                       newContactPairIndex, mMemoryManager.getPoolAllocator());
                    mCurrentContactPairs->add(overlappingPairContact);
                    pairContact = &((*mCurrentContactPairs)[newContactPairIndex]);
                    mCurrentMapPairIdToContactPairIndex->add(Pair<OverlappingPair::OverlappingPairId, uint>(pairId, newContactPairIndex));

                    auto itbodyContactPairs = mMapBodyToContactPairs.find(body1Entity);
                    if (itbodyContactPairs != mMapBodyToContactPairs.end()) {
                        itbodyContactPairs->second.add(newContactPairIndex);
                    }
                    else {
                        List<uint> contactPairs(mMemoryManager.getSingleFrameAllocator(), 1);
                        contactPairs.add(newContactPairIndex);
                        mMapBodyToContactPairs.add(Pair<Entity, List<uint>>(body1Entity, contactPairs));
                    }
                    itbodyContactPairs = mMapBodyToContactPairs.find(body2Entity);
                    if (itbodyContactPairs != mMapBodyToContactPairs.end()) {
                        itbodyContactPairs->second.add(newContactPairIndex);
                    }
                    else {
                        List<uint> contactPairs(mMemoryManager.getSingleFrameAllocator(), 1);
                        contactPairs.add(newContactPairIndex);
                        mMapBodyToContactPairs.add(Pair<Entity, List<uint>>(body2Entity, contactPairs));
                    }
                }

                assert(pairContact != nullptr);

                // Add the potential contact manifold
                uint contactManifoldIndex = static_cast<uint>(mPotentialContactManifolds.size());
                mPotentialContactManifolds.add(contactManifoldInfo);

                // Add the contact manifold to the overlapping pair contact
                assert(pairContact->pairId == contactManifoldInfo.pairId);
                pairContact->potentialContactManifoldsIndices.add(contactManifoldIndex);
            }

            assert(pairContact->potentialContactManifoldsIndices.size() > 0);
        }

        narrowPhaseInfoBatch.resetContactPoints(i);
    }
}

// Clear the obsolete manifolds and contact points and reduce the number of contacts points of the remaining manifolds
void CollisionDetection::reducePotentialContactManifolds() {

    RP3D_PROFILE("CollisionDetection::reducePotentialContactManifolds()", mProfiler);

    // Reduce the number of potential contact manifolds in a contact pair
    for (uint i=0; i < mCurrentContactPairs->size(); i++) {

        ContactPair& contactPair = (*mCurrentContactPairs)[i];

        assert(contactPair.potentialContactManifoldsIndices.size() > 0);

        // While there are too many manifolds
        while(contactPair.potentialContactManifoldsIndices.size() > mWorld->mConfig.nbMaxContactManifolds) {

            // Look for a manifold with the smallest contact penetration depth.
            decimal minDepth = DECIMAL_LARGEST;
            int minDepthManifoldIndex = -1;
            for (uint j=0; j < contactPair.potentialContactManifoldsIndices.size(); j++) {

                ContactManifoldInfo& manifold = mPotentialContactManifolds[contactPair.potentialContactManifoldsIndices[j]];

                // Get the largest contact point penetration depth of the manifold
                const decimal depth = computePotentialManifoldLargestContactDepth(manifold);

                if (depth < minDepth) {
                    minDepth = depth;
                    minDepthManifoldIndex = static_cast<int>(j);
                }
            }

            // Remove the non optimal manifold
            assert(minDepthManifoldIndex >= 0);
            contactPair.potentialContactManifoldsIndices.removeAt(minDepthManifoldIndex);
        }
    }

    // Reduce the number of potential contact points in the manifolds
    for (uint i=0; i < mCurrentContactPairs->size(); i++) {

        const ContactPair& pairContact = (*mCurrentContactPairs)[i];

        // For each potential contact manifold
        for (uint j=0; j < pairContact.potentialContactManifoldsIndices.size(); j++) {

            ContactManifoldInfo& manifold = mPotentialContactManifolds[pairContact.potentialContactManifoldsIndices[j]];

            // If there are two many contact points in the manifold
            if (manifold.potentialContactPointsIndices.size() > MAX_CONTACT_POINTS_IN_MANIFOLD) {

                Transform shape1LocalToWorldTransoform = mOverlappingPairs[manifold.pairId]->getShape1()->getLocalToWorldTransform();

                // Reduce the number of contact points in the manifold
                reduceContactPoints(manifold, shape1LocalToWorldTransoform);
            }

            assert(manifold.potentialContactPointsIndices.size() <= MAX_CONTACT_POINTS_IN_MANIFOLD);
        }
    }
}

// Return the largest depth of all the contact points of a potential manifold
decimal CollisionDetection::computePotentialManifoldLargestContactDepth(const ContactManifoldInfo& manifold) const {

    decimal largestDepth = 0.0f;

    assert(manifold.potentialContactPointsIndices.size() > 0);

    for (uint i=0; i < manifold.potentialContactPointsIndices.size(); i++) {
        decimal depth = mPotentialContactPoints[manifold.potentialContactPointsIndices[i]].penetrationDepth;

        if (depth > largestDepth) {
            largestDepth = depth;
        }
    }

    return largestDepth;
}

// Reduce the number of contact points of a potential contact manifold
// This is based on the technique described by Dirk Gregorius in his
// "Contacts Creation" GDC presentation. This method will reduce the number of
// contact points to a maximum of 4 points (but it can be less).
void CollisionDetection::reduceContactPoints(ContactManifoldInfo& manifold, const Transform& shape1ToWorldTransform) {

    assert(manifold.potentialContactPointsIndices.size() > MAX_CONTACT_POINTS_IN_MANIFOLD);

    // The following algorithm only works to reduce to a maximum of 4 contact points
    assert(MAX_CONTACT_POINTS_IN_MANIFOLD == 4);

    // List of the candidate contact points indices in the manifold. Every time that we have found a
    // point we want to keep, we will remove it from this list
    List<uint> candidatePointsIndices(manifold.potentialContactPointsIndices);

    // TODO : DELETE THIS
    uint nbPoints = candidatePointsIndices.size();

    int8 nbReducedPoints = 0;

    uint pointsToKeepIndices[MAX_CONTACT_POINTS_IN_MANIFOLD];
    for (int8 i=0; i<MAX_CONTACT_POINTS_IN_MANIFOLD; i++) {
        pointsToKeepIndices[i] = 0;
    }

    //  Compute the initial contact point we need to keep.
    // The first point we keep is always the point in a given
    // constant direction (in order to always have same contact points
    // between frames for better stability)

    const Transform worldToShape1Transform = shape1ToWorldTransform.getInverse();

    // Compute the contact normal of the manifold (we use the first contact point)
    // in the local-space of the first collision shape
    const Vector3 contactNormalShape1Space = worldToShape1Transform.getOrientation() * mPotentialContactPoints[candidatePointsIndices[0]].normal;

    // Compute a search direction
    const Vector3 searchDirection(1, 1, 1);
    decimal maxDotProduct = DECIMAL_SMALLEST;
    uint elementIndexToKeep = 0;
    for (uint i=0; i < candidatePointsIndices.size(); i++) {

        const ContactPointInfo& element = mPotentialContactPoints[candidatePointsIndices[i]];
        decimal dotProduct = searchDirection.dot(element.localPoint1);
        if (dotProduct > maxDotProduct) {
            maxDotProduct = dotProduct;
            elementIndexToKeep = i;
            nbReducedPoints = 1;
        }
    }
    pointsToKeepIndices[0] = candidatePointsIndices[elementIndexToKeep];
    candidatePointsIndices.removeAt(elementIndexToKeep);
    assert(nbReducedPoints == 1);

    // Compute the second contact point we need to keep.
    // The second point we keep is the one farthest away from the first point.

    decimal maxDistance = decimal(0.0);
    elementIndexToKeep = 0;
    for (uint i=0; i < candidatePointsIndices.size(); i++) {

        const ContactPointInfo& element = mPotentialContactPoints[candidatePointsIndices[i]];
        const ContactPointInfo& pointToKeep0 = mPotentialContactPoints[pointsToKeepIndices[0]];

        assert(candidatePointsIndices[i] != pointsToKeepIndices[0]);

        const decimal distance = (pointToKeep0.localPoint1 - element.localPoint1).lengthSquare();
        if (distance >= maxDistance) {
            maxDistance = distance;
            elementIndexToKeep = i;
            nbReducedPoints = 2;
        }

    }
    pointsToKeepIndices[1] = candidatePointsIndices[elementIndexToKeep];
    candidatePointsIndices.removeAt(elementIndexToKeep);
    assert(nbReducedPoints == 2);

    // Compute the third contact point we need to keep.
    // The third point is the one producing the triangle with the larger area
    // with first and second point.

    // We compute the most positive or most negative triangle area (depending on winding)
    uint thirdPointMaxAreaIndex = 0;
    uint thirdPointMinAreaIndex = 0;
    decimal minArea = decimal(0.0);
    decimal maxArea = decimal(0.0);
    bool isPreviousAreaPositive = true;
    for (uint i=0; i < candidatePointsIndices.size(); i++) {

        const ContactPointInfo& element = mPotentialContactPoints[candidatePointsIndices[i]];
        const ContactPointInfo& pointToKeep0 = mPotentialContactPoints[pointsToKeepIndices[0]];
        const ContactPointInfo& pointToKeep1 = mPotentialContactPoints[pointsToKeepIndices[1]];

        assert(candidatePointsIndices[i] != pointsToKeepIndices[0]);
        assert(candidatePointsIndices[i] != pointsToKeepIndices[1]);

        const Vector3 newToFirst = pointToKeep0.localPoint1 - element.localPoint1;
        const Vector3 newToSecond = pointToKeep1.localPoint1 - element.localPoint1;

        // Compute the triangle area
        decimal area = newToFirst.cross(newToSecond).dot(contactNormalShape1Space);

        if (area >= maxArea) {
            maxArea = area;
            thirdPointMaxAreaIndex = i;
        }
        if (area <= minArea) {
            minArea = area;
            thirdPointMinAreaIndex = i;
        }
    }
    assert(minArea <= decimal(0.0));
    assert(maxArea >= decimal(0.0));
    if (maxArea > (-minArea)) {
        isPreviousAreaPositive = true;
        pointsToKeepIndices[2] = candidatePointsIndices[thirdPointMaxAreaIndex];
        candidatePointsIndices.removeAt(thirdPointMaxAreaIndex);
    }
    else {
        isPreviousAreaPositive = false;
        pointsToKeepIndices[2] = candidatePointsIndices[thirdPointMinAreaIndex];
        candidatePointsIndices.removeAt(thirdPointMinAreaIndex);
    }
    nbReducedPoints = 3;

    // Compute the 4th point by choosing the triangle that add the most
    // triangle area to the previous triangle and has opposite sign area (opposite winding)

    decimal largestArea = decimal(0.0); // Largest area (positive or negative)
    elementIndexToKeep = 0;
    nbReducedPoints = 4;
    decimal area;

    // For each remaining candidate points
    for (uint i=0; i < candidatePointsIndices.size(); i++) {

        const ContactPointInfo& element = mPotentialContactPoints[candidatePointsIndices[i]];

        assert(candidatePointsIndices[i] != pointsToKeepIndices[0]);
        assert(candidatePointsIndices[i] != pointsToKeepIndices[1]);
        assert(candidatePointsIndices[i] != pointsToKeepIndices[2]);

        // For each edge of the triangle made by the first three points
        for (uint j=0; j<3; j++) {

            uint edgeVertex1Index = j;
            uint edgeVertex2Index = j < 2 ? j + 1 : 0;

            const ContactPointInfo& pointToKeepEdgeV1 = mPotentialContactPoints[pointsToKeepIndices[edgeVertex1Index]];
            const ContactPointInfo& pointToKeepEdgeV2 = mPotentialContactPoints[pointsToKeepIndices[edgeVertex2Index]];

            const Vector3 newToFirst = pointToKeepEdgeV1.localPoint1 - element.localPoint1;
            const Vector3 newToSecond = pointToKeepEdgeV2.localPoint1 - element.localPoint1;

            // Compute the triangle area
            area = newToFirst.cross(newToSecond).dot(contactNormalShape1Space);

            // We are looking at the triangle with maximal area (positive or negative).
            // If the previous area is positive, we are looking at negative area now.
            // If the previous area is negative, we are looking at the positive area now.
            if (isPreviousAreaPositive && area <= largestArea) {
                largestArea = area;
                elementIndexToKeep = i;
            }
            else if (!isPreviousAreaPositive && area >= largestArea) {
                largestArea = area;
                elementIndexToKeep = i;
            }
        }
    }
    pointsToKeepIndices[3] = candidatePointsIndices[elementIndexToKeep];
    candidatePointsIndices.removeAt(elementIndexToKeep);

    // Only keep the four selected contact points in the manifold
    manifold.potentialContactPointsIndices.clear();
    manifold.potentialContactPointsIndices.add(pointsToKeepIndices[0]);
    manifold.potentialContactPointsIndices.add(pointsToKeepIndices[1]);
    manifold.potentialContactPointsIndices.add(pointsToKeepIndices[2]);
    manifold.potentialContactPointsIndices.add(pointsToKeepIndices[3]);
}

// Report contacts for all the colliding overlapping pairs
void CollisionDetection::reportAllContacts() {

    RP3D_PROFILE("CollisionDetection::reportAllContacts()", mProfiler);

    // TODO : Rework how we report contacts
    /*
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
    */
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
    List<int> overlappingNodes(mMemoryManager.getPoolAllocator());
    mBroadPhaseSystem.reportAllShapesOverlappingWithAABB(aabb, overlappingNodes);

    // For each overlaping proxy shape
    for (uint i=0; i < overlappingNodes.size(); i++) {

        // Get the overlapping proxy shape
        const int broadPhaseId = overlappingNodes[i];
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
            List<int> overlappingNodes(mMemoryManager.getPoolAllocator());
            mBroadPhaseSystem.reportAllShapesOverlappingWithAABB(shapeAABB, overlappingNodes);

            const bodyindex bodyId = body->getId();

            // For each overlaping proxy shape
            for (uint i=0; i < overlappingNodes.size(); i++) {

                // Get the overlapping proxy shape
                const int broadPhaseId = overlappingNodes[i];
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
                const Pair<uint, uint> pairID = OverlappingPair::computeID(body1ProxyShape->getBroadPhaseId(), body2ProxyShape->getBroadPhaseId());

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
    //reducePotentialContactManifolds(overlappingPairs);

    // TODO : Rework how we report contacts
    /*
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
    */
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
            List<int> overlappingNodes(mMemoryManager.getPoolAllocator());
            mBroadPhaseSystem.reportAllShapesOverlappingWithAABB(shapeAABB, overlappingNodes);

            const bodyindex bodyId = body->getId();

            // For each overlaping proxy shape
            for (uint i=0; i < overlappingNodes.size(); i++) {

                // Get the overlapping proxy shape
                const int broadPhaseId = overlappingNodes[i];
                ProxyShape* proxyShape = mBroadPhaseSystem.getProxyShapeForBroadPhaseId(broadPhaseId);

                // If the two proxy collision shapes are not from the same body
                if (proxyShape->getBody()->getId() != bodyId) {

                    // Check if the collision filtering allows collision between the two shapes
                    if ((proxyShape->getCollisionCategoryBits() & categoryMaskBits) != 0) {

                        OverlappingPair* pair;
                        const Pair<uint, uint> pairID = OverlappingPair::computeID(bodyProxyShape->getBroadPhaseId(), proxyShape->getBroadPhaseId());

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
            }
        }
    }

    // Test narrow-phase collision
    testNarrowPhaseCollision(narrowPhaseInput, false, true, mMemoryManager.getPoolAllocator());

    // Process the potential contacts
    processAllPotentialContacts(narrowPhaseInput, false);

    // Reduce the number of contact points in the manifolds
    //reducePotentialContactManifolds(overlappingPairs);

    // TODO : Rework how we report contacts
    /*
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
    */
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
        const Pair<uint, uint> pairID = OverlappingPair::computeID(originalPair->getShape1()->getBroadPhaseId(), originalPair->getShape2()->getBroadPhaseId());

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
    //reducePotentialContactManifolds(overlappingPairs);

    // TODO : Rework how we report contacts
    /*
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
    */
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
