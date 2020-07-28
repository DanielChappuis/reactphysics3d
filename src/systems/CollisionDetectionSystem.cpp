﻿/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2020 Daniel Chappuis                                       *
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
#include <reactphysics3d/systems/CollisionDetectionSystem.h>
#include <reactphysics3d/engine/PhysicsWorld.h>
#include <reactphysics3d/collision/OverlapCallback.h>
#include <reactphysics3d/collision/shapes/BoxShape.h>
#include <reactphysics3d/collision/shapes/ConcaveShape.h>
#include <reactphysics3d/collision/ContactManifoldInfo.h>
#include <reactphysics3d/constraint/ContactPoint.h>
#include <reactphysics3d/body/RigidBody.h>
#include <reactphysics3d/configuration.h>
#include <reactphysics3d/collision/CollisionCallback.h>
#include <reactphysics3d/collision/OverlapCallback.h>
#include <reactphysics3d/collision/narrowphase/NarrowPhaseInfoBatch.h>
#include <reactphysics3d/collision/ContactManifold.h>
#include <reactphysics3d/utils/Profiler.h>
#include <reactphysics3d/engine/EventListener.h>
#include <reactphysics3d/collision/RaycastInfo.h>
#include <reactphysics3d/containers/Pair.h>
#include <cassert>
#include <iostream>

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;
using namespace std;

// Constructor
CollisionDetectionSystem::CollisionDetectionSystem(PhysicsWorld* world, ColliderComponents& collidersComponents,  TransformComponents& transformComponents,
                                                   CollisionBodyComponents& collisionBodyComponents, RigidBodyComponents& rigidBodyComponents,
                                                   MemoryManager& memoryManager)
                   : mMemoryManager(memoryManager), mCollidersComponents(collidersComponents), mRigidBodyComponents(rigidBodyComponents),
                     mCollisionDispatch(mMemoryManager.getPoolAllocator()), mWorld(world),
                     mNoCollisionPairs(mMemoryManager.getPoolAllocator()),
                     mOverlappingPairs(mMemoryManager.getPoolAllocator(), mMemoryManager.getSingleFrameAllocator(), mCollidersComponents,
                                       collisionBodyComponents, rigidBodyComponents, mNoCollisionPairs, mCollisionDispatch),
                     mBroadPhaseSystem(*this, mCollidersComponents, transformComponents, rigidBodyComponents),
                     mMapBroadPhaseIdToColliderEntity(memoryManager.getPoolAllocator()),
                     mNarrowPhaseInput(mMemoryManager.getSingleFrameAllocator(), mOverlappingPairs), mPotentialContactPoints(mMemoryManager.getSingleFrameAllocator()),
                     mPotentialContactManifolds(mMemoryManager.getSingleFrameAllocator()), mContactPairs1(mMemoryManager.getPoolAllocator()),
                     mContactPairs2(mMemoryManager.getPoolAllocator()), mPreviousContactPairs(&mContactPairs1), mCurrentContactPairs(&mContactPairs2),
                     mLostContactPairs(mMemoryManager.getSingleFrameAllocator()), mPreviousMapPairIdToContactPairIndex(mMemoryManager.getHeapAllocator()),
                     mContactManifolds1(mMemoryManager.getPoolAllocator()), mContactManifolds2(mMemoryManager.getPoolAllocator()),
                     mPreviousContactManifolds(&mContactManifolds1), mCurrentContactManifolds(&mContactManifolds2),
                     mContactPoints1(mMemoryManager.getPoolAllocator()), mContactPoints2(mMemoryManager.getPoolAllocator()),
                     mPreviousContactPoints(&mContactPoints1), mCurrentContactPoints(&mContactPoints2) {

#ifdef IS_RP3D_PROFILING_ENABLED


	mProfiler = nullptr;
    mCollisionDispatch.setProfiler(mProfiler);
    mOverlappingPairs.setProfiler(mProfiler);

#endif

}

// Compute the collision detection
void CollisionDetectionSystem::computeCollisionDetection() {

    RP3D_PROFILE("CollisionDetectionSystem::computeCollisionDetection()", mProfiler);
	    
    // Compute the broad-phase collision detection
    computeBroadPhase();

    // Compute the middle-phase collision detection
    computeMiddlePhase(mNarrowPhaseInput, true);
    
    // Compute the narrow-phase collision detection
    computeNarrowPhase();
}

// Compute the broad-phase collision detection
void CollisionDetectionSystem::computeBroadPhase() {

    RP3D_PROFILE("CollisionDetectionSystem::computeBroadPhase()", mProfiler);

    // Ask the broad-phase to compute all the shapes overlapping with the shapes that
    // have moved or have been added in the last frame. This call can only add new
    // overlapping pairs in the collision detection.
    List<Pair<int32, int32>> overlappingNodes(mMemoryManager.getPoolAllocator(), 32);
    mBroadPhaseSystem.computeOverlappingPairs(mMemoryManager, overlappingNodes);

    // Create new overlapping pairs if necessary
    updateOverlappingPairs(overlappingNodes);

    // Remove non overlapping pairs
    removeNonOverlappingPairs();
}

// Remove pairs that are not overlapping anymore
void CollisionDetectionSystem::removeNonOverlappingPairs() {

    RP3D_PROFILE("CollisionDetectionSystem::removeNonOverlappingPairs()", mProfiler);

    for (uint64 i=0; i < mOverlappingPairs.getNbPairs(); i++) {

        // Check if we need to test overlap. If so, test if the two shapes are still overlapping.
        // Otherwise, we destroy the overlapping pair
        if (mOverlappingPairs.mNeedToTestOverlap[i]) {

            if(mBroadPhaseSystem.testOverlappingShapes(mOverlappingPairs.mPairBroadPhaseId1[i], mOverlappingPairs.mPairBroadPhaseId2[i])) {
                mOverlappingPairs.mNeedToTestOverlap[i] = false;
            }
            else {

                // If the two colliders of the pair were colliding in the previous frame
                if (mOverlappingPairs.mCollidingInPreviousFrame[i]) {

                    // Create a new lost contact pair
                    addLostContactPair(i);
                }

                mOverlappingPairs.removePair(mOverlappingPairs.mPairIds[i]);
                i--;
            }
        }
    }
}

// Add a lost contact pair (pair of colliders that are not in contact anymore)
void CollisionDetectionSystem::addLostContactPair(uint64 overlappingPairIndex) {

    const Entity collider1Entity = mOverlappingPairs.mColliders1[overlappingPairIndex];
    const Entity collider2Entity = mOverlappingPairs.mColliders2[overlappingPairIndex];

    const Entity body1Entity = mCollidersComponents.getBody(collider1Entity);
    const Entity body2Entity = mCollidersComponents.getBody(collider2Entity);

    const bool isCollider1Trigger = mCollidersComponents.getIsTrigger(collider1Entity);
    const bool isCollider2Trigger = mCollidersComponents.getIsTrigger(collider2Entity);
    const bool isTrigger = isCollider1Trigger || isCollider2Trigger;

    // Create a lost contact pair
    ContactPair lostContactPair(mOverlappingPairs.mPairIds[overlappingPairIndex], body1Entity, body2Entity, collider1Entity, collider2Entity, mLostContactPairs.size(),
                                true, isTrigger);
    mLostContactPairs.add(lostContactPair);
}

// Take a list of overlapping nodes in the broad-phase and create new overlapping pairs if necessary
void CollisionDetectionSystem::updateOverlappingPairs(const List<Pair<int32, int32>>& overlappingNodes) {

    RP3D_PROFILE("CollisionDetectionSystem::updateOverlappingPairs()", mProfiler);

    // For each overlapping pair of nodes
    const uint nbOverlappingNodes = overlappingNodes.size();
    for (uint i=0; i < nbOverlappingNodes; i++) {

        Pair<int32, int32> nodePair = overlappingNodes[i];

        assert(nodePair.first != -1);
        assert(nodePair.second != -1);

        // Skip pairs with same overlapping nodes
        if (nodePair.first != nodePair.second) {

            // Get the two colliders
            const Entity collider1Entity = mMapBroadPhaseIdToColliderEntity[nodePair.first];
            const Entity collider2Entity = mMapBroadPhaseIdToColliderEntity[nodePair.second];

            const uint collider1Index = mCollidersComponents.getEntityIndex(collider1Entity);
            const uint collider2Index = mCollidersComponents.getEntityIndex(collider2Entity);

            // Get the two bodies
            const Entity body1Entity = mCollidersComponents.mBodiesEntities[collider1Index];
            const Entity body2Entity = mCollidersComponents.mBodiesEntities[collider2Index];

            // If the two colliders are from the same body, skip it
            if (body1Entity != body2Entity) {

                // Compute the overlapping pair ID
                const uint64 pairId = pairNumbers(std::max(nodePair.first, nodePair.second), std::min(nodePair.first, nodePair.second));

                // Check if the overlapping pair already exists
                auto it = mOverlappingPairs.mMapPairIdToPairIndex.find(pairId);
                if (it == mOverlappingPairs.mMapPairIdToPairIndex.end()) {

                    const unsigned short shape1CollideWithMaskBits = mCollidersComponents.mCollideWithMaskBits[collider1Index];
                    const unsigned short shape2CollideWithMaskBits = mCollidersComponents.mCollideWithMaskBits[collider2Index];

                    const unsigned short shape1CollisionCategoryBits = mCollidersComponents.mCollisionCategoryBits[collider1Index];
                    const unsigned short shape2CollisionCategoryBits = mCollidersComponents.mCollisionCategoryBits[collider2Index];

                    // Check if the collision filtering allows collision between the two shapes
                    if ((shape1CollideWithMaskBits & shape2CollisionCategoryBits) != 0 &&
                        (shape1CollisionCategoryBits & shape2CollideWithMaskBits) != 0) {

                        Collider* shape1 = mCollidersComponents.mColliders[collider1Index];
                        Collider* shape2 = mCollidersComponents.mColliders[collider2Index];

                        // Check that at least one collision shape is convex
                        if (shape1->getCollisionShape()->isConvex() || shape2->getCollisionShape()->isConvex()) {

                            // Add the new overlapping pair
                            mOverlappingPairs.addPair(shape1, shape2);
                        }
                    }
                }
                else {

                    // We do not need to test the pair for overlap because it has just been reported that they still overlap
                    mOverlappingPairs.mNeedToTestOverlap[it->second] = false;
                }
            }
        }
    }
}

// Compute the middle-phase collision detection
void CollisionDetectionSystem::computeMiddlePhase(NarrowPhaseInput& narrowPhaseInput, bool needToReportContacts) {

    RP3D_PROFILE("CollisionDetectionSystem::computeMiddlePhase()", mProfiler);

    // Reserve memory for the narrow-phase input using cached capacity from previous frame
    narrowPhaseInput.reserveMemory();

    // Remove the obsolete last frame collision infos and mark all the others as obsolete
    mOverlappingPairs.clearObsoleteLastFrameCollisionInfos();

    // For each possible convex vs convex pair of bodies
    const uint64 nbConvexVsConvexPairs = mOverlappingPairs.getNbConvexVsConvexPairs();
    for (uint64 i=0; i < nbConvexVsConvexPairs; i++) {

        assert(mCollidersComponents.getBroadPhaseId(mOverlappingPairs.mColliders1[i]) != -1);
        assert(mCollidersComponents.getBroadPhaseId(mOverlappingPairs.mColliders2[i]) != -1);
        assert(mCollidersComponents.getBroadPhaseId(mOverlappingPairs.mColliders1[i]) != mCollidersComponents.getBroadPhaseId(mOverlappingPairs.mColliders2[i]));

        // Check that at least one body is enabled (active and awake) and not static
        if (mOverlappingPairs.mIsActive[i]) {

            const Entity collider1Entity = mOverlappingPairs.mColliders1[i];
            const Entity collider2Entity = mOverlappingPairs.mColliders2[i];

            const uint collider1Index = mCollidersComponents.getEntityIndex(collider1Entity);
            const uint collider2Index = mCollidersComponents.getEntityIndex(collider2Entity);

            CollisionShape* collisionShape1 = mCollidersComponents.mCollisionShapes[collider1Index];
            CollisionShape* collisionShape2 = mCollidersComponents.mCollisionShapes[collider2Index];

            NarrowPhaseAlgorithmType algorithmType = mOverlappingPairs.mNarrowPhaseAlgorithmType[i];

            const bool isCollider1Trigger = mCollidersComponents.mIsTrigger[collider1Index];
            const bool isCollider2Trigger = mCollidersComponents.mIsTrigger[collider2Index];
            const bool reportContacts = needToReportContacts && !isCollider1Trigger && !isCollider2Trigger;

            // No middle-phase is necessary, simply create a narrow phase info
            // for the narrow-phase collision detection
            narrowPhaseInput.addNarrowPhaseTest(mOverlappingPairs.mPairIds[i], i, collider1Entity, collider2Entity, collisionShape1, collisionShape2,
                                                      mCollidersComponents.mLocalToWorldTransforms[collider1Index],
                                                      mCollidersComponents.mLocalToWorldTransforms[collider2Index],
                                                      algorithmType, reportContacts, mMemoryManager.getSingleFrameAllocator());

            mOverlappingPairs.mCollidingInCurrentFrame[i] = false;
        }
    }

    // For each possible convex vs concave pair of bodies
    const uint64 convexVsConcaveStartIndex = mOverlappingPairs.getConvexVsConcavePairsStartIndex();
    const uint64 nbConvexVsConcavePairs = mOverlappingPairs.getNbConvexVsConcavePairs();
    for (uint64 i=convexVsConcaveStartIndex; i < convexVsConcaveStartIndex + nbConvexVsConcavePairs; i++) {

        assert(mCollidersComponents.getBroadPhaseId(mOverlappingPairs.mColliders1[i]) != -1);
        assert(mCollidersComponents.getBroadPhaseId(mOverlappingPairs.mColliders2[i]) != -1);
        assert(mCollidersComponents.getBroadPhaseId(mOverlappingPairs.mColliders1[i]) != mCollidersComponents.getBroadPhaseId(mOverlappingPairs.mColliders2[i]));

        // Check that at least one body is enabled (active and awake) and not static
        if (mOverlappingPairs.mIsActive[i]) {

            computeConvexVsConcaveMiddlePhase(i, mMemoryManager.getSingleFrameAllocator(), narrowPhaseInput);

            mOverlappingPairs.mCollidingInCurrentFrame[i] = false;
        }
    }
}

// Compute the middle-phase collision detection
void CollisionDetectionSystem::computeMiddlePhaseCollisionSnapshot(List<uint64>& convexPairs, List<uint64>& concavePairs, NarrowPhaseInput& narrowPhaseInput,
                                                                   bool reportContacts) {

    RP3D_PROFILE("CollisionDetectionSystem::computeMiddlePhase()", mProfiler);

    // Reserve memory for the narrow-phase input using cached capacity from previous frame
    narrowPhaseInput.reserveMemory();

    // Remove the obsolete last frame collision infos and mark all the others as obsolete
    mOverlappingPairs.clearObsoleteLastFrameCollisionInfos();

    // For each possible convex vs convex pair of bodies
    const uint64 nbConvexPairs = convexPairs.size();
    for (uint64 p=0; p < nbConvexPairs; p++) {

        const uint64 pairId = convexPairs[p];

        const uint64 pairIndex = mOverlappingPairs.mMapPairIdToPairIndex[pairId];
        assert(pairIndex < mOverlappingPairs.getNbPairs());

        const Entity collider1Entity = mOverlappingPairs.mColliders1[pairIndex];
        const Entity collider2Entity = mOverlappingPairs.mColliders2[pairIndex];

        const uint collider1Index = mCollidersComponents.getEntityIndex(collider1Entity);
        const uint collider2Index = mCollidersComponents.getEntityIndex(collider2Entity);

        assert(mCollidersComponents.getBroadPhaseId(collider1Entity) != -1);
        assert(mCollidersComponents.getBroadPhaseId(collider2Entity) != -1);
        assert(mCollidersComponents.getBroadPhaseId(collider1Entity) != mCollidersComponents.getBroadPhaseId(collider2Entity));

        CollisionShape* collisionShape1 = mCollidersComponents.mCollisionShapes[collider1Index];
        CollisionShape* collisionShape2 = mCollidersComponents.mCollisionShapes[collider2Index];

        NarrowPhaseAlgorithmType algorithmType = mOverlappingPairs.mNarrowPhaseAlgorithmType[pairIndex];

        // No middle-phase is necessary, simply create a narrow phase info
        // for the narrow-phase collision detection
        narrowPhaseInput.addNarrowPhaseTest(pairId, pairIndex, collider1Entity, collider2Entity, collisionShape1, collisionShape2,
                                                  mCollidersComponents.mLocalToWorldTransforms[collider1Index],
                                                  mCollidersComponents.mLocalToWorldTransforms[collider2Index],
                                                  algorithmType, reportContacts, mMemoryManager.getSingleFrameAllocator());

    }

    // For each possible convex vs concave pair of bodies
    const uint nbConcavePairs = concavePairs.size();
    for (uint p=0; p < nbConcavePairs; p++) {

        const uint64 pairId = concavePairs[p];
        const uint64 pairIndex = mOverlappingPairs.mMapPairIdToPairIndex[pairId];

        assert(mCollidersComponents.getBroadPhaseId(mOverlappingPairs.mColliders1[pairIndex]) != -1);
        assert(mCollidersComponents.getBroadPhaseId(mOverlappingPairs.mColliders2[pairIndex]) != -1);
        assert(mCollidersComponents.getBroadPhaseId(mOverlappingPairs.mColliders1[pairIndex]) != mCollidersComponents.getBroadPhaseId(mOverlappingPairs.mColliders2[pairIndex]));

        computeConvexVsConcaveMiddlePhase(pairIndex, mMemoryManager.getSingleFrameAllocator(), narrowPhaseInput);
    }
}

// Compute the concave vs convex middle-phase algorithm for a given pair of bodies
void CollisionDetectionSystem::computeConvexVsConcaveMiddlePhase(uint64 pairIndex, MemoryAllocator& allocator, NarrowPhaseInput& narrowPhaseInput) {

    RP3D_PROFILE("CollisionDetectionSystem::computeConvexVsConcaveMiddlePhase()", mProfiler);

    const Entity collider1 = mOverlappingPairs.mColliders1[pairIndex];
    const Entity collider2 = mOverlappingPairs.mColliders2[pairIndex];

    const uint collider1Index = mCollidersComponents.getEntityIndex(collider1);
    const uint collider2Index = mCollidersComponents.getEntityIndex(collider2);

    Transform& shape1LocalToWorldTransform = mCollidersComponents.mLocalToWorldTransforms[collider1Index];
    Transform& shape2LocalToWorldTransform = mCollidersComponents.mLocalToWorldTransforms[collider2Index];

    Transform convexToConcaveTransform;

    // Collision shape 1 is convex, collision shape 2 is concave
    ConvexShape* convexShape;
    ConcaveShape* concaveShape;
    const bool isShape1Convex = mOverlappingPairs.mIsShape1Convex[pairIndex];
    if (isShape1Convex) {
        convexShape = static_cast<ConvexShape*>(mCollidersComponents.mCollisionShapes[collider1Index]);
        concaveShape = static_cast<ConcaveShape*>(mCollidersComponents.mCollisionShapes[collider2Index]);
        convexToConcaveTransform = shape2LocalToWorldTransform.getInverse() * shape1LocalToWorldTransform;
    }
    else {  // Collision shape 2 is convex, collision shape 1 is concave
        convexShape = static_cast<ConvexShape*>(mCollidersComponents.mCollisionShapes[collider2Index]);
        concaveShape = static_cast<ConcaveShape*>(mCollidersComponents.mCollisionShapes[collider1Index]);
        convexToConcaveTransform = shape1LocalToWorldTransform.getInverse() * shape2LocalToWorldTransform;
    }

    assert(convexShape->isConvex());
    assert(!concaveShape->isConvex());
    assert(mOverlappingPairs.mNarrowPhaseAlgorithmType[pairIndex] != NarrowPhaseAlgorithmType::None);

    // Compute the convex shape AABB in the local-space of the convex shape
    AABB aabb;
    convexShape->computeAABB(aabb, convexToConcaveTransform);

    // Compute the concave shape triangles that are overlapping with the convex mesh AABB
    List<Vector3> triangleVertices(allocator);
    List<Vector3> triangleVerticesNormals(allocator);
    List<uint> shapeIds(allocator);
    concaveShape->computeOverlappingTriangles(aabb, triangleVertices, triangleVerticesNormals, shapeIds, allocator);

    assert(triangleVertices.size() == triangleVerticesNormals.size());
    assert(shapeIds.size() == triangleVertices.size() / 3);
    assert(triangleVertices.size() % 3 == 0);
    assert(triangleVerticesNormals.size() % 3 == 0);

    const bool isCollider1Trigger = mCollidersComponents.mIsTrigger[collider1Index];
    const bool isCollider2Trigger = mCollidersComponents.mIsTrigger[collider2Index];
    const bool reportContacts = !isCollider1Trigger && !isCollider2Trigger;

    // For each overlapping triangle
    const uint nbShapeIds = shapeIds.size();
    for (uint i=0; i < nbShapeIds; i++)
    {
        // Create a triangle collision shape (the allocated memory for the TriangleShape will be released in the
        // destructor of the corresponding NarrowPhaseInfo.
        TriangleShape* triangleShape = new (allocator.allocate(sizeof(TriangleShape)))
                                       TriangleShape(&(triangleVertices[i * 3]), &(triangleVerticesNormals[i * 3]), shapeIds[i], allocator);

    #ifdef IS_RP3D_PROFILING_ENABLED


        // Set the profiler to the triangle shape
        triangleShape->setProfiler(mProfiler);

    #endif

        // Create a narrow phase info for the narrow-phase collision detection
        narrowPhaseInput.addNarrowPhaseTest(mOverlappingPairs.mPairIds[pairIndex], pairIndex, collider1, collider2, isShape1Convex ? convexShape : triangleShape,
                                                isShape1Convex ? triangleShape : convexShape,
                                                shape1LocalToWorldTransform, shape2LocalToWorldTransform,
                                                mOverlappingPairs.mNarrowPhaseAlgorithmType[pairIndex], reportContacts, allocator);
    }
}

// Execute the narrow-phase collision detection algorithm on batches
bool CollisionDetectionSystem::testNarrowPhaseCollision(NarrowPhaseInput& narrowPhaseInput,
                                                        bool clipWithPreviousAxisIfStillColliding, MemoryAllocator& allocator) {

    bool contactFound = false;

    // Get the narrow-phase collision detection algorithms for each kind of collision shapes
    SphereVsSphereAlgorithm* sphereVsSphereAlgo = mCollisionDispatch.getSphereVsSphereAlgorithm();
    SphereVsCapsuleAlgorithm* sphereVsCapsuleAlgo = mCollisionDispatch.getSphereVsCapsuleAlgorithm();
    CapsuleVsCapsuleAlgorithm* capsuleVsCapsuleAlgo = mCollisionDispatch.getCapsuleVsCapsuleAlgorithm();
    SphereVsConvexPolyhedronAlgorithm* sphereVsConvexPolyAlgo = mCollisionDispatch.getSphereVsConvexPolyhedronAlgorithm();
    CapsuleVsConvexPolyhedronAlgorithm* capsuleVsConvexPolyAlgo = mCollisionDispatch.getCapsuleVsConvexPolyhedronAlgorithm();
    ConvexPolyhedronVsConvexPolyhedronAlgorithm* convexPolyVsConvexPolyAlgo = mCollisionDispatch.getConvexPolyhedronVsConvexPolyhedronAlgorithm();

    // get the narrow-phase batches to test for collision for contacts
    NarrowPhaseInfoBatch& sphereVsSphereBatchContacts = narrowPhaseInput.getSphereVsSphereBatch();
    NarrowPhaseInfoBatch& sphereVsCapsuleBatchContacts = narrowPhaseInput.getSphereVsCapsuleBatch();
    NarrowPhaseInfoBatch& capsuleVsCapsuleBatchContacts = narrowPhaseInput.getCapsuleVsCapsuleBatch();
    NarrowPhaseInfoBatch& sphereVsConvexPolyhedronBatchContacts = narrowPhaseInput.getSphereVsConvexPolyhedronBatch();
    NarrowPhaseInfoBatch& capsuleVsConvexPolyhedronBatchContacts = narrowPhaseInput.getCapsuleVsConvexPolyhedronBatch();
    NarrowPhaseInfoBatch& convexPolyhedronVsConvexPolyhedronBatchContacts = narrowPhaseInput.getConvexPolyhedronVsConvexPolyhedronBatch();

    // Compute the narrow-phase collision detection for each kind of collision shapes (for contacts)
    if (sphereVsSphereBatchContacts.getNbObjects() > 0) {
        contactFound |= sphereVsSphereAlgo->testCollision(sphereVsSphereBatchContacts, 0, sphereVsSphereBatchContacts.getNbObjects(), allocator);
    }
    if (sphereVsCapsuleBatchContacts.getNbObjects() > 0) {
        contactFound |= sphereVsCapsuleAlgo->testCollision(sphereVsCapsuleBatchContacts, 0, sphereVsCapsuleBatchContacts.getNbObjects(), allocator);
    }
    if (capsuleVsCapsuleBatchContacts.getNbObjects() > 0) {
        contactFound |= capsuleVsCapsuleAlgo->testCollision(capsuleVsCapsuleBatchContacts, 0, capsuleVsCapsuleBatchContacts.getNbObjects(), allocator);
    }
    if (sphereVsConvexPolyhedronBatchContacts.getNbObjects() > 0) {
        contactFound |= sphereVsConvexPolyAlgo->testCollision(sphereVsConvexPolyhedronBatchContacts, 0, sphereVsConvexPolyhedronBatchContacts.getNbObjects(), clipWithPreviousAxisIfStillColliding, allocator);
    }
    if (capsuleVsConvexPolyhedronBatchContacts.getNbObjects() > 0) {
        contactFound |= capsuleVsConvexPolyAlgo->testCollision(capsuleVsConvexPolyhedronBatchContacts, 0, capsuleVsConvexPolyhedronBatchContacts.getNbObjects(), clipWithPreviousAxisIfStillColliding, allocator);
    }
    if (convexPolyhedronVsConvexPolyhedronBatchContacts.getNbObjects() > 0) {
        contactFound |= convexPolyVsConvexPolyAlgo->testCollision(convexPolyhedronVsConvexPolyhedronBatchContacts, 0, convexPolyhedronVsConvexPolyhedronBatchContacts.getNbObjects(), clipWithPreviousAxisIfStillColliding, allocator);
    }

    return contactFound;
}

// Process the potential contacts after narrow-phase collision detection
void CollisionDetectionSystem::processAllPotentialContacts(NarrowPhaseInput& narrowPhaseInput, bool updateLastFrameInfo,
                                                     List<ContactPointInfo>& potentialContactPoints,
                                                     List<ContactManifoldInfo>& potentialContactManifolds,
                                                     List<ContactPair>* contactPairs) {

    assert(contactPairs->size() == 0);

    Map<uint64, uint> mapPairIdToContactPairIndex(mMemoryManager.getHeapAllocator(), mPreviousMapPairIdToContactPairIndex.size());

    // get the narrow-phase batches to test for collision
    NarrowPhaseInfoBatch& sphereVsSphereBatch = narrowPhaseInput.getSphereVsSphereBatch();
    NarrowPhaseInfoBatch& sphereVsCapsuleBatch = narrowPhaseInput.getSphereVsCapsuleBatch();
    NarrowPhaseInfoBatch& capsuleVsCapsuleBatch = narrowPhaseInput.getCapsuleVsCapsuleBatch();
    NarrowPhaseInfoBatch& sphereVsConvexPolyhedronBatch = narrowPhaseInput.getSphereVsConvexPolyhedronBatch();
    NarrowPhaseInfoBatch& capsuleVsConvexPolyhedronBatch = narrowPhaseInput.getCapsuleVsConvexPolyhedronBatch();
    NarrowPhaseInfoBatch& convexPolyhedronVsConvexPolyhedronBatch = narrowPhaseInput.getConvexPolyhedronVsConvexPolyhedronBatch();

    // Process the potential contacts
    processPotentialContacts(sphereVsSphereBatch, updateLastFrameInfo, potentialContactPoints, potentialContactManifolds, mapPairIdToContactPairIndex, contactPairs);
    processPotentialContacts(sphereVsCapsuleBatch, updateLastFrameInfo, potentialContactPoints, potentialContactManifolds, mapPairIdToContactPairIndex, contactPairs);
    processPotentialContacts(capsuleVsCapsuleBatch, updateLastFrameInfo, potentialContactPoints, potentialContactManifolds, mapPairIdToContactPairIndex, contactPairs);
    processPotentialContacts(sphereVsConvexPolyhedronBatch, updateLastFrameInfo, potentialContactPoints, potentialContactManifolds, mapPairIdToContactPairIndex, contactPairs);
    processPotentialContacts(capsuleVsConvexPolyhedronBatch, updateLastFrameInfo, potentialContactPoints, potentialContactManifolds, mapPairIdToContactPairIndex, contactPairs);
    processPotentialContacts(convexPolyhedronVsConvexPolyhedronBatch, updateLastFrameInfo, potentialContactPoints,
                             potentialContactManifolds, mapPairIdToContactPairIndex, contactPairs);
}

// Compute the narrow-phase collision detection
void CollisionDetectionSystem::computeNarrowPhase() {

    RP3D_PROFILE("CollisionDetectionSystem::computeNarrowPhase()", mProfiler);

    MemoryAllocator& allocator = mMemoryManager.getSingleFrameAllocator();

    // Swap the previous and current contacts lists
    swapPreviousAndCurrentContacts();

    mPotentialContactManifolds.reserve(mPreviousContactManifolds->size());
    mPotentialContactPoints.reserve(mPreviousContactPoints->size());

    // Test the narrow-phase collision detection on the batches to be tested
    testNarrowPhaseCollision(mNarrowPhaseInput, true, allocator);

    // Process all the potential contacts after narrow-phase collision
    processAllPotentialContacts(mNarrowPhaseInput, true, mPotentialContactPoints,
                                mPotentialContactManifolds, mCurrentContactPairs);

    // Reduce the number of contact points in the manifolds
    reducePotentialContactManifolds(mCurrentContactPairs, mPotentialContactManifolds, mPotentialContactPoints);

    assert(mCurrentContactManifolds->size() == 0);
    assert(mCurrentContactPoints->size() == 0);

    // Create the actual narrow-phase contacts
    createContacts();

    // Compute the map from contact pairs ids to contact pair for the next frame
    computeMapPreviousContactPairs();

    mNarrowPhaseInput.clear();
}

/// Compute the map from contact pairs ids to contact pair for the next frame
void CollisionDetectionSystem::computeMapPreviousContactPairs() {

    mPreviousMapPairIdToContactPairIndex.clear();
    for (uint i=0; i < mCurrentContactPairs->size(); i++) {
        mPreviousMapPairIdToContactPairIndex.add(Pair<uint64, uint>((*mCurrentContactPairs)[i].pairId, i));
    }
}

// Compute the narrow-phase collision detection for the testOverlap() methods.
/// This method returns true if contacts are found.
bool CollisionDetectionSystem::computeNarrowPhaseOverlapSnapshot(NarrowPhaseInput& narrowPhaseInput, OverlapCallback* callback) {

    RP3D_PROFILE("CollisionDetectionSystem::computeNarrowPhaseOverlapSnapshot()", mProfiler);

    MemoryAllocator& allocator = mMemoryManager.getPoolAllocator();

    // Test the narrow-phase collision detection on the batches to be tested
    bool collisionFound = testNarrowPhaseCollision(narrowPhaseInput, false, allocator);
    if (collisionFound && callback != nullptr) {

        // Compute the overlapping colliders
        List<ContactPair> contactPairs(allocator);
        List<ContactPair> lostContactPairs(allocator);          // Always empty in this case (snapshot)
        computeOverlapSnapshotContactPairs(narrowPhaseInput, contactPairs);

        // Report overlapping colliders
        OverlapCallback::CallbackData callbackData(contactPairs, lostContactPairs, false, *mWorld);
        (*callback).onOverlap(callbackData);
    }

    return collisionFound;
}

// Process the potential overlapping bodies  for the testOverlap() methods
void CollisionDetectionSystem::computeOverlapSnapshotContactPairs(NarrowPhaseInput& narrowPhaseInput, List<ContactPair>& contactPairs) const {

    Set<uint64> setOverlapContactPairId(mMemoryManager.getHeapAllocator());

    // get the narrow-phase batches to test for collision
    NarrowPhaseInfoBatch& sphereVsSphereBatch = narrowPhaseInput.getSphereVsSphereBatch();
    NarrowPhaseInfoBatch& sphereVsCapsuleBatch = narrowPhaseInput.getSphereVsCapsuleBatch();
    NarrowPhaseInfoBatch& capsuleVsCapsuleBatch = narrowPhaseInput.getCapsuleVsCapsuleBatch();
    NarrowPhaseInfoBatch& sphereVsConvexPolyhedronBatch = narrowPhaseInput.getSphereVsConvexPolyhedronBatch();
    NarrowPhaseInfoBatch& capsuleVsConvexPolyhedronBatch = narrowPhaseInput.getCapsuleVsConvexPolyhedronBatch();
    NarrowPhaseInfoBatch& convexPolyhedronVsConvexPolyhedronBatch = narrowPhaseInput.getConvexPolyhedronVsConvexPolyhedronBatch();

    // Process the potential contacts
    computeOverlapSnapshotContactPairs(sphereVsSphereBatch, contactPairs, setOverlapContactPairId);
    computeOverlapSnapshotContactPairs(sphereVsCapsuleBatch, contactPairs, setOverlapContactPairId);
    computeOverlapSnapshotContactPairs(capsuleVsCapsuleBatch, contactPairs, setOverlapContactPairId);
    computeOverlapSnapshotContactPairs(sphereVsConvexPolyhedronBatch, contactPairs, setOverlapContactPairId);
    computeOverlapSnapshotContactPairs(capsuleVsConvexPolyhedronBatch, contactPairs, setOverlapContactPairId);
    computeOverlapSnapshotContactPairs(convexPolyhedronVsConvexPolyhedronBatch, contactPairs, setOverlapContactPairId);
}

// Notify that the overlapping pairs where a given collider is involved need to be tested for overlap
void CollisionDetectionSystem::notifyOverlappingPairsToTestOverlap(Collider* collider) {

    // Get the overlapping pairs involved with this collider
    List<uint64>& overlappingPairs = mCollidersComponents.getOverlappingPairs(collider->getEntity());

    for (uint i=0; i < overlappingPairs.size(); i++) {

        // Notify that the overlapping pair needs to be testbed for overlap
        mOverlappingPairs.setNeedToTestOverlap(overlappingPairs[i], true);
    }
}

// Convert the potential overlapping bodies for the testOverlap() methods
void CollisionDetectionSystem::computeOverlapSnapshotContactPairs(NarrowPhaseInfoBatch& narrowPhaseInfoBatch, List<ContactPair>& contactPairs,
                                                           Set<uint64>& setOverlapContactPairId) const {

    RP3D_PROFILE("CollisionDetectionSystem::computeSnapshotContactPairs()", mProfiler);

    // For each narrow phase info object
    for(uint i=0; i < narrowPhaseInfoBatch.getNbObjects(); i++) {

        // If there is a collision
        if (narrowPhaseInfoBatch.narrowPhaseInfos[i].isColliding) {

            // If the contact pair does not already exist
            if (!setOverlapContactPairId.contains(narrowPhaseInfoBatch.narrowPhaseInfos[i].overlappingPairId)) {

                const Entity collider1Entity = narrowPhaseInfoBatch.narrowPhaseInfos[i].colliderEntity1;
                const Entity collider2Entity = narrowPhaseInfoBatch.narrowPhaseInfos[i].colliderEntity2;

                const uint32 collider1Index = mCollidersComponents.getEntityIndex(collider1Entity);
                const uint32 collider2Index = mCollidersComponents.getEntityIndex(collider2Entity);

                const Entity body1Entity = mCollidersComponents.mBodiesEntities[collider1Index];
                const Entity body2Entity = mCollidersComponents.mBodiesEntities[collider2Index];

                const bool isTrigger = mCollidersComponents.mIsTrigger[collider1Index] || mCollidersComponents.mIsTrigger[collider2Index];

                // Create a new contact pair
                ContactPair contactPair(narrowPhaseInfoBatch.narrowPhaseInfos[i].overlappingPairId, body1Entity, body2Entity, collider1Entity, collider2Entity, contactPairs.size(), isTrigger, false);
                contactPairs.add(contactPair);

                setOverlapContactPairId.add(narrowPhaseInfoBatch.narrowPhaseInfos[i].overlappingPairId);
            }
        }

        narrowPhaseInfoBatch.resetContactPoints(i);
    }
}

// Compute the narrow-phase collision detection for the testCollision() methods.
// This method returns true if contacts are found.
bool CollisionDetectionSystem::computeNarrowPhaseCollisionSnapshot(NarrowPhaseInput& narrowPhaseInput, CollisionCallback& callback) {

    RP3D_PROFILE("CollisionDetectionSystem::computeNarrowPhaseCollisionSnapshot()", mProfiler);

    MemoryAllocator& allocator = mMemoryManager.getHeapAllocator();

    // Test the narrow-phase collision detection on the batches to be tested
    bool collisionFound = testNarrowPhaseCollision(narrowPhaseInput, false, allocator);

    // If collision has been found, create contacts
    if (collisionFound) {

        List<ContactPointInfo> potentialContactPoints(allocator);
        List<ContactManifoldInfo> potentialContactManifolds(allocator);
        List<ContactPair> contactPairs(allocator);
        List<ContactPair> lostContactPairs(allocator);                  // Not used during collision snapshots
        List<ContactManifold> contactManifolds(allocator);
        List<ContactPoint> contactPoints(allocator);

        // Process all the potential contacts after narrow-phase collision
        processAllPotentialContacts(narrowPhaseInput, true, potentialContactPoints, potentialContactManifolds, &contactPairs);

        // Reduce the number of contact points in the manifolds
        reducePotentialContactManifolds(&contactPairs, potentialContactManifolds, potentialContactPoints);

        // Create the actual contact manifolds and contact points
        createSnapshotContacts(contactPairs, contactManifolds, contactPoints, potentialContactManifolds, potentialContactPoints);

        // Report the contacts to the user
        reportContacts(callback, &contactPairs, &contactManifolds, &contactPoints, lostContactPairs);
    }

    return collisionFound;
}

// Swap the previous and current contacts lists
void CollisionDetectionSystem::swapPreviousAndCurrentContacts() {

    if (mPreviousContactPairs == &mContactPairs1) {

        mPreviousContactPairs = &mContactPairs2;
        mPreviousContactManifolds = &mContactManifolds2;
        mPreviousContactPoints = &mContactPoints2;

        mCurrentContactPairs = &mContactPairs1;
        mCurrentContactManifolds = &mContactManifolds1;
        mCurrentContactPoints = &mContactPoints1;
    }
    else {

        mPreviousContactPairs = &mContactPairs1;
        mPreviousContactManifolds = &mContactManifolds1;
        mPreviousContactPoints = &mContactPoints1;

        mCurrentContactPairs = &mContactPairs2;
        mCurrentContactManifolds = &mContactManifolds2;
        mCurrentContactPoints = &mContactPoints2;
    }
}

// Create the actual contact manifolds and contacts points
void CollisionDetectionSystem::createContacts() {

    RP3D_PROFILE("CollisionDetectionSystem::createContacts()", mProfiler);

    mCurrentContactManifolds->reserve(mCurrentContactPairs->size());
    mCurrentContactPoints->reserve(mCurrentContactManifolds->size());

    // For each contact pair
    const uint nbCurrentContactPairs = (*mCurrentContactPairs).size();
    for (uint p=0; p < nbCurrentContactPairs; p++) {

        ContactPair& contactPair = (*mCurrentContactPairs)[p];

        contactPair.contactManifoldsIndex = mCurrentContactManifolds->size();
        contactPair.nbContactManifolds = contactPair.nbPotentialContactManifolds;
        contactPair.contactPointsIndex = mCurrentContactPoints->size();

        // Add the associated contact pair to both bodies of the pair (used to create islands later)
        uint32 rigidBody1Index = 0;
        uint32 rigidBody2Index = 0;
        if (mRigidBodyComponents.hasComponentGetIndex(contactPair.body1Entity, rigidBody1Index)) {
           mRigidBodyComponents.addContacPair(contactPair.body1Entity, p);
        }
        if (mRigidBodyComponents.hasComponentGetIndex(contactPair.body2Entity, rigidBody2Index)) {
           mRigidBodyComponents.addContacPair(contactPair.body2Entity, p);
        }

        // For each potential contact manifold of the pair
        for (uint m=0; m < contactPair.nbPotentialContactManifolds; m++) {

            ContactManifoldInfo& potentialManifold = mPotentialContactManifolds[contactPair.potentialContactManifoldsIndices[m]];

            // Start index and number of contact points for this manifold
            const uint contactPointsIndex = mCurrentContactPoints->size();
            const int8 nbContactPoints = static_cast<int8>(potentialManifold.nbPotentialContactPoints);
            contactPair.nbToTalContactPoints += nbContactPoints;

            // Create and add the contact manifold
            mCurrentContactManifolds->emplace(contactPair.body1Entity, contactPair.body2Entity, contactPair.collider1Entity,
                                              contactPair.collider2Entity, contactPointsIndex, nbContactPoints, rigidBody1Index, rigidBody2Index);

            assert(potentialManifold.nbPotentialContactPoints > 0);

            // For each contact point of the manifold
            for (uint c=0; c < potentialManifold.nbPotentialContactPoints; c++) {

                ContactPointInfo& potentialContactPoint = mPotentialContactPoints[potentialManifold.potentialContactPointsIndices[c]];

                // Create and add the contact point
                mCurrentContactPoints->emplace(potentialContactPoint, mWorld->mConfig.persistentContactDistanceThreshold);
            }
        }
    }

    // Initialize the current contacts with the contacts from the previous frame (for warmstarting)
    initContactsWithPreviousOnes();

    // Compute the lost contacts (contact pairs that were colliding in previous frame but not in this one)
    computeLostContactPairs();

    mPreviousContactPoints->clear();
    mPreviousContactManifolds->clear();
    mPreviousContactPairs->clear();

    // Reset the potential contacts
    mPotentialContactPoints.clear(true);
    mPotentialContactManifolds.clear(true);
}

// Compute the lost contact pairs (contact pairs in contact in the previous frame but not in the current one)
void CollisionDetectionSystem::computeLostContactPairs() {

    // For each overlapping pair
    for (uint i=0; i < mOverlappingPairs.getNbPairs(); i++) {

        // If the two colliders of the pair were colliding in the previous frame but not in the current one
        if (mOverlappingPairs.mCollidingInPreviousFrame[i] && !mOverlappingPairs.mCollidingInCurrentFrame[i]) {

            // If both bodies still exist
            if (mCollidersComponents.hasComponent(mOverlappingPairs.mColliders1[i]) && mCollidersComponents.hasComponent(mOverlappingPairs.mColliders2[i])) {

                // Create a lost contact pair
                addLostContactPair(i);
            }
        }
    }
}

// Create the actual contact manifolds and contacts points for testCollision() methods
void CollisionDetectionSystem::createSnapshotContacts(List<ContactPair>& contactPairs,
                                                 List<ContactManifold>& contactManifolds,
                                                 List<ContactPoint>& contactPoints,
                                                 List<ContactManifoldInfo>& potentialContactManifolds,
                                                 List<ContactPointInfo>& potentialContactPoints) {

    RP3D_PROFILE("CollisionDetectionSystem::createSnapshotContacts()", mProfiler);

    contactManifolds.reserve(contactPairs.size());
    contactPoints.reserve(contactManifolds.size());

    // For each contact pair
    const uint nbContactPairs = contactPairs.size();
    for (uint p=0; p < nbContactPairs; p++) {

        ContactPair& contactPair = contactPairs[p];
        assert(contactPair.nbPotentialContactManifolds > 0);

        contactPair.contactManifoldsIndex = contactManifolds.size();
        contactPair.nbContactManifolds = contactPair.nbPotentialContactManifolds;
        contactPair.contactPointsIndex = contactPoints.size();

        // For each potential contact manifold of the pair
        for (uint m=0; m < contactPair.nbPotentialContactManifolds; m++) {

            ContactManifoldInfo& potentialManifold = potentialContactManifolds[contactPair.potentialContactManifoldsIndices[m]];

            // Start index and number of contact points for this manifold
            const uint contactPointsIndex = contactPoints.size();
            const uint8 nbContactPoints = potentialManifold.nbPotentialContactPoints;
            contactPair.nbToTalContactPoints += nbContactPoints;

            // Create and add the contact manifold
            contactManifolds.emplace(contactPair.body1Entity, contactPair.body2Entity, contactPair.collider1Entity,
                                     contactPair.collider2Entity, contactPointsIndex, nbContactPoints, 0, 0);

            assert(potentialManifold.nbPotentialContactPoints > 0);

            // For each contact point of the manifold
            for (uint c=0; c < potentialManifold.nbPotentialContactPoints; c++) {

                ContactPointInfo& potentialContactPoint = potentialContactPoints[potentialManifold.potentialContactPointsIndices[c]];

                // Create a new contact point
                ContactPoint contactPoint(potentialContactPoint, mWorld->mConfig.persistentContactDistanceThreshold);

                // Add the contact point
                contactPoints.add(contactPoint);
            }
        }
    }
}

// Initialize the current contacts with the contacts from the previous frame (for warmstarting)
void CollisionDetectionSystem::initContactsWithPreviousOnes() {

    // For each contact pair of the current frame
    for (uint i=0; i < mCurrentContactPairs->size(); i++) {

        ContactPair& currentContactPair = (*mCurrentContactPairs)[i];

        // Find the corresponding contact pair in the previous frame (if any)
        auto itPrevContactPair = mPreviousMapPairIdToContactPairIndex.find(currentContactPair.pairId);

        // If we have found a corresponding contact pair in the previous frame
        if (itPrevContactPair != mPreviousMapPairIdToContactPairIndex.end()) {

            const uint previousContactPairIndex = itPrevContactPair->second;
            ContactPair& previousContactPair = (*mPreviousContactPairs)[previousContactPairIndex];

            // --------------------- Contact Manifolds --------------------- //

            const uint contactManifoldsIndex = currentContactPair.contactManifoldsIndex;
            const uint nbContactManifolds = currentContactPair.nbContactManifolds;

            // For each current contact manifold of the current contact pair
            for (uint m=contactManifoldsIndex; m < contactManifoldsIndex + nbContactManifolds; m++) {

                assert(m < mCurrentContactManifolds->size());
                ContactManifold& currentContactManifold = (*mCurrentContactManifolds)[m];
                assert(currentContactManifold.nbContactPoints > 0);
                ContactPoint& currentContactPoint = (*mCurrentContactPoints)[currentContactManifold.contactPointsIndex];
                const Vector3& currentContactPointNormal = currentContactPoint.getNormal();

                // Find a similar contact manifold among the contact manifolds from the previous frame (for warmstarting)
                const uint previousContactManifoldIndex = previousContactPair.contactManifoldsIndex;
                const uint previousNbContactManifolds = previousContactPair.nbContactManifolds;
                for (uint p=previousContactManifoldIndex; p < previousContactManifoldIndex + previousNbContactManifolds; p++) {

                    ContactManifold& previousContactManifold = (*mPreviousContactManifolds)[p];
                    assert(previousContactManifold.nbContactPoints > 0);
                    ContactPoint& previousContactPoint = (*mPreviousContactPoints)[previousContactManifold.contactPointsIndex];

                    // If the previous contact manifold has a similar contact normal with the current manifold
                    if (previousContactPoint.getNormal().dot(currentContactPointNormal) >= mWorld->mConfig.cosAngleSimilarContactManifold) {

                        // Transfer data from the previous contact manifold to the current one
                        currentContactManifold.frictionVector1 = previousContactManifold.frictionVector1;
                        currentContactManifold.frictionVector2 = previousContactManifold.frictionVector2;
                        currentContactManifold.frictionImpulse1 = previousContactManifold.frictionImpulse1;
                        currentContactManifold.frictionImpulse2 = previousContactManifold.frictionImpulse2;
                        currentContactManifold.frictionTwistImpulse = previousContactManifold.frictionTwistImpulse;
                        currentContactManifold.rollingResistanceImpulse = previousContactManifold.rollingResistanceImpulse;

                        break;
                    }
                }
            }

            // --------------------- Contact Points --------------------- //

            const uint contactPointsIndex = currentContactPair.contactPointsIndex;
            const uint nbTotalContactPoints = currentContactPair.nbToTalContactPoints;

            // For each current contact point of the current contact pair
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
}

// Remove a body from the collision detection
void CollisionDetectionSystem::removeCollider(Collider* collider) {

    const int colliderBroadPhaseId = collider->getBroadPhaseId();

    assert(colliderBroadPhaseId != -1);
    assert(mMapBroadPhaseIdToColliderEntity.containsKey(colliderBroadPhaseId));

    // Remove all the overlapping pairs involving this collider
    List<uint64>& overlappingPairs = mCollidersComponents.getOverlappingPairs(collider->getEntity());
    while(overlappingPairs.size() > 0) {

        // TODO : Remove all the contact manifold of the overlapping pair from the contact manifolds list of the two bodies involved

        // Remove the overlapping pair
        mOverlappingPairs.removePair(overlappingPairs[0]);
    }

    mMapBroadPhaseIdToColliderEntity.remove(colliderBroadPhaseId);

    // Remove the body from the broad-phase
    mBroadPhaseSystem.removeCollider(collider);
}

// Ray casting method
void CollisionDetectionSystem::raycast(RaycastCallback* raycastCallback,
                                        const Ray& ray,
                                        unsigned short raycastWithCategoryMaskBits) const {

    RP3D_PROFILE("CollisionDetectionSystem::raycast()", mProfiler);

    RaycastTest rayCastTest(raycastCallback);

    // Ask the broad-phase algorithm to call the testRaycastAgainstShape()
    // callback method for each collider hit by the ray in the broad-phase
    mBroadPhaseSystem.raycast(ray, rayCastTest, raycastWithCategoryMaskBits);
}

// Convert the potential contact into actual contacts
void CollisionDetectionSystem::processPotentialContacts(NarrowPhaseInfoBatch& narrowPhaseInfoBatch, bool updateLastFrameInfo,
                                                        List<ContactPointInfo>& potentialContactPoints,
                                                        List<ContactManifoldInfo>& potentialContactManifolds,
                                                        Map<uint64, uint>& mapPairIdToContactPairIndex,
                                                        List<ContactPair>* contactPairs) {

    RP3D_PROFILE("CollisionDetectionSystem::processPotentialContacts()", mProfiler);

    const uint nbObjects = narrowPhaseInfoBatch.getNbObjects();

    if (updateLastFrameInfo) {

        // For each narrow phase info object
        for(uint i=0; i < nbObjects; i++) {

            narrowPhaseInfoBatch.narrowPhaseInfos[i].lastFrameCollisionInfo->wasColliding = narrowPhaseInfoBatch.narrowPhaseInfos[i].isColliding;

            // The previous frame collision info is now valid
            narrowPhaseInfoBatch.narrowPhaseInfos[i].lastFrameCollisionInfo->isValid = true;
        }
    }

    // For each narrow phase info object
    for(uint i=0; i < nbObjects; i++) {

        const uint64 pairId = narrowPhaseInfoBatch.narrowPhaseInfos[i].overlappingPairId;
        const uint64 pairIndex = mOverlappingPairs.mMapPairIdToPairIndex[pairId];

        // If the two colliders are colliding
        if (narrowPhaseInfoBatch.narrowPhaseInfos[i].isColliding) {

            mOverlappingPairs.mCollidingInCurrentFrame[pairIndex] = true;

            const Entity collider1Entity = narrowPhaseInfoBatch.narrowPhaseInfos[i].colliderEntity1;
            const Entity collider2Entity = narrowPhaseInfoBatch.narrowPhaseInfos[i].colliderEntity2;

            const uint32 collider1Index = mCollidersComponents.getEntityIndex(collider1Entity);
            const uint32 collider2Index = mCollidersComponents.getEntityIndex(collider2Entity);

            const Entity body1Entity = mCollidersComponents.mBodiesEntities[collider1Index];
            const Entity body2Entity = mCollidersComponents.mBodiesEntities[collider2Index];

            // If we have a convex vs convex collision (if we consider the base collision shapes of the colliders)
            if (mCollidersComponents.mCollisionShapes[collider1Index]->isConvex() &&
                mCollidersComponents.mCollisionShapes[collider2Index]->isConvex()) {

                // Create a new ContactPair

                const bool isTrigger = mCollidersComponents.mIsTrigger[collider1Index] || mCollidersComponents.mIsTrigger[collider2Index];

                assert(!mWorld->mCollisionBodyComponents.getIsEntityDisabled(body1Entity) || !mWorld->mCollisionBodyComponents.getIsEntityDisabled(body2Entity));

                const uint newContactPairIndex = contactPairs->size();

                contactPairs->emplace(pairId, body1Entity, body2Entity, collider1Entity, collider2Entity,
                                      newContactPairIndex, mOverlappingPairs.getCollidingInPreviousFrame(pairId), isTrigger);

                ContactPair* pairContact = &((*contactPairs)[newContactPairIndex]);

                // Create a new potential contact manifold for the overlapping pair
                uint contactManifoldIndex = static_cast<uint>(potentialContactManifolds.size());
                potentialContactManifolds.emplace(pairId);
                ContactManifoldInfo& contactManifoldInfo = potentialContactManifolds[contactManifoldIndex];

                const uint contactPointIndexStart = static_cast<uint>(potentialContactPoints.size());

                // Add the potential contacts
                for (uint j=0; j < narrowPhaseInfoBatch.narrowPhaseInfos[i].nbContactPoints; j++) {

                    if (contactManifoldInfo.nbPotentialContactPoints < NB_MAX_CONTACT_POINTS_IN_POTENTIAL_MANIFOLD) {

                        // Add the contact point to the manifold
                        contactManifoldInfo.potentialContactPointsIndices[contactManifoldInfo.nbPotentialContactPoints] = contactPointIndexStart + j;
                        contactManifoldInfo.nbPotentialContactPoints++;

                        // Add the contact point to the list of potential contact points
                        const ContactPointInfo& contactPoint = narrowPhaseInfoBatch.narrowPhaseInfos[i].contactPoints[j];

                        potentialContactPoints.add(contactPoint);
                    }
                }

                // Add the contact manifold to the overlapping pair contact
                assert(pairId == contactManifoldInfo.pairId);
                pairContact->potentialContactManifoldsIndices[0] = contactManifoldIndex;
                pairContact->nbPotentialContactManifolds = 1;
            }
            else {

                // If there is not already a contact pair for this overlapping pair
                auto it = mapPairIdToContactPairIndex.find(pairId);
                ContactPair* pairContact = nullptr;
                if (it == mapPairIdToContactPairIndex.end()) {

                    // Create a new ContactPair

                    const bool isTrigger = mCollidersComponents.mIsTrigger[collider1Index] || mCollidersComponents.mIsTrigger[collider2Index];

                    assert(!mWorld->mCollisionBodyComponents.getIsEntityDisabled(body1Entity) || !mWorld->mCollisionBodyComponents.getIsEntityDisabled(body2Entity));

                    const uint newContactPairIndex = contactPairs->size();
                    contactPairs->emplace(pairId, body1Entity, body2Entity, collider1Entity, collider2Entity,
                                                       newContactPairIndex, mOverlappingPairs.getCollidingInPreviousFrame(pairId), isTrigger);
                    pairContact = &((*contactPairs)[newContactPairIndex]);
                    mapPairIdToContactPairIndex.add(Pair<uint64, uint>(pairId, newContactPairIndex));

                }
                else { // If a ContactPair already exists for this overlapping pair, we use this one

                    assert(it->first == pairId);

                    const uint pairContactIndex = it->second;
                    pairContact = &((*contactPairs)[pairContactIndex]);
                }

                assert(pairContact != nullptr);

                // Add the potential contacts
                for (uint j=0; j < narrowPhaseInfoBatch.narrowPhaseInfos[i].nbContactPoints; j++) {

                    const ContactPointInfo& contactPoint = narrowPhaseInfoBatch.narrowPhaseInfos[i].contactPoints[j];

                    // Add the contact point to the list of potential contact points
                    const uint contactPointIndex = static_cast<uint>(potentialContactPoints.size());

                    potentialContactPoints.add(contactPoint);

                    bool similarManifoldFound = false;

                    // For each contact manifold of the overlapping pair
                    for (uint m=0; m < pairContact->nbPotentialContactManifolds; m++) {

                       uint contactManifoldIndex = pairContact->potentialContactManifoldsIndices[m];

                       if (potentialContactManifolds[contactManifoldIndex].nbPotentialContactPoints < NB_MAX_CONTACT_POINTS_IN_POTENTIAL_MANIFOLD) {

                           // Get the first contact point of the current manifold
                           assert(potentialContactManifolds[contactManifoldIndex].nbPotentialContactPoints > 0);
                           const uint manifoldContactPointIndex = potentialContactManifolds[contactManifoldIndex].potentialContactPointsIndices[0];
                           const ContactPointInfo& manifoldContactPoint = potentialContactPoints[manifoldContactPointIndex];

                            // If we have found a corresponding manifold for the new contact point
                            // (a manifold with a similar contact normal direction)
                            if (manifoldContactPoint.normal.dot(contactPoint.normal) >= mWorld->mConfig.cosAngleSimilarContactManifold) {

                                // Add the contact point to the manifold
                                potentialContactManifolds[contactManifoldIndex].potentialContactPointsIndices[potentialContactManifolds[contactManifoldIndex].nbPotentialContactPoints] = contactPointIndex;
                                potentialContactManifolds[contactManifoldIndex].nbPotentialContactPoints++;

                                similarManifoldFound = true;

                                break;
                            }
                       }
                    }

                    // If we have not found a manifold with a similar contact normal for the contact point
                    if (!similarManifoldFound && pairContact->nbPotentialContactManifolds < NB_MAX_POTENTIAL_CONTACT_MANIFOLDS) {

                        // Create a new potential contact manifold for the overlapping pair
                        uint contactManifoldIndex = static_cast<uint>(potentialContactManifolds.size());
                        potentialContactManifolds.emplace(pairId);
                        ContactManifoldInfo& contactManifoldInfo = potentialContactManifolds[contactManifoldIndex];

                        // Add the contact point to the manifold
                        contactManifoldInfo.potentialContactPointsIndices[0] = contactPointIndex;
                        contactManifoldInfo.nbPotentialContactPoints = 1;

                        assert(pairContact != nullptr);

                        // Add the contact manifold to the overlapping pair contact
                        assert(pairContact->pairId == contactManifoldInfo.pairId);
                        pairContact->potentialContactManifoldsIndices[pairContact->nbPotentialContactManifolds] = contactManifoldIndex;
                        pairContact->nbPotentialContactManifolds++;
                    }

                    assert(pairContact->nbPotentialContactManifolds > 0);
                }
            }

            narrowPhaseInfoBatch.resetContactPoints(i);
        }
    }
}

// Clear the obsolete manifolds and contact points and reduce the number of contacts points of the remaining manifolds
void CollisionDetectionSystem::reducePotentialContactManifolds(List<ContactPair>* contactPairs,
                                                         List<ContactManifoldInfo>& potentialContactManifolds,
                                                         const List<ContactPointInfo>& potentialContactPoints) const {

    RP3D_PROFILE("CollisionDetectionSystem::reducePotentialContactManifolds()", mProfiler);

    // Reduce the number of potential contact manifolds in a contact pair
    const uint nbContactPairs = contactPairs->size();
    for (uint i=0; i < nbContactPairs; i++) {

        ContactPair& contactPair = (*contactPairs)[i];

        // While there are too many manifolds in the contact pair
        while(contactPair.nbPotentialContactManifolds > NB_MAX_CONTACT_MANIFOLDS) {

            // Look for a manifold with the smallest contact penetration depth.
            decimal minDepth = DECIMAL_LARGEST;
            int minDepthManifoldIndex = -1;
            for (uint j=0; j < contactPair.nbPotentialContactManifolds; j++) {

                ContactManifoldInfo& manifold = potentialContactManifolds[contactPair.potentialContactManifoldsIndices[j]];

                // Get the largest contact point penetration depth of the manifold
                const decimal depth = computePotentialManifoldLargestContactDepth(manifold, potentialContactPoints);

                if (depth < minDepth) {
                    minDepth = depth;
                    minDepthManifoldIndex = static_cast<int>(j);
                }
            }

            // Remove the non optimal manifold
            assert(minDepthManifoldIndex >= 0);
            contactPair.removePotentialManifoldAtIndex(minDepthManifoldIndex);
        }
    }

    // Reduce the number of potential contact points in the manifolds
    for (uint i=0; i < nbContactPairs; i++) {

        const ContactPair& pairContact = (*contactPairs)[i];

        // For each potential contact manifold
        for (uint j=0; j < pairContact.nbPotentialContactManifolds; j++) {

            ContactManifoldInfo& manifold = potentialContactManifolds[pairContact.potentialContactManifoldsIndices[j]];

            // If there are two many contact points in the manifold
            if (manifold.nbPotentialContactPoints > MAX_CONTACT_POINTS_IN_MANIFOLD) {

                Entity collider1 = mOverlappingPairs.mColliders1[mOverlappingPairs.mMapPairIdToPairIndex[manifold.pairId]];

                Transform shape1LocalToWorldTransoform = mCollidersComponents.getLocalToWorldTransform(collider1);

                // Reduce the number of contact points in the manifold
                reduceContactPoints(manifold, shape1LocalToWorldTransoform, potentialContactPoints);
            }

            assert(manifold.nbPotentialContactPoints <= MAX_CONTACT_POINTS_IN_MANIFOLD);
        }
    }
}

// Return the largest depth of all the contact points of a potential manifold
decimal CollisionDetectionSystem::computePotentialManifoldLargestContactDepth(const ContactManifoldInfo& manifold,
                                                                        const List<ContactPointInfo>& potentialContactPoints) const {

    decimal largestDepth = 0.0f;

    assert(manifold.nbPotentialContactPoints > 0);

    for (uint i=0; i < manifold.nbPotentialContactPoints; i++) {
        decimal depth = potentialContactPoints[manifold.potentialContactPointsIndices[i]].penetrationDepth;

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
void CollisionDetectionSystem::reduceContactPoints(ContactManifoldInfo& manifold, const Transform& shape1ToWorldTransform,
                                             const List<ContactPointInfo>& potentialContactPoints) const {

    assert(manifold.nbPotentialContactPoints > MAX_CONTACT_POINTS_IN_MANIFOLD);

    // The following algorithm only works to reduce to a maximum of 4 contact points
    assert(MAX_CONTACT_POINTS_IN_MANIFOLD == 4);

    // List of the candidate contact points indices in the manifold. Every time that we have found a
    // point we want to keep, we will remove it from this list
    uint candidatePointsIndices[NB_MAX_CONTACT_POINTS_IN_POTENTIAL_MANIFOLD];
    uint8 nbCandidatePoints = manifold.nbPotentialContactPoints;
    for (uint8 i=0 ; i < manifold.nbPotentialContactPoints; i++) {
        candidatePointsIndices[i] = manifold.potentialContactPointsIndices[i];
    }

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
    const Vector3 contactNormalShape1Space = worldToShape1Transform.getOrientation() * potentialContactPoints[candidatePointsIndices[0]].normal;

    // Compute a search direction
    const Vector3 searchDirection(1, 1, 1);
    decimal maxDotProduct = DECIMAL_SMALLEST;
    uint elementIndexToKeep = 0;
    for (uint i=0; i < nbCandidatePoints; i++) {

        const ContactPointInfo& element = potentialContactPoints[candidatePointsIndices[i]];
        decimal dotProduct = searchDirection.dot(element.localPoint1);
        if (dotProduct > maxDotProduct) {
            maxDotProduct = dotProduct;
            elementIndexToKeep = i;
            nbReducedPoints = 1;
        }
    }
    pointsToKeepIndices[0] = candidatePointsIndices[elementIndexToKeep];
    removeItemAtInArray(candidatePointsIndices, elementIndexToKeep, nbCandidatePoints);
    //candidatePointsIndices.removeAt(elementIndexToKeep);
    assert(nbReducedPoints == 1);

    // Compute the second contact point we need to keep.
    // The second point we keep is the one farthest away from the first point.

    decimal maxDistance = decimal(0.0);
    elementIndexToKeep = 0;
    for (uint i=0; i < nbCandidatePoints; i++) {

        const ContactPointInfo& element = potentialContactPoints[candidatePointsIndices[i]];
        const ContactPointInfo& pointToKeep0 = potentialContactPoints[pointsToKeepIndices[0]];

        assert(candidatePointsIndices[i] != pointsToKeepIndices[0]);

        const decimal distance = (pointToKeep0.localPoint1 - element.localPoint1).lengthSquare();
        if (distance >= maxDistance) {
            maxDistance = distance;
            elementIndexToKeep = i;
            nbReducedPoints = 2;
        }

    }
    pointsToKeepIndices[1] = candidatePointsIndices[elementIndexToKeep];
    removeItemAtInArray(candidatePointsIndices, elementIndexToKeep, nbCandidatePoints);
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
    for (uint i=0; i < nbCandidatePoints; i++) {

        const ContactPointInfo& element = potentialContactPoints[candidatePointsIndices[i]];
        const ContactPointInfo& pointToKeep0 = potentialContactPoints[pointsToKeepIndices[0]];
        const ContactPointInfo& pointToKeep1 = potentialContactPoints[pointsToKeepIndices[1]];

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
        removeItemAtInArray(candidatePointsIndices, thirdPointMaxAreaIndex, nbCandidatePoints);
    }
    else {
        isPreviousAreaPositive = false;
        pointsToKeepIndices[2] = candidatePointsIndices[thirdPointMinAreaIndex];
        removeItemAtInArray(candidatePointsIndices, thirdPointMinAreaIndex, nbCandidatePoints);
    }
    nbReducedPoints = 3;

    // Compute the 4th point by choosing the triangle that adds the most
    // triangle area to the previous triangle and has opposite sign area (opposite winding)

    decimal largestArea = decimal(0.0); // Largest area (positive or negative)
    elementIndexToKeep = 0;
    nbReducedPoints = 4;
    decimal area;

    // For each remaining candidate points
    for (uint i=0; i < nbCandidatePoints; i++) {

        const ContactPointInfo& element = potentialContactPoints[candidatePointsIndices[i]];

        assert(candidatePointsIndices[i] != pointsToKeepIndices[0]);
        assert(candidatePointsIndices[i] != pointsToKeepIndices[1]);
        assert(candidatePointsIndices[i] != pointsToKeepIndices[2]);

        // For each edge of the triangle made by the first three points
        for (uint j=0; j<3; j++) {

            uint edgeVertex1Index = j;
            uint edgeVertex2Index = j < 2 ? j + 1 : 0;

            const ContactPointInfo& pointToKeepEdgeV1 = potentialContactPoints[pointsToKeepIndices[edgeVertex1Index]];
            const ContactPointInfo& pointToKeepEdgeV2 = potentialContactPoints[pointsToKeepIndices[edgeVertex2Index]];

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
    removeItemAtInArray(candidatePointsIndices, elementIndexToKeep, nbCandidatePoints);

    // Only keep the four selected contact points in the manifold
    manifold.potentialContactPointsIndices[0] = pointsToKeepIndices[0];
    manifold.potentialContactPointsIndices[1] = pointsToKeepIndices[1];
    manifold.potentialContactPointsIndices[2] = pointsToKeepIndices[2];
    manifold.potentialContactPointsIndices[3] = pointsToKeepIndices[3];
    manifold.nbPotentialContactPoints = 4;
}

// Remove an element in an array (and replace it by the last one in the array)
void CollisionDetectionSystem::removeItemAtInArray(uint array[], uint8 index, uint8& arraySize) const {
    assert(index < arraySize);
    assert(arraySize > 0);
    array[index] = array[arraySize - 1];
    arraySize--;
}

// Report contacts and triggers
void CollisionDetectionSystem::reportContactsAndTriggers() {

    // Report contacts and triggers to the user
    if (mWorld->mEventListener != nullptr) {

        reportContacts(*(mWorld->mEventListener), mCurrentContactPairs, mCurrentContactManifolds, mCurrentContactPoints, mLostContactPairs);
        reportTriggers(*(mWorld->mEventListener), mCurrentContactPairs, mLostContactPairs);
    }

    // Report contacts for debug rendering (if enabled)
    if (mWorld->mIsDebugRenderingEnabled) {

        reportDebugRenderingContacts(mCurrentContactPairs, mCurrentContactManifolds, mCurrentContactPoints, mLostContactPairs);
    }

    mOverlappingPairs.updateCollidingInPreviousFrame();

    mLostContactPairs.clear(true);
}

// Report all contacts to the user
void CollisionDetectionSystem::reportContacts(CollisionCallback& callback, List<ContactPair>* contactPairs,
                                              List<ContactManifold>* manifolds, List<ContactPoint>* contactPoints, List<ContactPair>& lostContactPairs) {

    RP3D_PROFILE("CollisionDetectionSystem::reportContacts()", mProfiler);

    // If there are contacts
    if (contactPairs->size() + lostContactPairs.size() > 0) {

        CollisionCallback::CallbackData callbackData(contactPairs, manifolds, contactPoints, lostContactPairs, *mWorld);

        // Call the callback method to report the contacts
        callback.onContact(callbackData);
    }
}

// Report all triggers to the user
void CollisionDetectionSystem::reportTriggers(EventListener& eventListener, List<ContactPair>* contactPairs, List<ContactPair>& lostContactPairs) {

    RP3D_PROFILE("CollisionDetectionSystem::reportTriggers()", mProfiler);

    // If there are contacts
    if (contactPairs->size() + lostContactPairs.size() > 0) {

        OverlapCallback::CallbackData callbackData(*contactPairs, lostContactPairs, true, *mWorld);

        // Call the callback method to report the overlapping shapes
        eventListener.onTrigger(callbackData);
    }
}

// Report all contacts for debug rendering
void CollisionDetectionSystem::reportDebugRenderingContacts(List<ContactPair>* contactPairs, List<ContactManifold>* manifolds, List<ContactPoint>* contactPoints, List<ContactPair>& lostContactPairs) {

    RP3D_PROFILE("CollisionDetectionSystem::reportDebugRenderingContacts()", mProfiler);

    // If there are contacts
    if (contactPairs->size() + lostContactPairs.size() > 0) {

        CollisionCallback::CallbackData callbackData(contactPairs, manifolds, contactPoints, lostContactPairs, *mWorld);

        // Call the callback method to report the contacts
        mWorld->mDebugRenderer.onContact(callbackData);
    }
}

// Return true if two bodies overlap (collide)
bool CollisionDetectionSystem::testOverlap(CollisionBody* body1, CollisionBody* body2) {

    NarrowPhaseInput narrowPhaseInput(mMemoryManager.getPoolAllocator(), mOverlappingPairs);

    // Compute the broad-phase collision detection
    computeBroadPhase();

    // Filter the overlapping pairs to get only the ones with the selected body involved
    List<uint64> convexPairs(mMemoryManager.getPoolAllocator());
    List<uint64> concavePairs(mMemoryManager.getPoolAllocator());
    filterOverlappingPairs(body1->getEntity(), body2->getEntity(), convexPairs, concavePairs);

    if (convexPairs.size() > 0 || concavePairs.size() > 0) {

        // Compute the middle-phase collision detection
        computeMiddlePhaseCollisionSnapshot(convexPairs, concavePairs, narrowPhaseInput, false);

        // Compute the narrow-phase collision detection
        return computeNarrowPhaseOverlapSnapshot(narrowPhaseInput, nullptr);
    }

    return false;
}

// Report all the bodies that overlap (collide) in the world
void CollisionDetectionSystem::testOverlap(OverlapCallback& callback) {

    NarrowPhaseInput narrowPhaseInput(mMemoryManager.getPoolAllocator(), mOverlappingPairs);

    // Compute the broad-phase collision detection
    computeBroadPhase();

    // Compute the middle-phase collision detection
    computeMiddlePhase(narrowPhaseInput, false);

    // Compute the narrow-phase collision detection and report overlapping shapes
    computeNarrowPhaseOverlapSnapshot(narrowPhaseInput, &callback);
}

// Report all the bodies that overlap (collide) with the body in parameter
void CollisionDetectionSystem::testOverlap(CollisionBody* body, OverlapCallback& callback) {

    NarrowPhaseInput narrowPhaseInput(mMemoryManager.getPoolAllocator(), mOverlappingPairs);

    // Compute the broad-phase collision detection
    computeBroadPhase();

    // Filter the overlapping pairs to get only the ones with the selected body involved
    List<uint64> convexPairs(mMemoryManager.getPoolAllocator());
    List<uint64> concavePairs(mMemoryManager.getPoolAllocator());
    filterOverlappingPairs(body->getEntity(), convexPairs, concavePairs);

    if (convexPairs.size() > 0 || concavePairs.size() > 0) {

        // Compute the middle-phase collision detection
        computeMiddlePhaseCollisionSnapshot(convexPairs, concavePairs, narrowPhaseInput, false);

        // Compute the narrow-phase collision detection
        computeNarrowPhaseOverlapSnapshot(narrowPhaseInput, &callback);
    }
}

// Test collision and report contacts between two bodies.
void CollisionDetectionSystem::testCollision(CollisionBody* body1, CollisionBody* body2, CollisionCallback& callback) {

    NarrowPhaseInput narrowPhaseInput(mMemoryManager.getPoolAllocator(), mOverlappingPairs);

    // Compute the broad-phase collision detection
    computeBroadPhase();

    // Filter the overlapping pairs to get only the ones with the selected body involved
    List<uint64> convexPairs(mMemoryManager.getPoolAllocator());
    List<uint64> concavePairs(mMemoryManager.getPoolAllocator());
    filterOverlappingPairs(body1->getEntity(), body2->getEntity(), convexPairs, concavePairs);

    if (convexPairs.size() > 0 || concavePairs.size() > 0) {

        // Compute the middle-phase collision detection
        computeMiddlePhaseCollisionSnapshot(convexPairs, concavePairs, narrowPhaseInput, true);

        // Compute the narrow-phase collision detection and report contacts
        computeNarrowPhaseCollisionSnapshot(narrowPhaseInput, callback);
    }
}

// Test collision and report all the contacts involving the body in parameter
void CollisionDetectionSystem::testCollision(CollisionBody* body, CollisionCallback& callback) {

    NarrowPhaseInput narrowPhaseInput(mMemoryManager.getPoolAllocator(), mOverlappingPairs);

    // Compute the broad-phase collision detection
    computeBroadPhase();

    // Filter the overlapping pairs to get only the ones with the selected body involved
    List<uint64> convexPairs(mMemoryManager.getPoolAllocator());
    List<uint64> concavePairs(mMemoryManager.getPoolAllocator());
    filterOverlappingPairs(body->getEntity(), convexPairs, concavePairs);

    if (convexPairs.size() > 0 || concavePairs.size() > 0) {

        // Compute the middle-phase collision detection
        computeMiddlePhaseCollisionSnapshot(convexPairs, concavePairs, narrowPhaseInput, true);

        // Compute the narrow-phase collision detection and report contacts
        computeNarrowPhaseCollisionSnapshot(narrowPhaseInput, callback);
    }
}

// Test collision and report contacts between each colliding bodies in the world
void CollisionDetectionSystem::testCollision(CollisionCallback& callback) {

    NarrowPhaseInput narrowPhaseInput(mMemoryManager.getPoolAllocator(), mOverlappingPairs);

    // Compute the broad-phase collision detection
    computeBroadPhase();

    // Compute the middle-phase collision detection
    computeMiddlePhase(narrowPhaseInput, true);

    // Compute the narrow-phase collision detection and report contacts
    computeNarrowPhaseCollisionSnapshot(narrowPhaseInput, callback);
}

// Filter the overlapping pairs to keep only the pairs where a given body is involved
void CollisionDetectionSystem::filterOverlappingPairs(Entity bodyEntity, List<uint64>& convexPairs, List<uint64>& concavePairs) const {

    // For each possible collision pair of bodies
    for (uint i=0; i < mOverlappingPairs.getNbPairs(); i++) {

        if (mCollidersComponents.getBody(mOverlappingPairs.mColliders1[i]) == bodyEntity ||
            mCollidersComponents.getBody(mOverlappingPairs.mColliders2[i]) == bodyEntity) {

            if (i < mOverlappingPairs.getNbConvexVsConvexPairs()) {
                convexPairs.add(mOverlappingPairs.mPairIds[i]);
            }
            else {
                concavePairs.add(mOverlappingPairs.mPairIds[i]);
            }
        }
    }
}

// Filter the overlapping pairs to keep only the pairs where two given bodies are involved
void CollisionDetectionSystem::filterOverlappingPairs(Entity body1Entity, Entity body2Entity, List<uint64>& convexPairs, List<uint64>& concavePairs) const {

    // For each possible collision pair of bodies
    for (uint i=0; i < mOverlappingPairs.getNbPairs(); i++) {

        const Entity collider1Body = mCollidersComponents.getBody(mOverlappingPairs.mColliders1[i]);
        const Entity collider2Body = mCollidersComponents.getBody(mOverlappingPairs.mColliders2[i]);

        if ((collider1Body == body1Entity && collider2Body == body2Entity) ||
            (collider1Body == body2Entity && collider2Body == body1Entity)) {

            if (i < mOverlappingPairs.getNbConvexVsConvexPairs()) {
                convexPairs.add(mOverlappingPairs.mPairIds[i]);
            }
            else {
                concavePairs.add(mOverlappingPairs.mPairIds[i]);
            }
        }
    }
}

// Return the world event listener
EventListener* CollisionDetectionSystem::getWorldEventListener() {
   return mWorld->mEventListener;
}

// Return the world-space AABB of a given collider
const AABB CollisionDetectionSystem::getWorldAABB(const Collider* collider) const {
    assert(collider->getBroadPhaseId() > -1);
    return mBroadPhaseSystem.getFatAABB(collider->getBroadPhaseId());
}