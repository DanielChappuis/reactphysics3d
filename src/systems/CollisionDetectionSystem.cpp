/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2022 Daniel Chappuis                                       *
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
                                                   MemoryManager& memoryManager, HalfEdgeStructure& triangleHalfEdgeStructure)
                   : mMemoryManager(memoryManager), mCollidersComponents(collidersComponents), mRigidBodyComponents(rigidBodyComponents),
                     mCollisionDispatch(mMemoryManager.getPoolAllocator()), mWorld(world),
                     mNoCollisionPairs(mMemoryManager.getPoolAllocator()),
                     mOverlappingPairs(mMemoryManager, mCollidersComponents, collisionBodyComponents, rigidBodyComponents,
                                       mNoCollisionPairs, mCollisionDispatch),
                     mBroadPhaseOverlappingNodes(mMemoryManager.getHeapAllocator(), 32),
                     mBroadPhaseSystem(*this, mCollidersComponents, transformComponents, rigidBodyComponents),
                     mMapBroadPhaseIdToColliderEntity(memoryManager.getPoolAllocator()),
                     mNarrowPhaseInput(mMemoryManager.getSingleFrameAllocator(), mOverlappingPairs), mPotentialContactPoints(mMemoryManager.getSingleFrameAllocator()),
                     mPotentialContactManifolds(mMemoryManager.getSingleFrameAllocator()), mContactPairs1(mMemoryManager.getPoolAllocator()),
                     mContactPairs2(mMemoryManager.getPoolAllocator()), mPreviousContactPairs(&mContactPairs1), mCurrentContactPairs(&mContactPairs2),
                     mLostContactPairs(mMemoryManager.getSingleFrameAllocator()), mPreviousMapPairIdToContactPairIndex(mMemoryManager.getHeapAllocator()),
                     mContactManifolds1(mMemoryManager.getPoolAllocator()), mContactManifolds2(mMemoryManager.getPoolAllocator()),
                     mPreviousContactManifolds(&mContactManifolds1), mCurrentContactManifolds(&mContactManifolds2),
                     mContactPoints1(mMemoryManager.getPoolAllocator()), mContactPoints2(mMemoryManager.getPoolAllocator()),
                     mPreviousContactPoints(&mContactPoints1), mCurrentContactPoints(&mContactPoints2), mCollisionBodyContactPairsIndices(mMemoryManager.getSingleFrameAllocator()),
                     mNbPreviousPotentialContactManifolds(0), mNbPreviousPotentialContactPoints(0), mTriangleHalfEdgeStructure(triangleHalfEdgeStructure) {

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

    assert(mBroadPhaseOverlappingNodes.size() == 0);

    // Ask the broad-phase to compute all the shapes overlapping with the shapes that
    // have moved or have been added in the last frame. This call can only add new
    // overlapping pairs in the collision detection.
    mBroadPhaseSystem.computeOverlappingPairs(mMemoryManager, mBroadPhaseOverlappingNodes);

    // Create new overlapping pairs if necessary
    updateOverlappingPairs(mBroadPhaseOverlappingNodes);

    // Remove non overlapping pairs
    removeNonOverlappingPairs();

    mBroadPhaseOverlappingNodes.clear();
}

// Remove pairs that are not overlapping anymore
void CollisionDetectionSystem::removeNonOverlappingPairs() {

    RP3D_PROFILE("CollisionDetectionSystem::removeNonOverlappingPairs()", mProfiler);

    // For each convex pairs
    for (uint64 i=0; i < mOverlappingPairs.mConvexPairs.size(); i++) {

        OverlappingPairs::ConvexOverlappingPair& overlappingPair = mOverlappingPairs.mConvexPairs[i];

        // Check if we need to test overlap. If so, test if the two shapes are still overlapping.
        // Otherwise, we destroy the overlapping pair
        if (overlappingPair.needToTestOverlap) {

            if(mBroadPhaseSystem.testOverlappingShapes(overlappingPair.broadPhaseId1, overlappingPair.broadPhaseId2)) {
                overlappingPair.needToTestOverlap = false;
            }
            else {

                // If the two colliders of the pair were colliding in the previous frame
                if (overlappingPair.collidingInPreviousFrame) {

                    // Create a new lost contact pair
                    addLostContactPair(overlappingPair);
                }

                mOverlappingPairs.removePair(i, true);
                i--;
            }
        }
    }

    // For each concave pairs
    for (uint64 i=0; i < mOverlappingPairs.mConcavePairs.size(); i++) {

        OverlappingPairs::ConcaveOverlappingPair& overlappingPair = mOverlappingPairs.mConcavePairs[i];

        // Check if we need to test overlap. If so, test if the two shapes are still overlapping.
        // Otherwise, we destroy the overlapping pair
        if (overlappingPair.needToTestOverlap) {

            if(mBroadPhaseSystem.testOverlappingShapes(overlappingPair.broadPhaseId1, overlappingPair.broadPhaseId2)) {
                overlappingPair.needToTestOverlap = false;
            }
            else {

                // If the two colliders of the pair were colliding in the previous frame
                if (overlappingPair.collidingInPreviousFrame) {

                    // Create a new lost contact pair
                    addLostContactPair(overlappingPair);
                }

                mOverlappingPairs.removePair(i, false);
                i--;
            }
        }
    }
}

// Add a lost contact pair (pair of colliders that are not in contact anymore)
void CollisionDetectionSystem::addLostContactPair(OverlappingPairs::OverlappingPair& overlappingPair) {

    const uint32 collider1Index = mCollidersComponents.getEntityIndex(overlappingPair.collider1);
    const uint32 collider2Index = mCollidersComponents.getEntityIndex(overlappingPair.collider2);

    const Entity body1Entity = mCollidersComponents.mBodiesEntities[collider1Index];
    const Entity body2Entity = mCollidersComponents.mBodiesEntities[collider2Index];

    const bool isCollider1Trigger = mCollidersComponents.mIsTrigger[collider1Index];
    const bool isCollider2Trigger = mCollidersComponents.mIsTrigger[collider2Index];
    const bool isTrigger = isCollider1Trigger || isCollider2Trigger;

    // Create a lost contact pair
    ContactPair lostContactPair(overlappingPair.pairID, body1Entity, body2Entity, overlappingPair.collider1, overlappingPair.collider2, static_cast<uint32>(mLostContactPairs.size()),
                                true, isTrigger);
    mLostContactPairs.add(lostContactPair);
}

// Add a pair of bodies that cannot collide with each other
void CollisionDetectionSystem::addNoCollisionPair(Entity body1Entity, Entity body2Entity) {
    mNoCollisionPairs.add(OverlappingPairs::computeBodiesIndexPair(body1Entity, body2Entity));

    // If there already are OverlappingPairs involved, they should be removed; Or they will remain in collision state
    Array<uint64> toBeRemoved(mMemoryManager.getPoolAllocator());
    const Array<Entity>& colliderEntities = mWorld->mCollisionBodyComponents.getColliders(body1Entity);
    for (uint32 i = 0; i < colliderEntities.size(); ++i) {

        // Get the currently overlapping pairs for colliders of body1
        const Array<uint64>& overlappingPairs = mCollidersComponents.getOverlappingPairs(colliderEntities[i]);

        for (uint32 j = 0; j < overlappingPairs.size(); ++j) {

            OverlappingPairs::OverlappingPair* pair = mOverlappingPairs.getOverlappingPair(overlappingPairs[j]);
            assert(pair != nullptr);

            const Entity overlappingBody1 = mOverlappingPairs.mColliderComponents.getBody(pair->collider1);
            const Entity overlappingBody2 = mOverlappingPairs.mColliderComponents.getBody(pair->collider2);
            if (overlappingBody1 == body2Entity || overlappingBody2 == body2Entity) {
                toBeRemoved.add(overlappingPairs[j]);
            }
        }
    }

    // Remove the overlapping pairs that needs to be removed
    for (uint32 i = 0; i < toBeRemoved.size(); ++i) {
        mOverlappingPairs.removePair(toBeRemoved[i]);
    }
}

// Take an array of overlapping nodes in the broad-phase and create new overlapping pairs if necessary
void CollisionDetectionSystem::updateOverlappingPairs(const Array<Pair<int32, int32>>& overlappingNodes) {

    RP3D_PROFILE("CollisionDetectionSystem::updateOverlappingPairs()", mProfiler);

    // For each overlapping pair of nodes
    const uint32 nbOverlappingNodes = static_cast<uint32>(overlappingNodes.size());
    for (uint32 i=0; i < nbOverlappingNodes; i++) {

        Pair<int32, int32> nodePair = overlappingNodes[i];

        assert(nodePair.first != -1);
        assert(nodePair.second != -1);

        // Skip pairs with same overlapping nodes
        if (nodePair.first != nodePair.second) {

            // Get the two colliders
            const Entity collider1Entity = mMapBroadPhaseIdToColliderEntity[nodePair.first];
            const Entity collider2Entity = mMapBroadPhaseIdToColliderEntity[nodePair.second];

            const uint32 collider1Index = mCollidersComponents.getEntityIndex(collider1Entity);
            const uint32 collider2Index = mCollidersComponents.getEntityIndex(collider2Entity);

            // Get the two bodies
            const Entity body1Entity = mCollidersComponents.mBodiesEntities[collider1Index];
            const Entity body2Entity = mCollidersComponents.mBodiesEntities[collider2Index];

            // If the two colliders are from the same body, skip it
            if (body1Entity != body2Entity) {

                const uint32 nbEnabledColliderComponents = mCollidersComponents.getNbEnabledComponents();
                const bool isBody1Enabled = collider1Index < nbEnabledColliderComponents;
                const bool isBody2Enabled = collider2Index < nbEnabledColliderComponents;
                bool isBody1Static = false;
                bool isBody2Static = false;
                uint32 rigidBody1Index, rigidBody2Index;
                if (mRigidBodyComponents.hasComponentGetIndex(body1Entity, rigidBody1Index)) {
                    isBody1Static = mRigidBodyComponents.mBodyTypes[rigidBody1Index] == BodyType::STATIC;
                }
                if (mRigidBodyComponents.hasComponentGetIndex(body2Entity, rigidBody2Index)) {
                    isBody2Static = mRigidBodyComponents.mBodyTypes[rigidBody2Index] == BodyType::STATIC;
                }

                const bool isBody1Active = isBody1Enabled && !isBody1Static;
                const bool isBody2Active = isBody2Enabled && !isBody2Static;

                if (isBody1Active || isBody2Active) {

                    // Check if the bodies are in the set of bodies that cannot collide between each other
                    const bodypair bodiesIndex = OverlappingPairs::computeBodiesIndexPair(body1Entity, body2Entity);
                    if (!mNoCollisionPairs.contains(bodiesIndex)) {

                        // Compute the overlapping pair ID
                        const uint64 pairId = pairNumbers(std::max(nodePair.first, nodePair.second), std::min(nodePair.first, nodePair.second));

                        // Check if the overlapping pair already exists
                        OverlappingPairs::OverlappingPair* overlappingPair = mOverlappingPairs.getOverlappingPair(pairId);
                        if (overlappingPair == nullptr) {

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
                                const bool isShape1Convex = shape1->getCollisionShape()->isConvex();
                                const bool isShape2Convex = shape2->getCollisionShape()->isConvex();
                                if (isShape1Convex || isShape2Convex) {

                                    // Add the new overlapping pair
                                    mOverlappingPairs.addPair(collider1Index, collider2Index, isShape1Convex && isShape2Convex);
                                }
                            }
                        }
                        else {

                            // We do not need to test the pair for overlap because it has just been reported that they still overlap
                            overlappingPair->needToTestOverlap = false;
                        }
                    }
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
    const uint64 nbConvexVsConvexPairs = mOverlappingPairs.mConvexPairs.size();
    for (uint64 i=0; i < nbConvexVsConvexPairs; i++) {

        OverlappingPairs::ConvexOverlappingPair& overlappingPair = mOverlappingPairs.mConvexPairs[i];

        assert(mCollidersComponents.getBroadPhaseId(overlappingPair.collider1) != -1);
        assert(mCollidersComponents.getBroadPhaseId(overlappingPair.collider2) != -1);
        assert(mCollidersComponents.getBroadPhaseId(overlappingPair.collider1) != mCollidersComponents.getBroadPhaseId(overlappingPair.collider2));


        const Entity collider1Entity = overlappingPair.collider1;
        const Entity collider2Entity = overlappingPair.collider2;

        const uint32 collider1Index = mCollidersComponents.getEntityIndex(collider1Entity);
        const uint32 collider2Index = mCollidersComponents.getEntityIndex(collider2Entity);

        CollisionShape* collisionShape1 = mCollidersComponents.mCollisionShapes[collider1Index];
        CollisionShape* collisionShape2 = mCollidersComponents.mCollisionShapes[collider2Index];

        NarrowPhaseAlgorithmType algorithmType = overlappingPair.narrowPhaseAlgorithmType;

        const bool isCollider1Trigger = mCollidersComponents.mIsTrigger[collider1Index];
        const bool isCollider2Trigger = mCollidersComponents.mIsTrigger[collider2Index];
        const bool reportContacts = needToReportContacts && !isCollider1Trigger && !isCollider2Trigger;

        // No middle-phase is necessary, simply create a narrow phase info
        // for the narrow-phase collision detection
        narrowPhaseInput.addNarrowPhaseTest(overlappingPair.pairID, collider1Entity, collider2Entity, collisionShape1, collisionShape2,
                                            mCollidersComponents.mLocalToWorldTransforms[collider1Index],
                                            mCollidersComponents.mLocalToWorldTransforms[collider2Index],
                                            algorithmType, reportContacts, &overlappingPair.lastFrameCollisionInfo,
                                            mMemoryManager.getSingleFrameAllocator());

        overlappingPair.collidingInCurrentFrame = false;
    }

    // For each possible convex vs concave pair of bodies
    const uint64 nbConcavePairs = mOverlappingPairs.mConcavePairs.size();
    for (uint64 i=0; i < nbConcavePairs; i++) {

        OverlappingPairs::ConcaveOverlappingPair& overlappingPair = mOverlappingPairs.mConcavePairs[i];

        assert(mCollidersComponents.getBroadPhaseId(overlappingPair.collider1) != -1);
        assert(mCollidersComponents.getBroadPhaseId(overlappingPair.collider2) != -1);
        assert(mCollidersComponents.getBroadPhaseId(overlappingPair.collider1) != mCollidersComponents.getBroadPhaseId(overlappingPair.collider2));

        computeConvexVsConcaveMiddlePhase(overlappingPair, mMemoryManager.getSingleFrameAllocator(), narrowPhaseInput, needToReportContacts);

        overlappingPair.collidingInCurrentFrame = false;
    }
}

// Compute the middle-phase collision detection
void CollisionDetectionSystem::computeMiddlePhaseCollisionSnapshot(Array<uint64>& convexPairs, Array<uint64>& concavePairs,
                                                                   NarrowPhaseInput& narrowPhaseInput, bool reportContacts) {

    RP3D_PROFILE("CollisionDetectionSystem::computeMiddlePhase()", mProfiler);

    // Reserve memory for the narrow-phase input using cached capacity from previous frame
    narrowPhaseInput.reserveMemory();

    // Remove the obsolete last frame collision infos and mark all the others as obsolete
    mOverlappingPairs.clearObsoleteLastFrameCollisionInfos();

    // For each possible convex vs convex pair of bodies
    const uint64 nbConvexPairs = convexPairs.size();
    for (uint64 p=0; p < nbConvexPairs; p++) {

        const uint64 pairId = convexPairs[p];

        const uint64 pairIndex = mOverlappingPairs.mMapConvexPairIdToPairIndex[pairId];
        assert(pairIndex < mOverlappingPairs.mConvexPairs.size());

        const Entity collider1Entity = mOverlappingPairs.mConvexPairs[pairIndex].collider1;
        const Entity collider2Entity = mOverlappingPairs.mConvexPairs[pairIndex].collider2;

        const uint32 collider1Index = mCollidersComponents.getEntityIndex(collider1Entity);
        const uint32 collider2Index = mCollidersComponents.getEntityIndex(collider2Entity);

        assert(mCollidersComponents.getBroadPhaseId(collider1Entity) != -1);
        assert(mCollidersComponents.getBroadPhaseId(collider2Entity) != -1);
        assert(mCollidersComponents.getBroadPhaseId(collider1Entity) != mCollidersComponents.getBroadPhaseId(collider2Entity));

        CollisionShape* collisionShape1 = mCollidersComponents.mCollisionShapes[collider1Index];
        CollisionShape* collisionShape2 = mCollidersComponents.mCollisionShapes[collider2Index];

        NarrowPhaseAlgorithmType algorithmType = mOverlappingPairs.mConvexPairs[pairIndex].narrowPhaseAlgorithmType;

        // No middle-phase is necessary, simply create a narrow phase info
        // for the narrow-phase collision detection
        narrowPhaseInput.addNarrowPhaseTest(pairId, collider1Entity, collider2Entity, collisionShape1, collisionShape2,
                                                  mCollidersComponents.mLocalToWorldTransforms[collider1Index],
                                                  mCollidersComponents.mLocalToWorldTransforms[collider2Index],
                                                  algorithmType, reportContacts, &mOverlappingPairs.mConvexPairs[pairIndex].lastFrameCollisionInfo, mMemoryManager.getSingleFrameAllocator());

    }

    // For each possible convex vs concave pair of bodies
    const uint32 nbConcavePairs = static_cast<uint32>(concavePairs.size());
    for (uint32 p=0; p < nbConcavePairs; p++) {

        const uint64 pairId = concavePairs[p];
        const uint64 pairIndex = mOverlappingPairs.mMapConcavePairIdToPairIndex[pairId];

        assert(mCollidersComponents.getBroadPhaseId(mOverlappingPairs.mConcavePairs[pairIndex].collider1) != -1);
        assert(mCollidersComponents.getBroadPhaseId(mOverlappingPairs.mConcavePairs[pairIndex].collider2) != -1);
        assert(mCollidersComponents.getBroadPhaseId(mOverlappingPairs.mConcavePairs[pairIndex].collider1) != mCollidersComponents.getBroadPhaseId(mOverlappingPairs.mConcavePairs[pairIndex].collider2));

        computeConvexVsConcaveMiddlePhase(mOverlappingPairs.mConcavePairs[pairIndex], mMemoryManager.getSingleFrameAllocator(), narrowPhaseInput, reportContacts);
    }
}

// Compute the concave vs convex middle-phase algorithm for a given pair of bodies
void CollisionDetectionSystem::computeConvexVsConcaveMiddlePhase(OverlappingPairs::ConcaveOverlappingPair& overlappingPair, MemoryAllocator& allocator, NarrowPhaseInput& narrowPhaseInput, bool reportContacts) {

    RP3D_PROFILE("CollisionDetectionSystem::computeConvexVsConcaveMiddlePhase()", mProfiler);

    const Entity collider1 = overlappingPair.collider1;
    const Entity collider2 = overlappingPair.collider2;

    const uint32 collider1Index = mCollidersComponents.getEntityIndex(collider1);
    const uint32 collider2Index = mCollidersComponents.getEntityIndex(collider2);

    Transform& shape1LocalToWorldTransform = mCollidersComponents.mLocalToWorldTransforms[collider1Index];
    Transform& shape2LocalToWorldTransform = mCollidersComponents.mLocalToWorldTransforms[collider2Index];

    Transform convexToConcaveTransform;

    // Collision shape 1 is convex, collision shape 2 is concave
    ConvexShape* convexShape;
    ConcaveShape* concaveShape;
    if (overlappingPair.isShape1Convex) {
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
    assert(overlappingPair.narrowPhaseAlgorithmType != NarrowPhaseAlgorithmType::None);

    // Compute the convex shape AABB in the local-space of the concave shape
    AABB aabb;
    convexShape->computeAABB(aabb, convexToConcaveTransform);

    // Compute the concave shape triangles that are overlapping with the convex mesh AABB
    Array<Vector3> triangleVertices(allocator, 64);
    Array<Vector3> triangleVerticesNormals(allocator, 64);
    Array<uint> shapeIds(allocator, 64);
    concaveShape->computeOverlappingTriangles(aabb, triangleVertices, triangleVerticesNormals, shapeIds, allocator);

    assert(triangleVertices.size() == triangleVerticesNormals.size());
    assert(shapeIds.size() == triangleVertices.size() / 3);
    assert(triangleVertices.size() % 3 == 0);
    assert(triangleVerticesNormals.size() % 3 == 0);

    const bool isCollider1Trigger = mCollidersComponents.mIsTrigger[collider1Index];
    const bool isCollider2Trigger = mCollidersComponents.mIsTrigger[collider2Index];
    reportContacts = reportContacts && !isCollider1Trigger && !isCollider2Trigger;

    CollisionShape* shape1;
    CollisionShape* shape2;

    if (overlappingPair.isShape1Convex) {
        shape1 = convexShape;
    }
    else {
        shape2 = convexShape;
    }

    // For each overlapping triangle
    const uint32 nbShapeIds = static_cast<uint32>(shapeIds.size());
    for (uint32 i=0; i < nbShapeIds; i++) {

        // Create a triangle collision shape (the allocated memory for the TriangleShape will be released in the
        // destructor of the corresponding NarrowPhaseInfo.
        TriangleShape* triangleShape = new (allocator.allocate(sizeof(TriangleShape)))
                                       TriangleShape(&(triangleVertices[i * 3]), &(triangleVerticesNormals[i * 3]), shapeIds[i], mTriangleHalfEdgeStructure, allocator);

    #ifdef IS_RP3D_PROFILING_ENABLED


        // Set the profiler to the triangle shape
        triangleShape->setProfiler(mProfiler);

    #endif

        if (overlappingPair.isShape1Convex) {
            shape2 = triangleShape;
        }
        else {
            shape1 = triangleShape;
        }

        // Add a collision info for the two collision shapes into the overlapping pair (if not present yet)
        LastFrameCollisionInfo* lastFrameInfo = overlappingPair.addLastFrameInfoIfNecessary(shape1->getId(), shape2->getId());

        // Create a narrow phase info for the narrow-phase collision detection
        narrowPhaseInput.addNarrowPhaseTest(overlappingPair.pairID, collider1, collider2, shape1, shape2,
                                            shape1LocalToWorldTransform, shape2LocalToWorldTransform,
                                            overlappingPair.narrowPhaseAlgorithmType, reportContacts, lastFrameInfo, allocator);
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
                                                     Array<ContactPointInfo>& potentialContactPoints,
                                                     Array<ContactManifoldInfo>& potentialContactManifolds,
                                                     Array<ContactPair>* contactPairs) {

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

    // Swap the previous and current contacts arrays
    swapPreviousAndCurrentContacts();

    mPotentialContactManifolds.reserve(mNbPreviousPotentialContactManifolds);
    mPotentialContactPoints.reserve(mNbPreviousPotentialContactPoints);

    // Test the narrow-phase collision detection on the batches to be tested
    testNarrowPhaseCollision(mNarrowPhaseInput, true, allocator);

    // Process all the potential contacts after narrow-phase collision
    processAllPotentialContacts(mNarrowPhaseInput, true, mPotentialContactPoints,
                                mPotentialContactManifolds, mCurrentContactPairs);

    // Reduce the number of contact points in the manifolds
    reducePotentialContactManifolds(mCurrentContactPairs, mPotentialContactManifolds, mPotentialContactPoints);

    // Add the contact pairs to the bodies
    addContactPairsToBodies();

    assert(mCurrentContactManifolds->size() == 0);
    assert(mCurrentContactPoints->size() == 0);
}

// Add the contact pairs to the corresponding bodies
void CollisionDetectionSystem::addContactPairsToBodies() {

    const uint32 nbContactPairs = static_cast<uint32>(mCurrentContactPairs->size());
    for (uint32 p=0 ; p < nbContactPairs; p++) {

        ContactPair& contactPair = (*mCurrentContactPairs)[p];

        const bool isBody1Rigid = mRigidBodyComponents.hasComponent(contactPair.body1Entity);
        const bool isBody2Rigid = mRigidBodyComponents.hasComponent(contactPair.body2Entity);

        // Add the associated contact pair to both bodies of the pair (used to create islands later)
        if (isBody1Rigid) {
           mRigidBodyComponents.addContacPair(contactPair.body1Entity, p);
        }
        if (isBody2Rigid) {
           mRigidBodyComponents.addContacPair(contactPair.body2Entity, p);
        }

        // If at least of body is a CollisionBody
        if (!isBody1Rigid || !isBody2Rigid) {

            // Add the pair index to the array of pairs with CollisionBody
            mCollisionBodyContactPairsIndices.add(p);
        }
    }
}

// Compute the map from contact pairs ids to contact pair for the next frame
void CollisionDetectionSystem::computeMapPreviousContactPairs() {

    mPreviousMapPairIdToContactPairIndex.clear();
    const uint32 nbCurrentContactPairs = static_cast<uint32>(mCurrentContactPairs->size());
    for (uint32 i=0; i < nbCurrentContactPairs; i++) {
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
        Array<ContactPair> contactPairs(allocator);
        Array<ContactPair> lostContactPairs(allocator);          // Always empty in this case (snapshot)
        computeOverlapSnapshotContactPairs(narrowPhaseInput, contactPairs);

        // Report overlapping colliders
        OverlapCallback::CallbackData callbackData(contactPairs, lostContactPairs, false, *mWorld);
        (*callback).onOverlap(callbackData);
    }

    return collisionFound;
}

// Process the potential overlapping bodies  for the testOverlap() methods
void CollisionDetectionSystem::computeOverlapSnapshotContactPairs(NarrowPhaseInput& narrowPhaseInput, Array<ContactPair>& contactPairs) const {

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
    Array<uint64>& overlappingPairs = mCollidersComponents.getOverlappingPairs(collider->getEntity());

    const uint32 nbPairs = static_cast<uint32>(overlappingPairs.size());
    for (uint32 i=0; i < nbPairs; i++) {

        // Notify that the overlapping pair needs to be testbed for overlap
        mOverlappingPairs.setNeedToTestOverlap(overlappingPairs[i], true);
    }
}

// Convert the potential overlapping bodies for the testOverlap() methods
void CollisionDetectionSystem::computeOverlapSnapshotContactPairs(NarrowPhaseInfoBatch& narrowPhaseInfoBatch, Array<ContactPair>& contactPairs,
                                                           Set<uint64>& setOverlapContactPairId) const {

    RP3D_PROFILE("CollisionDetectionSystem::computeSnapshotContactPairs()", mProfiler);

    // For each narrow phase info object
    for(uint32 i=0; i < narrowPhaseInfoBatch.getNbObjects(); i++) {

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
                ContactPair contactPair(narrowPhaseInfoBatch.narrowPhaseInfos[i].overlappingPairId, body1Entity, body2Entity, collider1Entity, collider2Entity, static_cast<uint32>(contactPairs.size()), false, isTrigger);
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

        Array<ContactPointInfo> potentialContactPoints(allocator);
        Array<ContactManifoldInfo> potentialContactManifolds(allocator);
        Array<ContactPair> contactPairs(allocator);
        Array<ContactPair> lostContactPairs(allocator);                  // Not used during collision snapshots
        Array<ContactManifold> contactManifolds(allocator);
        Array<ContactPoint> contactPoints(allocator);

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

// Swap the previous and current contacts arrays
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

    // We go through all the contact pairs and add the pairs with a least a CollisionBody at the end of the
    // mProcessContactPairsOrderIslands array because those pairs have not been added during the islands
    // creation (only the pairs with two RigidBodies are added during island creation)
    mWorld->mProcessContactPairsOrderIslands.addRange(mCollisionBodyContactPairsIndices);

    assert(mWorld->mProcessContactPairsOrderIslands.size() == (*mCurrentContactPairs).size());

    // Process the contact pairs in the order defined by the islands such that the contact manifolds and
    // contact points of a given island are packed together in the array of manifolds and contact points
    const uint32 nbContactPairsToProcess = static_cast<uint32>(mWorld->mProcessContactPairsOrderIslands.size());
    for (uint32 p=0; p < nbContactPairsToProcess; p++) {

        uint32 contactPairIndex = mWorld->mProcessContactPairsOrderIslands[p];

        ContactPair& contactPair = (*mCurrentContactPairs)[contactPairIndex];

        contactPair.contactManifoldsIndex = static_cast<uint32>(mCurrentContactManifolds->size());
        contactPair.nbContactManifolds = contactPair.nbPotentialContactManifolds;
        contactPair.contactPointsIndex = static_cast<uint32>(mCurrentContactPoints->size());

        // For each potential contact manifold of the pair
        for (uint32 m=0; m < contactPair.nbPotentialContactManifolds; m++) {

            ContactManifoldInfo& potentialManifold = mPotentialContactManifolds[contactPair.potentialContactManifoldsIndices[m]];

            // Start index and number of contact points for this manifold
            const uint32 contactPointsIndex = static_cast<uint32>(mCurrentContactPoints->size());
            const int8 nbContactPoints = static_cast<int8>(potentialManifold.nbPotentialContactPoints);
            contactPair.nbToTalContactPoints += nbContactPoints;

            // Create and add the contact manifold
            mCurrentContactManifolds->emplace(contactPair.body1Entity, contactPair.body2Entity, contactPair.collider1Entity,
                                              contactPair.collider2Entity, contactPointsIndex, nbContactPoints);

            assert(potentialManifold.nbPotentialContactPoints > 0);

            // For each contact point of the manifold
            for (uint32 c=0; c < potentialManifold.nbPotentialContactPoints; c++) {

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

    mNbPreviousPotentialContactManifolds = static_cast<uint32>(mPotentialContactManifolds.capacity());
    mNbPreviousPotentialContactPoints = static_cast<uint32>(mPotentialContactPoints.capacity());

    // Reset the potential contacts
    mPotentialContactPoints.clear(true);
    mPotentialContactManifolds.clear(true);

    // Compute the map from contact pairs ids to contact pair for the next frame
    computeMapPreviousContactPairs();

    mCollisionBodyContactPairsIndices.clear(true);

    mNarrowPhaseInput.clear();
}

// Compute the lost contact pairs (contact pairs in contact in the previous frame but not in the current one)
void CollisionDetectionSystem::computeLostContactPairs() {

    // For each convex pair
    const uint32 nbConvexPairs = static_cast<uint32>(mOverlappingPairs.mConvexPairs.size());
    for (uint32 i=0; i < nbConvexPairs; i++) {

        // If the two colliders of the pair were colliding in the previous frame but not in the current one
        if (mOverlappingPairs.mConvexPairs[i].collidingInPreviousFrame && !mOverlappingPairs.mConvexPairs[i].collidingInCurrentFrame) {

            // If both bodies still exist
            if (mCollidersComponents.hasComponent(mOverlappingPairs.mConvexPairs[i].collider1) && mCollidersComponents.hasComponent(mOverlappingPairs.mConvexPairs[i].collider2)) {

                // Create a lost contact pair
                addLostContactPair(mOverlappingPairs.mConvexPairs[i]);
            }
        }
    }

    // For each convex pair
    const uint32 nbConcavePairs = static_cast<uint32>(mOverlappingPairs.mConcavePairs.size());
    for (uint32 i=0; i < nbConcavePairs; i++) {

        // If the two colliders of the pair were colliding in the previous frame but not in the current one
        if (mOverlappingPairs.mConcavePairs[i].collidingInPreviousFrame && !mOverlappingPairs.mConcavePairs[i].collidingInCurrentFrame) {

            // If both bodies still exist
            if (mCollidersComponents.hasComponent(mOverlappingPairs.mConcavePairs[i].collider1) && mCollidersComponents.hasComponent(mOverlappingPairs.mConcavePairs[i].collider2)) {

                // Create a lost contact pair
                addLostContactPair(mOverlappingPairs.mConcavePairs[i]);
            }
        }
    }
}

// Create the actual contact manifolds and contacts points for testCollision() methods
void CollisionDetectionSystem::createSnapshotContacts(Array<ContactPair>& contactPairs,
                                                 Array<ContactManifold>& contactManifolds,
                                                 Array<ContactPoint>& contactPoints,
                                                 Array<ContactManifoldInfo>& potentialContactManifolds,
                                                 Array<ContactPointInfo>& potentialContactPoints) {

    RP3D_PROFILE("CollisionDetectionSystem::createSnapshotContacts()", mProfiler);

    contactManifolds.reserve(contactPairs.size());
    contactPoints.reserve(contactManifolds.size());

    // For each contact pair
    const uint32 nbContactPairs = static_cast<uint32>(contactPairs.size());
    for (uint32 p=0; p < nbContactPairs; p++) {

        ContactPair& contactPair = contactPairs[p];
        assert(contactPair.nbPotentialContactManifolds > 0);

        contactPair.contactManifoldsIndex = static_cast<uint32>(contactManifolds.size());
        contactPair.nbContactManifolds = contactPair.nbPotentialContactManifolds;
        contactPair.contactPointsIndex = static_cast<uint32>(contactPoints.size());

        // For each potential contact manifold of the pair
        for (uint32 m=0; m < contactPair.nbPotentialContactManifolds; m++) {

            ContactManifoldInfo& potentialManifold = potentialContactManifolds[contactPair.potentialContactManifoldsIndices[m]];

            // Start index and number of contact points for this manifold
            const uint32 contactPointsIndex = static_cast<uint32>(contactPoints.size());
            const uint8 nbContactPoints = potentialManifold.nbPotentialContactPoints;
            contactPair.nbToTalContactPoints += nbContactPoints;

            // Create and add the contact manifold
            contactManifolds.emplace(contactPair.body1Entity, contactPair.body2Entity, contactPair.collider1Entity,
                                     contactPair.collider2Entity, contactPointsIndex, nbContactPoints);

            assert(potentialManifold.nbPotentialContactPoints > 0);

            // For each contact point of the manifold
            for (uint32 c=0; c < potentialManifold.nbPotentialContactPoints; c++) {

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

    const decimal persistentContactDistThresholdSqr = mWorld->mConfig.persistentContactDistanceThreshold * mWorld->mConfig.persistentContactDistanceThreshold;

    // For each contact pair of the current frame
    const uint32 nbCurrentContactPairs = static_cast<uint32>(mCurrentContactPairs->size());
    for (uint32 i=0; i < nbCurrentContactPairs; i++) {

        ContactPair& currentContactPair = (*mCurrentContactPairs)[i];

        // Find the corresponding contact pair in the previous frame (if any)
        auto itPrevContactPair = mPreviousMapPairIdToContactPairIndex.find(currentContactPair.pairId);

        // If we have found a corresponding contact pair in the previous frame
        if (itPrevContactPair != mPreviousMapPairIdToContactPairIndex.end()) {

            const uint32 previousContactPairIndex = itPrevContactPair->second;
            ContactPair& previousContactPair = (*mPreviousContactPairs)[previousContactPairIndex];

            // --------------------- Contact Manifolds --------------------- //

            const uint32 contactManifoldsIndex = currentContactPair.contactManifoldsIndex;
            const uint32 nbContactManifolds = currentContactPair.nbContactManifolds;

            // For each current contact manifold of the current contact pair
            for (uint32 m=contactManifoldsIndex; m < contactManifoldsIndex + nbContactManifolds; m++) {

                assert(m < mCurrentContactManifolds->size());
                ContactManifold& currentContactManifold = (*mCurrentContactManifolds)[m];
                assert(currentContactManifold.nbContactPoints > 0);
                ContactPoint& currentContactPoint = (*mCurrentContactPoints)[currentContactManifold.contactPointsIndex];
                const Vector3& currentContactPointNormal = currentContactPoint.getNormal();

                // Find a similar contact manifold among the contact manifolds from the previous frame (for warmstarting)
                const uint32 previousContactManifoldIndex = previousContactPair.contactManifoldsIndex;
                const uint32 previousNbContactManifolds = previousContactPair.nbContactManifolds;
                for (uint32 p=previousContactManifoldIndex; p < previousContactManifoldIndex + previousNbContactManifolds; p++) {

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

                        break;
                    }
                }
            }

            // --------------------- Contact Points --------------------- //

            const uint32 contactPointsIndex = currentContactPair.contactPointsIndex;
            const uint32 nbTotalContactPoints = currentContactPair.nbToTalContactPoints;

            // For each current contact point of the current contact pair
            for (uint32 c=contactPointsIndex; c < contactPointsIndex + nbTotalContactPoints; c++) {

                assert(c < mCurrentContactPoints->size());
                ContactPoint& currentContactPoint = (*mCurrentContactPoints)[c];

                const Vector3& currentContactPointLocalShape1 = currentContactPoint.getLocalPointOnShape1();

                // Find a similar contact point among the contact points from the previous frame (for warmstarting)
                const uint32 previousContactPointsIndex = previousContactPair.contactPointsIndex;
                const uint32 previousNbContactPoints = previousContactPair.nbToTalContactPoints;
                for (uint32 p=previousContactPointsIndex; p < previousContactPointsIndex + previousNbContactPoints; p++) {

                    const ContactPoint& previousContactPoint = (*mPreviousContactPoints)[p];

                    // If the previous contact point is very close to th current one
                    const decimal distSquare = (currentContactPointLocalShape1 - previousContactPoint.getLocalPointOnShape1()).lengthSquare();
                    if (distSquare <= persistentContactDistThresholdSqr) {

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
    Array<uint64>& overlappingPairs = mCollidersComponents.getOverlappingPairs(collider->getEntity());
    while(overlappingPairs.size() > 0) {

        // TODO : Remove all the contact manifold of the overlapping pair from the contact manifolds array of the two bodies involved

        // Remove the overlapping pair
        mOverlappingPairs.removePair(overlappingPairs[0]);
    }

    mMapBroadPhaseIdToColliderEntity.remove(colliderBroadPhaseId);

    // Remove the body from the broad-phase
    mBroadPhaseSystem.removeCollider(collider);
}

// Ray casting method
void CollisionDetectionSystem::raycast(RaycastCallback* raycastCallback, const Ray& ray, unsigned short raycastWithCategoryMaskBits) const {

    RP3D_PROFILE("CollisionDetectionSystem::raycast()", mProfiler);

    RaycastTest rayCastTest(raycastCallback);

    // Ask the broad-phase algorithm to call the testRaycastAgainstShape()
    // callback method for each collider hit by the ray in the broad-phase
    mBroadPhaseSystem.raycast(ray, rayCastTest, raycastWithCategoryMaskBits);
}

// Convert the potential contact into actual contacts
void CollisionDetectionSystem::processPotentialContacts(NarrowPhaseInfoBatch& narrowPhaseInfoBatch, bool updateLastFrameInfo,
                                                        Array<ContactPointInfo>& potentialContactPoints,
                                                        Array<ContactManifoldInfo>& potentialContactManifolds,
                                                        Map<uint64, uint>& mapPairIdToContactPairIndex,
                                                        Array<ContactPair>* contactPairs) {

    RP3D_PROFILE("CollisionDetectionSystem::processPotentialContacts()", mProfiler);

    const uint32 nbObjects = narrowPhaseInfoBatch.getNbObjects();

    if (updateLastFrameInfo) {

        // For each narrow phase info object
        for(uint32 i=0; i < nbObjects; i++) {

            narrowPhaseInfoBatch.narrowPhaseInfos[i].lastFrameCollisionInfo->wasColliding = narrowPhaseInfoBatch.narrowPhaseInfos[i].isColliding;

            // The previous frame collision info is now valid
            narrowPhaseInfoBatch.narrowPhaseInfos[i].lastFrameCollisionInfo->isValid = true;
        }
    }

    // For each narrow phase info object
    for(uint32 i=0; i < nbObjects; i++) {

        // If the two colliders are colliding
        if (narrowPhaseInfoBatch.narrowPhaseInfos[i].isColliding) {

            const uint64 pairId = narrowPhaseInfoBatch.narrowPhaseInfos[i].overlappingPairId;
            OverlappingPairs::OverlappingPair* overlappingPair = mOverlappingPairs.getOverlappingPair(pairId);
            assert(overlappingPair != nullptr);

            overlappingPair->collidingInCurrentFrame = true;

            const Entity collider1Entity = narrowPhaseInfoBatch.narrowPhaseInfos[i].colliderEntity1;
            const Entity collider2Entity = narrowPhaseInfoBatch.narrowPhaseInfos[i].colliderEntity2;

            const uint32 collider1Index = mCollidersComponents.getEntityIndex(collider1Entity);
            const uint32 collider2Index = mCollidersComponents.getEntityIndex(collider2Entity);

            const Entity body1Entity = mCollidersComponents.mBodiesEntities[collider1Index];
            const Entity body2Entity = mCollidersComponents.mBodiesEntities[collider2Index];

            // If we have a convex vs convex collision (if we consider the base collision shapes of the colliders)
            if (mCollidersComponents.mCollisionShapes[collider1Index]->isConvex() && mCollidersComponents.mCollisionShapes[collider2Index]->isConvex()) {

                // Create a new ContactPair

                const bool isTrigger = mCollidersComponents.mIsTrigger[collider1Index] || mCollidersComponents.mIsTrigger[collider2Index];

                assert(!mWorld->mCollisionBodyComponents.getIsEntityDisabled(body1Entity) || !mWorld->mCollisionBodyComponents.getIsEntityDisabled(body2Entity));

                const uint32 newContactPairIndex = static_cast<uint32>(contactPairs->size());

                contactPairs->emplace(pairId, body1Entity, body2Entity, collider1Entity, collider2Entity,
                                      newContactPairIndex, overlappingPair->collidingInPreviousFrame, isTrigger);

                ContactPair* pairContact = &((*contactPairs)[newContactPairIndex]);

                // Create a new potential contact manifold for the overlapping pair
                uint32 contactManifoldIndex = static_cast<uint>(potentialContactManifolds.size());
                potentialContactManifolds.emplace(pairId);
                ContactManifoldInfo& contactManifoldInfo = potentialContactManifolds[contactManifoldIndex];

                const uint32 contactPointIndexStart = static_cast<uint>(potentialContactPoints.size());

                // Add the potential contacts
                for (uint32 j=0; j < narrowPhaseInfoBatch.narrowPhaseInfos[i].nbContactPoints; j++) {

                    if (contactManifoldInfo.nbPotentialContactPoints < NB_MAX_CONTACT_POINTS_IN_POTENTIAL_MANIFOLD) {

                        // Add the contact point to the manifold
                        contactManifoldInfo.potentialContactPointsIndices[contactManifoldInfo.nbPotentialContactPoints] = contactPointIndexStart + j;
                        contactManifoldInfo.nbPotentialContactPoints++;

                        // Add the contact point to the array of potential contact points
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

                    const uint32 newContactPairIndex = static_cast<uint32>(contactPairs->size());
                    contactPairs->emplace(pairId, body1Entity, body2Entity, collider1Entity, collider2Entity,
                                                       newContactPairIndex, overlappingPair->collidingInPreviousFrame , isTrigger);
                    pairContact = &((*contactPairs)[newContactPairIndex]);
                    mapPairIdToContactPairIndex.add(Pair<uint64, uint>(pairId, newContactPairIndex));

                }
                else { // If a ContactPair already exists for this overlapping pair, we use this one

                    assert(it->first == pairId);

                    const uint32 pairContactIndex = it->second;
                    pairContact = &((*contactPairs)[pairContactIndex]);
                }

                assert(pairContact != nullptr);

                // Add the potential contacts
                for (uint32 j=0; j < narrowPhaseInfoBatch.narrowPhaseInfos[i].nbContactPoints; j++) {

                    const ContactPointInfo& contactPoint = narrowPhaseInfoBatch.narrowPhaseInfos[i].contactPoints[j];

                    // Add the contact point to the array of potential contact points
                    const uint32 contactPointIndex = static_cast<uint32>(potentialContactPoints.size());

                    potentialContactPoints.add(contactPoint);

                    bool similarManifoldFound = false;

                    // For each contact manifold of the overlapping pair
                    for (uint32 m=0; m < pairContact->nbPotentialContactManifolds; m++) {

                       uint32 contactManifoldIndex = pairContact->potentialContactManifoldsIndices[m];

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
                        uint32 contactManifoldIndex = static_cast<uint32>(potentialContactManifolds.size());
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
void CollisionDetectionSystem::reducePotentialContactManifolds(Array<ContactPair>* contactPairs,
                                                         Array<ContactManifoldInfo>& potentialContactManifolds,
                                                         const Array<ContactPointInfo>& potentialContactPoints) const {

    RP3D_PROFILE("CollisionDetectionSystem::reducePotentialContactManifolds()", mProfiler);

    // Reduce the number of potential contact manifolds in a contact pair
    const uint32 nbContactPairs = static_cast<uint32>(contactPairs->size());
    for (uint32 i=0; i < nbContactPairs; i++) {

        ContactPair& contactPair = (*contactPairs)[i];

        // While there are too many manifolds in the contact pair
        while(contactPair.nbPotentialContactManifolds > NB_MAX_CONTACT_MANIFOLDS) {

            // Look for a manifold with the smallest contact penetration depth.
            decimal minDepth = DECIMAL_LARGEST;
            int minDepthManifoldIndex = -1;
            for (uint32 j=0; j < contactPair.nbPotentialContactManifolds; j++) {

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
    for (uint32 i=0; i < nbContactPairs; i++) {

        const ContactPair& pairContact = (*contactPairs)[i];

        // For each potential contact manifold
        for (uint32 j=0; j < pairContact.nbPotentialContactManifolds; j++) {

            ContactManifoldInfo& manifold = potentialContactManifolds[pairContact.potentialContactManifoldsIndices[j]];

            // If there are two many contact points in the manifold
            if (manifold.nbPotentialContactPoints > MAX_CONTACT_POINTS_IN_MANIFOLD) {

                Transform shape1LocalToWorldTransoform = mCollidersComponents.getLocalToWorldTransform(pairContact.collider1Entity);

                // Reduce the number of contact points in the manifold
                reduceContactPoints(manifold, shape1LocalToWorldTransoform, potentialContactPoints);
            }

            assert(manifold.nbPotentialContactPoints <= MAX_CONTACT_POINTS_IN_MANIFOLD);

            // Remove the duplicated contact points in the manifold (if any)
            removeDuplicatedContactPointsInManifold(manifold, potentialContactPoints);
        }
    }
}

// Return the largest depth of all the contact points of a potential manifold
decimal CollisionDetectionSystem::computePotentialManifoldLargestContactDepth(const ContactManifoldInfo& manifold,
                                                                        const Array<ContactPointInfo>& potentialContactPoints) const {

    decimal largestDepth = 0.0f;

    assert(manifold.nbPotentialContactPoints > 0);

    for (uint32 i=0; i < manifold.nbPotentialContactPoints; i++) {
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
                                             const Array<ContactPointInfo>& potentialContactPoints) const {

    assert(manifold.nbPotentialContactPoints > MAX_CONTACT_POINTS_IN_MANIFOLD);

    // The following algorithm only works to reduce to a maximum of 4 contact points
    assert(MAX_CONTACT_POINTS_IN_MANIFOLD == 4);

    // Array of the candidate contact points indices in the manifold. Every time that we have found a
    // point we want to keep, we will remove it from this array
    uint candidatePointsIndices[NB_MAX_CONTACT_POINTS_IN_POTENTIAL_MANIFOLD];
    uint8 nbCandidatePoints = manifold.nbPotentialContactPoints;
    for (uint8 i=0 ; i < manifold.nbPotentialContactPoints; i++) {
        candidatePointsIndices[i] = manifold.potentialContactPointsIndices[i];
    }

    int8 nbReducedPoints = 0;

    uint32 pointsToKeepIndices[MAX_CONTACT_POINTS_IN_MANIFOLD];
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
    uint32 elementIndexToKeep = 0;
    for (uint32 i=0; i < nbCandidatePoints; i++) {

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
    for (uint32 i=0; i < nbCandidatePoints; i++) {

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
    uint32 thirdPointMaxAreaIndex = 0;
    uint32 thirdPointMinAreaIndex = 0;
    decimal minArea = decimal(0.0);
    decimal maxArea = decimal(0.0);
    bool isPreviousAreaPositive = true;
    for (uint32 i=0; i < nbCandidatePoints; i++) {

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
    for (uint32 i=0; i < nbCandidatePoints; i++) {

        const ContactPointInfo& element = potentialContactPoints[candidatePointsIndices[i]];

        assert(candidatePointsIndices[i] != pointsToKeepIndices[0]);
        assert(candidatePointsIndices[i] != pointsToKeepIndices[1]);
        assert(candidatePointsIndices[i] != pointsToKeepIndices[2]);

        // For each edge of the triangle made by the first three points
        for (uint32 j=0; j<3; j++) {

            uint32 edgeVertex1Index = j;
            uint32 edgeVertex2Index = j < 2 ? j + 1 : 0;

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

// Remove the duplicated contact points in a given contact manifold
void CollisionDetectionSystem::removeDuplicatedContactPointsInManifold(ContactManifoldInfo& manifold,
                                                                       const Array<ContactPointInfo>& potentialContactPoints) const {

    RP3D_PROFILE("CollisionDetectionSystem::removeDuplicatedContactPointsInManifold()", mProfiler);

    const decimal distThresholdSqr = SAME_CONTACT_POINT_DISTANCE_THRESHOLD * SAME_CONTACT_POINT_DISTANCE_THRESHOLD;

    // For each contact point of the manifold
    for (uint32 i=0; i < manifold.nbPotentialContactPoints; i++) {
        for (uint32 j=i+1; j < manifold.nbPotentialContactPoints; j++) {

            const ContactPointInfo& point1 = potentialContactPoints[manifold.potentialContactPointsIndices[i]];
            const ContactPointInfo& point2 = potentialContactPoints[manifold.potentialContactPointsIndices[j]];

            // Compute the distance between the two contact points
            const decimal distSqr = (point2.localPoint1 - point1.localPoint1).lengthSquare();

            // We have a found a duplicated contact point
            if (distSqr < distThresholdSqr) {

                // Remove the duplicated contact point
                manifold.potentialContactPointsIndices[j] = manifold.potentialContactPointsIndices[manifold.nbPotentialContactPoints-1];
                manifold.nbPotentialContactPoints--;

                j--;
            }
        }
    }
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
void CollisionDetectionSystem::reportContacts(CollisionCallback& callback, Array<ContactPair>* contactPairs,
                                              Array<ContactManifold>* manifolds, Array<ContactPoint>* contactPoints, Array<ContactPair>& lostContactPairs) {

    RP3D_PROFILE("CollisionDetectionSystem::reportContacts()", mProfiler);

    // If there are contacts
    if (contactPairs->size() + lostContactPairs.size() > 0) {

        CollisionCallback::CallbackData callbackData(contactPairs, manifolds, contactPoints, lostContactPairs, *mWorld);

        // Call the callback method to report the contacts
        callback.onContact(callbackData);
    }
}

// Report all triggers to the user
void CollisionDetectionSystem::reportTriggers(EventListener& eventListener, Array<ContactPair>* contactPairs, Array<ContactPair>& lostContactPairs) {

    RP3D_PROFILE("CollisionDetectionSystem::reportTriggers()", mProfiler);

    // If there are contacts
    if (contactPairs->size() + lostContactPairs.size() > 0) {

        OverlapCallback::CallbackData callbackData(*contactPairs, lostContactPairs, true, *mWorld);

        // Call the callback method to report the overlapping shapes
        eventListener.onTrigger(callbackData);
    }
}

// Report all contacts for debug rendering
void CollisionDetectionSystem::reportDebugRenderingContacts(Array<ContactPair>* contactPairs, Array<ContactManifold>* manifolds, Array<ContactPoint>* contactPoints, Array<ContactPair>& lostContactPairs) {

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
    Array<uint64> convexPairs(mMemoryManager.getPoolAllocator());
    Array<uint64> concavePairs(mMemoryManager.getPoolAllocator());
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
    Array<uint64> convexPairs(mMemoryManager.getPoolAllocator());
    Array<uint64> concavePairs(mMemoryManager.getPoolAllocator());
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
    Array<uint64> convexPairs(mMemoryManager.getPoolAllocator());
    Array<uint64> concavePairs(mMemoryManager.getPoolAllocator());
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
    Array<uint64> convexPairs(mMemoryManager.getPoolAllocator());
    Array<uint64> concavePairs(mMemoryManager.getPoolAllocator());
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
void CollisionDetectionSystem::filterOverlappingPairs(Entity bodyEntity, Array<uint64>& convexPairs, Array<uint64>& concavePairs) const {

    // For each convex pairs
    const uint32 nbConvexPairs = static_cast<uint32>(mOverlappingPairs.mConvexPairs.size());
    for (uint32 i=0; i < nbConvexPairs; i++) {

        if (mCollidersComponents.getBody(mOverlappingPairs.mConvexPairs[i].collider1) == bodyEntity ||
            mCollidersComponents.getBody(mOverlappingPairs.mConvexPairs[i].collider2) == bodyEntity) {

            convexPairs.add(mOverlappingPairs.mConvexPairs[i].pairID);
        }
    }

    // For each concave pairs
    const uint32 nbConcavePairs = static_cast<uint32>(mOverlappingPairs.mConcavePairs.size());
    for (uint32 i=0; i < nbConcavePairs; i++) {

        if (mCollidersComponents.getBody(mOverlappingPairs.mConcavePairs[i].collider1) == bodyEntity ||
            mCollidersComponents.getBody(mOverlappingPairs.mConcavePairs[i].collider2) == bodyEntity) {

            concavePairs.add(mOverlappingPairs.mConcavePairs[i].pairID);
        }
    }
}

// Filter the overlapping pairs to keep only the pairs where two given bodies are involved
void CollisionDetectionSystem::filterOverlappingPairs(Entity body1Entity, Entity body2Entity, Array<uint64>& convexPairs, Array<uint64>& concavePairs) const {

    // For each convex pair
    const uint32 nbConvexPairs = static_cast<uint32>(mOverlappingPairs.mConvexPairs.size());
    for (uint32 i=0; i < nbConvexPairs; i++) {

        const Entity collider1Body = mCollidersComponents.getBody(mOverlappingPairs.mConvexPairs[i].collider1);
        const Entity collider2Body = mCollidersComponents.getBody(mOverlappingPairs.mConvexPairs[i].collider2);

        if ((collider1Body == body1Entity && collider2Body == body2Entity) ||
            (collider1Body == body2Entity && collider2Body == body1Entity)) {

            convexPairs.add(mOverlappingPairs.mConvexPairs[i].pairID);
        }
    }

    // For each concave pair
    const uint32 nbConcavePairs = static_cast<uint32>(mOverlappingPairs.mConcavePairs.size());
    for (uint32 i=0; i < nbConcavePairs; i++) {

        const Entity collider1Body = mCollidersComponents.getBody(mOverlappingPairs.mConcavePairs[i].collider1);
        const Entity collider2Body = mCollidersComponents.getBody(mOverlappingPairs.mConcavePairs[i].collider2);

        if ((collider1Body == body1Entity && collider2Body == body2Entity) ||
            (collider1Body == body2Entity && collider2Body == body1Entity)) {

            concavePairs.add(mOverlappingPairs.mConcavePairs[i].pairID);
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
