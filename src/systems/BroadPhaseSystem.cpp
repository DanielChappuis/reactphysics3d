/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2019 Daniel Chappuis                                       *
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
#include "BroadPhaseSystem.h"
#include "systems/CollisionDetectionSystem.h"
#include "utils/Profiler.h"
#include "collision/RaycastInfo.h"
#include "memory/MemoryManager.h"
#include "engine/PhysicsWorld.h"

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor
BroadPhaseSystem::BroadPhaseSystem(CollisionDetectionSystem& collisionDetection, ColliderComponents& collidersComponents,
                                   TransformComponents& transformComponents, RigidBodyComponents &rigidBodyComponents)
                    :mDynamicAABBTree(collisionDetection.getMemoryManager().getPoolAllocator(), DYNAMIC_TREE_AABB_GAP),
                     mCollidersComponents(collidersComponents), mTransformsComponents(transformComponents),
                     mRigidBodyComponents(rigidBodyComponents), mMovedShapes(collisionDetection.getMemoryManager().getPoolAllocator()),
                     mCollisionDetection(collisionDetection) {

#ifdef IS_PROFILING_ACTIVE

	mProfiler = nullptr;

#endif

}

// Return true if the two broad-phase collision shapes are overlapping
bool BroadPhaseSystem::testOverlappingShapes(int32 shape1BroadPhaseId, int32 shape2BroadPhaseId) const {

    RP3D_PROFILE("BroadPhaseSystem::testOverlappingShapes()", mProfiler);

    assert(shape1BroadPhaseId != -1 && shape2BroadPhaseId != -1);

    // Get the two AABBs of the collision shapes
    const AABB& aabb1 = mDynamicAABBTree.getFatAABB(shape1BroadPhaseId);
    const AABB& aabb2 = mDynamicAABBTree.getFatAABB(shape2BroadPhaseId);

    // Check if the two AABBs are overlapping
    return aabb1.testCollision(aabb2);
}

// Ray casting method
void BroadPhaseSystem::raycast(const Ray& ray, RaycastTest& raycastTest,
                                         unsigned short raycastWithCategoryMaskBits) const {

    RP3D_PROFILE("BroadPhaseSystem::raycast()", mProfiler);

    BroadPhaseRaycastCallback broadPhaseRaycastCallback(mDynamicAABBTree, raycastWithCategoryMaskBits, raycastTest);

    mDynamicAABBTree.raycast(ray, broadPhaseRaycastCallback);
}

// Add a collider into the broad-phase collision detection
void BroadPhaseSystem::addCollider(Collider* collider, const AABB& aabb) {

    assert(collider->getBroadPhaseId() == -1);

    // Add the collision shape into the dynamic AABB tree and get its broad-phase ID
    int nodeId = mDynamicAABBTree.addObject(aabb, collider);

    // Set the broad-phase ID of the collider
    mCollidersComponents.setBroadPhaseId(collider->getEntity(), nodeId);

    // Add the collision shape into the array of bodies that have moved (or have been created)
    // during the last simulation step
    addMovedCollisionShape(collider->getBroadPhaseId(), collider);
}

// Remove a collider from the broad-phase collision detection
void BroadPhaseSystem::removeCollider(Collider* collider) {

    assert(collider->getBroadPhaseId() != -1);

    int broadPhaseID = collider->getBroadPhaseId();

    mCollidersComponents.setBroadPhaseId(collider->getEntity(), -1);

    // Remove the collision shape from the dynamic AABB tree
    mDynamicAABBTree.removeObject(broadPhaseID);

    // Remove the collision shape into the array of shapes that have moved (or have been created)
    // during the last simulation step
    removeMovedCollisionShape(broadPhaseID);
}

// Update the broad-phase state of a single collider
void BroadPhaseSystem::updateCollider(Entity colliderEntity, decimal timeStep) {

    assert(mCollidersComponents.mMapEntityToComponentIndex.containsKey(colliderEntity));

    // Get the index of the collider component in the array
    uint32 index = mCollidersComponents.mMapEntityToComponentIndex[colliderEntity];

    // Update the collider component
    updateCollidersComponents(index, 1, timeStep);
}

// Update the broad-phase state of all the enabled colliders
void BroadPhaseSystem::updateColliders(decimal timeStep) {

    RP3D_PROFILE("BroadPhaseSystem::updateColliders()", mProfiler);

    // Update all the enabled collider components
    if (mCollidersComponents.getNbEnabledComponents() > 0) {
        updateCollidersComponents(0, mCollidersComponents.getNbEnabledComponents(), timeStep);
    }
}

// Notify the broad-phase that a collision shape has moved and need to be updated
void BroadPhaseSystem::updateColliderInternal(int32 broadPhaseId, Collider* collider, const AABB& aabb, const Vector3& displacement) {

    assert(broadPhaseId >= 0);

    // Update the dynamic AABB tree according to the movement of the collision shape
    bool hasBeenReInserted = mDynamicAABBTree.updateObject(broadPhaseId, aabb, displacement);

    // If the collision shape has moved out of its fat AABB (and therefore has been reinserted
    // into the tree).
    if (hasBeenReInserted) {

        // Add the collision shape into the array of shapes that have moved (or have been created)
        // during the last simulation step
        addMovedCollisionShape(broadPhaseId, collider);
    }
}

// Update the broad-phase state of some colliders components
void BroadPhaseSystem::updateCollidersComponents(uint32 startIndex, uint32 nbItems, decimal timeStep) {

    RP3D_PROFILE("BroadPhaseSystem::updateCollidersComponents()", mProfiler);

    assert(nbItems > 0);
    assert(startIndex < mCollidersComponents.getNbComponents());
    assert(startIndex + nbItems <= mCollidersComponents.getNbComponents());

    // Make sure we do not update disabled components
    startIndex = std::min(startIndex, mCollidersComponents.getNbEnabledComponents());
    uint32 endIndex = std::min(startIndex + nbItems, mCollidersComponents.getNbEnabledComponents());
    nbItems = endIndex - startIndex;
    assert(nbItems >= 0);

    // For each collider component to update
    for (uint32 i = startIndex; i < startIndex + nbItems; i++) {

        // TODO : Can we remove this test
        const int32 broadPhaseId = mCollidersComponents.mBroadPhaseIds[i];
        if (broadPhaseId != -1) {

            const Entity& bodyEntity = mCollidersComponents.mBodiesEntities[i];
            const Transform& transform = mTransformsComponents.getTransform(bodyEntity);

            // If there is a dynamics component for the current entity
            Vector3 displacement(0, 0, 0);
            if (mRigidBodyComponents.hasComponent(bodyEntity)) {

                // Get the linear velocity from the dynamics component
                const Vector3& linearVelocity = mRigidBodyComponents.getLinearVelocity(bodyEntity);

                displacement = timeStep * linearVelocity;
            }

            // Recompute the world-space AABB of the collision shape
            AABB aabb;
            mCollidersComponents.mCollisionShapes[i]->computeAABB(aabb, transform * mCollidersComponents.mLocalToBodyTransforms[i]);

            // Update the broad-phase state for the collider
            updateColliderInternal(broadPhaseId, mCollidersComponents.mColliders[i], aabb, displacement);
        }
    }
}


// Add a collision shape in the array of shapes that have moved in the last simulation step
// and that need to be tested again for broad-phase overlapping.
void BroadPhaseSystem::addMovedCollisionShape(int broadPhaseID, Collider* collider) {

    assert(broadPhaseID != -1);

    // Store the broad-phase ID into the array of shapes that have moved
    mMovedShapes.add(broadPhaseID);

    // Notify that the overlapping pairs where this shape is involved need to be tested for overlap
    mCollisionDetection.notifyOverlappingPairsToTestOverlap(collider);
}

// Compute all the overlapping pairs of collision shapes
void BroadPhaseSystem::computeOverlappingPairs(MemoryManager& memoryManager, List<Pair<int32, int32>>& overlappingNodes) {

    RP3D_PROFILE("BroadPhaseSystem::computeOverlappingPairs()", mProfiler);

    // Get the list of the colliders that have moved or have been created in the last frame
    List<int> shapesToTest = mMovedShapes.toList(memoryManager.getPoolAllocator());

    // Ask the dynamic AABB tree to report all collision shapes that overlap with the shapes to test
    mDynamicAABBTree.reportAllShapesOverlappingWithShapes(shapesToTest, 0, shapesToTest.size(), overlappingNodes);

    // Reset the array of collision shapes that have move (or have been created) during the
    // last simulation step
    mMovedShapes.clear();
}

// Called when a overlapping node has been found during the call to
// DynamicAABBTree:reportAllShapesOverlappingWithAABB()
void AABBOverlapCallback::notifyOverlappingNode(int nodeId) {
    mOverlappingNodes.add(nodeId);
}

// Called for a broad-phase shape that has to be tested for raycast
decimal BroadPhaseRaycastCallback::raycastBroadPhaseShape(int32 nodeId, const Ray& ray) {

    decimal hitFraction = decimal(-1.0);

    // Get the collider from the node
    Collider* collider = static_cast<Collider*>(mDynamicAABBTree.getNodeDataPointer(nodeId));

    // Check if the raycast filtering mask allows raycast against this shape
    if ((mRaycastWithCategoryMaskBits & collider->getCollisionCategoryBits()) != 0) {

        // Ask the collision detection to perform a ray cast test against
        // the collider of this node because the ray is overlapping
        // with the shape in the broad-phase
        hitFraction = mRaycastTest.raycastAgainstShape(collider, ray);
    }

    return hitFraction;
}
