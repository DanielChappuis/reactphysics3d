/********************************************************************************
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

#ifndef REACTPHYSICS3D_BROAD_PHASE_ALGORITHM_H
#define REACTPHYSICS3D_BROAD_PHASE_ALGORITHM_H

// Libraries
#include <reactphysics3d/collision/broadphase/DynamicAABBTree.h>
#include <reactphysics3d/containers/LinkedList.h>
#include <reactphysics3d/containers/Set.h>
#include <reactphysics3d/components/ColliderComponents.h>
#include <reactphysics3d/components/TransformComponents.h>
#include <reactphysics3d/components/RigidBodyComponents.h>
#include <cstring>

/// Namespace ReactPhysics3D
namespace reactphysics3d {

// Declarations
class CollisionDetectionSystem;
class BroadPhaseSystem;
class CollisionBody;
class Collider;
class MemoryManager;
class Profiler;

// class AABBOverlapCallback
class AABBOverlapCallback : public DynamicAABBTreeOverlapCallback {

    public:

        List<int>& mOverlappingNodes;

        // Constructor
        AABBOverlapCallback(List<int>& overlappingNodes) : mOverlappingNodes(overlappingNodes) {

        }

        // Called when a overlapping node has been found during the call to
        // DynamicAABBTree:reportAllShapesOverlappingWithAABB()
        virtual void notifyOverlappingNode(int nodeId) override;

};

// Class BroadPhaseRaycastCallback
/**
 * Callback called when the AABB of a leaf node is hit by a ray the
 * broad-phase Dynamic AABB Tree.
 */
class BroadPhaseRaycastCallback : public DynamicAABBTreeRaycastCallback {

    private :

        const DynamicAABBTree& mDynamicAABBTree;

        unsigned short mRaycastWithCategoryMaskBits;

        RaycastTest& mRaycastTest;

    public:

        // Constructor
        BroadPhaseRaycastCallback(const DynamicAABBTree& dynamicAABBTree, unsigned short raycastWithCategoryMaskBits,
                                  RaycastTest& raycastTest)
            : mDynamicAABBTree(dynamicAABBTree), mRaycastWithCategoryMaskBits(raycastWithCategoryMaskBits),
              mRaycastTest(raycastTest) {

        }

        // Destructor
        virtual ~BroadPhaseRaycastCallback() override = default;

        // Called for a broad-phase shape that has to be tested for raycast
        virtual decimal raycastBroadPhaseShape(int32 nodeId, const Ray& ray) override;

};

// Class BroadPhaseSystem
/**
 * This class represents the broad-phase collision detection. The
 * goal of the broad-phase collision detection is to compute the pairs of colliders
 * that have their AABBs overlapping. Only those pairs of bodies will be tested
 * later for collision during the narrow-phase collision detection. A dynamic AABB
 * tree data structure is used for fast broad-phase collision detection.
 */
class BroadPhaseSystem {

    protected :

        // -------------------- Attributes -------------------- //

        /// Dynamic AABB tree
        DynamicAABBTree mDynamicAABBTree;

        /// Reference to the colliders components
        ColliderComponents& mCollidersComponents;

        /// Reference to the transform components
        TransformComponents& mTransformsComponents;

        /// Reference to the rigid body components
        RigidBodyComponents& mRigidBodyComponents;

        /// Set with the broad-phase IDs of all collision shapes that have moved (or have been
        /// created) during the last simulation step. Those are the shapes that need to be tested
        /// for overlapping in the next simulation step.
        Set<int> mMovedShapes;

        /// Reference to the collision detection object
        CollisionDetectionSystem& mCollisionDetection;

#ifdef IS_RP3D_PROFILING_ENABLED

		/// Pointer to the profiler
		Profiler* mProfiler;

#endif
        // -------------------- Methods -------------------- //

        /// Notify the Dynamic AABB tree that a collider needs to be updated
        void updateColliderInternal(int32 broadPhaseId, Collider* collider, const AABB& aabb,
                                    bool forceReInsert);

        /// Update the broad-phase state of some colliders components
        void updateCollidersComponents(uint32 startIndex, uint32 nbItems, decimal timeStep);

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        BroadPhaseSystem(CollisionDetectionSystem& collisionDetection, ColliderComponents& collidersComponents,
                         TransformComponents& transformComponents, RigidBodyComponents& rigidBodyComponents);

        /// Destructor
        ~BroadPhaseSystem() = default;

        /// Deleted copy-constructor
        BroadPhaseSystem(const BroadPhaseSystem& algorithm) = delete;

        /// Deleted assignment operator
        BroadPhaseSystem& operator=(const BroadPhaseSystem& algorithm) = delete;
        
        /// Add a collider into the broad-phase collision detection
        void addCollider(Collider* collider, const AABB& aabb);

        /// Remove a collider from the broad-phase collision detection
        void removeCollider(Collider* collider);

        /// Update the broad-phase state of a single collider
        void updateCollider(Entity colliderEntity, decimal timeStep);

        /// Update the broad-phase state of all the enabled colliders
        void updateColliders(decimal timeStep);

        /// Add a collider in the array of colliders that have moved in the last simulation step
        /// and that need to be tested again for broad-phase overlapping.
        void addMovedCollider(int broadPhaseID, Collider* collider);

        /// Remove a collider from the array of colliders that have moved in the last simulation
        /// step and that need to be tested again for broad-phase overlapping.
        void removeMovedCollider(int broadPhaseID);

        /// Compute all the overlapping pairs of collision shapes
        void computeOverlappingPairs(MemoryManager& memoryManager, List<Pair<int32, int32>>& overlappingNodes);

        /// Return the collider corresponding to the broad-phase node id in parameter
        Collider* getColliderForBroadPhaseId(int broadPhaseId) const;

        /// Return true if the two broad-phase collision shapes are overlapping
        bool testOverlappingShapes(int32 shape1BroadPhaseId, int32 shape2BroadPhaseId) const;

        /// Return the fat AABB of a given broad-phase shape
        const AABB& getFatAABB(int broadPhaseId) const;

        /// Ray casting method
        void raycast(const Ray& ray, RaycastTest& raycastTest, unsigned short raycastWithCategoryMaskBits) const;

#ifdef IS_RP3D_PROFILING_ENABLED

		/// Set the profiler
		void setProfiler(Profiler* profiler);

#endif

};

// Return the fat AABB of a given broad-phase shape
inline const AABB& BroadPhaseSystem::getFatAABB(int broadPhaseId) const  {
    return mDynamicAABBTree.getFatAABB(broadPhaseId);
}

// Remove a collider from the array of colliders that have moved in the last simulation step
// and that need to be tested again for broad-phase overlapping.
inline void BroadPhaseSystem::removeMovedCollider(int broadPhaseID) {

    // Remove the broad-phase ID from the set
    mMovedShapes.remove(broadPhaseID);
}

// Return the collider corresponding to the broad-phase node id in parameter
inline Collider* BroadPhaseSystem::getColliderForBroadPhaseId(int broadPhaseId) const {
    return static_cast<Collider*>(mDynamicAABBTree.getNodeDataPointer(broadPhaseId));
}

#ifdef IS_RP3D_PROFILING_ENABLED

// Set the profiler
inline void BroadPhaseSystem::setProfiler(Profiler* profiler) {
	mProfiler = profiler;
	mDynamicAABBTree.setProfiler(profiler);
}

#endif

}

#endif

