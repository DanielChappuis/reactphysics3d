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

#ifndef REACTPHYSICS3D_BROAD_PHASE_ALGORITHM_H
#define REACTPHYSICS3D_BROAD_PHASE_ALGORITHM_H

// Libraries
#include "collision/broadphase/DynamicAABBTree.h"
#include "containers/LinkedList.h"
#include "containers/Set.h"
#include "components/ProxyShapeComponents.h"
#include "components/TransformComponents.h"
#include "components/RigidBodyComponents.h"
#include <cstring>

/// Namespace ReactPhysics3D
namespace reactphysics3d {

// Declarations
class CollisionDetectionSystem;
class BroadPhaseSystem;
class CollisionBody;
class ProxyShape;
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
 * goal of the broad-phase collision detection is to compute the pairs of proxy shapes
 * that have their AABBs overlapping. Only those pairs of bodies will be tested
 * later for collision during the narrow-phase collision detection. A dynamic AABB
 * tree data structure is used for fast broad-phase collision detection.
 */
class BroadPhaseSystem {

    protected :

        // -------------------- Attributes -------------------- //

        /// Dynamic AABB tree
        DynamicAABBTree mDynamicAABBTree;

        /// Reference to the proxy-shapes components
        ProxyShapeComponents& mProxyShapesComponents;

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

#ifdef IS_PROFILING_ACTIVE

		/// Pointer to the profiler
		Profiler* mProfiler;

#endif
        // -------------------- Methods -------------------- //

        /// Notify the Dynamic AABB tree that a proxy-shape needs to be updated
        void updateProxyShapeInternal(int32 broadPhaseId, const AABB& aabb, const Vector3& displacement);

        /// Update the broad-phase state of some proxy-shapes components
        void updateProxyShapesComponents(uint32 startIndex, uint32 nbItems, decimal timeStep);

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        BroadPhaseSystem(CollisionDetectionSystem& collisionDetection, ProxyShapeComponents& proxyShapesComponents,
                         TransformComponents& transformComponents, RigidBodyComponents& rigidBodyComponents);

        /// Destructor
        ~BroadPhaseSystem() = default;

        /// Deleted copy-constructor
        BroadPhaseSystem(const BroadPhaseSystem& algorithm) = delete;

        /// Deleted assignment operator
        BroadPhaseSystem& operator=(const BroadPhaseSystem& algorithm) = delete;
        
        /// Add a proxy collision shape into the broad-phase collision detection
        void addProxyCollisionShape(ProxyShape* proxyShape, const AABB& aabb);

        /// Remove a proxy collision shape from the broad-phase collision detection
        void removeProxyCollisionShape(ProxyShape* proxyShape);

        /// Update the broad-phase state of a single proxy-shape
        void updateProxyShape(Entity proxyShapeEntity, decimal timeStep);

        /// Update the broad-phase state of all the enabled proxy-shapes
        void updateProxyShapes(decimal timeStep);

        /// Add a collision shape in the array of shapes that have moved in the last simulation step
        /// and that need to be tested again for broad-phase overlapping.
        void addMovedCollisionShape(int broadPhaseID);

        /// Remove a collision shape from the array of shapes that have moved in the last simulation
        /// step and that need to be tested again for broad-phase overlapping.
        void removeMovedCollisionShape(int broadPhaseID);

        /// Compute all the overlapping pairs of collision shapes
        void computeOverlappingPairs(MemoryManager& memoryManager, List<Pair<int, int>>& overlappingNodes);

        /// Return the proxy shape corresponding to the broad-phase node id in parameter
        ProxyShape* getProxyShapeForBroadPhaseId(int broadPhaseId) const;

        /// Return true if the two broad-phase collision shapes are overlapping
        bool testOverlappingShapes(Entity proxyShape1Entity, Entity proxyShape2Entity) const;

        /// Return the fat AABB of a given broad-phase shape
        const AABB& getFatAABB(int broadPhaseId) const;

        /// Ray casting method
        void raycast(const Ray& ray, RaycastTest& raycastTest, unsigned short raycastWithCategoryMaskBits) const;

#ifdef IS_PROFILING_ACTIVE

		/// Set the profiler
		void setProfiler(Profiler* profiler);

#endif

};

// Return the fat AABB of a given broad-phase shape
inline const AABB& BroadPhaseSystem::getFatAABB(int broadPhaseId) const  {
    return mDynamicAABBTree.getFatAABB(broadPhaseId);
}

// Add a collision shape in the array of shapes that have moved in the last simulation step
// and that need to be tested again for broad-phase overlapping.
inline void BroadPhaseSystem::addMovedCollisionShape(int broadPhaseID) {

    assert(broadPhaseID != -1);

    // Store the broad-phase ID into the array of shapes that have moved
    mMovedShapes.add(broadPhaseID);
}

// Remove a collision shape from the array of shapes that have moved in the last simulation step
// and that need to be tested again for broad-phase overlapping.
inline void BroadPhaseSystem::removeMovedCollisionShape(int broadPhaseID) {

    // Remove the broad-phase ID from the set
    mMovedShapes.remove(broadPhaseID);
}

// Return the proxy shape corresponding to the broad-phase node id in parameter
inline ProxyShape* BroadPhaseSystem::getProxyShapeForBroadPhaseId(int broadPhaseId) const {
    return static_cast<ProxyShape*>(mDynamicAABBTree.getNodeDataPointer(broadPhaseId));
}

#ifdef IS_PROFILING_ACTIVE

// Set the profiler
inline void BroadPhaseSystem::setProfiler(Profiler* profiler) {
	mProfiler = profiler;
	mDynamicAABBTree.setProfiler(profiler);
}

#endif

}

#endif

