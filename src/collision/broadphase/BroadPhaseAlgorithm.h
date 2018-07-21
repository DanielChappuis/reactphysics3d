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
#include "DynamicAABBTree.h"
#include "containers/LinkedList.h"
#include "containers/Set.h"

/// Namespace ReactPhysics3D
namespace reactphysics3d {

// Declarations
class CollisionDetection;
class BroadPhaseAlgorithm;
class CollisionBody;
class ProxyShape;
class MemoryManager;
class Profiler;

// Structure BroadPhasePair
/**
 * This structure represent a potential overlapping pair during the
 * broad-phase collision detection.
 */
struct BroadPhasePair {

    // -------------------- Attributes -------------------- //

    /// Broad-phase ID of the first collision shape
    int collisionShape1ID;

    /// Broad-phase ID of the second collision shape
    int collisionShape2ID;

    // -------------------- Methods -------------------- //

    /// Method used to compare two pairs for sorting algorithm
    static bool smallerThan(const BroadPhasePair& pair1, const BroadPhasePair& pair2);
};

// class AABBOverlapCallback
class AABBOverlapCallback : public DynamicAABBTreeOverlapCallback {

    private:

    public:

        LinkedList<int>& mOverlappingNodes;

        // Constructor
        AABBOverlapCallback(LinkedList<int>& overlappingNodes)
             : mOverlappingNodes(overlappingNodes) {

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

// Class BroadPhaseAlgorithm
/**
 * This class represents the broad-phase collision detection. The
 * goal of the broad-phase collision detection is to compute the pairs of proxy shapes
 * that have their AABBs overlapping. Only those pairs of bodies will be tested
 * later for collision during the narrow-phase collision detection. A dynamic AABB
 * tree data structure is used for fast broad-phase collision detection.
 */
class BroadPhaseAlgorithm {

    protected :

        // -------------------- Attributes -------------------- //

        /// Dynamic AABB tree
        DynamicAABBTree mDynamicAABBTree;

        /// Set with the broad-phase IDs of all collision shapes that have moved (or have been
        /// created) during the last simulation step. Those are the shapes that need to be tested
        /// for overlapping in the next simulation step.
        Set<int> mMovedShapes;

        /// Temporary array of potential overlapping pairs (with potential duplicates)
        BroadPhasePair* mPotentialPairs;

        /// Number of potential overlapping pairs
        uint mNbPotentialPairs;

        /// Number of allocated elements for the array of potential overlapping pairs
        uint mNbAllocatedPotentialPairs;

        /// Reference to the collision detection object
        CollisionDetection& mCollisionDetection;

#ifdef IS_PROFILING_ACTIVE

		/// Pointer to the profiler
		Profiler* mProfiler;

#endif

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        BroadPhaseAlgorithm(CollisionDetection& collisionDetection);

        /// Destructor
        ~BroadPhaseAlgorithm();

        /// Deleted copy-constructor
        BroadPhaseAlgorithm(const BroadPhaseAlgorithm& algorithm) = delete;

        /// Deleted assignment operator
        BroadPhaseAlgorithm& operator=(const BroadPhaseAlgorithm& algorithm) = delete;
        
        /// Add a proxy collision shape into the broad-phase collision detection
        void addProxyCollisionShape(ProxyShape* proxyShape, const AABB& aabb);

        /// Remove a proxy collision shape from the broad-phase collision detection
        void removeProxyCollisionShape(ProxyShape* proxyShape);

        /// Notify the broad-phase that a collision shape has moved and need to be updated
        void updateProxyCollisionShape(ProxyShape* proxyShape, const AABB& aabb,
                                       const Vector3& displacement, bool forceReinsert = false);

        /// Add a collision shape in the array of shapes that have moved in the last simulation step
        /// and that need to be tested again for broad-phase overlapping.
        void addMovedCollisionShape(int broadPhaseID);

        /// Remove a collision shape from the array of shapes that have moved in the last simulation
        /// step and that need to be tested again for broad-phase overlapping.
        void removeMovedCollisionShape(int broadPhaseID);

        /// Add potential overlapping pairs in the dynamic AABB tree
        void addOverlappingNodes(int broadPhaseId1, const LinkedList<int>& overlappingNodes);

        /// Report all the shapes that are overlapping with a given AABB
        void reportAllShapesOverlappingWithAABB(const AABB& aabb, LinkedList<int>& overlappingNodes) const;

        /// Compute all the overlapping pairs of collision shapes
        void computeOverlappingPairs(MemoryManager& memoryManager);

        /// Return the proxy shape corresponding to the broad-phase node id in parameter
        ProxyShape* getProxyShapeForBroadPhaseId(int broadPhaseId) const;

        /// Return true if the two broad-phase collision shapes are overlapping
        bool testOverlappingShapes(const ProxyShape* shape1, const ProxyShape* shape2) const;

        /// Return the fat AABB of a given broad-phase shape
        const AABB& getFatAABB(int broadPhaseId) const;

        /// Ray casting method
        void raycast(const Ray& ray, RaycastTest& raycastTest, unsigned short raycastWithCategoryMaskBits) const;

#ifdef IS_PROFILING_ACTIVE

		/// Set the profiler
		void setProfiler(Profiler* profiler);

#endif

};

// Method used to compare two pairs for sorting algorithm
inline bool BroadPhasePair::smallerThan(const BroadPhasePair& pair1, const BroadPhasePair& pair2) {

    if (pair1.collisionShape1ID < pair2.collisionShape1ID) return true;
    if (pair1.collisionShape1ID == pair2.collisionShape1ID) {
        return pair1.collisionShape2ID < pair2.collisionShape2ID;
    }
    return false;
}

// Return the fat AABB of a given broad-phase shape
inline const AABB& BroadPhaseAlgorithm::getFatAABB(int broadPhaseId) const  {
    return mDynamicAABBTree.getFatAABB(broadPhaseId);
}

// Add a collision shape in the array of shapes that have moved in the last simulation step
// and that need to be tested again for broad-phase overlapping.
inline void BroadPhaseAlgorithm::addMovedCollisionShape(int broadPhaseID) {

    // Store the broad-phase ID into the array of shapes that have moved
    mMovedShapes.add(broadPhaseID);
}

// Remove a collision shape from the array of shapes that have moved in the last simulation step
// and that need to be tested again for broad-phase overlapping.
inline void BroadPhaseAlgorithm::removeMovedCollisionShape(int broadPhaseID) {

    // Remove the broad-phase ID from the set
    mMovedShapes.remove(broadPhaseID);
}

// Return the proxy shape corresponding to the broad-phase node id in parameter
inline ProxyShape* BroadPhaseAlgorithm::getProxyShapeForBroadPhaseId(int broadPhaseId) const {
    return static_cast<ProxyShape*>(mDynamicAABBTree.getNodeDataPointer(broadPhaseId));
}

#ifdef IS_PROFILING_ACTIVE

// Set the profiler
inline void BroadPhaseAlgorithm::setProfiler(Profiler* profiler) {
	mProfiler = profiler;
	mDynamicAABBTree.setProfiler(profiler);
}

#endif

}

#endif

