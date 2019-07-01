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

#ifndef REACTPHYSICS3D_DYNAMIC_AABB_TREE_H
#define REACTPHYSICS3D_DYNAMIC_AABB_TREE_H

// Libraries
#include "configuration.h"
#include "collision/shapes/AABB.h"

/// Namespace ReactPhysics3D
namespace reactphysics3d {

// Declarations
class BroadPhaseAlgorithm;
class BroadPhaseRaycastTestCallback;
class DynamicAABBTreeOverlapCallback;
class CollisionBody;
struct RaycastTest;
class AABB;
class Profiler;
class MemoryAllocator;


// Structure TreeNode
/**
 * This structure represents a node of the dynamic AABB tree.
 */
struct TreeNode {

    // -------------------- Constants -------------------- //

    /// Null tree node constant
    const static int NULL_TREE_NODE;

    // -------------------- Attributes -------------------- //

    // A node is either in the tree (has a parent) or in the free nodes list
    // (has a next node)
    union {

        /// Parent node ID
        int32 parentID;

        /// Next allocated node ID
        int32 nextNodeID;
    };

    // A node is either a leaf (has data) or is an internal node (has children)
    union {

        /// Left and right child of the node (children[0] = left child)
        int32 children[2];

        /// Two pieces of data stored at that node (in case the node is a leaf)
        union {
            int32 dataInt[2];
            void* dataPointer;
        };
    };

    /// Height of the node in the tree
    int16 height;

    /// Fat axis aligned bounding box (AABB) corresponding to the node
    AABB aabb;

    // -------------------- Methods -------------------- //

    /// Return true if the node is a leaf of the tree
    bool isLeaf() const;
};

// Class DynamicAABBTreeOverlapCallback
/**
 * Overlapping callback method that has to be used as parameter of the
 * reportAllShapesOverlappingWithNode() method.
 */
class DynamicAABBTreeOverlapCallback {

    public :

        // Called when a overlapping node has been found during the call to
        // DynamicAABBTree:reportAllShapesOverlappingWithAABB()
        virtual void notifyOverlappingNode(int nodeId)=0;

        // Destructor
        virtual ~DynamicAABBTreeOverlapCallback() = default;
};

// Class DynamicAABBTreeRaycastCallback
/**
 * Raycast callback in the Dynamic AABB Tree called when the AABB of a leaf
 * node is hit by the ray.
 */
class DynamicAABBTreeRaycastCallback {

    public:

        // Called when the AABB of a leaf node is hit by a ray
        virtual decimal raycastBroadPhaseShape(int32 nodeId, const Ray& ray)=0;

        virtual ~DynamicAABBTreeRaycastCallback() = default;

};

// Class DynamicAABBTree
/**
 * This class implements a dynamic AABB tree that is used for broad-phase
 * collision detection. This data structure is inspired by Nathanael Presson's
 * dynamic tree implementation in BulletPhysics. The following implementation is
 * based on the one from Erin Catto in Box2D as described in the book
 * "Introduction to Game Physics with Box2D" by Ian Parberry.
 */
class DynamicAABBTree {

    private:

        // -------------------- Attributes -------------------- //

        /// Memory allocator
        MemoryAllocator& mAllocator;

        /// Pointer to the memory location of the nodes of the tree
        TreeNode* mNodes;

        /// ID of the root node of the tree
        int mRootNodeID;

        /// ID of the first node of the list of free (allocated) nodes in the tree that we can use
        int mFreeNodeID;

        /// Number of allocated nodes in the tree
        int mNbAllocatedNodes;

        /// Number of nodes in the tree
        int mNbNodes;

        /// Extra AABB Gap used to allow the collision shape to move a little bit
        /// without triggering a large modification of the tree which can be costly
        decimal mExtraAABBGap;

#ifdef IS_PROFILING_ACTIVE

		/// Pointer to the profiler
		Profiler* mProfiler;

#endif

        // -------------------- Methods -------------------- //

        /// Allocate and return a node to use in the tree
        int allocateNode();

        /// Release a node
        void releaseNode(int nodeID);

        /// Insert a leaf node in the tree
        void insertLeafNode(int nodeID);

        /// Remove a leaf node from the tree
        void removeLeafNode(int nodeID);

        /// Balance the sub-tree of a given node using left or right rotations.
        int balanceSubTreeAtNode(int nodeID);

        /// Compute the height of a given node in the tree
        int computeHeight(int nodeID);

        /// Internally add an object into the tree
        int addObjectInternal(const AABB& aabb);

        /// Initialize the tree
        void init();

#ifndef NDEBUG

        /// Check if the tree structure is valid (for debugging purpose)
        void check() const;

        /// Check if the node structure is valid (for debugging purpose)
        void checkNode(int nodeID) const;

#endif

    public:

        // -------------------- Methods -------------------- //

        /// Constructor
        DynamicAABBTree(MemoryAllocator& allocator, decimal extraAABBGap = decimal(0.0));

        /// Destructor
        ~DynamicAABBTree();

        /// Add an object into the tree (where node data are two integers)
        int addObject(const AABB& aabb, int32 data1, int32 data2);

        /// Add an object into the tree (where node data is a pointer)
        int addObject(const AABB& aabb, void* data);

        /// Remove an object from the tree
        void removeObject(int nodeID);

        /// Update the dynamic tree after an object has moved.
        bool updateObject(int nodeID, const AABB& newAABB, const Vector3& displacement, bool forceReinsert = false);

        /// Return the fat AABB corresponding to a given node ID
        const AABB& getFatAABB(int nodeID) const;

        /// Return the pointer to the data array of a given leaf node of the tree
        int32* getNodeDataInt(int nodeID) const;

        /// Return the data pointer of a given leaf node of the tree
        void* getNodeDataPointer(int nodeID) const;

        /// Report all shapes overlapping with the AABB given in parameter.
        void reportAllShapesOverlappingWithAABB(const AABB& aabb,
                                                DynamicAABBTreeOverlapCallback& callback) const;

        /// Ray casting method
        void raycast(const Ray& ray, DynamicAABBTreeRaycastCallback& callback) const;

        /// Compute the height of the tree
        int computeHeight();

        /// Return the root AABB of the tree
        AABB getRootAABB() const;

        /// Clear all the nodes and reset the tree
        void reset();

#ifdef IS_PROFILING_ACTIVE

		/// Set the profiler
		void setProfiler(Profiler* profiler);

#endif

};

// Return true if the node is a leaf of the tree
inline bool TreeNode::isLeaf() const {
    return (height == 0);
}

// Return the fat AABB corresponding to a given node ID
inline const AABB& DynamicAABBTree::getFatAABB(int nodeID) const {
    assert(nodeID >= 0 && nodeID < mNbAllocatedNodes);
    return mNodes[nodeID].aabb;
}

// Return the pointer to the data array of a given leaf node of the tree
inline int32* DynamicAABBTree::getNodeDataInt(int nodeID) const {
    assert(nodeID >= 0 && nodeID < mNbAllocatedNodes);
    assert(mNodes[nodeID].isLeaf());
    return mNodes[nodeID].dataInt;
}

// Return the pointer to the data pointer of a given leaf node of the tree
inline void* DynamicAABBTree::getNodeDataPointer(int nodeID) const {
    assert(nodeID >= 0 && nodeID < mNbAllocatedNodes);
    assert(mNodes[nodeID].isLeaf());
    return mNodes[nodeID].dataPointer;
}

// Return the root AABB of the tree
inline AABB DynamicAABBTree::getRootAABB() const {
    return getFatAABB(mRootNodeID);
}

// Add an object into the tree. This method creates a new leaf node in the tree and
// returns the ID of the corresponding node.
inline int DynamicAABBTree::addObject(const AABB& aabb, int32 data1, int32 data2) {

    int nodeId = addObjectInternal(aabb);

    mNodes[nodeId].dataInt[0] = data1;
    mNodes[nodeId].dataInt[1] = data2;

    return nodeId;
}

// Add an object into the tree. This method creates a new leaf node in the tree and
// returns the ID of the corresponding node.
inline int DynamicAABBTree::addObject(const AABB& aabb, void* data) {

    int nodeId = addObjectInternal(aabb);

    mNodes[nodeId].dataPointer = data;

    return nodeId;
}

#ifdef IS_PROFILING_ACTIVE

// Set the profiler
inline void DynamicAABBTree::setProfiler(Profiler* profiler) {
	mProfiler = profiler;
}

#endif

}

#endif
