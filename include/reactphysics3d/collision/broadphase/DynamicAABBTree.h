/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2024 Daniel Chappuis                                       *
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
#include <reactphysics3d/configuration.h>
#include <reactphysics3d/collision/shapes/AABB.h>
#include <reactphysics3d/containers/Set.h>

/// Namespace ReactPhysics3D
namespace reactphysics3d {

// Declarations
class BroadPhaseSystem;
class BroadPhaseRaycastTestCallback;
class DynamicAABBTreeOverlapCallback;
class Body;
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
    const static int32 NULL_TREE_NODE;

    // -------------------- Attributes -------------------- //

    // A node is either in the tree (has a parent) or in the free nodes array
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
            uint32 dataInt;
            void* dataPointer;
        };
    };

    /// Height of the node in the tree
    int16 height;

    /// Fat axis aligned bounding box (AABB) corresponding to the node
    AABB aabb;

    // -------------------- Methods -------------------- //

    /// Constructor
    TreeNode() : nextNodeID(NULL_TREE_NODE), height(-1) {

    }

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
 * collision detection. The following implementation is
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
        int32 mRootNodeID;

        /// ID of the first node of the array of free (allocated) nodes in the tree that we can use
        int32 mFreeNodeID;

        /// Number of allocated nodes in the tree
        int32 mNbAllocatedNodes;

        /// Number of nodes in the tree
        int32 mNbNodes;

        /// The fat AABB is the initial AABB inflated by a given percentage of its size.
        decimal mFatAABBInflatePercentage;

#ifdef IS_RP3D_PROFILING_ENABLED

		/// Pointer to the profiler
		Profiler* mProfiler;

#endif

        // -------------------- Methods -------------------- //

        /// Allocate and return a node to use in the tree
        int32 allocateNode();

        /// Release a node
        void releaseNode(int32 nodeID);

        /// Insert a leaf node in the tree
        void insertLeafNode(int32 nodeID);

        /// Remove a leaf node from the tree
        void removeLeafNode(int32 nodeID);

        /// Balance the sub-tree of a given node using left or right rotations.
        int32 balanceSubTreeAtNode(int32 nodeID);

        /// Compute the height of a given node in the tree
        int computeHeight(int32 nodeID);

        /// Internally add an object into the tree
        int32 addObjectInternal(const AABB& aabb);

        /// Initialize the tree
        void init();

#ifndef NDEBUG

        /// Check if the tree structure is valid (for debugging purpose)
        void check() const;

        /// Check if the node structure is valid (for debugging purpose)
        void checkNode(int32 nodeID) const;

#endif

    public:

        // -------------------- Methods -------------------- //

        /// Constructor
        DynamicAABBTree(MemoryAllocator& allocator, decimal fatAABBInflatePercentage = decimal(0.0));

        /// Destructor
        ~DynamicAABBTree();

        /// Add an object into the tree (where node data are two integers)
        int32 addObject(const AABB& aabb, uint32 data);

        /// Add an object into the tree (where node data is a pointer)
        int32 addObject(const AABB& aabb, void* data);

        /// Remove an object from the tree
        void removeObject(int32 nodeID);

        /// Update the dynamic tree after an object has moved.
        bool updateObject(int32 nodeID, const AABB& newAABB, bool forceReinsert = false);

        /// Return the fat AABB corresponding to a given node ID
        const AABB& getFatAABB(int32 nodeID) const;

        /// Return the pointer to the data array of a given leaf node of the tree
        int32 getNodeDataInt(int32 nodeID) const;

        /// Return the data pointer of a given leaf node of the tree
        void* getNodeDataPointer(int32 nodeID) const;

        /// Report all shapes overlapping with all the shapes in the map in parameter
        void reportAllShapesOverlappingWithShapes(const Array<int32>& nodesToTest, uint32 startIndex,
                                                  size_t endIndex, Array<Pair<int32, int32>>& outOverlappingNodes) const;

        /// Report all shapes overlapping with the AABB given in parameter.
        void reportAllShapesOverlappingWithAABB(const AABB& aabb, Array<int>& overlappingNodes) const;

        /// Ray casting method
        void raycast(const Ray& ray, DynamicAABBTreeRaycastCallback& callback) const;

        /// Compute the height of the tree
        int computeHeight();

        /// Return the root AABB of the tree
        const AABB& getRootAABB() const;

        /// Clear all the nodes and reset the tree
        void reset();

#ifdef IS_RP3D_PROFILING_ENABLED

		/// Set the profiler
		void setProfiler(Profiler* profiler);

#endif

};

// Return true if the node is a leaf of the tree
RP3D_FORCE_INLINE bool TreeNode::isLeaf() const {
    return (height == 0);
}

// Return the fat AABB corresponding to a given node ID
RP3D_FORCE_INLINE const AABB& DynamicAABBTree::getFatAABB(int32 nodeID) const {
    assert(nodeID >= 0 && nodeID < mNbAllocatedNodes);
    return mNodes[nodeID].aabb;
}

// Return the pointer to the data array of a given leaf node of the tree
RP3D_FORCE_INLINE int32 DynamicAABBTree::getNodeDataInt(int32 nodeID) const {
    assert(nodeID >= 0 && nodeID < mNbAllocatedNodes);
    assert(mNodes[nodeID].isLeaf());
    return mNodes[nodeID].dataInt;
}

// Return the pointer to the data pointer of a given leaf node of the tree
RP3D_FORCE_INLINE void* DynamicAABBTree::getNodeDataPointer(int32 nodeID) const {
    assert(nodeID >= 0 && nodeID < mNbAllocatedNodes);
    assert(mNodes[nodeID].isLeaf());
    return mNodes[nodeID].dataPointer;
}

// Return the root AABB of the tree
RP3D_FORCE_INLINE const AABB& DynamicAABBTree::getRootAABB() const {
    return getFatAABB(mRootNodeID);
}

// Add an object into the tree. This method creates a new leaf node in the tree and
// returns the ID of the corresponding node.
RP3D_FORCE_INLINE int32 DynamicAABBTree::addObject(const AABB& aabb, uint32 data) {

    int32 nodeId = addObjectInternal(aabb);

    mNodes[nodeId].dataInt = data;

    return nodeId;
}

// Add an object into the tree. This method creates a new leaf node in the tree and
// returns the ID of the corresponding node.
RP3D_FORCE_INLINE int32 DynamicAABBTree::addObject(const AABB& aabb, void* data) {

    int32 nodeId = addObjectInternal(aabb);

    mNodes[nodeId].dataPointer = data;

    return nodeId;
}

#ifdef IS_RP3D_PROFILING_ENABLED

// Set the profiler
RP3D_FORCE_INLINE void DynamicAABBTree::setProfiler(Profiler* profiler) {
	mProfiler = profiler;
}

#endif

}

#endif
