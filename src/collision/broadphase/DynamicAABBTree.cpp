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

// Libraries
#include <reactphysics3d/collision/broadphase/DynamicAABBTree.h>
#include <reactphysics3d/systems/BroadPhaseSystem.h>
#include <reactphysics3d/containers/Stack.h>
#include <reactphysics3d/utils/Profiler.h>

using namespace reactphysics3d;

// Initialization of static variables
const int32 TreeNode::NULL_TREE_NODE = -1;

// Constructor
DynamicAABBTree::DynamicAABBTree(MemoryAllocator& allocator, decimal fatAABBInflatePercentage)
                : mAllocator(allocator), mFatAABBInflatePercentage(fatAABBInflatePercentage) {

    init();
}

// Destructor
DynamicAABBTree::~DynamicAABBTree() {

    // Free the allocated memory for the nodes
    mAllocator.release(mNodes, static_cast<size_t>(mNbAllocatedNodes) * sizeof(TreeNode));
}

// Initialize the tree
void DynamicAABBTree::init() {

    mRootNodeID = TreeNode::NULL_TREE_NODE;
    mNbNodes = 0;
    mNbAllocatedNodes = 8;

    // Allocate memory for the nodes of the tree
    mNodes = static_cast<TreeNode*>(mAllocator.allocate(static_cast<size_t>(mNbAllocatedNodes) * sizeof(TreeNode)));
    assert(mNodes);
    std::memset(mNodes, 0, static_cast<size_t>(mNbAllocatedNodes) * sizeof(TreeNode));

    // Initialize the allocated nodes
    for (int32 i=0; i<mNbAllocatedNodes - 1; i++) {
        mNodes[i].nextNodeID = i + 1;
        mNodes[i].height = -1;
    }
    mNodes[mNbAllocatedNodes - 1].nextNodeID = TreeNode::NULL_TREE_NODE;
    mNodes[mNbAllocatedNodes - 1].height = -1;
    mFreeNodeID = 0;
}

// Clear all the nodes and reset the tree
void DynamicAABBTree::reset() {

    // Free the allocated memory for the nodes
    mAllocator.release(mNodes, static_cast<size_t>(mNbAllocatedNodes) * sizeof(TreeNode));

    // Initialize the tree
    init();
}

// Allocate and return a new node in the tree
int32 DynamicAABBTree::allocateNode() {

    // If there is no more allocated node to use
    if (mFreeNodeID == TreeNode::NULL_TREE_NODE) {

        assert(mNbNodes == mNbAllocatedNodes);

        // Allocate more nodes in the tree
        int32 oldNbAllocatedNodes = mNbAllocatedNodes;
        mNbAllocatedNodes *= 2;
        TreeNode* oldNodes = mNodes;
        mNodes = static_cast<TreeNode*>(mAllocator.allocate(static_cast<size_t>(mNbAllocatedNodes) * sizeof(TreeNode)));
        assert(mNodes);
        memcpy(mNodes, oldNodes, static_cast<size_t>(mNbNodes) * sizeof(TreeNode));
        mAllocator.release(oldNodes, static_cast<size_t>(oldNbAllocatedNodes) * sizeof(TreeNode));

        // Initialize the allocated nodes
        for (int32 i=mNbNodes; i<mNbAllocatedNodes - 1; i++) {
            mNodes[i].nextNodeID = i + 1;
            mNodes[i].height = -1;
        }
        mNodes[mNbAllocatedNodes - 1].nextNodeID = TreeNode::NULL_TREE_NODE;
        mNodes[mNbAllocatedNodes - 1].height = -1;
        mFreeNodeID = mNbNodes;
    }

    // Get the next free node
    int32 freeNodeID = mFreeNodeID;
    mFreeNodeID = mNodes[freeNodeID].nextNodeID;
    mNodes[freeNodeID].parentID = TreeNode::NULL_TREE_NODE;
    mNodes[freeNodeID].height = 0;
    mNbNodes++;

    return freeNodeID;
}

// Release a node
void DynamicAABBTree::releaseNode(int nodeID) {

    assert(mNbNodes > 0);
    assert(nodeID >= 0 && nodeID < mNbAllocatedNodes);
    assert(mNodes[nodeID].height >= 0);
    mNodes[nodeID].nextNodeID = mFreeNodeID;
    mNodes[nodeID].height = -1;
    mFreeNodeID = nodeID;
    mNbNodes--;
}

// Internally add an object into the tree
int32 DynamicAABBTree::addObjectInternal(const AABB& aabb) {

    // Get the next available node (or allocate new ones if necessary)
    int32 nodeID = allocateNode();

    // Create the fat aabb to use in the tree (inflate the aabb by a constant percentage of its size)
    const Vector3 gap(aabb.getExtent() * mFatAABBInflatePercentage * decimal(0.5f));
    mNodes[nodeID].aabb.setMin(aabb.getMin() - gap);
    mNodes[nodeID].aabb.setMax(aabb.getMax() + gap);

    // Set the height of the node in the tree
    mNodes[nodeID].height = 0;

    // Insert the new leaf node in the tree
    insertLeafNode(nodeID);
    assert(mNodes[nodeID].isLeaf());

    assert(nodeID >= 0);

    // Return the Id of the node
    return nodeID;
}

// Remove an object from the tree
void DynamicAABBTree::removeObject(int32 nodeID) {

    assert(nodeID >= 0 && nodeID < mNbAllocatedNodes);
    assert(mNodes[nodeID].isLeaf());

    // Remove the node from the tree
    removeLeafNode(nodeID);
    releaseNode(nodeID);
}

// Update the dynamic tree after an object has moved.
/// If the new AABB of the object that has moved is still inside its fat AABB, then
/// nothing is done. Otherwise, the corresponding node is removed and reinserted into the tree.
/// The method returns true if the object has been reinserted into the tree.
/// If the "forceReInsert" parameter is true, we force the existing AABB to take the size
/// of the "newAABB" parameter even if it is larger than "newAABB". This can be used to shrink the
/// AABB in the tree for instance if the corresponding collision shape has been shrunk.
bool DynamicAABBTree::updateObject(int32 nodeID, const AABB& newAABB, bool forceReinsert) {

    RP3D_PROFILE("DynamicAABBTree::updateObject()", mProfiler);

    assert(nodeID >= 0 && nodeID < mNbAllocatedNodes);
    assert(mNodes[nodeID].isLeaf());
    assert(mNodes[nodeID].height >= 0);

    // If the new AABB is still inside the fat AABB of the node
    if (!forceReinsert && mNodes[nodeID].aabb.contains(newAABB)) {
        return false;
    }

    // If the new AABB is outside the fat AABB, we remove the corresponding node
    removeLeafNode(nodeID);

    // Compute the fat AABB by inflating the AABB with by a constant percentage of the size of the AABB
    mNodes[nodeID].aabb = newAABB;
    const Vector3 gap(newAABB.getExtent() * mFatAABBInflatePercentage * decimal(0.5f));
    mNodes[nodeID].aabb.mMinCoordinates -= gap;
    mNodes[nodeID].aabb.mMaxCoordinates += gap;

    assert(mNodes[nodeID].aabb.contains(newAABB));

    // Reinsert the node into the tree
    insertLeafNode(nodeID);

    return true;
}

// Insert a leaf node in the tree. The process of inserting a new leaf node
// in the dynamic tree is described in the book "Introduction to Game Physics
// with Box2D" by Ian Parberry.
void DynamicAABBTree::insertLeafNode(int nodeID) {

    // If the tree is empty
    if (mRootNodeID == TreeNode::NULL_TREE_NODE) {
        mRootNodeID = nodeID;
        mNodes[mRootNodeID].parentID = TreeNode::NULL_TREE_NODE;
        return;
    }

    assert(mRootNodeID != TreeNode::NULL_TREE_NODE);

    // Find the best sibling node for the new node
    AABB newNodeAABB = mNodes[nodeID].aabb;
    int currentNodeID = mRootNodeID;
    while (!mNodes[currentNodeID].isLeaf()) {

        int leftChild = mNodes[currentNodeID].children[0];
        int rightChild = mNodes[currentNodeID].children[1];

        // Compute the merged AABB
        decimal volumeAABB = mNodes[currentNodeID].aabb.getVolume();
        AABB mergedAABBs;
        mergedAABBs.mergeTwoAABBs(mNodes[currentNodeID].aabb, newNodeAABB);
        decimal mergedVolume = mergedAABBs.getVolume();

        // Compute the cost of making the current node the sibbling of the new node
        decimal costS = decimal(2.0) * mergedVolume;

        // Compute the minimum cost of pushing the new node further down the tree (inheritance cost)
        decimal costI = decimal(2.0) * (mergedVolume - volumeAABB);

        // Compute the cost of descending into the left child
        decimal costLeft;
        AABB currentAndLeftAABB;
        currentAndLeftAABB.mergeTwoAABBs(newNodeAABB, mNodes[leftChild].aabb);
        if (mNodes[leftChild].isLeaf()) {   // If the left child is a leaf
            costLeft = currentAndLeftAABB.getVolume() + costI;
        }
        else {
            decimal leftChildVolume = mNodes[leftChild].aabb.getVolume();
            costLeft = costI + currentAndLeftAABB.getVolume() - leftChildVolume;
        }

        // Compute the cost of descending into the right child
        decimal costRight;
        AABB currentAndRightAABB;
        currentAndRightAABB.mergeTwoAABBs(newNodeAABB, mNodes[rightChild].aabb);
        if (mNodes[rightChild].isLeaf()) {   // If the right child is a leaf
            costRight = currentAndRightAABB.getVolume() + costI;
        }
        else {
            decimal rightChildVolume = mNodes[rightChild].aabb.getVolume();
            costRight = costI + currentAndRightAABB.getVolume() - rightChildVolume;
        }

        // If the cost of making the current node a sibbling of the new node is smaller than
        // the cost of going down into the left or right child
        if (costS < costLeft && costS < costRight) break;

        // It is cheaper to go down into a child of the current node, choose the best child
        if (costLeft < costRight) {
            currentNodeID = leftChild;
        }
        else {
            currentNodeID = rightChild;
        }
    }

    int siblingNode = currentNodeID;

    // Create a new parent for the new node and the sibling node
    int oldParentNode = mNodes[siblingNode].parentID;
    int newParentNode = allocateNode();
    mNodes[newParentNode].parentID = oldParentNode;
    mNodes[newParentNode].aabb.mergeTwoAABBs(mNodes[siblingNode].aabb, newNodeAABB);
    mNodes[newParentNode].height = mNodes[siblingNode].height + 1;
    assert(mNodes[newParentNode].height > 0);

    // If the sibling node was not the root node
    if (oldParentNode != TreeNode::NULL_TREE_NODE) {
        assert(!mNodes[oldParentNode].isLeaf());
        if (mNodes[oldParentNode].children[0] == siblingNode) {
            mNodes[oldParentNode].children[0] = newParentNode;
        }
        else {
            mNodes[oldParentNode].children[1] = newParentNode;
        }
        mNodes[newParentNode].children[0] = siblingNode;
        mNodes[newParentNode].children[1] = nodeID;
        mNodes[siblingNode].parentID = newParentNode;
        mNodes[nodeID].parentID = newParentNode;
    }
    else {  // If the sibling node was the root node
        mNodes[newParentNode].children[0] = siblingNode;
        mNodes[newParentNode].children[1] = nodeID;
        mNodes[siblingNode].parentID = newParentNode;
        mNodes[nodeID].parentID = newParentNode;
        mRootNodeID = newParentNode;
    }

    // Move up in the tree to change the AABBs that have changed
    currentNodeID = mNodes[nodeID].parentID;
    assert(!mNodes[currentNodeID].isLeaf());
    while (currentNodeID != TreeNode::NULL_TREE_NODE) {

        // Balance the sub-tree of the current node if it is not balanced
        currentNodeID = balanceSubTreeAtNode(currentNodeID);
        assert(mNodes[nodeID].isLeaf());

        assert(!mNodes[currentNodeID].isLeaf());
        int leftChild = mNodes[currentNodeID].children[0];
        int rightChild = mNodes[currentNodeID].children[1];
        assert(leftChild != TreeNode::NULL_TREE_NODE);
        assert(rightChild != TreeNode::NULL_TREE_NODE);

        // Recompute the height of the node in the tree
        mNodes[currentNodeID].height = std::max(mNodes[leftChild].height,
                                                mNodes[rightChild].height) + 1;
        assert(mNodes[currentNodeID].height > 0);

        // Recompute the AABB of the node
        mNodes[currentNodeID].aabb.mergeTwoAABBs(mNodes[leftChild].aabb, mNodes[rightChild].aabb);

        currentNodeID = mNodes[currentNodeID].parentID;
    }

    assert(mNodes[nodeID].isLeaf());
}

// Remove a leaf node from the tree
void DynamicAABBTree::removeLeafNode(int nodeID) {

    assert(nodeID >= 0 && nodeID < mNbAllocatedNodes);
    assert(mNodes[nodeID].isLeaf());

    // If we are removing the root node (root node is a leaf in this case)
    if (mRootNodeID == nodeID) {
        mRootNodeID = TreeNode::NULL_TREE_NODE;
        return;
    }

    int parentNodeID = mNodes[nodeID].parentID;
    int grandParentNodeID = mNodes[parentNodeID].parentID;
    int siblingNodeID;
    if (mNodes[parentNodeID].children[0] == nodeID) {
        siblingNodeID = mNodes[parentNodeID].children[1];
    }
    else {
        siblingNodeID = mNodes[parentNodeID].children[0];
    }

    // If the parent of the node to remove is not the root node
    if (grandParentNodeID != TreeNode::NULL_TREE_NODE) {

        // Destroy the parent node
        if (mNodes[grandParentNodeID].children[0] == parentNodeID) {
            mNodes[grandParentNodeID].children[0] = siblingNodeID;
        }
        else {
            assert(mNodes[grandParentNodeID].children[1] == parentNodeID);
            mNodes[grandParentNodeID].children[1] = siblingNodeID;
        }
        mNodes[siblingNodeID].parentID = grandParentNodeID;
        releaseNode(parentNodeID);

        // Now, we need to recompute the AABBs of the node on the path back to the root
        // and make sure that the tree is still balanced
        int currentNodeID = grandParentNodeID;
        while(currentNodeID != TreeNode::NULL_TREE_NODE) {

            // Balance the current sub-tree if necessary
            currentNodeID = balanceSubTreeAtNode(currentNodeID);

            assert(!mNodes[currentNodeID].isLeaf());

            // Get the two children of the current node
            int leftChildID = mNodes[currentNodeID].children[0];
            int rightChildID = mNodes[currentNodeID].children[1];

            // Recompute the AABB and the height of the current node
            mNodes[currentNodeID].aabb.mergeTwoAABBs(mNodes[leftChildID].aabb,
                                                     mNodes[rightChildID].aabb);
            mNodes[currentNodeID].height = std::max(mNodes[leftChildID].height,
                                                    mNodes[rightChildID].height) + 1;
            assert(mNodes[currentNodeID].height > 0);

            currentNodeID = mNodes[currentNodeID].parentID;
        }
    }
    else { // If the parent of the node to remove is the root node

        // The sibling node becomes the new root node
        mRootNodeID = siblingNodeID;
        mNodes[siblingNodeID].parentID = TreeNode::NULL_TREE_NODE;
        releaseNode(parentNodeID);
    }
}

// Balance the sub-tree of a given node using left or right rotations.
/// The rotation schemes are described in the book "Introduction to Game Physics
/// with Box2D" by Ian Parberry. This method returns the new root node ID.
int32 DynamicAABBTree::balanceSubTreeAtNode(int32 nodeID) {

    assert(nodeID != TreeNode::NULL_TREE_NODE);

    TreeNode* nodeA = mNodes + nodeID;

    // If the node is a leaf or the height of A's sub-tree is less than 2
    if (nodeA->isLeaf() || nodeA->height < 2) {

        // Do not perform any rotation
        return nodeID;
    }

    // Get the two children nodes
    int nodeBID = nodeA->children[0];
    int nodeCID = nodeA->children[1];
    assert(nodeBID >= 0 && nodeBID < mNbAllocatedNodes);
    assert(nodeCID >= 0 && nodeCID < mNbAllocatedNodes);
    TreeNode* nodeB = mNodes + nodeBID;
    TreeNode* nodeC = mNodes + nodeCID;

    // Compute the factor of the left and right sub-trees
    int balanceFactor = nodeC->height - nodeB->height;

    // If the right node C is 2 higher than left node B
    if (balanceFactor > 1) {

        assert(!nodeC->isLeaf());

        int nodeFID = nodeC->children[0];
        int nodeGID = nodeC->children[1];
        assert(nodeFID >= 0 && nodeFID < mNbAllocatedNodes);
        assert(nodeGID >= 0 && nodeGID < mNbAllocatedNodes);
        TreeNode* nodeF = mNodes + nodeFID;
        TreeNode* nodeG = mNodes + nodeGID;

        nodeC->children[0] = nodeID;
        nodeC->parentID = nodeA->parentID;
        nodeA->parentID = nodeCID;

        if (nodeC->parentID != TreeNode::NULL_TREE_NODE) {

            if (mNodes[nodeC->parentID].children[0] == nodeID) {
                mNodes[nodeC->parentID].children[0] = nodeCID;
            }
            else {
                assert(mNodes[nodeC->parentID].children[1] == nodeID);
                mNodes[nodeC->parentID].children[1] = nodeCID;
            }
        }
        else {
            mRootNodeID = nodeCID;
        }

        assert(!nodeC->isLeaf());
        assert(!nodeA->isLeaf());

        // If the right node C was higher than left node B because of the F node
        if (nodeF->height > nodeG->height) {

            nodeC->children[1] = nodeFID;
            nodeA->children[1] = nodeGID;
            nodeG->parentID = nodeID;

            // Recompute the AABB of node A and C
            nodeA->aabb.mergeTwoAABBs(nodeB->aabb, nodeG->aabb);
            nodeC->aabb.mergeTwoAABBs(nodeA->aabb, nodeF->aabb);

            // Recompute the height of node A and C
            nodeA->height = std::max(nodeB->height, nodeG->height) + 1;
            nodeC->height = std::max(nodeA->height, nodeF->height) + 1;
            assert(nodeA->height > 0);
            assert(nodeC->height > 0);
        }
        else {  // If the right node C was higher than left node B because of node G
            nodeC->children[1] = nodeGID;
            nodeA->children[1] = nodeFID;
            nodeF->parentID = nodeID;

            // Recompute the AABB of node A and C
            nodeA->aabb.mergeTwoAABBs(nodeB->aabb, nodeF->aabb);
            nodeC->aabb.mergeTwoAABBs(nodeA->aabb, nodeG->aabb);

            // Recompute the height of node A and C
            nodeA->height = std::max(nodeB->height, nodeF->height) + 1;
            nodeC->height = std::max(nodeA->height, nodeG->height) + 1;
            assert(nodeA->height > 0);
            assert(nodeC->height > 0);
        }

        // Return the new root of the sub-tree
        return nodeCID;
    }

    // If the left node B is 2 higher than right node C
    if (balanceFactor < -1) {

        assert(!nodeB->isLeaf());

        int nodeFID = nodeB->children[0];
        int nodeGID = nodeB->children[1];
        assert(nodeFID >= 0 && nodeFID < mNbAllocatedNodes);
        assert(nodeGID >= 0 && nodeGID < mNbAllocatedNodes);
        TreeNode* nodeF = mNodes + nodeFID;
        TreeNode* nodeG = mNodes + nodeGID;

        nodeB->children[0] = nodeID;
        nodeB->parentID = nodeA->parentID;
        nodeA->parentID = nodeBID;

        if (nodeB->parentID != TreeNode::NULL_TREE_NODE) {

            if (mNodes[nodeB->parentID].children[0] == nodeID) {
                mNodes[nodeB->parentID].children[0] = nodeBID;
            }
            else {
                assert(mNodes[nodeB->parentID].children[1] == nodeID);
                mNodes[nodeB->parentID].children[1] = nodeBID;
            }
        }
        else {
            mRootNodeID = nodeBID;
        }

        assert(!nodeB->isLeaf());
        assert(!nodeA->isLeaf());

        // If the left node B was higher than right node C because of the F node
        if (nodeF->height > nodeG->height) {

            nodeB->children[1] = nodeFID;
            nodeA->children[0] = nodeGID;
            nodeG->parentID = nodeID;

            // Recompute the AABB of node A and B
            nodeA->aabb.mergeTwoAABBs(nodeC->aabb, nodeG->aabb);
            nodeB->aabb.mergeTwoAABBs(nodeA->aabb, nodeF->aabb);

            // Recompute the height of node A and B
            nodeA->height = std::max(nodeC->height, nodeG->height) + 1;
            nodeB->height = std::max(nodeA->height, nodeF->height) + 1;
            assert(nodeA->height > 0);
            assert(nodeB->height > 0);
        }
        else {  // If the left node B was higher than right node C because of node G
            nodeB->children[1] = nodeGID;
            nodeA->children[0] = nodeFID;
            nodeF->parentID = nodeID;

            // Recompute the AABB of node A and B
            nodeA->aabb.mergeTwoAABBs(nodeC->aabb, nodeF->aabb);
            nodeB->aabb.mergeTwoAABBs(nodeA->aabb, nodeG->aabb);

            // Recompute the height of node A and B
            nodeA->height = std::max(nodeC->height, nodeF->height) + 1;
            nodeB->height = std::max(nodeA->height, nodeG->height) + 1;
            assert(nodeA->height > 0);
            assert(nodeB->height > 0);
        }

        // Return the new root of the sub-tree
        return nodeBID;
    }

    // If the sub-tree is balanced, return the current root node
    return nodeID;
}

/// Take a list of shapes to be tested for broad-phase overlap and return a list of pair of overlapping shapes
void DynamicAABBTree::reportAllShapesOverlappingWithShapes(const List<int32>& nodesToTest, size_t startIndex,
                                                           size_t endIndex, List<Pair<int32, int32>>& outOverlappingNodes) const {

    RP3D_PROFILE("DynamicAABBTree::reportAllShapesOverlappingWithAABB()", mProfiler);

    // Create a stack with the nodes to visit
    Stack<int32> stack(mAllocator, 64);

    // For each shape to be tested for overlap
    for (uint i=startIndex; i < endIndex; i++) {

        assert(nodesToTest[i] != -1);

        stack.push(mRootNodeID);

        const AABB& shapeAABB = getFatAABB(nodesToTest[i]);

        // While there are still nodes to visit
        while(stack.size() > 0) {

            // Get the next node ID to visit
            const int32 nodeIDToVisit = stack.pop();

            // Skip it if it is a null node
            if (nodeIDToVisit == TreeNode::NULL_TREE_NODE) continue;

            // Get the corresponding node
            const TreeNode* nodeToVisit = mNodes + nodeIDToVisit;

            // If the AABB in parameter overlaps with the AABB of the node to visit
            if (shapeAABB.testCollision(nodeToVisit->aabb)) {

                // If the node is a leaf
                if (nodeToVisit->isLeaf()) {

                    // Add the node in the list of overlapping nodes
                    outOverlappingNodes.add(Pair<int32, int32>(nodesToTest[i], nodeIDToVisit));
                }
                else {  // If the node is not a leaf

                    // We need to visit its children
                    stack.push(nodeToVisit->children[0]);
                    stack.push(nodeToVisit->children[1]);
                }
            }
        }

        stack.clear();
    }
}

// Report all shapes overlapping with the AABB given in parameter.
void DynamicAABBTree::reportAllShapesOverlappingWithAABB(const AABB& aabb, List<int32>& overlappingNodes) const {

    RP3D_PROFILE("DynamicAABBTree::reportAllShapesOverlappingWithAABB()", mProfiler);

    // Create a stack with the nodes to visit
    Stack<int32> stack(mAllocator, 64);
    stack.push(mRootNodeID);

    // While there are still nodes to visit
    while(stack.size() > 0) {

        // Get the next node ID to visit
        const int32 nodeIDToVisit = stack.pop();

        // Skip it if it is a null node
        if (nodeIDToVisit == TreeNode::NULL_TREE_NODE) continue;

        // Get the corresponding node
        const TreeNode* nodeToVisit = mNodes + nodeIDToVisit;

        // If the AABB in parameter overlaps with the AABB of the node to visit
        if (aabb.testCollision(nodeToVisit->aabb)) {

            // If the node is a leaf
            if (nodeToVisit->isLeaf()) {

                // Notify the broad-phase about a new potential overlapping pair
                overlappingNodes.add(nodeIDToVisit);
            }
            else {  // If the node is not a leaf

                // We need to visit its children
                stack.push(nodeToVisit->children[0]);
                stack.push(nodeToVisit->children[1]);
            }
        }
    }
}

// Ray casting method
void DynamicAABBTree::raycast(const Ray& ray, DynamicAABBTreeRaycastCallback& callback) const {

    RP3D_PROFILE("DynamicAABBTree::raycast()", mProfiler);

    decimal maxFraction = ray.maxFraction;

    Stack<int32> stack(mAllocator, 128);
    stack.push(mRootNodeID);

    // Walk through the tree from the root looking for colliders
    // that overlap with the ray AABB
    while (stack.size() > 0) {

        // Get the next node in the stack
        int32 nodeID = stack.pop();

        // If it is a null node, skip it
        if (nodeID == TreeNode::NULL_TREE_NODE) continue;

        // Get the corresponding node
        const TreeNode* node = mNodes + nodeID;

        Ray rayTemp(ray.point1, ray.point2, maxFraction);

        // Test if the ray intersects with the current node AABB
        if (!node->aabb.testRayIntersect(rayTemp)) continue;

        // If the node is a leaf of the tree
        if (node->isLeaf()) {

            // Call the callback that will raycast again the broad-phase shape
            decimal hitFraction = callback.raycastBroadPhaseShape(nodeID, rayTemp);

            // If the user returned a hitFraction of zero, it means that
            // the raycasting should stop here
            if (hitFraction == decimal(0.0)) {
                return;
            }

            // If the user returned a positive fraction
            if (hitFraction > decimal(0.0)) {

                // We update the maxFraction value and the ray
                // AABB using the new maximum fraction
                if (hitFraction < maxFraction) {
                    maxFraction = hitFraction;
                }
            }

            // If the user returned a negative fraction, we continue
            // the raycasting as if the collider did not exist
        }
        else {  // If the node has children

            // Push its children in the stack of nodes to explore
            stack.push(node->children[0]);
            stack.push(node->children[1]);
        }
    }
}

#ifndef NDEBUG

// Check if the tree structure is valid (for debugging purpose)
void DynamicAABBTree::check() const {

    // Recursively check each node
    checkNode(mRootNodeID);

    int32 nbFreeNodes = 0;
    int32 freeNodeID = mFreeNodeID;

    // Check the free nodes
    while(freeNodeID != TreeNode::NULL_TREE_NODE) {
        assert(0 <= freeNodeID && freeNodeID < mNbAllocatedNodes);
        freeNodeID = mNodes[freeNodeID].nextNodeID;
        nbFreeNodes++;
    }

    assert(mNbNodes + nbFreeNodes == mNbAllocatedNodes);
}

// Check if the node structure is valid (for debugging purpose)
void DynamicAABBTree::checkNode(int32 nodeID) const {

    if (nodeID == TreeNode::NULL_TREE_NODE) return;

    // If it is the root
    if (nodeID == mRootNodeID) {
        assert(mNodes[nodeID].parentID == TreeNode::NULL_TREE_NODE);
    }

    // Get the children nodes
    TreeNode* pNode = mNodes + nodeID;
    assert(!pNode->isLeaf());
    int32 leftChild = pNode->children[0];
    int32 rightChild = pNode->children[1];

    assert(pNode->height >= 0);
    assert(pNode->aabb.getVolume() > 0);

    // If the current node is a leaf
    if (pNode->isLeaf()) {

        // Check that there are no children
        assert(leftChild == TreeNode::NULL_TREE_NODE);
        assert(rightChild == TreeNode::NULL_TREE_NODE);
        assert(pNode->height == 0);
    }
    else {

        // Check that the children node IDs are valid
        assert(0 <= leftChild && leftChild < mNbAllocatedNodes);
        assert(0 <= rightChild && rightChild < mNbAllocatedNodes);

        // Check that the children nodes have the correct parent node
        assert(mNodes[leftChild].parentID == nodeID);
        assert(mNodes[rightChild].parentID == nodeID);

        // Check the height of node
        int height = 1 + std::max(mNodes[leftChild].height, mNodes[rightChild].height);
        assert(mNodes[nodeID].height == height);

        // Check the AABB of the node
        AABB aabb;
        aabb.mergeTwoAABBs(mNodes[leftChild].aabb, mNodes[rightChild].aabb);
        assert(aabb.getMin() == mNodes[nodeID].aabb.getMin());
        assert(aabb.getMax() == mNodes[nodeID].aabb.getMax());

        // Recursively check the children nodes
        checkNode(leftChild);
        checkNode(rightChild);
    }
}

// Compute the height of the tree
int DynamicAABBTree::computeHeight() {
   return computeHeight(mRootNodeID);
}

// Compute the height of a given node in the tree
int DynamicAABBTree::computeHeight(int32 nodeID) {
    assert(nodeID >= 0 && nodeID < mNbAllocatedNodes);
    TreeNode* node = mNodes + nodeID;

    // If the node is a leaf, its height is zero
    if (node->isLeaf()) {
        return 0;
    }

    // Compute the height of the left and right sub-tree
    int leftHeight = computeHeight(node->children[0]);
    int rightHeight = computeHeight(node->children[1]);

    // Return the height of the node
    return 1 + std::max(leftHeight, rightHeight);
}

#endif
