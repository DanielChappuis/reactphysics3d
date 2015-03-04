/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2015 Daniel Chappuis                                       *
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
#include "DynamicAABBTree.h"
#include "BroadPhaseAlgorithm.h"
#include "memory/Stack.h"
#include "engine/Profiler.h"

using namespace reactphysics3d;

// Initialization of static variables
const int TreeNode::NULL_TREE_NODE = -1;

// Constructor
DynamicAABBTree::DynamicAABBTree(BroadPhaseAlgorithm& broadPhase) : mBroadPhase(broadPhase){

    mRootNodeID = TreeNode::NULL_TREE_NODE;
    mNbNodes = 0;
    mNbAllocatedNodes = 8;

    // Allocate memory for the nodes of the tree
    mNodes = (TreeNode*) malloc(mNbAllocatedNodes * sizeof(TreeNode));
    assert(mNodes);
    memset(mNodes, 0, mNbAllocatedNodes * sizeof(TreeNode));

    // Initialize the allocated nodes
    for (int i=0; i<mNbAllocatedNodes - 1; i++) {
        mNodes[i].nextNodeID = i + 1;
        mNodes[i].height = -1;
    }
    mNodes[mNbAllocatedNodes - 1].nextNodeID = TreeNode::NULL_TREE_NODE;
    mNodes[mNbAllocatedNodes - 1].height = -1;
    mFreeNodeID = 0;
}

// Destructor
DynamicAABBTree::~DynamicAABBTree() {

    // Free the allocated memory for the nodes
    free(mNodes);
}

// Allocate and return a new node in the tree
int DynamicAABBTree::allocateNode() {

    // If there is no more allocated node to use
    if (mFreeNodeID == TreeNode::NULL_TREE_NODE) {

        assert(mNbNodes == mNbAllocatedNodes);

        // Allocate more nodes in the tree
        mNbAllocatedNodes *= 2;
        TreeNode* oldNodes = mNodes;
        mNodes = (TreeNode*) malloc(mNbAllocatedNodes * sizeof(TreeNode));
        assert(mNodes);
        memcpy(mNodes, oldNodes, mNbNodes * sizeof(TreeNode));
        free(oldNodes);

        // Initialize the allocated nodes
        for (int i=mNbNodes; i<mNbAllocatedNodes - 1; i++) {
            mNodes[i].nextNodeID = i + 1;
            mNodes[i].height = -1;
        }
        mNodes[mNbAllocatedNodes - 1].nextNodeID = TreeNode::NULL_TREE_NODE;
        mNodes[mNbAllocatedNodes - 1].height = -1;
        mFreeNodeID = mNbNodes;
    }

    // Get the next free node
    int freeNodeID = mFreeNodeID;
    mFreeNodeID = mNodes[freeNodeID].nextNodeID;
    mNodes[freeNodeID].parentID = TreeNode::NULL_TREE_NODE;
    mNodes[freeNodeID].leftChildID = TreeNode::NULL_TREE_NODE;
    mNodes[freeNodeID].rightChildID = TreeNode::NULL_TREE_NODE;
    mNodes[freeNodeID].proxyShape = NULL;
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

// Add an object into the tree. This method creates a new leaf node in the tree and
// returns the ID of the corresponding node.
void DynamicAABBTree::addObject(ProxyShape* proxyShape, const AABB& aabb) {

    // Get the next available node (or allocate new ones if necessary)
    int nodeID = allocateNode();

    // Create the fat aabb to use in the tree
    const Vector3 gap(DYNAMIC_TREE_AABB_GAP, DYNAMIC_TREE_AABB_GAP, DYNAMIC_TREE_AABB_GAP);
    mNodes[nodeID].aabb.setMin(aabb.getMin() - gap);
    mNodes[nodeID].aabb.setMax(aabb.getMax() + gap);

    // Set the collision shape
    mNodes[nodeID].proxyShape = proxyShape;

    // Set the height of the node in the tree
    mNodes[nodeID].height = 0;

    // Insert the new leaf node in the tree
    insertLeafNode(nodeID);
    assert(mNodes[nodeID].isLeaf());

    // Set the broad-phase ID of the proxy shape
    proxyShape->mBroadPhaseID = nodeID;
    assert(nodeID >= 0);
}

// Remove an object from the tree
void DynamicAABBTree::removeObject(int nodeID) {

    assert(nodeID >= 0 && nodeID < mNbAllocatedNodes);
    assert(mNodes[nodeID].isLeaf());

    // Remove the node from the tree
    removeLeafNode(nodeID);
    releaseNode(nodeID);
}

// Update the dynamic tree after an object has moved.
/// If the new AABB of the object that has moved is still inside its fat AABB, then
/// nothing is done. Otherwise, the corresponding node is removed and reinserted into the tree.
/// The method returns true if the object has been reinserted into the tree. The "displacement"
/// argument is the linear velocity of the AABB multiplied by the elapsed time between two
/// frames.
bool DynamicAABBTree::updateObject(int nodeID, const AABB& newAABB, const Vector3& displacement) {

    PROFILE("DynamicAABBTree::updateObject()");

    assert(nodeID >= 0 && nodeID < mNbAllocatedNodes);
    assert(mNodes[nodeID].isLeaf());
    assert(mNodes[nodeID].height >= 0);

    // If the new AABB is still inside the fat AABB of the node
    if (mNodes[nodeID].aabb.contains(newAABB)) {
        return false;
    }

    // If the new AABB is outside the fat AABB, we remove the corresponding node
    removeLeafNode(nodeID);

    // Compute the fat AABB by inflating the AABB with a constant gap
    mNodes[nodeID].aabb = newAABB;
    const Vector3 gap(DYNAMIC_TREE_AABB_GAP, DYNAMIC_TREE_AABB_GAP, DYNAMIC_TREE_AABB_GAP);
    mNodes[nodeID].aabb.mMinCoordinates -= gap;
    mNodes[nodeID].aabb.mMaxCoordinates += gap;

    // Inflate the fat AABB in direction of the linear motion of the AABB
    if (displacement.x < decimal(0.0)) {
      mNodes[nodeID].aabb.mMinCoordinates.x += DYNAMIC_TREE_AABB_LIN_GAP_MULTIPLIER *displacement.x;
    }
    else {
      mNodes[nodeID].aabb.mMaxCoordinates.x += DYNAMIC_TREE_AABB_LIN_GAP_MULTIPLIER *displacement.x;
    }
    if (displacement.y < decimal(0.0)) {
      mNodes[nodeID].aabb.mMinCoordinates.y += DYNAMIC_TREE_AABB_LIN_GAP_MULTIPLIER *displacement.y;
    }
    else {
      mNodes[nodeID].aabb.mMaxCoordinates.y += DYNAMIC_TREE_AABB_LIN_GAP_MULTIPLIER *displacement.y;
    }
    if (displacement.z < decimal(0.0)) {
      mNodes[nodeID].aabb.mMinCoordinates.z += DYNAMIC_TREE_AABB_LIN_GAP_MULTIPLIER *displacement.z;
    }
    else {
      mNodes[nodeID].aabb.mMaxCoordinates.z += DYNAMIC_TREE_AABB_LIN_GAP_MULTIPLIER *displacement.z;
    }

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

        int leftChild = mNodes[currentNodeID].leftChildID;
        int rightChild = mNodes[currentNodeID].rightChildID;

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
    mNodes[newParentNode].proxyShape = NULL;
    mNodes[newParentNode].aabb.mergeTwoAABBs(mNodes[siblingNode].aabb, newNodeAABB);
    mNodes[newParentNode].height = mNodes[siblingNode].height + 1;
    assert(mNodes[newParentNode].height > 0);

    // If the sibling node was not the root node
    if (oldParentNode != TreeNode::NULL_TREE_NODE) {
        if (mNodes[oldParentNode].leftChildID == siblingNode) {
            mNodes[oldParentNode].leftChildID = newParentNode;
        }
        else {
            mNodes[oldParentNode].rightChildID = newParentNode;
        }
        mNodes[newParentNode].leftChildID = siblingNode;
        mNodes[newParentNode].rightChildID = nodeID;
        mNodes[siblingNode].parentID = newParentNode;
        mNodes[nodeID].parentID = newParentNode;
    }
    else {  // If the sibling node was the root node
        mNodes[newParentNode].leftChildID = siblingNode;
        mNodes[newParentNode].rightChildID = nodeID;
        mNodes[siblingNode].parentID = newParentNode;
        mNodes[nodeID].parentID = newParentNode;
        mRootNodeID = newParentNode;
    }

    // Move up in the tree to change the AABBs that have changed
    currentNodeID = mNodes[nodeID].parentID;
    while (currentNodeID != TreeNode::NULL_TREE_NODE) {

        // Balance the sub-tree of the current node if it is not balanced
        currentNodeID = balanceSubTreeAtNode(currentNodeID);
        assert(mNodes[nodeID].isLeaf());

        int leftChild = mNodes[currentNodeID].leftChildID;
        int rightChild = mNodes[currentNodeID].rightChildID;
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
    if (mNodes[parentNodeID].leftChildID == nodeID) {
        siblingNodeID = mNodes[parentNodeID].rightChildID;
    }
    else {
        siblingNodeID = mNodes[parentNodeID].leftChildID;
    }

    // If the parent of the node to remove is not the root node
    if (grandParentNodeID != TreeNode::NULL_TREE_NODE) {

        // Destroy the parent node
        if (mNodes[grandParentNodeID].leftChildID == parentNodeID) {
            mNodes[grandParentNodeID].leftChildID = siblingNodeID;
        }
        else {
            assert(mNodes[grandParentNodeID].rightChildID == parentNodeID);
            mNodes[grandParentNodeID].rightChildID = siblingNodeID;
        }
        mNodes[siblingNodeID].parentID = grandParentNodeID;
        releaseNode(parentNodeID);

        // Now, we need to recompute the AABBs of the node on the path back to the root
        // and make sure that the tree is still balanced
        int currentNodeID = grandParentNodeID;
        while(currentNodeID != TreeNode::NULL_TREE_NODE) {

            // Balance the current sub-tree if necessary
            currentNodeID = balanceSubTreeAtNode(currentNodeID);

            // Get the two children of the current node
            int leftChildID = mNodes[currentNodeID].leftChildID;
            int rightChildID = mNodes[currentNodeID].rightChildID;

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
int DynamicAABBTree::balanceSubTreeAtNode(int nodeID) {

    assert(nodeID != TreeNode::NULL_TREE_NODE);

    TreeNode* nodeA = mNodes + nodeID;

    // If the node is a leaf or the height of A's sub-tree is less than 2
    if (nodeA->isLeaf() || nodeA->height < 2) {

        // Do not perform any rotation
        return nodeID;
    }

    // Get the two children nodes
    int nodeBID = nodeA->leftChildID;
    int nodeCID = nodeA->rightChildID;
    assert(nodeBID >= 0 && nodeBID < mNbAllocatedNodes);
    assert(nodeCID >= 0 && nodeCID < mNbAllocatedNodes);
    TreeNode* nodeB = mNodes + nodeBID;
    TreeNode* nodeC = mNodes + nodeCID;

    // Compute the factor of the left and right sub-trees
    int balanceFactor = nodeC->height - nodeB->height;

    // If the right node C is 2 higher than left node B
    if (balanceFactor > 1) {

        int nodeFID = nodeC->leftChildID;
        int nodeGID = nodeC->rightChildID;
        assert(nodeFID >= 0 && nodeFID < mNbAllocatedNodes);
        assert(nodeGID >= 0 && nodeGID < mNbAllocatedNodes);
        TreeNode* nodeF = mNodes + nodeFID;
        TreeNode* nodeG = mNodes + nodeGID;

        nodeC->leftChildID = nodeID;
        nodeC->parentID = nodeA->parentID;
        nodeA->parentID = nodeCID;

        if (nodeC->parentID != TreeNode::NULL_TREE_NODE) {

            if (mNodes[nodeC->parentID].leftChildID == nodeID) {
                mNodes[nodeC->parentID].leftChildID = nodeCID;
            }
            else {
                assert(mNodes[nodeC->parentID].rightChildID == nodeID);
                mNodes[nodeC->parentID].rightChildID = nodeCID;
            }
        }
        else {
            mRootNodeID = nodeCID;
        }

        // If the right node C was higher than left node B because of the F node
        if (nodeF->height > nodeG->height) {
            nodeC->rightChildID = nodeFID;
            nodeA->rightChildID = nodeGID;
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
            nodeC->rightChildID = nodeGID;
            nodeA->rightChildID = nodeFID;
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

        int nodeFID = nodeB->leftChildID;
        int nodeGID = nodeB->rightChildID;
        assert(nodeFID >= 0 && nodeFID < mNbAllocatedNodes);
        assert(nodeGID >= 0 && nodeGID < mNbAllocatedNodes);
        TreeNode* nodeF = mNodes + nodeFID;
        TreeNode* nodeG = mNodes + nodeGID;

        nodeB->leftChildID = nodeID;
        nodeB->parentID = nodeA->parentID;
        nodeA->parentID = nodeBID;

        if (nodeB->parentID != TreeNode::NULL_TREE_NODE) {

            if (mNodes[nodeB->parentID].leftChildID == nodeID) {
                mNodes[nodeB->parentID].leftChildID = nodeBID;
            }
            else {
                assert(mNodes[nodeB->parentID].rightChildID == nodeID);
                mNodes[nodeB->parentID].rightChildID = nodeBID;
            }
        }
        else {
            mRootNodeID = nodeBID;
        }

        // If the left node B was higher than right node C because of the F node
        if (nodeF->height > nodeG->height) {
            nodeB->rightChildID = nodeFID;
            nodeA->leftChildID = nodeGID;
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
            nodeB->rightChildID = nodeGID;
            nodeA->leftChildID = nodeFID;
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

// Report all shapes overlapping with the AABB given in parameter.
/// For each overlapping shape with the AABB given in parameter, the
/// BroadPhase::notifyOverlappingPair() method is called to store a
/// potential overlapping pair.
void DynamicAABBTree::reportAllShapesOverlappingWith(int nodeID, const AABB& aabb) {

    // Create a stack with the nodes to visit
    Stack<int, 64> stack;
    stack.push(mRootNodeID);

    // While there are still nodes to visit
    while(stack.getNbElements() > 0) {

        // Get the next node ID to visit
        int nodeIDToVisit = stack.pop();

        // Skip it if it is a null node
        if (nodeIDToVisit == TreeNode::NULL_TREE_NODE) continue;

        // Get the corresponding node
        const TreeNode* nodeToVisit = mNodes + nodeIDToVisit;

        // If the AABB in parameter overlaps with the AABB of the node to visit
        if (aabb.testCollision(nodeToVisit->aabb)) {

            // If the node is a leaf
            if (nodeToVisit->isLeaf()) {

                // Notify the broad-phase about a new potential overlapping pair
                mBroadPhase.notifyOverlappingPair(nodeID, nodeIDToVisit);
            }
            else {  // If the node is not a leaf

                // We need to visit its children
                stack.push(nodeToVisit->leftChildID);
                stack.push(nodeToVisit->rightChildID);
            }
        }
    }
}

// Ray casting method
void DynamicAABBTree::raycast(const Ray& ray, RaycastTest& raycastTest,
                              unsigned short raycastWithCategoryMaskBits) const {

    decimal maxFraction = ray.maxFraction;

    // Create an AABB for the ray
    Vector3 endPoint = ray.point1 +
                             maxFraction * (ray.point2 - ray.point1);
    AABB rayAABB(Vector3::min(ray.point1, endPoint),
                 Vector3::max(ray.point1, endPoint));

    Stack<int, 128> stack;
    stack.push(mRootNodeID);

    // Walk through the tree from the root looking for proxy shapes
    // that overlap with the ray AABB
    while (stack.getNbElements() > 0) {

        // Get the next node in the stack
        int nodeID = stack.pop();

        // If it is a null node, skip it
        if (nodeID == TreeNode::NULL_TREE_NODE) continue;

        // Get the corresponding node
        const TreeNode* node = mNodes + nodeID;

        // Test if the node AABB overlaps with the ray AABB
        if (!rayAABB.testCollision(node->aabb)) continue;

        // If the node is a leaf of the tree
        if (node->isLeaf()) {

            // Check if the raycast filtering mask allows raycast against this shape
            if ((raycastWithCategoryMaskBits & node->proxyShape->getCollisionCategoryBits()) != 0) {

                Ray rayTemp(ray.point1, ray.point2, maxFraction);

                // Ask the collision detection to perform a ray cast test against
                // the proxy shape of this node because the ray is overlapping
                // with the shape in the broad-phase
                decimal hitFraction = raycastTest.raycastAgainstShape(node->proxyShape,
                                                                      rayTemp);

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
                    endPoint = ray.point1 + maxFraction * (ray.point2 - ray.point1);
                    rayAABB.mMinCoordinates = Vector3::min(ray.point1, endPoint);
                    rayAABB.mMaxCoordinates = Vector3::max(ray.point1, endPoint);
                }

                // If the user returned a negative fraction, we continue
                // the raycasting as if the proxy shape did not exist
            }

        }
        else {  // If the node has children

            // Push its children in the stack of nodes to explore
            stack.push(node->leftChildID);
            stack.push(node->rightChildID);
        }
    }
}

#ifndef NDEBUG

// Check if the tree structure is valid (for debugging purpose)
void DynamicAABBTree::check() const {

    // Recursively check each node
    checkNode(mRootNodeID);

    int nbFreeNodes = 0;
    int freeNodeID = mFreeNodeID;

    // Check the free nodes
    while(freeNodeID != TreeNode::NULL_TREE_NODE) {
        assert(0 <= freeNodeID && freeNodeID < mNbAllocatedNodes);
        freeNodeID = mNodes[freeNodeID].nextNodeID;
        nbFreeNodes++;
    }

    assert(mNbNodes + nbFreeNodes == mNbAllocatedNodes);
}

// Check if the node structure is valid (for debugging purpose)
void DynamicAABBTree::checkNode(int nodeID) const {

    if (nodeID == TreeNode::NULL_TREE_NODE) return;

    // If it is the root
    if (nodeID == mRootNodeID) {
        assert(mNodes[nodeID].parentID == TreeNode::NULL_TREE_NODE);
    }

    // Get the children nodes
    TreeNode* pNode = mNodes + nodeID;
    int leftChild = pNode->leftChildID;
    int rightChild = pNode->rightChildID;

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
int DynamicAABBTree::computeHeight(int nodeID) {
    assert(nodeID >= 0 && nodeID < mNbAllocatedNodes);
    TreeNode* node = mNodes + nodeID;

    // If the node is a leaf, its height is zero
    if (node->isLeaf()) {
        return 0;
    }

    // Compute the height of the left and right sub-tree
    int leftHeight = computeHeight(node->leftChildID);
    int rightHeight = computeHeight(node->rightChildID);

    // Return the height of the node
    return 1 + std::max(leftHeight, rightHeight);
}

#endif
