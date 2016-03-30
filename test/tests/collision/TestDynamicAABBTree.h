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

#ifndef TEST_DYNAMIC_AABB_TREE_H
#define TEST_DYNAMIC_AABB_TREE_H

// Libraries
#include "Test.h"
#include "collision/broadphase/DynamicAABBTree.h"
#include <vector>

/// Reactphysics3D namespace
namespace reactphysics3d {

class OverlapCallback : public DynamicAABBTreeOverlapCallback {

    public :

        std::vector<int> mOverlapNodes;

        // Called when a overlapping node has been found during the call to
        // DynamicAABBTree:reportAllShapesOverlappingWithAABB()
        virtual void notifyOverlappingNode(int nodeId) {
            mOverlapNodes.push_back(nodeId);
        }

        void reset() {
            mOverlapNodes.clear();
        }

        bool isOverlapping(int nodeId) const {
            return std::find(mOverlapNodes.begin(), mOverlapNodes.end(), nodeId) != mOverlapNodes.end();
        }
};

class DynamicTreeRaycastCallback : public DynamicAABBTreeRaycastCallback {

    public:

        std::vector<int> mHitNodes;

        // Called when the AABB of a leaf node is hit by a ray
        virtual decimal raycastBroadPhaseShape(int32 nodeId, const Ray& ray) {
            mHitNodes.push_back(nodeId);
            return 1.0;
        }

        void reset() {
            mHitNodes.clear();
        }

        bool isHit(int nodeId) const {
            return std::find(mHitNodes.begin(), mHitNodes.end(), nodeId) != mHitNodes.end();
        }
};

// Class TestDynamicAABBTree
/**
 * Unit test for the dynamic AABB tree
 */
class TestDynamicAABBTree : public Test {

    private :

        // ---------- Atributes ---------- //

        // Dynamic AABB Tree
        DynamicAABBTree mTree;

        AABB mAABB1;
        AABB mAABB2;
        AABB mAABB3;
        AABB mAABB4;

        OverlapCallback mOverlapCallback;
        DynamicTreeRaycastCallback mRaycastCallback;

        int mObject1Id;
        int mObject2Id;
        int mObject3Id;
        int mObject4Id;

        int mObject1Data = 56;
        int mObject2Data = 23;
        int mObject3Data = 13;
        int mObject4Data = 7;

    public :

        // ---------- Methods ---------- //

        /// Constructor
        TestDynamicAABBTree(const std::string& name): Test(name)  {

            // First object
            mAABB1 = AABB(Vector3(-6, 4, -3), Vector3(4, 8, 3));
            mObject1Id = mTree.addObject(mAABB1, &mObject1Data);

            // Second object
            mAABB2 = AABB(Vector3(5, 2, -3), Vector3(10, 7, 3));
            mObject2Id = mTree.addObject(mAABB2, &mObject2Data);

            // Third object
            mAABB3 = AABB(Vector3(-5, 1, -3), Vector3(-2, 3, 3));
            mObject3Id = mTree.addObject(mAABB3, &mObject3Data);

            // Fourth object
            mAABB4 = AABB(Vector3(0, -4, -3), Vector3(3, -2, 3));
            mObject4Id = mTree.addObject(mAABB4, &mObject4Data);
        }

        /// Run the tests
        void run() {

            testBasicsMethods();
            testOverlapping();
            testRaycast();

        }

        void testBasicsMethods() {

            // Test data stored at the nodes of the tree
            test(*(int*)(mTree.getNodeDataPointer(mObject1Id)) == mObject1Data);
            test(*(int*)(mTree.getNodeDataPointer(mObject2Id)) == mObject2Data);
            test(*(int*)(mTree.getNodeDataPointer(mObject3Id)) == mObject3Data);
            test(*(int*)(mTree.getNodeDataPointer(mObject4Id)) == mObject4Data);
        }

        void testOverlapping() {

            // AABB overlapping nothing
            mOverlapCallback.reset();
            mTree.reportAllShapesOverlappingWithAABB(AABB(Vector3(-10, 12, -4), Vector3(10, 50, 4)), mOverlapCallback);
            test(!mOverlapCallback.isOverlapping(mObject1Id));
            test(!mOverlapCallback.isOverlapping(mObject2Id));
            test(!mOverlapCallback.isOverlapping(mObject3Id));
            test(!mOverlapCallback.isOverlapping(mObject4Id));

            // AABB overlapping everything
            mOverlapCallback.reset();
            mTree.reportAllShapesOverlappingWithAABB(AABB(Vector3(-15, -15, -4), Vector3(15, 15, 4)), mOverlapCallback);
            test(mOverlapCallback.isOverlapping(mObject1Id));
            test(mOverlapCallback.isOverlapping(mObject2Id));
            test(mOverlapCallback.isOverlapping(mObject3Id));
            test(mOverlapCallback.isOverlapping(mObject4Id));

            // AABB overlapping object 1 and 3
            mOverlapCallback.reset();
            mTree.reportAllShapesOverlappingWithAABB(AABB(Vector3(-4, 2, -4), Vector3(-1, 7, 4)), mOverlapCallback);
            test(mOverlapCallback.isOverlapping(mObject1Id));
            test(!mOverlapCallback.isOverlapping(mObject2Id));
            test(mOverlapCallback.isOverlapping(mObject3Id));
            test(!mOverlapCallback.isOverlapping(mObject4Id));

            // AABB overlapping object 3 and 4
            mOverlapCallback.reset();
            mTree.reportAllShapesOverlappingWithAABB(AABB(Vector3(-6, -5, -2), Vector3(2, 2, 0)), mOverlapCallback);
            test(!mOverlapCallback.isOverlapping(mObject1Id));
            test(!mOverlapCallback.isOverlapping(mObject2Id));
            test(mOverlapCallback.isOverlapping(mObject3Id));
            test(mOverlapCallback.isOverlapping(mObject4Id));

            // AABB overlapping object 2
            mOverlapCallback.reset();
            mTree.reportAllShapesOverlappingWithAABB(AABB(Vector3(5, -10, -2), Vector3(7, 10, 9)), mOverlapCallback);
            test(!mOverlapCallback.isOverlapping(mObject1Id));
            test(mOverlapCallback.isOverlapping(mObject2Id));
            test(!mOverlapCallback.isOverlapping(mObject3Id));
            test(!mOverlapCallback.isOverlapping(mObject4Id));

            // ---- Update the object AABBs with the initial AABBs (no reinsertion) ----- //

            mTree.updateObject(mObject1Id, mAABB1, Vector3::zero(), false);
            mTree.updateObject(mObject2Id, mAABB2, Vector3::zero(), false);
            mTree.updateObject(mObject3Id, mAABB3, Vector3::zero(), false);
            mTree.updateObject(mObject4Id, mAABB4, Vector3::zero(), false);

            // AABB overlapping nothing
            mOverlapCallback.reset();
            mTree.reportAllShapesOverlappingWithAABB(AABB(Vector3(-10, 12, -4), Vector3(10, 50, 4)), mOverlapCallback);
            test(!mOverlapCallback.isOverlapping(mObject1Id));
            test(!mOverlapCallback.isOverlapping(mObject2Id));
            test(!mOverlapCallback.isOverlapping(mObject3Id));
            test(!mOverlapCallback.isOverlapping(mObject4Id));

            // AABB overlapping everything
            mOverlapCallback.reset();
            mTree.reportAllShapesOverlappingWithAABB(AABB(Vector3(-15, -15, -4), Vector3(15, 15, 4)), mOverlapCallback);
            test(mOverlapCallback.isOverlapping(mObject1Id));
            test(mOverlapCallback.isOverlapping(mObject2Id));
            test(mOverlapCallback.isOverlapping(mObject3Id));
            test(mOverlapCallback.isOverlapping(mObject4Id));

            // AABB overlapping object 1 and 3
            mOverlapCallback.reset();
            mTree.reportAllShapesOverlappingWithAABB(AABB(Vector3(-4, 2, -4), Vector3(-1, 7, 4)), mOverlapCallback);
            test(mOverlapCallback.isOverlapping(mObject1Id));
            test(!mOverlapCallback.isOverlapping(mObject2Id));
            test(mOverlapCallback.isOverlapping(mObject3Id));
            test(!mOverlapCallback.isOverlapping(mObject4Id));

            // AABB overlapping object 3 and 4
            mOverlapCallback.reset();
            mTree.reportAllShapesOverlappingWithAABB(AABB(Vector3(-6, -5, -2), Vector3(2, 2, 0)), mOverlapCallback);
            test(!mOverlapCallback.isOverlapping(mObject1Id));
            test(!mOverlapCallback.isOverlapping(mObject2Id));
            test(mOverlapCallback.isOverlapping(mObject3Id));
            test(mOverlapCallback.isOverlapping(mObject4Id));

            // AABB overlapping object 2
            mOverlapCallback.reset();
            mTree.reportAllShapesOverlappingWithAABB(AABB(Vector3(5, -10, -2), Vector3(7, 10, 9)), mOverlapCallback);
            test(!mOverlapCallback.isOverlapping(mObject1Id));
            test(mOverlapCallback.isOverlapping(mObject2Id));
            test(!mOverlapCallback.isOverlapping(mObject3Id));
            test(!mOverlapCallback.isOverlapping(mObject4Id));

            // ---- Update the object AABBs with the initial AABBs (with reinsertion) ----- //

            mTree.updateObject(mObject1Id, mAABB1, Vector3::zero(), true);
            mTree.updateObject(mObject2Id, mAABB2, Vector3::zero(), true);
            mTree.updateObject(mObject3Id, mAABB3, Vector3::zero(), true);
            mTree.updateObject(mObject4Id, mAABB4, Vector3::zero(), true);

            // AABB overlapping nothing
            mOverlapCallback.reset();
            mTree.reportAllShapesOverlappingWithAABB(AABB(Vector3(-10, 12, -4), Vector3(10, 50, 4)), mOverlapCallback);
            test(!mOverlapCallback.isOverlapping(mObject1Id));
            test(!mOverlapCallback.isOverlapping(mObject2Id));
            test(!mOverlapCallback.isOverlapping(mObject3Id));
            test(!mOverlapCallback.isOverlapping(mObject4Id));

            // AABB overlapping everything
            mOverlapCallback.reset();
            mTree.reportAllShapesOverlappingWithAABB(AABB(Vector3(-15, -15, -4), Vector3(15, 15, 4)), mOverlapCallback);
            test(mOverlapCallback.isOverlapping(mObject1Id));
            test(mOverlapCallback.isOverlapping(mObject2Id));
            test(mOverlapCallback.isOverlapping(mObject3Id));
            test(mOverlapCallback.isOverlapping(mObject4Id));

            // AABB overlapping object 1 and 3
            mOverlapCallback.reset();
            mTree.reportAllShapesOverlappingWithAABB(AABB(Vector3(-4, 2, -4), Vector3(-1, 7, 4)), mOverlapCallback);
            test(mOverlapCallback.isOverlapping(mObject1Id));
            test(!mOverlapCallback.isOverlapping(mObject2Id));
            test(mOverlapCallback.isOverlapping(mObject3Id));
            test(!mOverlapCallback.isOverlapping(mObject4Id));

            // AABB overlapping object 3 and 4
            mOverlapCallback.reset();
            mTree.reportAllShapesOverlappingWithAABB(AABB(Vector3(-6, -5, -2), Vector3(2, 2, 0)), mOverlapCallback);
            test(!mOverlapCallback.isOverlapping(mObject1Id));
            test(!mOverlapCallback.isOverlapping(mObject2Id));
            test(mOverlapCallback.isOverlapping(mObject3Id));
            test(mOverlapCallback.isOverlapping(mObject4Id));

            // AABB overlapping object 2
            mOverlapCallback.reset();
            mTree.reportAllShapesOverlappingWithAABB(AABB(Vector3(5, -10, -2), Vector3(7, 10, 9)), mOverlapCallback);
            test(!mOverlapCallback.isOverlapping(mObject1Id));
            test(mOverlapCallback.isOverlapping(mObject2Id));
            test(!mOverlapCallback.isOverlapping(mObject3Id));
            test(!mOverlapCallback.isOverlapping(mObject4Id));
        }

        void testRaycast() {

            // Ray with no hits
            mRaycastCallback.reset();
            Ray ray1(Vector3(4.5, -10, -5), Vector3(4.5, 10, -5));
            mTree.raycast(ray1, mRaycastCallback);
            test(!mRaycastCallback.isHit(mObject1Id));
            test(!mRaycastCallback.isHit(mObject2Id));
            test(!mRaycastCallback.isHit(mObject3Id));
            test(!mRaycastCallback.isHit(mObject4Id));

            // Ray that hits object 1
            mRaycastCallback.reset();
            Ray ray2(Vector3(-1, -20, -2), Vector3(-1, 20, -2));
            mTree.raycast(ray2, mRaycastCallback);
            test(mRaycastCallback.isHit(mObject1Id));
            test(!mRaycastCallback.isHit(mObject2Id));
            test(!mRaycastCallback.isHit(mObject3Id));
            test(!mRaycastCallback.isHit(mObject4Id));

            // Ray that hits object 1 and 2
            mRaycastCallback.reset();
            Ray ray3(Vector3(-7, 6, -2), Vector3(8, 6, -2));
            mTree.raycast(ray3, mRaycastCallback);
            test(mRaycastCallback.isHit(mObject1Id));
            test(mRaycastCallback.isHit(mObject2Id));
            test(!mRaycastCallback.isHit(mObject3Id));
            test(!mRaycastCallback.isHit(mObject4Id));

            // Ray that hits object 3
            mRaycastCallback.reset();
            Ray ray4(Vector3(-7, 2, 0), Vector3(-1, 2, 0));
            mTree.raycast(ray4, mRaycastCallback);
            test(!mRaycastCallback.isHit(mObject1Id));
            test(!mRaycastCallback.isHit(mObject2Id));
            test(mRaycastCallback.isHit(mObject3Id));
            test(!mRaycastCallback.isHit(mObject4Id));
        }
 };

}

#endif
