/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2016 Daniel Chappuis                                       *
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

#ifndef REACTPHYSICS3D_CONCAVE_MESH_SHAPE_H
#define REACTPHYSICS3D_CONCAVE_MESH_SHAPE_H

// Libraries
#include "ConcaveShape.h"
#include "collision/broadphase/DynamicAABBTree.h"
#include "collision/TriangleMesh.h"
#include "collision/shapes/TriangleShape.h"
#include "engine/Profiler.h"

namespace reactphysics3d {

class ConcaveMeshShape;

// class ConvexTriangleAABBOverlapCallback
class ConvexTriangleAABBOverlapCallback : public DynamicAABBTreeOverlapCallback {

    private:

        TriangleCallback& mTriangleTestCallback;

        // Reference to the concave mesh shape
        const ConcaveMeshShape& mConcaveMeshShape;

        // Reference to the Dynamic AABB tree
        const DynamicAABBTree& mDynamicAABBTree;

    public:

        // Constructor
        ConvexTriangleAABBOverlapCallback(TriangleCallback& triangleCallback, const ConcaveMeshShape& concaveShape,
                                          const DynamicAABBTree& dynamicAABBTree)
          : mTriangleTestCallback(triangleCallback), mConcaveMeshShape(concaveShape), mDynamicAABBTree(dynamicAABBTree) {

        }

        // Called when a overlapping node has been found during the call to
        // DynamicAABBTree:reportAllShapesOverlappingWithAABB()
        virtual void notifyOverlappingNode(int nodeId);

};

/// Class ConcaveMeshRaycastCallback
class ConcaveMeshRaycastCallback : public DynamicAABBTreeRaycastCallback {

    private :

        std::vector<int32> mHitAABBNodes;
        const DynamicAABBTree& mDynamicAABBTree;
        const ConcaveMeshShape& mConcaveMeshShape;
        ProxyShape* mProxyShape;
        RaycastInfo& mRaycastInfo;
        const Ray& mRay;
        bool mIsHit;

    public:

        // Constructor
        ConcaveMeshRaycastCallback(const DynamicAABBTree& dynamicAABBTree, const ConcaveMeshShape& concaveMeshShape,
                                   ProxyShape* proxyShape, RaycastInfo& raycastInfo, const Ray& ray)
            : mDynamicAABBTree(dynamicAABBTree), mConcaveMeshShape(concaveMeshShape), mProxyShape(proxyShape),
              mRaycastInfo(raycastInfo), mRay(ray), mIsHit(false) {

        }

        /// Collect all the AABB nodes that are hit by the ray in the Dynamic AABB Tree
        virtual decimal raycastBroadPhaseShape(int32 nodeId, const Ray& ray);

        /// Raycast all collision shapes that have been collected
        void raycastTriangles();

        /// Return true if a raycast hit has been found
        bool getIsHit() const {
            return mIsHit;
        }
};

// Class ConcaveMeshShape
/**
 * This class represents a static concave mesh shape. Note that collision detection
 * with a concave mesh shape can be very expensive. You should use only use
 * this shape for a static mesh.
 */
class ConcaveMeshShape : public ConcaveShape {

    protected:

        // -------------------- Attributes -------------------- //

        /// Triangle mesh
        TriangleMesh* mTriangleMesh;

        /// Dynamic AABB tree to accelerate collision with the triangles
        DynamicAABBTree mDynamicAABBTree;

        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        ConcaveMeshShape(const ConcaveMeshShape& shape);

        /// Private assignment operator
        ConcaveMeshShape& operator=(const ConcaveMeshShape& shape);

        /// Raycast method with feedback information
        virtual bool raycast(const Ray& ray, RaycastInfo& raycastInfo, ProxyShape* proxyShape) const;

        /// Return the number of bytes used by the collision shape
        virtual size_t getSizeInBytes() const;

        /// Insert all the triangles into the dynamic AABB tree
        void initBVHTree();

        /// Return the three vertices coordinates (in the array outTriangleVertices) of a triangle
        /// given the start vertex index pointer of the triangle.
        void getTriangleVerticesWithIndexPointer(int32 subPart, int32 triangleIndex,
                                                 Vector3* outTriangleVertices) const;

    public:

        /// Constructor
        ConcaveMeshShape(TriangleMesh* triangleMesh);

        /// Destructor
        ~ConcaveMeshShape();

        /// Return the local bounds of the shape in x, y and z directions.
        virtual void getLocalBounds(Vector3& min, Vector3& max) const;

        /// Set the local scaling vector of the collision shape
        virtual void setLocalScaling(const Vector3& scaling);

        /// Return the local inertia tensor of the collision shape
        virtual void computeLocalInertiaTensor(Matrix3x3& tensor, decimal mass) const;

        /// Use a callback method on all triangles of the concave shape inside a given AABB
        virtual void testAllTriangles(TriangleCallback& callback, const AABB& localAABB) const;

        // ---------- Friendship ----------- //

        friend class ConvexTriangleAABBOverlapCallback;
        friend class ConcaveMeshRaycastCallback;
};

// Return the number of bytes used by the collision shape
inline size_t ConcaveMeshShape::getSizeInBytes() const {
    return sizeof(ConcaveMeshShape);
}

// Return the local bounds of the shape in x, y and z directions.
// This method is used to compute the AABB of the box
/**
 * @param min The minimum bounds of the shape in local-space coordinates
 * @param max The maximum bounds of the shape in local-space coordinates
 */
inline void ConcaveMeshShape::getLocalBounds(Vector3& min, Vector3& max) const {

    // Get the AABB of the whole tree
    AABB treeAABB = mDynamicAABBTree.getRootAABB();

    min = treeAABB.getMin();
    max = treeAABB.getMax();
}

// Set the local scaling vector of the collision shape
inline void ConcaveMeshShape::setLocalScaling(const Vector3& scaling) {

    CollisionShape::setLocalScaling(scaling);

    // Reset the Dynamic AABB Tree
    mDynamicAABBTree.reset();

    // Rebuild Dynamic AABB Tree here
    initBVHTree();
}

// Return the local inertia tensor of the shape
/**
 * @param[out] tensor The 3x3 inertia tensor matrix of the shape in local-space
 *                    coordinates
 * @param mass Mass to use to compute the inertia tensor of the collision shape
 */
inline void ConcaveMeshShape::computeLocalInertiaTensor(Matrix3x3& tensor, decimal mass) const {

    // Default inertia tensor
    // Note that this is not very realistic for a concave triangle mesh.
    // However, in most cases, it will only be used static bodies and therefore,
    // the inertia tensor is not used.
    tensor.setAllValues(mass, 0, 0,
                        0, mass, 0,
                        0, 0, mass);
}

// Called when a overlapping node has been found during the call to
// DynamicAABBTree:reportAllShapesOverlappingWithAABB()
inline void ConvexTriangleAABBOverlapCallback::notifyOverlappingNode(int nodeId) {

    // Get the node data (triangle index and mesh subpart index)
    int32* data = mDynamicAABBTree.getNodeDataInt(nodeId);

    // Get the triangle vertices for this node from the concave mesh shape
    Vector3 trianglePoints[3];
    mConcaveMeshShape.getTriangleVerticesWithIndexPointer(data[0], data[1], trianglePoints);

    // Call the callback to test narrow-phase collision with this triangle
    mTriangleTestCallback.testTriangle(trianglePoints);
}

}
#endif

