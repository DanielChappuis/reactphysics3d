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

#ifndef REACTPHYSICS3D_CONCAVE_MESH_SHAPE_H
#define REACTPHYSICS3D_CONCAVE_MESH_SHAPE_H

// Libraries
#include "ConcaveShape.h"
#include "collision/broadphase/DynamicAABBTree.h"
#include "containers/List.h"

namespace reactphysics3d {

// Declarations
class ConcaveMeshShape;
class Profiler;
class TriangleShape;
class TriangleMesh;

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
        virtual void notifyOverlappingNode(int nodeId) override;

};

/// Class ConcaveMeshRaycastCallback
class ConcaveMeshRaycastCallback : public DynamicAABBTreeRaycastCallback {

    private :

        List<int32> mHitAABBNodes;
        const DynamicAABBTree& mDynamicAABBTree;
        const ConcaveMeshShape& mConcaveMeshShape;
        ProxyShape* mProxyShape;
        RaycastInfo& mRaycastInfo;
        const Ray& mRay;
        bool mIsHit;
        MemoryAllocator& mAllocator;

#ifdef IS_PROFILING_ACTIVE

		/// Pointer to the profiler
		Profiler* mProfiler;

#endif

    public:

        // Constructor
        ConcaveMeshRaycastCallback(const DynamicAABBTree& dynamicAABBTree, const ConcaveMeshShape& concaveMeshShape,
                                   ProxyShape* proxyShape, RaycastInfo& raycastInfo, const Ray& ray, MemoryAllocator& allocator)
            : mHitAABBNodes(allocator), mDynamicAABBTree(dynamicAABBTree), mConcaveMeshShape(concaveMeshShape), mProxyShape(proxyShape),
              mRaycastInfo(raycastInfo), mRay(ray), mIsHit(false), mAllocator(allocator) {

        }

        /// Collect all the AABB nodes that are hit by the ray in the Dynamic AABB Tree
        virtual decimal raycastBroadPhaseShape(int32 nodeId, const Ray& ray) override;

        /// Raycast all collision shapes that have been collected
        void raycastTriangles();

        /// Return true if a raycast hit has been found
        bool getIsHit() const {
            return mIsHit;
        }

#ifdef IS_PROFILING_ACTIVE

		/// Set the profiler
		void setProfiler(Profiler* profiler) {
			mProfiler = profiler;
		}

#endif
};

// Class ConcaveMeshShape
/**
 * This class represents a static concave mesh shape. Note that collision detection
 * with a concave mesh shape can be very expensive. You should only use
 * this shape for a static mesh.
 */
class ConcaveMeshShape : public ConcaveShape {

    protected:

        // -------------------- Attributes -------------------- //

        /// Triangle mesh
        TriangleMesh* mTriangleMesh;

        /// Dynamic AABB tree to accelerate collision with the triangles
        DynamicAABBTree mDynamicAABBTree;

        /// Array with computed vertices normals for each TriangleVertexArray of the triangle mesh (only
        /// if the user did not provide its own vertices normals)
        Vector3** mComputedVerticesNormals;

        /// Scaling
        const Vector3 mScaling;

        // -------------------- Methods -------------------- //

        /// Raycast method with feedback information
        virtual bool raycast(const Ray& ray, RaycastInfo& raycastInfo, ProxyShape* proxyShape, MemoryAllocator& allocator) const override;

        /// Return the number of bytes used by the collision shape
        virtual size_t getSizeInBytes() const override;

        /// Insert all the triangles into the dynamic AABB tree
        void initBVHTree();

        /// Compute the shape Id for a given triangle of the mesh
        uint computeTriangleShapeId(uint subPart, uint triangleIndex) const;

    public:

        /// Constructor
        ConcaveMeshShape(TriangleMesh* triangleMesh, const Vector3& scaling = Vector3(1, 1, 1));

        /// Destructor
        virtual ~ConcaveMeshShape() override = default;

        /// Deleted copy-constructor
        ConcaveMeshShape(const ConcaveMeshShape& shape) = delete;

        /// Deleted assignment operator
        ConcaveMeshShape& operator=(const ConcaveMeshShape& shape) = delete;

        /// Return the scaling vector
        const Vector3& getScaling() const;
		
        /// Return the number of sub parts contained in this mesh
		uint getNbSubparts() const;
		
        /// Return the number of triangles in a sub part of the mesh
		uint getNbTriangles(uint subPart) const;

        /// Return the indices of the three vertices of a given triangle in the array
        void getTriangleVerticesIndices(uint subPart, uint triangleIndex, uint* outVerticesIndices) const;

        /// Return the three vertices coordinates (in the array outTriangleVertices) of a triangle
        void getTriangleVertices(uint subPart, uint triangleIndex, Vector3* outTriangleVertices) const;

        /// Return the three vertex normals (in the array outVerticesNormals) of a triangle
        void getTriangleVerticesNormals(uint subPart, uint triangleIndex, Vector3* outVerticesNormals) const;

        /// Return the local bounds of the shape in x, y and z directions.
        virtual void getLocalBounds(Vector3& min, Vector3& max) const override;

        /// Return the local inertia tensor of the collision shape
        virtual void computeLocalInertiaTensor(Matrix3x3& tensor, decimal mass) const override;

        /// Use a callback method on all triangles of the concave shape inside a given AABB
        virtual void testAllTriangles(TriangleCallback& callback, const AABB& localAABB) const override;

        /// Return the string representation of the shape
        virtual std::string to_string() const override;

#ifdef IS_PROFILING_ACTIVE

        /// Set the profiler
        virtual void setProfiler(Profiler* profiler) override;

#endif

        // ---------- Friendship ----------- //

        friend class ConvexTriangleAABBOverlapCallback;
        friend class ConcaveMeshRaycastCallback;
};

// Return the number of bytes used by the collision shape
inline size_t ConcaveMeshShape::getSizeInBytes() const {
    return sizeof(ConcaveMeshShape);
}

// Return the scaling vector
inline const Vector3& ConcaveMeshShape::getScaling() const {
    return mScaling;
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
    mConcaveMeshShape.getTriangleVertices(data[0], data[1], trianglePoints);

    // Get the vertices normals of the triangle
    Vector3 verticesNormals[3];
    mConcaveMeshShape.getTriangleVerticesNormals(data[0], data[1], verticesNormals);

    // Call the callback to test narrow-phase collision with this triangle
    mTriangleTestCallback.testTriangle(trianglePoints, verticesNormals, mConcaveMeshShape.computeTriangleShapeId(data[0], data[1]));
}

#ifdef IS_PROFILING_ACTIVE

// Set the profiler
inline void ConcaveMeshShape::setProfiler(Profiler* profiler) {

    CollisionShape::setProfiler(profiler);

    mDynamicAABBTree.setProfiler(profiler);
}


#endif

}
#endif

