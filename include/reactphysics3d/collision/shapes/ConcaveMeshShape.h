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

#ifndef REACTPHYSICS3D_CONCAVE_MESH_SHAPE_H
#define REACTPHYSICS3D_CONCAVE_MESH_SHAPE_H

// Libraries
#include <reactphysics3d/collision/shapes/ConcaveShape.h>
#include <reactphysics3d/collision/broadphase/DynamicAABBTree.h>
#include <reactphysics3d/containers/Array.h>

namespace reactphysics3d {

// Declarations
class ConcaveMeshShape;
class Profiler;
class TriangleShape;
class TriangleMesh;

// class ConvexTriangleAABBOverlapCallback
/**
 * This class represents a callback when an overlap occurs
 */
class ConvexTriangleAABBOverlapCallback : public DynamicAABBTreeOverlapCallback {

    private:

        TriangleCallback& mTriangleTestCallback;

        // Reference to the concave mesh shape
        const ConcaveMeshShape& mConcaveMeshShape;

    public:

        // Constructor
        ConvexTriangleAABBOverlapCallback(TriangleCallback& triangleCallback, const ConcaveMeshShape& concaveShape)
          : mTriangleTestCallback(triangleCallback), mConcaveMeshShape(concaveShape) {

        }

        // Called when a overlapping node has been found during the call to
        // DynamicAABBTree:reportAllShapesOverlappingWithAABB()
        virtual void notifyOverlappingNode(int nodeId) override;

};

/// Class ConcaveMeshRaycastCallback
class ConcaveMeshRaycastCallback : public DynamicAABBTreeRaycastCallback {

    private :

        Array<int32> mHitAABBNodes;
        const ConcaveMeshShape& mConcaveMeshShape;
        Collider* mCollider;
        RaycastInfo& mRaycastInfo;
        const Ray& mRay;
        bool mIsHit;
        MemoryAllocator& mAllocator;

#ifdef IS_RP3D_PROFILING_ENABLED

		/// Pointer to the profiler
		Profiler* mProfiler;

#endif

    public:

        // Constructor
        ConcaveMeshRaycastCallback(const ConcaveMeshShape& concaveMeshShape,
                                   Collider* collider, RaycastInfo& raycastInfo, const Ray& ray, MemoryAllocator& allocator)
            : mHitAABBNodes(allocator), mConcaveMeshShape(concaveMeshShape), mCollider(collider),
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

#ifdef IS_RP3D_PROFILING_ENABLED

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

        /// Pointer to the triangle mesh
        TriangleMesh* mTriangleMesh;

        /// Reference to the triangle half-edge structure
        HalfEdgeStructure& mTriangleHalfEdgeStructure;

        /// Array with the scaled face normals
        Array<Vector3> mScaledVerticesNormals;

        // -------------------- Methods -------------------- //

        /// Constructor
        ConcaveMeshShape(TriangleMesh* triangleMesh, MemoryAllocator& allocator, HalfEdgeStructure& triangleHalfEdgeStructure, const Vector3& scaling = Vector3(1, 1, 1));

        /// Raycast method with feedback information
        virtual bool raycast(const Ray& ray, RaycastInfo& raycastInfo, Collider* collider, MemoryAllocator& allocator) const override;

        /// Return the number of bytes used by the collision shape
        virtual size_t getSizeInBytes() const override;

        /// Insert all the triangles into the dynamic AABB tree
        void initBVHTree();

        /// Compute the shape Id for a given triangle of the mesh
        uint32 computeTriangleShapeId(uint32 triangleIndex) const;

        /// Compute all the triangles of the mesh that are overlapping with the AABB in parameter
        virtual void computeOverlappingTriangles(const AABB& localAABB, Array<Vector3>& triangleVertices,
                                                 Array<Vector3> &triangleVerticesNormals, Array<uint32>& shapeIds,
                                                 MemoryAllocator& allocator) const override;

        /// Destructor
        virtual ~ConcaveMeshShape() override = default;

        // Return the integer data of leaf node of the dynamic AABB tree
        int32 getDynamicAABBTreeNodeDataInt(int32 nodeID) const;

        /// Compute the scaled faces normals
        void computeScaledVerticesNormals();

    public:

        /// Deleted copy-constructor
        ConcaveMeshShape(const ConcaveMeshShape& shape) = delete;

        /// Deleted assignment operator
        ConcaveMeshShape& operator=(const ConcaveMeshShape& shape) = delete;

        /// Set the scale of the shape
        virtual void setScale(const Vector3& scale) override;

        /// Return the number of vertices in the mesh
        uint32 getNbVertices() const;

        /// Return the number of triangles of the mesh
        uint32 getNbTriangles() const;

        /// Return the indices of the three vertices of a given triangle in the array
        void getTriangleVerticesIndices(uint32 triangleIndex, uint32& outV1Index, uint32& outV2Index,
                                        uint32& outV3Index) const;

        /// Return the coordinates of the three vertices of a given triangle face
        void getTriangleVertices(uint32 triangleIndex, Vector3& outV1, Vector3& outV2, Vector3& outV3) const;

        /// Return the normals of the three vertices of a given triangle face
        void getTriangleVerticesNormals(uint32 triangleIndex, Vector3& outN1,
                                        Vector3& outN2, Vector3& outN3) const;

        /// Return the coordinates of a given vertex
        const Vector3 getVertex(uint32 vertexIndex) const;

        /// Return the normal of a given vertex
        const Vector3& getVertexNormal(uint32 vertexIndex) const;

        /// Return the local bounds of the shape in x, y and z directions.
        virtual AABB getLocalBounds() const override;

        /// Return the string representation of the shape
        virtual std::string to_string() const override;

#ifdef IS_RP3D_PROFILING_ENABLED

        /// Set the profiler
        virtual void setProfiler(Profiler* profiler) override;

#endif

        // ---------- Friendship ----------- //

        friend class ConvexTriangleAABBOverlapCallback;
        friend class ConcaveMeshRaycastCallback;
        friend class PhysicsCommon;
        friend class DebugRenderer;
};

// Return the number of bytes used by the collision shape
RP3D_FORCE_INLINE size_t ConcaveMeshShape::getSizeInBytes() const {
    return sizeof(ConcaveMeshShape);
}

// Called when a overlapping node has been found during the call to
// DynamicAABBTree:reportAllShapesOverlappingWithAABB()
RP3D_FORCE_INLINE void ConvexTriangleAABBOverlapCallback::notifyOverlappingNode(int nodeId) {

    // Get the node data (triangle index and mesh subpart index)
    int32 data = mConcaveMeshShape.getDynamicAABBTreeNodeDataInt(nodeId);

    // Get the triangle vertices for this node from the concave mesh shape
    Vector3 trianglePoints[3];
    mConcaveMeshShape.getTriangleVertices(data, trianglePoints[0], trianglePoints[1], trianglePoints[2]);

    // Get the vertices normals of the triangle
    Vector3 verticesNormals[3];
    mConcaveMeshShape.getTriangleVerticesNormals(data, verticesNormals[0], verticesNormals[1], verticesNormals[2]);

    // Call the callback to test narrow-phase collision with this triangle
    mTriangleTestCallback.testTriangle(trianglePoints, verticesNormals, mConcaveMeshShape.computeTriangleShapeId(data));
}

// Compute the shape Id for a given triangle of the mesh
RP3D_FORCE_INLINE uint32 ConcaveMeshShape::computeTriangleShapeId(uint32 triangleIndex) const {

    RP3D_PROFILE("ConcaveMeshShape::computeTriangleShapeId()", mProfiler);

    return getNbTriangles() + triangleIndex;
}

}
#endif

