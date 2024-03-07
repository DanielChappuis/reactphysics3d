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

#ifndef REACTPHYSICS3D_TRIANGLE_MESH_H
#define REACTPHYSICS3D_TRIANGLE_MESH_H

// Libraries
#include <cassert>
#include <reactphysics3d/containers/Array.h>
#include <reactphysics3d/memory/MemoryAllocator.h>
#include <reactphysics3d/collision/broadphase/DynamicAABBTree.h>
#include <reactphysics3d/containers/Map.h>

namespace reactphysics3d {

// Declarations
class TriangleVertexArray;
struct Message;

// Class TriangleMesh
/**
 * This class represents a mesh made of triangles.
 * A single TriangleMesh object can be used to create one or many ConcaveMeshShape (with
 * different scaling for instance).
 */
class TriangleMesh {

    protected:

        /// Reference to the memory allocator
        MemoryAllocator& mAllocator;

        /// All the vertices of the mesh
        Array<Vector3> mVertices;

        /// The three vertices indices for each triangle face of the mesh
        Array<uint32> mTriangles;

        /// The normal vector at each vertex of the mesh
        Array<Vector3> mVerticesNormals;

        /// Dynamic AABB tree to accelerate collision with the triangles
        DynamicAABBTree mDynamicAABBTree;

        /// Epsilon value for this mesh
        decimal mEpsilon;

        /// Constructor
        TriangleMesh(reactphysics3d::MemoryAllocator& allocator);

        /// Copy the vertices into the mesh
        bool copyVertices(const TriangleVertexArray& triangleVertexArray, std::vector<Message>& messages);

        /// Copy or compute the vertices normals
        void computeVerticesNormals();

        /// Compute the epsilon value for this mesh
        void computeEpsilon(const TriangleVertexArray& triangleVertexArray);

        /// Copy the triangles into the mesh
        bool copyData(const TriangleVertexArray& triangleVertexArray, std::vector<Message>& errors);

        /// Insert all the triangles into the dynamic AABB tree
        void initBVHTree();

        /// Initialize the mesh using a TriangleVertexArray
        bool init(const TriangleVertexArray& triangleVertexArray, std::vector<Message>& messages);

        /// Report all shapes overlapping with the AABB given in parameter.
        void reportAllShapesOverlappingWithAABB(const AABB& aabb, Array<int32>& overlappingNodes);

        /// Remove the ununsed vertices (because they are not used in any triangles or are part of discarded triangles)
        void removeUnusedVertices(Array<bool>& areUsedVertices);

        /// Return the integer data of leaf node of the dynamic AABB tree
        int32 getDynamicAABBTreeNodeDataInt(int32 nodeID) const;

        /// Ray casting method
        void raycast(const Ray& ray, DynamicAABBTreeRaycastCallback& callback) const;

    public:

        /// Return the number of vertices in the mesh
        uint32 getNbVertices() const;

        /// Return the number of triangles faces of the mesh
        uint32 getNbTriangles() const;

        /// Return the bounds of the mesh in the x,y,z direction
        const AABB& getBounds() const;

        /// Return the three vertex indices of a given triangle face
        void getTriangleVerticesIndices(uint32 triangleIndex, uint32& outV1Index, uint32& outV2Index,
                                      uint32& outV3Index) const;

        /// Return the coordinates of the three vertices of a given triangle face
        void getTriangleVertices(uint32 triangleIndex, Vector3& outV1, Vector3& outV2, Vector3& outV3) const;

        /// Return the normals of the three vertices of a given triangle face
        void getTriangleVerticesNormals(uint32 triangleIndex, Vector3& outN1,
                                        Vector3& outN2, Vector3& outN3) const;

        /// Return the coordinates of a given vertex
        const Vector3& getVertex(uint32 vertexIndex) const;

        /// Return the normal of a given vertex
        const Vector3& getVertexNormal(uint32 vertexIndex) const;

#ifdef IS_RP3D_PROFILING_ENABLED

        /// Set the profiler
        void setProfiler(Profiler* profiler);

#endif

        // ---------- Friendship ---------- //

        friend class PhysicsCommon;
        friend class ConcaveMeshShape;
};

// Return the number of vertices in the mesh
RP3D_FORCE_INLINE uint32 TriangleMesh::getNbVertices() const {
    return mVertices.size();
}

// Return the number of triangles faces of the mesh
RP3D_FORCE_INLINE uint32 TriangleMesh::getNbTriangles() const {
    return mTriangles.size() / 3;
}

// Return the three vertex indices of a given triangle face
RP3D_FORCE_INLINE void TriangleMesh::getTriangleVerticesIndices(uint32 triangleIndex, uint32& outV1Index,
                                                                uint32& outV2Index, uint32& outV3Index) const {
   assert(triangleIndex < mTriangles.size() / 3);

   outV1Index = mTriangles[triangleIndex * 3];
   outV2Index = mTriangles[triangleIndex * 3 + 1];
   outV3Index = mTriangles[triangleIndex * 3 + 2];
}

// Return the coordinates of the three vertices of a given triangle face
RP3D_FORCE_INLINE void TriangleMesh::getTriangleVertices(uint32 triangleIndex, Vector3& outV1, Vector3& outV2,
                                                         Vector3& outV3) const {
    assert(triangleIndex < mTriangles.size() / 3);

    outV1 = mVertices[mTriangles[triangleIndex * 3]];
    outV2 = mVertices[mTriangles[triangleIndex * 3 + 1]];
    outV3 = mVertices[mTriangles[triangleIndex * 3 + 2]];
}

// Return the normals of the three vertices of a given triangle face
RP3D_FORCE_INLINE void TriangleMesh::getTriangleVerticesNormals(uint32 triangleIndex, Vector3& outN1,
                                                                Vector3& outN2, Vector3& outN3) const {
    assert(triangleIndex < mTriangles.size() / 3);

    outN1 = mVerticesNormals[mTriangles[triangleIndex * 3]];
    outN2 = mVerticesNormals[mTriangles[triangleIndex * 3 + 1]];
    outN3 = mVerticesNormals[mTriangles[triangleIndex * 3 + 2]];
}

// Return the coordinates of a given vertex
RP3D_FORCE_INLINE const Vector3& TriangleMesh::getVertex(uint32 vertexIndex) const {
    assert(vertexIndex < mVertices.size());
    return mVertices[vertexIndex];
}

// Return the normal of a given vertex
RP3D_FORCE_INLINE const Vector3& TriangleMesh::getVertexNormal(uint32 vertexIndex) const {
    assert(vertexIndex < mVertices.size());
   return mVerticesNormals[vertexIndex];
}

#ifdef IS_RP3D_PROFILING_ENABLED

// Set the profiler
RP3D_FORCE_INLINE void TriangleMesh::setProfiler(Profiler* profiler) {
    mDynamicAABBTree.setProfiler(profiler);
}

#endif
}

#endif

