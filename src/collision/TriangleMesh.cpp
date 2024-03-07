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

// Libraries
#include <reactphysics3d/collision/TriangleMesh.h>
#include <vector>
#include <reactphysics3d/utils/Message.h>
#include <reactphysics3d/collision/TriangleVertexArray.h>

using namespace reactphysics3d;

// Constructor
TriangleMesh::TriangleMesh(MemoryAllocator& allocator)
             : mAllocator(allocator), mVertices(allocator), mTriangles(allocator),
               mVerticesNormals(allocator), mDynamicAABBTree(allocator), mEpsilon(0) {

}

// Initialize the mesh using a TriangleVertexArray
bool TriangleMesh::init(const TriangleVertexArray& triangleVertexArray, std::vector<Message>& messages) {

    bool isValid = true;

    // Reserve memory for the vertices, faces and edges
    mVertices.reserve(triangleVertexArray.getNbVertices());
    mTriangles.reserve(triangleVertexArray.getNbTriangles() * 3);
    mVerticesNormals.reserve(triangleVertexArray.getNbVertices());

    computeEpsilon(triangleVertexArray);

    // Create the half-edge structure of the mesh
    isValid &= copyData(triangleVertexArray, messages);

    // If the normals are not provided by the user
    if (!triangleVertexArray.getHasNormals() && isValid) {

        // Compute the normals
        computeVerticesNormals();
    }

    // Insert all the triangles into the dynamic AABB tree
    initBVHTree();

    return isValid;
}

// Compute the epsilon value for this mesh
void TriangleMesh::computeEpsilon(const TriangleVertexArray& triangleVertexArray) {

    // Compute the bounds of the mesh
    Vector3 max(0, 0, 0);
    for (uint32 i=0 ; i < triangleVertexArray.getNbVertices(); i++) {

        const Vector3 vertex = triangleVertexArray.getVertex(i);

        decimal maxX = std::abs(vertex.x);
        decimal maxY = std::abs(vertex.y);
        decimal maxZ = std::abs(vertex.z);
        if (maxX > max.x) max.x = maxX;
        if (maxY > max.y) max.y = maxY;
        if (maxZ > max.z) max.z = maxZ;
    }

    // Compute the 'epsilon' value for this set of points
    mEpsilon = 3 * (max.x + max.y + max.z) * MACHINE_EPSILON;
}

// Copy the triangles faces
bool TriangleMesh::copyData(const TriangleVertexArray& triangleVertexArray, std::vector<Message>& messages) {

    bool isValid = true;

    assert(mEpsilon > 0);

    const decimal epsilonSquare = mEpsilon * mEpsilon;

    Array<bool> areUserVerticesUsed(mAllocator);
    for (uint32 i=0 ; i < triangleVertexArray.getNbVertices(); i++) {
       areUserVerticesUsed.add(false);
       mVertices.add(Vector3::zero());
       mVerticesNormals.add(Vector3::zero());
    }

    // For each face
    for (uint32 i=0 ; i < triangleVertexArray.getNbTriangles(); i++) {

        bool isValidFace = true;
        uint32 vertexIndices[3];
        Vector3 vertexNormal[3] = {Vector3::zero(), Vector3::zero(), Vector3::zero()};

        // Get the vertex indices from the user
        triangleVertexArray.getTriangleVerticesIndices(i, vertexIndices[0], vertexIndices[1], vertexIndices[2]);

        for (int v=0; v < 3; v++) {

            if (vertexIndices[v] >= triangleVertexArray.getNbVertices()) {

                // Add an error message for the user
                messages.push_back(Message("The face with index " + std::to_string(i) +
                                           " has a vertex with index " + std::to_string(vertexIndices[v]) +
                                           " but the TriangleVertexArray only has " +
                                           std::to_string(triangleVertexArray.getNbVertices()) + " vertices"));

                isValid = false;
                isValidFace = false;
            }
        }

        if (isValidFace) {

            // Check if the triangle area is not almost zero
            const Vector3 v1 = triangleVertexArray.getVertex(vertexIndices[0]);
            const Vector3 v2 = triangleVertexArray.getVertex(vertexIndices[1]);
            const Vector3 v3 = triangleVertexArray.getVertex(vertexIndices[2]);
            const Vector3 faceNormal = (v3 - v1).cross(v2 - v1);
            const bool isFaceZeroArea = faceNormal.lengthSquare() < epsilonSquare;
            if (isFaceZeroArea) {

                // Add a warning message for the user
                messages.push_back(Message("The face with index " + std::to_string(i) + " has almost zero area. This triangle will not be part of the final collision shape.",
                                           Message::Type::Warning));
            }

            // Check that edges lengths are not almost zero
            decimal edgesLengthsSquare[3];
            edgesLengthsSquare[0] = (v2 - v1).lengthSquare();
            edgesLengthsSquare[1] = (v3 - v2).lengthSquare();
            edgesLengthsSquare[2] = (v1 - v3).lengthSquare();
            bool hasFaceZeroLengthEdge = edgesLengthsSquare[0] < epsilonSquare || edgesLengthsSquare[1] < epsilonSquare ||
                                         edgesLengthsSquare[2] < epsilonSquare;
            if (hasFaceZeroLengthEdge) {

                // Add a warning message for the user
                messages.push_back(Message("The face with index " + std::to_string(i) + " has an almost zero length edge. This triangle will not be part of the final collision shape.",
                                           Message::Type::Warning));
            }

            // If the face does not have a zero area
            if (!isFaceZeroArea && !hasFaceZeroLengthEdge) {

                // If the vertices normals are provided by the user
                if (triangleVertexArray.getHasNormals()) {

                    for (int v=0; v < 3; v++) {

                        vertexNormal[v] = triangleVertexArray.getVertexNormal(vertexIndices[v]);

                        // Check that the normal is not too small
                        if (vertexNormal[v].lengthSquare() < epsilonSquare) {

                            messages.push_back(Message("The length of the provided normal for vertex with index " + std::to_string(vertexIndices[v]) + " is too small"))  ;
                            isValid = false;
                        }
                    }
                }

                // Add the vertices to the mesh
                mVerticesNormals[vertexIndices[0]] = vertexNormal[0];
                mVertices[vertexIndices[0]] = v1;
                mVerticesNormals[vertexIndices[1]] = vertexNormal[1];
                mVertices[vertexIndices[1]] = v2;
                mVerticesNormals[vertexIndices[2]] = vertexNormal[2];
                mVertices[vertexIndices[2]] = v3;

                // Add the triangle to the mesh
                mTriangles.add(vertexIndices[0]);
                mTriangles.add(vertexIndices[1]);
                mTriangles.add(vertexIndices[2]);

                areUserVerticesUsed[vertexIndices[0]] = true;
                areUserVerticesUsed[vertexIndices[1]] = true;
                areUserVerticesUsed[vertexIndices[2]] = true;

                assert(mVertices.size() == mVerticesNormals.size());
            }
        }
    }

    removeUnusedVertices(areUserVerticesUsed);

    if (mTriangles.size() == 0) {

        messages.push_back(Message("The mesh does not have any valid triangle faces"));

        isValid = false;
    }

    assert(mTriangles.size() % 3 == 0);

    return isValid;
}

// Remove the ununsed vertices (because they are not used in any triangles or
// are part of discarded triangles)
void TriangleMesh::removeUnusedVertices(Array<bool>& areUsedVertices) {

    // For each vertex of the user mesh
    for (uint32 i=mVertices.size() - 1; i > 0; i--) {

        // If the vertex is not used
        if (!areUsedVertices[i]) {

            mVertices.removeAt(i);
            mVerticesNormals.removeAt(i);

            // For each triangle index of the mesh
            for (uint32 t=0; t < mTriangles.size(); t++) {

                assert(mTriangles[t] != i);

                if (mTriangles[t] > i) {
                    mTriangles[t]--;
                }
            }
        }
    }
}

// Insert all the triangles into the dynamic AABB tree
void TriangleMesh::initBVHTree() {

    assert(mTriangles.size() % 3 == 0);

    // TODO : Try to randomly add the triangles into the tree to obtain a better tree

    // For each triangle of the mesh
    for (uint32 f=0; f < mTriangles.size() / 3; f++) {

        // Get the triangle vertices
        Vector3 trianglePoints[3];
        trianglePoints[0] = mVertices[mTriangles[f * 3]];
        trianglePoints[1] = mVertices[mTriangles[f * 3 + 1]];
        trianglePoints[2] = mVertices[mTriangles[f * 3 + 2]];

        // Create the AABB for the triangle
        AABB aabb = AABB::createAABBForTriangle(trianglePoints);

        // Add the AABB with the index of the triangle into the dynamic AABB tree
        mDynamicAABBTree.addObject(aabb, f);
    }
}

// Return the minimum bounds of the mesh in the x,y,z direction
/**
 * @return The three mimimum bounds of the mesh in the x,y,z direction
 */
const AABB& TriangleMesh::getBounds() const {
    return mDynamicAABBTree.getRootAABB();
}

// Compute the vertices normals
/// Compute the vertices normals if they are not provided by the user
/// The vertices normals are computed with weighted average of the associated
/// triangle face normal. The weights are the angle between the associated edges of neighbor triangle face.
void TriangleMesh::computeVerticesNormals() {

    // For each triangle face in the array
    for (uint32 f=0; f < mTriangles.size() / 3; f++) {

        // Get the triangle vertices
        Vector3 triangleVertices[3];
        assert(mTriangles[f * 3 + 2] < mVertices.size());
        triangleVertices[0] = mVertices[mTriangles[f * 3]];
        triangleVertices[1] = mVertices[mTriangles[f * 3 + 1]];
        triangleVertices[2] = mVertices[mTriangles[f * 3 + 2]];

        // Edges lengths
        decimal edgesLengths[3];
        edgesLengths[0] = (triangleVertices[1] - triangleVertices[0]).length();
        edgesLengths[1] = (triangleVertices[2] - triangleVertices[1]).length();
        edgesLengths[2] = (triangleVertices[0] - triangleVertices[2]).length();

        // For each vertex of the face
        for (uint32 v=0; v < 3; v++) {

            uint32 previousVertex = (v == 0) ? 2 : v-1;
            uint32 nextVertex = (v == 2) ? 0 : v+1;
            const Vector3 a = triangleVertices[nextVertex] - triangleVertices[v];
            const Vector3 b = triangleVertices[previousVertex] - triangleVertices[v];

            // Weighted normal using angle
            const decimal dotProduct = a.dot(b);
            decimal lengthATimesLengthB = (edgesLengths[previousVertex] * edgesLengths[v]);
            assert(lengthATimesLengthB * lengthATimesLengthB >= mEpsilon * mEpsilon);

            decimal cosA = dotProduct / lengthATimesLengthB;
            cosA = std::min(std::max(cosA, decimal(0.0)), decimal(1.0));    // Angle inside a triangle should be in [0, pi]
            const decimal angle = std::acos(cosA);
            assert(angle >= decimal(0.0));

            Vector3 normal = a.cross(b);
            assert(normal.lengthSquare() > mEpsilon * mEpsilon);
            normal.normalize();

            // Add the normal component of this vertex into the normals array
            mVerticesNormals[mTriangles[f * 3 + v]] += angle * normal;
        }
    }

    assert(mVertices.size() == mVerticesNormals.size());

    // Normalize the computed vertices normals
    for (uint32 v=0; v < mVertices.size(); v++) {

        assert(mVerticesNormals[v].lengthSquare() >= mEpsilon * mEpsilon);

        // Normalize the normal
        mVerticesNormals[v].normalize();
    }
}

// Report all shapes overlapping with the AABB given in parameter.
void TriangleMesh::reportAllShapesOverlappingWithAABB(const AABB& aabb, Array<int32>& overlappingNodes) {
    mDynamicAABBTree.reportAllShapesOverlappingWithAABB(aabb, overlappingNodes);
}

// Return the integer data of leaf node of the dynamic AABB tree
int32 TriangleMesh::getDynamicAABBTreeNodeDataInt(int32 nodeID) const {
   return mDynamicAABBTree.getNodeDataInt(nodeID);
}

// Ray casting method
void TriangleMesh::raycast(const Ray& ray, DynamicAABBTreeRaycastCallback& callback) const {
    mDynamicAABBTree.raycast(ray, callback);
}
