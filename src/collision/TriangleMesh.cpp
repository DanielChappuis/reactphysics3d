/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2022 Daniel Chappuis                                       *
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
#include <reactphysics3d/utils/Error.h>
#include <reactphysics3d/collision/TriangleVertexArray.h>

using namespace reactphysics3d;

// Constructor
TriangleMesh::TriangleMesh(MemoryAllocator& allocator)
             : mVertices(allocator), mTriangles(allocator), mVerticesNormals(allocator),
               mDynamicAABBTree(allocator) {

}

// Initialize the mesh using a TriangleVertexArray
bool TriangleMesh::init(const TriangleVertexArray& triangleVertexArray, std::vector<Error>& errors) {

    bool isValid = true;

    // Reserve memory for the vertices, faces and edges
    mVertices.reserve(triangleVertexArray.getNbVertices());
    mTriangles.reserve(triangleVertexArray.getNbTriangles() * 3);
    mVerticesNormals.reserve(triangleVertexArray.getNbVertices());

    // Copy the vertices from the PolygonVertexArray into the mesh
    isValid &= copyVertices(triangleVertexArray, errors);

    // Create the half-edge structure of the mesh
    isValid &= copyTriangles(triangleVertexArray, errors);

    // Compute the vertices normals
    isValid &= copyOrComputeVerticesNormals(triangleVertexArray, errors);

    // Insert all the triangles into the dynamic AABB tree
    initBVHTree();

    return isValid;
}

// Copy the vertices into the mesh
bool TriangleMesh::copyVertices(const TriangleVertexArray& triangleVertexArray, std::vector<Error>& errors) {

    bool isValid = true;

    // For each vertex
    for (uint32 i=0 ; i < triangleVertexArray.getNbVertices(); i++) {

        const Vector3 vertex = triangleVertexArray.getVertex(i);
        mVertices.add(vertex);
    }

    if (mVertices.size() == 0) {

        errors.push_back(Error("The mesh does not have any vertices"));

        isValid = false;
    }

    return isValid;
}

// Copy the triangles faces
bool TriangleMesh::copyTriangles(const TriangleVertexArray& triangleVertexArray, std::vector<Error>& errors) {

    bool isValid = true;

    // For each face
    for (uint32 i=0 ; i < triangleVertexArray.getNbTriangles(); i++) {

        uint32 v1Index, v2Index, v3Index;
        triangleVertexArray.getTriangleVerticesIndices(i, v1Index, v2Index, v3Index);

        mTriangles.add(v1Index);
        mTriangles.add(v2Index);
        mTriangles.add(v3Index);
    }

    if (mTriangles.size() == 0) {

        errors.push_back(Error("The mesh does not have any triangle faces"));

        isValid = false;
    }

    assert(mTriangles.size() % 3 == 0);

    return isValid;
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

// Copy or compute the vertices normals
/// If the normals are provided by the user in the TriangleVertexArray we copy them into the TriangleMesh.
/// Otherwise, we compute them.
/// The vertices normals are computed with weighted average of the associated
/// triangle face normal. The weights are the angle between the associated edges of neighbor triangle face.
bool TriangleMesh::copyOrComputeVerticesNormals(const TriangleVertexArray& triangleVertexArray,
                                                std::vector<Error>& errors) {

    bool isValid = true;

    // If the normals are provided by the user
    if (triangleVertexArray.getHasNormals()) {

        // For each vertex of the mesh
        for (uint32 i=0; i < mVertices.size(); i++) {

            Vector3 normal = triangleVertexArray.getVertexNormal(i);

            // Check that the normal is not too small
            if (normal.length() < MACHINE_EPSILON) {

                errors.push_back(Error("The normal at vertex index " + std::to_string(i) + " is too small"))  ;
                isValid = false;
            }

            mVerticesNormals.add(normal);
        }

        return isValid;
    }

    // The normals are not provided, we compute them

    // Init vertices normals to zero
    for (uint32 i=0; i < mVertices.size(); i++) {
        mVerticesNormals.add(Vector3(0, 0, 0));
    }

    // For each triangle face in the array
    for (uint32 f=0; f < mTriangles.size() / 3; f++) {

        // Get the triangle vertices
        Vector3 triangleVertices[3];
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
            Vector3 a = triangleVertices[nextVertex] - triangleVertices[v];
            Vector3 b = triangleVertices[previousVertex] - triangleVertices[v];

            const Vector3 crossProduct = a.cross(b);
            decimal edgeLengths = (edgesLengths[previousVertex] * edgesLengths[v]);
            if (std::abs(edgeLengths) > decimal(MACHINE_EPSILON)) {

                decimal sinA = crossProduct.length() / edgeLengths;
                sinA = std::min(std::max(sinA, decimal(0.0)), decimal(1.0));
                decimal arcSinA = std::asin(sinA);
                assert(arcSinA >= decimal(0.0));
                Vector3 normalComponent = arcSinA * crossProduct;

                // Add the normal component of this vertex into the normals array
                mVerticesNormals[mTriangles[f * 3 + v]] += normalComponent;
            }
        }
    }

    // Normalize the computed vertices normals
    for (uint32 v=0; v < mVertices.size(); v++) {

        const decimal length = mVerticesNormals[v].length();

        if (length < MACHINE_EPSILON) {
            errors.push_back(Error("The normal at vertex index " + std::to_string(v) + " is too small"))  ;
            isValid = false;
        }
        else {

            // Normalize the normal
            mVerticesNormals[v] /= length;
        }
    }

    return isValid;
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
