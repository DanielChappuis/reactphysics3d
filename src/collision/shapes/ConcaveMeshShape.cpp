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

// Libraries
#include "ConcaveMeshShape.h"

using namespace reactphysics3d;

// Constructor
ConcaveMeshShape::ConcaveMeshShape(TriangleMesh* triangleMesh)
                 : ConcaveShape(CONCAVE_MESH) {
    mTriangleMesh = triangleMesh;
    mRaycastTestType = FRONT;

    // Insert all the triangles into the dynamic AABB tree
    initBVHTree();
}

// Destructor
ConcaveMeshShape::~ConcaveMeshShape() {

}

// Insert all the triangles into the dynamic AABB tree
void ConcaveMeshShape::initBVHTree() {

    // TODO : Try to randomly add the triangles into the tree to obtain a better tree

    // For each sub-part of the mesh
    for (uint subPart=0; subPart<mTriangleMesh->getNbSubparts(); subPart++) {

        // Get the triangle vertex array of the current sub-part
        TriangleVertexArray* triangleVertexArray = mTriangleMesh->getSubpart(subPart);

        TriangleVertexArray::VertexDataType vertexType = triangleVertexArray->getVertexDataType();
        TriangleVertexArray::IndexDataType indexType = triangleVertexArray->getIndexDataType();
        unsigned char* verticesStart = triangleVertexArray->getVerticesStart();
        unsigned char* indicesStart = triangleVertexArray->getIndicesStart();
        int vertexStride = triangleVertexArray->getVerticesStride();
        int indexStride = triangleVertexArray->getIndicesStride();

        // For each triangle of the concave mesh
        for (uint triangleIndex=0; triangleIndex<triangleVertexArray->getNbTriangles(); triangleIndex++) {

            void* vertexIndexPointer = (indicesStart + triangleIndex * 3 * indexStride);
            Vector3 trianglePoints[3];

            // For each vertex of the triangle
            for (int k=0; k < 3; k++) {

                // Get the index of the current vertex in the triangle
                int vertexIndex = 0;
                if (indexType == TriangleVertexArray::INDEX_INTEGER_TYPE) {
                    vertexIndex = ((uint*)vertexIndexPointer)[k];
                }
                else if (indexType == TriangleVertexArray::INDEX_SHORT_TYPE) {
                    vertexIndex = ((unsigned short*)vertexIndexPointer)[k];
                }
                else {
                    assert(false);
                }

                // Get the vertices components of the triangle
                if (vertexType == TriangleVertexArray::VERTEX_FLOAT_TYPE) {
                    const float* vertices = (float*)(verticesStart + vertexIndex * vertexStride);
                    trianglePoints[k][0] = decimal(vertices[0]) * mScaling.x;
                    trianglePoints[k][1] = decimal(vertices[1]) * mScaling.y;
                    trianglePoints[k][2] = decimal(vertices[2]) * mScaling.z;
                }
                else if (vertexType == TriangleVertexArray::VERTEX_DOUBLE_TYPE) {
                    const double* vertices = (double*)(verticesStart + vertexIndex * vertexStride);
                    trianglePoints[k][0] = decimal(vertices[0]) * mScaling.x;
                    trianglePoints[k][1] = decimal(vertices[1]) * mScaling.y;
                    trianglePoints[k][2] = decimal(vertices[2]) * mScaling.z;
                }
                else {
                    assert(false);
                }
            }

            // Create the AABB for the triangle
            AABB aabb = AABB::createAABBForTriangle(trianglePoints);
            aabb.inflate(mTriangleMargin, mTriangleMargin, mTriangleMargin);

            // Add the AABB with the index of the triangle into the dynamic AABB tree
            mDynamicAABBTree.addObject(aabb, subPart, triangleIndex);
        }
    }
}

// Return the three vertices coordinates (in the array outTriangleVertices) of a triangle
// given the start vertex index pointer of the triangle
void ConcaveMeshShape::getTriangleVerticesWithIndexPointer(int32 subPart, int32 triangleIndex,
                                                           Vector3* outTriangleVertices) const {

    // Get the triangle vertex array of the current sub-part
    TriangleVertexArray* triangleVertexArray = mTriangleMesh->getSubpart(subPart);

    TriangleVertexArray::VertexDataType vertexType = triangleVertexArray->getVertexDataType();
    TriangleVertexArray::IndexDataType indexType = triangleVertexArray->getIndexDataType();
    unsigned char* verticesStart = triangleVertexArray->getVerticesStart();
    unsigned char* indicesStart = triangleVertexArray->getIndicesStart();
    int vertexStride = triangleVertexArray->getVerticesStride();
    int indexStride = triangleVertexArray->getIndicesStride();

    void* vertexIndexPointer = (indicesStart + triangleIndex * 3 * indexStride);

    // For each vertex of the triangle
    for (int k=0; k < 3; k++) {

        // Get the index of the current vertex in the triangle
        int vertexIndex = 0;
        if (indexType == TriangleVertexArray::INDEX_INTEGER_TYPE) {
            vertexIndex = ((uint*)vertexIndexPointer)[k];
        }
        else if (indexType == TriangleVertexArray::INDEX_SHORT_TYPE) {
            vertexIndex = ((unsigned short*)vertexIndexPointer)[k];
        }
        else {
            assert(false);
        }

        // Get the vertices components of the triangle
        if (vertexType == TriangleVertexArray::VERTEX_FLOAT_TYPE) {
            const float* vertices = (float*)(verticesStart + vertexIndex * vertexStride);
            outTriangleVertices[k][0] = decimal(vertices[0]) * mScaling.x;
            outTriangleVertices[k][1] = decimal(vertices[1]) * mScaling.y;
            outTriangleVertices[k][2] = decimal(vertices[2]) * mScaling.z;
        }
        else if (vertexType == TriangleVertexArray::VERTEX_DOUBLE_TYPE) {
            const double* vertices = (double*)(verticesStart + vertexIndex * vertexStride);
            outTriangleVertices[k][0] = decimal(vertices[0]) * mScaling.x;
            outTriangleVertices[k][1] = decimal(vertices[1]) * mScaling.y;
            outTriangleVertices[k][2] = decimal(vertices[2]) * mScaling.z;
        }
        else {
            assert(false);
        }
    }
}

// Use a callback method on all triangles of the concave shape inside a given AABB
void ConcaveMeshShape::testAllTriangles(TriangleCallback& callback, const AABB& localAABB) const {

    ConvexTriangleAABBOverlapCallback overlapCallback(callback, *this, mDynamicAABBTree);

    // Ask the Dynamic AABB Tree to report all the triangles that are overlapping
    // with the AABB of the convex shape.
    mDynamicAABBTree.reportAllShapesOverlappingWithAABB(localAABB, overlapCallback);
}

// Raycast method with feedback information
/// Note that only the first triangle hit by the ray in the mesh will be returned, even if
/// the ray hits many triangles.
bool ConcaveMeshShape::raycast(const Ray& ray, RaycastInfo& raycastInfo, ProxyShape* proxyShape) const {

    PROFILE("ConcaveMeshShape::raycast()");

    // Create the callback object that will compute ray casting against triangles
    ConcaveMeshRaycastCallback raycastCallback(mDynamicAABBTree, *this, proxyShape, raycastInfo, ray);

    // Ask the Dynamic AABB Tree to report all AABB nodes that are hit by the ray.
    // The raycastCallback object will then compute ray casting against the triangles
    // in the hit AABBs.
    mDynamicAABBTree.raycast(ray, raycastCallback);

    raycastCallback.raycastTriangles();

    return raycastCallback.getIsHit();
}

// Collect all the AABB nodes that are hit by the ray in the Dynamic AABB Tree
decimal ConcaveMeshRaycastCallback::raycastBroadPhaseShape(int32 nodeId, const Ray& ray) {

    // Add the id of the hit AABB node into
    mHitAABBNodes.push_back(nodeId);

    return ray.maxFraction;
}

// Raycast all collision shapes that have been collected
void ConcaveMeshRaycastCallback::raycastTriangles() {

    std::vector<int>::const_iterator it;
    decimal smallestHitFraction = mRay.maxFraction;

    for (it = mHitAABBNodes.begin(); it != mHitAABBNodes.end(); ++it) {

        // Get the node data (triangle index and mesh subpart index)
        int32* data = mDynamicAABBTree.getNodeDataInt(*it);

        // Get the triangle vertices for this node from the concave mesh shape
        Vector3 trianglePoints[3];
        mConcaveMeshShape.getTriangleVerticesWithIndexPointer(data[0], data[1], trianglePoints);

        // Create a triangle collision shape
        decimal margin = mConcaveMeshShape.getTriangleMargin();
        TriangleShape triangleShape(trianglePoints[0], trianglePoints[1], trianglePoints[2], margin);
        triangleShape.setRaycastTestType(mConcaveMeshShape.getRaycastTestType());

        // Ray casting test against the collision shape
        RaycastInfo raycastInfo;
        bool isTriangleHit = triangleShape.raycast(mRay, raycastInfo, mProxyShape);

        // If the ray hit the collision shape
        if (isTriangleHit && raycastInfo.hitFraction <= smallestHitFraction) {

            assert(raycastInfo.hitFraction >= decimal(0.0));

            mRaycastInfo.body = raycastInfo.body;
            mRaycastInfo.proxyShape = raycastInfo.proxyShape;
            mRaycastInfo.hitFraction = raycastInfo.hitFraction;
            mRaycastInfo.worldPoint = raycastInfo.worldPoint;
            mRaycastInfo.worldNormal = raycastInfo.worldNormal;
            mRaycastInfo.meshSubpart = data[0];
            mRaycastInfo.triangleIndex = data[1];

            smallestHitFraction = raycastInfo.hitFraction;
            mIsHit = true;
        }
    }
}
