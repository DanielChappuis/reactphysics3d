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
#include "ConcaveMeshShape.h"
#include <iostream>

using namespace reactphysics3d;

// Constructor
ConcaveMeshShape::ConcaveMeshShape(TriangleMesh* triangleMesh) : ConcaveShape(CONCAVE_MESH) {
    mTriangleMesh = triangleMesh;

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
    for (int subPart=0; subPart<mTriangleMesh->getNbSubparts(); subPart++) {

        // Get the triangle vertex array of the current sub-part
        TriangleVertexArray* triangleVertexArray = mTriangleMesh->getSubpart(subPart);

        TriangleVertexArray::VertexDataType vertexType = triangleVertexArray->getVertexDataType();
        TriangleVertexArray::IndexDataType indexType = triangleVertexArray->getIndexDataType();
        unsigned char* verticesStart = triangleVertexArray->getVerticesStart();
        unsigned char* indicesStart = triangleVertexArray->getIndicesStart();
        int vertexStride = triangleVertexArray->getVerticesStride();
        int indexStride = triangleVertexArray->getIndicesStride();

        // For each triangle of the concave mesh
        for (int triangleIndex=0; triangleIndex<triangleVertexArray->getNbTriangles(); triangleIndex++) {

            void* vertexIndexPointer = (indicesStart + triangleIndex * 3 * indexStride);
            Vector3 trianglePoints[3];

            // For each vertex of the triangle
            for (int k=0; k < 3; k++) {

                // Get the index of the current vertex in the triangle
                int vertexIndex;
                if (indexType == TriangleVertexArray::INDEX_INTEGER_TYPE) {
                    vertexIndex = ((uint*)vertexIndexPointer)[k];
                }
                else if (indexType == TriangleVertexArray::INDEX_SHORT_TYPE) {
                    vertexIndex = ((unsigned short*)vertexIndexPointer)[k];
                }

                // Get the vertices components of the triangle
                if (vertexType == TriangleVertexArray::VERTEX_FLOAT_TYPE) {
                    const float* vertices = (float*)(verticesStart + vertexIndex * vertexStride);
                    trianglePoints[k][0] = decimal(vertices[0]);
                    trianglePoints[k][1] = decimal(vertices[1]);
                    trianglePoints[k][2] = decimal(vertices[2]);
                }
                else if (vertexType == TriangleVertexArray::VERTEX_DOUBLE_TYPE) {
                    const double* vertices = (double*)(verticesStart + vertexIndex * vertexStride);
                    trianglePoints[k][0] = decimal(vertices[0]);
                    trianglePoints[k][1] = decimal(vertices[1]);
                    trianglePoints[k][2] = decimal(vertices[2]);
                }
            }

            // Create the AABB for the triangle
            AABB aabb = AABB::createAABBForTriangle(trianglePoints);

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
        int vertexIndex;
        if (indexType == TriangleVertexArray::INDEX_INTEGER_TYPE) {
            vertexIndex = ((uint*)vertexIndexPointer)[k];
        }
        else if (indexType == TriangleVertexArray::INDEX_SHORT_TYPE) {
            vertexIndex = ((unsigned short*)vertexIndexPointer)[k];
        }

        // Get the vertices components of the triangle
        if (vertexType == TriangleVertexArray::VERTEX_FLOAT_TYPE) {
            const float* vertices = (float*)(verticesStart + vertexIndex * vertexStride);
            outTriangleVertices[k][0] = decimal(vertices[0]);
            outTriangleVertices[k][1] = decimal(vertices[1]);
            outTriangleVertices[k][2] = decimal(vertices[2]);
        }
        else if (vertexType == TriangleVertexArray::VERTEX_DOUBLE_TYPE) {
            const double* vertices = (double*)(verticesStart + vertexIndex * vertexStride);
            outTriangleVertices[k][0] = decimal(vertices[0]);
            outTriangleVertices[k][1] = decimal(vertices[1]);
            outTriangleVertices[k][2] = decimal(vertices[2]);
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
bool ConcaveMeshShape::raycast(const Ray& ray, RaycastInfo& raycastInfo, ProxyShape* proxyShape) const {


    // TODO : Implement this

    return false;
}
