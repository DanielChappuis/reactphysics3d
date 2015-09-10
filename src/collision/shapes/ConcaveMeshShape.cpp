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

using namespace reactphysics3d;

// Constructor
ConcaveMeshShape::ConcaveMeshShape(TriangleMesh* triangleMesh) : ConcaveShape(CONCAVE_MESH) {
    mTriangleMesh = triangleMesh;
}

// Destructor
ConcaveMeshShape::~ConcaveMeshShape() {

}

// Use a callback method on all triangles of the concave shape inside a given AABB
void ConcaveMeshShape::testAllTriangles(TriangleCallback& callback, const AABB& localAABB) const {

    // For each sub-part of the mesh
    for (int i=0; i<mTriangleMesh->getNbSubparts(); i++) {

        // Get the triangle vertex array of the current sub-part
        TriangleVertexArray* triangleVertexArray = mTriangleMesh->getSubpart(i);

        TriangleVertexArray::VertexDataType vertexType = triangleVertexArray->getVertexDataType();
        TriangleVertexArray::IndexDataType indexType = triangleVertexArray->getIndexDataType();
        unsigned char* verticesStart = triangleVertexArray->getVerticesStart();
        unsigned char* indicesStart = triangleVertexArray->getIndicesStart();
        int vertexStride = triangleVertexArray->getVerticesStride();
        int indexStride = triangleVertexArray->getIndicesStride();

        // For each triangle of the concave mesh
        for (int j=0; j<triangleVertexArray->getNbTriangles(); j++) {

            Vector3 trianglePoints[3];

            // For each vertex of the triangle
            for (int k=0; k < 3; k++) {

                // Get the index of the current vertex in the triangle
                int vertexIndex;
                if (indexType == TriangleVertexArray::INDEX_INTEGER_TYPE) {
                    vertexIndex = ((unsigned int*)(indicesStart + j * indexStride))[k];
                }
                else if (indexType == TriangleVertexArray::INDEX_SHORT_TYPE) {
                    vertexIndex = ((unsigned short*)(indicesStart + j * indexStride))[k];
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

            // If the triangle AABB intersects with the convex shape AABB
            if (localAABB.testCollisionTriangleAABB(trianglePoints)) {

                // Call the callback to report this triangle
                callback.reportTriangle(trianglePoints);
            }
        }
    }
}

// Raycast method with feedback information
bool ConcaveMeshShape::raycast(const Ray& ray, RaycastInfo& raycastInfo, ProxyShape* proxyShape) const {


    // TODO : Implement this

    return false;
}
