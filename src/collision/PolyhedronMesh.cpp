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
#include "PolyhedronMesh.h"

using namespace reactphysics3d;


// Constructor
PolyhedronMesh::PolyhedronMesh(PolygonVertexArray* polygonVertexArray) {

   mPolygonVertexArray = polygonVertexArray;

   // Create the half-edge structure of the mesh
   createHalfEdgeStructure();
}

// Create the half-edge structure of the mesh
void PolyhedronMesh::createHalfEdgeStructure() {

    // For each vertex of the mesh
    for (uint v=0; v < mPolygonVertexArray->getNbVertices(); v++) {
        mHalfEdgeStructure.addVertex(v);
    }

    // For each polygon face of the mesh
    for (uint f=0; f < mPolygonVertexArray->getNbFaces(); f++) {

        // Get the polygon face
        PolygonVertexArray::PolygonFace* face = mPolygonVertexArray->getPolygonFace(f);

        std::vector<uint> faceVertices;

        // For each vertex of the face
        for (uint v=0; v < face->nbVertices; v++) {
            faceVertices.push_back(mPolygonVertexArray->getVertexIndexInFace(f, v));
        }

        // Addd the face into the half-edge structure
        mHalfEdgeStructure.addFace(faceVertices);
    }

    // Initialize the half-edge structure
    mHalfEdgeStructure.init();
}

/// Return a vertex
Vector3 PolyhedronMesh::getVertex(uint index) const {
    assert(index < getNbVertices());

    // Get the vertex index in the array with all vertices
    uint vertexIndex = mHalfEdgeStructure.getVertex(index).vertexPointIndex;

    PolygonVertexArray::VertexDataType vertexType = mPolygonVertexArray->getVertexDataType();
    unsigned char* verticesStart = mPolygonVertexArray->getVerticesStart();
    int vertexStride = mPolygonVertexArray->getVerticesStride();

    Vector3 vertex;
    if (vertexType == PolygonVertexArray::VertexDataType::VERTEX_FLOAT_TYPE) {
        const float* vertices = (float*)(verticesStart + vertexIndex * vertexStride);
        vertex.x = decimal(vertices[0]);
        vertex.y = decimal(vertices[1]);
        vertex.z = decimal(vertices[2]);
    }
    else if (vertexType == PolygonVertexArray::VertexDataType::VERTEX_DOUBLE_TYPE) {
        const double* vertices = (double*)(verticesStart + vertexIndex * vertexStride);
        vertex.x = decimal(vertices[0]);
        vertex.y = decimal(vertices[1]);
        vertex.z = decimal(vertices[2]);
    }
    else {
        assert(false);
    }

    return vertex;
}
