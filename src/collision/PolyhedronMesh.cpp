/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2020 Daniel Chappuis                                       *
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
#include <reactphysics3d/collision/PolyhedronMesh.h>
#include <reactphysics3d/memory/MemoryManager.h>
#include <reactphysics3d/collision/PolygonVertexArray.h>
#include <cstdlib>

using namespace reactphysics3d;


// Constructor
/**
 * Create a polyhedron mesh given an array of polygons.
 * @param polygonVertexArray Pointer to the array of polygons and their vertices
 */
PolyhedronMesh::PolyhedronMesh(PolygonVertexArray* polygonVertexArray, MemoryAllocator &allocator)
               : mMemoryAllocator(allocator), mHalfEdgeStructure(allocator, polygonVertexArray->getNbFaces(), polygonVertexArray->getNbVertices(),
                                    (polygonVertexArray->getNbFaces() + polygonVertexArray->getNbVertices() - 2) * 2) {

   mPolygonVertexArray = polygonVertexArray;

   // Create the half-edge structure of the mesh
   createHalfEdgeStructure();

   // Create the face normals array
   mFacesNormals = new Vector3[mHalfEdgeStructure.getNbFaces()];

   // Compute the faces normals
   computeFacesNormals();

   // Compute the centroid
   computeCentroid();
}

// Destructor
PolyhedronMesh::~PolyhedronMesh() {
    delete[] mFacesNormals;
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

        List<uint> faceVertices(mMemoryAllocator, face->nbVertices);

        // For each vertex of the face
        for (uint v=0; v < face->nbVertices; v++) {
            faceVertices.add(mPolygonVertexArray->getVertexIndexInFace(f, v));
        }

        assert(faceVertices.size() >= 3);

        // Addd the face into the half-edge structure
        mHalfEdgeStructure.addFace(faceVertices);
    }

    // Initialize the half-edge structure
    mHalfEdgeStructure.init();
}

/// Return a vertex
/**
 * @param index Index of a given vertex in the mesh
 * @return The coordinates of a given vertex in the mesh
 */
Vector3 PolyhedronMesh::getVertex(uint index) const {
    assert(index < getNbVertices());

    // Get the vertex index in the array with all vertices
    uint vertexIndex = mHalfEdgeStructure.getVertex(index).vertexPointIndex;

    PolygonVertexArray::VertexDataType vertexType = mPolygonVertexArray->getVertexDataType();
    const unsigned char* verticesStart = mPolygonVertexArray->getVerticesStart();
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

// Compute the faces normals
void PolyhedronMesh::computeFacesNormals() {

    // For each face
    for (uint f=0; f < mHalfEdgeStructure.getNbFaces(); f++) {
        const HalfEdgeStructure::Face& face = mHalfEdgeStructure.getFace(f);

        assert(face.faceVertices.size() >= 3);

        const Vector3 vec1 = getVertex(face.faceVertices[1]) - getVertex(face.faceVertices[0]);
        const Vector3 vec2 = getVertex(face.faceVertices[2]) - getVertex(face.faceVertices[0]);
        mFacesNormals[f] = vec1.cross(vec2);
        mFacesNormals[f].normalize();
    }
}

// Compute the centroid of the polyhedron
void PolyhedronMesh::computeCentroid() {

    mCentroid.setToZero();

    for (uint v=0; v < getNbVertices(); v++) {
        mCentroid += getVertex(v);
    }

    mCentroid /= getNbVertices();
}

// Compute and return the area of a face
decimal PolyhedronMesh::getFaceArea(uint faceIndex) const {

    Vector3 sumCrossProducts(0, 0, 0);

    const HalfEdgeStructure::Face& face = mHalfEdgeStructure.getFace(faceIndex);
    assert(face.faceVertices.size() >= 3);

    Vector3 v1 = getVertex(face.faceVertices[0]);

    // For each vertex of the face
    for (uint i=2; i < face.faceVertices.size(); i++) {

        const Vector3 v2 = getVertex(face.faceVertices[i-1]);
        const Vector3 v3 = getVertex(face.faceVertices[i]);

        const Vector3 v1v2 = v2 - v1;
        const Vector3 v1v3 = v3 - v1;

        sumCrossProducts +=  v1v2.cross(v1v3);
    }

    return decimal(0.5) * sumCrossProducts.length();
}

// Compute and return the volume of the polyhedron
/// We use the divergence theorem to compute the volume of the polyhedron using a sum over its faces.
decimal PolyhedronMesh::getVolume() const {

    decimal sum = 0.0;

    // For each face of the polyhedron
    for (uint f=0; f < getNbFaces(); f++) {

        const HalfEdgeStructure::Face& face = mHalfEdgeStructure.getFace(f);
        const decimal faceArea = getFaceArea(f);
        const Vector3 faceNormal = mFacesNormals[f];
        const Vector3 faceVertex = getVertex(face.faceVertices[0]);

        sum += faceVertex.dot(faceNormal) * faceArea;
    }

    return std::abs(sum) / decimal(3.0);
}
