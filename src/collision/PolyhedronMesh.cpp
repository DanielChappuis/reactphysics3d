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
#include <reactphysics3d/collision/PolyhedronMesh.h>
#include <reactphysics3d/memory/MemoryManager.h>
#include <reactphysics3d/collision/PolygonVertexArray.h>
#include <reactphysics3d/utils/DefaultLogger.h>
#include <reactphysics3d/engine/PhysicsCommon.h>
#include <cstdlib>

using namespace reactphysics3d;


// Constructor
/**
 * Create a polyhedron mesh given an array of polygons.
 * @param polygonVertexArray Pointer to the array of polygons and their vertices
 */
PolyhedronMesh::PolyhedronMesh(PolygonVertexArray* polygonVertexArray, MemoryAllocator& allocator)
               : mMemoryAllocator(allocator), mHalfEdgeStructure(allocator, polygonVertexArray->getNbFaces(), polygonVertexArray->getNbVertices(),
                                    (polygonVertexArray->getNbFaces() + polygonVertexArray->getNbVertices() - 2) * 2), mFacesNormals(nullptr) {

   mPolygonVertexArray = polygonVertexArray;
}

// Destructor
PolyhedronMesh::~PolyhedronMesh() {

    if (mFacesNormals != nullptr) {

        for (uint32 f=0; f < mHalfEdgeStructure.getNbFaces(); f++) {
            mFacesNormals[f].~Vector3();
        }

        mMemoryAllocator.release(mFacesNormals, mHalfEdgeStructure.getNbFaces() * sizeof(Vector3));
    }
}

/// Static factory method to create a polyhedron mesh. This methods returns null_ptr if the mesh is not valid
PolyhedronMesh* PolyhedronMesh::create(PolygonVertexArray* polygonVertexArray, MemoryAllocator& polyhedronMeshAllocator, MemoryAllocator& dataAllocator) {

    PolyhedronMesh* mesh = new (polyhedronMeshAllocator.allocate(sizeof(PolyhedronMesh))) PolyhedronMesh(polygonVertexArray, dataAllocator);

    // Create the half-edge structure of the mesh
    bool isValid = mesh->createHalfEdgeStructure();

    if (isValid) {

        // Compute the faces normals
        mesh->computeFacesNormals();

        // Compute the centroid
        mesh->computeCentroid();
    }
    else {
        mesh->~PolyhedronMesh();
        polyhedronMeshAllocator.release(mesh, sizeof(PolyhedronMesh));
        mesh = nullptr;
    }

    return mesh;
}

// Create the half-edge structure of the mesh
/// This method returns true if the mesh is valid or false otherwise
bool PolyhedronMesh::createHalfEdgeStructure() {

    // For each vertex of the mesh
    for (uint32 v=0; v < mPolygonVertexArray->getNbVertices(); v++) {
        mHalfEdgeStructure.addVertex(v);
    }

    uint32 nbEdges = 0;

    // For each polygon face of the mesh
    for (uint32 f=0; f < mPolygonVertexArray->getNbFaces(); f++) {

        // Get the polygon face
        PolygonVertexArray::PolygonFace* face = mPolygonVertexArray->getPolygonFace(f);

        Array<uint32> faceVertices(mMemoryAllocator, face->nbVertices);

        // For each vertex of the face
        for (uint32 v=0; v < face->nbVertices; v++) {
            faceVertices.add(mPolygonVertexArray->getVertexIndexInFace(f, v));
        }

        nbEdges += face->nbVertices;

        assert(faceVertices.size() >= 3);

        // Addd the face into the half-edge structure
        mHalfEdgeStructure.addFace(faceVertices);
    }

    nbEdges /= 2;

    // If the mesh is valid (check Euler formula V + F - E = 2) and does not use duplicated vertices
    if (2 + nbEdges - mPolygonVertexArray->getNbFaces() != mPolygonVertexArray->getNbVertices()) {

        RP3D_LOG("PhysicsCommon", Logger::Level::Error, Logger::Category::PhysicCommon,
                 "Error when creating a PolyhedronMesh: input PolygonVertexArray is not valid. Mesh with duplicated vertices is not supported.",  __FILE__, __LINE__);

        assert(false);

        return false;
    }

    // Initialize the half-edge structure
    mHalfEdgeStructure.init();

    // Create the face normals array
    mFacesNormals = new (mMemoryAllocator.allocate(mHalfEdgeStructure.getNbFaces() * sizeof(Vector3))) Vector3[mHalfEdgeStructure.getNbFaces()];

    return true;
}

/// Return a vertex
/**
 * @param index Index of a given vertex in the mesh
 * @return The coordinates of a given vertex in the mesh
 */
Vector3 PolyhedronMesh::getVertex(uint32 index) const {
    assert(index < getNbVertices());

    // Get the vertex index in the array with all vertices
    uint32 vertexIndex = mHalfEdgeStructure.getVertex(index).vertexPointIndex;

    PolygonVertexArray::VertexDataType vertexType = mPolygonVertexArray->getVertexDataType();
    const unsigned char* verticesStart = mPolygonVertexArray->getVerticesStart();
    uint32 vertexStride = mPolygonVertexArray->getVerticesStride();

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
    const uint32 nbFaces = mHalfEdgeStructure.getNbFaces();
    for (uint32 f=0; f < nbFaces; f++) {
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

    for (uint32 v=0; v < getNbVertices(); v++) {
        mCentroid += getVertex(v);
    }

    mCentroid /= static_cast<decimal>(getNbVertices());
}

// Compute and return the area of a face
decimal PolyhedronMesh::getFaceArea(uint32 faceIndex) const {

    Vector3 sumCrossProducts(0, 0, 0);

    const HalfEdgeStructure::Face& face = mHalfEdgeStructure.getFace(faceIndex);
    assert(face.faceVertices.size() >= 3);

    Vector3 v1 = getVertex(face.faceVertices[0]);

    // For each vertex of the face
    const uint32 nbFaceVertices = static_cast<uint32>(face.faceVertices.size());
    for (uint32 i=2; i < nbFaceVertices; i++) {

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
    for (uint32 f=0; f < getNbFaces(); f++) {

        const HalfEdgeStructure::Face& face = mHalfEdgeStructure.getFace(f);
        const decimal faceArea = getFaceArea(f);
        const Vector3 faceNormal = mFacesNormals[f];
        const Vector3 faceVertex = getVertex(face.faceVertices[0]);

        sum += faceVertex.dot(faceNormal) * faceArea;
    }

    return std::abs(sum) / decimal(3.0);
}
