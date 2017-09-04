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
#include "TriangleVertexArray.h"
#include "mathematics/Vector3.h"
#include <cassert>

using namespace reactphysics3d;

// Constructor without vertices normals
/// Note that your data will not be copied into the TriangleVertexArray and
/// therefore, you need to make sure that those data are always valid during
/// the lifetime of the TriangleVertexArray. With this constructor, you do not
/// need to provide vertices normals for smooth mesh collision. Therefore, the
/// vertices normals will be computed automatically. The vertices normals are
/// computed with weighted average of the associated triangle face normal. The
/// weights are the angle between the associated edges of neighbor triangle face.
/**
 * @param nbVertices Number of vertices in the array
 * @param verticesStart Pointer to the first vertices of the array
 * @param verticesStride Number of bytes between the beginning of two consecutive vertices
 * @param nbTriangles Number of triangles in the array
 * @param indexesStart Pointer to the first triangle index
 * @param indexesStride Number of bytes between the beginning of two consecutive triangle indices
 * @param vertexDataType Type of data for the vertices (float, double)
 * @param indexDataType Type of data for the indices (short, int)
 */
TriangleVertexArray::TriangleVertexArray(uint nbVertices, void* verticesStart, int verticesStride,
                                         uint nbTriangles, void* indexesStart, int indexesStride,
                                         VertexDataType vertexDataType, IndexDataType indexDataType) {
    mNbVertices = nbVertices;
    mVerticesStart = reinterpret_cast<unsigned char*>(verticesStart);
    mVerticesStride = verticesStride;
    mVerticesNormalsStart = nullptr;
    mVerticesNormalsStride = 0;
    mNbTriangles = nbTriangles;
    mIndicesStart = reinterpret_cast<unsigned char*>(indexesStart);
    mIndicesStride = indexesStride;
    mVertexDataType = vertexDataType;
    mVertexNormaldDataType = NormalDataType::NORMAL_FLOAT_TYPE;
    mIndexDataType = indexDataType;
    mAreVerticesNormalsProvidedByUser = false;

    // Compute the vertices normals because they are not provided by the user
    computeVerticesNormals();
}

// Constructor with vertices normals
/// Note that your data will not be copied into the TriangleVertexArray and
/// therefore, you need to make sure that those data are always valid during
/// the lifetime of the TriangleVertexArray. With this constructor, you need
/// to provide the vertices normals that will be used for smooth mesh collision.
/**
 * @param nbVertices Number of vertices in the array
 * @param verticesStart Pointer to the first vertices of the array
 * @param verticesStride Number of bytes between the beginning of two consecutive vertices
 * @param verticesNormalsStart Pointer to the first vertex normal of the array
 * @param verticesNormalsStride Number of bytes between the beginning of two consecutive vertex normals
 * @param nbTriangles Number of triangles in the array
 * @param indexesStart Pointer to the first triangle index
 * @param indexesStride Number of bytes between the beginning of two consecutive triangle indices
 * @param vertexDataType Type of data for the vertices (float, double)
 * @param indexDataType Type of data for the indices (short, int)
 */
TriangleVertexArray::TriangleVertexArray(uint nbVertices, void* verticesStart, int verticesStride,
                                         void* verticesNormalsStart, int verticesNormalsStride,
                                         uint nbTriangles, void* indexesStart, int indexesStride,
                                         VertexDataType vertexDataType, NormalDataType normalDataType,
                                         IndexDataType indexDataType) {

    mNbVertices = nbVertices;
    mVerticesStart = reinterpret_cast<unsigned char*>(verticesStart);
    mVerticesStride = verticesStride;
    mVerticesNormalsStart = reinterpret_cast<unsigned char*>(verticesNormalsStart);
    mVerticesNormalsStride = verticesNormalsStride;
    mNbTriangles = nbTriangles;
    mIndicesStart = reinterpret_cast<unsigned char*>(indexesStart);
    mIndicesStride = indexesStride;
    mVertexDataType = vertexDataType;
    mVertexNormaldDataType = normalDataType;
    mIndexDataType = indexDataType;
    mAreVerticesNormalsProvidedByUser = true;

    assert(mVerticesNormalsStart != nullptr);
}

// Destructor
TriangleVertexArray::~TriangleVertexArray() {

    // If the vertices normals have not been provided by the user
    if (!mAreVerticesNormalsProvidedByUser) {

        // Release the allocated memory
        float* verticesNormals = reinterpret_cast<float*>(mVerticesNormalsStart);
        delete[] verticesNormals;
    }
}

// Compute the vertices normals when they are not provided by the user
/// The vertices normals are computed with weighted average of the associated
/// triangle face normal. The weights are the angle between the associated edges
/// of neighbor triangle face.
void TriangleVertexArray::computeVerticesNormals() {

    // Allocate memory for the vertices normals
    float* verticesNormals = new float[mNbVertices * 3];

    // Init vertices normals to zero
    for (uint i=0; i<mNbVertices * 3; i++) {
        verticesNormals[i] = 0.0f;
    }

    // For each triangle face in the array
    for (uint f=0; f < mNbTriangles; f++) {

        // Get the indices of the three vertices of the triangle in the array
        uint verticesIndices[3];
        getTriangleVerticesIndices(f, verticesIndices);

        // Get the triangle vertices
        Vector3 triangleVertices[3];
        getTriangleVertices(f, triangleVertices);

        // Edges lengths
        decimal edgesLengths[3];
        edgesLengths[0] = (triangleVertices[1] - triangleVertices[0]).length();
        edgesLengths[1] = (triangleVertices[2] - triangleVertices[1]).length();
        edgesLengths[2] = (triangleVertices[0] - triangleVertices[2]).length();

        // For each vertex of the face
        for (uint v=0; v < 3; v++) {

            uint previousVertex = (v == 0) ? 2 : v-1;
            uint nextVertex = (v == 2) ? 0 : v+1;
            Vector3 a = triangleVertices[nextVertex] - triangleVertices[v];
            Vector3 b = triangleVertices[previousVertex] - triangleVertices[v];

            Vector3 crossProduct = a.cross(b);
            decimal sinA = crossProduct.length() / (edgesLengths[previousVertex] * edgesLengths[v]);
            Vector3 normalComponent = std::asin(sinA) * crossProduct;

            // Add the normal component of this vertex into the normals array
            verticesNormals[verticesIndices[v] * 3] = normalComponent.x;
            verticesNormals[verticesIndices[v] * 3 + 1] = normalComponent.y;
            verticesNormals[verticesIndices[v] * 3 + 2] = normalComponent.z;
        }
    }

    // Normalize the computed vertices normals
    for (uint v=0; v<mNbVertices * 3; v += 3) {

        // Normalize the normal
        Vector3 normal(verticesNormals[v], verticesNormals[v + 1], verticesNormals[v + 2]);
        normal.normalize();

        verticesNormals[v] = normal.x;
        verticesNormals[v + 1] = normal.y;
        verticesNormals[v + 2] = normal.z;
    }

    mVerticesNormalsStart = reinterpret_cast<unsigned char*>(verticesNormals);
}

// Return the indices of the three vertices of a given triangle in the array
void TriangleVertexArray::getTriangleVerticesIndices(uint triangleIndex, uint* outVerticesIndices) const {

    assert(triangleIndex >= 0 && triangleIndex < mNbTriangles);

    void* vertexIndexPointer = (mIndicesStart + triangleIndex * 3 * mIndicesStride);

    // For each vertex of the triangle
    for (int i=0; i < 3; i++) {

        // Get the index of the current vertex in the triangle
        if (mIndexDataType == TriangleVertexArray::IndexDataType::INDEX_INTEGER_TYPE) {
            outVerticesIndices[i] = ((uint*)vertexIndexPointer)[i];
        }
        else if (mIndexDataType == TriangleVertexArray::IndexDataType::INDEX_SHORT_TYPE) {
            outVerticesIndices[i] = ((unsigned short*)vertexIndexPointer)[i];
        }
        else {
            assert(false);
        }
    }
}

// Return the vertices coordinates of a triangle
void TriangleVertexArray::getTriangleVertices(uint triangleIndex, Vector3* outTriangleVertices) const {

    assert(triangleIndex >= 0 && triangleIndex < mNbTriangles);

    void* vertexIndexPointer = (mIndicesStart + triangleIndex * 3 * mIndicesStride);

    // For each vertex of the triangle
    for (int k=0; k < 3; k++) {

        // Get the index of the current vertex in the triangle
        int vertexIndex = 0;
        if (mIndexDataType == TriangleVertexArray::IndexDataType::INDEX_INTEGER_TYPE) {
            vertexIndex = ((uint*)vertexIndexPointer)[k];
        }
        else if (mIndexDataType == TriangleVertexArray::IndexDataType::INDEX_SHORT_TYPE) {
            vertexIndex = ((unsigned short*)vertexIndexPointer)[k];
        }
        else {
            assert(false);
        }

        // Get the vertices components of the triangle
        if (mVertexDataType == TriangleVertexArray::VertexDataType::VERTEX_FLOAT_TYPE) {
            const float* vertices = (float*)(mVerticesStart + vertexIndex * mVerticesStride);
            outTriangleVertices[k][0] = decimal(vertices[0]);
            outTriangleVertices[k][1] = decimal(vertices[1]);
            outTriangleVertices[k][2] = decimal(vertices[2]);
        }
        else if (mVertexDataType == TriangleVertexArray::VertexDataType::VERTEX_DOUBLE_TYPE) {
            const double* vertices = (double*)(mVerticesStart + vertexIndex * mVerticesStride);
            outTriangleVertices[k][0] = decimal(vertices[0]);
            outTriangleVertices[k][1] = decimal(vertices[1]);
            outTriangleVertices[k][2] = decimal(vertices[2]);
        }
        else {
            assert(false);
        }
    }
}

// Return the three vertices normals of a triangle
void TriangleVertexArray::getTriangleVerticesNormals(uint triangleIndex, Vector3* outTriangleVerticesNormals) const {

    assert(triangleIndex >= 0 && triangleIndex < mNbTriangles);

    void* vertexIndexPointer = (mIndicesStart + triangleIndex * 3 * mIndicesStride);

    // For each vertex of the triangle
    for (int k=0; k < 3; k++) {

        // Get the index of the current vertex in the triangle
        int vertexIndex = 0;
        if (mIndexDataType == TriangleVertexArray::IndexDataType::INDEX_INTEGER_TYPE) {
            vertexIndex = ((uint*)vertexIndexPointer)[k];
        }
        else if (mIndexDataType == TriangleVertexArray::IndexDataType::INDEX_SHORT_TYPE) {
            vertexIndex = ((unsigned short*)vertexIndexPointer)[k];
        }
        else {
            assert(false);
        }

        // Get the normals from the array
        if (mVertexNormaldDataType == TriangleVertexArray::NormalDataType::NORMAL_FLOAT_TYPE) {
            const float* normal = (float*)(mVerticesNormalsStart + vertexIndex * mVerticesNormalsStride);
            outTriangleVerticesNormals[k][0] = decimal(normal[0]);
            outTriangleVerticesNormals[k][1] = decimal(normal[1]);
            outTriangleVerticesNormals[k][2] = decimal(normal[2]);
        }
        else if (mVertexNormaldDataType == TriangleVertexArray::NormalDataType::NORMAL_DOUBLE_TYPE) {
            const double* normal = (double*)(mVerticesNormalsStart + vertexIndex * mVerticesNormalsStride);
            outTriangleVerticesNormals[k][0] = decimal(normal[0]);
            outTriangleVerticesNormals[k][1] = decimal(normal[1]);
            outTriangleVerticesNormals[k][2] = decimal(normal[2]);
        }
        else {
            assert(false);
        }
    }
}
