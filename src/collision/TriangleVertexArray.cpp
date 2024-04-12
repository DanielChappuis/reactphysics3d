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
#include <reactphysics3d/collision/TriangleVertexArray.h>
#include <reactphysics3d/mathematics/Vector3.h>
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
 * @param indexesStride Number of bytes between the beginning of the three indices of two triangles
 * @param vertexDataType Type of data for the vertices (float, double)
 * @param indexDataType Type of data for the indices (short, int)
 */
TriangleVertexArray::TriangleVertexArray(uint32 nbVertices, const void* verticesStart, uint32 verticesStride,
                                         uint32 nbTriangles, const void* indexesStart, uint32 indexesStride,
                                         VertexDataType vertexDataType, IndexDataType indexDataType) {
    mNbVertices = nbVertices;
    mVerticesStart = static_cast<const uchar*>(verticesStart);
    mVerticesStride = verticesStride;
    mVerticesNormalsStart = nullptr;
    mVerticesNormalsStride = 3 * sizeof(float);
    mNbTriangles = nbTriangles;
    mIndicesStart = static_cast<const uchar*>(indexesStart);
    mIndicesStride = indexesStride;
    mVertexDataType = vertexDataType;
    mVertexNormaldDataType = NormalDataType::NORMAL_FLOAT_TYPE;
    mIndexDataType = indexDataType;
    mHasNormals = false;
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
 * @param normalDataType Type of data for the normals (float, double)
 * @param indexDataType Type of data for the indices (short, int)
 */
TriangleVertexArray::TriangleVertexArray(uint32 nbVertices, const void* verticesStart, uint32 verticesStride,
                                         const void* verticesNormalsStart, uint32 verticesNormalsStride,
                                         uint32 nbTriangles, const void* indexesStart, uint32 indexesStride,
                                         VertexDataType vertexDataType, NormalDataType normalDataType,
                                         IndexDataType indexDataType) {

    mNbVertices = nbVertices;
    mVerticesStart = static_cast<const uchar*>(verticesStart);
    mVerticesStride = verticesStride;
    mVerticesNormalsStart = static_cast<const uchar*>(verticesNormalsStart);
    mVerticesNormalsStride = verticesNormalsStride;
    mNbTriangles = nbTriangles;
    mIndicesStart = static_cast<const uchar*>(indexesStart);
    mIndicesStride = indexesStride;
    mVertexDataType = vertexDataType;
    mVertexNormaldDataType = normalDataType;
    mIndexDataType = indexDataType;
    mHasNormals = true;

    assert(mVerticesNormalsStart != nullptr);
}

// Return the indices of the three vertices of a given triangle in the array
/**
 * @param triangleIndex Index of a given triangle in the array
 * @param[out] outV1Index Index of the first vertex of the triangle in the vertex array
 * @param[out] outV2Index Index of the first vertex of the triangle in the vertex array
 * @param[out] outV3Index Index of the first vertex of the triangle in the vertex array
 */
void TriangleVertexArray::getTriangleVerticesIndices(uint32 triangleIndex, uint32& outV1Index, uint32& outV2Index, uint32& outV3Index) const {

    assert(triangleIndex < mNbTriangles);

    const uchar* triangleIndicesPointer = mIndicesStart + triangleIndex * mIndicesStride;
    const void* startTriangleIndices = static_cast<const void*>(triangleIndicesPointer);

    // Get the index of the current vertex in the triangle
    if (mIndexDataType == TriangleVertexArray::IndexDataType::INDEX_INTEGER_TYPE) {
        outV1Index = static_cast<const uint*>(startTriangleIndices)[0];
        outV2Index = static_cast<const uint*>(startTriangleIndices)[1];
        outV3Index = static_cast<const uint*>(startTriangleIndices)[2];
    }
    else if (mIndexDataType == TriangleVertexArray::IndexDataType::INDEX_SHORT_TYPE) {
        outV1Index = static_cast<const ushort*>(startTriangleIndices)[0];
        outV2Index = static_cast<const ushort*>(startTriangleIndices)[1];
        outV3Index = static_cast<const ushort*>(startTriangleIndices)[2];
    }
    else {
        assert(false);
    }
}

// Return a vertex of the array
/**
 * @param vertexIndex Index of a given vertex of the array
 * @return The vertex coordinates
 */
Vector3 TriangleVertexArray::getVertex(uint32 vertexIndex) const {

    assert(vertexIndex < mNbVertices);

    const uchar* vertexPointerChar = mVerticesStart + vertexIndex * mVerticesStride;
    const void* vertexPointer = static_cast<const void*>(vertexPointerChar);

    // Get the vertices components of the triangle
    if (mVertexDataType == TriangleVertexArray::VertexDataType::VERTEX_FLOAT_TYPE) {

        const float* vertices = static_cast<const float*>(vertexPointer);
        return Vector3(decimal(vertices[0]), decimal(vertices[1]), decimal(vertices[2]));
    }
    else if (mVertexDataType == TriangleVertexArray::VertexDataType::VERTEX_DOUBLE_TYPE) {
        const double* vertices = static_cast<const double*>(vertexPointer);
        return Vector3(decimal(vertices[0]), decimal(vertices[1]), decimal(vertices[2]));
    }
    else {
        assert(false);
    }
}

// Return a vertex normal of the array
/**
 * @param vertexIndex Index of a given vertex of the array
 * @return The normal vector of the vertex
 */
Vector3 TriangleVertexArray::getVertexNormal(uint32 vertexIndex) const {

    assert(vertexIndex < mNbVertices);

    const uchar* vertexNormalPointerChar = mVerticesNormalsStart + vertexIndex * mVerticesNormalsStride;
    const void* vertexNormalPointer = static_cast<const void*>(vertexNormalPointerChar);

    // Get the normals from the array
    if (mVertexNormaldDataType == TriangleVertexArray::NormalDataType::NORMAL_FLOAT_TYPE) {
        const float* normal = static_cast<const float*>(vertexNormalPointer);
        return Vector3(decimal(normal[0]), decimal(normal[1]), decimal(normal[2]));
    }
    else if (mVertexNormaldDataType == TriangleVertexArray::NormalDataType::NORMAL_DOUBLE_TYPE) {
        const double* normal = static_cast<const double*>(vertexNormalPointer);
        return Vector3(decimal(normal[0]), decimal(normal[1]), decimal(normal[2]));
    }
    else {
        assert(false);
    }

    return Vector3::zero();
}
