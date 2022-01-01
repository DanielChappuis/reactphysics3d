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

#ifndef REACTPHYSICS3D_TRIANGLE_VERTEX_ARRAY_H
#define REACTPHYSICS3D_TRIANGLE_VERTEX_ARRAY_H

// Libraries
#include <reactphysics3d/configuration.h>

namespace reactphysics3d {

// Declarations
struct Vector3;

// Class TriangleVertexArray
/**
 * This class is used to describe the vertices and faces of a triangular mesh.
 * A TriangleVertexArray represents a continuous array of vertices and indexes
 * of a triangular mesh. When you create a TriangleVertexArray, no data is copied
 * into the array. It only stores pointer to the data. The purpose is to allow
 * the user to share vertices data between the physics engine and the rendering
 * part. Therefore, make sure that the data pointed by a TriangleVertexArray
 * remains valid during the TriangleVertexArray life.
 */
class TriangleVertexArray {

    public:

        /// Data type for the vertices in the array
        enum class VertexDataType {VERTEX_FLOAT_TYPE, VERTEX_DOUBLE_TYPE};

        /// Data type for the vertex normals in the array
        enum class NormalDataType {NORMAL_FLOAT_TYPE, NORMAL_DOUBLE_TYPE};

        /// Data type for the indices in the array
        enum class IndexDataType {INDEX_INTEGER_TYPE, INDEX_SHORT_TYPE};

    protected:

        // -------------------- Attributes -------------------- //

        /// Number of vertices in the array
        uint32 mNbVertices;

        /// Pointer to the first vertex value in the array
        const uchar* mVerticesStart;

        /// Stride (number of bytes) between the beginning of two vertices
        /// values in the array
        uint32 mVerticesStride;

        /// Pointer to the first vertex normal value in the array
        const uchar* mVerticesNormalsStart;

        /// Stride (number of bytes) between the beginning of two vertex normals
        /// values in the array
        uint32 mVerticesNormalsStride;

        /// Number of triangles in the array
        uint32 mNbTriangles;

        /// Pointer to the first vertex index of the array
        const uchar* mIndicesStart;

        /// Stride (number of bytes) between the beginning of the three indices of two triangles
        uint32 mIndicesStride;

        /// Data type of the vertices in the array
        VertexDataType mVertexDataType;

        /// Data type of the vertex normals in the array
        NormalDataType mVertexNormaldDataType;

        /// Data type of the indices in the array
        IndexDataType mIndexDataType;

        /// True if the vertices normals are provided by the user
        bool mAreVerticesNormalsProvidedByUser;

        // -------------------- Methods -------------------- //

        /// Compute the vertices normals when they are not provided by the user
        void computeVerticesNormals();

    public:

        // -------------------- Methods -------------------- //

        /// Constructor without vertices normals
        TriangleVertexArray(uint32 nbVertices, const void* verticesStart, uint32 verticesStride,
                            uint32 nbTriangles, const void* indexesStart, uint32 indexesStride,
                            VertexDataType vertexDataType, IndexDataType indexDataType);

        /// Constructor with vertices normals
        TriangleVertexArray(uint32 nbVertices, const void* verticesStart, uint32 verticesStride,
                            const void* verticesNormalsStart, uint32 uverticesNormalsStride,
                            uint32 nbTriangles, const void* indexesStart, uint32 indexesStride,
                            VertexDataType vertexDataType, NormalDataType normalDataType,
                            IndexDataType indexDataType);

        /// Destructor
        ~TriangleVertexArray();

        /// Deleted assignment operator
        TriangleVertexArray& operator=(const TriangleVertexArray& triangleVertexArray) = delete;

        /// Deleted copy-constructor
        TriangleVertexArray(const TriangleVertexArray& triangleVertexArray) = delete;

        /// Return the vertex data type
        VertexDataType getVertexDataType() const;

        /// Return the vertex normal data type
        NormalDataType getVertexNormalDataType() const;

        /// Return the index data type
        IndexDataType getIndexDataType() const;

        /// Return the number of vertices
        uint32 getNbVertices() const;

        /// Return the number of triangles
        uint32 getNbTriangles() const;

        /// Return the vertices stride (number of bytes)
        uint32 getVerticesStride() const;

        /// Return the vertex normals stride (number of bytes)
        uint32 getVerticesNormalsStride() const;

        /// Return the indices stride (number of bytes)
        uint32 getIndicesStride() const;

        /// Return the pointer to the start of the vertices array
        const void* getVerticesStart() const;

        /// Return the pointer to the start of the vertex normals array
        const void* getVerticesNormalsStart() const;

        /// Return the pointer to the start of the indices array
        const void* getIndicesStart() const;

        /// Return the vertices coordinates of a triangle
        void getTriangleVertices(uint32 triangleIndex, Vector3* outTriangleVertices) const;

        /// Return the three vertices normals of a triangle
        void getTriangleVerticesNormals(uint32 triangleIndex, Vector3* outTriangleVerticesNormals) const;

        /// Return the indices of the three vertices of a given triangle in the array
        void getTriangleVerticesIndices(uint32 triangleIndex, uint32* outVerticesIndices) const;

        /// Return a vertex of the array
        void getVertex(uint32 vertexIndex, Vector3* outVertex);

        /// Return a vertex normal of the array
        void getNormal(uint32 vertexIndex, Vector3* outNormal);
};

// Return the vertex data type
/**
 * @return The data type of the vertices in the array
 */
RP3D_FORCE_INLINE TriangleVertexArray::VertexDataType TriangleVertexArray::getVertexDataType() const {
    return mVertexDataType;
}

// Return the vertex normal data type
/**
 * @return The data type of the normals in the array
 */
RP3D_FORCE_INLINE TriangleVertexArray::NormalDataType TriangleVertexArray::getVertexNormalDataType() const {
    return mVertexNormaldDataType;
}

// Return the index data type
/**
 * @return The data type of the face indices in the array
 */
RP3D_FORCE_INLINE TriangleVertexArray::IndexDataType TriangleVertexArray::getIndexDataType() const {
   return mIndexDataType;
}

// Return the number of vertices
/**
 * @return The number of vertices in the array
 */
RP3D_FORCE_INLINE uint32 TriangleVertexArray::getNbVertices() const {
    return mNbVertices;
}

// Return the number of triangles
/**
 * @return The number of triangles in the array
 */
RP3D_FORCE_INLINE uint32 TriangleVertexArray::getNbTriangles() const {
    return mNbTriangles;
}

// Return the vertices stride (number of bytes)
/**
 * @return The number of bytes separating two consecutive vertices in the array
 */
RP3D_FORCE_INLINE uint32 TriangleVertexArray::getVerticesStride() const {
    return mVerticesStride;
}

// Return the vertex normals stride (number of bytes)
/**
 * @return The number of bytes separating two consecutive normals in the array
 */
RP3D_FORCE_INLINE uint32 TriangleVertexArray::getVerticesNormalsStride() const {
    return mVerticesNormalsStride;
}

// Return the indices stride (number of bytes)
/**
 * @return The number of bytes separating two consecutive face indices in the array
 */
RP3D_FORCE_INLINE uint32 TriangleVertexArray::getIndicesStride() const {
    return mIndicesStride;
}

// Return the pointer to the start of the vertices array
/**
 * @return A pointer to the start of the vertices data in the array
 */
RP3D_FORCE_INLINE const void* TriangleVertexArray::getVerticesStart() const {
    return mVerticesStart;
}

// Return the pointer to the start of the vertex normals array
/**
 * @return A pointer to the start of the normals data in the array
 */
RP3D_FORCE_INLINE const void* TriangleVertexArray::getVerticesNormalsStart() const {
    return mVerticesNormalsStart;
}

// Return the pointer to the start of the indices array
/**
 * @return A pointer to the start of the face indices data in the array
 */
RP3D_FORCE_INLINE const void* TriangleVertexArray::getIndicesStart() const {
    return mIndicesStart;
}

}

#endif

