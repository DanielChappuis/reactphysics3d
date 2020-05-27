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

#ifndef REACTPHYSICS3D_POLYGON_VERTEX_ARRAY_H
#define REACTPHYSICS3D_POLYGON_VERTEX_ARRAY_H

// Libraries
#include <reactphysics3d/configuration.h>
#include <cassert>

namespace reactphysics3d {

// Class PolygonVertexArray
/**
 * This class is used to describe the vertices and faces of a polyhedron mesh.
 * A PolygonVertexArray represents an array of vertices and polygon faces
 * of a polyhedron mesh. When you create a PolygonVertexArray, no data is copied
 * into the array. It only stores pointer to the data. The purpose is to allow
 * the user to share vertices data between the physics engine and the rendering
 * part. Therefore, make sure that the data pointed by a PolygonVertexArray
 * remains valid during the PolygonVertexArray life.
 */
class PolygonVertexArray {

    public:

        /// Data type for the vertices in the array
        enum class VertexDataType {VERTEX_FLOAT_TYPE, VERTEX_DOUBLE_TYPE};

        /// Data type for the indices in the array
        enum class IndexDataType {INDEX_INTEGER_TYPE, INDEX_SHORT_TYPE};

        /// Represent a polygon face of the polyhedron
        struct PolygonFace {

            /// Number of vertices in the polygon face
            uint nbVertices;

            /// Index of the first vertex of the polygon face
            /// inside the array with all vertex indices
            uint indexBase;
        };

    protected:

        /// Number of vertices in the array
        uint mNbVertices;

        /// Pointer to the first vertex value in the array
        const unsigned char* mVerticesStart;

        /// Stride (number of bytes) between the beginning of two vertices
        /// values in the array
        int mVerticesStride;

        /// Pointer to the first vertex index of the array
        const unsigned char* mIndicesStart;

        /// Stride (number of bytes) between the beginning of two indices in
        /// the array
        int mIndicesStride;

        /// Number of polygon faces in the array
        uint mNbFaces;

        /// Pointer to the first polygon face in the polyhedron
        PolygonFace* mPolygonFacesStart;

        /// Data type of the vertices in the array
        VertexDataType mVertexDataType;

        /// Data type of the indices in the array
        IndexDataType mIndexDataType;

    public:

        /// Constructor
        PolygonVertexArray(uint nbVertices, const void* verticesStart, int verticesStride,
                           const void* indexesStart, int indexesStride,
                           uint nbFaces, PolygonFace* facesStart,
                           VertexDataType vertexDataType, IndexDataType indexDataType);

        /// Destructor
        ~PolygonVertexArray() = default;

        /// Return the vertex data type
        VertexDataType getVertexDataType() const;

        /// Return the index data type
        IndexDataType getIndexDataType() const;

        /// Return the number of vertices
        uint getNbVertices() const;

        /// Return the number of faces
        uint getNbFaces() const;

        /// Return the vertices stride (number of bytes)
        int getVerticesStride() const;

        /// Return the indices stride (number of bytes)
        int getIndicesStride() const;

        /// Return the vertex index of a given vertex in a face
        uint getVertexIndexInFace(uint faceIndex, uint noVertexInFace) const;

        /// Return a polygon face of the polyhedron
        PolygonFace* getPolygonFace(uint faceIndex) const;

        /// Return the pointer to the start of the vertices array
        const unsigned char* getVerticesStart() const;

        /// Return the pointer to the start of the indices array
        const unsigned char* getIndicesStart() const;
};

// Return the vertex data type
/**
 * @return The data type of the vertices in the array
 */
inline PolygonVertexArray::VertexDataType PolygonVertexArray::getVertexDataType() const {
    return mVertexDataType;
}

// Return the index data type
/**
 * @return The data type of the indices in the array
 */
inline PolygonVertexArray::IndexDataType PolygonVertexArray::getIndexDataType() const {
   return mIndexDataType;
}

// Return the number of vertices
/**
 * @return The number of vertices in the array
 */
inline uint PolygonVertexArray::getNbVertices() const {
    return mNbVertices;
}

// Return the number of faces
/**
 * @return The number of faces in the array
 */
inline uint PolygonVertexArray::getNbFaces() const {
    return mNbFaces;
}

// Return the vertices stride (number of bytes)
/**
 * @return The number of bytes between two vertices
 */
inline int PolygonVertexArray::getVerticesStride() const {
    return mVerticesStride;
}

// Return the indices stride (number of bytes)
/**
 * @return The number of bytes between two consecutive face indices
 */
inline int PolygonVertexArray::getIndicesStride() const {
    return mIndicesStride;
}

// Return a polygon face of the polyhedron
/**
 * @param faceIndex Index of a given face
 * @return A polygon face
 */
inline PolygonVertexArray::PolygonFace* PolygonVertexArray::getPolygonFace(uint faceIndex) const {
    assert(faceIndex < mNbFaces);
    return &mPolygonFacesStart[faceIndex];
}

// Return the pointer to the start of the vertices array
/**
 * @return A pointer to the start of the vertex array of the polyhedron
 */
inline const unsigned char* PolygonVertexArray::getVerticesStart() const {
    return mVerticesStart;
}

// Return the pointer to the start of the indices array
/**
 * @return A pointer to the start of the face indices array of the polyhedron
 */
inline const unsigned char* PolygonVertexArray::getIndicesStart() const {
    return mIndicesStart;
}

}

#endif

