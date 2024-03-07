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

#ifndef REACTPHYSICS3D_VERTEX_ARRAY_H
#define REACTPHYSICS3D_VERTEX_ARRAY_H

// Libraries
#include <reactphysics3d/configuration.h>
#include <cassert>

namespace reactphysics3d {

// Declarations
struct Vector3;

// Class VertexArray
/**
 * This class is used to describe an array of vertices.
 */
class VertexArray {

    public:

        /// Data type for the vertices in the array
        enum class DataType {VERTEX_FLOAT_TYPE, VERTEX_DOUBLE_TYPE};

    protected:

        /// Number of vertices in the array
        uint32 mNbVertices;

        /// Pointer to the first vertex value in the array
        const unsigned char* mStart;

        /// Stride (number of bytes) between the beginning of two vertices values in the array
        uint32 mStride;

        /// Data type of the vertices in the array
        DataType mDataType;

    public:

        /// Constructor
        VertexArray(const void* start, uint32 stride, uint32 nbVertices, DataType dataType);

        // TODO : Add constructor with a reference to a vector of Vector3

        /// Return the vertex data type
        DataType getDataType() const;

        /// Return the number of vertices
        uint32 getNbVertices() const;

        /// Return the vertices stride (number of bytes)
        uint32 getStride() const;

        /// Return the coordinates of a given vertex
        Vector3 getVertex(uint32 index) const;

        /// Return the pointer to the start of the vertices array
        const unsigned char* getStart() const;

        // -------------------- Friendship -------------------- //

        friend class PhysicsCommon;
};

// Return the vertex data type
/**
 * @return The data type of the vertices in the array
 */
RP3D_FORCE_INLINE VertexArray::DataType VertexArray::getDataType() const {
    return mDataType;
}

// Return the number of vertices
/**
 * @return The number of vertices in the array
 */
RP3D_FORCE_INLINE uint32 VertexArray::getNbVertices() const {
    return mNbVertices;
}

// Return the vertices stride (number of bytes)
/**
 * @return The number of bytes between two vertices
 */
RP3D_FORCE_INLINE uint32 VertexArray::getStride() const {
    return mStride;
}

// Return the pointer to the start of the vertices array
/**
 * @return A pointer to the start of the vertex array of the mesh
 */
RP3D_FORCE_INLINE const unsigned char* VertexArray::getStart() const {
    return mStart;
}

}

#endif

