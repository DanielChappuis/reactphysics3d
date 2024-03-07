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
#include <reactphysics3d/collision/VertexArray.h>
#include <reactphysics3d/mathematics/Vector3.h>

using namespace reactphysics3d;

// Constructor
/// Note that your data will not be copied into the PolygonVertexArray
/**
 * @param start Pointer to the start of the vertices data
 * @param stride The number of bytes between two consecutive vertices in the array
 * @param nbVertices Number of vertices in the array
 * @param dataType Data type of the vertices data
 */
VertexArray::VertexArray(const void* start, uint32 stride, uint32 nbVertices, DataType dataType) {
    mNbVertices = nbVertices;
    mStart = reinterpret_cast<const unsigned char*>(start);
    mStride = stride;
    mDataType = dataType;
}

// Return the coordinates of a given vertex
Vector3 VertexArray::getVertex(uint32 vertexIndex) const {

    Vector3 vertex;

    if (mDataType == DataType::VERTEX_FLOAT_TYPE) {
        const float* vertices = (float*)(mStart + vertexIndex * mStride);
        vertex.x = decimal(vertices[0]);
        vertex.y = decimal(vertices[1]);
        vertex.z = decimal(vertices[2]);
    }
    else if (mDataType == DataType::VERTEX_DOUBLE_TYPE) {
        const double* vertices = (double*)(mStart + vertexIndex * mStride);
        vertex.x = decimal(vertices[0]);
        vertex.y = decimal(vertices[1]);
        vertex.z = decimal(vertices[2]);
    }
    else {
        assert(false);
    }

    return vertex;
}
