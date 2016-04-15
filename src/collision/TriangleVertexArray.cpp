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

using namespace reactphysics3d;

// Constructor
/// Note that your data will not be copied into the TriangleVertexArray and
/// therefore, you need to make sure that those data are always valid during
/// the lifetime of the TriangleVertexArray.
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
    mNbTriangles = nbTriangles;
    mIndicesStart = reinterpret_cast<unsigned char*>(indexesStart);
    mIndicesStride = indexesStride;
    mVertexDataType = vertexDataType;
    mIndexDataType = indexDataType;
}

// Destructor
TriangleVertexArray::~TriangleVertexArray() {

}
