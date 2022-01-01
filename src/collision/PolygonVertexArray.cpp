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
#include <reactphysics3d/collision/PolygonVertexArray.h>

using namespace reactphysics3d;

// Constructor
/// Note that your data will not be copied into the PolygonVertexArray and
/// therefore, you need to make sure that those data are always valid during
/// the lifetime of the PolygonVertexArray.
/**
 * @param nbVertices Number of vertices in the array
 * @param verticesStart Pointer to the start of the vertices data
 * @param verticesStride The number of bytes between two consecutive vertices in the array
 * @param indexesStart Pointer to the start of the face indices data
 * @param indexesStride The number of bytes between two consecutive face indices in the array
 * @param nbFaces The number of faces in the array
 * @param nbFaces Pointer to the start of the faces data
 * @param vertexDataType Data type of the vertices data
 * @param indexDataType Data type of the face indices data
 */
PolygonVertexArray::PolygonVertexArray(uint32 nbVertices, const void* verticesStart, uint32 verticesStride,
                                       const void* indexesStart, uint32 indexesStride,
                                       uint32 nbFaces, PolygonFace* facesStart,
                                       VertexDataType vertexDataType, IndexDataType indexDataType) {
    mNbVertices = nbVertices;
    mVerticesStart = reinterpret_cast<const unsigned char*>(verticesStart);
    mVerticesStride = verticesStride;
    mIndicesStart = reinterpret_cast<const unsigned char*>(indexesStart);
    mIndicesStride = indexesStride;
    mNbFaces = nbFaces;
    mPolygonFacesStart = facesStart;
    mVertexDataType = vertexDataType;
    mIndexDataType = indexDataType;
}

// Return the vertex index of a given vertex in a face
/**
 * @return The index of a vertex (in the array of vertices data) of a given vertex in a face
 */
uint32 PolygonVertexArray::getVertexIndexInFace(uint32 faceIndex32, uint32 noVertexInFace) const {

    assert(faceIndex32 < mNbFaces);

    // Get the face
    PolygonFace* face = getPolygonFace(faceIndex32);

    assert(noVertexInFace < face->nbVertices);

    const void* vertexIndexPointer = mIndicesStart + (face->indexBase + noVertexInFace) * mIndicesStride;

    if (mIndexDataType == PolygonVertexArray::IndexDataType::INDEX_INTEGER_TYPE) {
        return *((uint*)vertexIndexPointer);
    }
    else if (mIndexDataType == PolygonVertexArray::IndexDataType::INDEX_SHORT_TYPE) {
        return *((unsigned short*)vertexIndexPointer);
    }
    else {
        assert(false);
    }

    return 0;
}
