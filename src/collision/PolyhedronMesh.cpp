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
#include "PolyhedronMesh.h"

using namespace reactphysics3d;


// Constructor
PolyhedronMesh::PolyhedronMesh() : mIsFinalized(false) {

}

// Add a vertex into the polyhedron.
// This method returns the index of the vertex that you need to use
// to add faces.
uint PolyhedronMesh::addVertex(const Vector3& vertex) {
    mVertices.push_back(vertex);
    return mVertices.size() - 1;
}

// Add a face into the polyhedron.
// A face is a list of vertices indices (returned by addVertex() method).
// The order of the indices are important. You need to specify the vertices of
// of the face sorted counter-clockwise as seen from the outside of the polyhedron.
void PolyhedronMesh::addFace(std::vector<uint> faceVertices) {
    mFaces.push_back(faceVertices);
}

// Call this method when you are done adding vertices and faces
void PolyhedronMesh::finalize() {

    if (mIsFinalized) return;

    // Initialize the half-edge structure
    mHalfEdgeStructure.init(mVertices, mFaces);

    mIsFinalized = true;
}
