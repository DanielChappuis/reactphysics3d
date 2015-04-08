/********************************************************************************
* OpenGL-Framework                                                              *
* Copyright (c) 2013 Daniel Chappuis                                            *
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
#include "Mesh.h"

// Namespaces
using namespace openglframework;
using namespace std;

// Constructor
Mesh::Mesh() {

}

// Destructor
Mesh::~Mesh() {

}

// Destroy the mesh
void Mesh::destroy() {

    mVertices.clear();
    mNormals.clear();
    mTangents.clear();
    mIndices.clear();
    mColors.clear();
    mUVs.clear();
    mTextures.clear();
}

// Compute the normals of the mesh
void Mesh::calculateNormals() {

    mNormals = vector<Vector3>(getNbVertices(), Vector3(0, 0, 0));

    // For each triangular face
    for (uint i=0; i<getNbFaces(); i++) {

        // Get the three vertices index of the current face
        uint v1 = getVertexIndexInFace(i, 0);
        uint v2 = getVertexIndexInFace(i, 1);
        uint v3 = getVertexIndexInFace(i, 2);

        assert(v1 < getNbVertices());
        assert(v2 < getNbVertices());
        assert(v3 < getNbVertices());

        // Compute the normal of the face
        Vector3 p = getVertex(v1);
        Vector3 q = getVertex(v2);
        Vector3 r = getVertex(v3);
        Vector3 normal = (q-p).cross(r-p).normalize();

        // Add the face surface normal to the sum of normals at
        // each vertex of the face
        mNormals[v1] += normal;
        mNormals[v2] += normal;
        mNormals[v3] += normal;
    }

    // Normalize the normal at each vertex
    for (uint i=0; i<getNbVertices(); i++) {
        mNormals[i] = mNormals[i].normalize();
    }
}

// Compute the tangents of the mesh
void Mesh::calculateTangents() {

    mTangents = std::vector<Vector3>(getNbVertices(), Vector3(0, 0, 0));

    // For each face
    for (uint i=0; i<getNbFaces(); i++) {

        // Get the three vertices index of the face
        uint v1 = getVertexIndexInFace(i, 0);
        uint v2 = getVertexIndexInFace(i, 1);
        uint v3 = getVertexIndexInFace(i, 2);

        assert(v1 < getNbVertices());
        assert(v2 < getNbVertices());
        assert(v3 < getNbVertices());

        // Get the vertices positions
        Vector3 p = getVertex(v1);
        Vector3 q = getVertex(v2);
        Vector3 r = getVertex(v3);

        // Get the texture coordinates of each vertex
        Vector2 uvP = getUV(v1);
        Vector2 uvQ = getUV(v2);
        Vector2 uvR = getUV(v3);

        // Get the three edges
        Vector3 edge1 = q - p;
        Vector3 edge2 = r - p;
        Vector2 edge1UV = uvQ - uvP;
        Vector2 edge2UV = uvR - uvP;

        float cp = edge1UV.y * edge2UV.x - edge1UV.x * edge2UV.y;

        // Compute the tangent
        if (cp != 0.0f) {
            float factor = 1.0f / cp;
            Vector3 tangent = (edge1 * -edge2UV.y + edge2 * edge1UV.y) * factor;
            tangent.normalize();
            mTangents[v1] = tangent;
            mTangents[v2] = tangent;
            mTangents[v3] = tangent;
        }
    }
}

// Calculate the bounding box of the mesh
void Mesh::calculateBoundingBox(Vector3& min, Vector3& max) const {

    // If the mesh contains vertices
    if (!mVertices.empty())  {

        min = mVertices[0];
        max = mVertices[0];

        std::vector<Vector3>::const_iterator  it(mVertices.begin());

        // For each vertex of the mesh
        for (; it != mVertices.end(); ++it) {

            if( (*it).x < min.x ) min.x = (*it).x;
            else if ( (*it).x > max.x ) max.x = (*it).x;

            if( (*it).y < min.y ) min.y = (*it).y;
            else if ( (*it).y > max.y ) max.y = (*it).y;

            if( (*it).z < min.z ) min.z = (*it).z;
            else if ( (*it).z > max.z ) max.z = (*it).z;
        }
    }
    else {
        std::cerr << "Error : Impossible to calculate the bounding box of the mesh because there" <<
                    "are no vertices !" << std::endl;
        assert(false);
    }
}

// Scale of vertices of the mesh using a given factor
void Mesh::scaleVertices(float factor) {

    // For each vertex
    for (uint i=0; i<getNbVertices(); i++) {
        mVertices.at(i) *= factor;
    }
}
