/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2010-2013 Daniel Chappuis                                       *
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
#include "VisualContactPoint.h"

// Initialization of static variables
int VisualContactPoint::mNbTotalPoints = 0;
bool VisualContactPoint::mIsMeshInitialized = false;
openglframework::Mesh VisualContactPoint::mMesh;

// Constructor
VisualContactPoint::VisualContactPoint(const openglframework::Vector3& position) {

    assert(mIsMeshInitialized);

    // Initialize the position where the sphere will be rendered
    translateWorld(position);
}

// Destructor
VisualContactPoint::~VisualContactPoint() {

}

// Load and initialize the mesh for all the contact points
void VisualContactPoint::createStaticData(const std::string& meshFolderPath) {

    if (!mIsMeshInitialized) {

        // Load the mesh from a file
        openglframework::MeshReaderWriter::loadMeshFromFile(meshFolderPath + "sphere.obj", mMesh);

        // Calculate the normals of the mesh
        mMesh.calculateNormals();

        mMesh.scaleVertices(VISUAL_CONTACT_POINT_RADIUS);

        mIsMeshInitialized = true;
    }
}

// Destroy the mesh for the contact points
void VisualContactPoint::destroyStaticData() {

    mMesh.destroy();
    mIsMeshInitialized = false;
}

// Render the sphere at the correct position and with the correct orientation
void VisualContactPoint::render(openglframework::Shader& shader,
                                const openglframework::Matrix4& worldToCameraMatrix) {

    // Bind the shader
    shader.bind();

    // Set the model to camera matrix
    const openglframework::Matrix4 localToCameraMatrix = worldToCameraMatrix * mTransformMatrix;
    shader.setMatrix4x4Uniform("localToCameraMatrix", localToCameraMatrix);

    // Set the normal matrix (inverse transpose of the 3x3 upper-left sub matrix of the
    // model-view matrix)
    const openglframework::Matrix3 normalMatrix =
                       localToCameraMatrix.getUpperLeft3x3Matrix().getInverse().getTranspose();
    shader.setMatrix3x3Uniform("normalMatrix", normalMatrix);

    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_NORMAL_ARRAY);
    if (mMesh.hasTexture()) {
        glEnableClientState(GL_TEXTURE_COORD_ARRAY);
    }

    glVertexPointer(3, GL_FLOAT, 0, mMesh.getVerticesPointer());
    glNormalPointer(GL_FLOAT, 0, mMesh.getNormalsPointer());
    if(mMesh.hasTexture()) {
        glTexCoordPointer(2, GL_FLOAT, 0, mMesh.getUVTextureCoordinatesPointer());
    }

    // For each part of the mesh
    for (unsigned int i=0; i<mMesh.getNbParts(); i++) {
        glDrawElements(GL_TRIANGLES, mMesh.getNbFaces(i) * 3,
                       GL_UNSIGNED_INT, mMesh.getIndicesPointer());
    }

    glDisableClientState(GL_NORMAL_ARRAY);
    glDisableClientState(GL_VERTEX_ARRAY);
    if (mMesh.hasTexture()) {
        glDisableClientState(GL_TEXTURE_COORD_ARRAY);
    }

    // Unbind the shader
    shader.unbind();
}
