/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2015 Daniel Chappuis                                       *
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
openglframework::VertexBufferObject VisualContactPoint::mVBOVertices(GL_ARRAY_BUFFER);
openglframework::VertexBufferObject VisualContactPoint::mVBONormals(GL_ARRAY_BUFFER);
openglframework::VertexBufferObject VisualContactPoint::mVBOIndices(GL_ELEMENT_ARRAY_BUFFER);
openglframework::VertexArrayObject VisualContactPoint::mVAO;
int VisualContactPoint::mNbTotalPoints = 0;
openglframework::Mesh VisualContactPoint::mMesh;
int VisualContactPoint::totalNbBoxes = 0;

// Constructor
VisualContactPoint::VisualContactPoint(const openglframework::Vector3& position,
                                       openglframework::Shader& shader,
                                       const std::string& meshFolderPath)
    : mColor(1.0f, 0.0f, 0.0f, 1.0f) {

    // Initialize the position where the mesh will be rendered
    translateWorld(position);

    // Create the VBOs and VAO
    if (totalNbBoxes == 0) {
        createStaticData(meshFolderPath);
        createVBOAndVAO(shader);
    }

    totalNbBoxes++;
}

// Destructor
VisualContactPoint::~VisualContactPoint() {

    if (totalNbBoxes == 1) {

        // Destroy the VBOs and VAO
        mVBOIndices.destroy();
        mVBOVertices.destroy();
        mVBONormals.destroy();
        mVAO.destroy();

        destroyStaticData();
    }

    totalNbBoxes--;
}

// Load and initialize the mesh for all the contact points
void VisualContactPoint::createStaticData(const std::string& meshFolderPath) {

    // Load the mesh from a file
    openglframework::MeshReaderWriter::loadMeshFromFile(meshFolderPath + "sphere.obj", mMesh);

    // Calculate the normals of the mesh
    mMesh.calculateNormals();

    mMesh.scaleVertices(VISUAL_CONTACT_POINT_RADIUS);
}

// Destroy the mesh for the contact points
void VisualContactPoint::destroyStaticData() {

    mMesh.destroy();
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

    // Set the vertex color
    openglframework::Vector4 color(mColor.r, mColor.g, mColor.b, mColor.a);
    shader.setVector4Uniform("vertexColor", color);

    // Bind the VAO
    mVAO.bind();

    // For each part of the mesh
    for (unsigned int i=0; i<mMesh.getNbParts(); i++) {
        glDrawElements(GL_TRIANGLES, mMesh.getNbFaces(i) * 3, GL_UNSIGNED_INT, (char*)NULL);
    }

    // Unbind the VAO
    mVAO.unbind();

    // Unbind the shader
    shader.unbind();
}

// Create the Vertex Buffer Objects used to render with OpenGL.
/// We create two VBOs (one for vertices and one for indices)
void VisualContactPoint::createVBOAndVAO(openglframework::Shader& shader) {

    // Bind the shader
    shader.bind();

    // Get the location of shader attribute variables
    GLint vertexPositionLoc = shader.getAttribLocation("vertexPosition");
    GLint vertexNormalLoc = shader.getAttribLocation("vertexNormal");
    GLint vertexTexCoordLoc = shader.getAttribLocation("textureCoords");

    // Create the VBO for the vertices data
    mVBOVertices.create();
    mVBOVertices.bind();
    size_t sizeVertices = mMesh.getVertices().size() * sizeof(openglframework::Vector3);
    mVBOVertices.copyDataIntoVBO(sizeVertices, mMesh.getVerticesPointer(), GL_STATIC_DRAW);
    mVBOVertices.unbind();

    // Create the VBO for the normals data
    mVBONormals.create();
    mVBONormals.bind();
    size_t sizeNormals = mMesh.getNormals().size() * sizeof(openglframework::Vector3);
    mVBONormals.copyDataIntoVBO(sizeNormals, mMesh.getNormalsPointer(), GL_STATIC_DRAW);
    mVBONormals.unbind();

    // Create th VBO for the indices data
    mVBOIndices.create();
    mVBOIndices.bind();
    size_t sizeIndices = mMesh.getIndices(0).size() * sizeof(uint);
    mVBOIndices.copyDataIntoVBO(sizeIndices, mMesh.getIndicesPointer(), GL_STATIC_DRAW);
    mVBOIndices.unbind();

    // Create the VAO for both VBOs
    mVAO.create();
    mVAO.bind();

    // Bind the VBO of vertices
    mVBOVertices.bind();
    glEnableVertexAttribArray(vertexPositionLoc);
    glVertexAttribPointer(vertexPositionLoc, 3, GL_FLOAT, GL_FALSE, 0, (char*)NULL);

    // Bind the VBO of normals
    mVBONormals.bind();
    glEnableVertexAttribArray(vertexNormalLoc);
    glVertexAttribPointer(vertexNormalLoc, 3, GL_FLOAT, GL_FALSE, 0, (char*)NULL);

    // Bind the VBO of indices
    mVBOIndices.bind();

    // Unbind the VAO
    mVAO.unbind();

    // Unbind the shader
    shader.unbind();
}
