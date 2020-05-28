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
#include "Box.h"

// Macros
#define MEMBER_OFFSET(s,m) ((char *)nullptr + (offsetof(s,m)))

// Initialize static variables
openglframework::VertexBufferObject Box::mVBOVertices(GL_ARRAY_BUFFER);
openglframework::VertexBufferObject Box::mVBONormals(GL_ARRAY_BUFFER);
openglframework::VertexBufferObject Box::mVBOTextureCoords(GL_ARRAY_BUFFER);
openglframework::VertexBufferObject Box::mVBOIndices(GL_ELEMENT_ARRAY_BUFFER);
openglframework::VertexArrayObject Box::mVAO;
int Box::totalNbBoxes = 0;

// Constructor
Box::Box(bool createRigidBody, const openglframework::Vector3& size, reactphysics3d::PhysicsCommon& physicsCommon, reactphysics3d::PhysicsWorld* world,
         const std::string& meshFolderPath)
    : PhysicsObject(physicsCommon, meshFolderPath + "cube.obj") {

    // Initialize the size of the box
    mSize[0] = size.x * 0.5f;
    mSize[1] = size.y * 0.5f;
    mSize[2] = size.z * 0.5f;

    // Compute the scaling matrix
    mScalingMatrix = openglframework::Matrix4(mSize[0], 0, 0, 0,
                                              0, mSize[1], 0, 0,
                                              0, 0, mSize[2], 0,
                                              0, 0, 0, 1);

    // Create the collision shape for the rigid body (box shape)
    // ReactPhysics3D will clone this object to create an internal one. Therefore,
    // it is OK if this object is destroyed right after calling RigidBody::addCollisionShape()
    mBoxShape = mPhysicsCommon.createBoxShape(rp3d::Vector3(mSize[0], mSize[1], mSize[2]));

    mPreviousTransform = rp3d::Transform::identity();

    if (createRigidBody) {

        // Create a rigid body in the physics world
        rp3d::RigidBody* body = world->createRigidBody(mPreviousTransform);
        mCollider = body->addCollider(mBoxShape, rp3d::Transform::identity());
        body->updateMassPropertiesFromColliders();
        mBody = body;
    }
    else {

        // Create a body in the physics world
        mBody = world->createCollisionBody(mPreviousTransform);
        mCollider = mBody->addCollider(mBoxShape, rp3d::Transform::identity());
    }

    // If the Vertex Buffer object has not been created yet
    if (totalNbBoxes == 0) {

        // Create the Vertex Buffer
        createVBOAndVAO();
    }

    totalNbBoxes++;

    mTransformMatrix = mTransformMatrix * mScalingMatrix;
}

// Destructor
Box::~Box() {

    if (totalNbBoxes == 1) {

        // Destroy the VBOs and VAO
        mVBOVertices.destroy();
        mVBONormals.destroy();
        mVAO.destroy();
    }
    mPhysicsCommon.destroyBoxShape(mBoxShape);
    totalNbBoxes--;
}

// Render the cube at the correct position and with the correct orientation
void Box::render(openglframework::Shader& shader, const openglframework::Matrix4& worldToCameraMatrix) {

    // Bind the shader
    shader.bind();

    // Set the model to camera matrix
    shader.setMatrix4x4Uniform("localToWorldMatrix", mTransformMatrix);
    shader.setMatrix4x4Uniform("worldToCameraMatrix", worldToCameraMatrix);

    // Set the normal matrix (inverse transpose of the 3x3 upper-left sub matrix of the
    // model-view matrix)
    const openglframework::Matrix4 localToCameraMatrix = worldToCameraMatrix * mTransformMatrix;
    const openglframework::Matrix3 normalMatrix =
                       localToCameraMatrix.getUpperLeft3x3Matrix().getInverse().getTranspose();
    shader.setMatrix3x3Uniform("normalMatrix", normalMatrix, false);

    // Set the vertex color
    rp3d::RigidBody* rigidBody = dynamic_cast<rp3d::RigidBody*>(mBody);
    openglframework::Color currentColor = rigidBody != nullptr && rigidBody->isSleeping() ? mSleepingColor : mColor;
    openglframework::Vector4 color(currentColor.r, currentColor.g, currentColor.b, currentColor.a);
    shader.setVector4Uniform("globalVertexColor", color, false);

    // Bind the VAO
    mVAO.bind();

    mVBOVertices.bind();

    // Get the location of shader attribute variables
    GLint vertexPositionLoc = shader.getAttribLocation("vertexPosition");
    GLint vertexNormalLoc = shader.getAttribLocation("vertexNormal", false);

    glEnableVertexAttribArray(vertexPositionLoc);
    glVertexAttribPointer(vertexPositionLoc, 3, GL_FLOAT, GL_FALSE, 0, (char*)nullptr);

    mVBONormals.bind();

    if (vertexNormalLoc != -1) glVertexAttribPointer(vertexNormalLoc, 3, GL_FLOAT, GL_FALSE, 0, (char*)nullptr);
    if (vertexNormalLoc != -1) glEnableVertexAttribArray(vertexNormalLoc);

    // For each part of the mesh
    for (unsigned int i=0; i<getNbParts(); i++) {
        glDrawElements(GL_TRIANGLES, getNbFaces(i) * 3, GL_UNSIGNED_INT, (char*)nullptr);
    }

    glDisableVertexAttribArray(vertexPositionLoc);
    if (vertexNormalLoc != -1) glDisableVertexAttribArray(vertexNormalLoc);

    mVBONormals.unbind();
    mVBOVertices.unbind();

    // Unbind the VAO
    mVAO.unbind();

    // Unbind the shader
    shader.unbind();
}

// Create the Vertex Buffer Objects used to render to box with OpenGL.
/// We create two VBOs (one for vertices and one for indices) to render the box
void Box::createVBOAndVAO() {

    // Create the VBO for the vertices data
    mVBOVertices.create();
    mVBOVertices.bind();
    size_t sizeVertices = mVertices.size() * sizeof(openglframework::Vector3);
    mVBOVertices.copyDataIntoVBO(sizeVertices, getVerticesPointer(), GL_STATIC_DRAW);
    mVBOVertices.unbind();

    // Create the VBO for the normals data
    mVBONormals.create();
    mVBONormals.bind();
    size_t sizeNormals = mNormals.size() * sizeof(openglframework::Vector3);
    mVBONormals.copyDataIntoVBO(sizeNormals, getNormalsPointer(), GL_STATIC_DRAW);
    mVBONormals.unbind();

    if (hasTexture()) {
        // Create the VBO for the texture co data
        mVBOTextureCoords.create();
        mVBOTextureCoords.bind();
        size_t sizeTextureCoords = mUVs.size() * sizeof(openglframework::Vector2);
        mVBOTextureCoords.copyDataIntoVBO(sizeTextureCoords, getUVTextureCoordinatesPointer(), GL_STATIC_DRAW);
        mVBOTextureCoords.unbind();
    }

    // Create the VBO for the indices data
    mVBOIndices.create();
    mVBOIndices.bind();
    size_t sizeIndices = mIndices[0].size() * sizeof(unsigned int);
    mVBOIndices.copyDataIntoVBO(sizeIndices, getIndicesPointer(), GL_STATIC_DRAW);
    mVBOIndices.unbind();

    // Create the VAO for both VBOs
    mVAO.create();
    mVAO.bind();

    // Bind the VBO of vertices
    mVBOVertices.bind();

    // Bind the VBO of normals
    mVBONormals.bind();

    if (hasTexture()) {
        // Bind the VBO of texture coords
        mVBOTextureCoords.bind();
    }

    // Bind the VBO of indices
    mVBOIndices.bind();

    // Unbind the VAO
    mVAO.unbind();
}
