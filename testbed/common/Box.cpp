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
#define MEMBER_OFFSET(s,m) ((char *)NULL + (offsetof(s,m)))

// Initialize static variables
openglframework::VertexBufferObject Box::mVBOVertices(GL_ARRAY_BUFFER);
openglframework::VertexBufferObject Box::mVBONormals(GL_ARRAY_BUFFER);
openglframework::VertexArrayObject Box::mVAO;
int Box::totalNbBoxes = 0;
GLfloat Box::mCubeVertices[108] = {
    -1.0f,-1.0f,-1.0f, // triangle 1 : begin
    -1.0f,-1.0f, 1.0f,
    -1.0f, 1.0f, 1.0f, // triangle 1 : end
    1.0f, 1.0f,-1.0f, // triangle 2 : begin
    -1.0f,-1.0f,-1.0f,
    -1.0f, 1.0f,-1.0f, // triangle 2 : end
    1.0f,-1.0f, 1.0f,
    -1.0f,-1.0f,-1.0f,
    1.0f,-1.0f,-1.0f,
    1.0f, 1.0f,-1.0f,
    1.0f,-1.0f,-1.0f,
    -1.0f,-1.0f,-1.0f,
    -1.0f,-1.0f,-1.0f,
    -1.0f, 1.0f, 1.0f,
    -1.0f, 1.0f,-1.0f,
    1.0f,-1.0f, 1.0f,
    -1.0f,-1.0f, 1.0f,
    -1.0f,-1.0f,-1.0f,
    -1.0f, 1.0f, 1.0f,
    -1.0f,-1.0f, 1.0f,
    1.0f,-1.0f, 1.0f,
    1.0f, 1.0f, 1.0f,
    1.0f,-1.0f,-1.0f,
    1.0f, 1.0f,-1.0f,
    1.0f,-1.0f,-1.0f,
    1.0f, 1.0f, 1.0f,
    1.0f,-1.0f, 1.0f,
    1.0f, 1.0f, 1.0f,
    1.0f, 1.0f,-1.0f,
    -1.0f, 1.0f,-1.0f,
    1.0f, 1.0f, 1.0f,
    -1.0f, 1.0f,-1.0f,
    -1.0f, 1.0f, 1.0f,
    1.0f, 1.0f, 1.0f,
    -1.0f, 1.0f, 1.0f,
    1.0f,-1.0f, 1.0f
};
GLfloat Box::mCubeNormals[108] = {
    -1.0f, 0.0f, 0.0f, // triangle 1 : begin
    -1.0f, 0.0f, 0.0f,
    -1.0f, 0.0f, 0.0f, // triangle 1 : end
    0.0f, 0.0f,-1.0f, // triangle 2 : begin
    0.0f, 0.0f,-1.0f,
    0.0f, 0.0f,-1.0f, // triangle 2 : end
    0.0f,-1.0f, 0.0f,
    0.0f,-1.0f, 0.0f,
    0.0f,-1.0f, 0.0f,//
    0.0f, 0.0f,-1.0f,
    0.0f, 0.0f,-1.0f,
    0.0f, 0.0f,-1.0f,//
    -1.0f, 0.0f, 0.0f,
    -1.0f, 0.0f, 0.0f,
    -1.0f, 0.0f,0.0f,//
    0.0f,-1.0f, 0.0f,
    0.0f,-1.0f, 0.0f,
    0.0f,-1.0f, 0.0f,//
    0.0f, 0.0f, 1.0f,
    0.0f, 0.0f, 1.0f,
    0.0f, 0.0f, 1.0f,//
    1.0f, 0.0f, 0.0f,
    1.0f, 0.0f, 0.0f,
    1.0f, 0.0f, 0.0f,//
    1.0f, 0.0f, 0.0f,
    1.0f, 0.0f, 0.0f,
    1.0f, 0.0f, 0.0f,//
    0.0f, 1.0f, 0.0f,
    0.0f, 1.0f, 0.0f,
    0.0f, 1.0f, 0.0f,//
    0.0f, 1.0f, 0.0f,
    0.0f, 1.0f, 0.0f,
    0.0f, 1.0f, 0.0f,//
    0.0f, 0.0f, 1.0f,
    0.0f, 0.0f, 1.0f,
    0.0f, 0.0f, 1.0f//
};
// Constructor
Box::Box(const openglframework::Vector3& size, const openglframework::Vector3 &position,
         reactphysics3d::CollisionWorld* world)
    : openglframework::Object3D() {

    // Initialize the size of the box
    mSize[0] = size.x * 0.5f;
    mSize[1] = size.y * 0.5f;
    mSize[2] = size.z * 0.5f;

    // Compute the scaling matrix
    mScalingMatrix = openglframework::Matrix4(mSize[0], 0, 0, 0,
                                              0, mSize[1], 0, 0,
                                              0, 0, mSize[2], 0,
                                              0, 0, 0, 1);

    // Initialize the position where the cube will be rendered
    translateWorld(position);

    // Create the collision shape for the rigid body (box shape)
    // ReactPhysics3D will clone this object to create an internal one. Therefore,
    // it is OK if this object is destroyed right after calling RigidBody::addCollisionShape()
    mBoxShape = new rp3d::BoxShape(rp3d::Vector3(mSize[0], mSize[1], mSize[2]));

    // Initial position and orientation of the rigid body
    rp3d::Vector3 initPosition(position.x, position.y, position.z);
    rp3d::Quaternion initOrientation = rp3d::Quaternion::identity();
    rp3d::Transform transform(initPosition, initOrientation);

    mPreviousTransform = transform;

    // Create a rigid body in the dynamics world
    mBody = world->createCollisionBody(transform);

    // Add the collision shape to the body
    mProxyShape = mBody->addCollisionShape(mBoxShape, rp3d::Transform::identity());

    // If the Vertex Buffer object has not been created yet
    if (totalNbBoxes == 0) {

        // Create the Vertex Buffer
        createVBOAndVAO();
    }

    totalNbBoxes++;

    mTransformMatrix = mTransformMatrix * mScalingMatrix;
}

// Constructor
Box::Box(const openglframework::Vector3& size, const openglframework::Vector3& position,
         float mass, reactphysics3d::DynamicsWorld* world)
    : openglframework::Object3D() {

    // Initialize the size of the box
    mSize[0] = size.x * 0.5f;
    mSize[1] = size.y * 0.5f;
    mSize[2] = size.z * 0.5f;

    // Compute the scaling matrix
    mScalingMatrix = openglframework::Matrix4(mSize[0], 0, 0, 0,
                                              0, mSize[1], 0, 0,
                                              0, 0, mSize[2], 0,
                                              0, 0, 0, 1);

    // Initialize the position where the cube will be rendered
    translateWorld(position);

    // Create the collision shape for the rigid body (box shape)
    // ReactPhysics3D will clone this object to create an internal one. Therefore,
    // it is OK if this object is destroyed right after calling RigidBody::addCollisionShape()
    mBoxShape = new rp3d::BoxShape(rp3d::Vector3(mSize[0], mSize[1], mSize[2]));

    // Initial position and orientation of the rigid body
    rp3d::Vector3 initPosition(position.x, position.y, position.z);
    rp3d::Quaternion initOrientation = rp3d::Quaternion::identity();
    rp3d::Transform transform(initPosition, initOrientation);

    mPreviousTransform = transform;

    // Create a rigid body in the dynamics world
    rp3d::RigidBody* body = world->createRigidBody(transform);

    // Add the collision shape to the body
    mProxyShape = body->addCollisionShape(mBoxShape, rp3d::Transform::identity(), mass);

    mBody = body;

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
    delete mBoxShape;
    totalNbBoxes--;
}

// Render the cube at the correct position and with the correct orientation
void Box::render(openglframework::Shader& shader,
                 const openglframework::Matrix4& worldToCameraMatrix) {

    // Bind the VAO
    mVAO.bind();

    // Bind the shader
    shader.bind();

    mVBOVertices.bind();

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
    openglframework::Color currentColor = mBody->isSleeping() ? mSleepingColor : mColor;
    openglframework::Vector4 color(currentColor.r, currentColor.g, currentColor.b, currentColor.a);
    shader.setVector4Uniform("vertexColor", color, false);

    // Get the location of shader attribute variables
    GLint vertexPositionLoc = shader.getAttribLocation("vertexPosition");
    GLint vertexNormalLoc = shader.getAttribLocation("vertexNormal", false);

    glEnableVertexAttribArray(vertexPositionLoc);
    glVertexAttribPointer(vertexPositionLoc, 3, GL_FLOAT, GL_FALSE, 0, NULL);

    mVBONormals.bind();

    if (vertexNormalLoc != -1) glEnableVertexAttribArray(vertexNormalLoc);
    if (vertexNormalLoc != -1) glVertexAttribPointer(vertexNormalLoc, 3, GL_FLOAT, GL_FALSE, 0, NULL);

    // Draw the geometry of the box
    glDrawArrays(GL_TRIANGLES, 0, 36);

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
/// We create two VBOs (one for vertices and one for indices) to render all the boxes
/// in the simulation.
void Box::createVBOAndVAO() {

    // Create the VBO for the vertices data
    mVBOVertices.create();
    mVBOVertices.bind();
    mVBOVertices.copyDataIntoVBO(sizeof(mCubeVertices), mCubeVertices, GL_STATIC_DRAW);
    mVBOVertices.unbind();

    // Create th VBO for the normals data
    mVBONormals.create();
    mVBONormals.bind();
    mVBONormals.copyDataIntoVBO(sizeof(mCubeNormals), mCubeNormals, GL_STATIC_DRAW);
    mVBONormals.unbind();

    // Create the VAO for both VBOs
    mVAO.create();
    mVAO.bind();

    // Bind the VBO of vertices
    mVBOVertices.bind();

    // Bind the VBO of indices
    mVBONormals.bind();

    // Unbind the VAO
    mVAO.unbind();
}

// Reset the transform
void Box::resetTransform(const rp3d::Transform& transform) {

    // Reset the transform
    mBody->setTransform(transform);

    mBody->setIsSleeping(false);

    // Reset the velocity of the rigid body
    rp3d::RigidBody* rigidBody = dynamic_cast<rp3d::RigidBody*>(mBody);
    if (rigidBody != NULL) {
        rigidBody->setLinearVelocity(rp3d::Vector3(0, 0, 0));
        rigidBody->setAngularVelocity(rp3d::Vector3(0, 0, 0));
    }

    updateTransform(1.0f);
}

// Set the scaling of the object
void Box::setScaling(const openglframework::Vector3& scaling) {

    // Scale the collision shape
    mProxyShape->setLocalScaling(rp3d::Vector3(scaling.x, scaling.y, scaling.z));

    // Scale the graphics object
    mScalingMatrix = openglframework::Matrix4(mSize[0] * scaling.x, 0, 0, 0,
                                              0, mSize[1] * scaling.y, 0, 0,
                                              0, 0, mSize[2] * scaling.z, 0,
                                              0, 0, 0, 1);
}
