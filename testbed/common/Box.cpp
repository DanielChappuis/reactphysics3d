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
#include "Box.h"

// Macros
#define MEMBER_OFFSET(s,m) ((char *)NULL + (offsetof(s,m)))

// Initialize static variables
openglframework::VertexBufferObject Box::mVBOVertices(GL_ARRAY_BUFFER);
openglframework::VertexBufferObject Box::mVBOIndices(GL_ELEMENT_ARRAY_BUFFER);
openglframework::VertexArrayObject Box::mVAO;
int Box::totalNbBoxes = 0;
VertexData Box::mCubeVertices[8] = {
 {openglframework::Vector3(1,1,1),openglframework::Vector3(1,1,1),openglframework::Color(1,0,0,1)},
 {openglframework::Vector3(-1,1,1),openglframework::Vector3(-1,1,1),openglframework::Color(1,0,0,1)},
 {openglframework::Vector3(-1,-1,1),openglframework::Vector3(-1,-1,1),openglframework::Color(1,0,0,1)},
 {openglframework::Vector3(1,-1,1),openglframework::Vector3(1,-1,1),openglframework::Color(1,0,0,1)},
 {openglframework::Vector3(1,-1,-1),openglframework::Vector3(1,-1,-1),openglframework::Color(1,0,0,1)},
 {openglframework::Vector3(-1,-1,-1),openglframework::Vector3(-1,-1,-1),openglframework::Color(1,0,0,1)},
 {openglframework::Vector3(-1,1,-1),openglframework::Vector3(-1,1,-1),openglframework::Color(1,0,0,1)},
 {openglframework::Vector3(1,1,-1),openglframework::Vector3(1,1,-1),openglframework::Color(1,0,0,1)}
};
GLuint Box::mCubeIndices[36] = { 0, 1, 2,
                                 2, 3, 0,
                                 7, 4, 5,
                                 5, 6, 7,
                                 6, 5, 2,
                                 2, 1, 6,
                                 7, 0, 3,
                                 3, 4, 7,
                                 7, 6, 1,
                                 1, 0, 7,
                                 3, 2, 5,
                                 5, 4, 3};

// Constructor
Box::Box(const openglframework::Vector3& size, const openglframework::Vector3 &position,
         reactphysics3d::CollisionWorld* world, openglframework::Shader& shader)
    : openglframework::Object3D(), mColor(0.01f, 0.62f, 0.39f, 1.0f) {

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
    const rp3d::BoxShape collisionShape(rp3d::Vector3(mSize[0], mSize[1], mSize[2]));

    // Initial position and orientation of the rigid body
    rp3d::Vector3 initPosition(position.x, position.y, position.z);
    rp3d::Quaternion initOrientation = rp3d::Quaternion::identity();
    rp3d::Transform transform(initPosition, initOrientation);

    mPreviousTransform = transform;

    // Create a rigid body in the dynamics world
    mRigidBody = world->createCollisionBody(transform);

    // Add the collision shape to the body
    mRigidBody->addCollisionShape(collisionShape, rp3d::Transform::identity());

    // If the Vertex Buffer object has not been created yet
    if (totalNbBoxes == 0) {

        // Create the Vertex Buffer
        createVBOAndVAO(shader);
    }

    totalNbBoxes++;

    mTransformMatrix = mTransformMatrix * mScalingMatrix;
}

// Constructor
Box::Box(const openglframework::Vector3& size, const openglframework::Vector3 &position,
         float mass, reactphysics3d::DynamicsWorld* world, openglframework::Shader& shader)
    : openglframework::Object3D(), mColor(0.01f, 0.62f, 0.39f, 1.0f) {

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
    const rp3d::BoxShape collisionShape(rp3d::Vector3(mSize[0], mSize[1], mSize[2]));

    // Initial position and orientation of the rigid body
    rp3d::Vector3 initPosition(position.x, position.y, position.z);
    rp3d::Quaternion initOrientation = rp3d::Quaternion::identity();
    rp3d::Transform transform(initPosition, initOrientation);

    // Create a rigid body in the dynamics world
    rp3d::RigidBody* body = world->createRigidBody(transform);

    // Add the collision shape to the body
    body->addCollisionShape(collisionShape, rp3d::Transform::identity(), mass);

    mRigidBody = body;

    // If the Vertex Buffer object has not been created yet
    if (totalNbBoxes == 0) {

        // Create the Vertex Buffer
        createVBOAndVAO(shader);
    }

    totalNbBoxes++;

    mTransformMatrix = mTransformMatrix * mScalingMatrix;
}

// Destructor
Box::~Box() {

    if (totalNbBoxes == 1) {

        // Destroy the VBOs and VAO
        mVBOIndices.destroy();
        mVBOVertices.destroy();
        mVAO.destroy();
    }

    totalNbBoxes--;
}

// Render the cube at the correct position and with the correct orientation
void Box::render(openglframework::Shader& shader,
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

    // Draw the geometry of the box
    glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_INT, (char*)NULL);

    // Unbind the VAO
    mVAO.unbind();

    // Unbind the shader
    shader.unbind();
}

// Update the transform matrix of the box
void Box::updateTransform(float interpolationFactor) {

    // Get the transform of the rigid body
    rp3d::Transform transform = mRigidBody->getTransform();

    // Interpolate the transform between the previous one and the new one
    rp3d::Transform interpolatedTransform = rp3d::Transform::interpolateTransforms(mPreviousTransform,
                                                                                  transform,
                                                                                  interpolationFactor);
    mPreviousTransform = transform;

    // Compute the transform used for rendering the box
    rp3d::decimal matrix[16];
    interpolatedTransform.getOpenGLMatrix(matrix);
    openglframework::Matrix4 newMatrix(matrix[0], matrix[4], matrix[8], matrix[12],
                                       matrix[1], matrix[5], matrix[9], matrix[13],
                                       matrix[2], matrix[6], matrix[10], matrix[14],
                                       matrix[3], matrix[7], matrix[11], matrix[15]);

    // Apply the scaling matrix to have the correct box dimensions
    mTransformMatrix = newMatrix * mScalingMatrix;
}

// Create the Vertex Buffer Objects used to render to box with OpenGL.
/// We create two VBOs (one for vertices and one for indices) to render all the boxes
/// in the simulation.
void Box::createVBOAndVAO(openglframework::Shader& shader) {

    // Bind the shader
    shader.bind();

    // Get the location of shader attribute variables
    GLint vertexPositionLoc = shader.getAttribLocation("vertexPosition");
    GLint vertexNormalLoc = shader.getAttribLocation("vertexNormal");

    // Create the VBO for the vertices data
    mVBOVertices.create();
    mVBOVertices.bind();
    mVBOVertices.copyDataIntoVBO(sizeof(mCubeVertices), mCubeVertices, GL_STATIC_DRAW);
    mVBOVertices.unbind();

    // Create th VBO for the indices data
    mVBOIndices.create();
    mVBOIndices.bind();
    mVBOIndices.copyDataIntoVBO(sizeof(mCubeIndices), mCubeIndices, GL_STATIC_DRAW);
    mVBOIndices.unbind();

    // Create the VAO for both VBOs
    mVAO.create();
    mVAO.bind();

    // Bind the VBO of vertices
    mVBOVertices.bind();

    glEnableVertexAttribArray(vertexPositionLoc);
    glEnableVertexAttribArray(vertexNormalLoc);

    glVertexAttribPointer(vertexPositionLoc, 3, GL_FLOAT, GL_FALSE, sizeof(VertexData), MEMBER_OFFSET(VertexData, position));
    glVertexAttribPointer(vertexNormalLoc, 3, GL_FLOAT, GL_FALSE, sizeof(VertexData), MEMBER_OFFSET(VertexData, normal));

    // Bind the VBO of indices
    mVBOIndices.bind();

    // Unbind the VAO
    mVAO.unbind();

    // Unbind the shader
    shader.unbind();
}

// Reset the transform
void Box::resetTransform(const rp3d::Transform& transform) {

    // Reset the transform
    mRigidBody->setTransform(transform);

    mRigidBody->setIsSleeping(false);

    // Reset the velocity of the rigid body
    rp3d::RigidBody* rigidBody = dynamic_cast<rp3d::RigidBody*>(mRigidBody);
    if (rigidBody != NULL) {
        rigidBody->setLinearVelocity(rp3d::Vector3(0, 0, 0));
        rigidBody->setAngularVelocity(rp3d::Vector3(0, 0, 0));
    }

    updateTransform(1.0f);
}
