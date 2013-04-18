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
#include "Box.h"

// Macros
#define MEMBER_OFFSET(s,m) ((char *)NULL + (offsetof(s,m)))

// Initialize static variables
openglframework::VertexBufferObject Box::mVBOVertices(GL_ARRAY_BUFFER);
openglframework::VertexBufferObject Box::mVBOIndices(GL_ELEMENT_ARRAY_BUFFER);
bool Box::areVBOsCreated = false;
VertexData Box::mCubeVertices[8] = {
 {openglframework::Vector3(1,1,1),openglframework::Vector3(1,1,1),openglframework::Color(0,0,1,1)},
 {openglframework::Vector3(-1,1,1),openglframework::Vector3(-1,1,1),openglframework::Color(0,0,1,1)},
 {openglframework::Vector3(-1,-1,1),openglframework::Vector3(-1,-1,1),openglframework::Color(0,0,1,1)},
 {openglframework::Vector3(1,-1,1),openglframework::Vector3(1,-1,1),openglframework::Color(0,0,1,1)},
 {openglframework::Vector3(1,-1,-1),openglframework::Vector3(1,-1,-1),openglframework::Color(0,0,1,1)},
 {openglframework::Vector3(-1,-1,-1),openglframework::Vector3(-1,-1,-1),openglframework::Color(0,0,1,1)},
 {openglframework::Vector3(-1,1,-1),openglframework::Vector3(-1,1,-1),openglframework::Color(0,0,1,1)},
 {openglframework::Vector3(1,1,-1),openglframework::Vector3(1,1,-1),openglframework::Color(0,0,1,1)}
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
         float mass, reactphysics3d::DynamicsWorld* dynamicsWorld)
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
    mCollisionShape = new rp3d::BoxShape(rp3d::Vector3(mSize[0], mSize[1], mSize[2]));

    // Compute the inertia tensor of the body using its collision shape
    rp3d::Matrix3x3 inertiaTensor;
    mCollisionShape->computeLocalInertiaTensor(inertiaTensor, mass);

    // Initial position and orientation of the rigid body
    rp3d::Vector3 initPosition(position.x, position.y, position.z);
    rp3d::Quaternion initOrientation = rp3d::Quaternion::identity();
    rp3d::Transform transform(initPosition, initOrientation);

    // Create a rigid body corresponding to the cube in the dynamics world
    mRigidBody = dynamicsWorld->createRigidBody(transform, mass, inertiaTensor, mCollisionShape);

    // If the Vertex Buffer object has not been created yet
    if (!areVBOsCreated) {
        // Create the Vertex Buffer
        createVBO();
    }
}

// Destructor
Box::~Box() {

    // Destroy the collision shape
    delete mCollisionShape;
}

// Render the cube at the correct position and with the correct orientation
void Box::render(openglframework::Shader& shader) {

    // Bind the shader
    shader.bind();

    // Set the model to World matrix
    shader.setMatrix4x4Uniform("modelToWorldMatrix", mTransformMatrix);

    // Bind the vertices VBO
    mVBOVertices.bind();

    // Enable the vertex, normal and color arrays
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_COLOR_ARRAY);
    glEnableClientState(GL_NORMAL_ARRAY);

    // Set the arrays pointers
    glVertexPointer(3, GL_FLOAT, sizeof(VertexData), MEMBER_OFFSET(VertexData, position));
    glNormalPointer(GL_FLOAT, sizeof(VertexData), MEMBER_OFFSET(VertexData, normal));
    glColorPointer(3, GL_FLOAT, sizeof(VertexData), MEMBER_OFFSET(VertexData, color));

    // Bind the indices VBO
    mVBOIndices.bind();

    // Draw the geometry of the box
    glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_INT, (char*)NULL);

    // Unbind the VBOs
    mVBOVertices.unbind();
    mVBOIndices.unbind();

    // Disable the arrays
    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_COLOR_ARRAY);
    glDisableClientState(GL_NORMAL_ARRAY);

    // Unbind the shader
    shader.unbind();
}

// Update the transform matrix of the box
void Box::updateTransform() {

    // Get the interpolated transform of the rigid body
    rp3d::Transform transform = mRigidBody->getInterpolatedTransform();

    // Compute the transform used for rendering the box
    float matrix[16];
    transform.getOpenGLMatrix(matrix);
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
void Box::createVBO() {

    // Create the VBOs
    mVBOVertices.create();
    mVBOIndices.create();

    // Copy the data into the VBOs
    mVBOVertices.copyDataIntoVBO(sizeof(mCubeVertices), mCubeVertices, GL_STATIC_DRAW);
    mVBOIndices.copyDataIntoVBO(sizeof(mCubeIndices), mCubeIndices, GL_STATIC_DRAW);

    areVBOsCreated = true;
}
