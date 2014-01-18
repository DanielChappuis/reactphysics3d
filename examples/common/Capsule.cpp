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
#include "Capsule.h"


// Constructor
Capsule::Capsule(float radius, float height, const openglframework::Vector3& position,
                 float mass, reactphysics3d::DynamicsWorld* dynamicsWorld,
                 const std::string& meshFolderPath)
        : openglframework::Mesh(), mRadius(radius), mHeight(height) {

    // Load the mesh from a file
    openglframework::MeshReaderWriter::loadMeshFromFile(meshFolderPath + "capsule.obj", *this);

    // Calculate the normals of the mesh
    calculateNormals();

    // Compute the scaling matrix
    mScalingMatrix = openglframework::Matrix4(mRadius, 0, 0, 0,
                                              0, (mHeight + 2.0f * mRadius) / 3.0f, 0,0,
                                              0, 0, mRadius, 0,
                                              0, 0, 0, 1.0f);

    // Initialize the position where the sphere will be rendered
    translateWorld(position);

    // Create the collision shape for the rigid body (sphere shape)
    // ReactPhysics3D will clone this object to create an internal one. Therefore,
    // it is OK if this object is destroyed right after calling Dynamics::createRigidBody()
    const rp3d::CapsuleShape collisionShape(mRadius, mHeight);

    // Initial position and orientation of the rigid body
    rp3d::Vector3 initPosition(position.x, position.y, position.z);
    rp3d::Quaternion initOrientation = rp3d::Quaternion::identity();
    rp3d::Transform transform(initPosition, initOrientation);

    // Create a rigid body corresponding to the sphere in the dynamics world
    mRigidBody = dynamicsWorld->createRigidBody(transform, mass, collisionShape);
}

// Destructor
Capsule::~Capsule() {

    // Destroy the mesh
    destroy();
}

// Render the sphere at the correct position and with the correct orientation
void Capsule::render(openglframework::Shader& shader,
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
    if (hasTexture()) {
        glEnableClientState(GL_TEXTURE_COORD_ARRAY);
    }

    glVertexPointer(3, GL_FLOAT, 0, getVerticesPointer());
    glNormalPointer(GL_FLOAT, 0, getNormalsPointer());
    if(hasTexture()) {
        glTexCoordPointer(2, GL_FLOAT, 0, getUVTextureCoordinatesPointer());
    }

    // For each part of the mesh
    for (unsigned int i=0; i<getNbParts(); i++) {
        glDrawElements(GL_TRIANGLES, getNbFaces(i) * 3,
                       GL_UNSIGNED_INT, getIndicesPointer());
    }

    glDisableClientState(GL_NORMAL_ARRAY);
    glDisableClientState(GL_VERTEX_ARRAY);
    if (hasTexture()) {
        glDisableClientState(GL_TEXTURE_COORD_ARRAY);
    }

    // Unbind the shader
    shader.unbind();
}

// Update the transform matrix of the sphere
void Capsule::updateTransform() {

    // Get the interpolated transform of the rigid body
    rp3d::Transform transform = mRigidBody->getInterpolatedTransform();

    // Compute the transform used for rendering the sphere
    float matrix[16];
    transform.getOpenGLMatrix(matrix);
    openglframework::Matrix4 newMatrix(matrix[0], matrix[4], matrix[8], matrix[12],
                                       matrix[1], matrix[5], matrix[9], matrix[13],
                                       matrix[2], matrix[6], matrix[10], matrix[14],
                                       matrix[3], matrix[7], matrix[11], matrix[15]);

    // Apply the scaling matrix to have the correct sphere dimensions
    mTransformMatrix = newMatrix * mScalingMatrix;
}
