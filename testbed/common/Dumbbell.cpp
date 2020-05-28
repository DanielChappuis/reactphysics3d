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
#include "Dumbbell.h"

openglframework::VertexBufferObject Dumbbell::mVBOVertices(GL_ARRAY_BUFFER);
openglframework::VertexBufferObject Dumbbell::mVBONormals(GL_ARRAY_BUFFER);
openglframework::VertexBufferObject Dumbbell::mVBOTextureCoords(GL_ARRAY_BUFFER);
openglframework::VertexBufferObject Dumbbell::mVBOIndices(GL_ELEMENT_ARRAY_BUFFER);
openglframework::VertexArrayObject Dumbbell::mVAO;
int Dumbbell::totalNbDumbbells = 0;

// Constructor
Dumbbell::Dumbbell(bool createRigidBody, rp3d::PhysicsCommon& physicsCommon, rp3d::PhysicsWorld* physicsWorld, const std::string& meshFolderPath)
         : PhysicsObject(physicsCommon, meshFolderPath + "dumbbell.obj") {

    // Identity scaling matrix
    mScalingMatrix.setToIdentity();

    mDistanceBetweenSphere = 8.0f;

    // Create a sphere collision shape for the two ends of the dumbbell
    // ReactPhysics3D will clone this object to create an internal one. Therefore,
    // it is OK if this object is destroyed right after calling RigidBody::addCollisionShape()
    const rp3d::decimal radiusSphere = rp3d::decimal(1.5);
    mSphereShape = mPhysicsCommon.createSphereShape(radiusSphere);

    // Create a capsule collision shape for the middle of the dumbbell
    // ReactPhysics3D will clone this object to create an internal one. Therefore,
    // it is OK if this object is destroyed right after calling RigidBody::addCollisionShape()
    const rp3d::decimal radiusCapsule = rp3d::decimal(0.5);
    const rp3d::decimal heightCapsule = rp3d::decimal(7.0);
    mCapsuleShape = mPhysicsCommon.createCapsuleShape(radiusCapsule, heightCapsule);

    mPreviousTransform = rp3d::Transform::identity();

    // Initial transform of the first sphere collision shape of the dumbbell (in local-space)
    rp3d::Transform transformSphereShape1(rp3d::Vector3(0, mDistanceBetweenSphere / 2.0f, 0), rp3d::Quaternion::identity());

    // Initial transform of the second sphere collision shape of the dumbell (in local-space)
    rp3d::Transform transformSphereShape2(rp3d::Vector3(0, -mDistanceBetweenSphere / 2.0f, 0), rp3d::Quaternion::identity());

    // Initial transform of the cylinder collision shape of the dumbell (in local-space)
    rp3d::Transform transformCylinderShape(rp3d::Vector3(0, 0, 0), rp3d::Quaternion::identity());

    // Create a body corresponding to the dumbbell in the physics world
    if (createRigidBody) {

        rp3d::RigidBody* body = physicsWorld->createRigidBody(mPreviousTransform);
        mColliderSphere1 = body->addCollider(mSphereShape, transformSphereShape1);
        mColliderSphere2 = body->addCollider(mSphereShape, transformSphereShape2);
        mColliderCapsule = body->addCollider(mCapsuleShape, transformCylinderShape);
        mColliderSphere1->getMaterial().setMassDensity(2);
        mColliderSphere2->getMaterial().setMassDensity(2);
        body->updateMassPropertiesFromColliders();
        mBody = body;
    }
    else {

        mBody = physicsWorld->createCollisionBody(mPreviousTransform);
        mColliderSphere1 = mBody->addCollider(mSphereShape, transformSphereShape1);
        mColliderSphere2 = mBody->addCollider(mSphereShape, transformSphereShape2);
        mColliderCapsule = mBody->addCollider(mCapsuleShape, transformCylinderShape);
    }


    mTransformMatrix = mTransformMatrix * mScalingMatrix;

    // Create the VBOs and VAO
    if (totalNbDumbbells == 0) {
        createVBOAndVAO();
    }

    totalNbDumbbells++;
}

// Destructor
Dumbbell::~Dumbbell() {

    if (totalNbDumbbells == 1) {

        // Destroy the mesh
        destroy();

        // Destroy the VBOs and VAO
        mVBOIndices.destroy();
        mVBOVertices.destroy();
        mVBONormals.destroy();
        mVBOTextureCoords.destroy();
        mVAO.destroy();
    }
    mPhysicsCommon.destroySphereShape(mSphereShape);
    mPhysicsCommon.destroyCapsuleShape(mCapsuleShape);
    totalNbDumbbells--;
}

// Render the sphere at the correct position and with the correct orientation
void Dumbbell::render(openglframework::Shader& shader,
                    const openglframework::Matrix4& worldToCameraMatrix) {

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
    glVertexAttribPointer(vertexPositionLoc, 3, GL_FLOAT, GL_FALSE, 0, (char*)NULL);

    mVBONormals.bind();

    if (vertexNormalLoc != -1) glEnableVertexAttribArray(vertexNormalLoc);
    if (vertexNormalLoc != -1) glVertexAttribPointer(vertexNormalLoc, 3, GL_FLOAT, GL_FALSE, 0, (char*)NULL);

    // For each part of the mesh
    for (unsigned int i=0; i<getNbParts(); i++) {
        glDrawElements(GL_TRIANGLES, getNbFaces(i) * 3, GL_UNSIGNED_INT, (char*)NULL);
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

// Create the Vertex Buffer Objects used to render with OpenGL.
/// We create two VBOs (one for vertices and one for indices)
void Dumbbell::createVBOAndVAO() {

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

    // Create th VBO for the indices data
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
