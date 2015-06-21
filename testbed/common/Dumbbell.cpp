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
#include "Dumbbell.h"

// Constructor
Dumbbell::Dumbbell(const openglframework::Vector3 &position,
                   reactphysics3d::DynamicsWorld* dynamicsWorld, const std::string& meshFolderPath,
                   openglframework::Shader& shader)
         : openglframework::Mesh(),mVBOVertices(GL_ARRAY_BUFFER),
           mVBONormals(GL_ARRAY_BUFFER), mVBOTextureCoords(GL_ARRAY_BUFFER),
           mVBOIndices(GL_ELEMENT_ARRAY_BUFFER), mColor(0.5f, 0.5f, 0.5f, 1.0f) {

    // Load the mesh from a file
    openglframework::MeshReaderWriter::loadMeshFromFile(meshFolderPath + "dumbbell.obj", *this);

    // Calculate the normals of the mesh
    calculateNormals();

    // Identity scaling matrix
    mScalingMatrix.setToIdentity();

    // Initialize the position where the sphere will be rendered
    translateWorld(position);

    // Create a sphere collision shape for the two ends of the dumbbell
    // ReactPhysics3D will clone this object to create an internal one. Therefore,
    // it is OK if this object is destroyed right after calling RigidBody::addCollisionShape()
    const rp3d::decimal radiusSphere = rp3d::decimal(1.5);
    const rp3d::decimal massSphere = rp3d::decimal(2.0);
    const rp3d::SphereShape sphereCollisionShape(radiusSphere);

    // Create a cylinder collision shape for the middle of the dumbbell
    // ReactPhysics3D will clone this object to create an internal one. Therefore,
    // it is OK if this object is destroyed right after calling RigidBody::addCollisionShape()
    const rp3d::decimal radiusCylinder = rp3d::decimal(0.5);
    const rp3d::decimal heightCylinder = rp3d::decimal(8.0);
    const rp3d::decimal massCylinder = rp3d::decimal(1.0);
    const rp3d::CylinderShape cylinderCollisionShape(radiusCylinder, heightCylinder);

    // Initial position and orientation of the rigid body
    rp3d::Vector3 initPosition(position.x, position.y, position.z);
    rp3d::decimal angleAroundX = 0;//rp3d::PI / 2;
    rp3d::Quaternion initOrientation(angleAroundX, 0, 0);
    rp3d::Transform transformBody(initPosition, initOrientation);

    mPreviousTransform = transformBody;

    // Initial transform of the first sphere collision shape of the dumbbell (in local-space)
    rp3d::Transform transformSphereShape1(rp3d::Vector3(0, 4.0, 0), rp3d::Quaternion::identity());

    // Initial transform of the second sphere collision shape of the dumbell (in local-space)
    rp3d::Transform transformSphereShape2(rp3d::Vector3(0, -4.0, 0), rp3d::Quaternion::identity());

    // Initial transform of the cylinder collision shape of the dumbell (in local-space)
    rp3d::Transform transformCylinderShape(rp3d::Vector3(0, 0, 0), rp3d::Quaternion::identity());

    // Create a rigid body corresponding to the dumbbell in the dynamics world
    rp3d::RigidBody* body = dynamicsWorld->createRigidBody(transformBody);

    // Add the three collision shapes to the body and specify the mass and transform of the shapes
    body->addCollisionShape(sphereCollisionShape, transformSphereShape1, massSphere);
    body->addCollisionShape(sphereCollisionShape, transformSphereShape2, massSphere);
    body->addCollisionShape(cylinderCollisionShape, transformCylinderShape, massCylinder);

    mBody = body;

    mTransformMatrix = mTransformMatrix * mScalingMatrix;

    // Create the VBOs and VAO
    createVBOAndVAO(shader);
}

// Constructor
Dumbbell::Dumbbell(const openglframework::Vector3 &position,
                   reactphysics3d::CollisionWorld* world, const std::string& meshFolderPath,
                   openglframework::Shader& shader)
         : openglframework::Mesh(), mVBOVertices(GL_ARRAY_BUFFER),
           mVBONormals(GL_ARRAY_BUFFER), mVBOTextureCoords(GL_ARRAY_BUFFER),
           mVBOIndices(GL_ELEMENT_ARRAY_BUFFER), mColor(0.5f, 0.5f, 0.5f, 1.0f) {

    // Load the mesh from a file
    openglframework::MeshReaderWriter::loadMeshFromFile(meshFolderPath + "dumbbell.obj", *this);

    // Calculate the normals of the mesh
    calculateNormals();

    // Identity scaling matrix
    mScalingMatrix.setToIdentity();

    // Initialize the position where the sphere will be rendered
    translateWorld(position);

    // Create a sphere collision shape for the two ends of the dumbbell
    // ReactPhysics3D will clone this object to create an internal one. Therefore,
    // it is OK if this object is destroyed right after calling RigidBody::addCollisionShape()
    const rp3d::decimal radiusSphere = rp3d::decimal(1.5);
    const rp3d::decimal massSphere = rp3d::decimal(2.0);
    const rp3d::SphereShape sphereCollisionShape(radiusSphere);

    // Create a cylinder collision shape for the middle of the dumbbell
    // ReactPhysics3D will clone this object to create an internal one. Therefore,
    // it is OK if this object is destroyed right after calling RigidBody::addCollisionShape()
    const rp3d::decimal radiusCylinder = rp3d::decimal(0.5);
    const rp3d::decimal heightCylinder = rp3d::decimal(8.0);
    const rp3d::decimal massCylinder = rp3d::decimal(1.0);
    const rp3d::CylinderShape cylinderCollisionShape(radiusCylinder, heightCylinder);

    // Initial position and orientation of the rigid body
    rp3d::Vector3 initPosition(position.x, position.y, position.z);
    rp3d::decimal angleAroundX = 0;//rp3d::PI / 2;
    rp3d::Quaternion initOrientation(angleAroundX, 0, 0);
    rp3d::Transform transformBody(initPosition, initOrientation);

    // Initial transform of the first sphere collision shape of the dumbbell (in local-space)
    rp3d::Transform transformSphereShape1(rp3d::Vector3(0, 4.0, 0), rp3d::Quaternion::identity());

    // Initial transform of the second sphere collision shape of the dumbell (in local-space)
    rp3d::Transform transformSphereShape2(rp3d::Vector3(0, -4.0, 0), rp3d::Quaternion::identity());

    // Initial transform of the cylinder collision shape of the dumbell (in local-space)
    rp3d::Transform transformCylinderShape(rp3d::Vector3(0, 0, 0), rp3d::Quaternion::identity());

    // Create a rigid body corresponding to the dumbbell in the dynamics world
    mBody = world->createCollisionBody(transformBody);

    // Add the three collision shapes to the body and specify the mass and transform of the shapes
    mBody->addCollisionShape(sphereCollisionShape, transformSphereShape1);
    mBody->addCollisionShape(sphereCollisionShape, transformSphereShape2);
    mBody->addCollisionShape(cylinderCollisionShape, transformCylinderShape);

    mTransformMatrix = mTransformMatrix * mScalingMatrix;

    // Create the VBOs and VAO
    createVBOAndVAO(shader);
}

// Destructor
Dumbbell::~Dumbbell() {

    // Destroy the mesh
    destroy();

    // Destroy the VBOs and VAO
    mVBOIndices.destroy();
    mVBOVertices.destroy();
    mVBONormals.destroy();
    mVBOTextureCoords.destroy();
    mVAO.destroy();
}

// Render the sphere at the correct position and with the correct orientation
void Dumbbell::render(openglframework::Shader& shader,
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
    for (uint i = 0u; i < getNbParts(); i++) {
        glDrawElements(GL_TRIANGLES, getNbFaces(i) * 3, GL_UNSIGNED_INT, NULL);
    }

    // Unbind the VAO
    mVAO.unbind();

    // Unbind the shader
    shader.unbind();
}

// Update the transform matrix of the sphere
void Dumbbell::updateTransform(float interpolationFactor) {

    // Get the transform of the rigid body
    rp3d::Transform transform = mBody->getTransform();

    // Interpolate the transform between the previous one and the new one
    rp3d::Transform interpolatedTransform = rp3d::Transform::interpolateTransforms(mPreviousTransform,
                                                                                  transform,
                                                                                  interpolationFactor);
    mPreviousTransform = transform;

    // Compute the transform used for rendering the sphere
    rp3d::decimal matrix[16];
    interpolatedTransform.getOpenGLMatrix(matrix);
    openglframework::Matrix4 newMatrix(matrix[0], matrix[4], matrix[8], matrix[12],
                                       matrix[1], matrix[5], matrix[9], matrix[13],
                                       matrix[2], matrix[6], matrix[10], matrix[14],
                                       matrix[3], matrix[7], matrix[11], matrix[15]);

    // Apply the scaling matrix to have the correct sphere dimensions
    mTransformMatrix = newMatrix * mScalingMatrix;
}

// Create the Vertex Buffer Objects used to render with OpenGL.
/// We create two VBOs (one for vertices and one for indices)
void Dumbbell::createVBOAndVAO(openglframework::Shader& shader) {

    // Bind the shader
    shader.bind();

    // Get the location of shader attribute variables
    GLint vertexPositionLoc = shader.getAttribLocation("vertexPosition");
    GLint vertexNormalLoc = shader.getAttribLocation("vertexNormal");
    GLint vertexTexCoordLoc = shader.getAttribLocation("textureCoords");

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
    size_t sizeIndices = mIndices[0].size() * sizeof(uint);
    mVBOIndices.copyDataIntoVBO(sizeIndices, getIndicesPointer(), GL_STATIC_DRAW);
    mVBOIndices.unbind();

    // Create the VAO for both VBOs
    mVAO.create();
    mVAO.bind();

    // Bind the VBO of vertices
    mVBOVertices.bind();
    glEnableVertexAttribArray(vertexPositionLoc);
    glVertexAttribPointer(vertexPositionLoc, 3, GL_FLOAT, GL_FALSE, 0, NULL);

    // Bind the VBO of normals
    mVBONormals.bind();
    glEnableVertexAttribArray(vertexNormalLoc);
    glVertexAttribPointer(vertexNormalLoc, 3, GL_FLOAT, GL_FALSE, 0, NULL);

    if (hasTexture()) {
        // Bind the VBO of texture coords
        mVBOTextureCoords.bind();
        glEnableVertexAttribArray(vertexTexCoordLoc);
        glVertexAttribPointer(vertexTexCoordLoc, 2, GL_FLOAT, GL_FALSE, 0, NULL);
    }

    // Bind the VBO of indices
    mVBOIndices.bind();

    // Unbind the VAO
    mVAO.unbind();

    // Unbind the shader
    shader.unbind();
}
