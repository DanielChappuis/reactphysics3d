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
#include "HeightField.h"
#include "PerlinNoise.h"

// Constructor
HeightField::HeightField(bool createRigidBody, reactphysics3d::PhysicsCommon& physicsCommon, rp3d::PhysicsWorld* physicsWorld)
           : PhysicsObject(physicsCommon), mVBOVertices(GL_ARRAY_BUFFER),
             mVBONormals(GL_ARRAY_BUFFER), mVBOTextureCoords(GL_ARRAY_BUFFER),
             mVBOIndices(GL_ELEMENT_ARRAY_BUFFER) {

    // Compute the scaling matrix
    //mScalingMatrix = openglframework::Matrix4::identity();
    mScalingMatrix = openglframework::Matrix4(2, 0, 0, 0, 0, 2, 0, 0, 0, 0, 2, 0, 0, 0, 0, 1);

    // Generate the height field
    generateHeightField();

    // Generate the graphics mesh
    generateGraphicsMesh();

    // Create the collision shape for the rigid body (convex mesh shape) and
    // do not forget to delete it at the end
    mHeightFieldShape = mPhysicsCommon.createHeightFieldShape(NB_POINTS_WIDTH, NB_POINTS_LENGTH, mMinHeight, mMaxHeight,
                                                   mHeightData, rp3d::HeightFieldShape::HeightDataType::HEIGHT_FLOAT_TYPE);
    mHeightFieldShape->setScale(rp3d::Vector3(2, 2, 2));

    mPreviousTransform = rp3d::Transform::identity();

    // Create a body
    if (createRigidBody) {
        rp3d::RigidBody* body = physicsWorld->createRigidBody(mPreviousTransform);
        mCollider = body->addCollider(mHeightFieldShape, rp3d::Transform::identity());
        body->updateMassPropertiesFromColliders();
        mBody = body;
    }
    else {
        mBody = physicsWorld->createCollisionBody(mPreviousTransform);
        mCollider = mBody->addCollider(mHeightFieldShape, rp3d::Transform::identity());
    }


    // Create the VBOs and VAO
    createVBOAndVAO();

    mTransformMatrix = mTransformMatrix * mScalingMatrix;
}

// Destructor
HeightField::~HeightField() {

    // Destroy the mesh
    destroy();

    // Destroy the VBOs and VAO
    mVBOIndices.destroy();
    mVBOVertices.destroy();
    mVBONormals.destroy();
    mVBOTextureCoords.destroy();
    mVAO.destroy();

    mPhysicsCommon.destroyHeightFieldShape(mHeightFieldShape);
}

// Render the sphere at the correct position and with the correct orientation
void HeightField::render(openglframework::Shader& shader,
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

    if (vertexNormalLoc != -1) glVertexAttribPointer(vertexNormalLoc, 3, GL_FLOAT, GL_FALSE, 0, (char*)NULL);
    if (vertexNormalLoc != -1) glEnableVertexAttribArray(vertexNormalLoc);

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

// Compute the heights of the height field
void HeightField::generateHeightField() {

    double persistence = 9;
    double frequency = 0.28;
    double amplitude = 12;
    int octaves = 1;
    int randomseed = 23;
    PerlinNoise perlinNoise(persistence, frequency, amplitude, octaves, randomseed);

    mMinHeight = 0;
    mMaxHeight = 0;

    float width = (NB_POINTS_WIDTH - 1);
    float length = (NB_POINTS_LENGTH - 1);

    for (int i=0; i<NB_POINTS_WIDTH; i++) {
        for (int j=0; j<NB_POINTS_LENGTH; j++) {

            int arrayIndex = j * NB_POINTS_WIDTH + i;

            mHeightData[arrayIndex] = (float)(perlinNoise.GetHeight(-width * 0.5 + i, -length * 0.5 + j));

            if (i==0 && j==0) {
                mMinHeight = mHeightData[arrayIndex] ;
                mMaxHeight = mHeightData[arrayIndex] ;
            }

            if (mHeightData[arrayIndex] > mMaxHeight) mMaxHeight = mHeightData[arrayIndex] ;
            if (mHeightData[arrayIndex] < mMinHeight) mMinHeight = mHeightData[arrayIndex] ;
        }
    }
}

// Generate the graphics mesh to render the height field
void HeightField::generateGraphicsMesh() {

    std::vector<unsigned int> indices;
    int vertexId = 0;

    for (int i=0; i<NB_POINTS_WIDTH; i++) {
        for (int j=0; j<NB_POINTS_LENGTH; j++) {

            float originHeight = -(mMaxHeight - mMinHeight) * 0.5f - mMinHeight;
            float height = originHeight + mHeightData[j * NB_POINTS_WIDTH + i];
            openglframework::Vector3 vertex(-(NB_POINTS_WIDTH - 1) * 0.5f + i, height, -(NB_POINTS_LENGTH - 1) * 0.5f + j);

            mVertices.push_back(vertex);

            // Triangle indices
            if ((i < NB_POINTS_WIDTH - 1) && (j < NB_POINTS_LENGTH - 1)) {

                unsigned int v1 = vertexId;
                unsigned int v2 = vertexId + 1;
                unsigned int v3 = vertexId + NB_POINTS_LENGTH;
                unsigned int v4 = vertexId + NB_POINTS_LENGTH + 1;

                // First triangle
                indices.push_back(v1);
                indices.push_back(v2);
                indices.push_back(v3);

                // Second triangle
                indices.push_back(v2);
                indices.push_back(v4);
                indices.push_back(v3);
            }

            vertexId++;
        }
    }

    mIndices.push_back(indices);

    calculateNormals();
}

// Create the Vertex Buffer Objects used to render with OpenGL.
/// We create two VBOs (one for vertices and one for indices)
void HeightField::createVBOAndVAO() {

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
