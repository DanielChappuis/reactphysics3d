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
#include "ConvexMesh.h"
#include <unordered_set>

// Constructor
ConvexMesh::ConvexMesh(bool createRigidBody, rp3d::PhysicsCommon& physicsCommon, rp3d::PhysicsWorld* physicsWorld, const std::string& meshPath)
           : PhysicsObject(physicsCommon, meshPath), mVBOVertices(GL_ARRAY_BUFFER),
             mVBONormals(GL_ARRAY_BUFFER), mVBOTextureCoords(GL_ARRAY_BUFFER),
             mVBOIndices(GL_ELEMENT_ARRAY_BUFFER) {

    // Compute the scaling matrix
    mScalingMatrix = openglframework::Matrix4::identity();

    // Polygon faces descriptions for the polyhedron
    mPolygonFaces = new rp3d::PolygonVertexArray::PolygonFace[getNbFaces(0)];
    rp3d::PolygonVertexArray::PolygonFace* face = mPolygonFaces;
    for (uint f=0; f < getNbFaces(0); f++) {

		for (int v = 0; v < 3; v++) {
			
			const openglframework::Vector3 vertex = mVertices[mIndices[0][f*3 + v]];
			int vIndex = findVertexIndex(mConvexMeshVertices, vertex);
			if (vIndex == -1) {
				vIndex = mConvexMeshVertices.size();
				mConvexMeshVertices.push_back(vertex);
			}

			mConvexMeshIndices.push_back(vIndex);
		}

        face->indexBase = f * 3;
        face->nbVertices = 3;
        face++;
    }

    // Create the polygon vertex array
    mPolygonVertexArray =
            new rp3d::PolygonVertexArray(mConvexMeshVertices.size(), &(mConvexMeshVertices[0]), sizeof(openglframework::Vector3),
                                         &(mConvexMeshIndices[0]), sizeof(int),
                                         getNbFaces(0), mPolygonFaces,
                                         rp3d::PolygonVertexArray::VertexDataType::VERTEX_FLOAT_TYPE,
                                         rp3d::PolygonVertexArray::IndexDataType::INDEX_INTEGER_TYPE);

    // Create the polyhedron mesh
    mPolyhedronMesh = mPhysicsCommon.createPolyhedronMesh(mPolygonVertexArray);

    // Create the collision shape for the rigid body (convex mesh shape) and do
    // not forget to delete it at the end
    mConvexShape = mPhysicsCommon.createConvexMeshShape(mPolyhedronMesh);

    mPreviousTransform = rp3d::Transform::identity();

    // Create a rigid body corresponding to the sphere in the physics world
    if (createRigidBody) {
        rp3d::RigidBody* body = physicsWorld->createRigidBody(mPreviousTransform);
        mCollider = body->addCollider(mConvexShape, rp3d::Transform::identity());
        body->updateMassPropertiesFromColliders();
        mBody = body;
    }
    else {

        mBody = physicsWorld->createCollisionBody(mPreviousTransform);
        mCollider = mBody->addCollider(mConvexShape, rp3d::Transform::identity());
    }


    // Create the VBOs and VAO
    createVBOAndVAO();

    mTransformMatrix = mTransformMatrix * mScalingMatrix;
}

// Destructor
ConvexMesh::~ConvexMesh() {

    // Destroy the mesh
    destroy();

    // Destroy the VBOs and VAO
    mVBOIndices.destroy();
    mVBOVertices.destroy();
    mVBONormals.destroy();
    mVBOTextureCoords.destroy();
    mVAO.destroy();

    mPhysicsCommon.destroyConvexMeshShape(mConvexShape);
    mPhysicsCommon.destroyPolyhedronMesh(mPolyhedronMesh);
    delete mPolygonVertexArray;
    delete[] mPolygonFaces;
}

// Render the sphere at the correct position and with the correct orientation
void ConvexMesh::render(openglframework::Shader& shader,
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

// Create the Vertex Buffer Objects used to render with OpenGL.
/// We create two VBOs (one for vertices and one for indices)
void ConvexMesh::createVBOAndVAO() {

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

// Return the index of a given vertex in the mesh
int ConvexMesh::findVertexIndex(const std::vector<openglframework::Vector3>& vertices, const openglframework::Vector3& vertex) {

	for (int i = 0; i < vertices.size(); i++) {
		if (vertices[i] == vertex) {
			return i;
		}
	}

	return -1;
}

