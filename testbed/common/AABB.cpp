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
#include "AABB.h"

// Macros
#define MEMBER_OFFSET(s,m) ((char *)nullptr + (offsetof(s,m)))

// Initialize static variables
openglframework::VertexBufferObject AABB::mVBOVertices(GL_ARRAY_BUFFER);
openglframework::VertexBufferObject AABB::mVBONormals(GL_ARRAY_BUFFER);
openglframework::VertexBufferObject AABB::mVBOIndices(GL_ELEMENT_ARRAY_BUFFER);
openglframework::VertexArrayObject AABB::mVAO;


// Initialize the data to render AABBs
void AABB::init() {
    createVBOAndVAO();
}

// Destroy the data used to render AABBs
void AABB::destroy() {

    // Destroy the VBOs and VAO
    mVBOVertices.destroy();
    mVBONormals.destroy();
    mVAO.destroy();
}

// Render the AABB
void AABB::render(const openglframework::Vector3& position, const openglframework::Vector3& dimension,
                  openglframework::Color color, openglframework::Shader& shader,
                  const openglframework::Matrix4& worldToCameraMatrix) {

    // Render in wireframe mode
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    // Bind the shader
    shader.bind();

    // Transform matrix
    openglframework::Matrix4 transformMatrix = openglframework::Matrix4::identity();
    transformMatrix = openglframework::Matrix4::translationMatrix(position) * transformMatrix;

    // Set the normal matrix (inverse transpose of the 3x3 upper-left sub matrix of the
    // model-view matrix)
    const openglframework::Matrix4 localToCameraMatrix = worldToCameraMatrix * transformMatrix;
    const openglframework::Matrix3 normalMatrix =
                       localToCameraMatrix.getUpperLeft3x3Matrix().getInverse().getTranspose();
    shader.setMatrix3x3Uniform("normalMatrix", normalMatrix, false);


    // Compute the scaling matrix
    openglframework::Matrix4 scalingMatrix = openglframework::Matrix4(dimension.x, 0, 0, 0,
                                              0, dimension.y, 0, 0,
                                              0, 0, dimension.z, 0,
                                              0, 0, 0, 1);

    transformMatrix = transformMatrix * scalingMatrix;

    // Set the model to camera matrix
    shader.setMatrix4x4Uniform("localToWorldMatrix", transformMatrix);
    shader.setMatrix4x4Uniform("worldToCameraMatrix", worldToCameraMatrix);

    // Set the vertex color
    openglframework::Vector4 colorVec(color.r, color.g, color.b, color.a);
    shader.setIntUniform("isGlobalVertexColorEnabled", 1, false);
    shader.setVector4Uniform("globalVertexColor", colorVec, false);

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

    // Draw the geometry
    glDrawElements(GL_LINES, 12 * 2, GL_UNSIGNED_INT, (char*)nullptr);

    glDisableVertexAttribArray(vertexPositionLoc);
    if (vertexNormalLoc != -1) glDisableVertexAttribArray(vertexNormalLoc);

    mVBONormals.unbind();
    mVBOVertices.unbind();

    // Unbind the VAO
    mVAO.unbind();

    // Unbind the shader
    shader.unbind();

    // Disable wireframe mode
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}

// Create the Vertex Buffer Objects used to render to box with OpenGL.
/// We create two VBOs (one for vertices and one for indices)
void AABB::createVBOAndVAO() {

    std::vector<openglframework::Vector3> vertices;
    std::vector<openglframework::Vector3> normals;
    std::vector<unsigned int> indices;

    // Vertices
    vertices.push_back(openglframework::Vector3(-0.5, -0.5, 0.5));
    vertices.push_back(openglframework::Vector3(0.5, -0.5, 0.5));
    vertices.push_back(openglframework::Vector3(0.5, 0.5, 0.5));
    vertices.push_back(openglframework::Vector3(-0.5, 0.5, 0.5));
    vertices.push_back(openglframework::Vector3(0.5, -0.5, -0.5));
    vertices.push_back(openglframework::Vector3(0.5, 0.5, -0.5));
    vertices.push_back(openglframework::Vector3(-0.5, 0.5, -0.5));
    vertices.push_back(openglframework::Vector3(-0.5, -0.5, -0.5));

    // Normals
    normals.push_back(openglframework::Vector3(1, -1, 1));
    normals.push_back(openglframework::Vector3(1, 1, 1));
    normals.push_back(openglframework::Vector3(-1, 1, 1));
    normals.push_back(openglframework::Vector3(-1, -1, 1));
    normals.push_back(openglframework::Vector3(1, -1, -1));
    normals.push_back(openglframework::Vector3(1, 1, -1));
    normals.push_back(openglframework::Vector3(-1, 1, -1));
    normals.push_back(openglframework::Vector3(-1, -1, -1));

    // Indices
    indices.push_back(0); indices.push_back(1); indices.push_back(1); indices.push_back(2);
    indices.push_back(2); indices.push_back(3); indices.push_back(3); indices.push_back(0);

    indices.push_back(4); indices.push_back(5); indices.push_back(5); indices.push_back(6);
    indices.push_back(6); indices.push_back(7); indices.push_back(7); indices.push_back(4);

    indices.push_back(1); indices.push_back(4); indices.push_back(2); indices.push_back(5);
    indices.push_back(0); indices.push_back(7); indices.push_back(3); indices.push_back(6);

    // Create the VBO for the vertices data
    mVBOVertices.create();
    mVBOVertices.bind();
    GLsizei sizeVertices = static_cast<GLsizei>(vertices.size() * sizeof(openglframework::Vector3));
    mVBOVertices.copyDataIntoVBO(sizeVertices, &(vertices[0]), GL_STATIC_DRAW);
    mVBOVertices.unbind();

    // Create the VBO for the normals data
    mVBONormals.create();
    mVBONormals.bind();
    GLsizei sizeNormals = static_cast<GLsizei>(normals.size() * sizeof(openglframework::Vector3));
    mVBONormals.copyDataIntoVBO(sizeNormals, &(normals[0]), GL_STATIC_DRAW);
    mVBONormals.unbind();

    // Create th VBO for the indices data
    mVBOIndices.create();
    mVBOIndices.bind();
    GLsizei sizeIndices = static_cast<GLsizei>(indices.size() * sizeof(unsigned int));
    mVBOIndices.copyDataIntoVBO(sizeIndices, &(indices[0]), GL_STATIC_DRAW);
    mVBOIndices.unbind();

    // Create the VAO for both VBOs
    mVAO.create();
    mVAO.bind();

    // Bind the VBO of vertices
    mVBOVertices.bind();

    // Bind the VBO of normals
    mVBONormals.bind();

    // Bind the VBO of indices
    mVBOIndices.bind();

    // Unbind the VAO
    mVAO.unbind();
}
