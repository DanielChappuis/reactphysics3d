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

#ifndef CONVEX_MESH_H
#define CONVEX_MESH_H

// Libraries
#include "openglframework.h"
#include "reactphysics3d.h"

// Class ConvexMesh
class ConvexMesh : public openglframework::Mesh {

    private :

        // -------------------- Attributes -------------------- //

        /// Rigid body used to simulate the dynamics of the mesh
        rp3d::CollisionBody* mRigidBody;

        /// Previous transform (for interpolation)
        rp3d::Transform mPreviousTransform;

        /// Vertex Buffer Object for the vertices data
        openglframework::VertexBufferObject mVBOVertices;

        /// Vertex Buffer Object for the normals data
        openglframework::VertexBufferObject mVBONormals;

        /// Vertex Buffer Object for the texture coords
        openglframework::VertexBufferObject mVBOTextureCoords;

        /// Vertex Buffer Object for the indices
        openglframework::VertexBufferObject mVBOIndices;

        /// Vertex Array Object for the vertex data
        openglframework::VertexArrayObject mVAO;

        /// Color
        openglframework::Color mColor;

        // -------------------- Methods -------------------- //

        // Create the Vertex Buffer Objects used to render with OpenGL.
        void createVBOAndVAO();

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        ConvexMesh(const openglframework::Vector3& position,
                   rp3d::CollisionWorld* world, const std::string& meshFolderPath);

        /// Constructor
        ConvexMesh(const openglframework::Vector3& position, float mass,
                   rp3d::DynamicsWorld* dynamicsWorld, const std::string& meshFolderPath);

        /// Destructor
        ~ConvexMesh();

        /// Return a pointer to the collision body of the box
        reactphysics3d::CollisionBody* getCollisionBody();

        /// Return a pointer to the rigid body of the box
        reactphysics3d::RigidBody* getRigidBody();

        /// Update the transform matrix of the mesh
        void updateTransform(float interpolationFactor);

        /// Render the mesh at the correct position and with the correct orientation
        void render(openglframework::Shader& shader,
                    const openglframework::Matrix4& worldToCameraMatrix);

        /// Set the position of the box
        void resetTransform(const rp3d::Transform& transform);
};

// Return a pointer to the collision body of the box
inline rp3d::CollisionBody* ConvexMesh::getCollisionBody() {
    return mRigidBody;
}

// Return a pointer to the rigid body of the box
inline rp3d::RigidBody* ConvexMesh::getRigidBody() {
    return dynamic_cast<rp3d::RigidBody*>(mRigidBody);
}

#endif
