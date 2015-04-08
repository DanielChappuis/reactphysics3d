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

#ifndef BOX_H
#define BOX_H

// Libraries
#include "openglframework.h"
#include "reactphysics3d.h"

// Structure VertexData
struct VertexData {

    /// Vertex position
    openglframework::Vector3 position;

    /// Vertex normal
    openglframework::Vector3 normal;

    // Vertex color
    openglframework::Color color;
};

// Class Box
class Box : public openglframework::Object3D {

    private :

        // -------------------- Attributes -------------------- //

        /// Size of each side of the box
        float mSize[3];

        /// Rigid body used to simulate the dynamics of the box
        rp3d::CollisionBody* mRigidBody;

        /// Scaling matrix (applied to a cube to obtain the correct box dimensions)
        openglframework::Matrix4 mScalingMatrix;

        /// Vertex Buffer Object for the vertices data used to render the box with OpenGL
        static openglframework::VertexBufferObject mVBOVertices;

        /// Vertex Buffer Object for the indices used to render the box with OpenGL
        static openglframework::VertexBufferObject mVBOIndices;

        /// Vertex data for each vertex of the cube (used to render the box)
        static VertexData mCubeVertices[8];

        /// Indices of the cube (used to render the box)
        static GLuint mCubeIndices[36];

        /// True if the VBOs have already been created
        static bool areVBOsCreated;

        /// Main color of the box
        openglframework::Color mColor;

        // -------------------- Methods -------------------- //

        /// Create a Vertex Buffer Object to render to box with OpenGL
        static void createVBO();

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        Box(const openglframework::Vector3& size, const openglframework::Vector3& position,
            reactphysics3d::CollisionWorld* world);

        /// Constructor
        Box(const openglframework::Vector3& size, const openglframework::Vector3& position,
            float mass, reactphysics3d::DynamicsWorld *world);

        /// Destructor
        ~Box();

        /// Return a pointer to the collision body of the box
        reactphysics3d::CollisionBody* getCollisionBody();

        /// Return a pointer to the rigid body of the box
        reactphysics3d::RigidBody* getRigidBody();

        /// Update the transform matrix of the box
        void updateTransform();

        /// Render the cube at the correct position and with the correct orientation
        void render(openglframework::Shader& shader, const openglframework::Matrix4& worldToCameraMatrix);

        /// Set the color of the box
        void setColor(const openglframework::Color& color);
};

// Return a pointer to the collision body of the box
inline rp3d::CollisionBody* Box::getCollisionBody() {
    return mRigidBody;
}

// Return a pointer to the rigid body of the box
inline rp3d::RigidBody* Box::getRigidBody() {
    return dynamic_cast<rp3d::RigidBody*>(mRigidBody);
}

// Set the color of the box
inline void Box::setColor(const openglframework::Color& color) {
    mColor = color;
}

#endif
