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

#ifndef DUMBBELL_H
#define DUMBBELL_H

// Libraries
#include "openglframework.h"
#include "reactphysics3d.h"

// Class Sphere
class Dumbbell : public openglframework::Mesh {

    private :

        // -------------------- Attributes -------------------- //

        /// Radius of the spheres
        float mRadius;

        /// Rigid body used to simulate the dynamics of the sphere
        rp3d::CollisionBody* mBody;

        /// Scaling matrix (applied to a sphere to obtain the correct sphere dimensions)
        openglframework::Matrix4 mScalingMatrix;

        /// Previous transform (for interpolation)
        rp3d::Transform mPreviousTransform;

        /// Vertex Buffer Object for the vertices data
        static openglframework::VertexBufferObject mVBOVertices;

        /// Vertex Buffer Object for the normals data
        static openglframework::VertexBufferObject mVBONormals;

        /// Vertex Buffer Object for the texture coords
        static openglframework::VertexBufferObject mVBOTextureCoords;

        /// Vertex Buffer Object for the indices
        static openglframework::VertexBufferObject mVBOIndices;

        /// Vertex Array Object for the vertex data
        static openglframework::VertexArrayObject mVAO;

        // Total number of capsules created
        static int totalNbDumbbells;

        /// Color
        openglframework::Color mColor;

        // -------------------- Methods -------------------- //

        // Create the Vertex Buffer Objects used to render with OpenGL.
        void createVBOAndVAO();

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        Dumbbell(const openglframework::Vector3& position, rp3d::DynamicsWorld* dynamicsWorld,
                 const std::string& meshFolderPath);

        /// Constructor
        Dumbbell(const openglframework::Vector3& position, rp3d::CollisionWorld* world,
                 const std::string& meshFolderPath);


        /// Destructor
        ~Dumbbell();

        /// Return a pointer to the rigid body
        rp3d::RigidBody* getRigidBody();

        /// Return a pointer to the body
        rp3d::CollisionBody* getCollisionBody();

        /// Update the transform matrix of the sphere
        void updateTransform(float interpolationFactor);

        /// Render the sphere at the correct position and with the correct orientation
        void render(openglframework::Shader& shader,
                    const openglframework::Matrix4& worldToCameraMatrix);

        /// Set the position of the box
        void resetTransform(const rp3d::Transform& transform);
};

// Return a pointer to the rigid body of the sphere
inline rp3d::RigidBody* Dumbbell::getRigidBody() {
    return dynamic_cast<rp3d::RigidBody*>(mBody);
}

// Return a pointer to the body
inline rp3d::CollisionBody* Dumbbell::getCollisionBody() {
    return mBody;
}

#endif
