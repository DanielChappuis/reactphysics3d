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

#ifndef DUMBBELL_H
#define DUMBBELL_H

// Libraries
#include "openglframework.h"
#include <reactphysics3d/reactphysics3d.h>
#include "PhysicsObject.h"

// Class Dumbbell
class Dumbbell : public PhysicsObject {

    private :

        // -------------------- Attributes -------------------- //

        /// Radius of the spheres
        float mDistanceBetweenSphere;

        /// Collision shapes
        rp3d::CapsuleShape* mCapsuleShape;
        rp3d::SphereShape* mSphereShape;
        rp3d::Collider* mColliderCapsule;
        rp3d::Collider* mColliderSphere1;
        rp3d::Collider* mColliderSphere2;

        /// Scaling matrix (applied to a sphere to obtain the correct sphere dimensions)
        openglframework::Matrix4 mScalingMatrix;

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

        // -------------------- Methods -------------------- //

        // Create the Vertex Buffer Objects used to render with OpenGL.
        void createVBOAndVAO();

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        Dumbbell(bool createRigidBody, reactphysics3d::PhysicsCommon& physicsCommon, rp3d::PhysicsWorld* physicsWorld,
                 const std::string& meshFolderPath);

        /// Destructor
        virtual ~Dumbbell() override;

        /// Render the sphere at the correct position and with the correct orientation
        virtual void render(openglframework::Shader& shader, const openglframework::Matrix4& worldToCameraMatrix) override;

        /// Update the transform matrix of the object
        virtual void updateTransform(float interpolationFactor) override;

        /// Return the capsule collider
        rp3d::Collider* getCapsuleCollider();

        /// Return the first sphere collider
        rp3d::Collider* getSphere1Collider();

        /// Return the second sphere collider
        rp3d::Collider* getSphere2Collider();
};

// Update the transform matrix of the object
inline void Dumbbell::updateTransform(float interpolationFactor) {
    mTransformMatrix = computeTransform(interpolationFactor, mScalingMatrix);
}

// Return the capsule collider
inline rp3d::Collider* Dumbbell::getCapsuleCollider() {
   return mColliderCapsule;
}

// Return the first sphere collider
inline rp3d::Collider* Dumbbell::getSphere1Collider() {
    return mColliderSphere1;
}

// Return the second sphere collider
inline rp3d::Collider* Dumbbell::getSphere2Collider() {
    return mColliderSphere2;
}

#endif
