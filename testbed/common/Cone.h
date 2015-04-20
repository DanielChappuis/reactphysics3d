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

#ifndef CONE_H
#define CONE_H

// Libraries
#include "openglframework.h"
#include "reactphysics3d.h"

// Class Cone
class Cone : public openglframework::Mesh {

    private :

        // -------------------- Attributes -------------------- //

        /// Radius of the cone
        float mRadius;

        /// Height of the cone
        float mHeight;

        /// Rigid body used to simulate the dynamics of the cone
        rp3d::CollisionBody* mRigidBody;

        /// Scaling matrix (applied to a sphere to obtain the correct cone dimensions)
        openglframework::Matrix4 mScalingMatrix;

        /// Previous transform (for interpolation)
        rp3d::Transform mPreviousTransform;

        // -------------------- Methods -------------------- //

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        Cone(float radius, float height, const openglframework::Vector3& position,
             rp3d::CollisionWorld* world, const std::string& meshFolderPath);

        /// Constructor
        Cone(float radius, float height, const openglframework::Vector3& position,
             float mass, rp3d::DynamicsWorld* dynamicsWorld, const std::string& meshFolderPath);

        /// Destructor
        ~Cone();

        /// Return a pointer to the collision body of the box
        reactphysics3d::CollisionBody* getCollisionBody();

        /// Return a pointer to the rigid body of the box
        reactphysics3d::RigidBody* getRigidBody();

        /// Update the transform matrix of the cone
        void updateTransform(float interpolationFactor);

        /// Render the cone at the correct position and with the correct orientation
        void render(openglframework::Shader& shader,
                    const openglframework::Matrix4& worldToCameraMatrix);
};

// Return a pointer to the collision body of the box
inline rp3d::CollisionBody* Cone::getCollisionBody() {
    return mRigidBody;
}

// Return a pointer to the rigid body of the box
inline rp3d::RigidBody* Cone::getRigidBody() {
    return dynamic_cast<rp3d::RigidBody*>(mRigidBody);
}

#endif
