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

#ifndef SPHERE_H
#define SPHERE_H

// Libraries
#include "openglframework.h"
#include "reactphysics3d.h"

// Class Sphere
class Sphere : public openglframework::Mesh {

    private :

        // -------------------- Attributes -------------------- //

        /// Radius of the sphere
        float mRadius;

        /// Rigid body used to simulate the dynamics of the sphere
        rp3d::CollisionBody* mRigidBody;

        /// Scaling matrix (applied to a sphere to obtain the correct sphere dimensions)
        openglframework::Matrix4 mScalingMatrix;

        // -------------------- Methods -------------------- //

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        Sphere(float radius, const openglframework::Vector3& position,
               rp3d::CollisionWorld* world, const std::string& meshFolderPath);

        /// Constructor
        Sphere(float radius, const openglframework::Vector3& position,
               float mass, rp3d::DynamicsWorld* dynamicsWorld, const std::string& meshFolderPath);

        /// Destructor
        ~Sphere();

        /// Return a pointer to the collision body of the box
        reactphysics3d::CollisionBody* getCollisionBody();

        /// Return a pointer to the rigid body of the box
        reactphysics3d::RigidBody* getRigidBody();

        /// Update the transform matrix of the sphere
        void updateTransform();

        /// Render the sphere at the correct position and with the correct orientation
        void render(openglframework::Shader& shader,
                    const openglframework::Matrix4& worldToCameraMatrix);
};

// Return a pointer to the collision body of the box
inline rp3d::CollisionBody* Sphere::getCollisionBody() {
    return mRigidBody;
}

// Return a pointer to the rigid body of the box
inline rp3d::RigidBody* Sphere::getRigidBody() {
    return static_cast<rp3d::RigidBody*>(mRigidBody);
}

#endif
