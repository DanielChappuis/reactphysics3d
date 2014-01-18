/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2010-2013 Daniel Chappuis                                       *
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

#ifndef CYLINDER_H
#define CYLINDER_H

// Libraries
#include "openglframework.h"
#include "reactphysics3d.h"

// Class Cylinder
class Cylinder : public openglframework::Mesh {

    private :

        // -------------------- Attributes -------------------- //

        /// Radius of the cylinder
        float mRadius;

        /// Height of the cylinder
        float mHeight;

        /// Rigid body used to simulate the dynamics of the cylinder
        rp3d::RigidBody* mRigidBody;

        /// Scaling matrix (applied to a sphere to obtain the correct cylinder dimensions)
        openglframework::Matrix4 mScalingMatrix;

        // -------------------- Methods -------------------- //

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        Cylinder(float radius, float height, const openglframework::Vector3& position,
             float mass, rp3d::DynamicsWorld* dynamicsWorld, const std::string &meshFolderPath);

        /// Destructor
        ~Cylinder();

        /// Return a pointer to the rigid body of the cylinder
        rp3d::RigidBody* getRigidBody();

        /// Update the transform matrix of the cylinder
        void updateTransform();

        /// Render the cylinder at the correct position and with the correct orientation
        void render(openglframework::Shader& shader,
                    const openglframework::Matrix4& worldToCameraMatrix);
};

// Return a pointer to the rigid body of the cylinder
inline rp3d::RigidBody* Cylinder::getRigidBody() {
    return mRigidBody;
}

#endif
