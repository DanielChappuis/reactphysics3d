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

#ifndef PHYSICSOBJECT_H
#define PHYSICSOBJECT_H

// Libraries
#include "openglframework.h"
#include <reactphysics3d/reactphysics3d.h>

// Class PhysicsObject
class PhysicsObject : public openglframework::Mesh {

    protected:

        /// Reference to the physics common object
        rp3d::PhysicsCommon& mPhysicsCommon;

        /// Body used to simulate the dynamics of the box
        rp3d::CollisionBody* mBody;

        /// Previous transform of the body (for interpolation)
        rp3d::Transform mPreviousTransform;

        /// Main color of the box
        openglframework::Color mColor;

        /// Sleeping color
        openglframework::Color mSleepingColor;

        // Compute the new transform matrix
        openglframework::Matrix4 computeTransform(float interpolationFactor,
                                                 const openglframework::Matrix4 &scalingMatrix);

    public:

        /// Constructor
        PhysicsObject(rp3d::PhysicsCommon& physicsCommon);

        /// Constructor
        PhysicsObject(rp3d::PhysicsCommon& physicsCommon, const std::string& meshPath);

        /// Update the transform matrix of the object
        virtual void updateTransform(float interpolationFactor)=0;

		/// Render the sphere at the correct position and with the correct orientation
        virtual void render(openglframework::Shader& shader, const openglframework::Matrix4& worldToCameraMatrix)=0;

        /// Set the color of the box
        void setColor(const openglframework::Color& color);

        /// Set the sleeping color of the box
        void setSleepingColor(const openglframework::Color& color);

        /// Get the transform
        const rp3d::Transform& getTransform() const;

        /// Set the transform
        void setTransform(const rp3d::Transform& transform);

        /// Return a pointer to the collision body of the box
        reactphysics3d::CollisionBody* getCollisionBody();

        /// Return a pointer to the rigid body of the box
        reactphysics3d::RigidBody* getRigidBody();
};

// Set the color of the box
inline void PhysicsObject::setColor(const openglframework::Color& color) {
    mColor = color;
}

// Set the sleeping color of the box
inline void PhysicsObject::setSleepingColor(const openglframework::Color& color) {
    mSleepingColor = color;
}

// Get the transform
inline const rp3d::Transform& PhysicsObject::getTransform() const {
    return mBody->getTransform();
}

// Return a pointer to the collision body of the box
inline rp3d::CollisionBody* PhysicsObject::getCollisionBody() {
    return mBody;
}

// Return a pointer to the rigid body of the box (NULL if it's not a rigid body)
inline rp3d::RigidBody* PhysicsObject::getRigidBody() {
    return dynamic_cast<rp3d::RigidBody*>(mBody);
}

#endif

