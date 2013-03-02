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

#ifndef COLLISION_BODY_H
#define COLLISION_BODY_H

// Libraries
#include <stdexcept>
#include <cassert>
#include "Body.h"
#include "../mathematics/Transform.h"
#include "../collision/shapes/AABB.h"
#include "../collision/shapes/CollisionShape.h"
#include "../configuration.h"

// Namespace reactphysics3d
namespace reactphysics3d {


/*  -------------------------------------------------------------------
    Class CollisionBody :
        This class represents a body that is able to collide with others
        bodies. This class inherits from the Body class.
    -------------------------------------------------------------------
*/
class CollisionBody : public Body {

    protected :

        // -------------------- Attributes -------------------- //

        // Collision shape of the body
        CollisionShape* mCollisionShape;

        // Position and orientation of the body
        Transform mTransform;

        // Last position and orientation of the body
        Transform mOldTransform;

        // Interpolation factor used for the state interpolation
        decimal mInterpolationFactor;

        // True if the body is active (not sleeping)
        bool mIsActive;

        // True if the body is able to move
        bool mIsMotionEnabled;

        // True if the body can collide with others bodies
        bool mIsCollisionEnabled;

        // AABB for Broad-Phase collision detection
        AABB* mAabb;

        // True if the body has moved during the last frame
        bool mHasMoved;

        // -------------------- Methods -------------------- //

        // Private copy-constructor
        CollisionBody(const CollisionBody& body);

        // Private assignment operator
        CollisionBody& operator=(const CollisionBody& body);

    public :

        // -------------------- Methods -------------------- //

        // Constructor
        CollisionBody(const Transform& transform, CollisionShape* collisionShape, bodyindex id);

        // Destructor
        virtual ~CollisionBody();

        // Return true if the body has moved during the last frame
        bool getHasMoved() const;

        // Set the hasMoved variable (true if the body has moved during the last frame)
        void setHasMoved(bool hasMoved);

        // Return the collision shape
        CollisionShape* getCollisionShape() const;

        // Set the collision shape
        void setCollisionShape(CollisionShape* collisionShape);

        // Return true for an active body
        bool getIsActive() const;

        // Set the isActive variable
        void setIsActive(bool isActive);

        // Return the current position and orientation
        const Transform& getTransform() const;

        // Set the current position and orientation
        void setTransform(const Transform& transform);

        // Return the AAABB of the body
        const AABB* getAABB() const;

        // Return the interpolated transform for rendering
        Transform getInterpolatedTransform() const;

        // Set the interpolation factor of the body
        void setInterpolationFactor(decimal factor);

        // Return if the rigid body can move
        bool getIsMotionEnabled() const;

        // Set the value to true if the body can move
        void setIsMotionEnabled(bool isMotionEnabled);

        // Return true if the body can collide with others bodies
        bool getIsCollisionEnabled() const;

        // Set the isCollisionEnabled value
        void setIsCollisionEnabled(bool isCollisionEnabled);

        // Update the old transform with the current one
        void updateOldTransform();

        // Update the Axis-Aligned Bounding Box coordinates
        void updateAABB();
};

// Return true if the body has moved during the last frame
inline bool CollisionBody::getHasMoved() const {
    return mHasMoved;
}

// Set the hasMoved variable (true if the body has moved during the last frame)
inline void CollisionBody::setHasMoved(bool hasMoved) {
    mHasMoved = hasMoved;
}

// Return the collision shape
inline CollisionShape* CollisionBody::getCollisionShape() const {
    assert(mCollisionShape);
    return mCollisionShape;
}

// Set the collision shape
inline void CollisionBody::setCollisionShape(CollisionShape* collisionShape) {
    assert(collisionShape);
    mCollisionShape = collisionShape;
}

// Return true if the body is active
inline bool CollisionBody::getIsActive() const {
    return mIsActive;
}

// Set the isActive variable
inline void CollisionBody::setIsActive(bool isActive) {
    mIsActive = isActive;
}

// Return the interpolated transform for rendering
inline Transform CollisionBody::getInterpolatedTransform() const {
    return Transform::interpolateTransforms(mOldTransform, mTransform, mInterpolationFactor);
}

// Set the interpolation factor of the body
inline void CollisionBody::setInterpolationFactor(decimal factor) {
    // Set the factor
    mInterpolationFactor = factor;
}

// Return if the rigid body can move
inline bool CollisionBody::getIsMotionEnabled() const {
    return mIsMotionEnabled;
}

// Set the value to true if the body can move
inline void CollisionBody::setIsMotionEnabled(bool isMotionEnabled) {
    mIsMotionEnabled = isMotionEnabled;
}

// Return the current position and orientation
inline const Transform& CollisionBody::getTransform() const {
    return mTransform;
}

// Set the current position and orientation
inline void CollisionBody::setTransform(const Transform& transform) {

    // Check if the body has moved
    if (this->mTransform != transform) {
        mHasMoved = true;
    }

    mTransform = transform;
}

// Return the AAABB of the body
inline const AABB* CollisionBody::getAABB() const {
    return mAabb;
}

 // Return true if the body can collide with others bodies
inline bool CollisionBody::getIsCollisionEnabled() const {
    return mIsCollisionEnabled;
}

// Set the isCollisionEnabled value
inline void CollisionBody::setIsCollisionEnabled(bool isCollisionEnabled) {
    mIsCollisionEnabled = isCollisionEnabled;
}

// Update the old transform with the current one
// This is used to compute the interpolated position and orientation of the body
inline void CollisionBody::updateOldTransform() {
    mOldTransform = mTransform;
}

// Update the rigid body in order to reflect a change in the body state
inline void CollisionBody::updateAABB() {

    // TODO : An AABB should not be updated every frame but only if the body has moved

    // Update the AABB
    mAabb->update(mTransform, mCollisionShape->getLocalExtents(OBJECT_MARGIN));
}

}

 #endif
