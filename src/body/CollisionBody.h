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

#ifndef REACTPHYSICS3D_COLLISION_BODY_H
#define REACTPHYSICS3D_COLLISION_BODY_H

// Libraries
#include <stdexcept>
#include <cassert>
#include "Body.h"
#include "../mathematics/Transform.h"
#include "../collision/shapes/AABB.h"
#include "../collision/shapes/CollisionShape.h"
#include "../memory/MemoryAllocator.h"
#include "../configuration.h"

/// Namespace reactphysics3d
namespace reactphysics3d {

// Class declarations
struct ContactManifoldListElement;
class ProxyShape;
class CollisionWorld;

/// Enumeration for the type of a body
/// STATIC : A static body has infinite mass, zero velocity but the position can be
///          changed manually. A static body does not collide with other static or kinematic bodies.
/// KINEMATIC : A kinematic body has infinite mass, the velocity can be changed manually and its
///             position is computed by the physics engine. A kinematic body does not collide with
///             other static or kinematic bodies.
/// DYNAMIC : A dynamic body has non-zero mass, non-zero velocity determined by forces and its
///           position is determined by the physics engine. A dynamic body can collide with other
///           dynamic, static or kinematic bodies.
enum BodyType {STATIC, KINEMATIC, DYNAMIC};

// Class CollisionBody
/**
 * This class represents a body that is able to collide with others
 * bodies. This class inherits from the Body class.
 */
class CollisionBody : public Body {

    protected :

        // -------------------- Attributes -------------------- //

        /// Type of body (static, kinematic or dynamic)
        BodyType mType;

        /// Position and orientation of the body
        Transform mTransform;

        /// Last position and orientation of the body
        Transform mOldTransform;

        /// Interpolation factor used for the state interpolation
        decimal mInterpolationFactor;

        /// True if the body can collide with others bodies
        bool mIsCollisionEnabled;

        /// First element of the linked list of proxy collision shapes of this body
        ProxyShape* mProxyCollisionShapes;

        /// Number of collision shapes
        uint mNbCollisionShapes;

        /// First element of the linked list of contact manifolds involving this body
        ContactManifoldListElement* mContactManifoldsList;

        /// Reference to the world the body belongs to
        CollisionWorld& mWorld;

        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        CollisionBody(const CollisionBody& body);

        /// Private assignment operator
        CollisionBody& operator=(const CollisionBody& body);

        /// Reset the contact manifold lists
        void resetContactManifoldsList();

        /// Remove all the collision shapes
        void removeAllCollisionShapes();

        /// Update the old transform with the current one.
        void updateOldTransform();

        /// Update the broad-phase state for this body (because it has moved for instance)
        virtual void updateBroadPhaseState() const;

        /// Ask the broad-phase to test again the collision shapes of the body for collision
        /// (as if the body has moved).
        void askForBroadPhaseCollisionCheck() const;

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        CollisionBody(const Transform& transform, CollisionWorld& world, bodyindex id);

        /// Destructor
        virtual ~CollisionBody();

        /// Return the type of the body
        BodyType getType() const;

        /// Set the type of the body
        void setType(BodyType type);

        /// Return the current position and orientation
        const Transform& getTransform() const;

        /// Set the current position and orientation
        void setTransform(const Transform& transform);

        /// Add a collision shape to the body.
        const ProxyShape* addCollisionShape(const CollisionShape& collisionShape,
                                            const Transform& transform = Transform::identity());

        /// Remove a collision shape from the body
        virtual void removeCollisionShape(const ProxyShape* proxyShape);

        /// Return the interpolated transform for rendering
        Transform getInterpolatedTransform() const;

        /// Set the interpolation factor of the body
        void setInterpolationFactor(decimal factor);

        /// Return true if the body can collide with others bodies
        bool isCollisionEnabled() const;

        /// Enable/disable the collision with this body
        void enableCollision(bool isCollisionEnabled);

        /// Return the first element of the linked list of contact manifolds involving this body
        const ContactManifoldListElement* getContactManifoldsLists() const;

        // -------------------- Friendship -------------------- //

        friend class CollisionWorld;
        friend class DynamicsWorld;
        friend class CollisionDetection;
        friend class BroadPhaseAlgorithm;
};

// Return the type of the body
inline BodyType CollisionBody::getType() const {
    return mType;
}

// Set the type of the body
inline void CollisionBody::setType(BodyType type) {
    mType = type;

    if (mType == STATIC) {

        // Update the broad-phase state of the body
        updateBroadPhaseState();
    }
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

// Return the current position and orientation
inline const Transform& CollisionBody::getTransform() const {
    return mTransform;
}

// Set the current position and orientation
inline void CollisionBody::setTransform(const Transform& transform) {

    // Update the transform of the body
    mTransform = transform;

    // Update the broad-phase state of the body
    updateBroadPhaseState();
}

// Return true if the body can collide with others bodies
inline bool CollisionBody::isCollisionEnabled() const {
    return mIsCollisionEnabled;
}

// Enable/disable the collision with this body
inline void CollisionBody::enableCollision(bool isCollisionEnabled) {
    mIsCollisionEnabled = isCollisionEnabled;
}

// Update the old transform with the current one.
/// This is used to compute the interpolated position and orientation of the body
inline void CollisionBody::updateOldTransform() {
    mOldTransform = mTransform;
}

// Return the first element of the linked list of contact manifolds involving this body
inline const ContactManifoldListElement* CollisionBody::getContactManifoldsLists() const {
    return mContactManifoldsList;
}

}

 #endif
