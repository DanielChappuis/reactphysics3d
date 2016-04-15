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

#ifndef REACTPHYSICS3D_COLLISION_BODY_H
#define REACTPHYSICS3D_COLLISION_BODY_H

// Libraries
#include <stdexcept>
#include <cassert>
#include "Body.h"
#include "mathematics/Transform.h"
#include "collision/shapes/AABB.h"
#include "collision/shapes/CollisionShape.h"
#include "collision/RaycastInfo.h"
#include "memory/MemoryAllocator.h"
#include "configuration.h"

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

        /// Update the broad-phase state for this body (because it has moved for instance)
        virtual void updateBroadPhaseState() const;

        /// Update the broad-phase state of a proxy collision shape of the body
        void updateProxyShapeInBroadPhase(ProxyShape* proxyShape, bool forceReinsert = false) const;

        /// Ask the broad-phase to test again the collision shapes of the body for collision
        /// (as if the body has moved).
        void askForBroadPhaseCollisionCheck() const;

        /// Reset the mIsAlreadyInIsland variable of the body and contact manifolds
        int resetIsAlreadyInIslandAndCountManifolds();

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

        /// Set whether or not the body is active
        virtual void setIsActive(bool isActive);

        /// Return the current position and orientation
        const Transform& getTransform() const;

        /// Set the current position and orientation
        virtual void setTransform(const Transform& transform);

        /// Add a collision shape to the body.
        virtual ProxyShape* addCollisionShape(CollisionShape* collisionShape,
                                              const Transform& transform);

        /// Remove a collision shape from the body
        virtual void removeCollisionShape(const ProxyShape* proxyShape);

        /// Return the first element of the linked list of contact manifolds involving this body
        const ContactManifoldListElement* getContactManifoldsList() const;

        /// Return true if a point is inside the collision body
        bool testPointInside(const Vector3& worldPoint) const;

        /// Raycast method with feedback information
        bool raycast(const Ray& ray, RaycastInfo& raycastInfo);

        /// Compute and return the AABB of the body by merging all proxy shapes AABBs
        AABB getAABB() const;

        /// Return the linked list of proxy shapes of that body
        ProxyShape* getProxyShapesList();

        /// Return the linked list of proxy shapes of that body
        const ProxyShape* getProxyShapesList() const;

        /// Return the world-space coordinates of a point given the local-space coordinates of the body
        Vector3 getWorldPoint(const Vector3& localPoint) const;

        /// Return the world-space vector of a vector given in local-space coordinates of the body
        Vector3 getWorldVector(const Vector3& localVector) const;

        /// Return the body local-space coordinates of a point given in the world-space coordinates
        Vector3 getLocalPoint(const Vector3& worldPoint) const;

        /// Return the body local-space coordinates of a vector given in the world-space coordinates
        Vector3 getLocalVector(const Vector3& worldVector) const;

        // -------------------- Friendship -------------------- //

        friend class CollisionWorld;
        friend class DynamicsWorld;
        friend class CollisionDetection;
        friend class BroadPhaseAlgorithm;
        friend class ConvexMeshShape;
        friend class ProxyShape;
};

// Return the type of the body
/**
 * @return the type of the body (STATIC, KINEMATIC, DYNAMIC)
 */
inline BodyType CollisionBody::getType() const {
    return mType;
}

// Set the type of the body
/// The type of the body can either STATIC, KINEMATIC or DYNAMIC as described bellow:
/// STATIC : A static body has infinite mass, zero velocity but the position can be
///          changed manually. A static body does not collide with other static or kinematic bodies.
/// KINEMATIC : A kinematic body has infinite mass, the velocity can be changed manually and its
///             position is computed by the physics engine. A kinematic body does not collide with
///             other static or kinematic bodies.
/// DYNAMIC : A dynamic body has non-zero mass, non-zero velocity determined by forces and its
///           position is determined by the physics engine. A dynamic body can collide with other
///           dynamic, static or kinematic bodies.
/**
 * @param type The type of the body (STATIC, KINEMATIC, DYNAMIC)
 */
inline void CollisionBody::setType(BodyType type) {
    mType = type;

    if (mType == STATIC) {

        // Update the broad-phase state of the body
        updateBroadPhaseState();
    }
}

// Return the current position and orientation
/**
 * @return The current transformation of the body that transforms the local-space
 *         of the body into world-space
 */
inline const Transform& CollisionBody::getTransform() const {
    return mTransform;
}

// Set the current position and orientation
/**
 * @param transform The transformation of the body that transforms the local-space
 *                  of the body into world-space
 */
inline void CollisionBody::setTransform(const Transform& transform) {

    // Update the transform of the body
    mTransform = transform;

    // Update the broad-phase state of the body
    updateBroadPhaseState();
}

// Return the first element of the linked list of contact manifolds involving this body
/**
 * @return A pointer to the first element of the linked-list with the contact
 *         manifolds of this body
 */
inline const ContactManifoldListElement* CollisionBody::getContactManifoldsList() const {
    return mContactManifoldsList;
}

// Return the linked list of proxy shapes of that body
/**
* @return The pointer of the first proxy shape of the linked-list of all the
*         proxy shapes of the body
*/
inline ProxyShape* CollisionBody::getProxyShapesList() {
    return mProxyCollisionShapes;
}

// Return the linked list of proxy shapes of that body
/**
* @return The pointer of the first proxy shape of the linked-list of all the
*         proxy shapes of the body
*/
inline const ProxyShape* CollisionBody::getProxyShapesList() const {
    return mProxyCollisionShapes;
}

// Return the world-space coordinates of a point given the local-space coordinates of the body
/**
* @param localPoint A point in the local-space coordinates of the body
* @return The point in world-space coordinates
*/
inline Vector3 CollisionBody::getWorldPoint(const Vector3& localPoint) const {
    return mTransform * localPoint;
}

// Return the world-space vector of a vector given in local-space coordinates of the body
/**
* @param localVector A vector in the local-space coordinates of the body
* @return The vector in world-space coordinates
*/
inline Vector3 CollisionBody::getWorldVector(const Vector3& localVector) const {
    return mTransform.getOrientation() * localVector;
}

// Return the body local-space coordinates of a point given in the world-space coordinates
/**
* @param worldPoint A point in world-space coordinates
* @return The point in the local-space coordinates of the body
*/
inline Vector3 CollisionBody::getLocalPoint(const Vector3& worldPoint) const {
    return mTransform.getInverse() * worldPoint;
}

// Return the body local-space coordinates of a vector given in the world-space coordinates
/**
* @param worldVector A vector in world-space coordinates
* @return The vector in the local-space coordinates of the body
*/
inline Vector3 CollisionBody::getLocalVector(const Vector3& worldVector) const {
    return mTransform.getOrientation().getInverse() * worldVector;
}

}

 #endif
