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

#ifndef REACTPHYSICS3D_PROXY_SHAPE_H
#define REACTPHYSICS3D_PROXY_SHAPE_H

// Libraries
#include "reactphysics3d/body/CollisionBody.h"
#include "reactphysics3d/collision/shapes/CollisionShape.h"

/// ReactPhysics3D Namespace
namespace reactphysics3d {

// Class ProxyShape
/**
 * The CollisionShape instances are supposed to be unique for memory optimization. For instance,
 * consider two rigid bodies with the same sphere collision shape. In this situation, we will have
 * a unique instance of SphereShape but we need to differentiate between the two instances during
 * the collision detection. They do not have the same position in the world and they do not
 * belong to the same rigid body. The ProxyShape class is used for that purpose by attaching a
 * rigid body with one of its collision shape. A body can have multiple proxy shapes (one for
 * each collision shape attached to the body).
 */
class ProxyShape {

    protected:

        // -------------------- Attributes -------------------- //

        /// Pointer to the parent body
        CollisionBody* mBody;

        /// Internal collision shape
        CollisionShape* mCollisionShape;

        /// Local-space to parent body-space transform (does not change over time)
        const Transform mLocalToBodyTransform;

        /// Mass (in kilogramms) of the corresponding collision shape
        decimal mMass;

        /// Pointer to the next proxy shape of the body (linked list)
        ProxyShape* mNext;

        /// Broad-phase ID (node ID in the dynamic AABB tree)
        int mBroadPhaseID;

        /// Cached collision data
        void* mCachedCollisionData;

        /// Pointer to user data
        void* mUserData;

        /// Bits used to define the collision category of this shape.
        /// You can set a single bit to one to define a category value for this
        /// shape. This value is one (0x0001) by default. This variable can be used
        /// together with the mCollideWithMaskBits variable so that given
        /// categories of shapes collide with each other and do not collide with
        /// other categories.
        unsigned short mCollisionCategoryBits;

        /// Bits mask used to state which collision categories this shape can
        /// collide with. This value is 0xFFFF by default. It means that this
        /// proxy shape will collide with every collision categories by default.
        unsigned short mCollideWithMaskBits;

        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        ProxyShape(const ProxyShape& proxyShape);

        /// Private assignment operator
        ProxyShape& operator=(const ProxyShape& proxyShape);

        // Return a local support point in a given direction with the object margin
        Vector3 getLocalSupportPointWithMargin(const Vector3& direction);

        /// Return a local support point in a given direction without the object margin.
        Vector3 getLocalSupportPointWithoutMargin(const Vector3& direction);

        /// Return the collision shape margin
        decimal getMargin() const;

    public:

        // -------------------- Methods -------------------- //

        /// Constructor
        ProxyShape(CollisionBody* body, CollisionShape* shape,
                   const Transform& transform, decimal mass);

        /// Destructor
        ~ProxyShape();

        /// Return the collision shape
        const CollisionShape* getCollisionShape() const;

        /// Return the parent body
        CollisionBody* getBody() const;

        /// Return the mass of the collision shape
        decimal getMass() const;

        /// Return a pointer to the user data attached to this body
        void* getUserData() const;

        /// Attach user data to this body
        void setUserData(void* userData);

        /// Return the local to parent body transform
        const Transform& getLocalToBodyTransform() const;

        /// Return the local to world transform
        const Transform getLocalToWorldTransform() const;

        /// Return true if a point is inside the collision shape
        bool testPointInside(const Vector3& worldPoint);

        /// Raycast method with feedback information
        bool raycast(const Ray& ray, RaycastInfo& raycastInfo);

        /// Return the collision bits mask
        unsigned short getCollideWithMaskBits() const;

        /// Set the collision bits mask
        void setCollideWithMaskBits(unsigned short collideWithMaskBits);

        /// Return the collision category bits
        unsigned short getCollisionCategoryBits() const;

        /// Set the collision category bits
        void setCollisionCategoryBits(unsigned short collisionCategoryBits);

        /// Return the next proxy shape in the linked list of proxy shapes
        ProxyShape* getNext();

        /// Return the next proxy shape in the linked list of proxy shapes
        const ProxyShape* getNext() const;

        // -------------------- Friendship -------------------- //

        friend class OverlappingPair;
        friend class CollisionBody;
        friend class RigidBody;
        friend class BroadPhaseAlgorithm;
        friend class DynamicAABBTree;
        friend class CollisionDetection;
        friend class CollisionWorld;
        friend class DynamicsWorld;
        friend class EPAAlgorithm;
        friend class GJKAlgorithm;
        friend class ConvexMeshShape;
};

/// Return the collision shape
/**
 * @return Pointer to the internal collision shape
 */
inline const CollisionShape* ProxyShape::getCollisionShape() const {
    return mCollisionShape;
}

// Return the parent body
/**
 * @return Pointer to the parent body
 */
inline CollisionBody* ProxyShape::getBody() const {
    return mBody;
}

// Return the mass of the collision shape
/**
 * @return Mass of the collision shape (in kilograms)
 */
inline decimal ProxyShape::getMass() const {
    return mMass;
}

// Return a pointer to the user data attached to this body
/**
 * @return A pointer to the user data stored into the proxy shape
 */
inline void* ProxyShape::getUserData() const {
    return mUserData;
}

// Attach user data to this body
/**
 * @param userData Pointer to the user data you want to store within the proxy shape
 */
inline void ProxyShape::setUserData(void* userData) {
    mUserData = userData;
}

// Return the local to parent body transform
/**
 * @return The transformation that transforms the local-space of the collision shape
 *         to the local-space of the parent body
 */
inline const Transform& ProxyShape::getLocalToBodyTransform() const {
    return mLocalToBodyTransform;
}

// Return the local to world transform
/**
 * @return The transformation that transforms the local-space of the collision
 *         shape to the world-space
 */
inline const Transform ProxyShape::getLocalToWorldTransform() const {
    return mBody->mTransform * mLocalToBodyTransform;
}

// Return a local support point in a given direction with the object margin
inline Vector3 ProxyShape::getLocalSupportPointWithMargin(const Vector3& direction) {
    return mCollisionShape->getLocalSupportPointWithMargin(direction, &mCachedCollisionData);
}

// Return a local support point in a given direction without the object margin.
inline Vector3 ProxyShape::getLocalSupportPointWithoutMargin(const Vector3& direction) {
    return mCollisionShape->getLocalSupportPointWithoutMargin(direction, &mCachedCollisionData);
}

// Return the collision shape margin
inline decimal ProxyShape::getMargin() const {
    return mCollisionShape->getMargin();
}

// Raycast method with feedback information
/**
 * @param ray Ray to use for the raycasting
 * @param[out] raycastInfo Result of the raycasting that is valid only if the
 *             methods returned true
 * @return True if the ray hit the collision shape
 */
inline bool ProxyShape::raycast(const Ray& ray, RaycastInfo& raycastInfo) {

    // If the corresponding body is not active, it cannot be hit by rays
    if (!mBody->isActive()) return false;

    return mCollisionShape->raycast(ray, raycastInfo, this);
}

// Return the next proxy shape in the linked list of proxy shapes
/**
 * @return Pointer to the next proxy shape in the linked list of proxy shapes
 */
inline ProxyShape* ProxyShape::getNext() {
    return mNext;
}

// Return the next proxy shape in the linked list of proxy shapes
/**
 * @return Pointer to the next proxy shape in the linked list of proxy shapes
 */
inline const ProxyShape* ProxyShape::getNext() const {
    return mNext;
}

// Return the collision category bits
/**
 * @return The collision category bits mask of the proxy shape
 */
inline unsigned short ProxyShape::getCollisionCategoryBits() const {
    return mCollisionCategoryBits;
}

// Set the collision category bits
/**
 * @param collisionCategoryBits The collision category bits mask of the proxy shape
 */
inline void ProxyShape::setCollisionCategoryBits(unsigned short collisionCategoryBits) {
    mCollisionCategoryBits = collisionCategoryBits;
}

// Return the collision bits mask
/**
 * @return The bits mask that specifies with which collision category this shape will collide
 */
inline unsigned short ProxyShape::getCollideWithMaskBits() const {
    return mCollideWithMaskBits;
}

// Set the collision bits mask
/**
 * @param collideWithMaskBits The bits mask that specifies with which collision category this shape will collide
 */
inline void ProxyShape::setCollideWithMaskBits(unsigned short collideWithMaskBits) {
    mCollideWithMaskBits = collideWithMaskBits;
}

}

#endif
