#ifndef REACTPHYSICS3D_PROXY_SHAPE_H
#define REACTPHYSICS3D_PROXY_SHAPE_H

// Libraries
#include "body/CollisionBody.h"
#include "shapes/CollisionShape.h"

namespace  reactphysics3d {

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

        /// Return true if a point is inside the collision shape
        bool testPointInside(const Vector3& worldPoint);

        /// Raycast method
        bool raycast(const Ray& ray, decimal distance = RAYCAST_INFINITY_DISTANCE);

        /// Raycast method with feedback information
        bool raycast(const Ray& ray, RaycastInfo& raycastInfo,
                     decimal distance = RAYCAST_INFINITY_DISTANCE);

        // -------------------- Friendship -------------------- //

        friend class OverlappingPair;
        friend class CollisionBody;
        friend class RigidBody;
        friend class BroadPhaseAlgorithm;
        friend class DynamicAABBTree;
        friend class CollisionDetection;
        friend class EPAAlgorithm;
        friend class GJKAlgorithm;
        friend class ConvexMeshShape;
};

/// Return the collision shape
inline const CollisionShape* ProxyShape::getCollisionShape() const {
    return mCollisionShape;
}

// Return the parent body
inline CollisionBody* ProxyShape::getBody() const {
    return mBody;
}

// Return the mass of the collision shape
inline decimal ProxyShape::getMass() const {
    return mMass;
}

// Return a pointer to the user data attached to this body
inline void* ProxyShape::getUserData() const {
    return mUserData;
}

// Attach user data to this body
inline void ProxyShape::setUserData(void* userData) {
    mUserData = userData;
}

// Return the local to parent body transform
inline const Transform& ProxyShape::getLocalToBodyTransform() const {
    return mLocalToBodyTransform;
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

// Raycast method
inline bool ProxyShape::raycast(const Ray& ray, decimal distance) {
    return mCollisionShape->raycast(ray, distance);
}

// Raycast method with feedback information
inline bool ProxyShape::raycast(const Ray& ray, RaycastInfo& raycastInfo, decimal distance) {
    return mCollisionShape->raycast(ray, raycastInfo, distance);
}

}

#endif
