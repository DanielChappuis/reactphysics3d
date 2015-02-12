
// Libraries
#include "ProxyShape.h"

using namespace reactphysics3d;

// Constructor
/**
 * @param body Pointer to the parent body
 * @param shape Pointer to the collision shape
 * @param transform Transformation from collision shape local-space to body local-space
 * @param mass Mass of the collision shape (in kilograms)
 */
ProxyShape::ProxyShape(CollisionBody* body, CollisionShape* shape, const Transform& transform,
                       decimal mass)
           :mBody(body), mCollisionShape(shape), mLocalToBodyTransform(transform), mMass(mass),
            mNext(NULL), mBroadPhaseID(-1), mCachedCollisionData(NULL), mUserData(NULL),
            mCollisionCategoryBits(0x0001), mCollideWithMaskBits(0xFFFF) {

}

// Destructor
ProxyShape::~ProxyShape() {

    // Release the cached collision data memory
    if (mCachedCollisionData != NULL) {
        free(mCachedCollisionData);
    }
}

// Return true if a point is inside the collision shape
/**
 * @param worldPoint Point to test in world-space coordinates
 * @return True if the point is inside the collision shape
 */
bool ProxyShape::testPointInside(const Vector3& worldPoint) {
    const Transform localToWorld = mBody->getTransform() * mLocalToBodyTransform;
    const Vector3 localPoint = localToWorld.getInverse() * worldPoint;
    return mCollisionShape->testPointInside(localPoint, this);
}

