
// Libraries
#include "ProxyShape.h"

using namespace reactphysics3d;

// Constructor
ProxyShape::ProxyShape(CollisionBody* body, CollisionShape* shape, const Transform& transform,
                       decimal mass)
           :mBody(body), mCollisionShape(shape), mLocalToBodyTransform(transform), mMass(mass),
            mNext(NULL), mBroadPhaseID(-1), mCachedCollisionData(NULL), mUserData(NULL) {

}

// Destructor
ProxyShape::~ProxyShape() {

    // Release the cached collision data memory
    if (mCachedCollisionData != NULL) {
        free(mCachedCollisionData);
    }
}

// Return true if a point is inside the collision shape
bool ProxyShape::testPointInside(const Vector3& worldPoint) {
    const Transform localToWorld = mBody->getTransform() * mLocalToBodyTransform;
    const Vector3 localPoint = localToWorld.getInverse() * worldPoint;
    return mCollisionShape->testPointInside(localPoint);
}

