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
ProxyShape::ProxyShape(CollisionBody* body, CollisionShape* shape, const Transform& transform, decimal mass)
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

// Raycast method with feedback information
/**
 * @param ray Ray to use for the raycasting
 * @param[out] raycastInfo Result of the raycasting that is valid only if the
 *             methods returned true
 * @return True if the ray hit the collision shape
 */
bool ProxyShape::raycast(const Ray& ray, RaycastInfo& raycastInfo) {

    // If the corresponding body is not active, it cannot be hit by rays
    if (!mBody->isActive()) return false;

    // Convert the ray into the local-space of the collision shape
    const Transform localToWorldTransform = getLocalToWorldTransform();
    const Transform worldToLocalTransform = localToWorldTransform.getInverse();
    Ray rayLocal(worldToLocalTransform * ray.point1,
                 worldToLocalTransform * ray.point2,
                 ray.maxFraction);

    bool isHit = mCollisionShape->raycast(rayLocal, raycastInfo, this);

    // Convert the raycast info into world-space
    raycastInfo.worldPoint = localToWorldTransform * raycastInfo.worldPoint;
    raycastInfo.worldNormal = localToWorldTransform.getOrientation() * raycastInfo.worldNormal;
    raycastInfo.worldNormal.normalize();

    return isHit;
}
