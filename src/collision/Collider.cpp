/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2020 Daniel Chappuis                                       *
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
#include <reactphysics3d/collision/Collider.h>
#include <reactphysics3d/utils/Logger.h>
#include <reactphysics3d/collision/RaycastInfo.h>
#include <reactphysics3d/memory/MemoryManager.h>
#include <reactphysics3d/engine/PhysicsWorld.h>
#include <reactphysics3d/engine/PhysicsCommon.h>

using namespace reactphysics3d;

// Constructor
/**
 * @param body Pointer to the parent body
 * @param shape Pointer to the collision shape
 * @param transform Transformation from collision shape local-space to body local-space
 * @param mass Mass of the collision shape (in kilograms)
 */
Collider::Collider(Entity entity, CollisionBody* body, MemoryManager& memoryManager)
           :mMemoryManager(memoryManager), mEntity(entity), mBody(body),
            mMaterial(body->mWorld.mConfig.defaultFrictionCoefficient, body->mWorld.mConfig.defaultRollingRestistance,
                      body->mWorld.mConfig.defaultBounciness), mUserData(nullptr) {

}

// Destructor
Collider::~Collider() {

}

// Return true if a point is inside the collision shape
/**
 * @param worldPoint Point to test in world-space coordinates
 * @return True if the point is inside the collision shape
 */
bool Collider::testPointInside(const Vector3& worldPoint) {
    const Transform localToWorld = mBody->mWorld.mTransformComponents.getTransform(mBody->getEntity()) *
                                   mBody->mWorld.mCollidersComponents.getLocalToBodyTransform(mEntity);
    const Vector3 localPoint = localToWorld.getInverse() * worldPoint;
    const CollisionShape* collisionShape = mBody->mWorld.mCollidersComponents.getCollisionShape(mEntity);
    return collisionShape->testPointInside(localPoint, this);
}

// Set the collision category bits
/**
 * @param collisionCategoryBits The collision category bits mask of the collider
 */
void Collider::setCollisionCategoryBits(unsigned short collisionCategoryBits) {

    mBody->mWorld.mCollidersComponents.setCollisionCategoryBits(mEntity, collisionCategoryBits);

    int broadPhaseId = mBody->mWorld.mCollidersComponents.getBroadPhaseId(mEntity);

    // Ask the broad-phase collision detection to test this collider next frame
    mBody->mWorld.mCollisionDetection.askForBroadPhaseCollisionCheck(this);

    RP3D_LOG(mBody->mWorld.mConfig.worldName, Logger::Level::Information, Logger::Category::Collider,
             "Collider " + std::to_string(broadPhaseId) + ": Set collisionCategoryBits=" +
             std::to_string(collisionCategoryBits),  __FILE__, __LINE__);
}

// Set the collision bits mask
/**
 * @param collideWithMaskBits The bits mask that specifies with which collision category this shape will collide
 */
void Collider::setCollideWithMaskBits(unsigned short collideWithMaskBits) {

    mBody->mWorld.mCollidersComponents.setCollideWithMaskBits(mEntity, collideWithMaskBits);

    int broadPhaseId = mBody->mWorld.mCollidersComponents.getBroadPhaseId(mEntity);

    // Ask the broad-phase collision detection to test this collider next frame
    mBody->mWorld.mCollisionDetection.askForBroadPhaseCollisionCheck(this);

    RP3D_LOG(mBody->mWorld.mConfig.worldName, Logger::Level::Information, Logger::Category::Collider,
             "Collider" + std::to_string(broadPhaseId) + ": Set collideWithMaskBits=" +
             std::to_string(collideWithMaskBits),  __FILE__, __LINE__);
}

// Set the local to  body transform
/**
 * @param transform The transform from local-space of the collider into the local-space of the body
 */
void Collider::setLocalToBodyTransform(const Transform& transform) {

    mBody->mWorld.mCollidersComponents.setLocalToBodyTransform(mEntity, transform);

    // Update the local-to-world transform
    const Transform& bodyTransform = mBody->mWorld.mTransformComponents.getTransform(mBody->getEntity());
    mBody->mWorld.mCollidersComponents.setLocalToWorldTransform(mEntity, bodyTransform * transform);

    RigidBody* rigidBody = static_cast<RigidBody*>(mBody);
    if (rigidBody != nullptr) {
        mBody->mWorld.mRigidBodyComponents.setIsSleeping(mBody->getEntity(), false);
    }

    mBody->mWorld.mCollisionDetection.updateCollider(mEntity, 0);

    RP3D_LOG(mBody->mWorld.mConfig.worldName, Logger::Level::Information, Logger::Category::Collider,
             "Collider " + std::to_string(getBroadPhaseId()) + ": Set localToBodyTransform=" +
             transform.to_string(),  __FILE__, __LINE__);
}

// Return the AABB of the collider in world-space
/**
 * @return The AABB of the collider in world-space
 */
const AABB Collider::getWorldAABB() const {
    AABB aabb;
    CollisionShape* collisionShape = mBody->mWorld.mCollidersComponents.getCollisionShape(mEntity);
    collisionShape->computeAABB(aabb, getLocalToWorldTransform());
    return aabb;
}

// Return a pointer to the collision shape
/**
 * @return Pointer to the collision shape
 */
CollisionShape* Collider::getCollisionShape() {
    return mBody->mWorld.mCollidersComponents.getCollisionShape(mEntity);
}

// Return a const pointer to the collision shape
/**
 * @return Pointer to the collision shape
 */
const CollisionShape* Collider::getCollisionShape() const {
    return mBody->mWorld.mCollidersComponents.getCollisionShape(mEntity);
}

// Return the broad-phase id
int Collider::getBroadPhaseId() const {
    return mBody->mWorld.mCollidersComponents.getBroadPhaseId(mEntity);
}

// Return the local to parent body transform
/**
 * @return The transformation that transforms the local-space of the collider
 *         to the local-space of the body
 */
const Transform& Collider::getLocalToBodyTransform() const {
    return mBody->mWorld.mCollidersComponents.getLocalToBodyTransform(mEntity);
}

// Raycast method with feedback information
/**
 * @param ray Ray to use for the raycasting
 * @param[out] raycastInfo Result of the raycasting that is valid only if the
 *             methods returned true
 * @return True if the ray hits the collision shape
 */
bool Collider::raycast(const Ray& ray, RaycastInfo& raycastInfo) {

    // If the corresponding body is not active, it cannot be hit by rays
    if (!mBody->isActive()) return false;

    // Convert the ray into the local-space of the collision shape
    const Transform localToWorldTransform = mBody->mWorld.mCollidersComponents.getLocalToWorldTransform(mEntity);
    const Transform worldToLocalTransform = localToWorldTransform.getInverse();
    Ray rayLocal(worldToLocalTransform * ray.point1,
                 worldToLocalTransform * ray.point2,
                 ray.maxFraction);

    const CollisionShape* collisionShape = mBody->mWorld.mCollidersComponents.getCollisionShape(mEntity);
    bool isHit = collisionShape->raycast(rayLocal, raycastInfo, this, mMemoryManager.getPoolAllocator());

    // Convert the raycast info into world-space
    raycastInfo.worldPoint = localToWorldTransform * raycastInfo.worldPoint;
    raycastInfo.worldNormal = localToWorldTransform.getOrientation() * raycastInfo.worldNormal;
    raycastInfo.worldNormal.normalize();

    return isHit;
}

// Return the collision category bits
/**
 * @return The collision category bits mask of the collider
 */
unsigned short Collider::getCollisionCategoryBits() const {
    return mBody->mWorld.mCollidersComponents.getCollisionCategoryBits(mEntity);
}

// Return the collision bits mask
/**
 * @return The bits mask that specifies with which collision category this shape will collide
 */
unsigned short Collider::getCollideWithMaskBits() const {
    return mBody->mWorld.mCollidersComponents.getCollideWithMaskBits(mEntity);
}

// Notify the collider that the size of the collision shape has been changed by the user
void Collider::setHasCollisionShapeChangedSize(bool hasCollisionShapeChangedSize) {
    mBody->mWorld.mCollidersComponents.setHasCollisionShapeChangedSize(mEntity, hasCollisionShapeChangedSize);
}

// Set a new material for this rigid body
/**
 * @param material The material you want to set to the body
 */
void Collider::setMaterial(const Material& material) {

    mMaterial = material;

    RP3D_LOG(mBody->mWorld.mConfig.worldName, Logger::Level::Information, Logger::Category::Collider,
             "Collider " + std::to_string(mEntity.id) + ": Set Material" + mMaterial.to_string(),  __FILE__, __LINE__);
}

// Return the local to world transform
/**
 * @return The transformation that transforms the local-space of the collision
 *         shape to the world-space
 */
const Transform Collider::getLocalToWorldTransform() const {
    return mBody->mWorld.mCollidersComponents.getLocalToWorldTransform(mEntity);
}

// Return true if the collider is a trigger
/**
 * @return True if this collider is a trigger and false otherwise
 */
bool Collider::getIsTrigger() const {
   return mBody->mWorld.mCollidersComponents.getIsTrigger(mEntity);
}

// Set whether the collider is a trigger
/**
 * @param isTrigger True if you want to set this collider as a trigger and false otherwise
 */
void Collider::setIsTrigger(bool isTrigger) const {
   mBody->mWorld.mCollidersComponents.setIsTrigger(mEntity, isTrigger);
}

#ifdef IS_RP3D_PROFILING_ENABLED


// Set the profiler
void Collider::setProfiler(Profiler* profiler) {

    mProfiler = profiler;

    CollisionShape* collisionShape = mBody->mWorld.mCollidersComponents.getCollisionShape(mEntity);
    collisionShape->setProfiler(profiler);
}

#endif

