/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2024 Daniel Chappuis                                       *
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
#include <reactphysics3d/body/Body.h>
#include <reactphysics3d/engine/PhysicsCommon.h>
#include <reactphysics3d/engine/PhysicsWorld.h>
#include <reactphysics3d/collision/ContactManifold.h>
#include <reactphysics3d/collision/RaycastInfo.h>
#include <reactphysics3d/utils/Logger.h>

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor
/**
 * @param world The reference to the physics world where the body is created
 * @param entity Entity of the body
 */
Body::Body(PhysicsWorld& world, Entity entity)
              : mEntity(entity), mWorld(world), mIsDebugEnabled(false)  {

#ifdef IS_RP3D_PROFILING_ENABLED

        mProfiler = nullptr;
#endif

}

// Destructor
Body::~Body() {

}

// Create a new collider and add it to the body
/// This method will return a pointer to a new collider. A collider is
/// an object with a collision shape that is attached to a body. It is possible to
/// attach multiple colliders to a given body. You can use the
/// returned collider to get and set information about the corresponding
/// collision shape for that body.
/**
 * @param collisionShape A pointer to the collision shape of the new collider
 * @param transform The transformation of the collider that transforms the
 *        local-space of the collider into the local-space of the body
 * @return A pointer to the collider that has been created
 */
Collider* Body::addCollider(CollisionShape* collisionShape, const Transform& transform) {

    // Create a new entity for the collider
    Entity colliderEntity = mWorld.mEntityManager.createEntity();

    // Check that the transform is valid
    if (!transform.isValid()) {
        RP3D_LOG(mWorld.mConfig.worldName, Logger::Level::Error, Logger::Category::Collider,
                 "Error when adding a collider: the init transform is not valid",  __FILE__, __LINE__);
    }
    assert(transform.isValid());

    // Create a new collider to attach the collision shape to the body
    Collider* collider = new (mWorld.mMemoryManager.allocate(MemoryManager::AllocationType::Pool,
                                      sizeof(Collider))) Collider(colliderEntity, this, mWorld.mMemoryManager);

    // Add the collider component to the entity of the body
    AABB shapeAABB = collisionShape->getLocalBounds();
    const Transform localToWorldTransform = mWorld.mTransformComponents.getTransform(mEntity) * transform;
    Material material(mWorld.mConfig.defaultFrictionCoefficient, mWorld.mConfig.defaultBounciness);
    ColliderComponents::ColliderComponent colliderComponent(mEntity, collider, shapeAABB,
                                                                  transform, collisionShape, 0x0001, 0xFFFF, localToWorldTransform, material);
    bool isActive = mWorld.mBodyComponents.getIsActive(mEntity);
    mWorld.mCollidersComponents.addComponent(colliderEntity, !isActive, colliderComponent);


    mWorld.mBodyComponents.addColliderToBody(mEntity, colliderEntity);

    // Assign the collider with the collision shape
    collisionShape->addCollider(collider);

#ifdef IS_RP3D_PROFILING_ENABLED


	// Set the profiler
    collider->setProfiler(mProfiler);

#endif

    // If the body is active
    if (isActive) {

        // Compute the world-space AABB of the new collision shape
        const AABB aabb = collisionShape->computeTransformedAABB(mWorld.mTransformComponents.getTransform(mEntity) * transform);

        // Notify the collision detection about this new collision shape
        mWorld.mCollisionDetection.addCollider(collider, aabb);
    }

    RP3D_LOG(mWorld.mConfig.worldName, Logger::Level::Information, Logger::Category::Body,
             "Body " + std::to_string(mEntity.id) + ": Collider " + std::to_string(collider->getBroadPhaseId()) + " added to body",  __FILE__, __LINE__);

    RP3D_LOG(mWorld.mConfig.worldName, Logger::Level::Information, Logger::Category::Collider,
             "Collider " + std::to_string(collider->getBroadPhaseId()) + ":  collisionShape=" +
             collider->getCollisionShape()->to_string(),  __FILE__, __LINE__);

    // Return a pointer to the collision shape
    return collider;
}

// Return the number of colliders associated with this body
/**
* @return The number of colliders associated with this body
*/
uint32 Body::getNbColliders() const {
    return static_cast<uint32>(mWorld.mBodyComponents.getColliders(mEntity).size());
}

// Return a const pointer to a given collider of the body
/**
* @param colliderIndex Index of a Collider of the body
* @return The const pointer to the requested collider
*/
const Collider* Body::getCollider(uint32 colliderIndex) const {

    assert(colliderIndex < getNbColliders());

    Entity colliderEntity = mWorld.mBodyComponents.getColliders(mEntity)[colliderIndex];

    return mWorld.mCollidersComponents.getCollider(colliderEntity);
}

// Return a pointer to a given collider of the body
/**
* @param colliderIndex Index of a Collider of the body
* @return The pointer to the requested collider
*/
Collider* Body::getCollider(uint32 colliderIndex) {

    assert(colliderIndex < getNbColliders());

    Entity colliderEntity = mWorld.mBodyComponents.getColliders(mEntity)[colliderIndex];

    return mWorld.mCollidersComponents.getCollider(colliderEntity);
}

// Remove a collider from the body
/// To remove a collider, you need to specify its pointer
/**
 * @param collider The pointer of the collider you want to remove
 */
void Body::removeCollider(Collider* collider) {

    RP3D_LOG(mWorld.mConfig.worldName, Logger::Level::Information, Logger::Category::Body,
             "Body " + std::to_string(mEntity.id) + ": Collider " + std::to_string(collider->getBroadPhaseId()) + " removed from body",  __FILE__, __LINE__);

    // Remove the collider from the broad-phase
    if (collider->getBroadPhaseId() != -1) {
        mWorld.mCollisionDetection.removeCollider(collider);
    }

    mWorld.mBodyComponents.removeColliderFromBody(mEntity, collider->getEntity());

    // Unassign the collider from the collision shape
    collider->getCollisionShape()->removeCollider(collider);

    // Remove the collider component
    mWorld.mCollidersComponents.removeComponent(collider->getEntity());

    // Destroy the entity
    mWorld.mEntityManager.destroyEntity(collider->getEntity());

    // Call the constructor of the collider
    collider->~Collider();

    // Update whether the body still has a simulation collider
    if (mWorld.mBodyComponents.getHasSimulationCollider(mEntity)) {
        updateHasSimulationCollider();
    }

    // Release allocated memory for the collider
    mWorld.mMemoryManager.release(MemoryManager::AllocationType::Pool, collider, sizeof(Collider));
}

// Remove all the colliders
void Body::removeAllColliders() {

    // Look for the collider that contains the collision shape in parameter.
    // Note that we need to copy the array of collider entities because we are deleting them in a loop.
    const Array<Entity> collidersEntities = mWorld.mBodyComponents.getColliders(mEntity);
    for (uint32 i=0; i < collidersEntities.size(); i++) {

        removeCollider(mWorld.mCollidersComponents.getCollider(collidersEntities[i]));
    }
}

// Return the current position and orientation
/**
 * @return The current transformation of the body that transforms the local-space
 *         of the body into world-space
 */
const Transform& Body::getTransform() const {

    return mWorld.mTransformComponents.getTransform(mEntity);
}

// Update the broad-phase state for this body (because it has moved for instance)
void Body::updateBroadPhaseState() const {

    // For all the colliders of the body
    const Array<Entity>& colliderEntities = mWorld.mBodyComponents.getColliders(mEntity);
    const uint32 nbColliderEntities = static_cast<uint32>(colliderEntities.size());
    for (uint32 i=0; i < nbColliderEntities; i++) {

        // Update the local-to-world transform of the collider
        mWorld.mCollidersComponents.setLocalToWorldTransform(colliderEntities[i],
                                                               mWorld.mTransformComponents.getTransform(mEntity) *
                                                               mWorld.mCollidersComponents.getLocalToBodyTransform(colliderEntities[i]));

        // Update the collider
        mWorld.mCollisionDetection.updateCollider(colliderEntities[i]);
    }
}

// Set whether or not the body is active
/**
 * @param isActive True if you want to activate the body
 */
void Body::setIsActive(bool isActive) {

    // If the state does not change
    if (mWorld.mBodyComponents.getIsActive(mEntity) == isActive) return;

    mWorld.mBodyComponents.setIsActive(mEntity, isActive);

    // If we have to activate the body
    if (isActive) {

        const Transform& transform = mWorld.mTransformComponents.getTransform(mEntity);

        // For each collider of the body
        const Array<Entity>& colliderEntities = mWorld.mBodyComponents.getColliders(mEntity);
        for (uint32 i=0; i < colliderEntities.size(); i++) {

            Collider* collider = mWorld.mCollidersComponents.getCollider(colliderEntities[i]);

            // Compute the world-space AABB of the new collision shape
            const AABB aabb = collider->getCollisionShape()->computeTransformedAABB(transform * mWorld.mCollidersComponents.getLocalToBodyTransform(collider->getEntity()));

            // Add the collider to the collision detection
            mWorld.mCollisionDetection.addCollider(collider, aabb);
        }
    }
    else {  // If we have to deactivate the body

        // For each collider of the body
        const Array<Entity>& colliderEntities = mWorld.mBodyComponents.getColliders(mEntity);
        for (uint32 i=0; i < colliderEntities.size(); i++) {

            Collider* collider = mWorld.mCollidersComponents.getCollider(colliderEntities[i]);

            if (collider->getBroadPhaseId() != -1) {

                // Remove the collider from the collision detection
                mWorld.mCollisionDetection.removeCollider(collider);
            }
        }
    }

    RP3D_LOG(mWorld.mConfig.worldName, Logger::Level::Information, Logger::Category::Body,
             "Body " + std::to_string(mEntity.id) + ": Set isActive=" +
             (isActive ? "true" : "false"),  __FILE__, __LINE__);
}

// Ask the broad-phase to test again the collision shapes of the body for collision
// (as if the body has moved).
void Body::askForBroadPhaseCollisionCheck() const {

    // For all the colliders of the body
    const Array<Entity>& colliderEntities = mWorld.mBodyComponents.getColliders(mEntity);
    const uint32 nbColliderEntities = static_cast<uint32>(colliderEntities.size());
    for (uint32 i=0; i < nbColliderEntities; i++) {

        Collider* collider = mWorld.mCollidersComponents.getCollider(colliderEntities[i]);

        mWorld.mCollisionDetection.askForBroadPhaseCollisionCheck(collider);
    }
}

// Return true if a point is inside the collision body
/// This method returns true if a point is inside any collision shape of the body
/**
 * @param worldPoint The point to test (in world-space coordinates)
 * @return True if the point is inside the body
 */
bool Body::testPointInside(const Vector3& worldPoint) const {

    // For each collider of the body
    const Array<Entity>& colliderEntities = mWorld.mBodyComponents.getColliders(mEntity);
    for (uint32 i=0; i < colliderEntities.size(); i++) {

        Collider* collider = mWorld.mCollidersComponents.getCollider(colliderEntities[i]);

        // Test if the point is inside the collider
        if (collider->testPointInside(worldPoint)) return true;
    }

    return false;
}

// Raycast method with feedback information
/// The method returns the closest hit among all the collision shapes of the body
/**
* @param ray The ray used to raycast agains the body
* @param[out] raycastInfo Structure that contains the result of the raycasting
*                         (valid only if the method returned true)
* @return True if the ray hit the body and false otherwise
*/
bool Body::raycast(const Ray& ray, RaycastInfo& raycastInfo) {

    // If the body is not active, it cannot be hit by rays
    if (!mWorld.mBodyComponents.getIsActive(mEntity)) return false;

    bool isHit = false;
    Ray rayTemp(ray);

    // For each collider of the body
    const Array<Entity>& colliderEntities = mWorld.mBodyComponents.getColliders(mEntity);
    const uint32 nbColliderEntities = static_cast<uint32>(colliderEntities.size());
    for (uint32 i=0; i < nbColliderEntities; i++) {

        Collider* collider = mWorld.mCollidersComponents.getCollider(colliderEntities[i]);

        // Test if the ray hits the collider
        if (collider->raycast(rayTemp, raycastInfo)) {
            rayTemp.maxFraction = raycastInfo.hitFraction;
            isHit = true;
        }
    }

    return isHit;
}

// Compute and return the AABB of the body by merging all colliders AABBs
/**
* @return The axis-aligned bounding box (AABB) of the body in world-space coordinates
*/
AABB Body::getAABB() const {

    const Array<Entity>& colliderEntities = mWorld.mBodyComponents.getColliders(mEntity);
    if (colliderEntities.size() == 0) return AABB();

    const Transform& transform = mWorld.mTransformComponents.getTransform(mEntity);

    Collider* collider = mWorld.mCollidersComponents.getCollider(colliderEntities[0]);
    AABB bodyAABB = collider->getCollisionShape()->computeTransformedAABB(transform * collider->getLocalToBodyTransform());

    // For each collider of the body
    const uint32 nbColliderEntities = static_cast<uint32>(colliderEntities.size());
    for (uint32 i=1; i < nbColliderEntities; i++) {

        Collider* collider = mWorld.mCollidersComponents.getCollider(colliderEntities[i]);

        // Compute the world-space AABB of the collider
        const AABB aabb = collider->getCollisionShape()->computeTransformedAABB(transform * collider->getLocalToBodyTransform());

        // Merge the collider AABB with the current body AABB
        bodyAABB.mergeWithAABB(aabb);
    }

    return bodyAABB;
}

// Set the current position and orientation
/**
 * @param transform The transformation of the body that transforms the local-space
 *                  of the body into world-space
 */
void Body::setTransform(const Transform& transform) {

    // Update the transform of the body
    mWorld.mTransformComponents.setTransform(mEntity, transform);

    // Update the broad-phase state of the body
    updateBroadPhaseState();

    RP3D_LOG(mWorld.mConfig.worldName, Logger::Level::Information, Logger::Category::Body,
             "Body " + std::to_string(mEntity.id) + ": Set transform=" + transform.to_string(),  __FILE__, __LINE__);
}

// Return true if the body is active
/**
 * @return True if the body currently active and false otherwise
 */
bool Body::isActive() const {
    return mWorld.mBodyComponents.getIsActive(mEntity);
}

// Return a pointer to the user data attached to this body
/**
 * @return A pointer to the user data you have attached to the body
 */
void* Body::getUserData() const {
    return mWorld.mBodyComponents.getUserData(mEntity);
}

// Attach user data to this body
/**
 * @param userData A pointer to the user data you want to attach to the body
 */
void Body::setUserData(void* userData) {
    mWorld.mBodyComponents.setUserData(mEntity, userData);
}

// Return the world-space coordinates of a point given the local-space coordinates of the body
/**
* @param localPoint A point in the local-space coordinates of the body
* @return The point in world-space coordinates
*/
Vector3 Body::getWorldPoint(const Vector3& localPoint) const {
    return mWorld.mTransformComponents.getTransform(mEntity) * localPoint;
}

// Return the world-space vector of a vector given in local-space coordinates of the body
/**
* @param localVector A vector in the local-space coordinates of the body
* @return The vector in world-space coordinates
*/
Vector3 Body::getWorldVector(const Vector3& localVector) const {
    return mWorld.mTransformComponents.getTransform(mEntity).getOrientation() * localVector;
}

// Return the body local-space coordinates of a point given in the world-space coordinates
/**
* @param worldPoint A point in world-space coordinates
* @return The point in the local-space coordinates of the body
*/
Vector3 Body::getLocalPoint(const Vector3& worldPoint) const {
    return mWorld.mTransformComponents.getTransform(mEntity).getInverse() * worldPoint;
}

// Return the body local-space coordinates of a vector given in the world-space coordinates
/**
* @param worldVector A vector in world-space coordinates
* @return The vector in the local-space coordinates of the body
*/
Vector3 Body::getLocalVector(const Vector3& worldVector) const {
    return mWorld.mTransformComponents.getTransform(mEntity).getOrientation().getInverse() * worldVector;
}

// Update whether the body has at least one simulation provider
void Body::updateHasSimulationCollider() {

    // For each collider of the body
    const uint32 bodyIndex = mWorld.mBodyComponents.getEntityIndex(mEntity);
    const Array<Entity>& colliderEntities = mWorld.mBodyComponents.mColliders[bodyIndex];
    for (uint32 i=0; i < colliderEntities.size(); i++) {

        // Get the currently overlapping pairs for this collider
        const bool isSimulationCollider = mWorld.mCollidersComponents.getIsSimulationCollider(colliderEntities[i]);

        if (isSimulationCollider) {
            mWorld.mBodyComponents.mHasSimulationCollider[bodyIndex] = true;
            return;
        }
    }
}

#ifdef IS_RP3D_PROFILING_ENABLED

// Set the profiler
void Body::setProfiler(Profiler* profiler) {

    mProfiler = profiler;

    // Set the profiler for each collider
    const Array<Entity>& colliderEntities = mWorld.mBodyComponents.getColliders(mEntity);
    for (uint32 i=0; i < colliderEntities.size(); i++) {

        Collider* collider = mWorld.mCollidersComponents.getCollider(colliderEntities[i]);

        collider->setProfiler(profiler);
    }
}

#endif

// Set whether to compute debug information on this body
/**
 * @param enabled Set to true if this body should have it's debug information computed
 */
void Body::setIsDebugEnabled(bool enabled) {
    mIsDebugEnabled = enabled;
}

// Returns true if this collision body is computing debug information
/**
 * @return Returns true if this body is computing debug information
 */
bool Body::isDebugEnabled() const {
    return mIsDebugEnabled;
}
