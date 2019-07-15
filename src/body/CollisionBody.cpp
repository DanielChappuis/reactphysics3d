/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2018 Daniel Chappuis                                       *
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
#include "CollisionBody.h"
#include "engine/CollisionWorld.h"
#include "collision/ContactManifold.h"
#include "collision/RaycastInfo.h"
#include "utils/Logger.h"

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor
/**
 * @param transform The transform of the body
 * @param world The physics world where the body is created
 * @param id ID of the body
 */
CollisionBody::CollisionBody(CollisionWorld& world, Entity entity)
              : mEntity(entity), mWorld(world)  {

#ifdef IS_LOGGING_ACTIVE
        mLogger = nullptr;
#endif

#ifdef IS_PROFILING_ACTIVE
        mProfiler = nullptr;
#endif

}

// Destructor
CollisionBody::~CollisionBody() {

}

// Add a collision shape to the body. Note that you can share a collision
// shape between several bodies using the same collision shape instance to
// when you add the shape to the different bodies. Do not forget to delete
// the collision shape you have created at the end of your program.
/// This method will return a pointer to a new proxy shape. A proxy shape is
/// an object that links a collision shape and a given body. You can use the
/// returned proxy shape to get and set information about the corresponding
/// collision shape for that body.
/**
 * @param collisionShape A pointer to the collision shape you want to add to the body
 * @param transform The transformation of the collision shape that transforms the
 *        local-space of the collision shape into the local-space of the body
 * @return A pointer to the proxy shape that has been created to link the body to
 *         the new collision shape you have added.
 */
ProxyShape* CollisionBody::addCollisionShape(CollisionShape* collisionShape, const Transform& transform) {

    // Create a new entity for the proxy-shape
    Entity proxyShapeEntity = mWorld.mEntityManager.createEntity();

    // Create a new proxy collision shape to attach the collision shape to the body
    ProxyShape* proxyShape = new (mWorld.mMemoryManager.allocate(MemoryManager::AllocationType::Pool,
                                      sizeof(ProxyShape))) ProxyShape(proxyShapeEntity, this, mWorld.mMemoryManager);

    // Add the proxy-shape component to the entity of the body
    Vector3 localBoundsMin;
    Vector3 localBoundsMax;
    // TODO : Maybe this method can directly returns an AABB
    collisionShape->getLocalBounds(localBoundsMin, localBoundsMax);
    ProxyShapeComponents::ProxyShapeComponent proxyShapeComponent(mEntity, proxyShape, -1,
                                                                   AABB(localBoundsMin, localBoundsMax),
                                                                   transform, collisionShape, decimal(1), 0x0001, 0xFFFF);
    bool isSleeping = mWorld.mBodyComponents.getIsSleeping(mEntity);
    mWorld.mProxyShapesComponents.addComponent(proxyShapeEntity, isSleeping, proxyShapeComponent);

    mWorld.mBodyComponents.addProxyShapeToBody(mEntity, proxyShapeEntity);

#ifdef IS_PROFILING_ACTIVE

	// Set the profiler
	proxyShape->setProfiler(mProfiler);

#endif

#ifdef IS_LOGGING_ACTIVE

    // Set the logger
    proxyShape->setLogger(mLogger);

#endif

    // Compute the world-space AABB of the new collision shape
    AABB aabb;
    collisionShape->computeAABB(aabb, mWorld.mTransformComponents.getTransform(mEntity) * transform);

    // Notify the collision detection about this new collision shape
    mWorld.mCollisionDetection.addProxyCollisionShape(proxyShape, aabb);

    RP3D_LOG(mLogger, Logger::Level::Information, Logger::Category::Body,
             "Body " + std::to_string(mEntity.id) + ": Proxy shape " + std::to_string(proxyShape->getBroadPhaseId()) + " added to body");

    RP3D_LOG(mLogger, Logger::Level::Information, Logger::Category::ProxyShape,
             "ProxyShape " + std::to_string(proxyShape->getBroadPhaseId()) + ":  collisionShape=" +
             proxyShape->getCollisionShape()->to_string());

    // Return a pointer to the collision shape
    return proxyShape;
}

// Return the number of proxy-shapes associated with this body
/**
* @return The number of proxy-shapes associated with this body
*/
uint CollisionBody::getNbProxyShapes() const {
    return static_cast<uint>(mWorld.mBodyComponents.getProxyShapes(mEntity).size());
}

// Return a const pointer to a given proxy-shape of the body
/**
* @return The const pointer of a given proxy-shape of the body
*/
const ProxyShape* CollisionBody::getProxyShape(uint proxyShapeIndex) const {

    assert(proxyShapeIndex < getNbProxyShapes());

    Entity proxyShapeEntity = mWorld.mBodyComponents.getProxyShapes(mEntity)[proxyShapeIndex];

    return mWorld.mProxyShapesComponents.getProxyShape(proxyShapeEntity);
}

// Return a pointer to a given proxy-shape of the body
/**
* @return The pointer of a given proxy-shape of the body
*/
ProxyShape* CollisionBody::getProxyShape(uint proxyShapeIndex) {

    assert(proxyShapeIndex < getNbProxyShapes());

    Entity proxyShapeEntity = mWorld.mBodyComponents.getProxyShapes(mEntity)[proxyShapeIndex];

    return mWorld.mProxyShapesComponents.getProxyShape(proxyShapeEntity);
}

// Remove a collision shape from the body
/// To remove a collision shape, you need to specify the pointer to the proxy
/// shape that has been returned when you have added the collision shape to the
/// body
/**
 * @param proxyShape The pointer of the proxy shape you want to remove
 */
void CollisionBody::removeCollisionShape(ProxyShape* proxyShape) {

    RP3D_LOG(mLogger, Logger::Level::Information, Logger::Category::Body,
             "Body " + std::to_string(mEntity.id) + ": Proxy shape " + std::to_string(proxyShape->getBroadPhaseId()) + " removed from body");

    // Remove the proxy-shape from the broad-phase
    if (proxyShape->getBroadPhaseId() != -1) {
        mWorld.mCollisionDetection.removeProxyCollisionShape(proxyShape);
    }

    mWorld.mBodyComponents.removeProxyShapeFromBody(mEntity, proxyShape->getEntity());

    // Remove the proxy-shape component
    mWorld.mProxyShapesComponents.removeComponent(proxyShape->getEntity());

    // Call the constructor of the proxy-shape
    proxyShape->~ProxyShape();

    // Release allocated memory for the proxy-shape
    mWorld.mMemoryManager.release(MemoryManager::AllocationType::Pool, proxyShape, sizeof(ProxyShape));
}

// Remove all the collision shapes
void CollisionBody::removeAllCollisionShapes() {

    // Look for the proxy shape that contains the collision shape in parameter.
    // Note that we need to copy the list of proxy shapes entities because we are deleting them in a loop.
    const List<Entity> proxyShapesEntities = mWorld.mBodyComponents.getProxyShapes(mEntity);
    for (uint i=0; i < proxyShapesEntities.size(); i++) {

        removeCollisionShape(mWorld.mProxyShapesComponents.getProxyShape(proxyShapesEntities[i]));
    }
}

// Return the current position and orientation
/**
 * @return The current transformation of the body that transforms the local-space
 *         of the body into world-space
 */
const Transform& CollisionBody::getTransform() const {

    // TODO : Make sure we do not call this method from the internal physics engine

    return mWorld.mTransformComponents.getTransform(mEntity);
}

// Update the broad-phase state for this body (because it has moved for instance)
void CollisionBody::updateBroadPhaseState() const {

    // For all the proxy collision shapes of the body
    const List<Entity>& proxyShapesEntities = mWorld.mBodyComponents.getProxyShapes(mEntity);
    for (uint i=0; i < proxyShapesEntities.size(); i++) {

        // Update the proxy
        mWorld.mCollisionDetection.updateProxyShape(proxyShapesEntities[i]);
    }
}

// Set whether or not the body is active
/**
 * @param isActive True if you want to activate the body
 */
void CollisionBody::setIsActive(bool isActive) {

    // If the state does not change
    if (mWorld.mBodyComponents.getIsActive(mEntity) == isActive) return;

    mWorld.mBodyComponents.setIsActive(mEntity, isActive);

    // If we have to activate the body
    if (isActive) {

        const Transform& transform = mWorld.mTransformComponents.getTransform(mEntity);

        // For each proxy shape of the body
        const List<Entity>& proxyShapesEntities = mWorld.mBodyComponents.getProxyShapes(mEntity);
        for (uint i=0; i < proxyShapesEntities.size(); i++) {

            ProxyShape* proxyShape = mWorld.mProxyShapesComponents.getProxyShape(proxyShapesEntities[i]);

            // Compute the world-space AABB of the new collision shape
            AABB aabb;
            proxyShape->getCollisionShape()->computeAABB(aabb, transform * mWorld.mProxyShapesComponents.getLocalToBodyTransform(proxyShape->getEntity()));

            // Add the proxy shape to the collision detection
            mWorld.mCollisionDetection.addProxyCollisionShape(proxyShape, aabb);
        }
    }
    else {  // If we have to deactivate the body

        // For each proxy shape of the body
        const List<Entity>& proxyShapesEntities = mWorld.mBodyComponents.getProxyShapes(mEntity);
        for (uint i=0; i < proxyShapesEntities.size(); i++) {

            ProxyShape* proxyShape = mWorld.mProxyShapesComponents.getProxyShape(proxyShapesEntities[i]);

            if (proxyShape->getBroadPhaseId() != -1) {

                // Remove the proxy shape from the collision detection
                mWorld.mCollisionDetection.removeProxyCollisionShape(proxyShape);
            }
        }
    }

    RP3D_LOG(mLogger, Logger::Level::Information, Logger::Category::Body,
             "Body " + std::to_string(mEntity.id) + ": Set isActive=" +
             (isActive ? "true" : "false"));
}

// Ask the broad-phase to test again the collision shapes of the body for collision
// (as if the body has moved).
void CollisionBody::askForBroadPhaseCollisionCheck() const {

    // For all the proxy collision shapes of the body
    const List<Entity>& proxyShapesEntities = mWorld.mBodyComponents.getProxyShapes(mEntity);
    for (uint i=0; i < proxyShapesEntities.size(); i++) {

        ProxyShape* proxyShape = mWorld.mProxyShapesComponents.getProxyShape(proxyShapesEntities[i]);

        mWorld.mCollisionDetection.askForBroadPhaseCollisionCheck(proxyShape);
    }
}

// Return true if a point is inside the collision body
/// This method returns true if a point is inside any collision shape of the body
/**
 * @param worldPoint The point to test (in world-space coordinates)
 * @return True if the point is inside the body
 */
bool CollisionBody::testPointInside(const Vector3& worldPoint) const {

    // For each collision shape of the body
    const List<Entity>& proxyShapesEntities = mWorld.mBodyComponents.getProxyShapes(mEntity);
    for (uint i=0; i < proxyShapesEntities.size(); i++) {

        ProxyShape* proxyShape = mWorld.mProxyShapesComponents.getProxyShape(proxyShapesEntities[i]);

        // Test if the point is inside the collision shape
        if (proxyShape->testPointInside(worldPoint)) return true;
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
bool CollisionBody::raycast(const Ray& ray, RaycastInfo& raycastInfo) {

    // If the body is not active, it cannot be hit by rays
    if (!mWorld.mBodyComponents.getIsActive(mEntity)) return false;

    bool isHit = false;
    Ray rayTemp(ray);

    // For each collision shape of the body
    const List<Entity>& proxyShapesEntities = mWorld.mBodyComponents.getProxyShapes(mEntity);
    for (uint i=0; i < proxyShapesEntities.size(); i++) {

        ProxyShape* proxyShape = mWorld.mProxyShapesComponents.getProxyShape(proxyShapesEntities[i]);

        // Test if the ray hits the collision shape
        if (proxyShape->raycast(rayTemp, raycastInfo)) {
            rayTemp.maxFraction = raycastInfo.hitFraction;
            isHit = true;
        }
    }

    return isHit;
}

// Compute and return the AABB of the body by merging all proxy shapes AABBs
/**
* @return The axis-aligned bounding box (AABB) of the body in world-space coordinates
*/
AABB CollisionBody::getAABB() const {

    AABB bodyAABB;

    const List<Entity>& proxyShapesEntities = mWorld.mBodyComponents.getProxyShapes(mEntity);
    if (proxyShapesEntities.size() == 0) return bodyAABB;

    // TODO : Make sure we compute this in a system

    const Transform& transform = mWorld.mTransformComponents.getTransform(mEntity);

    ProxyShape* proxyShape = mWorld.mProxyShapesComponents.getProxyShape(proxyShapesEntities[0]);
    proxyShape->getCollisionShape()->computeAABB(bodyAABB, transform * proxyShape->getLocalToBodyTransform());

    // For each proxy shape of the body
    for (uint i=1; i < proxyShapesEntities.size(); i++) {

        ProxyShape* proxyShape = mWorld.mProxyShapesComponents.getProxyShape(proxyShapesEntities[i]);

        // Compute the world-space AABB of the collision shape
        AABB aabb;
        proxyShape->getCollisionShape()->computeAABB(aabb, transform * proxyShape->getLocalToBodyTransform());

        // Merge the proxy shape AABB with the current body AABB
        bodyAABB.mergeWithAABB(aabb);
    }

    return bodyAABB;
}

// Set the current position and orientation
/**
 * @param transform The transformation of the body that transforms the local-space
 *                  of the body into world-space
 */
void CollisionBody::setTransform(const Transform& transform) {

    // TODO : Make sure this method is never called from the internal physics engine

    // Update the transform of the body
    mWorld.mTransformComponents.setTransform(mEntity, transform);

    // Update the broad-phase state of the body
    updateBroadPhaseState();

    RP3D_LOG(mLogger, Logger::Level::Information, Logger::Category::Body,
             "Body " + std::to_string(mEntity.id) + ": Set transform=" + transform.to_string());
}

// Return true if the body is active
/**
 * @return True if the body currently active and false otherwise
 */
bool CollisionBody::isActive() const {
    return mWorld.mBodyComponents.getIsActive(mEntity);
}

// Return a pointer to the user data attached to this body
/**
 * @return A pointer to the user data you have attached to the body
 */
void* CollisionBody::getUserData() const {
    return mWorld.mBodyComponents.getUserData(mEntity);
}

// Attach user data to this body
/**
 * @param userData A pointer to the user data you want to attach to the body
 */
void CollisionBody::setUserData(void* userData) {
    mWorld.mBodyComponents.setUserData(mEntity, userData);
}

// Return the world-space coordinates of a point given the local-space coordinates of the body
/**
* @param localPoint A point in the local-space coordinates of the body
* @return The point in world-space coordinates
*/
Vector3 CollisionBody::getWorldPoint(const Vector3& localPoint) const {
    return mWorld.mTransformComponents.getTransform(mEntity) * localPoint;
}

// Return the world-space vector of a vector given in local-space coordinates of the body
/**
* @param localVector A vector in the local-space coordinates of the body
* @return The vector in world-space coordinates
*/
Vector3 CollisionBody::getWorldVector(const Vector3& localVector) const {
    return mWorld.mTransformComponents.getTransform(mEntity).getOrientation() * localVector;
}

// Return the body local-space coordinates of a point given in the world-space coordinates
/**
* @param worldPoint A point in world-space coordinates
* @return The point in the local-space coordinates of the body
*/
Vector3 CollisionBody::getLocalPoint(const Vector3& worldPoint) const {
    return mWorld.mTransformComponents.getTransform(mEntity).getInverse() * worldPoint;
}

// Return the body local-space coordinates of a vector given in the world-space coordinates
/**
* @param worldVector A vector in world-space coordinates
* @return The vector in the local-space coordinates of the body
*/
Vector3 CollisionBody::getLocalVector(const Vector3& worldVector) const {
    return mWorld.mTransformComponents.getTransform(mEntity).getOrientation().getInverse() * worldVector;
}
