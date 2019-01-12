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
CollisionBody::CollisionBody(CollisionWorld& world, Entity entity, bodyindex id)
              : Body(entity, id), mType(BodyType::DYNAMIC),
                mContactManifoldsList(nullptr), mWorld(world) {

#ifdef IS_PROFILING_ACTIVE
        mProfiler = nullptr;
#endif

}

// Destructor
CollisionBody::~CollisionBody() {
    assert(mContactManifoldsList == nullptr);
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
ProxyShape* CollisionBody::addCollisionShape(CollisionShape* collisionShape,
                                             const Transform& transform) {

    // Create a new proxy collision shape to attach the collision shape to the body
    ProxyShape* proxyShape = new (mWorld.mMemoryManager.allocate(MemoryManager::AllocationType::Pool,
                                      sizeof(ProxyShape))) ProxyShape(this, mWorld.mMemoryManager);

    // Add the proxy-shape component to the entity of the body
    Vector3 localBoundsMin;
    Vector3 localBoundsMax;
    // TODO : Maybe this method can directly returns an AABB
    collisionShape->getLocalBounds(localBoundsMin, localBoundsMax);

    ProxyShapesComponents::ProxyShapeComponent proxyShapeComponent(proxyShape, -1,
                                                                   AABB(localBoundsMin, localBoundsMax),
                                                                   transform, collisionShape, decimal(1));
    mWorld.mProxyShapesComponents.addComponent(mEntity, mIsSleeping, proxyShapeComponent);

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
             "Body " + std::to_string(mID) + ": Proxy shape " + std::to_string(proxyShape->getBroadPhaseId()) + " added to body");

    RP3D_LOG(mLogger, Logger::Level::Information, Logger::Category::ProxyShape,
             "ProxyShape " + std::to_string(proxyShape->getBroadPhaseId()) + ":  collisionShape=" +
             proxyShape->getCollisionShape()->to_string());

    // Return a pointer to the collision shape
    return proxyShape;
}

// Return the linked list of proxy shapes of that body
/**
* @return The pointer of the first proxy shape of the linked-list of all the
*         proxy shapes of the body
*/
ProxyShape* CollisionBody::getProxyShapesList() {
    return mWorld.mProxyShapesComponents.getFirstProxyShapeOfBody(mEntity);
}

// Return the linked list of proxy shapes of that body
/**
* @return The pointer of the first proxy shape of the linked-list of all the
*         proxy shapes of the body
*/
const ProxyShape* CollisionBody::getProxyShapesList() const {
    return mWorld.mProxyShapesComponents.getFirstProxyShapeOfBody(mEntity);
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
             "Body " + std::to_string(mID) + ": Proxy shape " + std::to_string(proxyShape->getBroadPhaseId()) + " removed from body");

    // Remove the proxy-shape component
    mWorld.mProxyShapesComponents.removeComponent(proxyShape);

    // Remove the proxy-shape from the broad-phase
    if (mIsActive && proxyShape->getBroadPhaseId() != -1) {
        mWorld.mCollisionDetection.removeProxyCollisionShape(proxyShape);
    }

    // Call the constructor of the proxy-shape
    proxyShape->~ProxyShape();

    // Release allocated memory for the proxy-shape
    mWorld.mMemoryManager.release(MemoryManager::AllocationType::Pool, proxyShape, sizeof(ProxyShape));
}

// Remove all the collision shapes
void CollisionBody::removeAllCollisionShapes() {

    ProxyShape* proxyShape = mWorld.mProxyShapesComponents.getFirstProxyShapeOfBody(mEntity);

    // Look for the proxy shape that contains the collision shape in parameter
    while(proxyShape != nullptr) {

        if (mIsActive && proxyShape->getBroadPhaseId() != -1) {

            mWorld.mCollisionDetection.removeProxyCollisionShape(proxyShape);
        }

        // Destroy the proxy-shape
        proxyShape->~ProxyShape();

        mWorld.mMemoryManager.release(MemoryManager::AllocationType::Pool, proxyShape, sizeof(ProxyShape));

        // Get the next element in the list
        proxyShape = mWorld.mProxyShapesComponents.getNextProxyShapeOfBody(proxyShape);
    }

    // Remove all the proxy-shapes components
    mWorld.mProxyShapesComponents.removeComponents(mEntity);
}

// Reset the contact manifold lists
void CollisionBody::resetContactManifoldsList() {

    // Delete the linked list of contact manifolds of that body
    ContactManifoldListElement* currentElement = mContactManifoldsList;
    while (currentElement != nullptr) {
        ContactManifoldListElement* nextElement = currentElement->getNext();

        // Delete the current element
        currentElement->~ContactManifoldListElement();
        mWorld.mMemoryManager.release(MemoryManager::AllocationType::Pool, currentElement, sizeof(ContactManifoldListElement));

        currentElement = nextElement;
    }
    mContactManifoldsList = nullptr;
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
    for (ProxyShape* shape = mWorld.mProxyShapesComponents.getFirstProxyShapeOfBody(mEntity); shape != nullptr; shape = mWorld.mProxyShapesComponents.getNextProxyShapeOfBody(shape)) {

        // Update the proxy
        updateProxyShapeInBroadPhase(shape);
    }
}

// Update the broad-phase state of a proxy collision shape of the body
void CollisionBody::updateProxyShapeInBroadPhase(ProxyShape* proxyShape, bool forceReinsert) const {

    const Transform& transform = mWorld.mTransformComponents.getTransform(mEntity);

    if (proxyShape->getBroadPhaseId() != -1) {

        // Recompute the world-space AABB of the collision shape
        AABB aabb;
        proxyShape->getCollisionShape()->computeAABB(aabb, transform * proxyShape->getLocalToBodyTransform());

        // Update the broad-phase state for the proxy collision shape
        mWorld.mCollisionDetection.updateProxyCollisionShape(proxyShape, aabb, Vector3(0, 0, 0), forceReinsert)	;
    }
}

// Set whether or not the body is active
/**
 * @param isActive True if you want to activate the body
 */
void CollisionBody::setIsActive(bool isActive) {

    // If the state does not change
    if (mIsActive == isActive) return;

    Body::setIsActive(isActive);

    // If we have to activate the body
    if (isActive) {

        const Transform& transform = mWorld.mTransformComponents.getTransform(mEntity);

        // For each proxy shape of the body
        for (ProxyShape* shape = mWorld.mProxyShapesComponents.getFirstProxyShapeOfBody(mEntity); shape != nullptr; shape = mWorld.mProxyShapesComponents.getNextProxyShapeOfBody(shape)) {

            // Compute the world-space AABB of the new collision shape
            AABB aabb;
            shape->getCollisionShape()->computeAABB(aabb, transform * mWorld.mProxyShapesComponents.getLocalToBodyTransform(shape));

            // Add the proxy shape to the collision detection
            mWorld.mCollisionDetection.addProxyCollisionShape(shape, aabb);
        }
    }
    else {  // If we have to deactivate the body

        // For each proxy shape of the body
        for (ProxyShape* shape = mWorld.mProxyShapesComponents.getFirstProxyShapeOfBody(mEntity); shape != nullptr; shape = mWorld.mProxyShapesComponents.getNextProxyShapeOfBody(shape)) {

            if (shape->getBroadPhaseId() != -1) {

                // Remove the proxy shape from the collision detection
                mWorld.mCollisionDetection.removeProxyCollisionShape(shape);
            }
        }

        // Reset the contact manifold list of the body
        resetContactManifoldsList();
    }

    RP3D_LOG(mLogger, Logger::Level::Information, Logger::Category::Body,
             "Body " + std::to_string(mID) + ": Set isActive=" +
             (mIsActive ? "true" : "false"));
}

// Ask the broad-phase to test again the collision shapes of the body for collision
// (as if the body has moved).
void CollisionBody::askForBroadPhaseCollisionCheck() const {

    // For all the proxy collision shapes of the body
    for (ProxyShape* shape = mWorld.mProxyShapesComponents.getFirstProxyShapeOfBody(mEntity); shape != nullptr; shape = mWorld.mProxyShapesComponents.getNextProxyShapeOfBody(shape)) {

        mWorld.mCollisionDetection.askForBroadPhaseCollisionCheck(shape);  
    }
}

// Reset the mIsAlreadyInIsland variable of the body and contact manifolds.
/// This method also returns the number of contact manifolds of the body.
int CollisionBody::resetIsAlreadyInIslandAndCountManifolds() {

    mIsAlreadyInIsland = false;

    int nbManifolds = 0;

    // Reset the mIsAlreadyInIsland variable of the contact manifolds for
    // this body
    ContactManifoldListElement* currentElement = mContactManifoldsList;
    while (currentElement != nullptr) {
        currentElement->getContactManifold()->mIsAlreadyInIsland = false;
        currentElement = currentElement->getNext();
        nbManifolds++;
    }

    return nbManifolds;
}

// Return true if a point is inside the collision body
/// This method returns true if a point is inside any collision shape of the body
/**
 * @param worldPoint The point to test (in world-space coordinates)
 * @return True if the point is inside the body
 */
bool CollisionBody::testPointInside(const Vector3& worldPoint) const {

    // For each collision shape of the body
    for (ProxyShape* shape = mWorld.mProxyShapesComponents.getFirstProxyShapeOfBody(mEntity); shape != nullptr; shape = mWorld.mProxyShapesComponents.getNextProxyShapeOfBody(shape)) {

        // Test if the point is inside the collision shape
        if (shape->testPointInside(worldPoint)) return true;
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
    if (!mIsActive) return false;

    bool isHit = false;
    Ray rayTemp(ray);

    // For each collision shape of the body
    for (ProxyShape* shape = mWorld.mProxyShapesComponents.getFirstProxyShapeOfBody(mEntity); shape != nullptr; shape = mWorld.mProxyShapesComponents.getNextProxyShapeOfBody(shape)) {

        // Test if the ray hits the collision shape
        if (shape->raycast(rayTemp, raycastInfo)) {
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

    if (mWorld.mProxyShapesComponents.getFirstProxyShapeOfBody(mEntity) == nullptr) return bodyAABB;

    // TODO : Make sure we compute this in a system

    const Transform& transform = mWorld.mTransformComponents.getTransform(mEntity);

    ProxyShape* proxyShape = mWorld.mProxyShapesComponents.getFirstProxyShapeOfBody(mEntity);
    proxyShape->getCollisionShape()->computeAABB(bodyAABB, transform * proxyShape->getLocalToBodyTransform());

    // For each proxy shape of the body
    for (ProxyShape* shape = mWorld.mProxyShapesComponents.getFirstProxyShapeOfBody(mEntity); shape != nullptr; shape = mWorld.mProxyShapesComponents.getNextProxyShapeOfBody(shape)) {

        // Compute the world-space AABB of the collision shape
        AABB aabb;
        shape->getCollisionShape()->computeAABB(aabb, transform * shape->getLocalToBodyTransform());

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

    // Update the transform of the body
    mWorld.mTransformComponents.setTransform(mEntity, transform);

    // Update the broad-phase state of the body
    updateBroadPhaseState();

    RP3D_LOG(mLogger, Logger::Level::Information, Logger::Category::Body,
             "Body " + std::to_string(mID) + ": Set transform=" + transform.to_string());
}


// Set the variable to know whether or not the body is sleeping
void CollisionBody::setIsSleeping(bool isSleeping) {

    Body::setIsSleeping(isSleeping);

    // Notify all the components
    mWorld.notifyBodySleeping(mEntity, isSleeping);
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
void CollisionBody::setType(BodyType type) {
    mType = type;

    if (mType == BodyType::STATIC) {

        // Update the broad-phase state of the body
        updateBroadPhaseState();
    }

    RP3D_LOG(mLogger, Logger::Level::Information, Logger::Category::Body,
             "Body " + std::to_string(mID) + ": Set type=" +
             (mType == BodyType::STATIC ? "Static" : (mType == BodyType::DYNAMIC ? "Dynamic" : "Kinematic")));
}

