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
#include "RigidBody.h"
#include "constraint/Joint.h"
#include "collision/shapes/CollisionShape.h"
#include "engine/DynamicsWorld.h"
#include "utils/Profiler.h"

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor
/**
* @param transform The transformation of the body
* @param world The world where the body has been added
* @param id The ID of the body
*/
RigidBody::RigidBody(const Transform& transform, CollisionWorld& world, Entity entity)
          : CollisionBody(world, entity),  mMaterial(world.mConfig), mJointsList(nullptr),
            mIsCenterOfMassSetByUser(false), mIsInertiaTensorSetByUser(false) {

    // Compute the inverse mass
    mWorld.mDynamicsComponents.setMassInverse(mEntity, decimal(1.0) / mWorld.mDynamicsComponents.getInitMass(mEntity));

    // Update the world inverse inertia tensor
    updateInertiaTensorInverseWorld();
}

// Destructor
RigidBody::~RigidBody() {
    assert(mJointsList == nullptr);
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
void RigidBody::setType(BodyType type) {

    if (mType == type) return;

    CollisionBody::setType(type);

    // Recompute the total mass, center of mass and inertia tensor
    recomputeMassInformation();

    // If it is a static body
    if (mType == BodyType::STATIC) {

        // Reset the velocity to zero
        mWorld.mDynamicsComponents.setLinearVelocity(mEntity, Vector3::zero());
        mWorld.mDynamicsComponents.setAngularVelocity(mEntity, Vector3::zero());
    }

    // If it is a static or a kinematic body
    if (mType == BodyType::STATIC || mType == BodyType::KINEMATIC) {

        // Reset the inverse mass and inverse inertia tensor to zero
        mWorld.mDynamicsComponents.setMassInverse(mEntity, decimal(0));
        mWorld.mDynamicsComponents.setInverseInertiaTensorLocal(mEntity, Matrix3x3::zero());
        mWorld.mDynamicsComponents.setInverseInertiaTensorWorld(mEntity, Matrix3x3::zero());
    }
    else {  // If it is a dynamic body
        mWorld.mDynamicsComponents.setMassInverse(mEntity, decimal(1.0) / mWorld.mDynamicsComponents.getInitMass(mEntity));

        if (mIsInertiaTensorSetByUser) {
            mWorld.mDynamicsComponents.setInverseInertiaTensorLocal(mEntity, mUserInertiaTensorLocalInverse);
        }
    }

    // Update the world inverse inertia tensor
    updateInertiaTensorInverseWorld();

    // Awake the body
    setIsSleeping(false);

    // Ask the broad-phase to test again the collision shapes of the body for collision
    // detection (as if the body has moved)
    askForBroadPhaseCollisionCheck();

    // Reset the force and torque on the body
    mWorld.mDynamicsComponents.setExternalForce(mEntity, Vector3::zero());
    mWorld.mDynamicsComponents.setExternalTorque(mEntity, Vector3::zero());
}

// Get the inverse local inertia tensor of the body (in body coordinates)
const Matrix3x3& RigidBody::getInverseInertiaTensorLocal() const {
    return mWorld.mDynamicsComponents.getInertiaTensorLocalInverse(mEntity);
}

// Return the inverse of the inertia tensor in world coordinates.
/// The inertia tensor I_w in world coordinates is computed with the
/// local inverse inertia tensor I_b^-1 in body coordinates
/// by I_w = R * I_b^-1 * R^T
/// where R is the rotation matrix (and R^T its transpose) of the
/// current orientation quaternion of the body
/**
 * @return The 3x3 inverse inertia tensor matrix of the body in world-space
 *         coordinates
 */
Matrix3x3 RigidBody::getInertiaTensorInverseWorld() const {

    // Compute and return the inertia tensor in world coordinates
    return mWorld.mDynamicsComponents.getInertiaTensorWorldInverse(mEntity);
}

// Method that return the mass of the body
/**
 * @return The mass (in kilograms) of the body
 */
decimal RigidBody::getMass() const {
    return mWorld.mDynamicsComponents.getInitMass(mEntity);
}

// Apply an external force to the body at a given point (in world-space coordinates).
/// If the point is not at the center of mass of the body, it will also
/// generate some torque and therefore, change the angular velocity of the body.
/// If the body is sleeping, calling this method will wake it up. Note that the
/// force will we added to the sum of the applied forces and that this sum will be
/// reset to zero at the end of each call of the DynamicsWorld::update() method.
/// You can only apply a force to a dynamic body otherwise, this method will do nothing.
/**
 * @param force The force to apply on the body
 * @param point The point where the force is applied (in world-space coordinates)
 */
void RigidBody::applyForce(const Vector3& force, const Vector3& point) {

    // If it is not a dynamic body, we do nothing
    if (mType != BodyType::DYNAMIC) return;

    // Awake the body if it was sleeping
    if (mWorld.mBodyComponents.getIsSleeping(mEntity)) {
        setIsSleeping(false);
    }

    // Add the force
    const Vector3& externalForce = mWorld.mDynamicsComponents.getExternalForce(mEntity);
    mWorld.mDynamicsComponents.setExternalForce(mEntity, externalForce + force);

    // Add the torque
    const Vector3& externalTorque = mWorld.mDynamicsComponents.getExternalTorque(mEntity);
    const Vector3& centerOfMassWorld = mWorld.mDynamicsComponents.getCenterOfMassWorld(mEntity);
    mWorld.mDynamicsComponents.setExternalTorque(mEntity, externalTorque + (point - centerOfMassWorld).cross(force));
}

// Set the local inertia tensor of the body (in local-space coordinates)
/// If the inertia tensor is set with this method, it will not be computed
/// using the collision shapes of the body.
/**
 * @param inertiaTensorLocal The 3x3 inertia tensor matrix of the body in local-space
 *                           coordinates
 */
void RigidBody::setInertiaTensorLocal(const Matrix3x3& inertiaTensorLocal) {

    mUserInertiaTensorLocalInverse = inertiaTensorLocal.getInverse();
    mIsInertiaTensorSetByUser = true;

    if (mType != BodyType::DYNAMIC) return;

    // Compute the inverse local inertia tensor
    mWorld.mDynamicsComponents.setInverseInertiaTensorLocal(mEntity, mUserInertiaTensorLocalInverse);

    // Update the world inverse inertia tensor
    updateInertiaTensorInverseWorld();

    RP3D_LOG(mLogger, Logger::Level::Information, Logger::Category::Body,
             "Body " + std::to_string(mEntity.id) + ": Set inertiaTensorLocal=" + inertiaTensorLocal.to_string());
}

// Apply an external force to the body at its center of mass.
/// If the body is sleeping, calling this method will wake it up. Note that the
/// force will we added to the sum of the applied forces and that this sum will be
/// reset to zero at the end of each call of the DynamicsWorld::update() method.
/// You can only apply a force to a dynamic body otherwise, this method will do nothing.
/**
 * @param force The external force to apply on the center of mass of the body
 */
void RigidBody::applyForceToCenterOfMass(const Vector3& force) {

    // If it is not a dynamic body, we do nothing
    if (mType != BodyType::DYNAMIC) return;

    // Awake the body if it was sleeping
    if (mWorld.mBodyComponents.getIsSleeping(mEntity)) {
        setIsSleeping(false);
    }

    // Add the force
    const Vector3& externalForce = mWorld.mDynamicsComponents.getExternalForce(mEntity);
    mWorld.mDynamicsComponents.setExternalForce(mEntity, externalForce + force);
}

// Return the linear velocity damping factor
/**
 * @return The linear damping factor of this body
 */
decimal RigidBody::getLinearDamping() const {
    return mWorld.mDynamicsComponents.getLinearDamping(mEntity);
}

// Return the angular velocity damping factor
/**
 * @return The angular damping factor of this body
 */
decimal RigidBody::getAngularDamping() const {
    return mWorld.mDynamicsComponents.getAngularDamping(mEntity);
}

// Set the inverse local inertia tensor of the body (in local-space coordinates)
/// If the inverse inertia tensor is set with this method, it will not be computed
/// using the collision shapes of the body.
/**
 * @param inverseInertiaTensorLocal The 3x3 inverse inertia tensor matrix of the body in local-space
 *                           		coordinates
 */
void RigidBody::setInverseInertiaTensorLocal(const Matrix3x3& inverseInertiaTensorLocal) {

    mUserInertiaTensorLocalInverse = inverseInertiaTensorLocal;
    mIsInertiaTensorSetByUser = true;

    if (mType != BodyType::DYNAMIC) return;

    // Compute the inverse local inertia tensor
    mWorld.mDynamicsComponents.setInverseInertiaTensorLocal(mEntity, mUserInertiaTensorLocalInverse);

    // Update the world inverse inertia tensor
    updateInertiaTensorInverseWorld();

    RP3D_LOG(mLogger, Logger::Level::Information, Logger::Category::Body,
             "Body " + std::to_string(mEntity.id) + ": Set inverseInertiaTensorLocal=" + inverseInertiaTensorLocal.to_string());
}

// Set the local center of mass of the body (in local-space coordinates)
/// If you set the center of mass with the method, it will not be computed
/// automatically using collision shapes.
/**
 * @param centerOfMassLocal The center of mass of the body in local-space
 *                          coordinates
 */
void RigidBody::setCenterOfMassLocal(const Vector3& centerOfMassLocal) {

    // TODO : Check if we need to update the postion of the body here at the end (transform of the body)

    if (mType != BodyType::DYNAMIC) return;

    mIsCenterOfMassSetByUser = true;

    const Vector3 oldCenterOfMass = mWorld.mDynamicsComponents.getCenterOfMassWorld(mEntity);
    mWorld.mDynamicsComponents.setCenterOfMassLocal(mEntity, centerOfMassLocal);

    // Compute the center of mass in world-space coordinates
    const Vector3& updatedCenterOfMassLocal = mWorld.mDynamicsComponents.getCenterOfMassLocal(mEntity);
    mWorld.mDynamicsComponents.setCenterOfMassWorld(mEntity, mWorld.mTransformComponents.getTransform(mEntity) * updatedCenterOfMassLocal);

    // Update the linear velocity of the center of mass
    Vector3 linearVelocity = mWorld.mDynamicsComponents.getAngularVelocity(mEntity);
    const Vector3& angularVelocity = mWorld.mDynamicsComponents.getAngularVelocity(mEntity);
    const Vector3& centerOfMassWorld = mWorld.mDynamicsComponents.getCenterOfMassWorld(mEntity);
    linearVelocity += angularVelocity.cross(centerOfMassWorld - oldCenterOfMass);
    mWorld.mDynamicsComponents.setLinearVelocity(mEntity, linearVelocity);

    RP3D_LOG(mLogger, Logger::Level::Information, Logger::Category::Body,
             "Body " + std::to_string(mEntity.id) + ": Set centerOfMassLocal=" + centerOfMassLocal.to_string());
}

// Set the mass of the rigid body
/**
 * @param mass The mass (in kilograms) of the body
 */
void RigidBody::setMass(decimal mass) {

    if (mType != BodyType::DYNAMIC) return;

    mWorld.mDynamicsComponents.setInitMass(mEntity, mass);

    if (mWorld.mDynamicsComponents.getInitMass(mEntity) > decimal(0.0)) {
        mWorld.mDynamicsComponents.setMassInverse(mEntity, decimal(1.0) / mWorld.mDynamicsComponents.getInitMass(mEntity));
    }
    else {
        mWorld.mDynamicsComponents.setInitMass(mEntity, decimal(1.0));
        mWorld.mDynamicsComponents.setMassInverse(mEntity, decimal(1.0));
    }

    RP3D_LOG(mLogger, Logger::Level::Information, Logger::Category::Body,
             "Body " + std::to_string(mEntity.id) + ": Set mass=" + std::to_string(mass));
}

// Remove a joint from the joints list
void RigidBody::removeJointFromJointsList(MemoryManager& memoryManager, const Joint* joint) {

    assert(joint != nullptr);
    assert(mJointsList != nullptr);

    // Remove the joint from the linked list of the joints of the first body
    if (mJointsList->joint == joint) {   // If the first element is the one to remove
        JointListElement* elementToRemove = mJointsList;
        mJointsList = elementToRemove->next;
        elementToRemove->~JointListElement();
        memoryManager.release(MemoryManager::AllocationType::Pool,
                              elementToRemove, sizeof(JointListElement));
    }
    else {  // If the element to remove is not the first one in the list
        JointListElement* currentElement = mJointsList;
        while (currentElement->next != nullptr) {
            if (currentElement->next->joint == joint) {
                JointListElement* elementToRemove = currentElement->next;
                currentElement->next = elementToRemove->next;
                elementToRemove->~JointListElement();
                memoryManager.release(MemoryManager::AllocationType::Pool,
                                      elementToRemove, sizeof(JointListElement));
                break;
            }
            currentElement = currentElement->next;
        }
    }
}

// Update the world inverse inertia tensor of the body
/// The inertia tensor I_w in world coordinates is computed with the
/// local inverse inertia tensor I_b^-1 in body coordinates
/// by I_w = R * I_b^-1 * R^T
/// where R is the rotation matrix (and R^T its transpose) of the
/// current orientation quaternion of the body
void RigidBody::updateInertiaTensorInverseWorld() {

    // TODO : Make sure we do this in a system

    Matrix3x3 orientation = mWorld.mTransformComponents.getTransform(mEntity).getOrientation().getMatrix();
    const Matrix3x3& inverseInertiaLocalTensor = mWorld.mDynamicsComponents.getInertiaTensorLocalInverse(mEntity);
    mWorld.mDynamicsComponents.setInverseInertiaTensorWorld(mEntity, orientation * inverseInertiaLocalTensor * orientation.getTranspose());
}

// Add a collision shape to the body.
/// When you add a collision shape to the body, an internal copy of this
/// collision shape will be created internally. Therefore, you can delete it
/// right after calling this method or use it later to add it to another body.
/// This method will return a pointer to a new proxy shape. A proxy shape is
/// an object that links a collision shape and a given body. You can use the
/// returned proxy shape to get and set information about the corresponding
/// collision shape for that body.
/**
 * @param collisionShape The collision shape you want to add to the body
 * @param transform The transformation of the collision shape that transforms the
 *        local-space of the collision shape into the local-space of the body
 * @param mass Mass (in kilograms) of the collision shape you want to add
 * @return A pointer to the proxy shape that has been created to link the body to
 *         the new collision shape you have added.
 */
ProxyShape* RigidBody::addCollisionShape(CollisionShape* collisionShape,
                                         const Transform& transform,
                                         decimal mass) {

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
                                                                   transform, collisionShape, mass, 0x0001, 0xFFFF);
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

    // Recompute the center of mass, total mass and inertia tensor of the body with the new
    // collision shape
    recomputeMassInformation();

    RP3D_LOG(mLogger, Logger::Level::Information, Logger::Category::Body,
             "Body " + std::to_string(mEntity.id) + ": Proxy shape " + std::to_string(proxyShape->getBroadPhaseId()) + " added to body");

    RP3D_LOG(mLogger, Logger::Level::Information, Logger::Category::ProxyShape,
             "ProxyShape " + std::to_string(proxyShape->getBroadPhaseId()) + ":  collisionShape=" +
             proxyShape->getCollisionShape()->to_string());

    // Return a pointer to the proxy collision shape
    return proxyShape;
}

// Remove a collision shape from the body
/// To remove a collision shape, you need to specify the pointer to the proxy
/// shape that has been returned when you have added the collision shape to the
/// body
/**
 * @param proxyShape The pointer of the proxy shape you want to remove
 */
void RigidBody::removeCollisionShape(ProxyShape* proxyShape) {

    // Remove the collision shape
    CollisionBody::removeCollisionShape(proxyShape);

    // Recompute the total mass, center of mass and inertia tensor
    recomputeMassInformation();
}

// Set the variable to know if the gravity is applied to this rigid body
/**
 * @param isEnabled True if you want the gravity to be applied to this body
 */
void RigidBody::enableGravity(bool isEnabled) {
    mWorld.mDynamicsComponents.setIsGravityEnabled(mEntity, isEnabled);

    RP3D_LOG(mLogger, Logger::Level::Information, Logger::Category::Body,
             "Body " + std::to_string(mEntity.id) + ": Set isGravityEnabled=" +
             (isEnabled ? "true" : "false"));
}

// Set the linear damping factor. This is the ratio of the linear velocity
// that the body will lose every at seconds of simulation.
/**
 * @param linearDamping The linear damping factor of this body
 */
void RigidBody::setLinearDamping(decimal linearDamping) {
    assert(linearDamping >= decimal(0.0));
    mWorld.mDynamicsComponents.setLinearDamping(mEntity, linearDamping);

    RP3D_LOG(mLogger, Logger::Level::Information, Logger::Category::Body,
             "Body " + std::to_string(mEntity.id) + ": Set linearDamping=" + std::to_string(linearDamping));
}

// Set the angular damping factor. This is the ratio of the angular velocity
// that the body will lose at every seconds of simulation.
/**
 * @param angularDamping The angular damping factor of this body
 */
void RigidBody::setAngularDamping(decimal angularDamping) {
    assert(angularDamping >= decimal(0.0));
    mWorld.mDynamicsComponents.setAngularDamping(mEntity, angularDamping);

    RP3D_LOG(mLogger, Logger::Level::Information, Logger::Category::Body,
             "Body " + std::to_string(mEntity.id) + ": Set angularDamping=" + std::to_string(angularDamping));
}

/// Update the transform of the body after a change of the center of mass
void RigidBody::updateTransformWithCenterOfMass() {

    // TODO : Make sure we compute this in a system

    // Translate the body according to the translation of the center of mass position
    Transform& transform = mWorld.mTransformComponents.getTransform(mEntity);
    const Vector3& centerOfMassWorld = mWorld.mDynamicsComponents.getCenterOfMassWorld(mEntity);
    const Vector3& centerOfMassLocal = mWorld.mDynamicsComponents.getCenterOfMassLocal(mEntity);
    transform.setPosition(centerOfMassWorld - transform.getOrientation() * centerOfMassLocal);
}

// Set a new material for this rigid body
/**
 * @param material The material you want to set to the body
 */
void RigidBody::setMaterial(const Material& material) {
    mMaterial = material;

    RP3D_LOG(mLogger, Logger::Level::Information, Logger::Category::Body,
             "Body " + std::to_string(mEntity.id) + ": Set Material" + mMaterial.to_string());
}

// Set the linear velocity of the rigid body.
/**
 * @param linearVelocity Linear velocity vector of the body
 */
void RigidBody::setLinearVelocity(const Vector3& linearVelocity) {

    // If it is a static body, we do nothing
    if (mType == BodyType::STATIC) return;

    // Update the linear velocity of the current body state
    mWorld.mDynamicsComponents.setLinearVelocity(mEntity, linearVelocity);

    // If the linear velocity is not zero, awake the body
    if (linearVelocity.lengthSquare() > decimal(0.0)) {
        setIsSleeping(false);
    }

    RP3D_LOG(mLogger, Logger::Level::Information, Logger::Category::Body,
             "Body " + std::to_string(mEntity.id) + ": Set linearVelocity=" + linearVelocity.to_string());
}

// Set the angular velocity.
/**
* @param angularVelocity The angular velocity vector of the body
*/
void RigidBody::setAngularVelocity(const Vector3& angularVelocity) {

    // TODO : Make sure this method is not called from the internal physics engine

    // If it is a static body, we do nothing
    if (mType == BodyType::STATIC) return;

    // Set the angular velocity
    mWorld.mDynamicsComponents.setAngularVelocity(mEntity, angularVelocity);

    // If the velocity is not zero, awake the body
    if (angularVelocity.lengthSquare() > decimal(0.0)) {
        setIsSleeping(false);
    }

    RP3D_LOG(mLogger, Logger::Level::Information, Logger::Category::Body,
             "Body " + std::to_string(mEntity.id) + ": Set angularVelocity=" + angularVelocity.to_string());
}

// Set the current position and orientation
/**
 * @param transform The transformation of the body that transforms the local-space
 *                  of the body into world-space
 */
void RigidBody::setTransform(const Transform& transform) {

    const Vector3 oldCenterOfMass = mWorld.mDynamicsComponents.getCenterOfMassWorld(mEntity);

    // Compute the new center of mass in world-space coordinates
    const Vector3& centerOfMassLocal = mWorld.mDynamicsComponents.getCenterOfMassLocal(mEntity);
    mWorld.mDynamicsComponents.setCenterOfMassWorld(mEntity, transform * centerOfMassLocal);

    // Update the linear velocity of the center of mass
    Vector3 linearVelocity = mWorld.mDynamicsComponents.getLinearVelocity(mEntity);
    const Vector3& angularVelocity = mWorld.mDynamicsComponents.getAngularVelocity(mEntity);
    const Vector3& centerOfMassWorld = mWorld.mDynamicsComponents.getCenterOfMassWorld(mEntity);
    linearVelocity += angularVelocity.cross(centerOfMassWorld - oldCenterOfMass);
    mWorld.mDynamicsComponents.setLinearVelocity(mEntity, linearVelocity);

    CollisionBody::setTransform(transform);

    // Update the transform of the body
    mWorld.mTransformComponents.setTransform(mEntity, transform);

    // Update the world inverse inertia tensor
    updateInertiaTensorInverseWorld();
}

// Recompute the center of mass, total mass and inertia tensor of the body using all
// the collision shapes attached to the body.
void RigidBody::recomputeMassInformation() {

    mWorld.mDynamicsComponents.setInitMass(mEntity, decimal(0.0));
    mWorld.mDynamicsComponents.setMassInverse(mEntity, decimal(0.0));
    if (!mIsInertiaTensorSetByUser) mWorld.mDynamicsComponents.setInverseInertiaTensorLocal(mEntity, Matrix3x3::zero());
    if (!mIsInertiaTensorSetByUser) mWorld.mDynamicsComponents.setInverseInertiaTensorWorld(mEntity, Matrix3x3::zero());
    if (!mIsCenterOfMassSetByUser) mWorld.mDynamicsComponents.setCenterOfMassLocal(mEntity, Vector3::zero());
    Matrix3x3 inertiaTensorLocal;
    inertiaTensorLocal.setToZero();

    const Transform& transform = mWorld.mTransformComponents.getTransform(mEntity);

    // If it is a STATIC or a KINEMATIC body
    if (mType == BodyType::STATIC || mType == BodyType::KINEMATIC) {
        mWorld.mDynamicsComponents.setCenterOfMassWorld(mEntity, transform.getPosition());
        return;
    }

    assert(mType == BodyType::DYNAMIC);

    // Compute the total mass of the body
    const List<Entity>& proxyShapesEntities = mWorld.mBodyComponents.getProxyShapes(mEntity);
    for (uint i=0; i < proxyShapesEntities.size(); i++) {
        ProxyShape* proxyShape = mWorld.mProxyShapesComponents.getProxyShape(proxyShapesEntities[i]);
        mWorld.mDynamicsComponents.setInitMass(mEntity, mWorld.mDynamicsComponents.getInitMass(mEntity) + proxyShape->getMass());

        if (!mIsCenterOfMassSetByUser) {
            mWorld.mDynamicsComponents.setCenterOfMassLocal(mEntity, mWorld.mDynamicsComponents.getCenterOfMassLocal(mEntity) +
                                                            proxyShape->getLocalToBodyTransform().getPosition() * proxyShape->getMass());
        }
    }

    if (mWorld.mDynamicsComponents.getInitMass(mEntity) > decimal(0.0)) {
        mWorld.mDynamicsComponents.setMassInverse(mEntity, decimal(1.0) / mWorld.mDynamicsComponents.getInitMass(mEntity));
    }
    else {
        mWorld.mDynamicsComponents.setCenterOfMassWorld(mEntity, transform.getPosition());
        return;
    }

    // Compute the center of mass
    const Vector3 oldCenterOfMass = mWorld.mDynamicsComponents.getCenterOfMassWorld(mEntity);

    if (!mIsCenterOfMassSetByUser) {
        mWorld.mDynamicsComponents.setCenterOfMassLocal(mEntity, mWorld.mDynamicsComponents.getCenterOfMassLocal(mEntity) * mWorld.mDynamicsComponents.getMassInverse(mEntity));
    }

    mWorld.mDynamicsComponents.setCenterOfMassWorld(mEntity, transform * mWorld.mDynamicsComponents.getCenterOfMassLocal(mEntity));

    if (!mIsInertiaTensorSetByUser) {

        // Compute the inertia tensor using all the collision shapes
        const List<Entity>& proxyShapesEntities = mWorld.mBodyComponents.getProxyShapes(mEntity);
        for (uint i=0; i < proxyShapesEntities.size(); i++) {

            ProxyShape* proxyShape = mWorld.mProxyShapesComponents.getProxyShape(proxyShapesEntities[i]);

            // Get the inertia tensor of the collision shape in its local-space
            Matrix3x3 inertiaTensor;
            proxyShape->getCollisionShape()->computeLocalInertiaTensor(inertiaTensor, proxyShape->getMass());

            // Convert the collision shape inertia tensor into the local-space of the body
            const Transform& shapeTransform = proxyShape->getLocalToBodyTransform();
            Matrix3x3 rotationMatrix = shapeTransform.getOrientation().getMatrix();
            inertiaTensor = rotationMatrix * inertiaTensor * rotationMatrix.getTranspose();

            // Use the parallel axis theorem to convert the inertia tensor w.r.t the collision shape
            // center into a inertia tensor w.r.t to the body origin.
            Vector3 offset = shapeTransform.getPosition() - mWorld.mDynamicsComponents.getCenterOfMassLocal(mEntity);
            decimal offsetSquare = offset.lengthSquare();
            Matrix3x3 offsetMatrix;
            offsetMatrix[0].setAllValues(offsetSquare, decimal(0.0), decimal(0.0));
            offsetMatrix[1].setAllValues(decimal(0.0), offsetSquare, decimal(0.0));
            offsetMatrix[2].setAllValues(decimal(0.0), decimal(0.0), offsetSquare);
            offsetMatrix[0] += offset * (-offset.x);
            offsetMatrix[1] += offset * (-offset.y);
            offsetMatrix[2] += offset * (-offset.z);
            offsetMatrix *= proxyShape->getMass();

            inertiaTensorLocal += inertiaTensor + offsetMatrix;
        }

        // Compute the local inverse inertia tensor
        mWorld.mDynamicsComponents.setInverseInertiaTensorLocal(mEntity, inertiaTensorLocal.getInverse());
    }

    // Update the world inverse inertia tensor
    updateInertiaTensorInverseWorld();

    // Update the linear velocity of the center of mass
    Vector3 linearVelocity = mWorld.mDynamicsComponents.getLinearVelocity(mEntity);
    Vector3 angularVelocity = mWorld.mDynamicsComponents.getAngularVelocity(mEntity);
    linearVelocity += angularVelocity.cross(mWorld.mDynamicsComponents.getCenterOfMassWorld(mEntity) - oldCenterOfMass);
    mWorld.mDynamicsComponents.setLinearVelocity(mEntity, linearVelocity);
}

// Return the linear velocity
/**
 * @return The linear velocity vector of the body
 */
Vector3 RigidBody::getLinearVelocity() const {
    return mWorld.mDynamicsComponents.getLinearVelocity(mEntity);
}

// Return the angular velocity of the body
/**
 * @return The angular velocity vector of the body
 */
Vector3 RigidBody::getAngularVelocity() const {
    return mWorld.mDynamicsComponents.getAngularVelocity(mEntity);
}

// Return true if the gravity needs to be applied to this rigid body
/**
 * @return True if the gravity is applied to the body
 */
bool RigidBody::isGravityEnabled() const {
    return mWorld.mDynamicsComponents.getIsGravityEnabled(mEntity);
}

// Apply an external torque to the body.
/// If the body is sleeping, calling this method will wake it up. Note that the
/// force will we added to the sum of the applied torques and that this sum will be
/// reset to zero at the end of each call of the DynamicsWorld::update() method.
/// You can only apply a force to a dynamic body otherwise, this method will do nothing.
/**
 * @param torque The external torque to apply on the body
 */
void RigidBody::applyTorque(const Vector3& torque) {

    // If it is not a dynamic body, we do nothing
    if (mType != BodyType::DYNAMIC) return;

    // Awake the body if it was sleeping
    if (mWorld.mBodyComponents.getIsSleeping(mEntity)) {
        setIsSleeping(false);
    }

    // Add the torque
    const Vector3& externalTorque = mWorld.mDynamicsComponents.getExternalTorque(mEntity);
    mWorld.mDynamicsComponents.setExternalTorque(mEntity, externalTorque + torque);
}

// Set the variable to know whether or not the body is sleeping
void RigidBody::setIsSleeping(bool isSleeping) {

    CollisionBody::setIsSleeping(isSleeping);

    if (isSleeping) {

        mWorld.mDynamicsComponents.setLinearVelocity(mEntity, Vector3::zero());
        mWorld.mDynamicsComponents.setAngularVelocity(mEntity, Vector3::zero());
        mWorld.mDynamicsComponents.setExternalForce(mEntity, Vector3::zero());
        mWorld.mDynamicsComponents.setExternalTorque(mEntity, Vector3::zero());
    }
}

#ifdef IS_PROFILING_ACTIVE

// Set the profiler
void RigidBody::setProfiler(Profiler* profiler) {

	CollisionBody::setProfiler(profiler);

	// Set the profiler for each proxy shape
    const List<Entity>& proxyShapesEntities = mWorld.mBodyComponents.getProxyShapes(mEntity);
    for (uint i=0; i < proxyShapesEntities.size(); i++) {

        ProxyShape* proxyShape = mWorld.mProxyShapesComponents.getProxyShape(proxyShapesEntities[i]);

		proxyShape->setProfiler(profiler);
	}
}

#endif
