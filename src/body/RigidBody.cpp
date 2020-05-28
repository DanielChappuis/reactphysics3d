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
#include <reactphysics3d/body/RigidBody.h>
#include <reactphysics3d/engine/PhysicsCommon.h>
#include <reactphysics3d/collision/shapes/CollisionShape.h>
#include <reactphysics3d/engine/PhysicsWorld.h>
#include <reactphysics3d/utils/Profiler.h>

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor
/**
* @param transform The transformation of the body
* @param world The world where the body has been added
* @param id The ID of the body
*/
RigidBody::RigidBody(PhysicsWorld& world, Entity entity) : CollisionBody(world, entity) {

}

// Return the type of the body
/**
 * @return The type of rigid body (static, kinematic or dynamic)
 */
BodyType RigidBody::getType() const {
    return mWorld.mRigidBodyComponents.getBodyType(mEntity);
}

// Set the type of the body
/// The type of the body can either STATIC, KINEMATIC or DYNAMIC as described bellow:
/// STATIC : A static body is simulated as if it has infinite mass, zero velocity but its position can be
///          changed manually. A static body does not collide with other static or kinematic bodies.
/// KINEMATIC : A kinematic body is simulated as if it has infinite mass, its velocity can be changed manually and its
///             position is computed by the physics engine. A kinematic body does not collide with
///             other static or kinematic bodies.
/// DYNAMIC : A dynamic body has non-zero mass, its velocity is determined by forces and its
///           position is determined by the physics engine. A dynamic body can collide with other
///           dynamic, static or kinematic bodies.
/**
 * @param type The type of the body (STATIC, KINEMATIC, DYNAMIC)
 */
void RigidBody::setType(BodyType type) {

    if (mWorld.mRigidBodyComponents.getBodyType(mEntity) == type) return;

    mWorld.mRigidBodyComponents.setBodyType(mEntity, type);

    // If it is a static body
    if (type == BodyType::STATIC) {

        // Reset the velocity to zero
        mWorld.mRigidBodyComponents.setLinearVelocity(mEntity, Vector3::zero());
        mWorld.mRigidBodyComponents.setAngularVelocity(mEntity, Vector3::zero());
    }

    // If it is a static or a kinematic body
    if (type == BodyType::STATIC || type == BodyType::KINEMATIC) {

        // Reset the inverse mass and inverse inertia tensor to zero
        mWorld.mRigidBodyComponents.setMassInverse(mEntity, decimal(0));
        mWorld.mRigidBodyComponents.setInverseInertiaTensorLocal(mEntity, Vector3::zero());
    }
    else {  // If it is a dynamic body

        const decimal mass = mWorld.mRigidBodyComponents.getMass(mEntity);

        if (mass > decimal(0.0)) {
            mWorld.mRigidBodyComponents.setMassInverse(mEntity, decimal(1.0) / mass) ;
        }
        else {
            mWorld.mRigidBodyComponents.setMassInverse(mEntity, decimal(0.0));
        }

        // Compute the inverse local inertia tensor
        const Vector3& inertiaTensorLocal = mWorld.mRigidBodyComponents.getLocalInertiaTensor(mEntity);
        Vector3 inverseInertiaTensorLocal(inertiaTensorLocal.x != decimal(0.0) ? decimal(1.0) / inertiaTensorLocal.x : 0,
                                          inertiaTensorLocal.y != decimal(0.0) ? decimal(1.0) / inertiaTensorLocal.y : 0,
                                          inertiaTensorLocal.z != decimal(0.0) ? decimal(1.0) / inertiaTensorLocal.z : 0);
        mWorld.mRigidBodyComponents.setInverseInertiaTensorLocal(mEntity, inverseInertiaTensorLocal);
    }

    // Awake the body
    setIsSleeping(false);

    // Update the active status of currently overlapping pairs
    updateOverlappingPairs();

    // Ask the broad-phase to test again the collision shapes of the body for collision
    // detection (as if the body has moved)
    askForBroadPhaseCollisionCheck();

    // Reset the force and torque on the body
    mWorld.mRigidBodyComponents.setExternalForce(mEntity, Vector3::zero());
    mWorld.mRigidBodyComponents.setExternalTorque(mEntity, Vector3::zero());

    RP3D_LOG(mWorld.mConfig.worldName, Logger::Level::Information, Logger::Category::Body,
             "Body " + std::to_string(mEntity.id) + ": Set type=" +
             (type == BodyType::STATIC ? "Static" : (type == BodyType::DYNAMIC ? "Dynamic" : "Kinematic")),  __FILE__, __LINE__);
}

// Method that return the mass of the body
/**
 * @return The mass (in kilograms) of the body
 */
decimal RigidBody::getMass() const {
    return mWorld.mRigidBodyComponents.getMass(mEntity);
}

// Apply an external force to the body at a given point (in local-space coordinates).
/// If the point is not at the center of mass of the body, it will also
/// generate some torque and therefore, change the angular velocity of the body.
/// If the body is sleeping, calling this method will wake it up. Note that the
/// force will we added to the sum of the applied forces and that this sum will be
/// reset to zero at the end of each call of the PhyscisWorld::update() method.
/// You can only apply a force to a dynamic body otherwise, this method will do nothing.
/**
 * @param force The force to apply on the body (in Newtons)
 * @param point The point where the force is applied (in local-space coordinates)
 */
void RigidBody::applyForceAtLocalPosition(const Vector3& force, const Vector3& point) {

    // If it is not a dynamic body, we do nothing
    if (mWorld.mRigidBodyComponents.getBodyType(mEntity) != BodyType::DYNAMIC) return;

    // Awake the body if it was sleeping
    if (mWorld.mRigidBodyComponents.getIsSleeping(mEntity)) {
        setIsSleeping(false);
    }

    // Add the force
    const Vector3& externalForce = mWorld.mRigidBodyComponents.getExternalForce(mEntity);
    mWorld.mRigidBodyComponents.setExternalForce(mEntity, externalForce + force);

    // Add the torque
    const Vector3& externalTorque = mWorld.mRigidBodyComponents.getExternalTorque(mEntity);
    const Vector3& centerOfMassWorld = mWorld.mRigidBodyComponents.getCenterOfMassWorld(mEntity);
    const Vector3 worldPoint = mWorld.mTransformComponents.getTransform(mEntity) * point;
    mWorld.mRigidBodyComponents.setExternalTorque(mEntity, externalTorque + (worldPoint - centerOfMassWorld).cross(force));
}

// Apply an external force to the body at a given point (in world-space coordinates).
/// If the point is not at the center of mass of the body, it will also
/// generate some torque and therefore, change the angular velocity of the body.
/// If the body is sleeping, calling this method will wake it up. Note that the
/// force will we added to the sum of the applied forces and that this sum will be
/// reset to zero at the end of each call of the PhyscisWorld::update() method.
/// You can only apply a force to a dynamic body otherwise, this method will do nothing.
/**
 * @param force The force to apply on the body (in Newtons)
 * @param point The point where the force is applied (in world-space coordinates)
 */
void RigidBody::applyForceAtWorldPosition(const Vector3& force, const Vector3& point) {

    // If it is not a dynamic body, we do nothing
    if (mWorld.mRigidBodyComponents.getBodyType(mEntity) != BodyType::DYNAMIC) return;

    // Awake the body if it was sleeping
    if (mWorld.mRigidBodyComponents.getIsSleeping(mEntity)) {
        setIsSleeping(false);
    }

    // Add the force
    const Vector3& externalForce = mWorld.mRigidBodyComponents.getExternalForce(mEntity);
    mWorld.mRigidBodyComponents.setExternalForce(mEntity, externalForce + force);

    // Add the torque
    const Vector3& externalTorque = mWorld.mRigidBodyComponents.getExternalTorque(mEntity);
    const Vector3& centerOfMassWorld = mWorld.mRigidBodyComponents.getCenterOfMassWorld(mEntity);
    mWorld.mRigidBodyComponents.setExternalTorque(mEntity, externalTorque + (point - centerOfMassWorld).cross(force));
}

// Return the local inertia tensor of the body (in body coordinates)
/**
 * @return A vector with the three values of the diagonal 3x3 matrix of the local-space inertia tensor
 */
const Vector3& RigidBody::getLocalInertiaTensor() const {

    return mWorld.mRigidBodyComponents.getLocalInertiaTensor(mEntity);
}

// Set the local inertia tensor of the body (in local-space coordinates)
/// Note that an inertia tensor with a zero value on its diagonal is interpreted as infinite inertia.
/**
 * @param inertiaTensorLocal A vector with the three values of the diagonal 3x3 matrix of the local-space inertia tensor
 */
void RigidBody::setLocalInertiaTensor(const Vector3& inertiaTensorLocal) {

    mWorld.mRigidBodyComponents.setLocalInertiaTensor(mEntity, inertiaTensorLocal);

    // Compute the inverse local inertia tensor
    Vector3 inverseInertiaTensorLocal(inertiaTensorLocal.x != decimal(0.0) ? decimal(1.0) / inertiaTensorLocal.x : 0,
                                      inertiaTensorLocal.y != decimal(0.0) ? decimal(1.0) / inertiaTensorLocal.y : 0,
                                      inertiaTensorLocal.z != decimal(0.0) ? decimal(1.0) / inertiaTensorLocal.z : 0);
    mWorld.mRigidBodyComponents.setInverseInertiaTensorLocal(mEntity, inverseInertiaTensorLocal);

    RP3D_LOG(mWorld.mConfig.worldName, Logger::Level::Information, Logger::Category::Body,
             "Body " + std::to_string(mEntity.id) + ": Set inertiaTensorLocal=" + inertiaTensorLocal.to_string(),  __FILE__, __LINE__);
}

// Apply an external force to the body at its center of mass.
/// If the body is sleeping, calling this method will wake it up. Note that the
/// force will we added to the sum of the applied forces and that this sum will be
/// reset to zero at the end of each call of the PhyscisWorld::update() method.
/// You can only apply a force to a dynamic body otherwise, this method will do nothing.
/**
 * @param force The external force to apply on the center of mass of the body (in Newtons)
 */
void RigidBody::applyForceToCenterOfMass(const Vector3& force) {

    // If it is not a dynamic body, we do nothing
    if (mWorld.mRigidBodyComponents.getBodyType(mEntity) != BodyType::DYNAMIC) return;

    // Awake the body if it was sleeping
    if (mWorld.mRigidBodyComponents.getIsSleeping(mEntity)) {
        setIsSleeping(false);
    }

    // Add the force
    const Vector3& externalForce = mWorld.mRigidBodyComponents.getExternalForce(mEntity);
    mWorld.mRigidBodyComponents.setExternalForce(mEntity, externalForce + force);
}

// Return the linear velocity damping factor
/**
 * @return The linear damping factor of this body
 */
decimal RigidBody::getLinearDamping() const {
    return mWorld.mRigidBodyComponents.getLinearDamping(mEntity);
}

// Return the angular velocity damping factor
/**
 * @return The angular damping factor of this body
 */
decimal RigidBody::getAngularDamping() const {
    return mWorld.mRigidBodyComponents.getAngularDamping(mEntity);
}

// Set the center of mass of the body (in local-space coordinates)
/// This method does not move the rigid body in the world.
/**
 * @param centerOfMass The center of mass of the body in local-space coordinates
 */
void RigidBody::setLocalCenterOfMass(const Vector3& centerOfMass) {

    const Vector3 oldCenterOfMass = mWorld.mRigidBodyComponents.getCenterOfMassWorld(mEntity);
    mWorld.mRigidBodyComponents.setCenterOfMassLocal(mEntity, centerOfMass);

    // Compute the center of mass in world-space coordinates
    mWorld.mRigidBodyComponents.setCenterOfMassWorld(mEntity, mWorld.mTransformComponents.getTransform(mEntity) * centerOfMass);

    // Update the linear velocity of the center of mass
    Vector3 linearVelocity = mWorld.mRigidBodyComponents.getAngularVelocity(mEntity);
    const Vector3& angularVelocity = mWorld.mRigidBodyComponents.getAngularVelocity(mEntity);
    const Vector3& centerOfMassWorld = mWorld.mRigidBodyComponents.getCenterOfMassWorld(mEntity);
    linearVelocity += angularVelocity.cross(centerOfMassWorld - oldCenterOfMass);
    mWorld.mRigidBodyComponents.setLinearVelocity(mEntity, linearVelocity);

    RP3D_LOG(mWorld.mConfig.worldName, Logger::Level::Information, Logger::Category::Body,
             "Body " + std::to_string(mEntity.id) + ": Set centerOfMassLocal=" + centerOfMass.to_string(),  __FILE__, __LINE__);
}

// Return the center of mass of the body (in local-space coordinates)
/**
 * @return The local-space position of the center of mass of the body
 */
const Vector3& RigidBody::getLocalCenterOfMass() const {
    return mWorld.mRigidBodyComponents.getCenterOfMassLocal(mEntity);
}

// Compute and set the local-space center of mass of the body using its colliders
/// This method uses the shape, mass density and transforms of the colliders to set
/// the center of mass of the body. Note that calling this method will overwrite the
/// mass that has been previously set with the RigidBody::setCenterOfMass() method. Moreover, this method
/// does not use the mass set by the user with the RigidBody::setMass() method to compute the center
/// of mass but only the mass density and volume of the colliders.
void RigidBody::updateLocalCenterOfMassFromColliders() {

    const Vector3 oldCenterOfMassWorld = mWorld.mRigidBodyComponents.getCenterOfMassWorld(mEntity);

    Vector3 centerOfMassLocal = computeCenterOfMass();

    const Vector3 centerOfMassWorld = mWorld.mTransformComponents.getTransform(mEntity) * centerOfMassLocal;

    // Set the center of mass
    mWorld.mRigidBodyComponents.setCenterOfMassLocal(mEntity, centerOfMassLocal);
    mWorld.mRigidBodyComponents.setCenterOfMassWorld(mEntity, centerOfMassWorld);

    // Update the linear velocity of the center of mass
    Vector3 linearVelocity = mWorld.mRigidBodyComponents.getAngularVelocity(mEntity);
    const Vector3& angularVelocity = mWorld.mRigidBodyComponents.getAngularVelocity(mEntity);
    linearVelocity += angularVelocity.cross(centerOfMassWorld - oldCenterOfMassWorld);
    mWorld.mRigidBodyComponents.setLinearVelocity(mEntity, linearVelocity);

    RP3D_LOG(mWorld.mConfig.worldName, Logger::Level::Information, Logger::Category::Body,
             "Body " + std::to_string(mEntity.id) + ": Set centerOfMassLocal=" + centerOfMassLocal.to_string(),  __FILE__, __LINE__);
}

// Compute and return the local-space center of mass of the body using its colliders
Vector3 RigidBody::computeCenterOfMass() const {

    decimal totalMass = decimal(0.0);
    Vector3 centerOfMassLocal(0, 0, 0);

    // Compute the local center of mass
    const List<Entity>& colliderEntities = mWorld.mCollisionBodyComponents.getColliders(mEntity);
    for (uint i=0; i < colliderEntities.size(); i++) {

        Collider* collider = mWorld.mCollidersComponents.getCollider(colliderEntities[i]);

        const decimal colliderVolume = mWorld.mCollidersComponents.getCollisionShape(colliderEntities[i])->getVolume();
        const decimal colliderMassDensity = collider->getMaterial().getMassDensity();

        const decimal colliderMass = colliderVolume * colliderMassDensity;

        totalMass += colliderMass;
        centerOfMassLocal += colliderMass * mWorld.mCollidersComponents.getLocalToBodyTransform(colliderEntities[i]).getPosition();
    }

    if (totalMass > decimal(0.0)) {
        centerOfMassLocal /= totalMass;
    }

    return centerOfMassLocal;
}

// Compute the local-space inertia tensor and total mass of the body using its colliders
void RigidBody::computeMassAndInertiaTensorLocal(Vector3& inertiaTensorLocal, decimal& totalMass) const {

    inertiaTensorLocal.setToZero();
    totalMass = decimal(0.0);

    Matrix3x3 tempLocalInertiaTensor = Matrix3x3::zero();

    const Vector3 centerOfMassLocal = mWorld.mRigidBodyComponents.getCenterOfMassLocal(mEntity);

    // Compute the inertia tensor using all the colliders
    const List<Entity>& colliderEntities = mWorld.mCollisionBodyComponents.getColliders(mEntity);
    for (uint i=0; i < colliderEntities.size(); i++) {

        Collider* collider = mWorld.mCollidersComponents.getCollider(colliderEntities[i]);

        const decimal colliderVolume = mWorld.mCollidersComponents.getCollisionShape(colliderEntities[i])->getVolume();
        const decimal colliderMassDensity = collider->getMaterial().getMassDensity();
        const decimal colliderMass = colliderVolume * colliderMassDensity;

        totalMass += colliderMass;

        // Get the inertia tensor of the collider in its local-space
        Vector3 shapeLocalInertiaTensor = collider->getCollisionShape()->getLocalInertiaTensor(colliderMass);

        // Convert the collider inertia tensor into the local-space of the body
        const Transform& shapeTransform = collider->getLocalToBodyTransform();
        Matrix3x3 rotationMatrix = shapeTransform.getOrientation().getMatrix();
        Matrix3x3 rotationMatrixTranspose = rotationMatrix.getTranspose();
        rotationMatrixTranspose[0] *= shapeLocalInertiaTensor.x;
        rotationMatrixTranspose[1] *= shapeLocalInertiaTensor.y;
        rotationMatrixTranspose[2] *= shapeLocalInertiaTensor.z;
        Matrix3x3 inertiaTensor = rotationMatrix * rotationMatrixTranspose;

        // Use the parallel axis theorem to convert the inertia tensor w.r.t the collider
        // center into a inertia tensor w.r.t to the body origin.
        Vector3 offset = shapeTransform.getPosition() - centerOfMassLocal;
        decimal offsetSquare = offset.lengthSquare();
        Matrix3x3 offsetMatrix;
        offsetMatrix[0].setAllValues(offsetSquare, decimal(0.0), decimal(0.0));
        offsetMatrix[1].setAllValues(decimal(0.0), offsetSquare, decimal(0.0));
        offsetMatrix[2].setAllValues(decimal(0.0), decimal(0.0), offsetSquare);
        offsetMatrix[0] += offset * (-offset.x);
        offsetMatrix[1] += offset * (-offset.y);
        offsetMatrix[2] += offset * (-offset.z);
        offsetMatrix *= colliderMass;

        tempLocalInertiaTensor += inertiaTensor + offsetMatrix;
    }

    // Get the diagonal value of the computed local inertia tensor
    inertiaTensorLocal.setAllValues(tempLocalInertiaTensor[0][0], tempLocalInertiaTensor[1][1], tempLocalInertiaTensor[2][2]);
}

// Compute and set the local-space inertia tensor of the body using its colliders
/// This method uses the shape, mass density and transforms of the colliders to set
/// the local-space inertia tensor of the body. Note that calling this method will overwrite the
/// mass that has been set with the RigidBody::setInertiaTensorLocal() method.
void RigidBody::updateLocalInertiaTensorFromColliders() {

    // Compute the local-space inertia tensor
    Vector3 inertiaTensorLocal;
    decimal totalMass;
    computeMassAndInertiaTensorLocal(inertiaTensorLocal, totalMass);

    mWorld.mRigidBodyComponents.setLocalInertiaTensor(mEntity, inertiaTensorLocal);

    // Compute the inverse local inertia tensor
    Vector3 inverseInertiaTensorLocal(inertiaTensorLocal.x != decimal(0.0) ? decimal(1.0) / inertiaTensorLocal.x : 0,
                                      inertiaTensorLocal.y != decimal(0.0) ? decimal(1.0) / inertiaTensorLocal.y : 0,
                                      inertiaTensorLocal.z != decimal(0.0) ? decimal(1.0) / inertiaTensorLocal.z : 0);
    mWorld.mRigidBodyComponents.setInverseInertiaTensorLocal(mEntity, inverseInertiaTensorLocal);

    RP3D_LOG(mWorld.mConfig.worldName, Logger::Level::Information, Logger::Category::Body,
             "Body " + std::to_string(mEntity.id) + ": Set inertiaTensorLocal=" + inertiaTensorLocal.to_string(),  __FILE__, __LINE__);
}

// Compute and set the mass of the body using its colliders
/// This method uses the shape, mass density and transforms of the colliders to set
/// the total mass of the body. Note that calling this method will overwrite the
/// mass that has been set with the RigidBody::setMass() method
void RigidBody::updateMassFromColliders() {

    decimal totalMass = decimal(0.0);

    // Compute the total mass of the body
    const List<Entity>& colliderEntities = mWorld.mCollisionBodyComponents.getColliders(mEntity);
    for (uint i=0; i < colliderEntities.size(); i++) {
        Collider* collider = mWorld.mCollidersComponents.getCollider(colliderEntities[i]);

        const decimal colliderVolume = mWorld.mCollidersComponents.getCollisionShape(colliderEntities[i])->getVolume();
        const decimal colliderMassDensity = collider->getMaterial().getMassDensity();

        const decimal colliderMass = colliderVolume * colliderMassDensity;

        totalMass += colliderMass;
    }

    // Set the mass
    mWorld.mRigidBodyComponents.setMass(mEntity, totalMass);

    // Compute the inverse mass
    if (totalMass > decimal(0.0)) {
        mWorld.mRigidBodyComponents.setMassInverse(mEntity, decimal(1.0) / totalMass);
    }
    else {
        mWorld.mRigidBodyComponents.setMassInverse(mEntity, decimal(0.0));
    }

    RP3D_LOG(mWorld.mConfig.worldName, Logger::Level::Information, Logger::Category::Body,
             "Body " + std::to_string(mEntity.id) + ": Set mass=" + std::to_string(totalMass),  __FILE__, __LINE__);
}

// Compute and set the center of mass, the mass and the local-space inertia tensor of the body using its colliders
/// This method uses the shape, mass density and transform of the colliders of the body to set
/// the total mass, the center of mass and the local inertia tensor of the body.
/// Note that calling this method will overwrite the
/// mass that has been set with the RigidBody::setMass(), the center of mass that has been
/// set with RigidBody::setCenterOfMass() and the local inertia tensor that has been set with
/// RigidBody::setInertiaTensorLocal().
void RigidBody::updateMassPropertiesFromColliders() {

    const Vector3 oldCenterOfMassWorld = mWorld.mRigidBodyComponents.getCenterOfMassWorld(mEntity);

    // Compute the local center of mass
    Vector3 centerOfMassLocal = computeCenterOfMass();

    const Vector3 centerOfMassWorld = mWorld.mTransformComponents.getTransform(mEntity) * centerOfMassLocal;

    // Set the center of mass
    mWorld.mRigidBodyComponents.setCenterOfMassLocal(mEntity, centerOfMassLocal);
    mWorld.mRigidBodyComponents.setCenterOfMassWorld(mEntity, centerOfMassWorld);

    // Update the linear velocity of the center of mass
    Vector3 linearVelocity = mWorld.mRigidBodyComponents.getAngularVelocity(mEntity);
    const Vector3& angularVelocity = mWorld.mRigidBodyComponents.getAngularVelocity(mEntity);
    linearVelocity += angularVelocity.cross(centerOfMassWorld - oldCenterOfMassWorld);
    mWorld.mRigidBodyComponents.setLinearVelocity(mEntity, linearVelocity);

    RP3D_LOG(mWorld.mConfig.worldName, Logger::Level::Information, Logger::Category::Body,
             "Body " + std::to_string(mEntity.id) + ": Set centerOfMassLocal=" + centerOfMassLocal.to_string(),  __FILE__, __LINE__);

    // Compute the mass and local-space inertia tensor
    Vector3 inertiaTensorLocal;
    decimal totalMass;
    computeMassAndInertiaTensorLocal(inertiaTensorLocal, totalMass);

    mWorld.mRigidBodyComponents.setLocalInertiaTensor(mEntity, inertiaTensorLocal);

    // Compute the inverse local inertia tensor
    Vector3 inverseInertiaTensorLocal(inertiaTensorLocal.x != decimal(0.0) ? decimal(1.0) / inertiaTensorLocal.x : 0,
                                      inertiaTensorLocal.y != decimal(0.0) ? decimal(1.0) / inertiaTensorLocal.y : 0,
                                      inertiaTensorLocal.z != decimal(0.0) ? decimal(1.0) / inertiaTensorLocal.z : 0);
    mWorld.mRigidBodyComponents.setInverseInertiaTensorLocal(mEntity, inverseInertiaTensorLocal);

    RP3D_LOG(mWorld.mConfig.worldName, Logger::Level::Information, Logger::Category::Body,
             "Body " + std::to_string(mEntity.id) + ": Set inertiaTensorLocal=" + inertiaTensorLocal.to_string(),  __FILE__, __LINE__);

    // Set the mass
    mWorld.mRigidBodyComponents.setMass(mEntity, totalMass);

    // Compute the inverse mass
    if (totalMass > decimal(0.0)) {
        mWorld.mRigidBodyComponents.setMassInverse(mEntity, decimal(1.0) / totalMass);
    }
    else {
        mWorld.mRigidBodyComponents.setMassInverse(mEntity, decimal(0.0));
    }

    RP3D_LOG(mWorld.mConfig.worldName, Logger::Level::Information, Logger::Category::Body,
             "Body " + std::to_string(mEntity.id) + ": Set mass=" + std::to_string(totalMass),  __FILE__, __LINE__);
}

// Set the mass of the rigid body
/// Note that a mass of zero is interpreted as infinite mass.
/**
 * @param mass The mass (in kilograms) of the body
 */
void RigidBody::setMass(decimal mass) {

    mWorld.mRigidBodyComponents.setMass(mEntity, mass);

    if (mass < decimal(0.0)) {

        RP3D_LOG(mWorld.mConfig.worldName, Logger::Level::Error, Logger::Category::Body,
                 "Error when setting the mass of a rigid body: the mass must be a positive value",  __FILE__, __LINE__);
    }

    if (mWorld.mRigidBodyComponents.getMass(mEntity) > decimal(0.0)) {
        mWorld.mRigidBodyComponents.setMassInverse(mEntity, decimal(1.0) / mass);
    }
    else {
        mWorld.mRigidBodyComponents.setMassInverse(mEntity, decimal(0.0));

        if (mWorld.mRigidBodyComponents.getMass(mEntity) < decimal(0.0)) {

            RP3D_LOG(mWorld.mConfig.worldName, Logger::Level::Error, Logger::Category::Body,
                     "Error when setting mass of body " + std::to_string(mEntity.id) + ": mass cannot be negative",  __FILE__, __LINE__);
        }
    }

    RP3D_LOG(mWorld.mConfig.worldName, Logger::Level::Information, Logger::Category::Body,
             "Body " + std::to_string(mEntity.id) + ": Set mass=" + std::to_string(mass),  __FILE__, __LINE__);
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
Collider* RigidBody::addCollider(CollisionShape* collisionShape, const Transform& transform) {

    // Create a new entity for the collider
    Entity colliderEntity = mWorld.mEntityManager.createEntity();

    // Check that the transform is valid
    if (!transform.isValid()) {
        RP3D_LOG(mWorld.mConfig.worldName, Logger::Level::Error, Logger::Category::Collider,
                 "Error when adding a collider: the init transform is not valid",  __FILE__, __LINE__);
    }
    assert(transform.isValid());

    // Create a new collider for the body
    Collider* collider = new (mWorld.mMemoryManager.allocate(MemoryManager::AllocationType::Pool,
                                      sizeof(Collider))) Collider(colliderEntity, this, mWorld.mMemoryManager);

    // Add the collider component to the entity of the body
    Vector3 localBoundsMin;
    Vector3 localBoundsMax;
    // TODO : Maybe this method can directly returns an AABB
    collisionShape->getLocalBounds(localBoundsMin, localBoundsMax);
    const Transform localToWorldTransform = mWorld.mTransformComponents.getTransform(mEntity) * transform;
    ColliderComponents::ColliderComponent colliderComponent(mEntity, collider, AABB(localBoundsMin, localBoundsMax),
                                                            transform, collisionShape, 0x0001, 0xFFFF, localToWorldTransform);
    bool isSleeping = mWorld.mRigidBodyComponents.getIsSleeping(mEntity);
    mWorld.mCollidersComponents.addComponent(colliderEntity, isSleeping, colliderComponent);

    mWorld.mCollisionBodyComponents.addColliderToBody(mEntity, colliderEntity);

    // Assign the collider with the collision shape
    collisionShape->addCollider(collider);

#ifdef IS_RP3D_PROFILING_ENABLED


	// Set the profiler
    collider->setProfiler(mProfiler);

#endif

    // Compute the world-space AABB of the new collision shape
    AABB aabb;
    collisionShape->computeAABB(aabb, mWorld.mTransformComponents.getTransform(mEntity) * transform);

    // Notify the collision detection about this new collision shape
    mWorld.mCollisionDetection.addCollider(collider, aabb);

    RP3D_LOG(mWorld.mConfig.worldName, Logger::Level::Information, Logger::Category::Body,
             "Body " + std::to_string(mEntity.id) + ": Collider " + std::to_string(collider->getBroadPhaseId()) + " added to body",  __FILE__, __LINE__);

    RP3D_LOG(mWorld.mConfig.worldName, Logger::Level::Information, Logger::Category::Collider,
             "Collider " + std::to_string(collider->getBroadPhaseId()) + ":  collisionShape=" +
             collider->getCollisionShape()->to_string(),  __FILE__, __LINE__);

    // Return a pointer to the collider
    return collider;
}

// Remove a collider from the body
/// To remove a collider, you need to specify its pointer.
/**
 * @param collider The pointer of the collider you want to remove
 */
void RigidBody::removeCollider(Collider* collider) {

    // Remove the collision shape
    CollisionBody::removeCollider(collider);
}

// Set the variable to know if the gravity is applied to this rigid body
/**
 * @param isEnabled True if you want the gravity to be applied to this body
 */
void RigidBody::enableGravity(bool isEnabled) {
    mWorld.mRigidBodyComponents.setIsGravityEnabled(mEntity, isEnabled);

    RP3D_LOG(mWorld.mConfig.worldName, Logger::Level::Information, Logger::Category::Body,
             "Body " + std::to_string(mEntity.id) + ": Set isGravityEnabled=" +
             (isEnabled ? "true" : "false"),  __FILE__, __LINE__);
}

// Set the linear damping factor. This is the ratio of the linear velocity
// that the body will lose every at seconds of simulation.
/**
 * @param linearDamping The linear damping factor of this body
 */
void RigidBody::setLinearDamping(decimal linearDamping) {
    assert(linearDamping >= decimal(0.0));

    if (linearDamping >= decimal(0.0)) {

        mWorld.mRigidBodyComponents.setLinearDamping(mEntity, linearDamping);

        RP3D_LOG(mWorld.mConfig.worldName, Logger::Level::Information, Logger::Category::Body,
                 "Body " + std::to_string(mEntity.id) + ": Set linearDamping=" + std::to_string(linearDamping),  __FILE__, __LINE__);
    }
    else {

        RP3D_LOG(mWorld.mConfig.worldName, Logger::Level::Error, Logger::Category::Body,
                 "Error when setting the linear damping of body " + std::to_string(mEntity.id) + ": linear damping cannot be negative",  __FILE__, __LINE__);
    }
}

// Set the angular damping factor. This is the ratio of the angular velocity
// that the body will lose at every seconds of simulation.
/**
 * @param angularDamping The angular damping factor of this body
 */
void RigidBody::setAngularDamping(decimal angularDamping) {
    assert(angularDamping >= decimal(0.0));

    if (angularDamping >= decimal(0.0)) {

        mWorld.mRigidBodyComponents.setAngularDamping(mEntity, angularDamping);

        RP3D_LOG(mWorld.mConfig.worldName, Logger::Level::Information, Logger::Category::Body,
                 "Body " + std::to_string(mEntity.id) + ": Set angularDamping=" + std::to_string(angularDamping),  __FILE__, __LINE__);
    }
    else {
        RP3D_LOG(mWorld.mConfig.worldName, Logger::Level::Error, Logger::Category::Body,
                 "Error when setting the angular damping of body " + std::to_string(mEntity.id) + ": angular damping cannot be negative",  __FILE__, __LINE__);
    }
}

// Set the linear velocity of the rigid body.
/**
 * @param linearVelocity Linear velocity vector of the body
 */
void RigidBody::setLinearVelocity(const Vector3& linearVelocity) {

    // If it is a static body, we do nothing
    if (mWorld.mRigidBodyComponents.getBodyType(mEntity) == BodyType::STATIC) return;

    // Update the linear velocity of the current body state
    mWorld.mRigidBodyComponents.setLinearVelocity(mEntity, linearVelocity);

    // If the linear velocity is not zero, awake the body
    if (linearVelocity.lengthSquare() > decimal(0.0)) {
        setIsSleeping(false);
    }

    RP3D_LOG(mWorld.mConfig.worldName, Logger::Level::Information, Logger::Category::Body,
             "Body " + std::to_string(mEntity.id) + ": Set linearVelocity=" + linearVelocity.to_string(),  __FILE__, __LINE__);
}

// Set the angular velocity.
/**
* @param angularVelocity The angular velocity vector of the body
*/
void RigidBody::setAngularVelocity(const Vector3& angularVelocity) {

    // If it is a static body, we do nothing
    if (mWorld.mRigidBodyComponents.getBodyType(mEntity) == BodyType::STATIC) return;

    // Set the angular velocity
    mWorld.mRigidBodyComponents.setAngularVelocity(mEntity, angularVelocity);

    // If the velocity is not zero, awake the body
    if (angularVelocity.lengthSquare() > decimal(0.0)) {
        setIsSleeping(false);
    }

    RP3D_LOG(mWorld.mConfig.worldName, Logger::Level::Information, Logger::Category::Body,
             "Body " + std::to_string(mEntity.id) + ": Set angularVelocity=" + angularVelocity.to_string(),  __FILE__, __LINE__);
}

// Set the current position and orientation
/**
 * @param transform The transformation of the body that transforms the local-space
 *                  of the body into world-space
 */
void RigidBody::setTransform(const Transform& transform) {

    const Vector3 oldCenterOfMass = mWorld.mRigidBodyComponents.getCenterOfMassWorld(mEntity);

    // Compute the new center of mass in world-space coordinates
    const Vector3& centerOfMassLocal = mWorld.mRigidBodyComponents.getCenterOfMassLocal(mEntity);
    mWorld.mRigidBodyComponents.setCenterOfMassWorld(mEntity, transform * centerOfMassLocal);

    // Update the linear velocity of the center of mass
    Vector3 linearVelocity = mWorld.mRigidBodyComponents.getLinearVelocity(mEntity);
    const Vector3& angularVelocity = mWorld.mRigidBodyComponents.getAngularVelocity(mEntity);
    const Vector3& centerOfMassWorld = mWorld.mRigidBodyComponents.getCenterOfMassWorld(mEntity);
    linearVelocity += angularVelocity.cross(centerOfMassWorld - oldCenterOfMass);
    mWorld.mRigidBodyComponents.setLinearVelocity(mEntity, linearVelocity);

    CollisionBody::setTransform(transform);

    // Awake the body if it is sleeping
    setIsSleeping(false);
}

// Return the linear velocity
/**
 * @return The linear velocity vector of the body
 */
Vector3 RigidBody::getLinearVelocity() const {
    return mWorld.mRigidBodyComponents.getLinearVelocity(mEntity);
}

// Return the angular velocity of the body
/**
 * @return The angular velocity vector of the body
 */
Vector3 RigidBody::getAngularVelocity() const {
    return mWorld.mRigidBodyComponents.getAngularVelocity(mEntity);
}

// Return true if the gravity needs to be applied to this rigid body
/**
 * @return True if the gravity is applied to the body
 */
bool RigidBody::isGravityEnabled() const {
    return mWorld.mRigidBodyComponents.getIsGravityEnabled(mEntity);
}

// Apply an external torque to the body.
/// If the body is sleeping, calling this method will wake it up. Note that the
/// force will we added to the sum of the applied torques and that this sum will be
/// reset to zero at the end of each call of the PhyscisWorld::update() method.
/// You can only apply a force to a dynamic body otherwise, this method will do nothing.
/**
 * @param torque The external torque to apply on the body
 */
void RigidBody::applyTorque(const Vector3& torque) {

    // If it is not a dynamic body, we do nothing
    if (mWorld.mRigidBodyComponents.getBodyType(mEntity) != BodyType::DYNAMIC) return;

    // Awake the body if it was sleeping
    if (mWorld.mRigidBodyComponents.getIsSleeping(mEntity)) {
        setIsSleeping(false);
    }

    // Add the torque
    const Vector3& externalTorque = mWorld.mRigidBodyComponents.getExternalTorque(mEntity);
    mWorld.mRigidBodyComponents.setExternalTorque(mEntity, externalTorque + torque);
}

// Set the variable to know whether or not the body is sleeping
void RigidBody::setIsSleeping(bool isSleeping) {

    bool isBodySleeping = mWorld.mRigidBodyComponents.getIsSleeping(mEntity);

    if (isBodySleeping == isSleeping) return;

    // If the body is not active, do nothing (it is sleeping)
    if (!mWorld.mCollisionBodyComponents.getIsActive(mEntity)) {
        assert(isBodySleeping);
        return;
    }

    if (isSleeping) {
        mWorld.mRigidBodyComponents.setSleepTime(mEntity, decimal(0.0));
    }
    else {
        if (isBodySleeping) {
            mWorld.mRigidBodyComponents.setSleepTime(mEntity, decimal(0.0));
        }
    }

    mWorld.mRigidBodyComponents.setIsSleeping(mEntity, isSleeping);

    // Notify all the components
    mWorld.setBodyDisabled(mEntity, isSleeping);

    // Update the currently overlapping pairs
    updateOverlappingPairs();

    if (isSleeping) {

        mWorld.mRigidBodyComponents.setLinearVelocity(mEntity, Vector3::zero());
        mWorld.mRigidBodyComponents.setAngularVelocity(mEntity, Vector3::zero());
        mWorld.mRigidBodyComponents.setExternalForce(mEntity, Vector3::zero());
        mWorld.mRigidBodyComponents.setExternalTorque(mEntity, Vector3::zero());
    }

    RP3D_LOG(mWorld.mConfig.worldName, Logger::Level::Information, Logger::Category::Body,
         "Body " + std::to_string(mEntity.id) + ": Set isSleeping=" +
         (isSleeping ? "true" : "false"),  __FILE__, __LINE__);
}

// Update whether the current overlapping pairs where this body is involed are active or not
void RigidBody::updateOverlappingPairs() {

    // For each collider of the body
    const List<Entity>& colliderEntities = mWorld.mCollisionBodyComponents.getColliders(mEntity);
    for (uint i=0; i < colliderEntities.size(); i++) {

        // Get the currently overlapping pairs for this collider
        List<uint64> overlappingPairs = mWorld.mCollidersComponents.getOverlappingPairs(colliderEntities[i]);

        for (uint j=0; j < overlappingPairs.size(); j++) {

            mWorld.mCollisionDetection.mOverlappingPairs.updateOverlappingPairIsActive(overlappingPairs[j]);
        }
    }
}

/// Return the inverse of the inertia tensor in world coordinates.
const Matrix3x3 RigidBody::getWorldInertiaTensorInverse(PhysicsWorld& world, Entity bodyEntity) {

    Matrix3x3 orientation = world.mTransformComponents.getTransform(bodyEntity).getOrientation().getMatrix();
    const Vector3& inverseInertiaLocalTensor = world.mRigidBodyComponents.getInertiaTensorLocalInverse(bodyEntity);
    Matrix3x3 orientationTranspose = orientation.getTranspose();
    orientationTranspose[0] *= inverseInertiaLocalTensor.x;
    orientationTranspose[1] *= inverseInertiaLocalTensor.y;
    orientationTranspose[2] *= inverseInertiaLocalTensor.z;
    return orientation * orientationTranspose;
}

// Set whether or not the body is allowed to go to sleep
/**
 * @param isAllowedToSleep True if the body is allowed to sleep
 */
void RigidBody::setIsAllowedToSleep(bool isAllowedToSleep) {

    mWorld.mRigidBodyComponents.setIsAllowedToSleep(mEntity, isAllowedToSleep);

    if (!isAllowedToSleep) setIsSleeping(false);

    RP3D_LOG(mWorld.mConfig.worldName, Logger::Level::Information, Logger::Category::Body,
             "Body " + std::to_string(mEntity.id) + ": Set isAllowedToSleep=" +
             (isAllowedToSleep ? "true" : "false"),  __FILE__, __LINE__);
}

// Return whether or not the body is allowed to sleep
/**
 * @return True if the body is allowed to sleep and false otherwise
 */
bool RigidBody::isAllowedToSleep() const {
    return mWorld.mRigidBodyComponents.getIsAllowedToSleep(mEntity);
}

// Return whether or not the body is sleeping
/**
 * @return True if the body is currently sleeping and false otherwise
 */
bool RigidBody::isSleeping() const {
    return mWorld.mRigidBodyComponents.getIsSleeping(mEntity);
}

// Set whether or not the body is active
/**
 * @param isActive True if you want to activate the body
 */
void RigidBody::setIsActive(bool isActive) {

    // If the state does not change
    if (mWorld.mCollisionBodyComponents.getIsActive(mEntity) == isActive) return;

    setIsSleeping(!isActive);

    CollisionBody::setIsActive(isActive);
}

#ifdef IS_RP3D_PROFILING_ENABLED


// Set the profiler
void RigidBody::setProfiler(Profiler* profiler) {

	CollisionBody::setProfiler(profiler);

    // Set the profiler for each collider
    const List<Entity>& colliderEntities = mWorld.mCollisionBodyComponents.getColliders(mEntity);
    for (uint i=0; i < colliderEntities.size(); i++) {

        Collider* collider = mWorld.mCollidersComponents.getCollider(colliderEntities[i]);

        collider->setProfiler(profiler);
	}
}

#endif
