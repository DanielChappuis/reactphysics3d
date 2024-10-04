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
#include <reactphysics3d/body/RigidBody.h>
#include <reactphysics3d/engine/PhysicsCommon.h>
#include <reactphysics3d/collision/shapes/CollisionShape.h>
#include <reactphysics3d/engine/PhysicsWorld.h>
#include <reactphysics3d/utils/Profiler.h>

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor
/**
* @param world The world where the body has been added
* @param entity The entity of the rigidbody
*/
RigidBody::RigidBody(PhysicsWorld& world, Entity entity) : Body(world, entity) {

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

        const Transform& transform = getTransform();
        mWorld.mRigidBodyComponents.setConstrainedPosition(mEntity, transform.getPosition());
        mWorld.mRigidBodyComponents.setConstrainedOrientation(mEntity, transform.getOrientation());
    }

    // If it is a static or a kinematic body
    if (type == BodyType::STATIC || type == BodyType::KINEMATIC) {

        // Reset the inverse mass and inverse inertia tensor to zero
        mWorld.mRigidBodyComponents.setMassInverse(mEntity, decimal(0));
        mWorld.mRigidBodyComponents.setInverseInertiaTensorLocal(mEntity, Vector3::zero());
        mWorld.mRigidBodyComponents.setInertiaTensorWorldInverse(mEntity, Matrix3x3::zero());
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

    // Disable/Enable the body if necessary (components of static bodies are disabled)
    mWorld.setBodyDisabled(mEntity, type == BodyType::STATIC);

    // Awake the body
    setIsSleeping(false);

    if (type == BodyType::STATIC) {

        // Disable overlapping pairs if both bodies are disabled (sleeping or static)
        checkForDisabledOverlappingPairs();
    }

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

// Manually apply an external force (in local-space) to the body at a given point (in local-space).
/// If the point is not at the center of mass of the body, it will also
/// generate some torque and therefore, change the angular velocity of the body.
/// If the body is sleeping, calling this method will wake it up. Note that the
/// force will we added to the sum of the applied forces and that this sum will be
/// reset to zero at the end of each call of the PhyscisWorld::update() method.
/// You can only apply a force to a dynamic body otherwise, this method will do nothing.
/**
 * @param force The force (in local-space of the body) to apply on the body (in Newtons)
 * @param point The point where the force is applied (in local-space of the body)
 */
void RigidBody::applyLocalForceAtLocalPosition(const Vector3& force, const Vector3& point) {

    // Convert the local-space force to world-space
    const Vector3 worldForce = mWorld.mTransformComponents.getTransform(mEntity).getOrientation() * force;

    applyWorldForceAtLocalPosition(worldForce, point);
}

// Manually apply an external force (in world-space) to the body at a given point (in local-space).
/// If the point is not at the center of mass of the body, it will also
/// generate some torque and therefore, change the angular velocity of the body.
/// If the body is sleeping, calling this method will wake it up. Note that the
/// force will we added to the sum of the applied forces and that this sum will be
/// reset to zero at the end of each call of the PhyscisWorld::update() method.
/// You can only apply a force to a dynamic body otherwise, this method will do nothing.
/**
 * @param force The force (in world-space) to apply on the body (in Newtons)
 * @param point The point where the force is applied (in local-space)
 */
void RigidBody::applyWorldForceAtLocalPosition(const Vector3& force, const Vector3& point) {

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

// Manually apply an external force (in local-space) to the body at a given point (in world-space).
/// If the point is not at the center of mass of the body, it will also
/// generate some torque and therefore, change the angular velocity of the body.
/// If the body is sleeping, calling this method will wake it up. Note that the
/// force will we added to the sum of the applied forces and that this sum will be
/// reset to zero at the end of each call of the PhyscisWorld::update() method.
/// You can only apply a force to a dynamic body otherwise, this method will do nothing.
/**
 * @param force The force (in local-space of the body) to apply on the body (in Newtons)
 * @param point The point where the force is applied (in world-space)
 */
void RigidBody::applyLocalForceAtWorldPosition(const Vector3& force, const Vector3& point) {

    // Convert the local-space force to world-space
    const Vector3 worldForce = mWorld.mTransformComponents.getTransform(mEntity).getOrientation() * force;

    applyWorldForceAtWorldPosition(worldForce, point);
}

// Manually apply an external force (in world-space) to the body at a given point (in world-space).
/// If the point is not at the center of mass of the body, it will also
/// generate some torque and therefore, change the angular velocity of the body.
/// If the body is sleeping, calling this method will wake it up. Note that the
/// force will we added to the sum of the applied forces and that this sum will be
/// reset to zero at the end of each call of the PhyscisWorld::update() method.
/// You can only apply a force to a dynamic body otherwise, this method will do nothing.
/**
 * @param force The force (in world-space) to apply on the body (in Newtons)
 * @param point The point where the force is applied (in world-space)
 */
void RigidBody::applyWorldForceAtWorldPosition(const Vector3& force, const Vector3& point) {

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

    // If it is a dynamic body
    const BodyType type = mWorld.mRigidBodyComponents.getBodyType(mEntity);
    if (type == BodyType::DYNAMIC) {

        // Compute the inverse local inertia tensor
        Vector3 inverseInertiaTensorLocal(inertiaTensorLocal.x != decimal(0.0) ? decimal(1.0) / inertiaTensorLocal.x : 0,
                                          inertiaTensorLocal.y != decimal(0.0) ? decimal(1.0) / inertiaTensorLocal.y : 0,
                                          inertiaTensorLocal.z != decimal(0.0) ? decimal(1.0) / inertiaTensorLocal.z : 0);
        mWorld.mRigidBodyComponents.setInverseInertiaTensorLocal(mEntity, inverseInertiaTensorLocal);
    }

    RP3D_LOG(mWorld.mConfig.worldName, Logger::Level::Information, Logger::Category::Body,
             "Body " + std::to_string(mEntity.id) + ": Set inertiaTensorLocal=" + inertiaTensorLocal.to_string(),  __FILE__, __LINE__);
}

// Manually apply an external force (in local-space) to the body at its center of mass.
/// If the body is sleeping, calling this method will wake it up. Note that the
/// force will we added to the sum of the applied forces and that this sum will be
/// reset to zero at the end of each call of the PhyscisWorld::update() method.
/// You can only apply a force to a dynamic body otherwise, this method will do nothing.
/**
 * @param force The external force (in local-space of the body) to apply on the center of mass of the body (in Newtons)
 */
void RigidBody::applyLocalForceAtCenterOfMass(const Vector3& force) {

    // Convert the local-space force to world-space
    const Vector3 worldForce = mWorld.mTransformComponents.getTransform(mEntity).getOrientation() * force;

    applyWorldForceAtCenterOfMass(worldForce);
}

// Manually apply an external force (in world-space) to the body at its center of mass.
/// If the body is sleeping, calling this method will wake it up. Note that the
/// force will we added to the sum of the applied forces and that this sum will be
/// reset to zero at the end of each call of the PhyscisWorld::update() method.
/// You can only apply a force to a dynamic body otherwise, this method will do nothing.
/**
 * @param force The external force (in world-space) to apply on the center of mass of the body (in Newtons)
 */
void RigidBody::applyWorldForceAtCenterOfMass(const Vector3& force) {

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
 * @return The linear damping factor of this body (in range [0; +inf]). Zero means no damping.
 */
decimal RigidBody::getLinearDamping() const {
    return mWorld.mRigidBodyComponents.getLinearDamping(mEntity);
}

// Return the angular velocity damping factor
/**
 * @return The angular damping factor of this body (in range [0; +inf]). Zero means no damping.
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
    Vector3 linearVelocity = mWorld.mRigidBodyComponents.getLinearVelocity(mEntity);
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
    Vector3 linearVelocity = mWorld.mRigidBodyComponents.getLinearVelocity(mEntity);
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
    const Array<Entity>& colliderEntities = mWorld.mBodyComponents.getColliders(mEntity);
    for (uint32 i=0; i < colliderEntities.size(); i++) {

        const uint32 colliderIndex = mWorld.mCollidersComponents.getEntityIndex(colliderEntities[i]);

        const decimal colliderVolume = mWorld.mCollidersComponents.mCollisionShapes[colliderIndex]->getVolume();
        const decimal colliderMassDensity = mWorld.mCollidersComponents.mMaterials[colliderIndex].getMassDensity();

        const decimal colliderMass = colliderVolume * colliderMassDensity;

        totalMass += colliderMass;
        centerOfMassLocal += colliderMass * mWorld.mCollidersComponents.mLocalToBodyTransforms[colliderIndex].getPosition();
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
    const Array<Entity>& colliderEntities = mWorld.mBodyComponents.getColliders(mEntity);
    for (uint32 i=0; i < colliderEntities.size(); i++) {

        const uint32 colliderIndex = mWorld.mCollidersComponents.getEntityIndex(colliderEntities[i]);

        const decimal colliderVolume = mWorld.mCollidersComponents.mCollisionShapes[colliderIndex]->getVolume();
        const decimal colliderMassDensity = mWorld.mCollidersComponents.mMaterials[colliderIndex].getMassDensity();
        const decimal colliderMass = colliderVolume * colliderMassDensity;

        totalMass += colliderMass;

        // Get the inertia tensor of the collider in its local-space
        Vector3 shapeLocalInertiaTensor = mWorld.mCollidersComponents.mCollisionShapes[colliderIndex]->getLocalInertiaTensor(colliderMass);

        // Convert the collider inertia tensor into the local-space of the body
        const Transform& shapeTransform = mWorld.mCollidersComponents.mLocalToBodyTransforms[colliderIndex];
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

    // If it is a dynamic body
    const BodyType type = mWorld.mRigidBodyComponents.getBodyType(mEntity);
    if (type == BodyType::DYNAMIC) {

        // Compute the inverse local inertia tensor
        Vector3 inverseInertiaTensorLocal(inertiaTensorLocal.x != decimal(0.0) ? decimal(1.0) / inertiaTensorLocal.x : 0,
                                          inertiaTensorLocal.y != decimal(0.0) ? decimal(1.0) / inertiaTensorLocal.y : 0,
                                          inertiaTensorLocal.z != decimal(0.0) ? decimal(1.0) / inertiaTensorLocal.z : 0);
        mWorld.mRigidBodyComponents.setInverseInertiaTensorLocal(mEntity, inverseInertiaTensorLocal);
    }

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
    const Array<Entity>& colliderEntities = mWorld.mBodyComponents.getColliders(mEntity);
    for (uint32 i=0; i < colliderEntities.size(); i++) {

        const uint32 colliderIndex = mWorld.mCollidersComponents.getEntityIndex(colliderEntities[i]);

        const decimal colliderVolume = mWorld.mCollidersComponents.mCollisionShapes[colliderIndex]->getVolume();
        const decimal colliderMassDensity = mWorld.mCollidersComponents.mMaterials[colliderIndex].getMassDensity();

        const decimal colliderMass = colliderVolume * colliderMassDensity;

        totalMass += colliderMass;
    }

    // Set the mass
    mWorld.mRigidBodyComponents.setMass(mEntity, totalMass);

    // If it is a dynamic body
    const BodyType type = mWorld.mRigidBodyComponents.getBodyType(mEntity);
    if (type == BodyType::DYNAMIC) {

        // Compute the inverse mass
        if (totalMass > decimal(0.0)) {
            mWorld.mRigidBodyComponents.setMassInverse(mEntity, decimal(1.0) / totalMass);
        }
        else {
            mWorld.mRigidBodyComponents.setMassInverse(mEntity, decimal(0.0));
        }
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

    // If it is a dynamic body
    const BodyType type = mWorld.mRigidBodyComponents.getBodyType(mEntity);
    if (type == BodyType::DYNAMIC) {

        // Update the linear velocity of the center of mass
        Vector3 linearVelocity = mWorld.mRigidBodyComponents.getLinearVelocity(mEntity);
        const Vector3& angularVelocity = mWorld.mRigidBodyComponents.getAngularVelocity(mEntity);
        linearVelocity += angularVelocity.cross(centerOfMassWorld - oldCenterOfMassWorld);
        mWorld.mRigidBodyComponents.setLinearVelocity(mEntity, linearVelocity);
    }

    RP3D_LOG(mWorld.mConfig.worldName, Logger::Level::Information, Logger::Category::Body,
             "Body " + std::to_string(mEntity.id) + ": Set centerOfMassLocal=" + centerOfMassLocal.to_string(),  __FILE__, __LINE__);

    // Compute the mass and local-space inertia tensor
    Vector3 inertiaTensorLocal;
    decimal totalMass;
    computeMassAndInertiaTensorLocal(inertiaTensorLocal, totalMass);

    mWorld.mRigidBodyComponents.setLocalInertiaTensor(mEntity, inertiaTensorLocal);

    // If it is a dynamic body
    if (type == BodyType::DYNAMIC) {

        // Compute the inverse local inertia tensor
        Vector3 inverseInertiaTensorLocal(inertiaTensorLocal.x != decimal(0.0) ? decimal(1.0) / inertiaTensorLocal.x : 0,
                                          inertiaTensorLocal.y != decimal(0.0) ? decimal(1.0) / inertiaTensorLocal.y : 0,
                                          inertiaTensorLocal.z != decimal(0.0) ? decimal(1.0) / inertiaTensorLocal.z : 0);
        mWorld.mRigidBodyComponents.setInverseInertiaTensorLocal(mEntity, inverseInertiaTensorLocal);
    }

    RP3D_LOG(mWorld.mConfig.worldName, Logger::Level::Information, Logger::Category::Body,
             "Body " + std::to_string(mEntity.id) + ": Set inertiaTensorLocal=" + inertiaTensorLocal.to_string(),  __FILE__, __LINE__);

    // Set the mass
    mWorld.mRigidBodyComponents.setMass(mEntity, totalMass);

    // If it is a dynamic body
    if (type == BodyType::DYNAMIC) {

        // Compute the inverse mass
        if (totalMass > decimal(0.0)) {
            mWorld.mRigidBodyComponents.setMassInverse(mEntity, decimal(1.0) / totalMass);
        }
        else {
            mWorld.mRigidBodyComponents.setMassInverse(mEntity, decimal(0.0));
        }
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

    if (mass < decimal(0.0)) {

        RP3D_LOG(mWorld.mConfig.worldName, Logger::Level::Error, Logger::Category::Body,
                 "Error when setting mass of body " + std::to_string(mEntity.id) + ": mass cannot be negative",  __FILE__, __LINE__);

        return;
    }

    mWorld.mRigidBodyComponents.setMass(mEntity, mass);

    // If it is a dynamic body
    const BodyType type = mWorld.mRigidBodyComponents.getBodyType(mEntity);
    if (type == BodyType::DYNAMIC) {

        if (mass > decimal(0.0)) {
            mWorld.mRigidBodyComponents.setMassInverse(mEntity, decimal(1.0) / mass);
        }
        else {
            mWorld.mRigidBodyComponents.setMassInverse(mEntity, decimal(0.0));

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
    AABB shapeAABB = collisionShape->getLocalBounds();
    const Transform localToWorldTransform = mWorld.mTransformComponents.getTransform(mEntity) * transform;
    Material material(mWorld.mConfig.defaultFrictionCoefficient, mWorld.mConfig.defaultBounciness);
    ColliderComponents::ColliderComponent colliderComponent(mEntity, collider, shapeAABB,
                                                            transform, collisionShape, 0x0001, 0xFFFF, localToWorldTransform, material);
    bool isDisabled = mWorld.mRigidBodyComponents.getIsEntityDisabled(mEntity);
    mWorld.mCollidersComponents.addComponent(colliderEntity, isDisabled, colliderComponent);

    mWorld.mBodyComponents.addColliderToBody(mEntity, colliderEntity);

    // Assign the collider with the collision shape
    collisionShape->addCollider(collider);

#ifdef IS_RP3D_PROFILING_ENABLED


	// Set the profiler
    collider->setProfiler(mProfiler);

#endif

    // If the body is active
    const bool isActive = mWorld.mBodyComponents.getIsActive(mEntity);
    if (isActive) {

        // Compute the world-space AABB of the new collision shape
        AABB aabb = collisionShape->computeTransformedAABB(mWorld.mTransformComponents.getTransform(mEntity) * transform);

        // Notify the collision detection about this new collision shape
        mWorld.mCollisionDetection.addCollider(collider, aabb);
    }

    RP3D_LOG(mWorld.mConfig.worldName, Logger::Level::Information, Logger::Category::Body,
             "Body " + std::to_string(mEntity.id) + ": Collider " + std::to_string(collider->getBroadPhaseId()) + " added to body",  __FILE__, __LINE__);

    RP3D_LOG(mWorld.mConfig.worldName, Logger::Level::Information, Logger::Category::Collider,
             "Collider " + std::to_string(collider->getBroadPhaseId()) + ":  collisionShape=" +
             collider->getCollisionShape()->to_string(),  __FILE__, __LINE__);

    mWorld.mBodyComponents.setHasSimulationCollider(mEntity, true);

    // Return a pointer to the collider
    return collider;
}

// Remove a collider from the body
/// To remove a collider, you need to specify its pointer.
/**
 * @param collider The pointer of the collider you want to remove
 */
void RigidBody::removeCollider(Collider* collider) {

    // Awake all the sleeping neighbor bodies of the current one
    awakeNeighborDisabledBodies();

    // Remove the collision shape
    Body::removeCollider(collider);
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

// Set the linear damping factor.
/**
 * @param linearDamping The linear damping factor of this body (in range [0; +inf]). Zero means no damping.
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

// Set the angular damping factor.
/**
 * @param angularDamping The angular damping factor of this body (in range [0; +inf]). Zero means no damping.
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

    if (getType() == BodyType::STATIC) {

        mWorld.mRigidBodyComponents.setConstrainedPosition(mEntity, transform.getPosition());
        mWorld.mRigidBodyComponents.setConstrainedOrientation(mEntity, transform.getOrientation());
    }

    // Awake the body if it is sleeping
    setIsSleeping(false);

    Body::setTransform(transform);
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

// Return the linear lock axis factor
/// The linear lock axis factor specify whether linear motion along world-space axes X,Y,Z is
/// restricted or not.
/**
 * @return A Vector3 with the linear lock axis factor for each X,Y,Z world-space axis
 */
const Vector3& RigidBody::getLinearLockAxisFactor() const {
    return mWorld.mRigidBodyComponents.getLinearLockAxisFactor(mEntity);
}

// Set the linear lock axis factor
/// This method allows to restrict the linear motion of a rigid body along the world-space
/// axes X,Y and Z. For instance, it's possible to disable the linear motion of a body
/// along a given axis by setting a lock axis factor of zero.
/**
 * @param linearLockAxisFactor A Vector3 with the lock factor for each world-space axis X,Y,Z
 */
void RigidBody::setLinearLockAxisFactor(const Vector3& linearLockAxisFactor) const {
    mWorld.mRigidBodyComponents.setLinearLockAxisFactor(mEntity, linearLockAxisFactor);
}

// Return the angular lock axis factor
/// The angular lock axis factor specify whether angular motion around world-space axes X,Y,Z is
/// restricted or not.
/**
 * @return A Vector3 with the angular lock axis factor for each X,Y,Z world-space axis
 */
const Vector3& RigidBody::getAngularLockAxisFactor() const {
    return mWorld.mRigidBodyComponents.getAngularLockAxisFactor(mEntity);
}

// Set the angular lock axis factor
/// This method allows to restrict the angular motion of a rigid body around the world-space
/// axes X,Y and Z. For instance, it's possible to disable the angular motion of a body
/// around a given axis by setting a lock axis factor of zero.
/**
 * @param angularLockAxisFactor A Vector3 with the lock factor for each world-space axis X,Y,Z
 */
void RigidBody::setAngularLockAxisFactor(const Vector3& angularLockAxisFactor) const {
    mWorld.mRigidBodyComponents.setAngularLockAxisFactor(mEntity, angularLockAxisFactor);
}

// Manually apply an external torque to the body (in world-space).
/// If the body is sleeping, calling this method will wake it up. Note that the
/// force will we added to the sum of the applied torques and that this sum will be
/// reset to zero at the end of each call of the PhyscisWorld::update() method.
/// You can only apply a force to a dynamic body otherwise, this method will do nothing.
/**
 * @param torque The external torque to apply on the body (in world-space)
 */
void RigidBody::applyWorldTorque(const Vector3& torque) {

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

// Manually apply an external torque to the body (in local-space).
/// If the body is sleeping, calling this method will wake it up. Note that the
/// force will we added to the sum of the applied torques and that this sum will be
/// reset to zero at the end of each call of the PhyscisWorld::update() method.
/// You can only apply a force to a dynamic body otherwise, this method will do nothing.
/**
 * @param torque The external torque to apply on the body (in local-space)
 */
void RigidBody::applyLocalTorque(const Vector3& torque) {

    // Convert the local-space torque to world-space
    const Vector3 worldTorque = mWorld.mTransformComponents.getTransform(mEntity).getOrientation() * torque;

    applyWorldTorque(worldTorque);
}

// Reset the accumulated force to zero
void RigidBody::resetForce() {

    // If it is not a dynamic body, we do nothing
    if (mWorld.mRigidBodyComponents.getBodyType(mEntity) != BodyType::DYNAMIC) return;

    // Set the external force to zero
    mWorld.mRigidBodyComponents.setExternalForce(mEntity, Vector3(0, 0, 0));
}

// Reset the accumulated torque to zero
void RigidBody::resetTorque() {

    // If it is not a dynamic body, we do nothing
    if (mWorld.mRigidBodyComponents.getBodyType(mEntity) != BodyType::DYNAMIC) return;

    // Set the external torque to zero
    mWorld.mRigidBodyComponents.setExternalTorque(mEntity, Vector3(0, 0, 0));
}

// Return the total manually applied force on the body (in world-space)
/**
 * @return The total manually applied force on the body (in world-space)
 */
const Vector3& RigidBody::getForce() const {
    return mWorld.mRigidBodyComponents.getExternalForce(mEntity);
}

// Return the total manually applied torque on the body (in world-space)
/**
 * @return The total manually applied torque on the body (in world-space)
 */
const Vector3& RigidBody::getTorque() const {
    return mWorld.mRigidBodyComponents.getExternalTorque(mEntity);
}

// Set the variable to know whether or not the body is sleeping
void RigidBody::setIsSleeping(bool isSleeping) {

    bool isBodySleeping = mWorld.mRigidBodyComponents.getIsSleeping(mEntity);

    if (isBodySleeping == isSleeping) return;

    // A static body is always awake
    if (mWorld.mRigidBodyComponents.getBodyType(mEntity) == BodyType::STATIC) return;

    // If the body is not active, do nothing (it is sleeping)
    if (!mWorld.mBodyComponents.getIsActive(mEntity)) {
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

    if (isSleeping) {

        // Disable overlapping pairs if both bodies are disabled (sleeping or static)
        checkForDisabledOverlappingPairs();

        mWorld.mRigidBodyComponents.setLinearVelocity(mEntity, Vector3::zero());
        mWorld.mRigidBodyComponents.setAngularVelocity(mEntity, Vector3::zero());
        mWorld.mRigidBodyComponents.setExternalForce(mEntity, Vector3::zero());
        mWorld.mRigidBodyComponents.setExternalTorque(mEntity, Vector3::zero());
    }
    else {

        // We need to enable the currently disabled overlapping pairs
        enableOverlappingPairs();

        // Make sure the broad-phase with recompute the overlapping pairs with this body
        askForBroadPhaseCollisionCheck();
    }

    RP3D_LOG(mWorld.mConfig.worldName, Logger::Level::Information, Logger::Category::Body,
         "Body " + std::to_string(mEntity.id) + ": Set isSleeping=" +
         (isSleeping ? "true" : "false"),  __FILE__, __LINE__);
}

// Enable the currently disabled overlapping pairs
void RigidBody::enableOverlappingPairs() {

    // For each collider of the body
    const Array<Entity>& colliderEntities = mWorld.mBodyComponents.getColliders(mEntity);
    for (uint32 i=0; i < colliderEntities.size(); i++) {

        // Get the currently overlapping pairs for this collider
        Array<uint64> overlappingPairs = mWorld.mCollidersComponents.getOverlappingPairs(colliderEntities[i]);

        // We enable all the overlapping pairs (there should be only disabled overlapping pairs at this point)
        const uint64 nbOverlappingPairs = overlappingPairs.size();
        for (uint64 j=0; j < nbOverlappingPairs; j++) {

            OverlappingPairs::OverlappingPair* pair = mWorld.mCollisionDetection.mOverlappingPairs.getOverlappingPair(overlappingPairs[j]);

            if (!pair->isEnabled) {

                mWorld.mCollisionDetection.mOverlappingPairs.enablePair(overlappingPairs[j]);
            }
        }
    }
}

// Awake the disabled neighbor bodies
void RigidBody::awakeNeighborDisabledBodies() {

    // For each collider of the body
    const Array<Entity>& colliderEntities = mWorld.mBodyComponents.getColliders(mEntity);
    for (uint32 i=0; i < colliderEntities.size(); i++) {

        // Get the currently overlapping pairs for this collider
        Array<uint64> overlappingPairs = mWorld.mCollidersComponents.getOverlappingPairs(colliderEntities[i]);

        const uint64 nbOverlappingPairs = overlappingPairs.size();
        for (uint64 j=0; j < nbOverlappingPairs; j++) {

            OverlappingPairs::OverlappingPair* pair = mWorld.mCollisionDetection.mOverlappingPairs.getOverlappingPair(overlappingPairs[j]);

            // If both collider where colliding in the previous frame
            if (pair->collidingInPreviousFrame) {

                const Entity body1Entity = mWorld.mCollidersComponents.getBody(pair->collider1);
                const Entity body2Entity = mWorld.mCollidersComponents.getBody(pair->collider2);

                const bool isCurrentBody1 = mEntity == body1Entity;

                const bool isNeighborDisabled = mWorld.mRigidBodyComponents.getIsEntityDisabled(isCurrentBody1 ? body2Entity : body1Entity);

                // If both bodies of the pair are disabled we awake the neighbor body
                if (isNeighborDisabled) {

                    // Awake the neighbor colliding body
                    RigidBody* neighborBody = mWorld.mRigidBodyComponents.getRigidBody(isCurrentBody1 ? body2Entity : body1Entity);
                    neighborBody->setIsSleeping(false);
                }
            }
        }
    }
}

// Disable the overlapping pairs if both bodies are disabled (sleeping or static)
void RigidBody::checkForDisabledOverlappingPairs() {

    // For each collider of the body
    const Array<Entity>& colliderEntities = mWorld.mBodyComponents.getColliders(mEntity);
    for (uint32 i=0; i < colliderEntities.size(); i++) {

        // Get the currently overlapping pairs for this collider
        Array<uint64> overlappingPairs = mWorld.mCollidersComponents.getOverlappingPairs(colliderEntities[i]);

        const uint64 nbOverlappingPairs = overlappingPairs.size();
        for (uint64 j=0; j < nbOverlappingPairs; j++) {

            OverlappingPairs::OverlappingPair* pair = mWorld.mCollisionDetection.mOverlappingPairs.getOverlappingPair(overlappingPairs[j]);

            const Entity body1Entity = mWorld.mCollidersComponents.getBody(pair->collider1);
            const Entity body2Entity = mWorld.mCollidersComponents.getBody(pair->collider2);

            const bool isBody1Disabled = mWorld.mRigidBodyComponents.getIsEntityDisabled(body1Entity);
            const bool isBody2Disabled = mWorld.mRigidBodyComponents.getIsEntityDisabled(body2Entity);

            // If both bodies of the pair are disabled, we disable the overlapping pair
            if (isBody1Disabled && isBody2Disabled) {

                mWorld.mCollisionDetection.disableOverlappingPair(overlappingPairs[j]);
            }
        }
    }
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
    if (mWorld.mBodyComponents.getIsActive(mEntity) == isActive) return;

    setIsSleeping(!isActive);

    Body::setIsActive(isActive);
}
