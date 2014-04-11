/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2010-2013 Daniel Chappuis                                       *
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
#include "../collision/shapes/CollisionShape.h"
#include "../engine/CollisionWorld.h"

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor
RigidBody::RigidBody(const Transform& transform, CollisionWorld& world, bodyindex id)
          : CollisionBody(transform, world, id), mInitMass(decimal(1.0)),
            mCenterOfMassLocal(0, 0, 0), mCenterOfMassWorld(transform.getPosition()),
            mIsGravityEnabled(true), mLinearDamping(decimal(0.0)), mAngularDamping(decimal(0.0)),
            mJointsList(NULL) {

    // Compute the inverse mass
    mMassInverse = decimal(1.0) / mInitMass;
}

// Destructor
RigidBody::~RigidBody() {
    assert(mJointsList == NULL);
}

// Set the type of the body (static, kinematic or dynamic)
void RigidBody::setType(BodyType type) {

    if (mType == type) return;

    CollisionBody::setType(type);

    // Recompute the total mass, center of mass and inertia tensor
    recomputeMassInformation();

    // If it is a static body
    if (mType == STATIC) {

        // Reset the velocity to zero
        mLinearVelocity.setToZero();
        mAngularVelocity.setToZero();
    }

    // If it is a static or a kinematic body
    if (mType == STATIC || mType == KINEMATIC) {

        // Reset the inverse mass and inverse inertia tensor to zero
        mMassInverse = decimal(0.0);
        mInertiaTensorLocal.setToZero();
        mInertiaTensorLocalInverse.setToZero();

    }
    else {  // If it is a dynamic body
        mMassInverse = decimal(1.0) / mInitMass;
        mInertiaTensorLocalInverse = mInertiaTensorLocal.getInverse();
    }

    // Awake the body
    setIsSleeping(false);
}

// Set the local inertia tensor of the body (in body coordinates)
void RigidBody::setInertiaTensorLocal(const Matrix3x3& inertiaTensorLocal) {
    mInertiaTensorLocal = inertiaTensorLocal;

    // Recompute the inverse local inertia tensor
    if (mType == DYNAMIC) {
        mInertiaTensorLocalInverse = mInertiaTensorLocal.getInverse();
    }
    else {
        mInertiaTensorLocalInverse = Matrix3x3::zero();
    }
}

// Remove a joint from the joints list
void RigidBody::removeJointFromJointsList(MemoryAllocator& memoryAllocator, const Joint* joint) {

    assert(joint != NULL);
    assert(mJointsList != NULL);

    // Remove the joint from the linked list of the joints of the first body
    if (mJointsList->joint == joint) {   // If the first element is the one to remove
        JointListElement* elementToRemove = mJointsList;
        mJointsList = elementToRemove->next;
        elementToRemove->JointListElement::~JointListElement();
        memoryAllocator.release(elementToRemove, sizeof(JointListElement));
    }
    else {  // If the element to remove is not the first one in the list
        JointListElement* currentElement = mJointsList;
        while (currentElement->next != NULL) {
            if (currentElement->next->joint == joint) {
                JointListElement* elementToRemove = currentElement->next;
                currentElement->next = elementToRemove->next;
                elementToRemove->JointListElement::~JointListElement();
                memoryAllocator.release(elementToRemove, sizeof(JointListElement));
                break;
            }
            currentElement = currentElement->next;
        }
    }
}

// Add a collision shape to the body.
/// This methods will create a copy of the collision shape you provided inside the world and
/// return a pointer to the actual collision shape in the world. You can use this pointer to
/// remove the collision from the body. Note that when the body is destroyed, all the collision
/// shapes will also be destroyed automatically. Because an internal copy of the collision shape
/// you provided is performed, you can delete it right after calling this method. The second
/// parameter is the transformation that transform the local-space of the collision shape into
/// the local-space of the body. By default, the second parameter is the identity transform.
/// The third parameter is the mass of the collision shape (this will used to compute the
/// total mass of the rigid body and its inertia tensor). The mass must be positive.
const CollisionShape* RigidBody::addCollisionShape(const CollisionShape& collisionShape,
                                                   decimal mass,
                                                   const Transform& transform
                                                   ) {

    assert(mass > decimal(0.0));

    // Create an internal copy of the collision shape into the world if it is not there yet
    CollisionShape* newCollisionShape = mWorld.createCollisionShape(collisionShape);

    // Create a new proxy collision shape to attach the collision shape to the body
    ProxyShape* proxyShape = newCollisionShape->createProxyShape(mWorld.mMemoryAllocator,
                                                                 this, transform, mass);

    // Add it to the list of proxy collision shapes of the body
    if (mProxyCollisionShapes == NULL) {
        mProxyCollisionShapes = proxyShape;
    }
    else {
        proxyShape->mNext = mProxyCollisionShapes;
        mProxyCollisionShapes = proxyShape;
    }

    // Notify the collision detection about this new collision shape
    mWorld.mCollisionDetection.addProxyCollisionShape(proxyShape);

    mNbCollisionShapes++;

    // Recompute the center of mass, total mass and inertia tensor of the body with the new
    // collision shape
    recomputeMassInformation();

    // Return a pointer to the collision shape
    return newCollisionShape;
}

// Remove a collision shape from the body
void RigidBody::removeCollisionShape(const CollisionShape* collisionShape) {

    // Remove the collision shape
    CollisionBody::removeCollisionShape(collisionShape);

    // Recompute the total mass, center of mass and inertia tensor
    recomputeMassInformation();
}

// Recompute the center of mass, total mass and inertia tensor of the body using all
// the collision shapes attached to the body.
void RigidBody::recomputeMassInformation() {

    mInitMass = decimal(0.0);
    mMassInverse = decimal(0.0);
    mInertiaTensorLocal.setToZero();
    mInertiaTensorLocalInverse.setToZero();
    mCenterOfMassLocal.setToZero();

    // If it is STATIC or KINEMATIC body
    if (mType == STATIC || mType == KINEMATIC) {
        mCenterOfMassWorld = mTransform.getPosition();
        return;
    }

    assert(mType == DYNAMIC);

    // Compute the total mass of the body
    for (ProxyShape* shape = mProxyCollisionShapes; shape != NULL; shape = shape->mNext) {
        mInitMass += shape->getMass();
        mCenterOfMassLocal += shape->getLocalToBodyTransform().getPosition() * shape->getMass();
    }

    if (mInitMass > decimal(0.0)) {
        mMassInverse = decimal(1.0) / mInitMass;
    }
    else {
        mInitMass = decimal(1.0);
        mMassInverse = decimal(1.0);
    }

    // Compute the center of mass
    const Vector3 oldCenterOfMass = mCenterOfMassWorld;
    mCenterOfMassLocal *= mMassInverse;
    mCenterOfMassWorld = mTransform * mCenterOfMassLocal;

    // Compute the total mass and inertia tensor using all the collision shapes
    for (ProxyShape* shape = mProxyCollisionShapes; shape != NULL; shape = shape->mNext) {

        // Get the inertia tensor of the collision shape in its local-space
        Matrix3x3 inertiaTensor;
        shape->getCollisionShape()->computeLocalInertiaTensor(inertiaTensor, shape->getMass());

        // Convert the collision shape inertia tensor into the local-space of the body
        const Transform& shapeTransform = shape->getLocalToBodyTransform();
        Matrix3x3 rotationMatrix = shapeTransform.getOrientation().getMatrix();
        inertiaTensor = rotationMatrix * inertiaTensor * rotationMatrix.getTranspose();

        // Use the parallel axis theorem to convert the inertia tensor w.r.t the collision shape
        // center into a inertia tensor w.r.t to the body origin.
        Vector3 offset = shapeTransform.getPosition() - mCenterOfMassLocal;
        decimal offsetSquare = offset.lengthSquare();
        Matrix3x3 offsetMatrix;
        offsetMatrix[0].setAllValues(offsetSquare, decimal(0.0), decimal(0.0));
        offsetMatrix[1].setAllValues(decimal(0.0), offsetSquare, decimal(0.0));
        offsetMatrix[2].setAllValues(decimal(0.0), decimal(0.0), offsetSquare);
        offsetMatrix[0] += offset * (-offset.x);
        offsetMatrix[1] += offset * (-offset.y);
        offsetMatrix[2] += offset * (-offset.z);
        offsetMatrix *= shape->getMass();

        mInertiaTensorLocal += inertiaTensor + offsetMatrix;
    }

    // Compute the local inverse inertia tensor
    mInertiaTensorLocalInverse = mInertiaTensorLocal.getInverse();

    // Update the linear velocity of the center of mass
    mLinearVelocity += mAngularVelocity.cross(mCenterOfMassWorld - oldCenterOfMass);
}

