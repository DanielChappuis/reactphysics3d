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

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor
RigidBody::RigidBody(const Transform& transform, decimal mass, CollisionShape *collisionShape,
                     bodyindex id)
          : CollisionBody(transform, collisionShape, id), mInitMass(mass), mIsGravityEnabled(true),
             mLinearDamping(decimal(0.0)), mAngularDamping(decimal(0.0)), mJointsList(NULL) {

    assert(collisionShape);

    // If the mass is not positive, set it to one
    if (mInitMass <= decimal(0.0)) {
        mInitMass = decimal(1.0);
    }

    // Compute the inertia tensor using the collision shape of the body
    mCollisionShape->computeLocalInertiaTensor(mInertiaTensorLocal, mInitMass);
    mInertiaTensorLocalInverse = mInertiaTensorLocal.getInverse();

    // Compute the inverse mass
    mMassInverse = decimal(1.0) / mass;
}

// Destructor
RigidBody::~RigidBody() {
    assert(mJointsList == NULL);
}

// Set the type of the body (static, kinematic or dynamic)
void RigidBody::setType(BodyType type) {

    if (mType == type) return;

    CollisionBody::setType(type);

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
        mInertiaTensorLocalInverse = Matrix3x3::zero();

    }
    else {  // If it is a dynamic body
        mMassInverse = decimal(1.0) / mInitMass;
        mInertiaTensorLocalInverse = mInertiaTensorLocal.getInverse();
    }

    // Awake the body
    setIsSleeping(false);
}

// Method that set the mass of the body
void RigidBody::setMass(decimal mass) {
    mInitMass = mass;

    // If the mass is negative, set it to one
    if (mInitMass <= decimal(0.0)) {
        mInitMass = decimal(1.0);
    }

    // Recompute the inverse mass
    if (mType == DYNAMIC) {
        mMassInverse = decimal(1.0) / mInitMass;
    }
    else {
        mMassInverse = decimal(0.0);
    }
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


