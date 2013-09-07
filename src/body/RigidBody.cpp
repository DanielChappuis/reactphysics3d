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
RigidBody::RigidBody(const Transform& transform, decimal mass, const Matrix3x3& inertiaTensorLocal,
                      CollisionShape *collisionShape, bodyindex id)
           : CollisionBody(transform, collisionShape, id), mInertiaTensorLocal(inertiaTensorLocal),
             mMass(mass), mInertiaTensorLocalInverse(inertiaTensorLocal.getInverse()),
             mMassInverse(decimal(1.0) / mass), mIsGravityEnabled(true),
             mLinearDamping(decimal(0.0)), mAngularDamping(decimal(0.0)), mJointsList(NULL) {

    assert(collisionShape);
}

// Destructor
RigidBody::~RigidBody() {
    assert(mJointsList == NULL);
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


