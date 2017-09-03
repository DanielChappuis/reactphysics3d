/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2016 Daniel Chappuis                                       *
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
#include <iostream>
#include "ContactManifold.h"

using namespace reactphysics3d;

// Constructor
ContactManifold::ContactManifold(const ContactManifoldInfo* manifoldInfo, ProxyShape* shape1, ProxyShape* shape2, Allocator& memoryAllocator)
                : mShape1(shape1), mShape2(shape2), mContactPoints(nullptr), mContactNormalId(manifoldInfo->getContactNormalId()),
                  mNbContactPoints(0), mFrictionImpulse1(0.0), mFrictionImpulse2(0.0),
                  mFrictionTwistImpulse(0.0), mIsAlreadyInIsland(false),
                  mMemoryAllocator(memoryAllocator), mNext(nullptr), mPrevious(nullptr), mIsObselete(false) {
    
    // For each contact point info in the manifold
    const ContactPointInfo* pointInfo = manifoldInfo->getFirstContactPointInfo();
    while(pointInfo != nullptr) {

        // Create the new contact point
        ContactPoint* contact = new (mMemoryAllocator.allocate(sizeof(ContactPoint)))
                ContactPoint(pointInfo, mShape1->getLocalToWorldTransform(), mShape2->getLocalToWorldTransform());

        // Add the new contact point into the manifold
        contact->setNext(mContactPoints);
        mContactPoints = contact;
        mNbContactPoints++;

        pointInfo = pointInfo->next;
    }

    assert(mNbContactPoints <= MAX_CONTACT_POINTS_IN_MANIFOLD);
    assert(mNbContactPoints > 0);
}

// Destructor
ContactManifold::~ContactManifold() {

    // Delete all the contact points
    ContactPoint* contactPoint = mContactPoints;
    while(contactPoint != nullptr) {

        ContactPoint* nextContactPoint = contactPoint->getNext();

        // Delete the contact point
        contactPoint->~ContactPoint();
        mMemoryAllocator.release(contactPoint, sizeof(ContactPoint));

        contactPoint = nextContactPoint;
    }
}

// Add a contact point
void ContactManifold::addContactPoint(const ContactPointInfo* contactPointInfo) {

    // Create the new contact point
    ContactPoint* contactPoint = new (mMemoryAllocator.allocate(sizeof(ContactPoint)))
            ContactPoint(contactPointInfo, mShape1->getLocalToWorldTransform(), mShape2->getLocalToWorldTransform());

    // Add the new contact point into the manifold
    contactPoint->setNext(mContactPoints);
    mContactPoints = contactPoint;

    mNbContactPoints++;
}

// Clear the obselete contact points
void ContactManifold::clearObseleteContactPoints() {

    ContactPoint* contactPoint = mContactPoints;
    ContactPoint* previousContactPoint = nullptr;
    while (contactPoint != nullptr) {

        ContactPoint* nextContactPoint =  contactPoint->getNext();

        if (contactPoint->getIsObselete()) {

            // Delete the contact point
            contactPoint->~ContactPoint();
            mMemoryAllocator.release(contactPoint, sizeof(ContactPoint));

            if (previousContactPoint != nullptr) {
                previousContactPoint->setNext(nextContactPoint);
            }
            else {
                mContactPoints = nextContactPoint;
            }

            mNbContactPoints--;
        }
        else {
            previousContactPoint = contactPoint;
        }

        contactPoint = nextContactPoint;
    }

    assert(mNbContactPoints >= 0);
    assert(mNbContactPoints <= MAX_CONTACT_POINTS_IN_MANIFOLD);
}
