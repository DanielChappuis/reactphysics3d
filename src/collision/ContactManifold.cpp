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
ContactManifold::ContactManifold(const ContactManifoldInfo& manifoldInfo, ProxyShape* shape1, ProxyShape* shape2,
                                 PoolAllocator& memoryAllocator, short normalDirectionId)
                : mShape1(shape1), mShape2(shape2), mNormalDirectionId(normalDirectionId),
                  mNbContactPoints(0), mFrictionImpulse1(0.0), mFrictionImpulse2(0.0),
                  mFrictionTwistImpulse(0.0), mIsAlreadyInIsland(false),
                  mMemoryAllocator(memoryAllocator) {
    
    // For each contact point info in the manifold
    const ContactPointInfo* pointInfo = manifoldInfo.getFirstContactPointInfo();
    while(pointInfo != nullptr) {

        // Create the new contact point
        ContactPoint* contact = new (mMemoryAllocator.allocate(sizeof(ContactPoint)))
                ContactPoint(pointInfo, mShape1->getLocalToWorldTransform(), mShape2->getLocalToWorldTransform());

        // Add the new contact point into the manifold
        mContactPoints[mNbContactPoints] = contact;
        mNbContactPoints++;

        pointInfo = pointInfo->next;
    }

    assert(mNbContactPoints <= MAX_CONTACT_POINTS_IN_MANIFOLD);
    assert(mNbContactPoints > 0);
}

// Destructor
ContactManifold::~ContactManifold() {
    clear();
}

// Clear the contact manifold
void ContactManifold::clear() {
    for (uint i=0; i<mNbContactPoints; i++) {
		
        // Call the destructor explicitly and tell the memory allocator that
		// the corresponding memory block is now free
        mContactPoints[i]->~ContactPoint();
        mMemoryAllocator.release(mContactPoints[i], sizeof(ContactPoint));
    }
    mNbContactPoints = 0;
}

// Add a contact point
void ContactManifold::addContactPoint(const ContactPointInfo* contactPointInfo) {

    assert(mNbContactPoints < MAX_CONTACT_POINTS_IN_MANIFOLD);

    // Create the new contact point
    ContactPoint* contactPoint = new (mMemoryAllocator.allocate(sizeof(ContactPoint)))
            ContactPoint(contactPointInfo, mShape1->getLocalToWorldTransform(), mShape2->getLocalToWorldTransform());

    // Add the new contact point into the manifold
    mContactPoints[mNbContactPoints] = contactPoint;
    mNbContactPoints++;

}

// Remove a contact point
void ContactManifold::removeContactPoint(int index) {

    assert(mNbContactPoints > 0);
    assert(index >= 0 && index < mNbContactPoints);

    // Delete the contact
    mContactPoints[index]->~ContactPoint();
    mMemoryAllocator.release(mContactPoints[index], sizeof(ContactPoint));

    for (int i=index; (i+1) < mNbContactPoints; i++) {
        mContactPoints[i] = mContactPoints[i+1];
    }

    mNbContactPoints--;
}
