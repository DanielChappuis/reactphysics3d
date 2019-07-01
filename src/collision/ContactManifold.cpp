/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2019 Daniel Chappuis                                       *
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
#include "ContactManifold.h"
#include "constraint/ContactPoint.h"
#include "collision/ContactManifoldInfo.h"

using namespace reactphysics3d;

// Constructor
ContactManifold::ContactManifold(const ContactManifoldInfo* manifoldInfo, ProxyShape* shape1, ProxyShape* shape2,
                                 MemoryAllocator& memoryAllocator, const WorldSettings& worldSettings)
                : mShape1(shape1), mShape2(shape2), mContactPoints(nullptr),
                  mNbContactPoints(0), mFrictionImpulse1(0.0), mFrictionImpulse2(0.0),
                  mFrictionTwistImpulse(0.0), mIsAlreadyInIsland(false),
                  mMemoryAllocator(memoryAllocator), mNext(nullptr), mPrevious(nullptr), mIsObsolete(false),
                  mWorldSettings(worldSettings) {
    
    // For each contact point info in the manifold
    const ContactPointInfo* pointInfo = manifoldInfo->getFirstContactPointInfo();
    while(pointInfo != nullptr) {

        // Add the new contact point
        addContactPoint(pointInfo);

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

// Remove a contact point
void ContactManifold::removeContactPoint(ContactPoint* contactPoint) {

    assert(mNbContactPoints > 0);
    assert(mContactPoints != nullptr);
    assert(contactPoint != nullptr);

    ContactPoint* previous = contactPoint->getPrevious();
    ContactPoint* next = contactPoint->getNext();

    if (previous != nullptr) {
        previous->setNext(next);
    }
    else {
        mContactPoints = next;
    }

    if (next != nullptr) {
        next->setPrevious(previous);
    }

    // Delete the contact point
    contactPoint->~ContactPoint();
    mMemoryAllocator.release(contactPoint, sizeof(ContactPoint));

    mNbContactPoints--;
    assert(mNbContactPoints >= 0);
}

// Return the largest depth of all the contact points
decimal ContactManifold::getLargestContactDepth() const {
    decimal largestDepth = 0.0f;

    assert(mNbContactPoints > 0);

    ContactPoint* contactPoint = mContactPoints;
    while(contactPoint != nullptr){
        decimal depth = contactPoint->getPenetrationDepth();
        if (depth > largestDepth) {
            largestDepth = depth;
        }

        contactPoint = contactPoint->getNext();
    }

    return largestDepth;
}

// Add a contact point
void ContactManifold::addContactPoint(const ContactPointInfo* contactPointInfo) {

    assert(contactPointInfo != nullptr);

    // Create the new contact point
    ContactPoint* contactPoint = new (mMemoryAllocator.allocate(sizeof(ContactPoint))) ContactPoint(contactPointInfo, mWorldSettings);

    // Add the new contact point into the manifold
    contactPoint->setNext(mContactPoints);
    contactPoint->setPrevious(nullptr);
    if (mContactPoints != nullptr) {
        mContactPoints->setPrevious(contactPoint);
    }
    mContactPoints = contactPoint;

    mNbContactPoints++;
}

// Set to true to make the manifold obsolete
void ContactManifold::setIsObsolete(bool isObsolete, bool setContactPoints) {
    mIsObsolete = isObsolete;

    if (setContactPoints) {
        ContactPoint* contactPoint = mContactPoints;
        while (contactPoint != nullptr) {
            contactPoint->setIsObsolete(isObsolete);

            contactPoint = contactPoint->getNext();
        }
    }
}

// Clear the obsolete contact points
void ContactManifold::clearObsoleteContactPoints() {

    assert(mContactPoints != nullptr);

    // For each contact point of the manifold
    ContactPoint* contactPoint = mContactPoints;
    while (contactPoint != nullptr) {

        ContactPoint* nextContactPoint =  contactPoint->getNext();

        // If the contact point is obsolete
        if (contactPoint->getIsObsolete()) {

            // Remove the contact point
            removeContactPoint(contactPoint);
        }

        contactPoint = nextContactPoint;
    }

    assert(mNbContactPoints > 0);
    assert(mContactPoints != nullptr);
}

// Make sure we do not have too much contact points by keeping only the best
// contact points of the manifold (with largest penetration depth)
void ContactManifold::reduce() {

    assert(mContactPoints != nullptr);

    // Remove contact points while there is too much contact points
    while (mNbContactPoints > MAX_CONTACT_POINTS_IN_MANIFOLD) {
        removeNonOptimalContactPoint();
    }

    assert(mNbContactPoints <= MAX_CONTACT_POINTS_IN_MANIFOLD && mNbContactPoints > 0);
    assert(mContactPoints != nullptr);
}

// Remove a contact point that is not optimal (with a small penetration depth)
void ContactManifold::removeNonOptimalContactPoint() {

    assert(mContactPoints != nullptr);
    assert(mNbContactPoints > MAX_CONTACT_POINTS_IN_MANIFOLD);

    // Get the contact point with the minimum penetration depth among all points
    ContactPoint* contactPoint = mContactPoints;
    ContactPoint* minContactPoint = nullptr;
    decimal minPenetrationDepth = DECIMAL_LARGEST;
    while (contactPoint != nullptr) {

        ContactPoint* nextContactPoint =  contactPoint->getNext();

        if (contactPoint->getPenetrationDepth() < minPenetrationDepth) {

            minContactPoint = contactPoint;
            minPenetrationDepth = contactPoint->getPenetrationDepth();
        }

        contactPoint = nextContactPoint;
    }

    assert(minContactPoint != nullptr);

    // Remove the non optimal contact point
    removeContactPoint(minContactPoint);

    assert(mNbContactPoints > 0);
    assert(mContactPoints != nullptr);
}
