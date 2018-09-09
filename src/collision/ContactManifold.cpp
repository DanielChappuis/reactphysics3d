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
#include "ContactManifold.h"
#include "constraint/ContactPoint.h"

using namespace reactphysics3d;

// Constructor
ContactManifold::ContactManifold(ProxyShape* shape1, ProxyShape* shape2,
                                 MemoryAllocator& memoryAllocator, const WorldSettings& worldSettings)
                : mShape1(shape1), mShape2(shape2), mContactPoints(nullptr),
                  mNbContactPoints(0), mFrictionImpulse1(0.0), mFrictionImpulse2(0.0),
                  mFrictionTwistImpulse(0.0), mIsAlreadyInIsland(false),
                  mMemoryAllocator(memoryAllocator), mNext(nullptr), mPrevious(nullptr), mIsObsolete(false),
                  mWorldSettings(worldSettings) {
    
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

       // For each contact point in the manifold
       bool isSimilarPointFound = false;
       ContactPoint* oldContactPoint = mContactPoints;
       while (oldContactPoint != nullptr) {

           assert(oldContactPoint != nullptr);

            // If the new contact point is similar (very close) to the old contact point
            if (oldContactPoint->isSimilarWithContactPoint(contactPointInfo)) {

                // Replace (update) the old contact point with the new one
                oldContactPoint->update(contactPointInfo);
                isSimilarPointFound = true;
                break;
            }

            oldContactPoint = oldContactPoint->getNext();
       }

       // If we have not found a similar contact point
       if (!isSimilarPointFound) {

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

    // The old manifold is no longer obsolete
    mIsObsolete = false;
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

// Reduce the number of contact points of the currently computed manifold
// This is based on the technique described by Dirk Gregorius in his
// "Contacts Creation" GDC presentation. This method will reduce the number of
// contact points to a maximum of 4 points (but it can be less).
void ContactManifold::reduce(const Transform& shape1ToWorldTransform) {

    assert(mContactPoints != nullptr);

    // The following algorithm only works to reduce to a maximum of 4 contact points
    assert(MAX_CONTACT_POINTS_IN_MANIFOLD == 4);

    // If there are too many contact points in the manifold
    if (mNbContactPoints > MAX_CONTACT_POINTS_IN_MANIFOLD) {

        uint nbReducedPoints = 0;

        ContactPoint* pointsToKeep[MAX_CONTACT_POINTS_IN_MANIFOLD];
        for (int i=0; i<MAX_CONTACT_POINTS_IN_MANIFOLD; i++) {
            pointsToKeep[i] = nullptr;
        }

        //  Compute the initial contact point we need to keep.
        // The first point we keep is always the point in a given
        // constant direction (in order to always have same contact points
        // between frames for better stability)

        const Transform worldToShape1Transform = shape1ToWorldTransform.getInverse();

        // Compute the contact normal of the manifold (we use the first contact point)
        // in the local-space of the first collision shape
        const Vector3 contactNormalShape1Space = worldToShape1Transform.getOrientation() * mContactPoints->getNormal();

        // Compute a search direction
        const Vector3 searchDirection(1, 1, 1);
        ContactPoint* element = mContactPoints;
        pointsToKeep[0] = element;
        decimal maxDotProduct = searchDirection.dot(element->getLocalPointOnShape1());
        element = element->getNext();
        nbReducedPoints = 1;
        while(element != nullptr) {

            decimal dotProduct = searchDirection.dot(element->getLocalPointOnShape1());
            if (dotProduct > maxDotProduct) {
                maxDotProduct = dotProduct;
                pointsToKeep[0] = element;
            }
            element = element->getNext();
        }
        assert(pointsToKeep[0] != nullptr);
        assert(nbReducedPoints == 1);

        // Compute the second contact point we need to keep.
        // The second point we keep is the one farthest away from the first point.

        decimal maxDistance = decimal(0.0);
        element = mContactPoints;
        while(element != nullptr) {

            if (element == pointsToKeep[0]) {
                element = element->getNext();
                continue;
            }

            decimal distance = (pointsToKeep[0]->getLocalPointOnShape1() - element->getLocalPointOnShape1()).lengthSquare();
            if (distance >= maxDistance) {
                maxDistance = distance;
                pointsToKeep[1] = element;
                nbReducedPoints = 2;
            }
            element = element->getNext();
        }
        assert(pointsToKeep[1] != nullptr);
        assert(nbReducedPoints == 2);

        // Compute the third contact point we need to keep.
        // The second point is the one producing the triangle with the larger area
        // with first and second point.

        // We compute the most positive or most negative triangle area (depending on winding)
        ContactPoint* thirdPointMaxArea = nullptr;
        ContactPoint* thirdPointMinArea = nullptr;
        decimal minArea = decimal(0.0);
        decimal maxArea = decimal(0.0);
        bool isPreviousAreaPositive = true;
        element = mContactPoints;
        while(element != nullptr) {

            if (element == pointsToKeep[0] || element == pointsToKeep[1]) {
                element = element->getNext();
                continue;
            }

            const Vector3 newToFirst = pointsToKeep[0]->getLocalPointOnShape1() - element->getLocalPointOnShape1();
            const Vector3 newToSecond = pointsToKeep[1]->getLocalPointOnShape1() - element->getLocalPointOnShape1();

            // Compute the triangle area
            decimal area = newToFirst.cross(newToSecond).dot(contactNormalShape1Space);

            if (area >= maxArea) {
                maxArea = area;
                thirdPointMaxArea = element;
            }
            if (area <= minArea) {
                minArea = area;
                thirdPointMinArea = element;
            }
            element = element->getNext();
        }
        assert(minArea <= decimal(0.0));
        assert(maxArea >= decimal(0.0));
        if (maxArea > (-minArea)) {
            isPreviousAreaPositive = true;
            pointsToKeep[2] = thirdPointMaxArea;
            nbReducedPoints = 3;
        }
        else {
            isPreviousAreaPositive = false;
            pointsToKeep[2] = thirdPointMinArea;
            nbReducedPoints = 3;
        }

        // Compute the 4th point by choosing the triangle that add the most
        // triangle area to the previous triangle and has opposite sign area (opposite winding)

        decimal largestArea = decimal(0.0); // Largest area (positive or negative)
        element = mContactPoints;

        if (nbReducedPoints == 3) {

            // For each remaining point
            while(element != nullptr) {

                if (element == pointsToKeep[0] || element == pointsToKeep[1] || element == pointsToKeep[2]) {
                element = element->getNext();
                continue;
                }

                // For each edge of the triangle made by the first three points
                for (uint i=0; i<3; i++) {

                uint edgeVertex1Index = i;
                uint edgeVertex2Index = i < 2 ? i + 1 : 0;

                const Vector3 newToFirst = pointsToKeep[edgeVertex1Index]->getLocalPointOnShape1() - element->getLocalPointOnShape1();
                const Vector3 newToSecond = pointsToKeep[edgeVertex2Index]->getLocalPointOnShape1() - element->getLocalPointOnShape1();

                // Compute the triangle area
                decimal area = newToFirst.cross(newToSecond).dot(contactNormalShape1Space);

                // We are looking at the triangle with maximal area (positive or negative).
                // If the previous area is positive, we are looking at negative area now.
                // If the previous area is negative, we are looking at the positive area now.
                if (isPreviousAreaPositive && area <= largestArea) {
                    largestArea = area;
                    pointsToKeep[3] = element;
                    nbReducedPoints = 4;
                }
                else if (!isPreviousAreaPositive && area >= largestArea) {
                    largestArea = area;
                    pointsToKeep[3] = element;
                    nbReducedPoints = 4;
                }
                }

                element = element->getNext();
            }
        }

        // Delete the contact points we do not want to keep from the linked list
        element = mContactPoints;
        ContactPoint* previousElement = nullptr;
        while(element != nullptr) {

            bool deletePoint = true;

            // Skip the points we want to keep
            for (uint i=0; i<nbReducedPoints; i++) {

                if (element == pointsToKeep[i]) {

                    previousElement = element;
                    element = element->getNext();
                    deletePoint = false;
                }
            }

            if (deletePoint) {

                ContactPoint* contactPointToDelete = element;
                element = element->getNext();

                removeContactPoint(contactPointToDelete);
            }
        }

        assert(nbReducedPoints > 0 && nbReducedPoints <= 4);
        mNbContactPoints = nbReducedPoints;
    }
}
