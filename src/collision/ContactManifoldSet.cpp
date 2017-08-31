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
#include "ContactManifoldSet.h"

using namespace reactphysics3d;

// Constructor
ContactManifoldSet::ContactManifoldSet(ProxyShape* shape1, ProxyShape* shape2,
                                       Allocator& memoryAllocator)
                   : mNbManifolds(0), mShape1(shape1),
                     mShape2(shape2), mMemoryAllocator(memoryAllocator), mManifolds(nullptr) {

    // Compute the maximum number of manifolds allowed between the two shapes
    mNbMaxManifolds = computeNbMaxContactManifolds(shape1->getCollisionShape(), shape2->getCollisionShape());
}

// Destructor
ContactManifoldSet::~ContactManifoldSet() {

    // Clear all the contact manifolds
    clear();
}

void ContactManifoldSet::addContactManifold(const ContactManifoldInfo* contactManifoldInfo) {

    assert(contactManifoldInfo->getFirstContactPointInfo() != nullptr);

    // Try to find an existing contact manifold with similar contact normal
    ContactManifold* similarManifold = selectManifoldWithSimilarNormal(contactManifoldInfo->getContactNormalId());

    // If a similar manifold has been found
    if (similarManifold != nullptr) {

        // Update the old manifold with the new one
        updateManifoldWithNewOne(similarManifold, contactManifoldInfo);
    }
    else {

        bool addNewManifold = true;

        // If there are too much contact manifolds in the set
        if (mNbManifolds >= mNbMaxManifolds) {

            // We need to remove a manifold from the set.
            // We seach for the one with the smallest maximum penetration depth among its contact points
            ContactManifold* minDepthManifold = getManifoldWithSmallestContactPenetrationDepth(contactManifoldInfo->getLargestPenetrationDepth());

            // If the manifold with the minimum penetration depth is an existing one
            if (minDepthManifold != nullptr) {

                // Remove the manifold
                removeManifold(minDepthManifold);
            }
            else {

                // The manifold we do not want to get is the new one. Therefore, we do not add it to the set
                addNewManifold = false;
            }
        }

        // If we need to add the new contact manifold
        if (addNewManifold) {

            // Create a new contact manifold
            createManifold(contactManifoldInfo);
        }
    }
}

// Update a previous similar manifold with a new one
void ContactManifoldSet::updateManifoldWithNewOne(ContactManifold* oldManifold, const ContactManifoldInfo* newManifold) {

    assert(oldManifold != nullptr);
    assert(newManifold != nullptr);

   // For each contact point of the new manifold
   ContactPointInfo* contactPointInfo = newManifold->getFirstContactPointInfo();
   assert(contactPointInfo != nullptr);
   while (contactPointInfo != nullptr) {

       // For each contact point in the old manifold
       bool isSimilarPointFound = false;
       ContactPoint* oldContactPoint = oldManifold->getContactPoints();
       while (oldContactPoint != nullptr) {

           assert(oldContactPoint != nullptr);

            // If the new contact point is similar (very close) to the old contact point
            if (oldContactPoint->isSimilarWithContactPoint(contactPointInfo)) {

                // Replace (update) the old contact point with the new one
                oldContactPoint->update(contactPointInfo, mShape1->getLocalToWorldTransform(),  mShape2->getLocalToWorldTransform());
                isSimilarPointFound = true;
                break;
            }

            oldContactPoint = oldContactPoint->getNext();
       }

       // If we have not found a similar contact point
       if (!isSimilarPointFound) {

           // Add the contact point to the manifold
           oldManifold->addContactPoint(contactPointInfo);
       }

       contactPointInfo = contactPointInfo->next;
   }

   // The old manifold is no longer obselete
   oldManifold->setIsObselete(false, false);
}

// Return the manifold with the smallest contact penetration depth among its points
ContactManifold* ContactManifoldSet::getManifoldWithSmallestContactPenetrationDepth(decimal initDepth) const {

    assert(mNbManifolds == mNbMaxManifolds);

    ContactManifold* minDepthManifold = nullptr;
    decimal minDepth = initDepth;
    ContactManifold* manifold = mManifolds;
    while (manifold != nullptr) {
        decimal depth = manifold->getLargestContactDepth();
        if (depth < minDepth) {
            minDepth = depth;
            minDepthManifold = manifold;
        }

        manifold = manifold->getNext();
    }

    return minDepthManifold;
}

// Return the contact manifold with a similar average normal.
// If no manifold has close enough average normal, it returns nullptr
ContactManifold* ContactManifoldSet::selectManifoldWithSimilarNormal(short int normalDirectionId) const {

    ContactManifold* manifold = mManifolds;

    // Return the Id of the manifold with the same normal direction id (if exists)
    while (manifold != nullptr) {
        if (normalDirectionId == manifold->getContactNormalId()) {
            return manifold;
        }

        manifold = manifold->getNext();
    }

    return nullptr;
}

// Map the normal vector into a cubemap face bucket (a face contains 4x4 buckets)
// Each face of the cube is divided into 4x4 buckets. This method maps the
// normal vector into of the of the bucket and returns a unique Id for the bucket
short int ContactManifoldSet::computeCubemapNormalId(const Vector3& normal) {

    assert(normal.lengthSquare() > MACHINE_EPSILON);

    int faceNo;
    decimal u, v;
    decimal max = max3(std::fabs(normal.x), std::fabs(normal.y), std::fabs(normal.z));
    Vector3 normalScaled = normal / max;

    if (normalScaled.x >= normalScaled.y && normalScaled.x >= normalScaled.z) {
        faceNo = normalScaled.x > 0 ? 0 : 1;
        u = normalScaled.y;
        v = normalScaled.z;
    }
    else if (normalScaled.y >= normalScaled.x && normalScaled.y >= normalScaled.z) {
        faceNo = normalScaled.y > 0 ? 2 : 3;
        u = normalScaled.x;
        v = normalScaled.z;
    }
    else {
        faceNo = normalScaled.z > 0 ? 4 : 5;
        u = normalScaled.x;
        v = normalScaled.y;
    }

    int indexU = std::floor(((u + 1)/2) * CONTACT_CUBEMAP_FACE_NB_SUBDIVISIONS);
    int indexV = std::floor(((v + 1)/2) * CONTACT_CUBEMAP_FACE_NB_SUBDIVISIONS);
    if (indexU == CONTACT_CUBEMAP_FACE_NB_SUBDIVISIONS) indexU--;
    if (indexV == CONTACT_CUBEMAP_FACE_NB_SUBDIVISIONS) indexV--;

    const int nbSubDivInFace = CONTACT_CUBEMAP_FACE_NB_SUBDIVISIONS * CONTACT_CUBEMAP_FACE_NB_SUBDIVISIONS;
    return faceNo * 200 + indexU * nbSubDivInFace + indexV;
}

// Clear the contact manifold set
void ContactManifoldSet::clear() {

    ContactManifold* manifold = mManifolds;
    while(manifold != nullptr) {

        ContactManifold* nextManifold = manifold->getNext();

        // Delete the contact manifold
        manifold->~ContactManifold();
        mMemoryAllocator.release(manifold, sizeof(ContactManifold));

        manifold = nextManifold;

        mNbManifolds--;
    }

    assert(mNbManifolds == 0);
}

// Create a new contact manifold and add it to the set
void ContactManifoldSet::createManifold(const ContactManifoldInfo* manifoldInfo) {
    assert(mNbManifolds < mNbMaxManifolds);

    ContactManifold* manifold = new (mMemoryAllocator.allocate(sizeof(ContactManifold)))
                                    ContactManifold(manifoldInfo, mShape1, mShape2, mMemoryAllocator);
    manifold->setPrevious(nullptr);
    manifold->setNext(mManifolds);
    mManifolds = manifold;

    mNbManifolds++;
}

// Remove a contact manifold from the set
void ContactManifoldSet::removeManifold(ContactManifold* manifold) {

    assert(mNbManifolds > 0);

    ContactManifold* previous = manifold->getPrevious();
    ContactManifold* next = manifold->getNext();

    if (previous != nullptr) {
        previous->setNext(manifold->getNext());
    }
	else {
		mManifolds = next;
	}

    if (next != nullptr) {
        next->setPrevious(manifold->getPrevious());
    }

    // Delete the contact manifold
    manifold->~ContactManifold();
    mMemoryAllocator.release(manifold, sizeof(ContactManifold));

    mNbManifolds--;
}

// Make all the contact manifolds and contact points obselete
void ContactManifoldSet::makeContactsObselete() {

    ContactManifold* manifold = mManifolds;
    while (manifold != nullptr) {

        manifold->setIsObselete(true, true);

        manifold = manifold->getNext();
    }
}

// Clear the obselete contact manifolds and contact points
void ContactManifoldSet::clearObseleteManifoldsAndContactPoints() {

    ContactManifold* manifold = mManifolds;
    ContactManifold* previousManifold = nullptr;
    while (manifold != nullptr) {
        ContactManifold* nextManifold = manifold->getNext();

        if (manifold->getIsObselete()) {

            if (previousManifold != nullptr) {
                previousManifold->setNext(nextManifold);

                if (nextManifold != nullptr) {
                    nextManifold->setPrevious(previousManifold);
                }
            }
            else {
                mManifolds = nextManifold;
            }

            // Delete the contact manifold
            removeManifold(manifold);

        }
        else {
            manifold->clearObseleteContactPoints();
            previousManifold = manifold;
        }

        manifold = nextManifold;
    }
}
