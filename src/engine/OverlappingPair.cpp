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
#include <cassert>
#include "OverlappingPair.h"
#include "collision/ContactManifoldInfo.h"

using namespace reactphysics3d;

// Constructor
OverlappingPair::OverlappingPair(ProxyShape* shape1, ProxyShape* shape2,
                                 Allocator& manifoldsAllocator, Allocator& temporaryMemoryAllocator)
                : mContactManifoldSet(shape1, shape2, manifoldsAllocator), mPotentialContactManifolds(nullptr),
                  mTempMemoryAllocator(temporaryMemoryAllocator) {
    
}                               

// Create a new potential contact manifold using contact-points from narrow-phase
void OverlappingPair::addPotentialContactPoints(NarrowPhaseInfo* narrowPhaseInfo) {

    assert(narrowPhaseInfo->contactPoints != nullptr);

    // For each potential contact point to add
    ContactPointInfo* contactPoint = narrowPhaseInfo->contactPoints;
    while (contactPoint != nullptr) {

        ContactPointInfo* nextContactPoint = contactPoint->next;

        // Compute the contact normal id
        short contactNormalId = ContactManifoldSet::computeCubemapNormalId(contactPoint->normal);

        // Look if the contact point correspond to an existing potential manifold
        // (if the contact point normal is similar to the normal of an existing manifold)
        ContactManifoldInfo* manifold = mPotentialContactManifolds;
        bool similarManifoldFound = false;
        while(manifold != nullptr) {

            // If we have found a corresponding manifold for the new contact point
            // (a manifold with a similar contact normal direction)
            if (manifold->getContactNormalId() == contactNormalId) {

                // Add the contact point to the manifold
                manifold->addContactPoint(contactPoint, contactNormalId);

               similarManifoldFound = true;

               break;
            }

            manifold = manifold->getNext();
        }

        // If we have not found an existing manifold with a similar contact normal
        if (!similarManifoldFound) {

            // Create a new potential contact manifold
            ContactManifoldInfo* manifoldInfo = new (mTempMemoryAllocator.allocate(sizeof(ContactManifoldInfo)))
                                            ContactManifoldInfo(mTempMemoryAllocator);

            // Add the manifold into the linked-list of potential contact manifolds
            manifoldInfo->mNext = mPotentialContactManifolds;
            mPotentialContactManifolds = manifoldInfo;

            // Add the contact point to the manifold
            manifoldInfo->addContactPoint(contactPoint, contactNormalId);
        }

        contactPoint = nextContactPoint;
    }

    // All the contact point info of the narrow-phase info have been moved
    // into the potential contacts of the overlapping pair
    narrowPhaseInfo->contactPoints = nullptr;
}

// Clear all the potential contact manifolds
void OverlappingPair::clearPotentialContactManifolds() {

    // Do we need to release memory
    if (mTempMemoryAllocator.isReleaseNeeded()) {

        ContactManifoldInfo* element = mPotentialContactManifolds;
        while(element != nullptr) {

            // Remove the proxy collision shape
            ContactManifoldInfo* elementToRemove = element;
            element = element->getNext();

            // Delete the element
            elementToRemove->~ContactManifoldInfo();
            mTempMemoryAllocator.release(elementToRemove, sizeof(ContactManifoldInfo));
        }
    }

    mPotentialContactManifolds = nullptr;
}

// Reduce the number of contact points of all the potential contact manifolds
void OverlappingPair::reducePotentialContactManifolds() {

    // For each potential contact manifold
    ContactManifoldInfo* manifold = mPotentialContactManifolds;
    while (manifold != nullptr) {

        // Reduce the number of contact points of the manifold
        manifold->reduce(mContactManifoldSet.getShape1()->getLocalToWorldTransform());

        manifold = manifold->getNext();
    }
}
