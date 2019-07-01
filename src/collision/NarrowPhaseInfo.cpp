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
#include "NarrowPhaseInfo.h"
#include "ContactPointInfo.h"
#include "collision/shapes/TriangleShape.h"
#include "engine/OverlappingPair.h"

using namespace reactphysics3d;

// Constructor
NarrowPhaseInfo::NarrowPhaseInfo(OverlappingPair* pair, CollisionShape* shape1,
                CollisionShape* shape2, const Transform& shape1Transform,
                const Transform& shape2Transform, MemoryAllocator& shapeAllocator)
      : overlappingPair(pair), collisionShape1(shape1), collisionShape2(shape2),
        shape1ToWorldTransform(shape1Transform), shape2ToWorldTransform(shape2Transform),
        contactPoints(nullptr), next(nullptr), collisionShapeAllocator(shapeAllocator) {

    // Add a collision info for the two collision shapes into the overlapping pair (if not present yet)
    overlappingPair->addLastFrameInfoIfNecessary(shape1->getId(), shape2->getId());
}

// Destructor
NarrowPhaseInfo::~NarrowPhaseInfo() {

    assert(contactPoints == nullptr);

	// Release the memory of the TriangleShape (this memory was allocated in the
	// MiddlePhaseTriangleCallback::testTriangle() method)
	if (collisionShape1->getName() == CollisionShapeName::TRIANGLE) {
        collisionShape1->~CollisionShape();
		collisionShapeAllocator.release(collisionShape1, sizeof(TriangleShape));
	}
	if (collisionShape2->getName() == CollisionShapeName::TRIANGLE) {
        collisionShape2->~CollisionShape();
        collisionShapeAllocator.release(collisionShape2, sizeof(TriangleShape));
    }
}

// Add a new contact point
void NarrowPhaseInfo::addContactPoint(const Vector3& contactNormal, decimal penDepth,
                     const Vector3& localPt1, const Vector3& localPt2) {

    assert(penDepth > decimal(0.0));
    assert(contactNormal.length() > decimal(0.8));

    // Get the memory allocator
    MemoryAllocator& allocator = overlappingPair->getTemporaryAllocator();

    // Create the contact point info
    ContactPointInfo* contactPointInfo = new (allocator.allocate(sizeof(ContactPointInfo)))
            ContactPointInfo(contactNormal, penDepth, localPt1, localPt2);

    // Add it into the linked list of contact points
    contactPointInfo->next = contactPoints;
    contactPoints = contactPointInfo;
}

/// Take all the generated contact points and create a new potential
/// contact manifold into the overlapping pair
void NarrowPhaseInfo::addContactPointsAsPotentialContactManifold() {
    overlappingPair->addPotentialContactPoints(this);
}

// Reset the remaining contact points
void NarrowPhaseInfo::resetContactPoints() {

    // Get the memory allocator
    MemoryAllocator& allocator = overlappingPair->getTemporaryAllocator();

    // For each remaining contact point info
    ContactPointInfo* element = contactPoints;
    while(element != nullptr) {

        ContactPointInfo* elementToDelete = element;

        element = element->next;

        // Call the destructor
        elementToDelete->~ContactPointInfo();

        // Delete the current element
        allocator.release(elementToDelete, sizeof(ContactPointInfo));
    }

    contactPoints = nullptr;
}
