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
#include "SphereVsSphereAlgorithm.h"
#include "collision/shapes/SphereShape.h"
#include "collision/NarrowPhaseInfo.h"

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;  

bool SphereVsSphereAlgorithm::testCollision(NarrowPhaseInfo* narrowPhaseInfo, bool reportContacts,
                                            MemoryAllocator& memoryAllocator) {
    
    assert(narrowPhaseInfo->collisionShape1->getType() == CollisionShapeType::SPHERE);
    assert(narrowPhaseInfo->collisionShape2->getType() == CollisionShapeType::SPHERE);

    // Get the sphere collision shapes
    const SphereShape* sphereShape1 = static_cast<const SphereShape*>(narrowPhaseInfo->collisionShape1);
    const SphereShape* sphereShape2 = static_cast<const SphereShape*>(narrowPhaseInfo->collisionShape2);

    // Get the local-space to world-space transforms
    const Transform& transform1 = narrowPhaseInfo->shape1ToWorldTransform;
    const Transform& transform2 = narrowPhaseInfo->shape2ToWorldTransform;

    // Compute the distance between the centers
    Vector3 vectorBetweenCenters = transform2.getPosition() - transform1.getPosition();
    decimal squaredDistanceBetweenCenters = vectorBetweenCenters.lengthSquare();

    // Compute the sum of the radius
    decimal sumRadius = sphereShape1->getRadius() + sphereShape2->getRadius();
    
    // If the sphere collision shapes intersect
    if (squaredDistanceBetweenCenters < sumRadius * sumRadius) {

        if (reportContacts) {

            Vector3 centerSphere2InBody1LocalSpace = transform1.getInverse() * transform2.getPosition();
            Vector3 centerSphere1InBody2LocalSpace = transform2.getInverse() * transform1.getPosition();
            decimal penetrationDepth = sumRadius - std::sqrt(squaredDistanceBetweenCenters);
			Vector3 intersectionOnBody1;
			Vector3 intersectionOnBody2;
			Vector3 normal;

			// If the two sphere centers are not at the same position
			if (squaredDistanceBetweenCenters > MACHINE_EPSILON) {

				intersectionOnBody1 = sphereShape1->getRadius() * centerSphere2InBody1LocalSpace.getUnit();
				intersectionOnBody2 = sphereShape2->getRadius() * centerSphere1InBody2LocalSpace.getUnit();
				normal = vectorBetweenCenters.getUnit();
			}
			else {    // If the sphere centers are at the same position (degenerate case)

				// Take any contact normal direction
				normal.setAllValues(0, 1, 0);

				intersectionOnBody1 = sphereShape1->getRadius() * (transform1.getInverse().getOrientation() * normal);
				intersectionOnBody2 = sphereShape2->getRadius() * (transform2.getInverse().getOrientation() * normal);
			}			
            
			// Create the contact info object
            narrowPhaseInfo->addContactPoint(normal, penetrationDepth, intersectionOnBody1, intersectionOnBody2);
        }

        return true;
    }

    return false;
}
