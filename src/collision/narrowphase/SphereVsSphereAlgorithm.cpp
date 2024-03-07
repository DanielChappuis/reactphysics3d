/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2024 Daniel Chappuis                                       *
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
#include <reactphysics3d/collision/narrowphase/SphereVsSphereAlgorithm.h>
#include <reactphysics3d/collision/shapes/SphereShape.h>
#include <reactphysics3d/collision/narrowphase/NarrowPhaseInfoBatch.h>

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;  

bool SphereVsSphereAlgorithm::testCollision(NarrowPhaseInfoBatch& narrowPhaseInfoBatch, uint32 batchStartIndex, uint32 batchNbItems, MemoryAllocator& /*memoryAllocator*/) {

    bool isCollisionFound = false;

    // For each item in the batch
    for (uint32 batchIndex = batchStartIndex; batchIndex < batchStartIndex + batchNbItems; batchIndex++) {

        assert(narrowPhaseInfoBatch.narrowPhaseInfos[batchIndex].nbContactPoints == 0);
        assert(!narrowPhaseInfoBatch.narrowPhaseInfos[batchIndex].isColliding);

        // Get the local-space to world-space transforms
        const Transform& transform1 = narrowPhaseInfoBatch.narrowPhaseInfos[batchIndex].shape1ToWorldTransform;
        const Transform& transform2 = narrowPhaseInfoBatch.narrowPhaseInfos[batchIndex].shape2ToWorldTransform;

        // Compute the distance between the centers
        Vector3 vectorBetweenCenters = transform2.getPosition() - transform1.getPosition();
        decimal squaredDistanceBetweenCenters = vectorBetweenCenters.lengthSquare();

        const SphereShape* sphereShape1 = static_cast<SphereShape*>(narrowPhaseInfoBatch.narrowPhaseInfos[batchIndex].collisionShape1);
        const SphereShape* sphereShape2 = static_cast<SphereShape*>(narrowPhaseInfoBatch.narrowPhaseInfos[batchIndex].collisionShape2);

        const decimal sphere1Radius = sphereShape1->getRadius();
        const decimal sphere2Radius = sphereShape2->getRadius();

        // Compute the sum of the radius
        const decimal sumRadiuses = sphere1Radius + sphere2Radius;

        // Compute the product of the sum of the radius
        const decimal sumRadiusesProducts = sumRadiuses * sumRadiuses;

        // If the sphere collision shapes intersect
        if (squaredDistanceBetweenCenters < sumRadiusesProducts) {

            const decimal penetrationDepth = sumRadiuses - std::sqrt(squaredDistanceBetweenCenters);

            // Make sure the penetration depth is not zero (even if the previous condition test was true the penetration depth can still be
            // zero because of precision issue of the computation at the previous line)
            if (penetrationDepth > 0) {

                // If we need to report contacts
                if (narrowPhaseInfoBatch.narrowPhaseInfos[batchIndex].reportContacts) {

                    const Transform transform1Inverse = transform1.getInverse();
                    const Transform transform2Inverse = transform2.getInverse();

                    Vector3 intersectionOnBody1;
                    Vector3 intersectionOnBody2;
                    Vector3 normal;

                    // If the two sphere centers are not at the same position
                    if (squaredDistanceBetweenCenters > MACHINE_EPSILON) {

                        const Vector3 centerSphere2InBody1LocalSpace = transform1Inverse * transform2.getPosition();
                        const Vector3 centerSphere1InBody2LocalSpace = transform2Inverse * transform1.getPosition();

                        intersectionOnBody1 = sphere1Radius * centerSphere2InBody1LocalSpace.getUnit();
                        intersectionOnBody2 = sphere2Radius * centerSphere1InBody2LocalSpace.getUnit();
                        normal = vectorBetweenCenters.getUnit();
                    }
                    else {    // If the sphere centers are at the same position (degenerate case)

                        // Take any contact normal direction
                        normal.setAllValues(0, 1, 0);

                        intersectionOnBody1 = sphere1Radius * (transform1Inverse.getOrientation() * normal);
                        intersectionOnBody2 = sphere2Radius * (transform2Inverse.getOrientation() * normal);
                    }

                    // Create the contact info object
                    narrowPhaseInfoBatch.addContactPoint(batchIndex, normal, penetrationDepth, intersectionOnBody1, intersectionOnBody2);
                }

                narrowPhaseInfoBatch.narrowPhaseInfos[batchIndex].isColliding = true;
                isCollisionFound = true;
            }
        }
    }

    return isCollisionFound;
}
