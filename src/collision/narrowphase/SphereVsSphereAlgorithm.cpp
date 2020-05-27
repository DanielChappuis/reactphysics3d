/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2020 Daniel Chappuis                                       *
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
#include <reactphysics3d/collision/narrowphase/SphereVsSphereNarrowPhaseInfoBatch.h>

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;  

bool SphereVsSphereAlgorithm::testCollision(SphereVsSphereNarrowPhaseInfoBatch& narrowPhaseInfoBatch, uint batchStartIndex, uint batchNbItems, MemoryAllocator& memoryAllocator) {

    bool isCollisionFound = false;

    // For each item in the batch
    for (uint batchIndex = batchStartIndex; batchIndex < batchStartIndex + batchNbItems; batchIndex++) {

        assert(narrowPhaseInfoBatch.contactPoints[batchIndex].size() == 0);
        assert(!narrowPhaseInfoBatch.isColliding[batchIndex]);

        // Get the local-space to world-space transforms
        const Transform& transform1 = narrowPhaseInfoBatch.shape1ToWorldTransforms[batchIndex];
        const Transform& transform2 = narrowPhaseInfoBatch.shape2ToWorldTransforms[batchIndex];

        // Compute the distance between the centers
        Vector3 vectorBetweenCenters = transform2.getPosition() - transform1.getPosition();
        decimal squaredDistanceBetweenCenters = vectorBetweenCenters.lengthSquare();

        // Compute the sum of the radius
        decimal sumRadiuses = narrowPhaseInfoBatch.sphere1Radiuses[batchIndex] + narrowPhaseInfoBatch.sphere2Radiuses[batchIndex];

        // Compute the product of the sum of the radius
        decimal sumRadiusesProducts = sumRadiuses * sumRadiuses;

        // If the sphere collision shapes intersect
        if (squaredDistanceBetweenCenters < sumRadiusesProducts) {

            // If we need to report contacts
            if (narrowPhaseInfoBatch.reportContacts[batchIndex]) {

                const Transform transform1Inverse = transform1.getInverse();
                const Transform transform2Inverse = transform2.getInverse();

                decimal penetrationDepth = sumRadiuses - std::sqrt(squaredDistanceBetweenCenters);
                Vector3 intersectionOnBody1;
                Vector3 intersectionOnBody2;
                Vector3 normal;

                // If the two sphere centers are not at the same position
                if (squaredDistanceBetweenCenters > MACHINE_EPSILON) {

                    Vector3 centerSphere2InBody1LocalSpace = transform1Inverse * transform2.getPosition();
                    Vector3 centerSphere1InBody2LocalSpace = transform2Inverse * transform1.getPosition();

                    intersectionOnBody1 = narrowPhaseInfoBatch.sphere1Radiuses[batchIndex] * centerSphere2InBody1LocalSpace.getUnit();
                    intersectionOnBody2 = narrowPhaseInfoBatch.sphere2Radiuses[batchIndex] * centerSphere1InBody2LocalSpace.getUnit();
                    normal = vectorBetweenCenters.getUnit();
                }
                else {    // If the sphere centers are at the same position (degenerate case)

                    // Take any contact normal direction
                    normal.setAllValues(0, 1, 0);

                    intersectionOnBody1 = narrowPhaseInfoBatch.sphere1Radiuses[batchIndex] * (transform1Inverse.getOrientation() * normal);
                    intersectionOnBody2 = narrowPhaseInfoBatch.sphere2Radiuses[batchIndex] * (transform2Inverse.getOrientation() * normal);
                }

                // Create the contact info object
                narrowPhaseInfoBatch.addContactPoint(batchIndex, normal, penetrationDepth, intersectionOnBody1, intersectionOnBody2);
            }

            narrowPhaseInfoBatch.isColliding[batchIndex] = true;
            isCollisionFound = true;
        }
    }

    return isCollisionFound;
}
