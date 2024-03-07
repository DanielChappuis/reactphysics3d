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
#include <reactphysics3d/collision/narrowphase/SphereVsCapsuleAlgorithm.h>
#include <reactphysics3d/collision/shapes/SphereShape.h>
#include <reactphysics3d/collision/shapes/CapsuleShape.h>
#include <reactphysics3d/collision/narrowphase/NarrowPhaseInfoBatch.h>

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;  

// Compute the narrow-phase collision detection between a sphere and a capsule
// This technique is based on the "Robust Contact Creation for Physics Simulations" presentation
// by Dirk Gregorius.
bool SphereVsCapsuleAlgorithm::testCollision(NarrowPhaseInfoBatch& narrowPhaseInfoBatch, uint32 batchStartIndex, uint32 batchNbItems, MemoryAllocator& /*memoryAllocator*/) {

    bool isCollisionFound = false;

    for (uint32 batchIndex = batchStartIndex; batchIndex < batchStartIndex + batchNbItems; batchIndex++) {

        assert(!narrowPhaseInfoBatch.narrowPhaseInfos[batchIndex].isColliding);
        assert(narrowPhaseInfoBatch.narrowPhaseInfos[batchIndex].nbContactPoints == 0);

        const bool isSphereShape1 = narrowPhaseInfoBatch.narrowPhaseInfos[batchIndex].collisionShape1->getType() == CollisionShapeType::SPHERE;

        const SphereShape* sphereShape = static_cast<SphereShape*>(isSphereShape1 ? narrowPhaseInfoBatch.narrowPhaseInfos[batchIndex].collisionShape1 : narrowPhaseInfoBatch.narrowPhaseInfos[batchIndex].collisionShape2);
        const CapsuleShape* capsuleShape = static_cast<CapsuleShape*>(isSphereShape1 ? narrowPhaseInfoBatch.narrowPhaseInfos[batchIndex].collisionShape2 : narrowPhaseInfoBatch.narrowPhaseInfos[batchIndex].collisionShape1);

        const decimal capsuleHeight = capsuleShape->getHeight();
        const decimal sphereRadius = sphereShape->getRadius();
        const decimal capsuleRadius = capsuleShape->getRadius();

        // Get the transform from sphere local-space to capsule local-space
        const Transform& sphereToWorldTransform = isSphereShape1 ? narrowPhaseInfoBatch.narrowPhaseInfos[batchIndex].shape1ToWorldTransform : narrowPhaseInfoBatch.narrowPhaseInfos[batchIndex].shape2ToWorldTransform;
        const Transform& capsuleToWorldTransform = isSphereShape1 ? narrowPhaseInfoBatch.narrowPhaseInfos[batchIndex].shape2ToWorldTransform : narrowPhaseInfoBatch.narrowPhaseInfos[batchIndex].shape1ToWorldTransform;
        const Transform worldToCapsuleTransform = capsuleToWorldTransform.getInverse();
        const Transform sphereToCapsuleSpaceTransform = worldToCapsuleTransform * sphereToWorldTransform;

        // Transform the center of the sphere into the local-space of the capsule shape
        const Vector3 sphereCenter = sphereToCapsuleSpaceTransform.getPosition();

        // Compute the end-points of the inner segment of the capsule
        const decimal capsuleHalfHeight = capsuleHeight * decimal(0.5);
        const Vector3 capsuleSegA(0, -capsuleHalfHeight, 0);
        const Vector3 capsuleSegB(0, capsuleHalfHeight, 0);

        // Compute the point on the inner capsule segment that is the closes to center of sphere
        const Vector3 closestPointOnSegment = computeClosestPointOnSegment(capsuleSegA, capsuleSegB, sphereCenter);

        // Compute the distance between the sphere center and the closest point on the segment
        Vector3 sphereCenterToSegment = (closestPointOnSegment - sphereCenter);
        const decimal sphereSegmentDistanceSquare = sphereCenterToSegment.lengthSquare();

        // Compute the sum of the radius of the sphere and the capsule (virtual sphere)
        decimal sumRadius = sphereRadius + capsuleRadius;

        // If the collision shapes overlap
        if (sphereSegmentDistanceSquare < sumRadius * sumRadius) {

            decimal penetrationDepth;
            Vector3 normalWorld;
            Vector3 contactPointSphereLocal;
            Vector3 contactPointCapsuleLocal;

            // If we need to report contacts
            if (narrowPhaseInfoBatch.narrowPhaseInfos[batchIndex].reportContacts) {

                // If the sphere center is not on the capsule inner segment
                if (sphereSegmentDistanceSquare > MACHINE_EPSILON) {

                    decimal sphereSegmentDistance = std::sqrt(sphereSegmentDistanceSquare);
                    sphereCenterToSegment /= sphereSegmentDistance;

                    contactPointSphereLocal = sphereToCapsuleSpaceTransform.getInverse() * (sphereCenter + sphereCenterToSegment * sphereRadius);
                    contactPointCapsuleLocal = closestPointOnSegment - sphereCenterToSegment * capsuleRadius;

                    normalWorld = capsuleToWorldTransform.getOrientation() * sphereCenterToSegment;

                    penetrationDepth = sumRadius - sphereSegmentDistance;

                    if (!isSphereShape1) {
                        normalWorld = -normalWorld;
                    }
                }
                else {  // If the sphere center is on the capsule inner segment (degenerate case)

                    // We take any direction that is orthogonal to the inner capsule segment as a contact normal

                    // Capsule inner segment
                    Vector3 capsuleSegment = (capsuleSegB - capsuleSegA).getUnit();

                    Vector3 vec1(1, 0, 0);
                    Vector3 vec2(0, 1, 0);

                    // Get the vectors (among vec1 and vec2) that is the most orthogonal to the capsule inner segment (smallest absolute dot product)
                    decimal cosA1 = std::abs(capsuleSegment.x);		// abs(vec1.dot(seg2))
                    decimal cosA2 = std::abs(capsuleSegment.y);	    // abs(vec2.dot(seg2))

                    penetrationDepth = sumRadius;

                    // We choose as a contact normal, any direction that is perpendicular to the inner capsule segment
                    Vector3 normalCapsuleSpace = cosA1 < cosA2 ? capsuleSegment.cross(vec1) : capsuleSegment.cross(vec2);
                    normalWorld = capsuleToWorldTransform.getOrientation() * normalCapsuleSpace;

                    // Compute the two local contact points
                    contactPointSphereLocal = sphereToCapsuleSpaceTransform.getInverse() * (sphereCenter + normalCapsuleSpace * sphereRadius);
                    contactPointCapsuleLocal = sphereCenter - normalCapsuleSpace * capsuleRadius;
                }

                if (penetrationDepth <= decimal(0.0)) {

                    // No collision
                    continue;
                }

                // Create the contact info object
                narrowPhaseInfoBatch.addContactPoint(batchIndex, normalWorld, penetrationDepth,
                                                 isSphereShape1 ? contactPointSphereLocal : contactPointCapsuleLocal,
                                                 isSphereShape1 ? contactPointCapsuleLocal : contactPointSphereLocal);
            }

            narrowPhaseInfoBatch.narrowPhaseInfos[batchIndex].isColliding = true;
            isCollisionFound = true;
            continue;
        }
    }

    return isCollisionFound;
}
