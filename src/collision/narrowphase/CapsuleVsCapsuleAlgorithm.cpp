/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2022 Daniel Chappuis                                       *
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
#include <reactphysics3d/collision/narrowphase/CapsuleVsCapsuleAlgorithm.h>
#include <reactphysics3d/collision/shapes/CapsuleShape.h>
#include <reactphysics3d/collision/narrowphase/NarrowPhaseInfoBatch.h>

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;  

// Compute the narrow-phase collision detection between two capsules
// This technique is based on the "Robust Contact Creation for Physics Simulations" presentation
// by Dirk Gregorius.
bool CapsuleVsCapsuleAlgorithm::testCollision(NarrowPhaseInfoBatch& narrowPhaseInfoBatch, uint32 batchStartIndex, uint32 batchNbItems, MemoryAllocator& /*memoryAllocator*/) {
    
    bool isCollisionFound = false;

    for (uint32 batchIndex = batchStartIndex; batchIndex < batchStartIndex + batchNbItems; batchIndex++) {

        assert(narrowPhaseInfoBatch.narrowPhaseInfos[batchIndex].nbContactPoints == 0);

        assert(!narrowPhaseInfoBatch.narrowPhaseInfos[batchIndex].isColliding);

        // Get the transform from capsule 1 local-space to capsule 2 local-space
        const Transform capsule1ToCapsule2SpaceTransform = narrowPhaseInfoBatch.narrowPhaseInfos[batchIndex].shape2ToWorldTransform.getInverse() *
                                                           narrowPhaseInfoBatch.narrowPhaseInfos[batchIndex].shape1ToWorldTransform;

        const CapsuleShape* capsuleShape1 = static_cast<CapsuleShape*>(narrowPhaseInfoBatch.narrowPhaseInfos[batchIndex].collisionShape1);
        const CapsuleShape* capsuleShape2 = static_cast<CapsuleShape*>(narrowPhaseInfoBatch.narrowPhaseInfos[batchIndex].collisionShape2);

        const decimal capsule1Height = capsuleShape1->getHeight();
        const decimal capsule2Height = capsuleShape2->getHeight();
        const decimal capsule1Radius = capsuleShape1->getRadius();
        const decimal capsule2Radius = capsuleShape2->getRadius();

        // Compute the end-points of the inner segment of the first capsule
        const decimal capsule1HalfHeight = capsule1Height * decimal(0.5);
        Vector3 capsule1SegA(0, -capsule1HalfHeight, 0);
        Vector3 capsule1SegB(0, capsule1HalfHeight, 0);
        capsule1SegA = capsule1ToCapsule2SpaceTransform * capsule1SegA;
        capsule1SegB = capsule1ToCapsule2SpaceTransform * capsule1SegB;

        // Compute the end-points of the inner segment of the second capsule
        const decimal capsule2HalfHeight = capsule2Height * decimal(0.5);
        const Vector3 capsule2SegA(0, -capsule2HalfHeight, 0);
        const Vector3 capsule2SegB(0, capsule2HalfHeight, 0);

        // The two inner capsule segments
        const Vector3 seg1 = capsule1SegB - capsule1SegA;
        const Vector3 seg2 = capsule2SegB - capsule2SegA;

        // Compute the sum of the radius of the two capsules (virtual spheres)
        const decimal sumRadius = capsule1Radius + capsule2Radius;

        // If the two capsules are parallel (we create two contact points)
        bool areCapsuleInnerSegmentsParralel = areParallelVectors(seg1, seg2);
        if (areCapsuleInnerSegmentsParralel) {

            // If the distance between the two segments is larger than the sum of the capsules radius (we do not have overlapping)
            const decimal segmentsPerpendicularDistance = computePointToLineDistance(capsule1SegA, capsule1SegB, capsule2SegA);
            if (segmentsPerpendicularDistance >= sumRadius) {

                // The capsule are parallel but their inner segment distance is larger than the sum of the capsules radius.
                // Therefore, we do not have overlap. If the inner segments overlap, we do not report any collision.
                continue;
            }

            // Compute the planes that goes through the extreme points of the inner segment of capsule 1
            decimal d1 = seg1.dot(capsule1SegA);
            decimal d2 = -seg1.dot(capsule1SegB);

            // Clip the inner segment of capsule 2 with the two planes that go through extreme points of inner
            // segment of capsule 1
            decimal t1 = computePlaneSegmentIntersection(capsule2SegB, capsule2SegA, d1, seg1);
            decimal t2 = computePlaneSegmentIntersection(capsule2SegA, capsule2SegB, d2, -seg1);

            // If the segments were overlapping (the clip segment is valid)
            if (t1 > decimal(0.0) && t2 > decimal(0.0)) {

                if (narrowPhaseInfoBatch.narrowPhaseInfos[batchIndex].reportContacts) {

                    // Clip the inner segment of capsule 2
                    if (t1 > decimal(1.0)) t1 = decimal(1.0);
                    const Vector3 clipPointA = capsule2SegB - t1 * seg2;
                    if (t2 > decimal(1.0)) t2 = decimal(1.0);
                    const Vector3 clipPointB = capsule2SegA + t2 * seg2;

                    // Project point capsule2SegA onto line of innner segment of capsule 1
                    const Vector3 seg1Normalized = seg1.getUnit();
                    Vector3 pointOnInnerSegCapsule1 = capsule1SegA + seg1Normalized.dot(capsule2SegA - capsule1SegA) * seg1Normalized;

                    Vector3 normalCapsule2SpaceNormalized;
                    Vector3 segment1ToSegment2;

                    // If the inner capsule segments perpendicular distance is not zero (the inner segments are not overlapping)
                    if (segmentsPerpendicularDistance > MACHINE_EPSILON) {

                        // Compute a perpendicular vector from segment 1 to segment 2
                        segment1ToSegment2 = (capsule2SegA - pointOnInnerSegCapsule1);
                        normalCapsule2SpaceNormalized = segment1ToSegment2.getUnit();
                    }
                    else {    // If the capsule inner segments are overlapping (degenerate case)

                        // We cannot use the vector between segments as a contact normal. To generate a contact normal, we take
                        // any vector that is orthogonal to the inner capsule segments.

                        Vector3 vec1(1, 0, 0);
                        Vector3 vec2(0, 1, 0);

                        Vector3 seg2Normalized = seg2.getUnit();

                        // Get the vectors (among vec1 and vec2) that is the most orthogonal to the capsule 2 inner segment (smallest absolute dot product)
                        decimal cosA1 = std::abs(seg2Normalized.x);		// abs(vec1.dot(seg2))
                        decimal cosA2 = std::abs(seg2Normalized.y);	    // abs(vec2.dot(seg2))

                        segment1ToSegment2.setToZero();

                        // We choose as a contact normal, any direction that is perpendicular to the inner capsules segments
                        normalCapsule2SpaceNormalized = cosA1 < cosA2 ? seg2Normalized.cross(vec1) : seg2Normalized.cross(vec2);
                    }

                    Transform capsule2ToCapsule1SpaceTransform = capsule1ToCapsule2SpaceTransform.getInverse();
                    const Vector3 contactPointACapsule1Local = capsule2ToCapsule1SpaceTransform * (clipPointA - segment1ToSegment2 + normalCapsule2SpaceNormalized * capsule1Radius);
                    const Vector3 contactPointBCapsule1Local = capsule2ToCapsule1SpaceTransform * (clipPointB - segment1ToSegment2 + normalCapsule2SpaceNormalized * capsule1Radius);
                    const Vector3 contactPointACapsule2Local = clipPointA - normalCapsule2SpaceNormalized * capsule2Radius;
                    const Vector3 contactPointBCapsule2Local = clipPointB - normalCapsule2SpaceNormalized * capsule2Radius;

                    decimal penetrationDepth = sumRadius - segmentsPerpendicularDistance;

                    const Vector3 normalWorld = narrowPhaseInfoBatch.narrowPhaseInfos[batchIndex].shape2ToWorldTransform.getOrientation() * normalCapsule2SpaceNormalized;

                    // Create the contact info object
                    narrowPhaseInfoBatch.addContactPoint(batchIndex, normalWorld, penetrationDepth, contactPointACapsule1Local, contactPointACapsule2Local);
                    narrowPhaseInfoBatch.addContactPoint(batchIndex, normalWorld, penetrationDepth, contactPointBCapsule1Local, contactPointBCapsule2Local);
                }

                narrowPhaseInfoBatch.narrowPhaseInfos[batchIndex].isColliding = true;
                isCollisionFound = true;
                continue;
            }
        }

        // Compute the closest points between the two inner capsule segments
        Vector3 closestPointCapsule1Seg;
        Vector3 closestPointCapsule2Seg;
        computeClosestPointBetweenTwoSegments(capsule1SegA, capsule1SegB, capsule2SegA, capsule2SegB,
                                              closestPointCapsule1Seg, closestPointCapsule2Seg);

        // Compute the distance between the sphere center and the closest point on the segment
        Vector3 closestPointsSeg1ToSeg2 = (closestPointCapsule2Seg - closestPointCapsule1Seg);
        const decimal closestPointsDistanceSquare = closestPointsSeg1ToSeg2.lengthSquare();

        // If the collision shapes overlap
        if (closestPointsDistanceSquare < sumRadius * sumRadius) {

            if (narrowPhaseInfoBatch.narrowPhaseInfos[batchIndex].reportContacts) {

                // If the distance between the inner segments is not zero
                if (closestPointsDistanceSquare > MACHINE_EPSILON) {

                    decimal closestPointsDistance = std::sqrt(closestPointsDistanceSquare);
                    closestPointsSeg1ToSeg2 /= closestPointsDistance;

                    const Vector3 contactPointCapsule1Local = capsule1ToCapsule2SpaceTransform.getInverse() * (closestPointCapsule1Seg + closestPointsSeg1ToSeg2 * capsule1Radius);
                    const Vector3 contactPointCapsule2Local = closestPointCapsule2Seg - closestPointsSeg1ToSeg2 * capsule2Radius;

                    const Vector3 normalWorld = narrowPhaseInfoBatch.narrowPhaseInfos[batchIndex].shape2ToWorldTransform.getOrientation() * closestPointsSeg1ToSeg2;

                    decimal penetrationDepth = sumRadius - closestPointsDistance;

                    // Create the contact info object
                    narrowPhaseInfoBatch.addContactPoint(batchIndex, normalWorld, penetrationDepth, contactPointCapsule1Local, contactPointCapsule2Local);
                }
                else { // The segment are overlapping (degenerate case)

                    // If the capsule segments are parralel
                    if (areCapsuleInnerSegmentsParralel) {

                        // The segment are parallel, not overlapping and their distance is zero.
                        // Therefore, the capsules are just touching at the top of their inner segments
                        decimal squareDistCapsule2PointToCapsuleSegA = (capsule1SegA - closestPointCapsule2Seg).lengthSquare();

                        Vector3 capsule1SegmentMostExtremePoint = squareDistCapsule2PointToCapsuleSegA > MACHINE_EPSILON ? capsule1SegA : capsule1SegB;
                        Vector3 normalCapsuleSpace2 = (closestPointCapsule2Seg - capsule1SegmentMostExtremePoint);
                        normalCapsuleSpace2.normalize();

                        const Vector3 contactPointCapsule1Local = capsule1ToCapsule2SpaceTransform.getInverse() * (closestPointCapsule1Seg + normalCapsuleSpace2 * capsule1Radius);
                        const Vector3 contactPointCapsule2Local = closestPointCapsule2Seg - normalCapsuleSpace2 * capsule2Radius;

                        const Vector3 normalWorld = narrowPhaseInfoBatch.narrowPhaseInfos[batchIndex].shape2ToWorldTransform.getOrientation() * normalCapsuleSpace2;

                        // Create the contact info object
                        narrowPhaseInfoBatch.addContactPoint(batchIndex, normalWorld, sumRadius, contactPointCapsule1Local, contactPointCapsule2Local);
                    }
                    else {   // If the capsules inner segments are not parallel

                        // We cannot use a vector between the segments as contact normal. We need to compute a new contact normal with the cross
                        // product between the two segments.
                        Vector3 normalCapsuleSpace2 = seg1.cross(seg2);
                        normalCapsuleSpace2.normalize();

                        // Compute the contact points on both shapes
                        const Vector3 contactPointCapsule1Local = capsule1ToCapsule2SpaceTransform.getInverse() * (closestPointCapsule1Seg + normalCapsuleSpace2 * capsule1Radius);
                        const Vector3 contactPointCapsule2Local = closestPointCapsule2Seg - normalCapsuleSpace2 * capsule2Radius;

                        const Vector3 normalWorld = narrowPhaseInfoBatch.narrowPhaseInfos[batchIndex].shape2ToWorldTransform.getOrientation() * normalCapsuleSpace2;

                        // Create the contact info object
                        narrowPhaseInfoBatch.addContactPoint(batchIndex, normalWorld, sumRadius, contactPointCapsule1Local, contactPointCapsule2Local);
                    }
                }
            }

            narrowPhaseInfoBatch.narrowPhaseInfos[batchIndex].isColliding = true;
            isCollisionFound = true;
        }
    }

    return isCollisionFound;
}
