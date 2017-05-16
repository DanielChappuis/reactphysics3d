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
#include "CapsuleVsCapsuleAlgorithm.h"
#include "collision/shapes/CapsuleShape.h"

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;  

// Compute the narrow-phase collision detection between two capsules
// This technique is based on the "Robust Contact Creation for Physics Simulations" presentation
// by Dirk Gregorius.
bool CapsuleVsCapsuleAlgorithm::testCollision(const NarrowPhaseInfo* narrowPhaseInfo, ContactManifoldInfo& contactManifoldInfo) {
    
    assert(narrowPhaseInfo->collisionShape1->getType() == CollisionShapeType::CAPSULE);
    assert(narrowPhaseInfo->collisionShape2->getType() == CollisionShapeType::CAPSULE);

    // Get the capsule collision shapes
    const CapsuleShape* capsuleShape1 = static_cast<const CapsuleShape*>(narrowPhaseInfo->collisionShape1);
    const CapsuleShape* capsuleShape2 = static_cast<const CapsuleShape*>(narrowPhaseInfo->collisionShape2);

	// Get the transform from capsule 1 local-space to capsule 2 local-space
    const Transform capsule1ToCapsule2SpaceTransform = narrowPhaseInfo->shape2ToWorldTransform.getInverse() * narrowPhaseInfo->shape1ToWorldTransform;

	// Compute the end-points of the inner segment of the first capsule
	Vector3 capsule1SegA(0, -capsuleShape1->getHeight() * decimal(0.5), 0);
	Vector3 capsule1SegB(0, capsuleShape1->getHeight() * decimal(0.5), 0);
	capsule1SegA = capsule1ToCapsule2SpaceTransform * capsule1SegA;
	capsule1SegB = capsule1ToCapsule2SpaceTransform * capsule1SegB;

	// Compute the end-points of the inner segment of the second capsule
	const Vector3 capsule2SegA(0, -capsuleShape2->getHeight() * decimal(0.5), 0);
	const Vector3 capsule2SegB(0, capsuleShape2->getHeight() * decimal(0.5), 0);
	
	// The two inner capsule segments
	const Vector3 seg1 = capsule1SegB - capsule1SegA;
	const Vector3 seg2 = capsule2SegB - capsule2SegA;

	// Compute the sum of the radius of the two capsules (virtual spheres)
	decimal sumRadius = capsuleShape2->getRadius() + capsuleShape1->getRadius();

	// If the two capsules are parallel (we create two contact points)
    if (areParallelVectors(seg1, seg2)) {

		// If the distance between the two segments is larger than the sum of the capsules radius (we do not have overlapping)
		const decimal segmentsDistance = computePointToLineDistance(capsule1SegA, capsule1SegB, capsule2SegA);
        if (segmentsDistance >= sumRadius) {

			// The capsule are parallel but their inner segment distance is larger than the sum of the capsules radius.
			// Therefore, we do not have overlap. If the inner segments overlap, we do not report any collision.
			return false;
		}

		// If the distance between the two segments is larger than zero (inner segments of capsules are not overlapping)
		// If the inner segments are overlapping, we cannot compute a contact normal (unknown direction). In this case,
		// we skip the parallel contact points calculation (there might still be contact in the spherical caps of the capsules)
		if (segmentsDistance > MACHINE_EPSILON) {

			// Compute the planes that goes through the extreme points of the inner segment of capsule 1
			decimal d1 = seg1.dot(capsule1SegA);
			decimal d2 = -seg1.dot(capsule1SegB);

			// Clip the inner segment of capsule 2 with the two planes that go through extreme points of inner
			// segment of capsule 1
			decimal t1 = computePlaneSegmentIntersection(capsule2SegB, capsule2SegA, d1, seg1);
			decimal t2 = computePlaneSegmentIntersection(capsule2SegA, capsule2SegB, d2, -seg1);

			// If the segments were overlapping (the clip segment is valid)
			if (t1 > decimal(0.0) && t2 > decimal(0.0)) {

				// Clip the inner segment of capsule 2
				if (t1 > decimal(1.0)) t1 = decimal(1.0);
				const Vector3 clipPointA = capsule2SegB - t1 * seg2;
				if (t2 > decimal(1.0)) t2 = decimal(1.0);
				const Vector3 clipPointB = capsule2SegA + t2 * seg2;

				// Project point capsule2SegA onto line of innner segment of capsule 1
				const Vector3 seg1Normalized = seg1.getUnit();
				Vector3 pointOnInnerSegCapsule1 = capsule1SegA + seg1Normalized.dot(capsule2SegA - capsule1SegA) * seg1Normalized;

				// Compute a perpendicular vector from segment 1 to segment 2
				Vector3 segment1ToSegment2 = (capsule2SegA - pointOnInnerSegCapsule1);
				Vector3 segment1ToSegment2Normalized = segment1ToSegment2.getUnit();

				Transform capsule2ToCapsule1SpaceTransform = capsule1ToCapsule2SpaceTransform.getInverse();
				const Vector3 contactPointACapsule1Local = capsule2ToCapsule1SpaceTransform * (clipPointA - segment1ToSegment2 + segment1ToSegment2Normalized * capsuleShape1->getRadius());
				const Vector3 contactPointBCapsule1Local = capsule2ToCapsule1SpaceTransform * (clipPointB - segment1ToSegment2 + segment1ToSegment2Normalized * capsuleShape1->getRadius());
				const Vector3 contactPointACapsule2Local = clipPointA - segment1ToSegment2Normalized * capsuleShape2->getRadius();
				const Vector3 contactPointBCapsule2Local = clipPointB - segment1ToSegment2Normalized * capsuleShape2->getRadius();

				const Vector3 normalWorld = narrowPhaseInfo->shape2ToWorldTransform.getOrientation() * segment1ToSegment2Normalized;

				decimal penetrationDepth = sumRadius - segmentsDistance;

				// Create the contact info object
				contactManifoldInfo.addContactPoint(normalWorld, penetrationDepth, contactPointACapsule1Local, contactPointACapsule2Local);
				contactManifoldInfo.addContactPoint(normalWorld, penetrationDepth, contactPointBCapsule1Local, contactPointBCapsule2Local);

				return true;
			}
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
    if (closestPointsDistanceSquare < sumRadius * sumRadius && closestPointsDistanceSquare > MACHINE_EPSILON) {

		decimal closestPointsDistance = std::sqrt(closestPointsDistanceSquare);
		closestPointsSeg1ToSeg2 /= closestPointsDistance;

		const Vector3 contactPointCapsule1Local = capsule1ToCapsule2SpaceTransform.getInverse() * (closestPointCapsule1Seg + closestPointsSeg1ToSeg2 * capsuleShape1->getRadius());
		const Vector3 contactPointCapsule2Local = closestPointCapsule2Seg - closestPointsSeg1ToSeg2 * capsuleShape2->getRadius();

		const Vector3 normalWorld = narrowPhaseInfo->shape2ToWorldTransform.getOrientation() * closestPointsSeg1ToSeg2;

		decimal penetrationDepth = sumRadius - closestPointsDistance;

		// Create the contact info object
        contactManifoldInfo.addContactPoint(normalWorld, penetrationDepth, contactPointCapsule1Local, contactPointCapsule2Local);

		return true;
	}

	return false;
}
