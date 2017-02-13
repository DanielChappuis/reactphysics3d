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
#include "SphereVsCapsuleAlgorithm.h"
#include "collision/shapes/SphereShape.h"
#include "collision/shapes/CapsuleShape.h"

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;  

bool SphereVsCapsuleAlgorithm::testCollision(const NarrowPhaseInfo* narrowPhaseInfo, ContactPointInfo& contactPointInfo) {
    
    // Get the collision shapes
    const SphereShape* sphereShape = static_cast<const SphereShape*>(narrowPhaseInfo->collisionShape1);
    const CapsuleShape* capsuleShape = static_cast<const CapsuleShape*>(narrowPhaseInfo->collisionShape2);

    // Get the transform from sphere local-space to capsule local-space
	const Transform sphereToCapsuleSpaceTransform = narrowPhaseInfo->shape1ToWorldTransform * narrowPhaseInfo->shape2ToWorldTransform.getInverse();

	// Transform the center of the sphere into the local-space of the capsule shape
	const Vector3 sphereCenter = sphereToCapsuleSpaceTransform.getPosition();

	// Compute the end-points of the inner segment of the capsule
	const Vector3 capsuleSegA(0, -capsuleShape->getHeight() * decimal(0.5), 0);
	const Vector3 capsuleSegB(0, capsuleShape->getHeight() * decimal(0.5), 0);

    // Compute the point on the inner capsule segment that is the closes to center of sphere
	const Vector3 closestPointOnSegment = computeClosestPointOnSegment(capsuleSegA, capsuleSegB, sphereCenter);

	// Compute the distance between the sphere center and the closest point on the segment
	Vector3 sphereCenterToSegment = (closestPointOnSegment - sphereCenter);
	const decimal sphereSegmentDistanceSquare = sphereCenterToSegment.lengthSquare();

    // Compute the sum of the radius of the sphere and the capsule (virtual sphere)
    decimal sumRadius = sphereShape->getRadius() + capsuleShape->getRadius();
    
    // If the collision shapes overlap
    if (sphereSegmentDistanceSquare <= sumRadius * sumRadius && sphereSegmentDistanceSquare > MACHINE_EPSILON) {

		decimal sphereSegmentDistance = std::sqrt(sphereSegmentDistanceSquare);
		sphereCenterToSegment /= sphereSegmentDistance;

		const Vector3 contactPointSphereLocal = sphereToCapsuleSpaceTransform.getInverse() * (sphereCenter + sphereCenterToSegment * sphereShape->getRadius());
		const Vector3 contactPointCapsuleLocal = closestPointOnSegment - sphereCenterToSegment * capsuleShape->getRadius();
		
		const Vector3 normalWorld = narrowPhaseInfo->shape2ToWorldTransform.getOrientation() * sphereCenterToSegment;
       
        decimal penetrationDepth = sumRadius - sphereSegmentDistance;
        
        // Create the contact info object
        contactPointInfo.init(normalWorld, penetrationDepth, contactPointSphereLocal, contactPointCapsuleLocal);

        return true;
    }

    return false;
}
