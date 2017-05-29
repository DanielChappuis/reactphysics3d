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

// Compute the narrow-phase collision detection between a sphere and a capsule
// This technique is based on the "Robust Contact Creation for Physics Simulations" presentation
// by Dirk Gregorius.
bool SphereVsCapsuleAlgorithm::testCollision(const NarrowPhaseInfo* narrowPhaseInfo, ContactManifoldInfo& contactManifoldInfo) {

    bool isSphereShape1 = narrowPhaseInfo->collisionShape1->getType() == CollisionShapeType::SPHERE;

    assert(isSphereShape1 || narrowPhaseInfo->collisionShape1->getType() == CollisionShapeType::CAPSULE);

    // Get the collision shapes
    const SphereShape* sphereShape = static_cast<const SphereShape*>(isSphereShape1 ? narrowPhaseInfo->collisionShape1 : narrowPhaseInfo->collisionShape2);
    const CapsuleShape* capsuleShape = static_cast<const CapsuleShape*>(isSphereShape1 ? narrowPhaseInfo->collisionShape2 : narrowPhaseInfo->collisionShape1);

    // Get the transform from sphere local-space to capsule local-space
    const Transform& sphereToWorldTransform = isSphereShape1 ? narrowPhaseInfo->shape1ToWorldTransform : narrowPhaseInfo->shape2ToWorldTransform;
    const Transform& capsuleToWorldTransform = isSphereShape1 ? narrowPhaseInfo->shape2ToWorldTransform : narrowPhaseInfo->shape1ToWorldTransform;
    const Transform worldToCapsuleTransform = capsuleToWorldTransform.getInverse();
    const Transform sphereToCapsuleSpaceTransform = worldToCapsuleTransform * sphereToWorldTransform;

	// Transform the center of the sphere into the local-space of the capsule shape
	const Vector3 sphereCenter = sphereToCapsuleSpaceTransform.getPosition();

	// Compute the end-points of the inner segment of the capsule
    const decimal capsuleHalfHeight = capsuleShape->getHeight() * decimal(0.5);
    const Vector3 capsuleSegA(0, -capsuleHalfHeight, 0);
    const Vector3 capsuleSegB(0, capsuleHalfHeight, 0);

    // Compute the point on the inner capsule segment that is the closes to center of sphere
	const Vector3 closestPointOnSegment = computeClosestPointOnSegment(capsuleSegA, capsuleSegB, sphereCenter);

	// Compute the distance between the sphere center and the closest point on the segment
	Vector3 sphereCenterToSegment = (closestPointOnSegment - sphereCenter);
	const decimal sphereSegmentDistanceSquare = sphereCenterToSegment.lengthSquare();

    // Compute the sum of the radius of the sphere and the capsule (virtual sphere)
    decimal sumRadius = sphereShape->getRadius() + capsuleShape->getRadius();
    
    // If the collision shapes overlap
    if (sphereSegmentDistanceSquare < sumRadius * sumRadius && sphereSegmentDistanceSquare > MACHINE_EPSILON) {

		decimal sphereSegmentDistance = std::sqrt(sphereSegmentDistanceSquare);
		sphereCenterToSegment /= sphereSegmentDistance;

		const Vector3 contactPointSphereLocal = sphereToCapsuleSpaceTransform.getInverse() * (sphereCenter + sphereCenterToSegment * sphereShape->getRadius());
		const Vector3 contactPointCapsuleLocal = closestPointOnSegment - sphereCenterToSegment * capsuleShape->getRadius();
		
        Vector3 normalWorld = capsuleToWorldTransform.getOrientation() * sphereCenterToSegment;
       
        decimal penetrationDepth = sumRadius - sphereSegmentDistance;

        if (!isSphereShape1) {
            normalWorld = -normalWorld;
        }
        
        // Create the contact info object
        contactManifoldInfo.addContactPoint(normalWorld, penetrationDepth,
                                            isSphereShape1 ? contactPointSphereLocal : contactPointCapsuleLocal,
                                            isSphereShape1 ? contactPointCapsuleLocal : contactPointSphereLocal);

        return true;
    }

    return false;
}
