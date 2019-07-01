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
#include "CapsuleVsConvexPolyhedronAlgorithm.h"
#include "SAT/SATAlgorithm.h"
#include "GJK/GJKAlgorithm.h"
#include "collision/shapes/CapsuleShape.h"
#include "collision/shapes/ConvexPolyhedronShape.h"
#include "collision/NarrowPhaseInfo.h"
#include "collision/ContactPointInfo.h"
#include <cassert>

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Compute the narrow-phase collision detection between a capsule and a polyhedron
// This technique is based on the "Robust Contact Creation for Physics Simulations" presentation
// by Dirk Gregorius.
bool CapsuleVsConvexPolyhedronAlgorithm::testCollision(NarrowPhaseInfo* narrowPhaseInfo, bool reportContacts,
                                                       MemoryAllocator& memoryAllocator) {

    // First, we run the GJK algorithm
    GJKAlgorithm gjkAlgorithm;
    SATAlgorithm satAlgorithm(memoryAllocator);

#ifdef IS_PROFILING_ACTIVE

	gjkAlgorithm.setProfiler(mProfiler);
	satAlgorithm.setProfiler(mProfiler);

#endif

    // Get the last frame collision info
    LastFrameCollisionInfo* lastFrameCollisionInfo = narrowPhaseInfo->getLastFrameCollisionInfo();

    GJKAlgorithm::GJKResult result = gjkAlgorithm.testCollision(narrowPhaseInfo, reportContacts);

    lastFrameCollisionInfo->wasUsingGJK = true;
    lastFrameCollisionInfo->wasUsingSAT = false;

	assert(narrowPhaseInfo->collisionShape1->getType() == CollisionShapeType::CONVEX_POLYHEDRON ||
		   narrowPhaseInfo->collisionShape2->getType() == CollisionShapeType::CONVEX_POLYHEDRON);
	assert(narrowPhaseInfo->collisionShape1->getType() == CollisionShapeType::CAPSULE ||
		   narrowPhaseInfo->collisionShape2->getType() == CollisionShapeType::CAPSULE);

    // If we have found a contact point inside the margins (shallow penetration)
    if (result == GJKAlgorithm::GJKResult::COLLIDE_IN_MARGIN) {

        if (reportContacts) {

            // GJK has found a shallow contact. If the face of the polyhedron mesh is orthogonal to the
            // capsule inner segment and parallel to the contact point normal, we would like to create
            // two contact points instead of a single one (as in the deep contact case with SAT algorithm)

            // Get the contact point created by GJK
            ContactPointInfo* contactPoint = narrowPhaseInfo->contactPoints;
            assert(contactPoint != nullptr);

            bool isCapsuleShape1 = narrowPhaseInfo->collisionShape1->getType() == CollisionShapeType::CAPSULE;

            // Get the collision shapes
            const CapsuleShape* capsuleShape = static_cast<const CapsuleShape*>(isCapsuleShape1 ? narrowPhaseInfo->collisionShape1 : narrowPhaseInfo->collisionShape2);
            const ConvexPolyhedronShape* polyhedron = static_cast<const ConvexPolyhedronShape*>(isCapsuleShape1 ? narrowPhaseInfo->collisionShape2 : narrowPhaseInfo->collisionShape1);

            // For each face of the polyhedron
            for (uint f = 0; f < polyhedron->getNbFaces(); f++) {

                const Transform polyhedronToWorld = isCapsuleShape1 ? narrowPhaseInfo->shape2ToWorldTransform : narrowPhaseInfo->shape1ToWorldTransform;
                const Transform capsuleToWorld = isCapsuleShape1 ? narrowPhaseInfo->shape1ToWorldTransform : narrowPhaseInfo->shape2ToWorldTransform;

                // Get the face normal
                const Vector3 faceNormal = polyhedron->getFaceNormal(f);
                Vector3 faceNormalWorld = polyhedronToWorld.getOrientation() * faceNormal;

                const Vector3 capsuleSegA(0, -capsuleShape->getHeight() * decimal(0.5), 0);
                const Vector3 capsuleSegB(0, capsuleShape->getHeight() * decimal(0.5), 0);
                Vector3 capsuleInnerSegmentDirection = capsuleToWorld.getOrientation() * (capsuleSegB - capsuleSegA);
                capsuleInnerSegmentDirection.normalize();

                bool isFaceNormalInDirectionOfContactNormal = faceNormalWorld.dot(contactPoint->normal) > decimal(0.0);
                bool isFaceNormalInContactDirection = (isCapsuleShape1 && !isFaceNormalInDirectionOfContactNormal) || (!isCapsuleShape1 && isFaceNormalInDirectionOfContactNormal);

                // If the polyhedron face normal is orthogonal to the capsule inner segment and parallel to the contact point normal and the face normal
                // is in direction of the contact normal (from the polyhedron point of view).
                if (isFaceNormalInContactDirection && areOrthogonalVectors(faceNormalWorld, capsuleInnerSegmentDirection)
                    && areParallelVectors(faceNormalWorld, contactPoint->normal)) {

                    // Remove the previous contact point computed by GJK
                    narrowPhaseInfo->resetContactPoints();

                    const Transform capsuleToWorld = isCapsuleShape1 ? narrowPhaseInfo->shape1ToWorldTransform : narrowPhaseInfo->shape2ToWorldTransform;
                    const Transform polyhedronToCapsuleTransform = capsuleToWorld.getInverse() * polyhedronToWorld;

                    // Compute the end-points of the inner segment of the capsule
                    const Vector3 capsuleSegA(0, -capsuleShape->getHeight() * decimal(0.5), 0);
                    const Vector3 capsuleSegB(0, capsuleShape->getHeight() * decimal(0.5), 0);

                    // Convert the inner capsule segment points into the polyhedron local-space
                    const Transform capsuleToPolyhedronTransform = polyhedronToCapsuleTransform.getInverse();
                    const Vector3 capsuleSegAPolyhedronSpace = capsuleToPolyhedronTransform * capsuleSegA;
                    const Vector3 capsuleSegBPolyhedronSpace = capsuleToPolyhedronTransform * capsuleSegB;

                    const Vector3 separatingAxisCapsuleSpace = polyhedronToCapsuleTransform.getOrientation() * faceNormal;

                    if (isCapsuleShape1) {
                        faceNormalWorld = -faceNormalWorld;
                    }

                    // Compute and create two contact points
                    bool contactsFound = satAlgorithm.computeCapsulePolyhedronFaceContactPoints(f, capsuleShape->getRadius(), polyhedron, contactPoint->penetrationDepth,
                                                              polyhedronToCapsuleTransform, faceNormalWorld, separatingAxisCapsuleSpace,
                                                              capsuleSegAPolyhedronSpace, capsuleSegBPolyhedronSpace,
                                                              narrowPhaseInfo, isCapsuleShape1);
                    if (!contactsFound) {
                        return false;
                    }

                    break;
                }
            }
        }

        lastFrameCollisionInfo->wasUsingSAT = false;
        lastFrameCollisionInfo->wasUsingGJK = false;

        // Return true
        return true;
    }

    // If we have overlap even without the margins (deep penetration)
    if (result == GJKAlgorithm::GJKResult::INTERPENETRATE) {

        // Run the SAT algorithm to find the separating axis and compute contact point
        bool isColliding = satAlgorithm.testCollisionCapsuleVsConvexPolyhedron(narrowPhaseInfo, reportContacts);

        lastFrameCollisionInfo->wasUsingGJK = false;
        lastFrameCollisionInfo->wasUsingSAT = true;

        return isColliding;
    }

    return false;
}
