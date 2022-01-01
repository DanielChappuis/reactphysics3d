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
#include <reactphysics3d/collision/narrowphase/GJK/GJKAlgorithm.h>
#include <reactphysics3d/constraint/ContactPoint.h>
#include <reactphysics3d/engine/OverlappingPairs.h>
#include <reactphysics3d/collision/shapes/TriangleShape.h>
#include <reactphysics3d/configuration.h>
#include <reactphysics3d/utils/Profiler.h>
#include <reactphysics3d/containers/Array.h>
#include <reactphysics3d/collision/narrowphase/NarrowPhaseInfoBatch.h>
#include <reactphysics3d/collision/narrowphase/GJK/VoronoiSimplex.h>
#include <cassert>

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Compute a contact info if the two collision shapes collide.
/// This method implements the Hybrid Technique for computing the penetration depth by
/// running the GJK algorithm on original objects (without margin). If the shapes intersect
/// only in the margins, the method compute the penetration depth and contact points
/// (of enlarged objects). If the original objects (without margin) intersect, we
/// call the computePenetrationDepthForEnlargedObjects() method that run the GJK
/// algorithm on the enlarged object to obtain a simplex polytope that contains the
/// origin, they we give that simplex polytope to the EPA algorithm which will compute
/// the correct penetration depth and contact points between the enlarged objects.
void GJKAlgorithm::testCollision(NarrowPhaseInfoBatch& narrowPhaseInfoBatch, uint32 batchStartIndex,
                                 uint32 batchNbItems, Array<GJKResult>& gjkResults) {

    RP3D_PROFILE("GJKAlgorithm::testCollision()", mProfiler);
    
    // For each item in the batch
    for (uint32 batchIndex = batchStartIndex; batchIndex < batchStartIndex + batchNbItems; batchIndex++) {

        Vector3 suppA;             // Support point of object A
        Vector3 suppB;             // Support point of object B
        Vector3 w;                 // Support point of Minkowski difference A-B
        Vector3 pA;                // Closest point of object A
        Vector3 pB;                // Closest point of object B
        decimal vDotw;
        decimal prevDistSquare;
        bool contactFound = false;

        assert(narrowPhaseInfoBatch.narrowPhaseInfos[batchIndex].collisionShape1->isConvex());
        assert(narrowPhaseInfoBatch.narrowPhaseInfos[batchIndex].collisionShape2->isConvex());

        const ConvexShape* shape1 = static_cast<const ConvexShape*>(narrowPhaseInfoBatch.narrowPhaseInfos[batchIndex].collisionShape1);
        const ConvexShape* shape2 = static_cast<const ConvexShape*>(narrowPhaseInfoBatch.narrowPhaseInfos[batchIndex].collisionShape2);

        // Get the local-space to world-space transforms
        const Transform& transform1 = narrowPhaseInfoBatch.narrowPhaseInfos[batchIndex].shape1ToWorldTransform;
        const Transform& transform2 = narrowPhaseInfoBatch.narrowPhaseInfos[batchIndex].shape2ToWorldTransform;

        // Transform a point from local space of body 2 to local
        // space of body 1 (the GJK algorithm is done in local space of body 1)
        Transform transform1Inverse = transform1.getInverse();
        Transform body2Tobody1 = transform1Inverse * transform2;

        // Quaternion that transform a direction from local
        // space of body 1 into local space of body 2
        Quaternion rotateToBody2 = transform2.getOrientation().getInverse() * transform1.getOrientation();

        // Initialize the margin (sum of margins of both objects)
        decimal margin = shape1->getMargin() + shape2->getMargin();
        decimal marginSquare = margin * margin;
        assert(margin > decimal(0.0));

        // Create a simplex set
        VoronoiSimplex simplex;

        // Get the last collision frame info
        LastFrameCollisionInfo* lastFrameCollisionInfo = narrowPhaseInfoBatch.narrowPhaseInfos[batchIndex].lastFrameCollisionInfo;

        // Get the previous point V (last cached separating axis)
        Vector3 v;
        if (lastFrameCollisionInfo->isValid && lastFrameCollisionInfo->wasUsingGJK) {
            v = lastFrameCollisionInfo->gjkSeparatingAxis;
            assert(v.lengthSquare() > decimal(0.000001));
        }
        else {
            v.setAllValues(0, 1, 0);
        }

        // Initialize the upper bound for the square distance
        decimal distSquare = DECIMAL_LARGEST;

        bool noIntersection = false;

        do {

            // Compute the support points for original objects (without margins) A and B
            suppA = shape1->getLocalSupportPointWithoutMargin(-v);
            suppB = body2Tobody1 * shape2->getLocalSupportPointWithoutMargin(rotateToBody2 * v);

            // Compute the support point for the Minkowski difference A-B
            w = suppA - suppB;

            vDotw = v.dot(w);

            // If the enlarge objects (with margins) do not intersect
            if (vDotw > decimal(0.0) && vDotw * vDotw > distSquare * marginSquare) {

                // Cache the current separating axis for frame coherence
                lastFrameCollisionInfo->gjkSeparatingAxis = v;

                // No intersection, we return
                assert(gjkResults.size() == batchIndex);
                gjkResults.add(GJKResult::SEPARATED);
                noIntersection = true;
                break;
            }

            // If the objects intersect only in the margins
            if (simplex.isPointInSimplex(w) || distSquare - vDotw <= distSquare * REL_ERROR_SQUARE) {

                // Contact point has been found
                contactFound = true;
                break;
            }

            // Add the new support point to the simplex
            simplex.addPoint(w, suppA, suppB);

            // If the simplex is affinely dependent
            if (simplex.isAffinelyDependent()) {

                // Contact point has been found
                contactFound = true;
                break;
            }

            // Compute the point of the simplex closest to the origin
            // If the computation of the closest point fails
            if (!simplex.computeClosestPoint(v)) {

                // Contact point has been found
                contactFound = true;
                break;
            }

            // Store and update the squared distance of the closest point
            prevDistSquare = distSquare;
            distSquare = v.lengthSquare();

            // If the distance to the closest point doesn't improve a lot
            if (prevDistSquare - distSquare <= MACHINE_EPSILON * prevDistSquare) {

                simplex.backupClosestPointInSimplex(v);

                // Get the new squared distance
                distSquare = v.lengthSquare();

                // Contact point has been found
                contactFound = true;
                break;
            }

        } while(!simplex.isFull() && distSquare > MACHINE_EPSILON * simplex.getMaxLengthSquareOfAPoint());

        if (noIntersection) {
            continue;
        }

        if (contactFound && distSquare > MACHINE_EPSILON) {

            // Compute the closet points of both objects (without the margins)
            simplex.computeClosestPointsOfAandB(pA, pB);

            // Project those two points on the margins to have the closest points of both
            // object with the margins
            decimal dist = std::sqrt(distSquare);
            assert(dist > decimal(0.0));
            pA = (pA - (shape1->getMargin() / dist) * v);
            pB = body2Tobody1.getInverse() * (pB + (shape2->getMargin() / dist) * v);

            // Compute the contact info
            Vector3 normal = transform1.getOrientation() * (-v.getUnit());
            decimal penetrationDepth = margin - dist;

            // If the penetration depth is negative (due too numerical errors), there is no contact
            if (penetrationDepth <= decimal(0.0)) {
                assert(gjkResults.size() == batchIndex);
                gjkResults.add(GJKResult::SEPARATED);
                continue;
            }

            // Do not generate a contact point with zero normal length
            if (normal.lengthSquare() < MACHINE_EPSILON) {
                assert(gjkResults.size() == batchIndex);
                gjkResults.add(GJKResult::SEPARATED);
                continue;
            }

            // If we need to report contacts
            if (narrowPhaseInfoBatch.narrowPhaseInfos[batchIndex].reportContacts) {

                // Compute smooth triangle mesh contact if one of the two collision shapes is a triangle
                TriangleShape::computeSmoothTriangleMeshContact(shape1, shape2, pA, pB, transform1, transform2,
                                                                penetrationDepth, normal);

                // Add a new contact point
                narrowPhaseInfoBatch.addContactPoint(batchIndex, normal, penetrationDepth, pA, pB);
            }

            assert(gjkResults.size() == batchIndex);
            gjkResults.add(GJKResult::COLLIDE_IN_MARGIN);

            continue;
        }

        assert(gjkResults.size() == batchIndex);
        gjkResults.add(GJKResult::INTERPENETRATE);
    }
}
