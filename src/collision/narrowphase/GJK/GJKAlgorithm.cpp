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
#include "GJKAlgorithm.h"
#include "constraint/ContactPoint.h"
#include "engine/OverlappingPair.h"
#include "collision/shapes/TriangleShape.h"
#include "configuration.h"
#include "engine/Profiler.h"
#include <algorithm>
#include <cmath>
#include <cfloat>
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
GJKAlgorithm::GJKResult GJKAlgorithm::testCollision(NarrowPhaseInfo* narrowPhaseInfo, bool reportContacts) {

    PROFILE("GJKAlgorithm::testCollision()", mProfiler);
    
    Vector3 suppA;             // Support point of object A
    Vector3 suppB;             // Support point of object B
    Vector3 w;                 // Support point of Minkowski difference A-B
    Vector3 pA;                // Closest point of object A
    Vector3 pB;                // Closest point of object B
    decimal vDotw;
    decimal prevDistSquare;
    bool contactFound = false;

    assert(narrowPhaseInfo->collisionShape1->isConvex());
    assert(narrowPhaseInfo->collisionShape2->isConvex());

    const ConvexShape* shape1 = static_cast<const ConvexShape*>(narrowPhaseInfo->collisionShape1);
    const ConvexShape* shape2 = static_cast<const ConvexShape*>(narrowPhaseInfo->collisionShape2);

    // Get the local-space to world-space transforms
    const Transform& transform1 = narrowPhaseInfo->shape1ToWorldTransform;
    const Transform& transform2 = narrowPhaseInfo->shape2ToWorldTransform;

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
    LastFrameCollisionInfo* lastFrameCollisionInfo = narrowPhaseInfo->getLastFrameCollisionInfo();

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
            return GJKResult::SEPARATED;
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
            return GJKResult::SEPARATED;
        }

        // Do not generate a contact point with zero normal length
        if (normal.lengthSquare() < MACHINE_EPSILON) {
            return GJKResult::SEPARATED;
        }

        if (reportContacts) {

            // Compute smooth triangle mesh contact if one of the two collision shapes is a triangle
            TriangleShape::computeSmoothTriangleMeshContact(shape1, shape2, pA, pB, transform1, transform2,
                                                            penetrationDepth, normal);

            // Add a new contact point
            narrowPhaseInfo->addContactPoint(normal, penetrationDepth, pA, pB);
        }

        return GJKResult::COLLIDE_IN_MARGIN;
    }

    return GJKResult::INTERPENETRATE;
}


// Use the GJK Algorithm to find if a point is inside a convex collision shape
bool GJKAlgorithm::testPointInside(const Vector3& localPoint, ProxyShape* proxyShape) {

    Vector3 suppA;             // Support point of object A
    Vector3 w;                 // Support point of Minkowski difference A-B
    decimal prevDistSquare;

    assert(proxyShape->getCollisionShape()->isConvex());

    const ConvexShape* shape = static_cast<const ConvexShape*>(proxyShape->getCollisionShape());

    // Support point of object B (object B is a single point)
    const Vector3 suppB(localPoint);

    // Create a simplex set
    VoronoiSimplex simplex;

    // Initial supporting direction
    Vector3 v(1, 1, 1);

    // Initialize the upper bound for the square distance
    decimal distSquare = DECIMAL_LARGEST;

    do {

        // Compute the support points for original objects (without margins) A and B
        suppA = shape->getLocalSupportPointWithoutMargin(-v);

        // Compute the support point for the Minkowski difference A-B
        w = suppA - suppB;

        // Add the new support point to the simplex
        simplex.addPoint(w, suppA, suppB);

        // If the simplex is affinely dependent
        if (simplex.isAffinelyDependent()) {

            return false;
        }

        // Compute the point of the simplex closest to the origin
        // If the computation of the closest point fail
        if (!simplex.computeClosestPoint(v)) {

            return false;
        }

        // Store and update the squared distance of the closest point
        prevDistSquare = distSquare;
        distSquare = v.lengthSquare();

        // If the distance to the closest point doesn't improve a lot
        if (prevDistSquare - distSquare <= MACHINE_EPSILON * prevDistSquare) {

            return false;
        }
    } while(!simplex.isFull() && distSquare > MACHINE_EPSILON *
                                 simplex.getMaxLengthSquareOfAPoint());

    // The point is inside the collision shape
    return true;
}

// Ray casting algorithm agains a convex collision shape using the GJK Algorithm
/// This method implements the GJK ray casting algorithm described by Gino Van Den Bergen in
/// "Ray Casting against General Convex Objects with Application to Continuous Collision Detection".
bool GJKAlgorithm::raycast(const Ray& ray, ProxyShape* proxyShape, RaycastInfo& raycastInfo) {

    assert(proxyShape->getCollisionShape()->isConvex());

    const ConvexShape* shape = static_cast<const ConvexShape*>(proxyShape->getCollisionShape());

    Vector3 suppA;      // Current lower bound point on the ray (starting at ray's origin)
    Vector3 suppB;      // Support point on the collision shape
    const decimal machineEpsilonSquare = MACHINE_EPSILON * MACHINE_EPSILON;
    const decimal epsilon = decimal(0.0001);

    // Convert the ray origin and direction into the local-space of the collision shape
    Vector3 rayDirection = ray.point2 - ray.point1;

    // If the points of the segment are two close, return no hit
    if (rayDirection.lengthSquare() < machineEpsilonSquare) return false;

    Vector3 w;

    // Create a simplex set
    VoronoiSimplex simplex;

    Vector3 n(decimal(0.0), decimal(0.0), decimal(0.0));
    decimal lambda = decimal(0.0);
    suppA = ray.point1;    // Current lower bound point on the ray (starting at ray's origin)
    suppB = shape->getLocalSupportPointWithoutMargin(rayDirection);
    Vector3 v = suppA - suppB;
    decimal vDotW, vDotR;
    decimal distSquare = v.lengthSquare();
    int nbIterations = 0;

    // GJK Algorithm loop
    while (distSquare > epsilon && nbIterations < MAX_ITERATIONS_GJK_RAYCAST) {

        // Compute the support points
        suppB = shape->getLocalSupportPointWithoutMargin(v);
        w = suppA - suppB;

        vDotW = v.dot(w);

        if (vDotW > decimal(0)) {

            vDotR = v.dot(rayDirection);

            if (vDotR >= -machineEpsilonSquare) {
                return false;
            }
            else {

                // We have found a better lower bound for the hit point along the ray
                lambda = lambda - vDotW / vDotR;
                suppA = ray.point1 + lambda * rayDirection;
                w = suppA - suppB;
                n = v;
            }
        }

        // Add the new support point to the simplex
        if (!simplex.isPointInSimplex(w)) {
            simplex.addPoint(w, suppA, suppB);
        }

        // Compute the closest point
        if (simplex.computeClosestPoint(v)) {

            distSquare = v.lengthSquare();
        }
        else {
            distSquare = decimal(0.0);
        }

        // If the current lower bound distance is larger than the maximum raycasting distance
        if (lambda > ray.maxFraction) return false;

        nbIterations++;
    }

    // If the origin was inside the shape, we return no hit
    if (lambda < MACHINE_EPSILON) return false;

    // Compute the closet points of both objects (without the margins)
    Vector3 pointA;
    Vector3 pointB;
    simplex.computeClosestPointsOfAandB(pointA, pointB);

    // A raycast hit has been found, we fill in the raycast info
    raycastInfo.hitFraction = lambda;
    raycastInfo.worldPoint = pointB;
    raycastInfo.body = proxyShape->getBody();
    raycastInfo.proxyShape = proxyShape;

    if (n.lengthSquare() >= machineEpsilonSquare) { // The normal vector is valid
        raycastInfo.worldNormal = n;
    }
    else {  // Degenerated normal vector, we return a zero normal vector
        raycastInfo.worldNormal = Vector3(decimal(0), decimal(0), decimal(0));
    }

    return true;
}
