/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2015 Daniel Chappuis                                       *
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
#include <cmath>
#include <cfloat>
#include <cassert>
#include <algorithm>
#include "reactphysics3d/configuration.h"
#include "reactphysics3d/collision/narrowphase/GJK/GJKAlgorithm.h"
#include "reactphysics3d/collision/narrowphase/GJK/Simplex.h"
#include "reactphysics3d/constraint/ContactPoint.h"

using namespace reactphysics3d;

// Constructor
GJKAlgorithm::GJKAlgorithm(MemoryAllocator& memoryAllocator)
             :NarrowPhaseAlgorithm(memoryAllocator), mAlgoEPA(memoryAllocator) {
    
}

// Destructor
GJKAlgorithm::~GJKAlgorithm() {

}

// Return true and compute a contact info if the two collision shapes collide.
/// This method implements the Hybrid Technique for computing the penetration depth by
/// running the GJK algorithm on original objects (without margin).
/// If the objects don't intersect, this method returns false. If they intersect
/// only in the margins, the method compute the penetration depth and contact points
/// (of enlarged objects). If the original objects (without margin) intersect, we
/// call the computePenetrationDepthForEnlargedObjects() method that run the GJK
/// algorithm on the enlarged object to obtain a simplex polytope that contains the
/// origin, they we give that simplex polytope to the EPA algorithm which will compute
/// the correct penetration depth and contact points between the enlarged objects.
bool GJKAlgorithm::testCollision(ProxyShape* collisionShape1, ProxyShape* collisionShape2,
                                 ContactPointInfo*& contactInfo) {
    
    Vector3 suppA;             // Support point of object A
    Vector3 suppB;             // Support point of object B
    Vector3 w;                 // Support point of Minkowski difference A-B
    Vector3 pA;                // Closest point of object A
    Vector3 pB;                // Closest point of object B
    decimal vDotw;
    decimal prevDistSquare;

    // Get the local-space to world-space transforms
    const Transform transform1 = collisionShape1->getBody()->getTransform() *
                                 collisionShape1->getLocalToBodyTransform();
    const Transform transform2 = collisionShape2->getBody()->getTransform() *
                                 collisionShape2->getLocalToBodyTransform();

    // Transform a point from local space of body 2 to local
    // space of body 1 (the GJK algorithm is done in local space of body 1)
    Transform body2Tobody1 = transform1.getInverse() * transform2;

    // Matrix that transform a direction from local
    // space of body 1 into local space of body 2
    Matrix3x3 rotateToBody2 = transform2.getOrientation().getMatrix().getTranspose() *
                              transform1.getOrientation().getMatrix();

    // Initialize the margin (sum of margins of both objects)
    decimal margin = collisionShape1->getMargin() + collisionShape2->getMargin();
    decimal marginSquare = margin * margin;
    assert(margin > 0.0);

    // Create a simplex set
    Simplex simplex;

    // Get the previous point V (last cached separating axis)
    Vector3 v = mCurrentOverlappingPair->getCachedSeparatingAxis();

    // Initialize the upper bound for the square distance
    decimal distSquare = DECIMAL_LARGEST;
    
    do {
              
        // Compute the support points for original objects (without margins) A and B
        suppA = collisionShape1->getLocalSupportPointWithoutMargin(-v);
        suppB = body2Tobody1 *
                     collisionShape2->getLocalSupportPointWithoutMargin(rotateToBody2 * v);

        // Compute the support point for the Minkowski difference A-B
        w = suppA - suppB;
        
        vDotw = v.dot(w);
        
        // If the enlarge objects (with margins) do not intersect
        if (vDotw > 0.0 && vDotw * vDotw > distSquare * marginSquare) {
                        
            // Cache the current separating axis for frame coherence
            mCurrentOverlappingPair->setCachedSeparatingAxis(v);
            
            // No intersection, we return false
            return false;
        }

        // If the objects intersect only in the margins
        if (simplex.isPointInSimplex(w) || distSquare - vDotw <= distSquare * REL_ERROR_SQUARE) {

            // Compute the closet points of both objects (without the margins)
            simplex.computeClosestPointsOfAandB(pA, pB);

            // Project those two points on the margins to have the closest points of both
            // object with the margins
            decimal dist = sqrt(distSquare);
            assert(dist > 0.0);
            pA = (pA - (collisionShape1->getMargin() / dist) * v);
            pB = body2Tobody1.getInverse() * (pB + (collisionShape2->getMargin() / dist) * v);

            // Compute the contact info
            Vector3 normal = transform1.getOrientation() * (-v.getUnit());
            decimal penetrationDepth = margin - dist;
			
			// Reject the contact if the penetration depth is negative (due too numerical errors)
			if (penetrationDepth <= 0.0) return false;
			
            // Create the contact info object
            contactInfo = new (mMemoryAllocator.allocate(sizeof(ContactPointInfo)))
                                 ContactPointInfo(collisionShape1, collisionShape2, normal,
                                                  penetrationDepth, pA, pB);

            // There is an intersection, therefore we return true
            return true;
        }

        // Add the new support point to the simplex
        simplex.addPoint(w, suppA, suppB);

        // If the simplex is affinely dependent
        if (simplex.isAffinelyDependent()) {

            // Compute the closet points of both objects (without the margins)
            simplex.computeClosestPointsOfAandB(pA, pB);

            // Project those two points on the margins to have the closest points of both
            // object with the margins
            decimal dist = sqrt(distSquare);
            assert(dist > 0.0);
            pA = (pA - (collisionShape1->getMargin() / dist) * v);
            pB = body2Tobody1.getInverse() * (pB + (collisionShape2->getMargin() / dist) * v);

            // Compute the contact info
            Vector3 normal = transform1.getOrientation() * (-v.getUnit());
            decimal penetrationDepth = margin - dist;
			
			// Reject the contact if the penetration depth is negative (due too numerical errors)
			if (penetrationDepth <= 0.0) return false;
			
            // Create the contact info object
            contactInfo = new (mMemoryAllocator.allocate(sizeof(ContactPointInfo)))
                                   ContactPointInfo(collisionShape1, collisionShape2, normal,
                                                    penetrationDepth, pA, pB);

            // There is an intersection, therefore we return true
            return true;
        }

        // Compute the point of the simplex closest to the origin
        // If the computation of the closest point fail
        if (!simplex.computeClosestPoint(v)) {

            // Compute the closet points of both objects (without the margins)
            simplex.computeClosestPointsOfAandB(pA, pB);

            // Project those two points on the margins to have the closest points of both
            // object with the margins
            decimal dist = sqrt(distSquare);
            assert(dist > 0.0);
            pA = (pA - (collisionShape1->getMargin() / dist) * v);
            pB = body2Tobody1.getInverse() * (pB + (collisionShape2->getMargin() / dist) * v);

            // Compute the contact info
            Vector3 normal = transform1.getOrientation() * (-v.getUnit());
            decimal penetrationDepth = margin - dist;
			
			// Reject the contact if the penetration depth is negative (due too numerical errors)
			if (penetrationDepth <= 0.0) return false;
			
            // Create the contact info object
            contactInfo = new (mMemoryAllocator.allocate(sizeof(ContactPointInfo)))
                                 ContactPointInfo(collisionShape1, collisionShape2, normal,
                                                  penetrationDepth, pA, pB);

            // There is an intersection, therefore we return true
            return true;
        }

        // Store and update the squared distance of the closest point
        prevDistSquare = distSquare;
        distSquare = v.lengthSquare();

        // If the distance to the closest point doesn't improve a lot
        if (prevDistSquare - distSquare <= MACHINE_EPSILON * prevDistSquare) {
            simplex.backupClosestPointInSimplex(v);
            
            // Get the new squared distance
            distSquare = v.lengthSquare();

            // Compute the closet points of both objects (without the margins)
            simplex.computeClosestPointsOfAandB(pA, pB);

            // Project those two points on the margins to have the closest points of both
            // object with the margins
            decimal dist = sqrt(distSquare);
            assert(dist > 0.0);
            pA = (pA - (collisionShape1->getMargin() / dist) * v);
            pB = body2Tobody1.getInverse() * (pB + (collisionShape2->getMargin() / dist) * v);

            // Compute the contact info
            Vector3 normal = transform1.getOrientation() * (-v.getUnit());
            decimal penetrationDepth = margin - dist;
			
			// Reject the contact if the penetration depth is negative (due too numerical errors)
			if (penetrationDepth <= 0.0) return false;
			
            // Create the contact info object
            contactInfo = new (mMemoryAllocator.allocate(sizeof(ContactPointInfo)))
                                   ContactPointInfo(collisionShape1, collisionShape2, normal,
                                                    penetrationDepth, pA, pB);

            // There is an intersection, therefore we return true
            return true;
        }
    } while(!simplex.isFull() && distSquare > MACHINE_EPSILON *
                                 simplex.getMaxLengthSquareOfAPoint());

    // The objects (without margins) intersect. Therefore, we run the GJK algorithm
    // again but on the enlarged objects to compute a simplex polytope that contains
    // the origin. Then, we give that simplex polytope to the EPA algorithm to compute
    // the correct penetration depth and contact points between the enlarged objects.
    return computePenetrationDepthForEnlargedObjects(collisionShape1, transform1, collisionShape2,
                                                     transform2, contactInfo, v);
}

/// This method runs the GJK algorithm on the two enlarged objects (with margin)
/// to compute a simplex polytope that contains the origin. The two objects are
/// assumed to intersect in the original objects (without margin). Therefore such
/// a polytope must exist. Then, we give that polytope to the EPA algorithm to
/// compute the correct penetration depth and contact points of the enlarged objects.
bool GJKAlgorithm::computePenetrationDepthForEnlargedObjects(ProxyShape* collisionShape1,
                                                             const Transform& transform1,
                                                             ProxyShape* collisionShape2,
                                                             const Transform& transform2,
                                                             ContactPointInfo*& contactInfo,
                                                             Vector3& v) {
    Simplex simplex;
    Vector3 suppA;
    Vector3 suppB;
    Vector3 w;
    decimal vDotw;
    decimal distSquare = DECIMAL_LARGEST;
    decimal prevDistSquare;

    // Transform a point from local space of body 2 to local space
    // of body 1 (the GJK algorithm is done in local space of body 1)
    Transform body2ToBody1 = transform1.getInverse() * transform2;

    // Matrix that transform a direction from local space of body 1 into local space of body 2
    Matrix3x3 rotateToBody2 = transform2.getOrientation().getMatrix().getTranspose() *
                              transform1.getOrientation().getMatrix();
    
    do {
        // Compute the support points for the enlarged object A and B
        suppA = collisionShape1->getLocalSupportPointWithMargin(-v);
        suppB = body2ToBody1 * collisionShape2->getLocalSupportPointWithMargin(rotateToBody2 * v);

        // Compute the support point for the Minkowski difference A-B
        w = suppA - suppB;

        vDotw = v.dot(w);

        // If the enlarge objects do not intersect
        if (vDotw > 0.0) {

            // No intersection, we return false
            return false;
        }

        // Add the new support point to the simplex
        simplex.addPoint(w, suppA, suppB);

        if (simplex.isAffinelyDependent()) {
            return false;
        }

        if (!simplex.computeClosestPoint(v)) {
            return false;
        }

        // Store and update the square distance
        prevDistSquare = distSquare;
        distSquare = v.lengthSquare();

        if (prevDistSquare - distSquare <= MACHINE_EPSILON * prevDistSquare) {
            return false;
        }

    } while(!simplex.isFull() && distSquare > MACHINE_EPSILON *
                                 simplex.getMaxLengthSquareOfAPoint());

    // Give the simplex computed with GJK algorithm to the EPA algorithm
    // which will compute the correct penetration depth and contact points
    // between the two enlarged objects
    return mAlgoEPA.computePenetrationDepthAndContactPoints(simplex, collisionShape1,
                                                            transform1, collisionShape2, transform2,
                                                            v, contactInfo);
}

// Use the GJK Algorithm to find if a point is inside a convex collision shape
bool GJKAlgorithm::testPointInside(const Vector3& localPoint, ProxyShape* collisionShape) {

    Vector3 suppA;             // Support point of object A
    Vector3 w;                 // Support point of Minkowski difference A-B
    decimal prevDistSquare;

    // Support point of object B (object B is a single point)
    const Vector3 suppB(localPoint);

    // Create a simplex set
    Simplex simplex;

    // Initial supporting direction
    Vector3 v(1, 1, 1);

    // Initialize the upper bound for the square distance
    decimal distSquare = DECIMAL_LARGEST;

    do {

        // Compute the support points for original objects (without margins) A and B
        suppA = collisionShape->getLocalSupportPointWithoutMargin(-v);

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
bool GJKAlgorithm::raycast(const Ray& ray, ProxyShape* collisionShape, RaycastInfo& raycastInfo) {

    Vector3 suppA;      // Current lower bound point on the ray (starting at ray's origin)
    Vector3 suppB;      // Support point on the collision shape
    const decimal machineEpsilonSquare = MACHINE_EPSILON * MACHINE_EPSILON;
    const decimal epsilon = decimal(0.0001);

    // Convert the ray origin and direction into the local-space of the collision shape
    const Transform localToWorldTransform = collisionShape->getLocalToWorldTransform();
    const Transform worldToLocalTransform = localToWorldTransform.getInverse();
    Vector3 point1 = worldToLocalTransform * ray.point1;
    Vector3 point2 = worldToLocalTransform * ray.point2;
    Vector3 rayDirection = point2 - point1;

    // If the points of the segment are two close, return no hit
    if (rayDirection.lengthSquare() < machineEpsilonSquare) return false;

    Vector3 w;

    // Create a simplex set
    Simplex simplex;

    Vector3 n(decimal(0.0), decimal(0.0), decimal(0.0));
    decimal lambda = decimal(0.0);
    suppA = point1;    // Current lower bound point on the ray (starting at ray's origin)
    suppB = collisionShape->getLocalSupportPointWithoutMargin(rayDirection);
    Vector3 v = suppA - suppB;
    decimal vDotW, vDotR;
    decimal distSquare = v.lengthSquare();
    int nbIterations = 0;

    // GJK Algorithm loop
    while (distSquare > epsilon && nbIterations < MAX_ITERATIONS_GJK_RAYCAST) {

        // Compute the support points
        suppB = collisionShape->getLocalSupportPointWithoutMargin(v);
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
                suppA = point1 + lambda * rayDirection;
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
    raycastInfo.worldPoint = localToWorldTransform * pointB;
    raycastInfo.body = collisionShape->getBody();
    raycastInfo.proxyShape = collisionShape;

    if (n.lengthSquare() >= machineEpsilonSquare) { // The normal vector is valid
        raycastInfo.worldNormal = localToWorldTransform.getOrientation() * n.getUnit();
    }
    else {  // Degenerated normal vector, we return a zero normal vector
        raycastInfo.worldNormal = Vector3(decimal(0), decimal(0), decimal(0));
    }

    return true;
}
