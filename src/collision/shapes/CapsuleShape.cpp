/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2010-2013 Daniel Chappuis                                       *
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
#include "CapsuleShape.h"
#include "collision/ProxyShape.h"
#include "configuration.h"
#include <cassert>

using namespace reactphysics3d;

// Constructor
CapsuleShape::CapsuleShape(decimal radius, decimal height)
            : CollisionShape(CAPSULE, radius), mRadius(radius), mHalfHeight(height * decimal(0.5)) {
    assert(radius > decimal(0.0));
    assert(height > decimal(0.0));
}

// Private copy-constructor
CapsuleShape::CapsuleShape(const CapsuleShape& shape)
             : CollisionShape(shape), mRadius(shape.mRadius), mHalfHeight(shape.mHalfHeight) {

}

// Destructor
CapsuleShape::~CapsuleShape() {

}

// Return a local support point in a given direction with the object margin.
/// A capsule is the convex hull of two spheres S1 and S2. The support point in the direction "d"
/// of the convex hull of a set of convex objects is the support point "p" in the set of all
/// support points from all the convex objects with the maximum dot product with the direction "d".
/// Therefore, in this method, we compute the support points of both top and bottom spheres of
/// the capsule and return the point with the maximum dot product with the direction vector. Note
/// that the object margin is implicitly the radius and height of the capsule.
Vector3 CapsuleShape::getLocalSupportPointWithMargin(const Vector3& direction,
                                                     void** cachedCollisionData) const {

    // If the direction vector is not the zero vector
    if (direction.lengthSquare() >= MACHINE_EPSILON * MACHINE_EPSILON) {

        Vector3 unitDirection = direction.getUnit();

        // Support point top sphere
        Vector3 centerTopSphere(0, mHalfHeight, 0);
        Vector3 topSpherePoint = centerTopSphere + unitDirection * mRadius;
        decimal dotProductTop = topSpherePoint.dot(direction);

        // Support point bottom sphere
        Vector3 centerBottomSphere(0, -mHalfHeight, 0);
        Vector3 bottomSpherePoint = centerBottomSphere + unitDirection * mRadius;
        decimal dotProductBottom = bottomSpherePoint.dot(direction);

        // Return the point with the maximum dot product
        if (dotProductTop > dotProductBottom) {
            return topSpherePoint;
        }
        else {
            return bottomSpherePoint;
        }
    }

    // If the direction vector is the zero vector we return a point on the
    // boundary of the capsule
    return Vector3(0, mRadius, 0);
}

// Return a local support point in a given direction without the object margin.
Vector3 CapsuleShape::getLocalSupportPointWithoutMargin(const Vector3& direction,
                                                        void** cachedCollisionData) const {

    // If the dot product of the direction and the local Y axis (dotProduct = direction.y)
    // is positive
    if (direction.y > 0.0) {

        // Return the top sphere center point
        return Vector3(0, mHalfHeight, 0);
    }
    else {

        // Return the bottom sphere center point
        return Vector3(0, -mHalfHeight, 0);
    }
}

// Return the local inertia tensor of the capsule
void CapsuleShape::computeLocalInertiaTensor(Matrix3x3& tensor, decimal mass) const {

	// The inertia tensor formula for a capsule can be found in : Game Engine Gems, Volume 1
	
    decimal height = mHalfHeight + mHalfHeight;
	decimal radiusSquare = mRadius * mRadius;
	decimal heightSquare = height * height;
	decimal radiusSquareDouble = radiusSquare + radiusSquare;
	decimal factor1 = decimal(2.0) * mRadius / (decimal(4.0) * mRadius + decimal(3.0) * height);
	decimal factor2 = decimal(3.0) * height / (decimal(4.0) * mRadius + decimal(3.0) * height);
	decimal sum1 = decimal(0.4) * radiusSquareDouble;
	decimal sum2 = decimal(0.75) * height * mRadius + decimal(0.5) * heightSquare;
	decimal sum3 = decimal(0.25) * radiusSquare + decimal(1.0 / 12.0) * heightSquare;
	decimal IxxAndzz = factor1 * mass * (sum1 + sum2) + factor2 * mass * sum3;
	decimal Iyy = factor1 * mass * sum1 + factor2 * mass * decimal(0.25) * radiusSquareDouble;
    tensor.setAllValues(IxxAndzz, 0.0, 0.0,
                        0.0, Iyy, 0.0,
                        0.0, 0.0, IxxAndzz);
}

// Return true if a point is inside the collision shape
bool CapsuleShape::testPointInside(const Vector3& localPoint, ProxyShape* proxyShape) const {

    const decimal diffYCenterSphere1 = localPoint.y - mHalfHeight;
    const decimal diffYCenterSphere2 = localPoint.y + mHalfHeight;
    const decimal xSquare = localPoint.x * localPoint.x;
    const decimal zSquare = localPoint.z * localPoint.z;
    const decimal squareRadius = mRadius * mRadius;

    // Return true if the point is inside the cylinder or one of the two spheres of the capsule
    return ((xSquare + zSquare) < squareRadius &&
            localPoint.y < mHalfHeight && localPoint.y > -mHalfHeight) ||
            (xSquare + zSquare + diffYCenterSphere1 * diffYCenterSphere1) < squareRadius ||
            (xSquare + zSquare + diffYCenterSphere2 * diffYCenterSphere2) < squareRadius;
}

// Raycast method
bool CapsuleShape::raycast(const Ray& ray, ProxyShape* proxyShape) const {

    // Transform the ray direction and origin in local-space coordinates
    const Transform localToWorldTransform = proxyShape->getLocalToWorldTransform();
    const Transform worldToLocalTransform = localToWorldTransform.getInverse();
    Vector3 origin = worldToLocalTransform * ray.origin;
    Vector3 n = worldToLocalTransform.getOrientation() * ray.direction.getUnit();

    const decimal epsilon = decimal(0.00001);
    Vector3 p(decimal(0), -mHalfHeight, decimal(0));
    Vector3 q(decimal(0), mHalfHeight, decimal(0));
    Vector3 d = q - p;
    Vector3 m = origin - p;
    decimal t;

    decimal mDotD = m.dot(d);
    decimal nDotD = n.dot(d);
    decimal dDotD = d.dot(d);
    decimal mDotN = m.dot(n);

    decimal a = dDotD - nDotD * nDotD;
    decimal k = m.dot(m) - mRadius * mRadius;
    decimal c = dDotD * k - mDotD * mDotD;

    // If the ray is parallel to the cylinder axis
    if (std::abs(a) < epsilon) {

        // If the origin is outside the surface of the cylinder, we return no hit
        if (c > decimal(0.0)) return false;

        // Here we know that the segment intersect an endcap of the cylinder

        // If the ray intersects with the "p" endcap of the capsule
        if (mDotD < decimal(0.0)) {

            // Check intersection with the sphere "p" endcap of the capsule
            return raycastWithSphereEndCap(origin, n, p);
        }
        else if (mDotD > dDotD) {   // If the ray intersects with the "q" endcap of the cylinder

            // Check intersection with the sphere "q" endcap of the capsule
            return raycastWithSphereEndCap(origin, n, q);
        }
        else {  // If the origin is inside the cylinder, we return no hit
            return false;
        }
    }
    decimal b = dDotD * mDotN - nDotD * mDotD;
    decimal discriminant = b * b - a * c;

    // If the discriminant is negative, no real roots and therfore, no hit
    if (discriminant < decimal(0.0)) return false;

    // Compute the smallest root (first intersection along the ray)
    decimal t0 = t = (-b - std::sqrt(discriminant)) / a;

    // If the intersection is outside the cylinder on "p" endcap side
    decimal value = mDotD + t * nDotD;
    if (value < decimal(0.0)) {

        // Check intersection with the sphere "p" endcap of the capsule
        return raycastWithSphereEndCap(origin, n, p);
    }
    else if (value > dDotD) {   // If the intersection is outside the cylinder on the "q" side

        // Check intersection with the sphere "q" endcap of the capsule
        return raycastWithSphereEndCap(origin, n, q);
    }

    // If the intersection is behind the origin of the ray, we return no hit
    return (t0 >= decimal(0.0));
}

// Raycast method with feedback information
bool CapsuleShape::raycast(const Ray& ray, RaycastInfo& raycastInfo, ProxyShape* proxyShape,
                           decimal distance) const {

    // Transform the ray direction and origin in local-space coordinates
    const Transform localToWorldTransform = proxyShape->getLocalToWorldTransform();
    const Transform worldToLocalTransform = localToWorldTransform.getInverse();
    Vector3 origin = worldToLocalTransform * ray.origin;
    Vector3 n = worldToLocalTransform.getOrientation() * ray.direction.getUnit();

    const decimal epsilon = decimal(0.00001);
    Vector3 p(decimal(0), -mHalfHeight, decimal(0));
    Vector3 q(decimal(0), mHalfHeight, decimal(0));
    Vector3 d = q - p;
    Vector3 m = origin - p;
    decimal t;

    decimal mDotD = m.dot(d);
    decimal nDotD = n.dot(d);
    decimal dDotD = d.dot(d);
    decimal mDotN = m.dot(n);

    decimal a = dDotD - nDotD * nDotD;
    decimal k = m.dot(m) - mRadius * mRadius;
    decimal c = dDotD * k - mDotD * mDotD;

    // If the ray is parallel to the capsule axis
    if (std::abs(a) < epsilon) {

        // If the origin is outside the surface of the capusle's cylinder, we return no hit
        if (c > decimal(0.0)) return false;

        // Here we know that the segment intersect an endcap of the capsule

        // If the ray intersects with the "p" endcap of the capsule
        if (mDotD < decimal(0.0)) {

            // Check intersection between the ray and the "p" sphere endcap of the capsule
            Vector3 hitLocalPoint;
            decimal hitDistance;
            if (raycastWithSphereEndCap(origin, n, p, distance, hitLocalPoint, hitDistance)) {
                raycastInfo.body = proxyShape->getBody();
                raycastInfo.proxyShape = proxyShape;
                raycastInfo.distance = hitDistance;
                raycastInfo.worldPoint = localToWorldTransform * hitLocalPoint;
                Vector3 normalDirection = (hitLocalPoint - p).getUnit();
                raycastInfo.worldNormal = localToWorldTransform.getOrientation() * normalDirection;

                return true;
            }

            return false;
        }
        else if (mDotD > dDotD) {   // If the ray intersects with the "q" endcap of the cylinder

            // Check intersection between the ray and the "q" sphere endcap of the capsule
            Vector3 hitLocalPoint;
            decimal hitDistance;
            if (raycastWithSphereEndCap(origin, n, q, distance, hitLocalPoint, hitDistance)) {
                raycastInfo.body = proxyShape->getBody();
                raycastInfo.proxyShape = proxyShape;
                raycastInfo.distance = hitDistance;
                raycastInfo.worldPoint = localToWorldTransform * hitLocalPoint;
                Vector3 normalDirection = (hitLocalPoint - q).getUnit();
                raycastInfo.worldNormal = localToWorldTransform.getOrientation() * normalDirection;

                return true;
            }

            return false;
        }
        else {  // If the origin is inside the cylinder, we return no hit
            return false;
        }
    }
    decimal b = dDotD * mDotN - nDotD * mDotD;
    decimal discriminant = b * b - a * c;

    // If the discriminant is negative, no real roots and therfore, no hit
    if (discriminant < decimal(0.0)) return false;

    // Compute the smallest root (first intersection along the ray)
    decimal t0 = t = (-b - std::sqrt(discriminant)) / a;

    // If the intersection is outside the finite cylinder of the capsule on "p" endcap side
    decimal value = mDotD + t * nDotD;
    if (value < decimal(0.0)) {

        // Check intersection between the ray and the "p" sphere endcap of the capsule
        Vector3 hitLocalPoint;
        decimal hitDistance;
        if (raycastWithSphereEndCap(origin, n, p, distance, hitLocalPoint, hitDistance)) {
            raycastInfo.body = proxyShape->getBody();
            raycastInfo.proxyShape = proxyShape;
            raycastInfo.distance = hitDistance;
            raycastInfo.worldPoint = localToWorldTransform * hitLocalPoint;
            Vector3 normalDirection = (hitLocalPoint - p).getUnit();
            raycastInfo.worldNormal = localToWorldTransform.getOrientation() * normalDirection;

            return true;
        }

        return false;
    }
    else if (value > dDotD) {  // If the intersection is outside the finite cylinder on the "q" side

        // Check intersection between the ray and the "q" sphere endcap of the capsule
        Vector3 hitLocalPoint;
        decimal hitDistance;
        if (raycastWithSphereEndCap(origin, n, q, distance, hitLocalPoint, hitDistance)) {
            raycastInfo.body = proxyShape->getBody();
            raycastInfo.proxyShape = proxyShape;
            raycastInfo.distance = hitDistance;
            raycastInfo.worldPoint = localToWorldTransform * hitLocalPoint;
            Vector3 normalDirection = (hitLocalPoint - q).getUnit();
            raycastInfo.worldNormal = localToWorldTransform.getOrientation() * normalDirection;

            return true;
        }

        return false;
    }

    t = t0;

    // If the intersection is behind the origin of the ray or beyond the maximum
    // raycasting distance, we return no hit
    if (t < decimal(0.0) || t > distance) return false;

    // Compute the hit information
    Vector3 localHitPoint = origin + t * n;
    raycastInfo.body = proxyShape->getBody();
    raycastInfo.proxyShape = proxyShape;
    raycastInfo.distance = t;
    raycastInfo.worldPoint = localToWorldTransform * localHitPoint;
    Vector3 v = localHitPoint - p;
    Vector3 w = (v.dot(d) / d.lengthSquare()) * d;
    Vector3 normalDirection = (localHitPoint - (p + w)).getUnit();
    raycastInfo.worldNormal = localToWorldTransform.getOrientation() * normalDirection;

    return true;
}

// Raycasting method between a ray one of the two spheres end cap of the capsule
bool CapsuleShape::raycastWithSphereEndCap(const Vector3& rayOrigin, const Vector3& rayDirection,
                                           const Vector3& sphereCenter, decimal maxDistance,
                                           Vector3& hitLocalPoint, decimal& hitDistance) const {

    Vector3 m = rayOrigin - sphereCenter;
    decimal c = m.dot(m) - mRadius * mRadius;

    // If the origin of the ray is inside the sphere, we return no intersection
    if (c < decimal(0.0)) return false;

    decimal b = m.dot(rayDirection);

    // If the origin of the ray is outside the sphere and the ray
    // is pointing away from the sphere and there is no intersection
    if (c >= decimal(0.0) && b > decimal(0.0)) return false;

    // Compute the discriminant of the quadratic equation
    decimal discriminant = b * b - c;

    // If the discriminant is negative, there is no intersection
    if (discriminant < decimal(0.0)) return false;

    // Compute the solution "t" closest to the origin
    decimal t = -b - std::sqrt(discriminant);

    assert(t >= decimal(0.0));

    // If the intersection distance is larger than the allowed distance, return no intersection
    if (t > maxDistance) return false;

    // Compute the hit point and distance
    hitLocalPoint = rayOrigin + t * rayDirection;
    hitDistance = t;

    return true;
}

// Raycasting method between a ray one of the two spheres end cap of the capsule
/// This method returns true if there is an intersection and false otherwise but does not
/// compute the intersection point.
bool CapsuleShape::raycastWithSphereEndCap(const Vector3& rayOrigin, const Vector3& rayDirection,
                                           const Vector3& sphereCenter) const {

    Vector3 m = rayOrigin - sphereCenter;
    decimal c = m.dot(m) - mRadius * mRadius;

    // If the origin of the ray is inside the sphere, we return no intersection
    if (c < decimal(0.0)) return false;

    decimal b = m.dot(rayDirection);

    // If the origin of the ray is outside the sphere and the ray
    // is pointing away from the sphere and there is no intersection
    if (c >= decimal(0.0) && b > decimal(0.0)) return false;

    // Compute the discriminant of the quadratic equation
    decimal discriminant = b * b - c;

    // If the discriminant is negative, there is no intersection
    return (discriminant >= decimal(0.0));
}
