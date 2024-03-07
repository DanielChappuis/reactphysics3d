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
#include <reactphysics3d/collision/shapes/CapsuleShape.h>
#include <reactphysics3d/collision/Collider.h>
#include <reactphysics3d/configuration.h>
#include <reactphysics3d/collision/RaycastInfo.h>
#include <cassert>

using namespace reactphysics3d;

// Constructor
/**
 * @param radius The radius of the capsule (in meters)
 * @param height The height of the capsule (in meters)
 */
CapsuleShape::CapsuleShape(decimal radius, decimal height, MemoryAllocator& allocator)
            : ConvexShape(CollisionShapeName::CAPSULE, CollisionShapeType::CAPSULE, allocator, radius), mHalfHeight(height * decimal(0.5)) {

    assert(radius > decimal(0.0));
    assert(height > decimal(0.0));
}

// Return the local inertia tensor of the capsule
/**
 * @param mass Mass to use to compute the inertia tensor of the collision shape
 */
Vector3 CapsuleShape::getLocalInertiaTensor(decimal mass) const {

	// The inertia tensor formula for a capsule can be found in : Game Engine Gems, Volume 1
	
    const decimal height = mHalfHeight + mHalfHeight;
    const decimal radiusSquare = mMargin * mMargin;
    const decimal heightSquare = height * height;
    const decimal radiusSquareDouble = radiusSquare + radiusSquare;
    const decimal factor1 = decimal(2.0) * mMargin / (decimal(4.0) * mMargin + decimal(3.0) * height);
    const decimal factor2 = decimal(3.0) * height / (decimal(4.0) * mMargin + decimal(3.0) * height);
    const decimal sum1 = decimal(0.4) * radiusSquareDouble;
    const decimal sum2 = decimal(0.75) * height * mMargin + decimal(0.5) * heightSquare;
    const decimal sum3 = decimal(0.25) * radiusSquare + decimal(1.0 / 12.0) * heightSquare;
    const decimal IxxAndzz = factor1 * mass * (sum1 + sum2) + factor2 * mass * sum3;
    const decimal Iyy = factor1 * mass * sum1 + factor2 * mass * decimal(0.25) * radiusSquareDouble;
    return Vector3(IxxAndzz, Iyy, IxxAndzz);
}

// Return true if a point is inside the collision shape
bool CapsuleShape::testPointInside(const Vector3& localPoint, Collider* /*collider*/) const {

    const decimal diffYCenterSphere1 = localPoint.y - mHalfHeight;
    const decimal diffYCenterSphere2 = localPoint.y + mHalfHeight;
    const decimal xSquare = localPoint.x * localPoint.x;
    const decimal zSquare = localPoint.z * localPoint.z;
    const decimal squareRadius = mMargin * mMargin;

    // Return true if the point is inside the cylinder or one of the two spheres of the capsule
    return ((xSquare + zSquare) < squareRadius &&
            localPoint.y < mHalfHeight && localPoint.y > -mHalfHeight) ||
            (xSquare + zSquare + diffYCenterSphere1 * diffYCenterSphere1) < squareRadius ||
            (xSquare + zSquare + diffYCenterSphere2 * diffYCenterSphere2) < squareRadius;
}

// Return the local bounds of the shape in x, y and z directions
// This method is used to compute the AABB of the box
/**
 * @return The AABB of the shape
 */
AABB CapsuleShape::getLocalBounds() const {

    return AABB(Vector3(-mMargin, -mHalfHeight - mMargin, -mMargin),
                Vector3(mMargin, mHalfHeight + mMargin, mMargin));
}

// Raycast method with feedback information
bool CapsuleShape::raycast(const Ray& ray, RaycastInfo& raycastInfo, Collider* collider, MemoryAllocator& /*allocator*/) const {

    const Vector3 n = ray.point2 - ray.point1;

    const decimal epsilon = decimal(0.01);
    Vector3 p(decimal(0), -mHalfHeight, decimal(0));
    Vector3 q(decimal(0), mHalfHeight, decimal(0));
    Vector3 d = q - p;
    Vector3 m = ray.point1 - p;
    decimal t;

    decimal mDotD = m.dot(d);
    decimal nDotD = n.dot(d);
    decimal dDotD = d.dot(d);

    // Test if the segment is outside the cylinder
    decimal vec1DotD = (ray.point1 - Vector3(decimal(0.0), -mHalfHeight - mMargin, decimal(0.0))).dot(d);
    if (vec1DotD < decimal(0.0) && vec1DotD + nDotD < decimal(0.0)) return false;
    decimal ddotDExtraCaps = decimal(2.0) * mMargin * d.y;
    if (vec1DotD > dDotD + ddotDExtraCaps && vec1DotD + nDotD > dDotD + ddotDExtraCaps) return false;

    decimal nDotN = n.dot(n);
    decimal mDotN = m.dot(n);

    decimal a = dDotD * nDotN - nDotD * nDotD;
    decimal k = m.dot(m) - mMargin * mMargin;
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
            decimal hitFraction;
            if (raycastWithSphereEndCap(ray.point1, ray.point2, p, ray.maxFraction, hitLocalPoint, hitFraction)) {
                raycastInfo.body = collider->getBody();
                raycastInfo.collider = collider;
                raycastInfo.hitFraction = hitFraction;
                raycastInfo.worldPoint = hitLocalPoint;
                Vector3 normalDirection = hitLocalPoint - p;
                raycastInfo.worldNormal = normalDirection;

                return true;
            }

            return false;
        }
        else if (mDotD > dDotD) {   // If the ray intersects with the "q" endcap of the cylinder

            // Check intersection between the ray and the "q" sphere endcap of the capsule
            Vector3 hitLocalPoint;
            decimal hitFraction;
            if (raycastWithSphereEndCap(ray.point1, ray.point2, q, ray.maxFraction, hitLocalPoint, hitFraction)) {
                raycastInfo.body = collider->getBody();
                raycastInfo.collider = collider;
                raycastInfo.hitFraction = hitFraction;
                raycastInfo.worldPoint = hitLocalPoint;
                Vector3 normalDirection = hitLocalPoint - q;
                raycastInfo.worldNormal = normalDirection;

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
        decimal hitFraction;
        if (raycastWithSphereEndCap(ray.point1, ray.point2, p, ray.maxFraction, hitLocalPoint, hitFraction)) {
            raycastInfo.body = collider->getBody();
            raycastInfo.collider = collider;
            raycastInfo.hitFraction = hitFraction;
            raycastInfo.worldPoint = hitLocalPoint;
            Vector3 normalDirection = hitLocalPoint - p;
            raycastInfo.worldNormal = normalDirection;

            return true;
        }

        return false;
    }
    else if (value > dDotD) {  // If the intersection is outside the finite cylinder on the "q" side

        // Check intersection between the ray and the "q" sphere endcap of the capsule
        Vector3 hitLocalPoint;
        decimal hitFraction;
        if (raycastWithSphereEndCap(ray.point1, ray.point2, q, ray.maxFraction, hitLocalPoint, hitFraction)) {
            raycastInfo.body = collider->getBody();
            raycastInfo.collider = collider;
            raycastInfo.hitFraction = hitFraction;
            raycastInfo.worldPoint = hitLocalPoint;
            Vector3 normalDirection = hitLocalPoint - q;
            raycastInfo.worldNormal = normalDirection;

            return true;
        }

        return false;
    }

    t = t0;

    // If the intersection is behind the origin of the ray or beyond the maximum
    // raycasting distance, we return no hit
    if (t < decimal(0.0) || t > ray.maxFraction) return false;

    // Compute the hit information
    Vector3 localHitPoint = ray.point1 + t * n;
    raycastInfo.body = collider->getBody();
    raycastInfo.collider = collider;
    raycastInfo.hitFraction = t;
    raycastInfo.worldPoint = localHitPoint;
    Vector3 v = localHitPoint - p;
    Vector3 w = (v.dot(d) / d.lengthSquare()) * d;
    Vector3 normalDirection = (localHitPoint - (p + w)).getUnit();
    raycastInfo.worldNormal = normalDirection;

    return true;
}

// Raycasting method between a ray one of the two spheres end cap of the capsule
bool CapsuleShape::raycastWithSphereEndCap(const Vector3& point1, const Vector3& point2,
                                           const Vector3& sphereCenter, decimal maxFraction,
                                           Vector3& hitLocalPoint, decimal& hitFraction) const {

     const Vector3 m = point1 - sphereCenter;
    decimal c = m.dot(m) - mMargin * mMargin;

    // If the origin of the ray is inside the sphere, we return no intersection
    if (c < decimal(0.0)) return false;

    const Vector3 rayDirection = point2 - point1;
    decimal b = m.dot(rayDirection);

    // If the origin of the ray is outside the sphere and the ray
    // is pointing away from the sphere, there is no intersection
    if (b > decimal(0.0)) return false;

    decimal raySquareLength = rayDirection.lengthSquare();

    // Compute the discriminant of the quadratic equation
    decimal discriminant = b * b - raySquareLength * c;

    // If the discriminant is negative or the ray length is very small, there is no intersection
    if (discriminant < decimal(0.0) || raySquareLength < MACHINE_EPSILON) return false;

    // Compute the solution "t" closest to the origin
    decimal t = -b - std::sqrt(discriminant);

    assert(t >= decimal(0.0));

    // If the hit point is withing the segment ray fraction
    if (t < maxFraction * raySquareLength) {

        // Compute the intersection information
        t /= raySquareLength;
        hitFraction = t;
        hitLocalPoint = point1 + t * rayDirection;

        return true;
    }

    return false;
}
