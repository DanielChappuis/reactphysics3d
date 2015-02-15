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
#include "CapsuleShape.h"
#include "collision/ProxyShape.h"
#include "configuration.h"
#include <cassert>

using namespace reactphysics3d;

// Constructor
/**
 * @param radius The radius of the capsule (in meters)
 * @param height The height of the capsule (in meters)
 */
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
/**
 * @param[out] tensor The 3x3 inertia tensor matrix of the shape in local-space
 *                    coordinates
 * @param mass Mass to use to compute the inertia tensor of the collision shape
 */
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

// Raycast method with feedback information
bool CapsuleShape::raycast(const Ray& ray, RaycastInfo& raycastInfo, ProxyShape* proxyShape) const {

    // Transform the ray direction and origin in local-space coordinates
    const Transform localToWorldTransform = proxyShape->getLocalToWorldTransform();
    const Transform worldToLocalTransform = localToWorldTransform.getInverse();
    const Vector3 point1 = worldToLocalTransform * ray.point1;
    const Vector3 point2 = worldToLocalTransform * ray.point2;
    const Vector3 n = point2 - point1;

    const decimal epsilon = decimal(0.01);
    Vector3 p(decimal(0), -mHalfHeight, decimal(0));
    Vector3 q(decimal(0), mHalfHeight, decimal(0));
    Vector3 d = q - p;
    Vector3 m = point1 - p;
    decimal t;

    decimal mDotD = m.dot(d);
    decimal nDotD = n.dot(d);
    decimal dDotD = d.dot(d);

    // Test if the segment is outside the cylinder
    decimal vec1DotD = (point1 - Vector3(decimal(0.0), -mHalfHeight - mRadius, decimal(0.0))).dot(d);
    if (vec1DotD < decimal(0.0) && vec1DotD + nDotD < decimal(0.0)) return false;
    decimal ddotDExtraCaps = decimal(2.0) * mRadius * d.y;
    if (vec1DotD > dDotD + ddotDExtraCaps && vec1DotD + nDotD > dDotD + ddotDExtraCaps) return false;

    decimal nDotN = n.dot(n);
    decimal mDotN = m.dot(n);

    decimal a = dDotD * nDotN - nDotD * nDotD;
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
            decimal hitFraction;
            if (raycastWithSphereEndCap(point1, point2, p, ray.maxFraction, hitLocalPoint, hitFraction)) {
                raycastInfo.body = proxyShape->getBody();
                raycastInfo.proxyShape = proxyShape;
                raycastInfo.hitFraction = hitFraction;
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
            decimal hitFraction;
            if (raycastWithSphereEndCap(point1, point2, q, ray.maxFraction, hitLocalPoint, hitFraction)) {
                raycastInfo.body = proxyShape->getBody();
                raycastInfo.proxyShape = proxyShape;
                raycastInfo.hitFraction = hitFraction;
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
        decimal hitFraction;
        if (raycastWithSphereEndCap(point1, point2, p, ray.maxFraction, hitLocalPoint, hitFraction)) {
            raycastInfo.body = proxyShape->getBody();
            raycastInfo.proxyShape = proxyShape;
            raycastInfo.hitFraction = hitFraction;
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
        decimal hitFraction;
        if (raycastWithSphereEndCap(point1, point2, q, ray.maxFraction, hitLocalPoint, hitFraction)) {
            raycastInfo.body = proxyShape->getBody();
            raycastInfo.proxyShape = proxyShape;
            raycastInfo.hitFraction = hitFraction;
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
    if (t < decimal(0.0) || t > ray.maxFraction) return false;

    // Compute the hit information
    Vector3 localHitPoint = point1 + t * n;
    raycastInfo.body = proxyShape->getBody();
    raycastInfo.proxyShape = proxyShape;
    raycastInfo.hitFraction = t;
    raycastInfo.worldPoint = localToWorldTransform * localHitPoint;
    Vector3 v = localHitPoint - p;
    Vector3 w = (v.dot(d) / d.lengthSquare()) * d;
    Vector3 normalDirection = (localHitPoint - (p + w)).getUnit();
    raycastInfo.worldNormal = localToWorldTransform.getOrientation() * normalDirection;

    return true;
}

// Raycasting method between a ray one of the two spheres end cap of the capsule
bool CapsuleShape::raycastWithSphereEndCap(const Vector3& point1, const Vector3& point2,
                                           const Vector3& sphereCenter, decimal maxFraction,
                                           Vector3& hitLocalPoint, decimal& hitFraction) const {

     const Vector3 m = point1 - sphereCenter;
    decimal c = m.dot(m) - mRadius * mRadius;

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
