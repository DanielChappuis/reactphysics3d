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
#include "CylinderShape.h"
#include "collision/ProxyShape.h"
#include "configuration.h"

using namespace reactphysics3d;

// Constructor
/**
 * @param radius Radius of the cylinder (in meters)
 * @param height Height of the cylinder (in meters)
 * @param margin Collision margin (in meters) around the collision shape
 */
CylinderShape::CylinderShape(decimal radius, decimal height, decimal margin)
              : ConvexShape(CYLINDER, margin), mRadius(radius),
                mHalfHeight(height/decimal(2.0)) {
    assert(radius > decimal(0.0));
    assert(height > decimal(0.0));
}

// Destructor
CylinderShape::~CylinderShape() {

}

// Return a local support point in a given direction without the object margin
Vector3 CylinderShape::getLocalSupportPointWithoutMargin(const Vector3& direction,
                                                         void** cachedCollisionData) const {

    Vector3 supportPoint(0.0, 0.0, 0.0);
    decimal uDotv = direction.y;
    Vector3 w(direction.x, 0.0, direction.z);
    decimal lengthW = sqrt(direction.x * direction.x + direction.z * direction.z);

    if (lengthW > MACHINE_EPSILON) {
        if (uDotv < 0.0) supportPoint.y = -mHalfHeight;
        else supportPoint.y = mHalfHeight;
        supportPoint += (mRadius / lengthW) * w;
    }
    else {
         if (uDotv < 0.0) supportPoint.y = -mHalfHeight;
         else supportPoint.y = mHalfHeight;
    }

    return supportPoint;
}

// Raycast method with feedback information
/// Algorithm based on the one described at page 194 in Real-ime Collision Detection by
/// Morgan Kaufmann.
bool CylinderShape::raycast(const Ray& ray, RaycastInfo& raycastInfo, ProxyShape* proxyShape) const {

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
    if (mDotD < decimal(0.0) && mDotD + nDotD < decimal(0.0)) return false;
    if (mDotD > dDotD && mDotD + nDotD > dDotD) return false;

    decimal nDotN = n.dot(n);
    decimal mDotN = m.dot(n);

    decimal a = dDotD * nDotN - nDotD * nDotD;
    decimal k = m.dot(m) - mRadius * mRadius;
    decimal c = dDotD * k - mDotD * mDotD;

    // If the ray is parallel to the cylinder axis
    if (std::abs(a) < epsilon) {

        // If the origin is outside the surface of the cylinder, we return no hit
        if (c > decimal(0.0)) return false;

        // Here we know that the segment intersect an endcap of the cylinder

        // If the ray intersects with the "p" endcap of the cylinder
        if (mDotD < decimal(0.0)) {

            t = -mDotN / nDotN;

            // If the intersection is behind the origin of the ray or beyond the maximum
            // raycasting distance, we return no hit
            if (t < decimal(0.0) || t > ray.maxFraction) return false;

            // Compute the hit information
            Vector3 localHitPoint = ray.point1 + t * n;
            raycastInfo.body = proxyShape->getBody();
            raycastInfo.proxyShape = proxyShape;
            raycastInfo.hitFraction = t;
            raycastInfo.worldPoint = localHitPoint;
            Vector3 normalDirection(0, decimal(-1), 0);
            raycastInfo.worldNormal = normalDirection;

            return true;
        }
        else if (mDotD > dDotD) {   // If the ray intersects with the "q" endcap of the cylinder

            t = (nDotD - mDotN) / nDotN;

            // If the intersection is behind the origin of the ray or beyond the maximum
            // raycasting distance, we return no hit
            if (t < decimal(0.0) || t > ray.maxFraction) return false;

            // Compute the hit information
            Vector3 localHitPoint = ray.point1 + t * n;
            raycastInfo.body = proxyShape->getBody();
            raycastInfo.proxyShape = proxyShape;
            raycastInfo.hitFraction = t;
            raycastInfo.worldPoint = localHitPoint;
            Vector3 normalDirection(0, decimal(1.0), 0);
            raycastInfo.worldNormal = normalDirection;

            return true;
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

        // If the ray is pointing away from the "p" endcap, we return no hit
        if (nDotD <= decimal(0.0)) return false;

        // Compute the intersection against the "p" endcap (intersection agains whole plane)
        t = -mDotD / nDotD;

        // Keep the intersection if the it is inside the cylinder radius
        if (k + t * (decimal(2.0) * mDotN + t) > decimal(0.0)) return false;

        // If the intersection is behind the origin of the ray or beyond the maximum
        // raycasting distance, we return no hit
        if (t < decimal(0.0) || t > ray.maxFraction) return false;

        // Compute the hit information
        Vector3 localHitPoint = ray.point1 + t * n;
        raycastInfo.body = proxyShape->getBody();
        raycastInfo.proxyShape = proxyShape;
        raycastInfo.hitFraction = t;
        raycastInfo.worldPoint = localHitPoint;
        Vector3 normalDirection(0, decimal(-1.0), 0);
        raycastInfo.worldNormal = normalDirection;

        return true;
    }
    else if (value > dDotD) {   // If the intersection is outside the cylinder on the "q" side

        // If the ray is pointing away from the "q" endcap, we return no hit
        if (nDotD >= decimal(0.0)) return false;

        // Compute the intersection against the "q" endcap (intersection against whole plane)
        t = (dDotD - mDotD) / nDotD;

        // Keep the intersection if it is inside the cylinder radius
        if (k + dDotD - decimal(2.0) * mDotD + t * (decimal(2.0) * (mDotN - nDotD) + t) >
            decimal(0.0)) return false;

        // If the intersection is behind the origin of the ray or beyond the maximum
        // raycasting distance, we return no hit
        if (t < decimal(0.0) || t > ray.maxFraction) return false;

        // Compute the hit information
        Vector3 localHitPoint = ray.point1 + t * n;
        raycastInfo.body = proxyShape->getBody();
        raycastInfo.proxyShape = proxyShape;
        raycastInfo.hitFraction = t;
        raycastInfo.worldPoint = localHitPoint;
        Vector3 normalDirection(0, decimal(1.0), 0);
        raycastInfo.worldNormal = normalDirection;

        return true;
    }

    t = t0;

    // If the intersection is behind the origin of the ray or beyond the maximum
    // raycasting distance, we return no hit
    if (t < decimal(0.0) || t > ray.maxFraction) return false;

    // Compute the hit information
    Vector3 localHitPoint = ray.point1 + t * n;
    raycastInfo.body = proxyShape->getBody();
    raycastInfo.proxyShape = proxyShape;
    raycastInfo.hitFraction = t;
    raycastInfo.worldPoint = localHitPoint;
    Vector3 v = localHitPoint - p;
    Vector3 w = (v.dot(d) / d.lengthSquare()) * d;
    Vector3 normalDirection = (localHitPoint - (p + w));
    raycastInfo.worldNormal = normalDirection;

    return true;
}
