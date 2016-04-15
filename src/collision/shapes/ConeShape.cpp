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
#include <complex>
#include "configuration.h"
#include "ConeShape.h"
#include "collision/ProxyShape.h"

using namespace reactphysics3d;

// Constructor
/**
 * @param radius Radius of the cone (in meters)
 * @param height Height of the cone (in meters)
 * @param margin Collision margin (in meters) around the collision shape
 */
ConeShape::ConeShape(decimal radius, decimal height, decimal margin)
          : ConvexShape(CONE, margin), mRadius(radius), mHalfHeight(height * decimal(0.5)) {
    assert(mRadius > decimal(0.0));
    assert(mHalfHeight > decimal(0.0));
    
    // Compute the sine of the semi-angle at the apex point
    mSinTheta = mRadius / (sqrt(mRadius * mRadius + height * height));
}

// Destructor
ConeShape::~ConeShape() {

}

// Return a local support point in a given direction without the object margin
Vector3 ConeShape::getLocalSupportPointWithoutMargin(const Vector3& direction,
                                                     void** cachedCollisionData) const {

    const Vector3& v = direction;
    decimal sinThetaTimesLengthV = mSinTheta * v.length();
    Vector3 supportPoint;

    if (v.y > sinThetaTimesLengthV) {
        supportPoint = Vector3(0.0, mHalfHeight, 0.0);
    }
    else {
        decimal projectedLength = sqrt(v.x * v.x + v.z * v.z);
        if (projectedLength > MACHINE_EPSILON) {
            decimal d = mRadius / projectedLength;
            supportPoint = Vector3(v.x * d, -mHalfHeight, v.z * d);
        }
        else {
            supportPoint = Vector3(0.0, -mHalfHeight, 0.0);
        }
    }

    return supportPoint;
}

// Raycast method with feedback information
// This implementation is based on the technique described by David Eberly in the article
// "Intersection of a Line and a Cone" that can be found at
// http://www.geometrictools.com/Documentation/IntersectionLineCone.pdf
bool ConeShape::raycast(const Ray& ray, RaycastInfo& raycastInfo, ProxyShape* proxyShape) const {

    const Vector3 r = ray.point2 - ray.point1;

    const decimal epsilon = decimal(0.00001);
    Vector3 V(0, mHalfHeight, 0);
    Vector3 centerBase(0, -mHalfHeight, 0);
    Vector3 axis(0, decimal(-1.0), 0);
    decimal heightSquare = decimal(4.0) * mHalfHeight * mHalfHeight;
    decimal cosThetaSquare = heightSquare / (heightSquare + mRadius * mRadius);
    decimal factor = decimal(1.0) - cosThetaSquare;
    Vector3 delta = ray.point1 - V;
    decimal c0 = -cosThetaSquare * delta.x * delta.x  + factor * delta.y * delta.y -
                  cosThetaSquare * delta.z * delta.z;
    decimal c1 = -cosThetaSquare * delta.x * r.x + factor * delta.y * r.y - cosThetaSquare * delta.z * r.z;
    decimal c2 = -cosThetaSquare * r.x * r.x  + factor * r.y * r.y - cosThetaSquare * r.z * r.z;
    decimal tHit[] = {decimal(-1.0), decimal(-1.0), decimal(-1.0)};
    Vector3 localHitPoint[3];
    Vector3 localNormal[3];

    // If c2 is different from zero
    if (std::abs(c2) > MACHINE_EPSILON) {
        decimal gamma = c1 * c1 - c0 * c2;

        // If there is no real roots in the quadratic equation
        if (gamma < decimal(0.0)) {
            return false;
        }
        else if (gamma > decimal(0.0)) {    // The equation has two real roots

            // Compute two intersections
            decimal sqrRoot = std::sqrt(gamma);
            tHit[0] = (-c1 - sqrRoot) / c2;
            tHit[1] = (-c1 + sqrRoot) / c2;
        }
        else {  // If the equation has a single real root

            // Compute the intersection
            tHit[0] = -c1 / c2;
        }
    }
    else {  // If c2 == 0

        // If c2 = 0 and c1 != 0
        if (std::abs(c1) > MACHINE_EPSILON) {
            tHit[0] = -c0 / (decimal(2.0) * c1);
        }
        else {  // If c2 = c1 = 0

            // If c0 is different from zero, no solution and if c0 = 0, we have a
            // degenerate case, the whole ray is contained in the cone side
            // but we return no hit in this case
            return false;
        }
    }

    // If the origin of the ray is inside the cone, we return no hit
    if (testPointInside(ray.point1, NULL)) return false;

    localHitPoint[0] = ray.point1 + tHit[0] * r;
    localHitPoint[1] = ray.point1 + tHit[1] * r;

    // Only keep hit points in one side of the double cone (the cone we are interested in)
    if (axis.dot(localHitPoint[0] - V) < decimal(0.0)) {
        tHit[0] = decimal(-1.0);
    }
    if (axis.dot(localHitPoint[1] - V) < decimal(0.0)) {
        tHit[1] = decimal(-1.0);
    }

    // Only keep hit points that are within the correct height of the cone
    if (localHitPoint[0].y < decimal(-mHalfHeight)) {
        tHit[0] = decimal(-1.0);
    }
    if (localHitPoint[1].y < decimal(-mHalfHeight)) {
        tHit[1] = decimal(-1.0);
    }

    // If the ray is in direction of the base plane of the cone
    if (r.y > epsilon) {

        // Compute the intersection with the base plane of the cone
        tHit[2] = (-ray.point1.y - mHalfHeight) / (r.y);

        // Only keep this intersection if it is inside the cone radius
        localHitPoint[2] = ray.point1 + tHit[2] * r;

        if ((localHitPoint[2] - centerBase).lengthSquare() > mRadius * mRadius) {
            tHit[2] = decimal(-1.0);
        }

        // Compute the normal direction
        localNormal[2] = axis;
    }

    // Find the smallest positive t value
    int hitIndex = -1;
    decimal t = DECIMAL_LARGEST;
    for (int i=0; i<3; i++) {
        if (tHit[i] < decimal(0.0)) continue;
        if (tHit[i] < t) {
            hitIndex = i;
            t = tHit[hitIndex];
        }
    }

    if (hitIndex < 0) return false;
    if (t > ray.maxFraction) return false;

    // Compute the normal direction for hit against side of the cone
    if (hitIndex != 2) {
        decimal h = decimal(2.0) * mHalfHeight;
        decimal value1 = (localHitPoint[hitIndex].x * localHitPoint[hitIndex].x +
                          localHitPoint[hitIndex].z * localHitPoint[hitIndex].z);
        decimal rOverH = mRadius / h;
        decimal value2 = decimal(1.0) + rOverH * rOverH;
        decimal factor = decimal(1.0) / std::sqrt(value1 * value2);
        decimal x = localHitPoint[hitIndex].x * factor;
        decimal z = localHitPoint[hitIndex].z * factor;
        localNormal[hitIndex].x = x;
        localNormal[hitIndex].y = std::sqrt(x * x + z * z) * rOverH;
        localNormal[hitIndex].z = z;
    }

    raycastInfo.body = proxyShape->getBody();
    raycastInfo.proxyShape = proxyShape;
    raycastInfo.hitFraction = t;
    raycastInfo.worldPoint = localHitPoint[hitIndex];
    raycastInfo.worldNormal = localNormal[hitIndex];

    return true;
}
