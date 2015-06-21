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
#include <cassert>
#include "reactphysics3d/configuration.h"
#include "reactphysics3d/collision/shapes/SphereShape.h"
#include "reactphysics3d/collision/ProxyShape.h"

using namespace reactphysics3d;

// Constructor
/**
 * @param radius Radius of the sphere (in meters)
 */
SphereShape::SphereShape(decimal radius) : CollisionShape(SPHERE, radius), mRadius(radius) {
    assert(radius > decimal(0.0));
}

// Private copy-constructor
SphereShape::SphereShape(const SphereShape& shape)
            : CollisionShape(shape), mRadius(shape.mRadius) {

}

// Destructor
SphereShape::~SphereShape() {

}

// Raycast method with feedback information
bool SphereShape::raycast(const Ray& ray, RaycastInfo& raycastInfo, ProxyShape* proxyShape) const {

    // We perform the intersection test in world-space

    const Vector3 sphereCenter = proxyShape->getLocalToWorldTransform().getPosition();
    const Vector3 m = ray.point1 - sphereCenter;
    decimal c = m.dot(m) - mRadius * mRadius;

    // If the origin of the ray is inside the sphere, we return no intersection
    if (c < decimal(0.0)) return false;

    const Vector3 rayDirection = ray.point2 - ray.point1;
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
    if (t < ray.maxFraction * raySquareLength) {

        // Compute the intersection information
        t /= raySquareLength;
        raycastInfo.body = proxyShape->getBody();
        raycastInfo.proxyShape = proxyShape;
        raycastInfo.hitFraction = t;
        raycastInfo.worldPoint = ray.point1 + t * rayDirection;
        raycastInfo.worldNormal = (raycastInfo.worldPoint - sphereCenter).getUnit();

        return true;
    }

    return false;
}
