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
#include "SphereShape.h"
#include "collision/ProxyShape.h"
#include "configuration.h"
#include <cassert>

using namespace reactphysics3d;

// Constructor
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

// Raycast method
bool SphereShape::raycast(const Ray& ray, ProxyShape* proxyShape) const {

    const Transform localToWorldTransform = proxyShape->getLocalToWorldTransform();
    const Transform worldToLocalTransform = localToWorldTransform.getInverse();
    Vector3 origin = worldToLocalTransform * ray.origin;
    decimal c = origin.dot(origin) - mRadius * mRadius;

    // If the origin of the ray is inside the sphere, we return no intersection
    if (c < decimal(0.0)) return false;

    Vector3 rayDirection = worldToLocalTransform.getOrientation() * ray.direction.getUnit();
    decimal b = origin.dot(rayDirection);

    // If the origin of the ray is outside the sphere and the ray
    // is pointing away from the sphere and there is no intersection
    if (c >= decimal(0.0) && b > decimal(0.0)) return false;

    // Compute the discriminant of the quadratic equation
    decimal discriminant = b*b - c;

    // If the discriminant is negative, there is no intersection
    return (discriminant >= decimal(0.0));
}

// Raycast method with feedback information
bool SphereShape::raycast(const Ray& ray, RaycastInfo& raycastInfo, ProxyShape* proxyShape,
                          decimal distance) const {

    const Transform localToWorldTransform = proxyShape->getLocalToWorldTransform();
    const Transform worldToLocalTransform = localToWorldTransform.getInverse();
    Vector3 origin = worldToLocalTransform * ray.origin;
    decimal c = origin.dot(origin) - mRadius * mRadius;

    // If the origin of the ray is inside the sphere, we return no intersection
    if (c < decimal(0.0)) return false;

    Vector3 rayDirection = worldToLocalTransform.getOrientation() * ray.direction.getUnit();
    decimal b = origin.dot(rayDirection);

    // If the origin of the ray is outside the sphere and the ray
    // is pointing away from the sphere and there is no intersection
    if (c >= decimal(0.0) && b > decimal(0.0)) return false;

    // Compute the discriminant of the quadratic equation
    decimal discriminant = b*b - c;

    // If the discriminant is negative, there is no intersection
    if (discriminant < decimal(0.0)) return false;

    // Compute the solution "t" closest to the origin
    decimal t = -b - std::sqrt(discriminant);

    assert(t >= decimal(0.0));

    // If the intersection distance is larger than the allowed distance, return no intersection
    if (t > distance) return false;

    // Compute the intersection information
    Vector3 localPoint = origin + t * rayDirection;
    raycastInfo.body = proxyShape->getBody();
    raycastInfo.proxyShape = proxyShape;
    raycastInfo.distance = t;
    raycastInfo.worldPoint = localToWorldTransform * localPoint;
    raycastInfo.worldNormal = (raycastInfo.worldPoint -
                               localToWorldTransform.getPosition()).getUnit();

    return true;
}
