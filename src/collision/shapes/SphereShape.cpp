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
#include <reactphysics3d/collision/shapes/SphereShape.h>
#include <reactphysics3d/collision/Collider.h>
#include <reactphysics3d/configuration.h>
#include <reactphysics3d/collision/RaycastInfo.h>
#include <cassert>

using namespace reactphysics3d;

// Constructor
/**
 * @param radius Radius of the sphere
 */
SphereShape::SphereShape(decimal radius, MemoryAllocator& allocator)
            : ConvexShape(CollisionShapeName::SPHERE, CollisionShapeType::SPHERE, allocator, radius) {
    assert(radius > decimal(0.0));
}

// Compute the transformed AABB of the collision shape given a transform
/**
 * @param transform Transform used to for the space conversion
 * @return aabb The transformed axis-aligned bounding box (AABB) of the collision shape
 */
AABB SphereShape::computeTransformedAABB(const Transform& transform) const {

    RP3D_PROFILE("SphereShape::computeAABB()", mProfiler);

    // Get the local extents in x,y and z direction
    Vector3 extents(mMargin, mMargin, mMargin);

    // Return the AABB with the new minimum and maximum coordinates
    return AABB(transform.getPosition() - extents, transform.getPosition() + extents);
}

// Raycast method with feedback information
bool SphereShape::raycast(const Ray& ray, RaycastInfo& raycastInfo, Collider* collider, MemoryAllocator& /*allocator*/) const {

    const Vector3 m = ray.point1;
    decimal c = m.dot(m) - mMargin * mMargin;

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
        raycastInfo.body = collider->getBody();
        raycastInfo.collider = collider;
        raycastInfo.hitFraction = t;
        raycastInfo.worldPoint = ray.point1 + t * rayDirection;
        raycastInfo.worldNormal = raycastInfo.worldPoint;

        return true;
    }

    return false;
}
