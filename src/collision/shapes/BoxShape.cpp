/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2020 Daniel Chappuis                                       *
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
#include <reactphysics3d/collision/shapes/BoxShape.h>
#include <reactphysics3d/collision/Collider.h>
#include <reactphysics3d/configuration.h>
#include <reactphysics3d/memory/MemoryManager.h>
#include <reactphysics3d/collision/RaycastInfo.h>
#include <cassert>

using namespace reactphysics3d;

// Constructor
/**
 * @param halfExtents The vector with the three half-extents of the box
 */
BoxShape::BoxShape(const Vector3& halfExtents, MemoryAllocator& allocator)
         : ConvexPolyhedronShape(CollisionShapeName::BOX, allocator), mHalfExtents(halfExtents),
           mHalfEdgeStructure(allocator, 6, 8, 24) {

    assert(halfExtents.x > decimal(0.0));
    assert(halfExtents.y > decimal(0.0));
    assert(halfExtents.z > decimal(0.0));

    // Vertices
    mHalfEdgeStructure.addVertex(0);
    mHalfEdgeStructure.addVertex(1);
    mHalfEdgeStructure.addVertex(2);
    mHalfEdgeStructure.addVertex(3);
    mHalfEdgeStructure.addVertex(4);
    mHalfEdgeStructure.addVertex(5);
    mHalfEdgeStructure.addVertex(6);
    mHalfEdgeStructure.addVertex(7);

    // Faces
    List<uint> face0(allocator, 4);
    face0.add(0); face0.add(1); face0.add(2); face0.add(3);
    List<uint> face1(allocator, 4);
    face1.add(1); face1.add(5); face1.add(6); face1.add(2);
    List<uint> face2(allocator, 4);
    face2.add(4); face2.add(7); face2.add(6); face2.add(5);
    List<uint> face3(allocator, 4);
    face3.add(4); face3.add(0); face3.add(3); face3.add(7);
    List<uint> face4(allocator, 4);
    face4.add(4); face4.add(5); face4.add(1); face4.add(0);
    List<uint> face5(allocator, 4);
    face5.add(2); face5.add(6); face5.add(7); face5.add(3);

    mHalfEdgeStructure.addFace(face0);
    mHalfEdgeStructure.addFace(face1);
    mHalfEdgeStructure.addFace(face2);
    mHalfEdgeStructure.addFace(face3);
    mHalfEdgeStructure.addFace(face4);
    mHalfEdgeStructure.addFace(face5);

	mHalfEdgeStructure.init();
}

// Return the local inertia tensor of the collision shape
/**
 * @param mass Mass to use to compute the inertia tensor of the collision shape
 */
Vector3 BoxShape::getLocalInertiaTensor(decimal mass) const {
    const decimal factor = (decimal(1.0) / decimal(3.0)) * mass;
    const decimal xSquare = mHalfExtents.x * mHalfExtents.x;
    const decimal ySquare = mHalfExtents.y * mHalfExtents.y;
    const decimal zSquare = mHalfExtents.z * mHalfExtents.z;
    return Vector3(factor * (ySquare + zSquare), factor * (xSquare + zSquare), factor * (xSquare + ySquare));
}

// Raycast method with feedback information
bool BoxShape::raycast(const Ray& ray, RaycastInfo& raycastInfo, Collider* collider, MemoryAllocator& allocator) const {

    Vector3 rayDirection = ray.point2 - ray.point1;
    decimal tMin = DECIMAL_SMALLEST;
    decimal tMax = DECIMAL_LARGEST;
    Vector3 normalDirection(decimal(0), decimal(0), decimal(0));
    Vector3 currentNormal;

    // For each of the three slabs
    for (int i=0; i<3; i++) {

        // If ray is parallel to the slab
        if (std::abs(rayDirection[i]) < MACHINE_EPSILON) {

            // If the ray's origin is not inside the slab, there is no hit
            if (ray.point1[i] > mHalfExtents[i] || ray.point1[i] < -mHalfExtents[i]) return false;
        }
        else {

            // Compute the intersection of the ray with the near and far plane of the slab
            decimal oneOverD = decimal(1.0) / rayDirection[i];
            decimal t1 = (-mHalfExtents[i] - ray.point1[i]) * oneOverD;
            decimal t2 = (mHalfExtents[i] - ray.point1[i]) * oneOverD;
            currentNormal[0] = (i == 0) ? -mHalfExtents[i] : decimal(0.0);
            currentNormal[1] = (i == 1) ? -mHalfExtents[i] : decimal(0.0);
            currentNormal[2] = (i == 2) ? -mHalfExtents[i] : decimal(0.0);

            // Swap t1 and t2 if need so that t1 is intersection with near plane and
            // t2 with far plane
            if (t1 > t2) {
                std::swap(t1, t2);
                currentNormal = -currentNormal;
            }

            // Compute the intersection of the of slab intersection interval with previous slabs
            if (t1 > tMin) {
                tMin = t1;
                normalDirection = currentNormal;
            }
            tMax = std::min(tMax, t2);

            // If tMin is larger than the maximum raycasting fraction, we return no hit
            if (tMin > ray.maxFraction) return false;

            // If the slabs intersection is empty, there is no hit
            if (tMin > tMax) return false;
        }
    }

    // If tMin is negative, we return no hit
    if (tMin < decimal(0.0) || tMin > ray.maxFraction) return false;

    // The ray intersects the three slabs, we compute the hit point
    Vector3 localHitPoint = ray.point1 + tMin * rayDirection;

    raycastInfo.body = collider->getBody();
    raycastInfo.collider = collider;
    raycastInfo.hitFraction = tMin;
    raycastInfo.worldPoint = localHitPoint;
    raycastInfo.worldNormal = normalDirection;

    return true;
}
