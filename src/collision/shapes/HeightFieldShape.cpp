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
#include <reactphysics3d/collision/shapes/HeightFieldShape.h>
#include <reactphysics3d/collision/RaycastInfo.h>
#include <reactphysics3d/utils/Profiler.h>
#include <iostream>

using namespace reactphysics3d;

// Constructor
/**
 * @param heightField Pointer to a height-field
 * @param scaling Scaling factor that needs to be applied to the height-field
 * @param allocator Memory allocator used to allocated memory inside the HeightFieldShape
 */
HeightFieldShape::HeightFieldShape(HeightField* heightField, MemoryAllocator& allocator, const Vector3& scaling)
                 : ConcaveShape(CollisionShapeName::HEIGHTFIELD, allocator, scaling),
                   mHeightField(heightField) {

}

// Return the local bounds of the shape in x, y and z directions.
/**
 * @return The AABB with the min/max bounds of the shape
 */
AABB HeightFieldShape::getLocalBounds() const {
    AABB aabb = mHeightField->getBounds();
    aabb.applyScale(mScale);
    return aabb;
}

// Test collision with the triangles of the height field shape. The idea is to use the AABB
// of the body when need to test and see against which triangles of the height-field we need
// to test for collision. We compute the sub-grid points that are inside the other body's AABB
// and then for each rectangle in the sub-grid we generate two triangles that we use to test collision.
void HeightFieldShape::computeOverlappingTriangles(const AABB& localAABB, Array<Vector3>& triangleVertices,
                                                   Array<Vector3>& triangleVerticesNormals, Array<uint32>& shapeIds,
                                                   MemoryAllocator& /*allocator*/) const {

    RP3D_PROFILE("HeightFieldShape::computeOverlappingTriangles()", mProfiler);

   // Compute the non-scaled AABB
   Vector3 inverseScale(decimal(1.0) / mScale.x, decimal(1.0) / mScale.y, decimal(1.0) / mScale.z);
   AABB aabb(localAABB.getMin() * inverseScale, localAABB.getMax() * inverseScale);

   mHeightField->computeOverlappingTriangles(aabb, triangleVertices, triangleVerticesNormals, shapeIds, mScale);
}

// Raycast method with feedback information
/// Note that only the first triangle hit by the ray in the mesh will be returned, even if
/// the ray hits many triangles.
bool HeightFieldShape::raycast(const Ray& ray, RaycastInfo& raycastInfo, Collider* collider, MemoryAllocator& allocator) const {

    RP3D_PROFILE("HeightFieldShape::raycast()", mProfiler);

    // Apply the height-field scale inverse scale factor because the mesh is stored without scaling
    // inside the dynamic AABB tree
    const Vector3 inverseScale(decimal(1.0) / mScale.x, decimal(1.0) / mScale.y, decimal(1.0) / mScale.z);
    Ray scaledRay(ray.point1 * inverseScale, ray.point2 * inverseScale, ray.maxFraction);

    if (mHeightField->raycast(scaledRay, raycastInfo, collider, getRaycastTestType(), allocator)) {

        // Scale back the contact point because we used a ray scale with inverse height-field scale
        raycastInfo.worldPoint = raycastInfo.worldPoint * mScale;

        return true;
    }

    return false;
}

// Return the string representation of the shape
std::string HeightFieldShape::to_string() const {

    std::stringstream ss;

    ss << "HeightFieldShape{" << std::endl;

    ss << "scaling=" << mScale.to_string() << std::endl;
    ss << ", HeightField=" << mHeightField->to_string() << std::endl;
    ss << "}";

    return ss.str();
}
