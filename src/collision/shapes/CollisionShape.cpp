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
#include <reactphysics3d/collision/shapes/CollisionShape.h>
#include <reactphysics3d/utils/Profiler.h>
#include <reactphysics3d/body/Body.h>
#include <reactphysics3d/collision/Collider.h>

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor
CollisionShape::CollisionShape(CollisionShapeName name, CollisionShapeType type, MemoryAllocator &allocator)
               : mType(type), mName(name), mId(0), mColliders(allocator) {

#ifdef IS_RP3D_PROFILING_ENABLED

        mProfiler = nullptr;
#endif

}

// Compute the transformed AABB of the collision shape in another space
// The technique is described in the book Real-Time Collision
// Detection by Christer Ericson. This can be used to compute AABB of the collision shape
// in world-space for instance
/**
 * @param transform Transform to use to for the space conversion
 * @return The transformed axis-aligned bounding box (AABB) of the collision shape
 */
AABB CollisionShape::computeTransformedAABB(const Transform& transform) const {

    RP3D_PROFILE("CollisionShape::computeAABB()", mProfiler);

    // Get the local bounds in x,y and z direction
    AABB aabb = getLocalBounds();

    const Vector3& translation = transform.getPosition();
    Matrix3x3 matrix = transform.getOrientation().getMatrix();
    Vector3 resultMin;
    Vector3 resultMax;

    // For each of the three axis
    for (int i=0; i<3; i++) {

        // Add translation component
        resultMin[i] = translation[i];
        resultMax[i] = translation[i];

        for (int j=0; j<3; j++) {
            decimal e = matrix[i][j] * aabb.getMin()[j];
            decimal f = matrix[i][j] * aabb.getMax()[j];

            if (e < f) {
                resultMin[i] += e;
                resultMax[i] += f;
            }
            else {
                resultMin[i] += f;
                resultMax[i] += e;
            }
        }
    }

    // Update the AABB with the new minimum and maximum coordinates
    aabb.setMin(resultMin);
    aabb.setMax(resultMax);

    return aabb;
}

/// Notify all the assign colliders that the size of the collision shape has changed
void CollisionShape::notifyColliderAboutChangedSize() {

    const uint32 nbColliders = static_cast<uint32>(mColliders.size());
    for (uint32 i=0; i < nbColliders; i++) {
        mColliders[i]->setHasCollisionShapeChangedSize(true);
    }
}
