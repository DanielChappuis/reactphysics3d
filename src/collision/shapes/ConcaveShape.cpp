/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2022 Daniel Chappuis                                       *
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
#include <reactphysics3d/collision/shapes/ConcaveShape.h>

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor
ConcaveShape::ConcaveShape(CollisionShapeName name, MemoryAllocator& allocator, const Vector3& scaling)
             : CollisionShape(name, CollisionShapeType::CONCAVE_SHAPE, allocator), mRaycastTestType(TriangleRaycastSide::FRONT),
               mScale(scaling) {

}

// Compute and return the volume of the collision shape
/// Note that we approximate the volume of a concave shape with the volume of its AABB
decimal ConcaveShape::getVolume() const {
    Vector3 minBounds, maxBounds;

    // Compute the local bounds
    getLocalBounds(minBounds, maxBounds);

    const decimal lengthX = maxBounds.x - minBounds.x;
    const decimal lengthY = maxBounds.y - minBounds.y;
    const decimal lengthZ = maxBounds.z - minBounds.z;

    // Approximate the volume of the concave shape as the volume of its AABB
    return lengthX * lengthY * lengthZ;
}
