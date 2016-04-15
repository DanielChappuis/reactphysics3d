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
#include "CollisionShape.h"
#include "engine/Profiler.h"
#include "body/CollisionBody.h"

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor
CollisionShape::CollisionShape(CollisionShapeType type) : mType(type), mScaling(1.0, 1.0, 1.0) {
    
}

// Destructor
CollisionShape::~CollisionShape() {

}

// Compute the world-space AABB of the collision shape given a transform
/**
 * @param[out] aabb The axis-aligned bounding box (AABB) of the collision shape
 *                  computed in world-space coordinates
 * @param transform Transform used to compute the AABB of the collision shape
 */
void CollisionShape::computeAABB(AABB& aabb, const Transform& transform) const {

    PROFILE("CollisionShape::computeAABB()");

    // Get the local bounds in x,y and z direction
    Vector3 minBounds;
    Vector3 maxBounds;
    getLocalBounds(minBounds, maxBounds);

    // Rotate the local bounds according to the orientation of the body
    Matrix3x3 worldAxis = transform.getOrientation().getMatrix().getAbsoluteMatrix();
    Vector3 worldMinBounds(worldAxis.getColumn(0).dot(minBounds),
                           worldAxis.getColumn(1).dot(minBounds),
                           worldAxis.getColumn(2).dot(minBounds));
    Vector3 worldMaxBounds(worldAxis.getColumn(0).dot(maxBounds),
                           worldAxis.getColumn(1).dot(maxBounds),
                           worldAxis.getColumn(2).dot(maxBounds));

    // Compute the minimum and maximum coordinates of the rotated extents
    Vector3 minCoordinates = transform.getPosition() + worldMinBounds;
    Vector3 maxCoordinates = transform.getPosition() + worldMaxBounds;

    // Update the AABB with the new minimum and maximum coordinates
    aabb.setMin(minCoordinates);
    aabb.setMax(maxCoordinates);
}
