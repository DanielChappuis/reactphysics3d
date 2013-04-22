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
#include "CollisionShape.h"

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor
CollisionShape::CollisionShape(CollisionShapeType type)
               : mType(type), mNbSimilarCreatedShapes(0) {
    
}

// Private copy-constructor
CollisionShape::CollisionShape(const CollisionShape& shape)
               : mType(shape.mType), mNbSimilarCreatedShapes(shape.mNbSimilarCreatedShapes){

}

// Destructor
CollisionShape::~CollisionShape() {
    assert(mNbSimilarCreatedShapes == 0);
}

// Update the AABB of a body using its collision shape
inline void CollisionShape::updateAABB(AABB& aabb, const Transform& transform) {

    // Get the local extents in x,y and z direction
    Vector3 extents = getLocalExtents(OBJECT_MARGIN);

    // Rotate the local extents according to the orientation of the body
    Matrix3x3 worldAxis = transform.getOrientation().getMatrix().getAbsoluteMatrix();
    Vector3 worldExtents = Vector3(worldAxis.getColumn(0).dot(extents),
                                   worldAxis.getColumn(1).dot(extents),
                                   worldAxis.getColumn(2).dot(extents));

    // Compute the minimum and maximum coordinates of the rotated extents
    Vector3 minCoordinates = transform.getPosition() - worldExtents;
    Vector3 maxCoordinates = transform.getPosition() + worldExtents;

    // Update the AABB with the new minimum and maximum coordinates
    aabb.setMin(minCoordinates);
    aabb.setMax(maxCoordinates);
}
