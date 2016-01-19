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
#include "HeightFieldShape.h"

using namespace reactphysics3d;

// Constructor
// TODO : Add documentation to this constructor
HeightFieldShape::HeightFieldShape(int width, int length, int minHeight, int maxHeight,
                                   const void* heightFieldData, HeightDataType dataType, int upAxis,
                                   decimal integerHeightScale)
                 : ConcaveShape(CONCAVE_MESH), mWidth(width), mLength(length), mMinHeight(minHeight),
                   mMaxHeight(maxHeight), mUpAxis(upAxis), mIntegerHeightScale(integerHeightScale),
                   mHeightDataType(dataType) {

    assert(width >= 1);
    assert(length >= 1);
    assert(minHeight <= maxHeight);
    assert(upAxis == 0 || upAxis == 1 || upAxis == 2);

    mHeightFieldData = heightFieldData;

    decimal halfHeight = (mMaxHeight - mMinHeight) * decimal(0.5);
    assert(halfHeight > 0);

    // Compute the local AABB of the height field
    if (mUpAxis == 0) {
        mAABB.setMin(Vector3(-halfHeight, -mWidth * decimal(0.5), -mLength * decimal(0.5)));
        mAABB.setMax(Vector3(halfHeight, mWidth * decimal(0.5), mLength* decimal(0.5)));
    }
    else if (mUpAxis == 1) {
        mAABB.setMin(Vector3(-mWidth * decimal(0.5), -halfHeight, -mLength * decimal(0.5)));
        mAABB.setMax(Vector3(mWidth * decimal(0.5), halfHeight, mLength * decimal(0.5)));
    }
    else if (mUpAxis == 2) {
        mAABB.setMin(Vector3(-mWidth * decimal(0.5), -mLength * decimal(0.5), -halfHeight));
        mAABB.setMax(Vector3(mWidth * decimal(0.5), mLength * decimal(0.5), halfHeight));
    }
}

// Destructor
HeightFieldShape::~HeightFieldShape() {

}

// Return the local bounds of the shape in x, y and z directions.
// This method is used to compute the AABB of the box
/**
 * @param min The minimum bounds of the shape in local-space coordinates
 * @param max The maximum bounds of the shape in local-space coordinates
 */
void HeightFieldShape::getLocalBounds(Vector3& min, Vector3& max) const {
    min = mAABB.getMin() * mScaling;
    max = mAABB.getMax() * mScaling;
}

// Use a callback method on all triangles of the concave shape inside a given AABB
void HeightFieldShape::testAllTriangles(TriangleCallback& callback, const AABB& localAABB) const {

   // Compute the non-scaled AABB
   Vector3 inverseScaling(decimal(1.0) / mScaling.x, decimal(1.0) / mScaling.y, decimal(1.0) / mScaling.z);
   AABB aabb(localAABB.getMin() * inverseScaling, localAABB.getMax() * inverseScaling);

   // Compute the integer grid coordinates inside the area we need to test for collision
   int minGridCoords[3];
   int maxGridCoords[3];
   computeMinMaxGridCoordinates(minGridCoords, maxGridCoords, localAABB);


}

// Compute the min/max grid coords corresponding to the intersection of the AABB of the height field and
// the AABB to collide
void HeightFieldShape::computeMinMaxGridCoordinates(int* minCoords, int* maxCoords, const AABB& aabbToCollide) const {

    // Clamp the min/max coords of the AABB to collide inside the height field AABB
    Vector3 minPoint = Vector3::max(aabbToCollide.getMin(), mAABB.getMin());
    minPoint = Vector3::min(minPoint, mAABB.getMax());

    Vector3 maxPoint = Vector3::max(aabbToCollide.getMax(), mAABB.getMin());
    maxPoint = Vector3::min(maxPoint, mAABB.getMax());

    // Convert the floating min/max coords of the AABB into closest integer
    // grid values (note that we use the closest grid coordinate that is out
    // of the AABB)
    minCoords[0] = computeIntegerGridValue(minPoint.x) - 1;
    minCoords[1] = computeIntegerGridValue(minPoint.y) - 1;
    minCoords[2] = computeIntegerGridValue(minPoint.z) - 1;

    maxCoords[0] = computeIntegerGridValue(maxPoint.x) + 1;
    maxCoords[1] = computeIntegerGridValue(maxPoint.y) + 1;
    maxCoords[2] = computeIntegerGridValue(maxPoint.z) + 1;
}

// Raycast method with feedback information
/// Note that only the first triangle hit by the ray in the mesh will be returned, even if
/// the ray hits many triangles.
bool HeightFieldShape::raycast(const Ray& ray, RaycastInfo& raycastInfo, ProxyShape* proxyShape) const {

    PROFILE("HeightFieldShape::raycast()");

    // TODO : Implement this
}
