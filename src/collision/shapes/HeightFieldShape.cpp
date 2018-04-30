/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2018 Daniel Chappuis                                       *
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
#include "collision/RaycastInfo.h"
#include "utils/Profiler.h"

using namespace reactphysics3d;

// Constructor
/**
 * @param nbGridColumns Number of columns in the grid of the height field
 * @param nbGridRows Number of rows in the grid of the height field
 * @param minHeight Minimum height value of the height field
 * @param maxHeight Maximum height value of the height field
 * @param heightFieldData Pointer to the first height value data (note that values are shared and not copied)
 * @param dataType Data type for the height values (int, float, double)
 * @param upAxis Integer representing the up axis direction (0 for x, 1 for y and 2 for z)
 * @param integerHeightScale Scaling factor used to scale the height values (only when height values type is integer)
 */
HeightFieldShape::HeightFieldShape(int nbGridColumns, int nbGridRows, decimal minHeight, decimal maxHeight,
                                   const void* heightFieldData, HeightDataType dataType, int upAxis,
                                   decimal integerHeightScale, const Vector3& scaling)
                 : ConcaveShape(CollisionShapeName::HEIGHTFIELD), mNbColumns(nbGridColumns), mNbRows(nbGridRows),
                   mWidth(nbGridColumns - 1), mLength(nbGridRows - 1), mMinHeight(minHeight),
                   mMaxHeight(maxHeight), mUpAxis(upAxis), mIntegerHeightScale(integerHeightScale),
                   mHeightDataType(dataType), mScaling(scaling) {

    assert(nbGridColumns >= 2);
    assert(nbGridRows >= 2);
    assert(mWidth >= 1);
    assert(mLength >= 1);
    assert(minHeight <= maxHeight);
    assert(upAxis == 0 || upAxis == 1 || upAxis == 2);

    mHeightFieldData = heightFieldData;

    decimal halfHeight = (mMaxHeight - mMinHeight) * decimal(0.5);
    assert(halfHeight >= 0);

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

// Test collision with the triangles of the height field shape. The idea is to use the AABB
// of the body when need to test and see against which triangles of the height-field we need
// to test for collision. We compute the sub-grid points that are inside the other body's AABB
// and then for each rectangle in the sub-grid we generate two triangles that we use to test collision.
void HeightFieldShape::testAllTriangles(TriangleCallback& callback, const AABB& localAABB) const {

   // Compute the non-scaled AABB
   Vector3 inverseScaling(decimal(1.0) / mScaling.x, decimal(1.0) / mScaling.y, decimal(1.0) / mScaling.z);
   AABB aabb(localAABB.getMin() * inverseScaling, localAABB.getMax() * inverseScaling);

   // Compute the integer grid coordinates inside the area we need to test for collision
   int minGridCoords[3];
   int maxGridCoords[3];
   computeMinMaxGridCoordinates(minGridCoords, maxGridCoords, aabb);

   // Compute the starting and ending coords of the sub-grid according to the up axis
   int iMin = 0;
   int iMax = 0;
   int jMin = 0;
   int jMax = 0;
   switch(mUpAxis) {
        case 0 : iMin = clamp(minGridCoords[1], 0, mNbColumns - 1);
                 iMax = clamp(maxGridCoords[1], 0, mNbColumns - 1);
                 jMin = clamp(minGridCoords[2], 0, mNbRows - 1);
                 jMax = clamp(maxGridCoords[2], 0, mNbRows - 1);
                 break;
        case 1 : iMin = clamp(minGridCoords[0], 0, mNbColumns - 1);
                 iMax = clamp(maxGridCoords[0], 0, mNbColumns - 1);
                 jMin = clamp(minGridCoords[2], 0, mNbRows - 1);
                 jMax = clamp(maxGridCoords[2], 0, mNbRows - 1);
                 break;
        case 2 : iMin = clamp(minGridCoords[0], 0, mNbColumns - 1);
                 iMax = clamp(maxGridCoords[0], 0, mNbColumns - 1);
                 jMin = clamp(minGridCoords[1], 0, mNbRows - 1);
                 jMax = clamp(maxGridCoords[1], 0, mNbRows - 1);
                 break;
   }

   assert(iMin >= 0 && iMin < mNbColumns);
   assert(iMax >= 0 && iMax < mNbColumns);
   assert(jMin >= 0 && jMin < mNbRows);
   assert(jMax >= 0 && jMax < mNbRows);

   // For each sub-grid points (except the last ones one each dimension)
   for (int i = iMin; i < iMax; i++) {
       for (int j = jMin; j < jMax; j++) {

           // Compute the four point of the current quad
           const Vector3 p1 = getVertexAt(i, j);
           const Vector3 p2 = getVertexAt(i, j + 1);
           const Vector3 p3 = getVertexAt(i + 1, j);
           const Vector3 p4 = getVertexAt(i + 1, j + 1);

           // Generate the first triangle for the current grid rectangle
           Vector3 trianglePoints[3] = {p1, p2, p3};

           // Compute the triangle normal
           Vector3 triangle1Normal = (p2 - p1).cross(p3 - p1).getUnit();

           // Use the triangle face normal as vertices normals (this is an aproximation. The correct
           // solution would be to compute all the normals of the neighbor triangles and use their
           // weighted average (with incident angle as weight) at the vertices. However, this solution
           // seems too expensive (it requires to compute the normal of all neighbor triangles instead
           // and compute the angle of incident edges with asin(). Maybe we could also precompute the
           // vertices normal at the HeightFieldShape constructor but it will require extra memory to
           // store them.
           Vector3 verticesNormals1[3] = {triangle1Normal, triangle1Normal, triangle1Normal};

           // Test collision against the first triangle
           callback.testTriangle(trianglePoints, verticesNormals1, computeTriangleShapeId(i, j, 0));

           // Generate the second triangle for the current grid rectangle
           trianglePoints[0] = p3;
           trianglePoints[1] = p2;
           trianglePoints[2] = p4;

           // Compute the triangle normal
           Vector3 triangle2Normal = (p2 - p3).cross(p4 - p3).getUnit();

           // Use the triangle face normal as vertices normals (this is an aproximation. The correct
           // solution would be to compute all the normals of the neighbor triangles and use their
           // weighted average (with incident angle as weight) at the vertices. However, this solution
           // seems too expensive (it requires to compute the normal of all neighbor triangles instead
           // and compute the angle of incident edges with asin(). Maybe we could also precompute the
           // vertices normal at the HeightFieldShape constructor but it will require extra memory to
           // store them.
           Vector3 verticesNormals2[3] = {triangle2Normal, triangle2Normal, triangle2Normal};

           // Test collision against the second triangle
           callback.testTriangle(trianglePoints, verticesNormals2, computeTriangleShapeId(i, j, 1));
       }
   }
}

// Compute the min/max grid coords corresponding to the intersection of the AABB of the height field and
// the AABB to collide
void HeightFieldShape::computeMinMaxGridCoordinates(int* minCoords, int* maxCoords, const AABB& aabbToCollide) const {

    // Clamp the min/max coords of the AABB to collide inside the height field AABB
    Vector3 minPoint = Vector3::max(aabbToCollide.getMin(), mAABB.getMin());
    minPoint = Vector3::min(minPoint, mAABB.getMax());

    Vector3 maxPoint = Vector3::min(aabbToCollide.getMax(), mAABB.getMax());
    maxPoint = Vector3::max(maxPoint, mAABB.getMin());

    // Translate the min/max points such that the we compute grid points from [0 ... mNbWidthGridPoints]
    // and from [0 ... mNbLengthGridPoints] because the AABB coordinates range are [-mWdith/2 ... mWidth/2]
    // and [-mLength/2 ... mLength/2]
    const Vector3 translateVec = mAABB.getExtent() * decimal(0.5);
    minPoint += translateVec;
    maxPoint += translateVec;

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
bool HeightFieldShape::raycast(const Ray& ray, RaycastInfo& raycastInfo, ProxyShape* proxyShape, MemoryAllocator& allocator) const {

    // TODO : Implement raycasting without using an AABB for the ray
    //        but using a dynamic AABB tree or octree instead

    RP3D_PROFILE("HeightFieldShape::raycast()", mProfiler);

    TriangleOverlapCallback triangleCallback(ray, proxyShape, raycastInfo, *this, allocator);

#ifdef IS_PROFILING_ACTIVE

	// Set the profiler
	triangleCallback.setProfiler(mProfiler);

#endif

    // Compute the AABB for the ray
    const Vector3 rayEnd = ray.point1 + ray.maxFraction * (ray.point2 - ray.point1);
    const AABB rayAABB(Vector3::min(ray.point1, rayEnd), Vector3::max(ray.point1, rayEnd));

    testAllTriangles(triangleCallback, rayAABB);

    return triangleCallback.getIsHit();
}

// Return the vertex (local-coordinates) of the height field at a given (x,y) position
Vector3 HeightFieldShape::getVertexAt(int x, int y) const {

    // Get the height value
    const decimal height = getHeightAt(x, y);

    // Height values origin
    const decimal heightOrigin = -(mMaxHeight - mMinHeight) * decimal(0.5) - mMinHeight;

    Vector3 vertex;
    switch (mUpAxis) {
        case 0: vertex = Vector3(heightOrigin + height, -mWidth * decimal(0.5) + x, -mLength * decimal(0.5) + y);
                break;
        case 1: vertex = Vector3(-mWidth * decimal(0.5) + x, heightOrigin + height, -mLength * decimal(0.5) + y);
                break;
        case 2: vertex = Vector3(-mWidth * decimal(0.5) + x, -mLength * decimal(0.5) + y, heightOrigin + height);
                break;
        default: assert(false);
    }

    assert(mAABB.contains(vertex));

    return vertex * mScaling;
}

// Raycast test between a ray and a triangle of the heightfield
void TriangleOverlapCallback::testTriangle(const Vector3* trianglePoints, const Vector3* verticesNormals, uint shapeId) {

    // Create a triangle collision shape
    TriangleShape triangleShape(trianglePoints, verticesNormals, shapeId, mAllocator);
    triangleShape.setRaycastTestType(mHeightFieldShape.getRaycastTestType());

#ifdef IS_PROFILING_ACTIVE

	// Set the profiler to the triangle shape
	triangleShape.setProfiler(mProfiler);

#endif

    // Ray casting test against the collision shape
    RaycastInfo raycastInfo;
    bool isTriangleHit = triangleShape.raycast(mRay, raycastInfo, mProxyShape, mAllocator);

    // If the ray hit the collision shape
    if (isTriangleHit && raycastInfo.hitFraction <= mSmallestHitFraction) {

        assert(raycastInfo.hitFraction >= decimal(0.0));

        mRaycastInfo.body = raycastInfo.body;
        mRaycastInfo.proxyShape = raycastInfo.proxyShape;
        mRaycastInfo.hitFraction = raycastInfo.hitFraction;
        mRaycastInfo.worldPoint = raycastInfo.worldPoint;
        mRaycastInfo.worldNormal = raycastInfo.worldNormal;
        mRaycastInfo.meshSubpart = -1;
        mRaycastInfo.triangleIndex = -1;

        mSmallestHitFraction = raycastInfo.hitFraction;
        mIsHit = true;
    }
}

// Return the string representation of the shape
std::string HeightFieldShape::to_string() const {

    std::stringstream ss;

    ss << "HeightFieldShape{" << std::endl;

    ss << "nbColumns=" << mNbColumns << std::endl;
    ss << ", nbRows=" << mNbRows << std::endl;
    ss << ", width=" << mWidth << std::endl;
    ss << ", length=" << mLength << std::endl;
    ss << ", minHeight=" << mMinHeight << std::endl;
    ss << ", maxHeight=" << mMaxHeight << std::endl;
    ss << ", upAxis=" << mUpAxis << std::endl;
    ss << ", integerHeightScale=" << mIntegerHeightScale << std::endl;
    ss << "}";

    return ss.str();
}
