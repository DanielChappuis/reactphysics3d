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
#include <reactphysics3d/collision/shapes/HeightFieldShape.h>
#include <reactphysics3d/collision/RaycastInfo.h>
#include <reactphysics3d/utils/Profiler.h>
#include <iostream>

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
                                   const void* heightFieldData, HeightDataType dataType, MemoryAllocator& allocator,
                                   HalfEdgeStructure& triangleHalfEdgeStructure, int upAxis,
                                   decimal integerHeightScale, const Vector3& scaling)
                 : ConcaveShape(CollisionShapeName::HEIGHTFIELD, allocator, scaling), mNbColumns(nbGridColumns), mNbRows(nbGridRows),
                   mWidth(static_cast<decimal>(nbGridColumns - 1)), mLength(static_cast<decimal>(nbGridRows - 1)), mMinHeight(minHeight),
                   mMaxHeight(maxHeight), mUpAxis(upAxis), mIntegerHeightScale(integerHeightScale),
                   mHeightDataType(dataType), mTriangleHalfEdgeStructure(triangleHalfEdgeStructure) {

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
    min = mAABB.getMin() * mScale;
    max = mAABB.getMax() * mScale;
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
           triangleVertices.add(p1);
           triangleVertices.add(p2);
           triangleVertices.add(p3);

           // Compute the triangle normal
           Vector3 triangle1Normal = (p2 - p1).cross(p3 - p1).getUnit();

           // Use the triangle face normal as vertices normals (this is an aproximation. The correct
           // solution would be to compute all the normals of the neighbor triangles and use their
           // weighted average (with incident angle as weight) at the vertices. However, this solution
           // seems too expensive (it requires to compute the normal of all neighbor triangles instead
           // and compute the angle of incident edges with asin(). Maybe we could also precompute the
           // vertices normal at the HeightFieldShape constructor but it will require extra memory to
           // store them.
           triangleVerticesNormals.add(triangle1Normal);
           triangleVerticesNormals.add(triangle1Normal);
           triangleVerticesNormals.add(triangle1Normal);

           // Compute the shape ID
           shapeIds.add(computeTriangleShapeId(i, j, 0));

           // Generate the second triangle for the current grid rectangle
           triangleVertices.add(p3);
           triangleVertices.add(p2);
           triangleVertices.add(p4);

           // Compute the triangle normal
           Vector3 triangle2Normal = (p2 - p3).cross(p4 - p3).getUnit();

           // Use the triangle face normal as vertices normals (this is an aproximation. The correct
           // solution would be to compute all the normals of the neighbor triangles and use their
           // weighted average (with incident angle as weight) at the vertices. However, this solution
           // seems too expensive (it requires to compute the normal of all neighbor triangles instead
           // and compute the angle of incident edges with asin(). Maybe we could also precompute the
           // vertices normal at the HeightFieldShape constructor but it will require extra memory to
           // store them.
           triangleVerticesNormals.add(triangle2Normal);
           triangleVerticesNormals.add(triangle2Normal);
           triangleVerticesNormals.add(triangle2Normal);

           // Compute the shape ID
           shapeIds.add(computeTriangleShapeId(i, j, 1));
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

    assert(minPoint.x >= 0);
    assert(minPoint.y >= 0);
    assert(minPoint.z >= 0);
    assert(maxPoint.x >= 0);
    assert(maxPoint.y >= 0);
    assert(maxPoint.z >= 0);

    // Convert the floating min/max coords of the AABB into closest integer
    // grid values (note that we use the closest grid coordinate that is out
    // of the AABB)
    minCoords[0] = static_cast<int>(minPoint.x + 0.5) - 1;
    minCoords[1] = static_cast<int>(minPoint.y + 0.5) - 1;
    minCoords[2] = static_cast<int>(minPoint.z + 0.5) - 1;

    maxCoords[0] = static_cast<int>(maxPoint.x + 0.5) + 1;
    maxCoords[1] = static_cast<int>(maxPoint.y + 0.5) + 1;
    maxCoords[2] = static_cast<int>(maxPoint.z + 0.5) + 1;
}

// Raycast method with feedback information
/// Note that only the first triangle hit by the ray in the mesh will be returned, even if
/// the ray hits many triangles.
bool HeightFieldShape::raycast(const Ray& ray, RaycastInfo& raycastInfo, Collider* collider, MemoryAllocator& allocator) const {

    RP3D_PROFILE("HeightFieldShape::raycast()", mProfiler);

    // Apply the concave mesh inverse scale factor because the mesh is stored without scaling
    // inside the dynamic AABB tree
    const Vector3 inverseScale(decimal(1.0) / mScale.x, decimal(1.0) / mScale.y, decimal(1.0) / mScale.z);
    Ray scaledRay(ray.point1 * inverseScale, ray.point2 * inverseScale, ray.maxFraction);

    bool isHit = false;

    // Compute the grid coordinates where the ray is entering the AABB of the height field
    int i, j;
    Vector3 outHitGridPoint;
    if (computeEnteringRayGridCoordinates(scaledRay, i, j, outHitGridPoint)) {

        const int nbCellsI = mNbColumns - 1;
        const int nbCellsJ = mNbRows - 1;

        const Vector3 aabbSize = mAABB.getExtent();

        const Vector3 rayDirection = scaledRay.point2 - scaledRay.point1;

        int stepI, stepJ;
        decimal tMaxI, tMaxJ, nextI, nextJ, tDeltaI, tDeltaJ, sizeI, sizeJ;

        switch(mUpAxis) {
            case 0 : stepI = rayDirection.y > 0 ? 1 : (rayDirection.y < 0 ? -1 : 0);
                     stepJ = rayDirection.z > 0 ? 1 : (rayDirection.z < 0 ? -1 : 0);
                     nextI = static_cast<decimal>(stepI >= 0 ? i + 1 : i);
                     nextJ = static_cast<decimal>(stepJ >= 0 ? j + 1 : j);
                     sizeI = aabbSize.y / nbCellsI;
                     sizeJ = aabbSize.z / nbCellsJ;
                     tMaxI = ((nextI * sizeI) - outHitGridPoint.y) / rayDirection.y;
                     tMaxJ = ((nextJ * sizeJ) - outHitGridPoint.z) / rayDirection.z;
                     tDeltaI = sizeI / std::abs(rayDirection.y);
                     tDeltaJ = sizeJ / std::abs(rayDirection.z);
                     break;
            case 1 : stepI = rayDirection.x > 0 ? 1 : (rayDirection.x < 0 ? -1 : 0);
                     stepJ = rayDirection.z > 0 ? 1 : (rayDirection.z < 0 ? -1 : 0);
                     nextI = static_cast<decimal>(stepI >= 0 ? i + 1 : i);
                     nextJ = static_cast<decimal>(stepJ >= 0 ? j + 1 : j);
                     sizeI = aabbSize.x / nbCellsI;
                     sizeJ = aabbSize.z / nbCellsJ;
                     tMaxI = ((nextI * sizeI) - outHitGridPoint.x) / rayDirection.x;
                     tMaxJ = ((nextJ * sizeJ) - outHitGridPoint.z) / rayDirection.z;
                     tDeltaI = sizeI / std::abs(rayDirection.x);
                     tDeltaJ = sizeJ / std::abs(rayDirection.z);
                     break;
            case 2 : stepI = rayDirection.x > 0 ? 1 : (rayDirection.x < 0 ? -1 : 0);
                     stepJ = rayDirection.y > 0 ? 1 : (rayDirection.y < 0 ? -1 : 0);
                     nextI = static_cast<decimal>(stepI >= 0 ? i + 1 : i);
                     nextJ = static_cast<decimal>(stepJ >= 0 ? j + 1 : j);
                     sizeI = aabbSize.x / nbCellsI;
                     sizeJ = aabbSize.y / nbCellsJ;
                     tMaxI = ((nextI * sizeI) - outHitGridPoint.x) / rayDirection.x;
                     tMaxJ = ((nextJ * sizeJ) - outHitGridPoint.y) / rayDirection.y;
                     tDeltaI = sizeI / std::abs(rayDirection.x);
                     tDeltaJ = sizeJ / std::abs(rayDirection.y);
                     break;
        }

        decimal smallestHitFraction = ray.maxFraction;

        while (i >= 0 && i < nbCellsI && j >= 0 && j < nbCellsJ) {

           // Compute the four point of the current quad
           const Vector3 p1 = getVertexAt(i, j);
           const Vector3 p2 = getVertexAt(i, j + 1);
           const Vector3 p3 = getVertexAt(i + 1, j);
           const Vector3 p4 = getVertexAt(i + 1, j + 1);

           // Raycast against the first triangle of the cell
           uint32 shapeId = computeTriangleShapeId(i, j, 0);
           isHit |= raycastTriangle(ray, p1, p2, p3, shapeId, collider, raycastInfo, smallestHitFraction, allocator);

           // Raycast against the second triangle of the cell
           shapeId = computeTriangleShapeId(i, j, 1);
           isHit |= raycastTriangle(ray, p3, p2, p4, shapeId, collider, raycastInfo, smallestHitFraction, allocator);

           if (stepI == 0 && stepJ == 0) break;

           if (tMaxI < tMaxJ) {
                tMaxI += tDeltaI;
                i += stepI;
            }
            else {
                tMaxJ += tDeltaJ;
                j += stepJ;
            }
        }
    }

    return isHit;
}

// Raycast a single triangle of the height-field
bool HeightFieldShape::raycastTriangle(const Ray& ray, const Vector3& p1, const Vector3& p2, const Vector3& p3, uint32 shapeId,
                                       Collider* collider, RaycastInfo& raycastInfo, decimal& smallestHitFraction, MemoryAllocator& allocator) const {

   // Generate the first triangle for the current grid rectangle
   Vector3 triangleVertices[3] = {p1, p2, p3};

    // Create a triangle collision shape
    TriangleShape triangleShape(triangleVertices, shapeId, mTriangleHalfEdgeStructure, allocator);
    triangleShape.setRaycastTestType(getRaycastTestType());

#ifdef IS_RP3D_PROFILING_ENABLED


    // Set the profiler to the triangle shape
    triangleShape.setProfiler(mProfiler);

#endif

    // Ray casting test against the collision shape
    RaycastInfo triangleRaycastInfo;
    bool isTriangleHit = triangleShape.raycast(ray, triangleRaycastInfo, collider, allocator);

    // If the ray hit the collision shape
    if (isTriangleHit && triangleRaycastInfo.hitFraction <= smallestHitFraction) {

        assert(triangleRaycastInfo.hitFraction >= decimal(0.0));

        raycastInfo.body = triangleRaycastInfo.body;
        raycastInfo.collider = triangleRaycastInfo.collider;
        raycastInfo.hitFraction = triangleRaycastInfo.hitFraction;
        raycastInfo.worldPoint = triangleRaycastInfo.worldPoint;
        raycastInfo.worldNormal = triangleRaycastInfo.worldNormal;
        raycastInfo.meshSubpart = -1;
        raycastInfo.triangleIndex = -1;

        smallestHitFraction = triangleRaycastInfo.hitFraction;

        return true;
    }

    return false;
}

// Compute the first grid cell of the heightfield intersected by a ray.
/// This method returns true if the ray hit the AABB of the height field and false otherwise
bool HeightFieldShape::computeEnteringRayGridCoordinates(const Ray& ray, int& i, int& j, Vector3& outHitGridPoint) const {
    
    decimal stepI, stepJ;
    const Vector3 aabbSize = mAABB.getExtent();

    assert(mNbColumns > 0);
    assert(mNbRows > 0);

    const int nbCellsI = mNbColumns - 1;
    const int nbCellsJ = mNbRows - 1;

    if (mAABB.raycast(ray, outHitGridPoint)) {

        // Map the hit point into the grid range [0, mNbColumns - 1], [0, mNbRows - 1]
        outHitGridPoint -= mAABB.getMin();

        switch(mUpAxis) {
            case 0 : stepI = aabbSize.y / nbCellsI;
                     stepJ = aabbSize.z / nbCellsJ;
                     i = clamp(int(outHitGridPoint.y / stepI), 0, nbCellsI - 1);
                     j = clamp(int(outHitGridPoint.z / stepJ), 0, nbCellsJ - 1);
                     break;
            case 1 : stepI = aabbSize.x / nbCellsI;
                     stepJ = aabbSize.z / nbCellsJ;
                     i = clamp(int(outHitGridPoint.x / stepI), 0, nbCellsI - 1);
                     j = clamp(int(outHitGridPoint.z / stepJ), 0, nbCellsJ - 1);
                     break;
            case 2 : stepI = aabbSize.x / nbCellsI;
                     stepJ = aabbSize.y / nbCellsJ;
                     i = clamp(int(outHitGridPoint.x / stepI), 0, nbCellsI - 1);
                     j = clamp(int(outHitGridPoint.y / stepJ), 0, nbCellsJ - 1);
                     break;
        }

        assert(i >= 0 && i < nbCellsI);
        assert(j >= 0 && j < nbCellsJ);
        return true;
    }
    
    return false;
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

    return vertex * mScale;
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
