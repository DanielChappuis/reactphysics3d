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
#include <reactphysics3d/collision/HeightField.h>
#include <reactphysics3d/collision/RaycastInfo.h>
#include <reactphysics3d/collision/shapes/TriangleShape.h>
#include <reactphysics3d/utils/Profiler.h>
#include <reactphysics3d/utils/Message.h>
#include <iostream>
#include <vector>

using namespace reactphysics3d;

// Constructor
HeightField::HeightField(MemoryAllocator& allocator, HalfEdgeStructure& triangleHalfEdgeStructure)
            : mAllocator(allocator), mHeightFieldData(allocator),
              mTriangleHalfEdgeStructure(triangleHalfEdgeStructure) {

}

// Initialize the height-field
bool HeightField::init(int nbGridColumns, int nbGridRows,
                       const void* heightFieldData, HeightDataType dataType,
                       std::vector<Message>& messages, decimal integerHeightScale) {

    bool isValid = true;

    if (nbGridColumns < 2 || nbGridRows < 2) {

        // Add a warning message for the user
        messages.push_back(Message("The number of grid columns and grid rows must be at least two", Message::Type::Error));
        return false;
    }

    mHeightFieldData.reserve(nbGridColumns * nbGridRows);
    mHeightFieldData.addWithoutInit(nbGridColumns * nbGridRows);

    mNbColumns = nbGridColumns;
    mNbRows = nbGridRows;
    mWidth = static_cast<decimal>(nbGridColumns - 1);
    mLength = static_cast<decimal>(nbGridRows - 1);
    mIntegerHeightScale = integerHeightScale;
    mHeightDataType = dataType;

    // Copy the height values from the user into the height-field
    copyData(heightFieldData);

    assert(mMinHeight <= mMaxHeight);

    const decimal halfHeight = (mMaxHeight - mMinHeight) * decimal(0.5);
    assert(halfHeight >= 0);

    assert(mWidth >= 1);
    assert(mLength >= 1);

    // Compute the local AABB of the height field
    mBounds.setMin(Vector3(-mWidth * decimal(0.5), -halfHeight, -mLength * decimal(0.5)));
    mBounds.setMax(Vector3(mWidth * decimal(0.5), halfHeight, mLength * decimal(0.5)));

    assert(mHeightFieldData.size() == mNbRows * mNbColumns);

    return isValid;
}

// Copy the data from the user into the height-field array
void HeightField::copyData(const void* heightFieldData) {

    // For each height value
    for (uint32 x=0; x < mNbColumns; x++) {

        for (uint32 y=0; y < mNbRows; y++) {

            decimal height = 0.0;

            switch(mHeightDataType) {
                case HeightDataType::HEIGHT_FLOAT_TYPE:
                    height = decimal(((float*)heightFieldData)[y * mNbColumns + x]);
                    break;
                case HeightDataType::HEIGHT_DOUBLE_TYPE:
                    height = decimal(((double*)heightFieldData)[y * mNbColumns + x]);
                    break;
                case HeightDataType::HEIGHT_INT_TYPE:
                    height = decimal(((int*)heightFieldData)[y * mNbColumns + x] * mIntegerHeightScale);
                    break;
                default:
                    assert(false);  // This should never happen
            }

            mHeightFieldData[y * mNbColumns + x] = height;

            if (x == 0 && y == 0) {
               mMinHeight = height;
               mMaxHeight = height;
            }

            // Compute minimum height
            if (height < mMinHeight) {
                mMinHeight = height;
            }

            // Compute maximum height
            if (height > mMaxHeight) {
                mMaxHeight = height;
            }
        }
    }

    // Compute the height origin
    mHeightOrigin = -(mMaxHeight - mMinHeight) * decimal(0.5) - mMinHeight;
}

// Test collision with the triangles of the height field shape. The idea is to use the AABB
// of the body when need to test and see against which triangles of the height-field we need
// to test for collision. We compute the sub-grid points that are inside the other body's AABB
// and then for each rectangle in the sub-grid we generate two triangles that we use to test collision.
void HeightField::computeOverlappingTriangles(const AABB& aabb, Array<Vector3>& triangleVertices,
                                              Array<Vector3>& triangleVerticesNormals,
                                              Array<uint32>& shapeIds, const Vector3& scale) const {

    RP3D_PROFILE("HeightField::computeOverlappingTriangles()", mProfiler);

    // Compute the integer grid coordinates inside the area we need to test for collision
    uint32 minGridCoords[3];
    uint32 maxGridCoords[3];
    computeMinMaxGridCoordinates(minGridCoords, maxGridCoords, aabb);

    // Compute the starting and ending coords of the sub-grid according to the up axis
    uint32 iMin = clamp(minGridCoords[0], 0, mNbColumns - 1);
    uint32 iMax = clamp(maxGridCoords[0], 0, mNbColumns - 1);
    uint32 jMin = clamp(minGridCoords[2], 0, mNbRows - 1);
    uint32 jMax = clamp(maxGridCoords[2], 0, mNbRows - 1);

   assert(iMin < mNbColumns);
   assert(iMax < mNbColumns);
   assert(jMin < mNbRows);
   assert(jMax < mNbRows);

   // For each sub-grid points (except the last ones one each dimension)
   for (uint32 i = iMin; i < iMax; i++) {
       for (uint32 j = jMin; j < jMax; j++) {

           // Compute the four point of the current quad
           const Vector3 p1 = getVertexAt(i, j) * scale;
           const Vector3 p2 = getVertexAt(i, j + 1) * scale;
           const Vector3 p3 = getVertexAt(i + 1, j) * scale;
           const Vector3 p4 = getVertexAt(i + 1, j + 1) * scale;

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
void HeightField::computeMinMaxGridCoordinates(uint32* minCoords, uint32* maxCoords, const AABB& aabbToCollide) const {

    // Clamp the min/max coords of the AABB to collide inside the height field AABB
    Vector3 minPoint = Vector3::max(aabbToCollide.getMin(), mBounds.getMin());
    minPoint = Vector3::min(minPoint, mBounds.getMax());

    Vector3 maxPoint = Vector3::min(aabbToCollide.getMax(), mBounds.getMax());
    maxPoint = Vector3::max(maxPoint, mBounds.getMin());

    // Translate the min/max points such that the we compute grid points from [0 ... mNbWidthGridPoints]
    // and from [0 ... mNbLengthGridPoints] because the AABB coordinates range are [-mWdith/2 ... mWidth/2]
    // and [-mLength/2 ... mLength/2]
    const Vector3 translateVec = mBounds.getExtent() * decimal(0.5);
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
bool HeightField::raycast(const Ray& ray, RaycastInfo& raycastInfo, Collider* collider, TriangleRaycastSide testSide,
                          MemoryAllocator& allocator) const {

    RP3D_PROFILE("HeightField::raycast()", mProfiler);

    bool isHit = false;

    // Compute the grid coordinates where the ray is entering the AABB of the height field
    int32 i, j;
    Vector3 outHitGridPoint;
    if (computeEnteringRayGridCoordinates(ray, i, j, outHitGridPoint)) {

        const int32 nbCellsI = mNbColumns - 1;
        const int32 nbCellsJ = mNbRows - 1;

        const Vector3 aabbSize = mBounds.getExtent();

        const Vector3 rayDirection = ray.point2 - ray.point1;

        int32 stepI = rayDirection.x > 0 ? 1 : (rayDirection.x < 0 ? -1 : 0);
        int32 stepJ = rayDirection.z > 0 ? 1 : (rayDirection.z < 0 ? -1 : 0);
        decimal nextI = static_cast<decimal>(stepI >= 0 ? i + 1 : i);
        decimal nextJ = static_cast<decimal>(stepJ >= 0 ? j + 1 : j);
        decimal sizeI = aabbSize.x / nbCellsI;
        decimal sizeJ = aabbSize.z / nbCellsJ;
        decimal tMaxI = ((nextI * sizeI) - outHitGridPoint.x) / rayDirection.x;
        decimal tMaxJ = ((nextJ * sizeJ) - outHitGridPoint.z) / rayDirection.z;
        decimal tDeltaI = sizeI / std::abs(rayDirection.x);
        decimal tDeltaJ = sizeJ / std::abs(rayDirection.z);

        decimal smallestHitFraction = ray.maxFraction;

        while (i >= 0 && i < nbCellsI && j >= 0 && j < nbCellsJ) {

           // Compute the four point of the current quad
           const Vector3 p1 = getVertexAt(i, j);
           const Vector3 p2 = getVertexAt(i, j + 1);
           const Vector3 p3 = getVertexAt(i + 1, j);
           const Vector3 p4 = getVertexAt(i + 1, j + 1);

           // Raycast against the first triangle of the cell
           uint32 shapeId = computeTriangleShapeId(i, j, 0);
           isHit |= raycastTriangle(ray, p1, p2, p3, shapeId, collider, raycastInfo, smallestHitFraction, testSide, allocator);

           // Raycast against the second triangle of the cell
           shapeId = computeTriangleShapeId(i, j, 1);
           isHit |= raycastTriangle(ray, p3, p2, p4, shapeId, collider, raycastInfo, smallestHitFraction, testSide, allocator);

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
bool HeightField::raycastTriangle(const Ray& ray, const Vector3& p1, const Vector3& p2, const Vector3& p3, uint32 shapeId,
                                  Collider* collider, RaycastInfo& raycastInfo, decimal& smallestHitFraction,
                                  TriangleRaycastSide testSide, MemoryAllocator& allocator) const {

   // Generate the first triangle for the current grid rectangle
   Vector3 triangleVertices[3] = {p1, p2, p3};

    // Create a triangle collision shape
    TriangleShape triangleShape(triangleVertices, shapeId, mTriangleHalfEdgeStructure, allocator);
    triangleShape.setRaycastTestType(testSide);

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
        raycastInfo.triangleIndex = -1;

        smallestHitFraction = triangleRaycastInfo.hitFraction;

        return true;
    }

    return false;
}

// Compute the first grid cell of the heightfield intersected by a ray.
/// This method returns true if the ray hit the AABB of the height field and false otherwise
bool HeightField::computeEnteringRayGridCoordinates(const Ray& ray, int32& i, int32& j, Vector3& outHitGridPoint) const {

    decimal stepI, stepJ;
    const Vector3 aabbSize = mBounds.getExtent();

    assert(mNbColumns > 0);
    assert(mNbRows > 0);

    const int32 nbCellsI = mNbColumns - 1;
    const int32 nbCellsJ = mNbRows - 1;

    if (mBounds.raycast(ray, outHitGridPoint)) {

        // Map the hit point into the grid range [0, mNbColumns - 1], [0, mNbRows - 1]
        outHitGridPoint -= mBounds.getMin();

        stepI = aabbSize.x / nbCellsI;
        stepJ = aabbSize.z / nbCellsJ;
        i = clamp(int(outHitGridPoint.x / stepI), 0, nbCellsI - 1);
        j = clamp(int(outHitGridPoint.z / stepJ), 0, nbCellsJ - 1);

        assert(i >= 0 && i < nbCellsI);
        assert(i >= 0 && j < nbCellsJ);

        return true;
    }

    return false;
}

// Return the vertex (local-coordinates) of the height field at a given (x,y) position
Vector3 HeightField::getVertexAt(uint32 x, uint32 y) const {

    // Get the height value
    const decimal height = getHeightAt(x, y);

    const Vector3 vertex = Vector3(-mWidth * decimal(0.5) + x, mHeightOrigin + height, -mLength * decimal(0.5) + y);

    assert(mBounds.contains(vertex, decimal(0.0001)));

    return vertex;
}

// Return the string representation of the shape
std::string HeightField::to_string() const {

    std::stringstream ss;

    ss << "HeightFiel{" << std::endl;

    ss << "nbColumns=" << mNbColumns << std::endl;
    ss << ", nbRows=" << mNbRows << std::endl;
    ss << ", width=" << mWidth << std::endl;
    ss << ", length=" << mLength << std::endl;
    ss << ", minHeight=" << mMinHeight << std::endl;
    ss << ", maxHeight=" << mMaxHeight << std::endl;
    ss << ", integerHeightScale=" << mIntegerHeightScale << std::endl;
    ss << "}";

    return ss.str();
}
