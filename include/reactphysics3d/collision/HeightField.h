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

#ifndef REACTPHYSICS3D_HEIGHT_FIELD_H
#define REACTPHYSICS3D_HEIGHT_FIELD_H

// Libraries
#include <cassert>
#include <reactphysics3d/mathematics/mathematics.h>
#include <reactphysics3d/collision/shapes/AABB.h>
#include <reactphysics3d/collision/shapes/TriangleShape.h>
#include <reactphysics3d/collision/Collider.h>
#include <reactphysics3d/collision/HalfEdgeStructure.h>

namespace reactphysics3d {

// Declarations
struct Message;

// Class HeightField
/**
 * This class represents a static height field that can be used to represent
 * a terrain. The height field is made of a grid with rows and columns with a
 * height value at each grid point. Note that the height values are copied into the shape.
 * The height values can be of type integer, float or double.
 * Note that the HeightField will be re-centered based on its AABB. It means
 * that for instance, if the minimum height value is -200 and the maximum value is 400, the final
 * minimum height of the field in the simulation will be -300 and the maximum height will be 300.
 */
class HeightField {

    public:

        /// Data type for the height data of the height field
        enum class HeightDataType {HEIGHT_FLOAT_TYPE, HEIGHT_DOUBLE_TYPE, HEIGHT_INT_TYPE};

    protected:

        // -------------------- Attributes -------------------- //

        /// Reference to a memory allocator
        MemoryAllocator& mAllocator;

        /// Number of columns in the grid of the height field (along the local x direction)
        uint32 mNbColumns;

        /// Number of rows in the grid of the height field (along the local z direction)
        uint32 mNbRows;

        /// Height field width
        decimal mWidth;

        /// Height field length
        decimal mLength;

        /// Minimum height value of the height field
        decimal mMinHeight;

        /// Maximum height value of the height field
        decimal mMaxHeight;

        /// Height origin
        decimal mHeightOrigin;

        /// Height values scale for height field with integer height values
        decimal mIntegerHeightScale;

        /// Data type of the height values
        HeightDataType mHeightDataType;

        /// Array of data with all the height values of the height field
        Array<decimal> mHeightFieldData;

        /// Local bounds of the height field
        AABB mBounds;

        /// Reference to the half-edge structure
        HalfEdgeStructure& mTriangleHalfEdgeStructure;

#ifdef IS_RP3D_PROFILING_ENABLED

        /// Pointer to the profiler
        Profiler* mProfiler;

#endif

        // -------------------- Methods -------------------- //

        /// Constructor
        HeightField(MemoryAllocator& allocator, HalfEdgeStructure& triangleHalfEdgeStructure);

        bool init(int nbGridColumns, int nbGridRows, const void* heightFieldData,
                  HeightDataType dataType, std::vector<Message>& messages, decimal integerHeightScale = 1.0f);

        /// Copy the data from the user into the height-field array
        void copyData(const void* heightFieldData);

        /// Raycast a single triangle of the height-field
        bool raycastTriangle(const Ray& ray, const Vector3& p1, const Vector3& p2, const Vector3& p3, uint32 shapeId,
                             Collider* collider, RaycastInfo& raycastInfo, decimal& smallestHitFraction,
                             TriangleRaycastSide testSide, MemoryAllocator& allocator) const;

        /// Raycast method with feedback information
        bool raycast(const Ray& ray, RaycastInfo& raycastInfo, Collider* collider, TriangleRaycastSide testSide,
                     MemoryAllocator& allocator) const;

        /// Compute the min/max grid coords corresponding to the intersection of the AABB of the height field and the AABB to collide
        void computeMinMaxGridCoordinates(uint32* minCoords, uint32* maxCoords, const AABB& aabbToCollide) const;

        /// Compute the shape Id for a given triangle
        uint32 computeTriangleShapeId(uint32 iIndex, uint32 jIndex, uint32 secondTriangleIncrement) const;

        /// Compute the first grid cell of the heightfield intersected by a ray
        bool computeEnteringRayGridCoordinates(const Ray& ray, int32& i, int32& j, Vector3& outHitPoint) const;

        /// Use a callback method on all triangles of the concave shape inside a given AABB
        void computeOverlappingTriangles(const AABB& aabb, Array<Vector3>& triangleVertices,
                                         Array<Vector3>& triangleVerticesNormals,
                                         Array<uint32>& shapeIds, const Vector3& scale) const;

    public:

        /// Deleted copy-constructor
        HeightField(const HeightField& heightField) = delete;

        /// Deleted assignment operator
        HeightField& operator=(const HeightField& heightField) = delete;

        /// Return the number of rows in the height-field (along the local x direction)
        uint32 getNbRows() const;

        /// Return the number of columns in the height-field (along the local z direction)
        uint32 getNbColumns() const;

        /// Return the minimum height value of the height-field
        decimal getMinHeight() const;

        /// Return the maximum height value of the height-field
        decimal getMaxHeight() const;

        /// Return the integer height scale
        decimal getIntegerHeightScale() const;

        /// Return the vertex (local-coordinates) of the height-field at a given (x,y) position
        Vector3 getVertexAt(uint32 x, uint32 y) const;

        /// Return the height value of a given (x,y) point in the height-field (in local-space)
        decimal getHeightAt(uint32 x, uint32 y) const;

        /// Return the type of height value in the height-field
        HeightDataType getHeightDataType() const;

        /// Return the minimum bounds of the height-field in the x,y,z direction
        const AABB& getBounds() const;

        /// Return the string representation of the shape
        std::string to_string() const;

#ifdef IS_RP3D_PROFILING_ENABLED

        /// Set the profiler
        void setProfiler(Profiler* profiler);

#endif

        // ---------- Friendship ----------- //

        friend class ConvexTriangleAABBOverlapCallback;
        friend class ConcaveMeshRaycastCallback;
        friend class HeightFieldShape;
        friend class PhysicsCommon;
};

// Return the minimum bounds of the height-field in the x,y,z direction
/**
 * @return The three mimimum bounds of the height-field in the x,y,z direction
 */
RP3D_FORCE_INLINE const AABB& HeightField::getBounds() const {
    return mBounds;
}

// Return the number of rows in the height-field (along the local x direction)
/**
 * @return The number of rows of the grid (along x direction)
 */
RP3D_FORCE_INLINE uint32 HeightField::getNbRows() const {
    return mNbRows;
}

// Return the number of columns in the height-field (along the local z direction)
/**
 * @return The number of columns of the grid (along z direction)
 */
RP3D_FORCE_INLINE uint32 HeightField::getNbColumns() const {
    return mNbColumns;
}

// Return the minimum height value of the height-field
/**
 * @return The mimimum height value of the height-field
 */
RP3D_FORCE_INLINE decimal HeightField::getMinHeight() const {
    return mMinHeight;
}

// Return the maximum height value of the height-field
/**
 * @return The maximum height value of the height-field
 */
RP3D_FORCE_INLINE decimal HeightField::getMaxHeight() const {
    return mMaxHeight;
}

// Return the integer height scale
/**
 * @return The integer height scale
 */
RP3D_FORCE_INLINE decimal HeightField::getIntegerHeightScale() const {
    return mIntegerHeightScale;
}

// Return the type of height value in the height-field
RP3D_FORCE_INLINE HeightField::HeightDataType HeightField::getHeightDataType() const {
    return mHeightDataType;
}

// Return the height value of a given (x,y) point in the height-field
RP3D_FORCE_INLINE decimal HeightField::getHeightAt(uint32 x, uint32 y) const {
    assert(x < mNbColumns);
    assert(y < mNbRows);
    return mHeightFieldData[y * mNbColumns + x];
}

// Compute the shape Id for a given triangle
RP3D_FORCE_INLINE uint32 HeightField::computeTriangleShapeId(uint32 iIndex, uint32 jIndex, uint32 secondTriangleIncrement) const {
    return (jIndex * (mNbColumns - 1) + iIndex) * 2 + secondTriangleIncrement;
}

#ifdef IS_RP3D_PROFILING_ENABLED

// Set the profiler
RP3D_FORCE_INLINE void HeightField::setProfiler(Profiler* profiler) {
    mProfiler = profiler;
}

#endif

}

#endif

