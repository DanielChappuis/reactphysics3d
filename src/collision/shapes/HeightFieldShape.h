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

#ifndef REACTPHYSICS3D_HEIGHTFIELD_SHAPE_H
#define REACTPHYSICS3D_HEIGHTFIELD_SHAPE_H

// Libraries
#include "ConcaveShape.h"
#include "collision/shapes/AABB.h"

namespace reactphysics3d {

class HeightFieldShape;
class Profiler;
class TriangleShape;

// Class TriangleOverlapCallback
/**
 * This class is used for testing AABB and triangle overlap for raycasting
 */
class TriangleOverlapCallback : public TriangleCallback {

    protected:

        const Ray& mRay;
        ProxyShape* mProxyShape;
        RaycastInfo& mRaycastInfo;
        bool mIsHit;
        decimal mSmallestHitFraction;
        const HeightFieldShape& mHeightFieldShape;
        MemoryAllocator& mAllocator;
		
#ifdef IS_PROFILING_ACTIVE

		/// Pointer to the profiler
		Profiler* mProfiler;

#endif

    public:

        // Constructor
        TriangleOverlapCallback(const Ray& ray, ProxyShape* proxyShape, RaycastInfo& raycastInfo,
                                const HeightFieldShape& heightFieldShape, MemoryAllocator& allocator)
                               : mRay(ray), mProxyShape(proxyShape), mRaycastInfo(raycastInfo),
                                 mHeightFieldShape (heightFieldShape), mAllocator(allocator) {
            mIsHit = false;
            mSmallestHitFraction = mRay.maxFraction;
        }

        bool getIsHit() const {return mIsHit;}

        /// Raycast test between a ray and a triangle of the heightfield
        virtual void testTriangle(const Vector3* trianglePoints, const Vector3* verticesNormals, uint shapeId) override;

#ifdef IS_PROFILING_ACTIVE

		/// Set the profiler
		void setProfiler(Profiler* profiler) {
			mProfiler = profiler;
		}

#endif

};


// Class HeightFieldShape
/**
 * This class represents a static height field that can be used to represent
 * a terrain. The height field is made of a grid with rows and columns with a
 * height value at each grid point. Note that the height values are not copied into the shape
 * but are shared instead. The height values can be of type integer, float or double.
 * When creating a HeightFieldShape, you need to specify the minimum and maximum height value of
 * your height field. Note that the HeightFieldShape will be re-centered based on its AABB. It means
 * that for instance, if the minimum height value is -200 and the maximum value is 400, the final
 * minimum height of the field in the simulation will be -300 and the maximum height will be 300.
 */
class HeightFieldShape : public ConcaveShape {

    public:

        /// Data type for the height data of the height field
        enum class HeightDataType {HEIGHT_FLOAT_TYPE, HEIGHT_DOUBLE_TYPE, HEIGHT_INT_TYPE};

    protected:

        // -------------------- Attributes -------------------- //

        /// Number of columns in the grid of the height field
        int mNbColumns;

        /// Number of rows in the grid of the height field
        int mNbRows;

        /// Height field width
        decimal mWidth;

        /// Height field length
        decimal mLength;

        /// Minimum height of the height field
        decimal mMinHeight;

        /// Maximum height of the height field
        decimal mMaxHeight;

        /// Up axis direction (0 => x, 1 => y, 2 => z)
        int mUpAxis;

        /// Height values scale for height field with integer height values
        decimal mIntegerHeightScale;

        /// Data type of the height values
        HeightDataType mHeightDataType;

        /// Array of data with all the height values of the height field
        const void*	mHeightFieldData;

        /// Local AABB of the height field (without scaling)
        AABB mAABB;

        /// Scaling vector
        const Vector3 mScaling;

        // -------------------- Methods -------------------- //

        /// Raycast method with feedback information
        virtual bool raycast(const Ray& ray, RaycastInfo& raycastInfo, ProxyShape* proxyShape, MemoryAllocator& allocator) const override;

        /// Return the number of bytes used by the collision shape
        virtual size_t getSizeInBytes() const override;

        /// Insert all the triangles into the dynamic AABB tree
        void initBVHTree();

        /// Return the three vertices coordinates (in the array outTriangleVertices) of a triangle
        /// given the start vertex index pointer of the triangle.
        void getTriangleVerticesWithIndexPointer(int32 subPart, int32 triangleIndex,
                                                 Vector3* outTriangleVertices) const;

        /// Return the closest inside integer grid value of a given floating grid value
        int computeIntegerGridValue(decimal value) const;

        /// Compute the min/max grid coords corresponding to the intersection of the AABB of the height field and the AABB to collide
        void computeMinMaxGridCoordinates(int* minCoords, int* maxCoords, const AABB& aabbToCollide) const;

        /// Compute the shape Id for a given triangle
        uint computeTriangleShapeId(uint iIndex, uint jIndex, uint secondTriangleIncrement) const;

    public:

        /// Constructor
        HeightFieldShape(int nbGridColumns, int nbGridRows, decimal minHeight, decimal maxHeight,
                         const void* heightFieldData, HeightDataType dataType,
                         int upAxis = 1, decimal integerHeightScale = 1.0f,
                         const Vector3& scaling = Vector3(1,1,1));

        /// Destructor
        virtual ~HeightFieldShape() override = default;

        /// Deleted copy-constructor
        HeightFieldShape(const HeightFieldShape& shape) = delete;

        /// Deleted assignment operator
        HeightFieldShape& operator=(const HeightFieldShape& shape) = delete;

        /// Return the scaling factor
        const Vector3& getScaling() const;

        /// Return the number of rows in the height field
        int getNbRows() const;

        /// Return the number of columns in the height field
        int getNbColumns() const;
		
        /// Return the vertex (local-coordinates) of the height field at a given (x,y) position
        Vector3 getVertexAt(int x, int y) const;

        /// Return the height of a given (x,y) point in the height field
        decimal getHeightAt(int x, int y) const;

        /// Return the type of height value in the height field
        HeightDataType getHeightDataType() const;

        /// Return the local bounds of the shape in x, y and z directions.
        virtual void getLocalBounds(Vector3& min, Vector3& max) const override;

        /// Return the local inertia tensor of the collision shape
        virtual void computeLocalInertiaTensor(Matrix3x3& tensor, decimal mass) const override;

        /// Use a callback method on all triangles of the concave shape inside a given AABB
        virtual void testAllTriangles(TriangleCallback& callback, const AABB& localAABB) const override;

        /// Return the string representation of the shape
        virtual std::string to_string() const override;

        // ---------- Friendship ----------- //

        friend class ConvexTriangleAABBOverlapCallback;
        friend class ConcaveMeshRaycastCallback;
};

// Return the scaling factor
inline const Vector3& HeightFieldShape::getScaling() const {
    return mScaling;
}

// Return the number of rows in the height field
inline int HeightFieldShape::getNbRows() const {
    return mNbRows;
}

// Return the number of columns in the height field
inline int HeightFieldShape::getNbColumns() const {
    return mNbColumns;
}

// Return the type of height value in the height field
inline HeightFieldShape::HeightDataType HeightFieldShape::getHeightDataType() const {
    return mHeightDataType;
}

// Return the number of bytes used by the collision shape
inline size_t HeightFieldShape::getSizeInBytes() const {
    return sizeof(HeightFieldShape);
}

// Return the height of a given (x,y) point in the height field
inline decimal HeightFieldShape::getHeightAt(int x, int y) const {

    switch(mHeightDataType) {
        case HeightDataType::HEIGHT_FLOAT_TYPE : return ((float*)mHeightFieldData)[y * mNbColumns + x];
        case HeightDataType::HEIGHT_DOUBLE_TYPE : return ((double*)mHeightFieldData)[y * mNbColumns + x];
        case HeightDataType::HEIGHT_INT_TYPE : return ((int*)mHeightFieldData)[y * mNbColumns + x] * mIntegerHeightScale;
        default: assert(false); return 0;
    }
}

// Return the closest inside integer grid value of a given floating grid value
inline int HeightFieldShape::computeIntegerGridValue(decimal value) const {
    return (value < decimal(0.0)) ? value - decimal(0.5) : value + decimal(0.5);
}

// Return the local inertia tensor
/**
 * @param[out] tensor The 3x3 inertia tensor matrix of the shape in local-space
 *                    coordinates
 * @param mass Mass to use to compute the inertia tensor of the collision shape
 */
inline void HeightFieldShape::computeLocalInertiaTensor(Matrix3x3& tensor, decimal mass) const {

    // Default inertia tensor
    // Note that this is not very realistic for a concave triangle mesh.
    // However, in most cases, it will only be used static bodies and therefore,
    // the inertia tensor is not used.
    tensor.setAllValues(mass, 0, 0,
                        0, mass, 0,
                        0, 0, mass);
}

// Compute the shape Id for a given triangle
inline uint HeightFieldShape::computeTriangleShapeId(uint iIndex, uint jIndex, uint secondTriangleIncrement) const {

    return (jIndex * (mNbColumns - 1) + iIndex) * 2 + secondTriangleIncrement;
}

}
#endif

