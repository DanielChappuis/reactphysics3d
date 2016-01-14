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

#ifndef REACTPHYSICS3D_HEIGHTFIELD_SHAPE_H
#define REACTPHYSICS3D_HEIGHTFIELD_SHAPE_H

// Libraries
#include "ConcaveShape.h"
#include "collision/shapes/TriangleShape.h"
#include "engine/Profiler.h"

namespace reactphysics3d {

// TODO : Implement raycasting for this shape

// TODO : Implement smooth collision mesh for this shape

// Class HeightFieldShape
/**
 * This class represents a static height field that can be used to represent
 * a terrain.
 */
class HeightFieldShape : public ConcaveShape {

    /// Data type for the height data of the height field
    enum HeightDataType {HEIGHT_FLOAT_TYPE, HEIGHT_DOUBLE_TYPE, HEIGHT_INT_TYPE};

    protected:

        // -------------------- Attributes -------------------- //

        /// Height field width
        int mWidth;

        /// Height field length
        int mLength;

        /// Minimum height of the height field
        decimal mMinHeight;

        /// Maximum height of the height field
        decimal mMaxHeight;

        /// Up axis direction (0 => x, 1 => y, 2 => z)
        int mUpAxis;

        /// Data type of the height values
        HeightDataType mHeightDataType;

        /// Array of data with all the height values of the height field
        const void*	mHeightFieldData;

        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        HeightFieldShape(const HeightFieldShape& shape);

        /// Private assignment operator
        HeightFieldShape& operator=(const HeightFieldShape& shape);

        /// Raycast method with feedback information
        virtual bool raycast(const Ray& ray, RaycastInfo& raycastInfo, ProxyShape* proxyShape) const;

        /// Return the number of bytes used by the collision shape
        virtual size_t getSizeInBytes() const;

        /// Insert all the triangles into the dynamic AABB tree
        void initBVHTree();

        /// Return the three vertices coordinates (in the array outTriangleVertices) of a triangle
        /// given the start vertex index pointer of the triangle.
        void getTriangleVerticesWithIndexPointer(int32 subPart, int32 triangleIndex,
                                                 Vector3* outTriangleVertices) const;

    public:

        /// Constructor
        HeightFieldShape(int width, int length, int minHeight, int maxHeight,
                         const void* heightFieldData, HeightDataType dataType, int upAxis = 1);

        /// Destructor
        ~HeightFieldShape();

        /// Return the local bounds of the shape in x, y and z directions.
        virtual void getLocalBounds(Vector3& min, Vector3& max) const;

        /// Set the local scaling vector of the collision shape
        virtual void setLocalScaling(const Vector3& scaling);

        /// Return the local inertia tensor of the collision shape
        virtual void computeLocalInertiaTensor(Matrix3x3& tensor, decimal mass) const;

        /// Use a callback method on all triangles of the concave shape inside a given AABB
        virtual void testAllTriangles(TriangleCallback& callback, const AABB& localAABB) const;

        // ---------- Friendship ----------- //

        friend class ConvexTriangleAABBOverlapCallback;
        friend class ConcaveMeshRaycastCallback;
};

// Return the number of bytes used by the collision shape
inline size_t HeightFieldShape::getSizeInBytes() const {
    return sizeof(HeightFieldShape);
}

// Return the local bounds of the shape in x, y and z directions.
// This method is used to compute the AABB of the box
/**
 * @param min The minimum bounds of the shape in local-space coordinates
 * @param max The maximum bounds of the shape in local-space coordinates
 */
inline void HeightFieldShape::getLocalBounds(Vector3& min, Vector3& max) const {

    // TODO : Implement this
}

// Set the local scaling vector of the collision shape
inline void HeightFieldShape::setLocalScaling(const Vector3& scaling) {

    CollisionShape::setLocalScaling(scaling);

    // TODO : Implement this
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

}
#endif

