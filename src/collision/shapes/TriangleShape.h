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

#ifndef REACTPHYSICS3D_TRIANGLE_SHAPE_H
#define REACTPHYSICS3D_TRIANGLE_SHAPE_H

// Libraries
#include "mathematics/mathematics.h"
#include "ConvexShape.h"

/// ReactPhysics3D namespace
namespace reactphysics3d {

// Class TriangleShape
/**
 * This class represents a triangle collision shape that is centered
 * at the origin and defined three points.
 */
class TriangleShape : public ConvexShape {

    protected:

        // -------------------- Attribute -------------------- //

        /// Three points of the triangle
        Vector3 mPoints[3];

        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        TriangleShape(const TriangleShape& shape);

        /// Private assignment operator
        TriangleShape& operator=(const TriangleShape& shape);

        /// Return a local support point in a given direction with the object margin
        virtual Vector3 getLocalSupportPointWithMargin(const Vector3& direction,
                                                       void** cachedCollisionData) const;

        /// Return a local support point in a given direction without the object margin
        virtual Vector3 getLocalSupportPointWithoutMargin(const Vector3& direction,
                                                          void** cachedCollisionData) const;

        /// Return true if a point is inside the collision shape
        virtual bool testPointInside(const Vector3& localPoint, ProxyShape* proxyShape) const;

        /// Raycast method with feedback information
        virtual bool raycast(const Ray& ray, RaycastInfo& raycastInfo, ProxyShape* proxyShape) const;

        /// Return the number of bytes used by the collision shape
        virtual size_t getSizeInBytes() const;

    public:

        // -------------------- Methods -------------------- //

        /// Constructor
        TriangleShape(const Vector3& point1, const Vector3& point2,
                      const Vector3& point3, decimal margin);

        /// Destructor
        virtual ~TriangleShape();

        /// Return the local bounds of the shape in x, y and z directions.
        virtual void getLocalBounds(Vector3& min, Vector3& max) const;

        /// Return the local inertia tensor of the collision shape
        virtual void computeLocalInertiaTensor(Matrix3x3& tensor, decimal mass) const;

        /// Update the AABB of a body using its collision shape
        virtual void computeAABB(AABB& aabb, const Transform& transform);
};

// Return the number of bytes used by the collision shape
inline size_t TriangleShape::getSizeInBytes() const {
    return sizeof(TriangleShape);
}

// Return a local support point in a given direction with the object margin
inline Vector3 TriangleShape::getLocalSupportPointWithMargin(const Vector3& direction,
                                                           void** cachedCollisionData) const {

    // TODO : Do we need to use margin for triangle support point ?

    return getLocalSupportPointWithoutMargin(direction, cachedCollisionData);
}

// Return a local support point in a given direction without the object margin
inline Vector3 TriangleShape::getLocalSupportPointWithoutMargin(const Vector3& direction,
                                                              void** cachedCollisionData) const {
    Vector3 dotProducts(direction.dot(mPoints[0]), direction.dot(mPoints[1]), direction.dot(mPoints[2]));
    return mPoints[dotProducts.getMaxAxis()];
}

// Return the local bounds of the shape in x, y and z directions.
// This method is used to compute the AABB of the box
/**
 * @param min The minimum bounds of the shape in local-space coordinates
 * @param max The maximum bounds of the shape in local-space coordinates
 */
inline void TriangleShape::getLocalBounds(Vector3& min, Vector3& max) const {

    const Vector3 xAxis(mPoints[0].x, mPoints[1].x, mPoints[2].x);
    const Vector3 yAxis(mPoints[0].y, mPoints[1].y, mPoints[2].y);
    const Vector3 zAxis(mPoints[0].z, mPoints[1].z, mPoints[2].z);
    min.setAllValues(xAxis.getMinValue(), yAxis.getMinValue(), zAxis.getMinValue());
    max.setAllValues(xAxis.getMaxValue(), yAxis.getMaxValue(), zAxis.getMaxValue());
}

// Return the local inertia tensor of the triangle shape
/**
 * @param[out] tensor The 3x3 inertia tensor matrix of the shape in local-space
 *                    coordinates
 * @param mass Mass to use to compute the inertia tensor of the collision shape
 */
inline void TriangleShape::computeLocalInertiaTensor(Matrix3x3& tensor, decimal mass) const {
    tensor.setToZero();
}

// Update the AABB of a body using its collision shape
/**
 * @param[out] aabb The axis-aligned bounding box (AABB) of the collision shape
 *                  computed in world-space coordinates
 * @param transform Transform used to compute the AABB of the collision shape
 */
inline void TriangleShape::computeAABB(AABB& aabb, const Transform& transform) {

    const Vector3 worldPoint1 = transform * mPoints[0];
    const Vector3 worldPoint2 = transform * mPoints[1];
    const Vector3 worldPoint3 = transform * mPoints[2];

    const Vector3 xAxis(worldPoint1.x, worldPoint2.x, worldPoint3.x);
    const Vector3 yAxis(worldPoint1.y, worldPoint2.y, worldPoint3.y);
    const Vector3 zAxis(worldPoint1.z, worldPoint2.z, worldPoint3.z);
    aabb.setMin(Vector3(xAxis.getMinValue(), yAxis.getMinValue(), zAxis.getMinValue()));
    aabb.setMax(Vector3(xAxis.getMaxValue(), yAxis.getMaxValue(), zAxis.getMaxValue()));
}

// Return true if a point is inside the collision shape
inline bool TriangleShape::testPointInside(const Vector3& localPoint, ProxyShape* proxyShape) const {
    return false;
}

}

#endif

