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

#ifndef REACTPHYSICS3D_CYLINDER_SHAPE_H
#define REACTPHYSICS3D_CYLINDER_SHAPE_H

// Libraries
#include "ConvexShape.h"
#include "body/CollisionBody.h"
#include "mathematics/mathematics.h"


/// ReactPhysics3D namespace
namespace reactphysics3d {

// Class CylinderShape
/**
 * This class represents a cylinder collision shape around the Y axis
 * and centered at the origin. The cylinder is defined by its height
 * and the radius of its base. The "transform" of the corresponding
 * rigid body gives an orientation and a position to the cylinder.
 * This collision shape uses an extra margin distance around it for collision
 * detection purpose. The default margin is 4cm (if your units are meters,
 * which is recommended). In case, you want to simulate small objects
 * (smaller than the margin distance), you might want to reduce the margin by
 * specifying your own margin distance using the "margin" parameter in the
 * constructor of the cylinder shape. Otherwise, it is recommended to use the
 * default margin distance by not using the "margin" parameter in the constructor.
 */
class CylinderShape : public ConvexShape {

    protected :

        // -------------------- Attributes -------------------- //

        /// Radius of the base
        decimal mRadius;

        /// Half height of the cylinder
        decimal mHalfHeight;

        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        CylinderShape(const CylinderShape& shape);

        /// Private assignment operator
        CylinderShape& operator=(const CylinderShape& shape);

        /// Return a local support point in a given direction without the object margin
        virtual Vector3 getLocalSupportPointWithoutMargin(const Vector3& direction,
                                                          void** cachedCollisionData) const;

        /// Return true if a point is inside the collision shape
        virtual bool testPointInside(const Vector3& localPoint, ProxyShape* proxyShape) const;

        /// Raycast method with feedback information
        virtual bool raycast(const Ray& ray, RaycastInfo& raycastInfo, ProxyShape* proxyShape) const;

        /// Return the number of bytes used by the collision shape
        virtual size_t getSizeInBytes() const;

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        CylinderShape(decimal radius, decimal height, decimal margin = OBJECT_MARGIN);

        /// Destructor
        virtual ~CylinderShape();

        /// Return the radius
        decimal getRadius() const;

        /// Return the height
        decimal getHeight() const;

        /// Set the scaling vector of the collision shape
        virtual void setLocalScaling(const Vector3& scaling);

        /// Return the local bounds of the shape in x, y and z directions
        virtual void getLocalBounds(Vector3& min, Vector3& max) const;

        /// Return the local inertia tensor of the collision shape
        virtual void computeLocalInertiaTensor(Matrix3x3& tensor, decimal mass) const;
};

// Return the radius
/**
 * @return Radius of the cylinder (in meters)
 */
inline decimal CylinderShape::getRadius() const {
    return mRadius;
}

// Return the height
/**
 * @return Height of the cylinder (in meters)
 */
inline decimal CylinderShape::getHeight() const {
    return mHalfHeight + mHalfHeight;
}

// Set the scaling vector of the collision shape
inline void CylinderShape::setLocalScaling(const Vector3& scaling) {

    mHalfHeight = (mHalfHeight / mScaling.y) * scaling.y;
    mRadius = (mRadius / mScaling.x) * scaling.x;

    CollisionShape::setLocalScaling(scaling);
}

// Return the number of bytes used by the collision shape
inline size_t CylinderShape::getSizeInBytes() const {
    return sizeof(CylinderShape);
}

// Return the local bounds of the shape in x, y and z directions
/**
 * @param min The minimum bounds of the shape in local-space coordinates
 * @param max The maximum bounds of the shape in local-space coordinates
 */
inline void CylinderShape::getLocalBounds(Vector3& min, Vector3& max) const {

    // Maximum bounds
    max.x = mRadius + mMargin;
    max.y = mHalfHeight + mMargin;
    max.z = max.x;

    // Minimum bounds
    min.x = -max.x;
    min.y = -max.y;
    min.z = min.x;
}

// Return the local inertia tensor of the cylinder
/**
 * @param[out] tensor The 3x3 inertia tensor matrix of the shape in local-space
 *                    coordinates
 * @param mass Mass to use to compute the inertia tensor of the collision shape
 */
inline void CylinderShape::computeLocalInertiaTensor(Matrix3x3& tensor, decimal mass) const {
    decimal height = decimal(2.0) * mHalfHeight;
    decimal diag = (decimal(1.0) / decimal(12.0)) * mass * (3 * mRadius * mRadius + height * height);
    tensor.setAllValues(diag, 0.0, 0.0, 0.0,
                        decimal(0.5) * mass * mRadius * mRadius, 0.0,
                        0.0, 0.0, diag);
}

// Return true if a point is inside the collision shape
inline bool CylinderShape::testPointInside(const Vector3& localPoint, ProxyShape* proxyShape) const{
    return ((localPoint.x * localPoint.x + localPoint.z * localPoint.z) < mRadius * mRadius &&
            localPoint.y < mHalfHeight && localPoint.y > -mHalfHeight);
}

}

#endif

