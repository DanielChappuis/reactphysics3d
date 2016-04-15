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

#ifndef REACTPHYSICS3D_CONE_SHAPE_H
#define REACTPHYSICS3D_CONE_SHAPE_H

// Libraries
#include "ConvexShape.h"
#include "body/CollisionBody.h"
#include "mathematics/mathematics.h"

/// ReactPhysics3D namespace
namespace reactphysics3d {

// Class ConeShape
/**
 * This class represents a cone collision shape centered at the
 * origin and alligned with the Y axis. The cone is defined
 * by its height and by the radius of its base. The center of the
 * cone is at the half of the height. The "transform" of the
 * corresponding rigid body gives an orientation and a position
 * to the cone. This collision shape uses an extra margin distance around
 * it for collision detection purpose. The default margin is 4cm (if your
 * units are meters, which is recommended). In case, you want to simulate small
 * objects (smaller than the margin distance), you might want to reduce the margin
 * by specifying your own margin distance using the "margin" parameter in the
 * constructor of the cone shape. Otherwise, it is recommended to use the
 * default margin distance by not using the "margin" parameter in the constructor.
 */
class ConeShape : public ConvexShape {

    protected :

        // -------------------- Attributes -------------------- //

        /// Radius of the base
        decimal mRadius;

        /// Half height of the cone
        decimal mHalfHeight;

        /// sine of the semi angle at the apex point
        decimal mSinTheta;

        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        ConeShape(const ConeShape& shape);

        /// Private assignment operator
        ConeShape& operator=(const ConeShape& shape);

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
        ConeShape(decimal mRadius, decimal height, decimal margin = OBJECT_MARGIN);

        /// Destructor
        virtual ~ConeShape();

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
 * @return Radius of the cone (in meters)
 */
inline decimal ConeShape::getRadius() const {
    return mRadius;
}

// Return the height
/**
 * @return Height of the cone (in meters)
 */
inline decimal ConeShape::getHeight() const {
    return decimal(2.0) * mHalfHeight;
}

// Set the scaling vector of the collision shape
inline void ConeShape::setLocalScaling(const Vector3& scaling) {

    mHalfHeight = (mHalfHeight / mScaling.y) * scaling.y;
    mRadius = (mRadius / mScaling.x) * scaling.x;

    CollisionShape::setLocalScaling(scaling);
}

// Return the number of bytes used by the collision shape
inline size_t ConeShape::getSizeInBytes() const {
    return sizeof(ConeShape);
}

// Return the local bounds of the shape in x, y and z directions
/**
 * @param min The minimum bounds of the shape in local-space coordinates
 * @param max The maximum bounds of the shape in local-space coordinates
 */
inline void ConeShape::getLocalBounds(Vector3& min, Vector3& max) const {

    // Maximum bounds
    max.x = mRadius + mMargin;
    max.y = mHalfHeight + mMargin;
    max.z = max.x;

    // Minimum bounds
    min.x = -max.x;
    min.y = -max.y;
    min.z = min.x;
}

// Return the local inertia tensor of the collision shape
/**
 * @param[out] tensor The 3x3 inertia tensor matrix of the shape in local-space
 *                    coordinates
 * @param mass Mass to use to compute the inertia tensor of the collision shape
 */
inline void ConeShape::computeLocalInertiaTensor(Matrix3x3& tensor, decimal mass) const {
    decimal rSquare = mRadius * mRadius;
    decimal diagXZ = decimal(0.15) * mass * (rSquare + mHalfHeight);
    tensor.setAllValues(diagXZ, 0.0, 0.0,
                        0.0, decimal(0.3) * mass * rSquare,
                        0.0, 0.0, 0.0, diagXZ);
}

// Return true if a point is inside the collision shape
inline bool ConeShape::testPointInside(const Vector3& localPoint, ProxyShape* proxyShape) const {
    const decimal radiusHeight = mRadius * (-localPoint.y + mHalfHeight) /
                                          (mHalfHeight * decimal(2.0));
    return (localPoint.y < mHalfHeight && localPoint.y > -mHalfHeight) &&
           (localPoint.x * localPoint.x + localPoint.z * localPoint.z < radiusHeight *radiusHeight);
}

}

#endif
