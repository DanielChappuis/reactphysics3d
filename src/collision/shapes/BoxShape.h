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

#ifndef REACTPHYSICS3D_BOX_SHAPE_H
#define REACTPHYSICS3D_BOX_SHAPE_H

// Libraries
#include <cfloat>
#include "ConvexShape.h"
#include "body/CollisionBody.h"
#include "mathematics/mathematics.h"


/// ReactPhysics3D namespace
namespace reactphysics3d {

// Class BoxShape
/**
 * This class represents a 3D box shape. Those axis are unit length.
 * The three extents are half-widths of the box along the three
 * axis x, y, z local axis. The "transform" of the corresponding
 * rigid body will give an orientation and a position to the box. This
 * collision shape uses an extra margin distance around it for collision
 * detection purpose. The default margin is 4cm (if your units are meters,
 * which is recommended). In case, you want to simulate small objects
 * (smaller than the margin distance), you might want to reduce the margin by
 * specifying your own margin distance using the "margin" parameter in the
 * constructor of the box shape. Otherwise, it is recommended to use the
 * default margin distance by not using the "margin" parameter in the constructor.
 */
class BoxShape : public ConvexShape {

    protected :

        // -------------------- Attributes -------------------- //

        /// Extent sizes of the box in the x, y and z direction
        Vector3 mExtent;

        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        BoxShape(const BoxShape& shape);

        /// Private assignment operator
        BoxShape& operator=(const BoxShape& shape);

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
        BoxShape(const Vector3& extent, decimal margin = OBJECT_MARGIN);

        /// Destructor
        virtual ~BoxShape();

        /// Return the extents of the box
        Vector3 getExtent() const;

        /// Set the scaling vector of the collision shape
        virtual void setLocalScaling(const Vector3& scaling);

        /// Return the local bounds of the shape in x, y and z directions
        virtual void getLocalBounds(Vector3& min, Vector3& max) const;

        /// Return the local inertia tensor of the collision shape
        virtual void computeLocalInertiaTensor(Matrix3x3& tensor, decimal mass) const;
};

// Return the extents of the box
/**
 * @return The vector with the three extents of the box shape (in meters)
 */
inline Vector3 BoxShape::getExtent() const {
    return mExtent + Vector3(mMargin, mMargin, mMargin);
}

// Set the scaling vector of the collision shape
inline void BoxShape::setLocalScaling(const Vector3& scaling) {

    mExtent = (mExtent / mScaling) * scaling;

    CollisionShape::setLocalScaling(scaling);
}

// Return the local bounds of the shape in x, y and z directions
/// This method is used to compute the AABB of the box
/**
 * @param min The minimum bounds of the shape in local-space coordinates
 * @param max The maximum bounds of the shape in local-space coordinates
 */
inline void BoxShape::getLocalBounds(Vector3& min, Vector3& max) const {

    // Maximum bounds
    max = mExtent + Vector3(mMargin, mMargin, mMargin);

    // Minimum bounds
    min = -max;
}

// Return the number of bytes used by the collision shape
inline size_t BoxShape::getSizeInBytes() const {
    return sizeof(BoxShape);
}

// Return a local support point in a given direction without the objec margin
inline Vector3 BoxShape::getLocalSupportPointWithoutMargin(const Vector3& direction,
                                                           void** cachedCollisionData) const {

    return Vector3(direction.x < 0.0 ? -mExtent.x : mExtent.x,
                   direction.y < 0.0 ? -mExtent.y : mExtent.y,
                   direction.z < 0.0 ? -mExtent.z : mExtent.z);
}

// Return true if a point is inside the collision shape
inline bool BoxShape::testPointInside(const Vector3& localPoint, ProxyShape* proxyShape) const {
    return (localPoint.x < mExtent[0] && localPoint.x > -mExtent[0] &&
            localPoint.y < mExtent[1] && localPoint.y > -mExtent[1] &&
            localPoint.z < mExtent[2] && localPoint.z > -mExtent[2]);
}

}

#endif
