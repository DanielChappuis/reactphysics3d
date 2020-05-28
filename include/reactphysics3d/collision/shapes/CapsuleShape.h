/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2020 Daniel Chappuis                                       *
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

#ifndef REACTPHYSICS3D_CAPSULE_SHAPE_H
#define REACTPHYSICS3D_CAPSULE_SHAPE_H

// Libraries
#include <reactphysics3d/collision/shapes/ConvexShape.h>
#include <reactphysics3d/mathematics/mathematics.h>

// ReactPhysics3D namespace
namespace reactphysics3d {

// Declarations
class CollisionBody;

// Class CapsuleShape
/**
 * This class represents a capsule collision shape that is defined around the Y axis.
 * A capsule shape can be seen as the convex hull of two spheres.
 * The capsule shape is defined by its radius (radius of the two spheres of the capsule)
 * and its height (distance between the centers of the two spheres). This collision shape
 * does not have an explicit object margin distance. The margin is implicitly the radius
 * and height of the shape. Therefore, no need to specify an object margin for a
 * capsule shape.
 */
class CapsuleShape : public ConvexShape {

    protected :

        // -------------------- Attributes -------------------- //

        /// Half height of the capsule (height = distance between the centers of the two spheres)
        decimal mHalfHeight;

        // -------------------- Methods -------------------- //

        /// Constructor
        CapsuleShape(decimal radius, decimal height, MemoryAllocator& allocator);

        /// Return a local support point in a given direction without the object margin
        virtual Vector3 getLocalSupportPointWithoutMargin(const Vector3& direction) const override;

        /// Return true if a point is inside the collision shape
        virtual bool testPointInside(const Vector3& localPoint, Collider* collider) const override;

        /// Raycast method with feedback information
        virtual bool raycast(const Ray& ray, RaycastInfo& raycastInfo, Collider* collider, MemoryAllocator& allocator) const override;

        /// Raycasting method between a ray one of the two spheres end cap of the capsule
        bool raycastWithSphereEndCap(const Vector3& point1, const Vector3& point2,
                                     const Vector3& sphereCenter, decimal maxFraction,
                                     Vector3& hitLocalPoint, decimal& hitFraction) const;

        /// Return the number of bytes used by the collision shape
        virtual size_t getSizeInBytes() const override;

        /// Destructor
        virtual ~CapsuleShape() override = default;

    public :

        // -------------------- Methods -------------------- //

        /// Deleted copy-constructor
        CapsuleShape(const CapsuleShape& shape) = delete;

        /// Deleted assignment operator
        CapsuleShape& operator=(const CapsuleShape& shape) = delete;

        /// Return the radius of the capsule
        decimal getRadius() const;

        /// Set the radius of the capsule
        void setRadius(decimal radius);

        /// Return the height of the capsule
        decimal getHeight() const;

        /// Set the height of the capsule
        void setHeight(decimal height);

        /// Return the local bounds of the shape in x, y and z directions
        virtual void getLocalBounds(Vector3& min, Vector3& max) const override;

        /// Compute and return the volume of the collision shape
        virtual decimal getVolume() const override;

        /// Return true if the collision shape is a polyhedron
        virtual bool isPolyhedron() const override;

        /// Return the local inertia tensor of the collision shape
        virtual Vector3 getLocalInertiaTensor(decimal mass) const override;

        /// Return the string representation of the shape
        virtual std::string to_string() const override;

        // ----- Friendship ----- //

        friend class PhysicsCommon;
};

// Get the radius of the capsule
/**
 * @return The radius of the capsule shape (in meters)
 */
inline decimal CapsuleShape::getRadius() const {
    return mMargin;
}

// Set the radius of the capsule
/// Note that you might want to recompute the inertia tensor and center of mass of the body
/// after changing the radius of the collision shape
/**
 * @param radius The radius of the capsule (in meters)
 */
inline void CapsuleShape::setRadius(decimal radius) {

    assert(radius > decimal(0.0));
    mMargin = radius;

    notifyColliderAboutChangedSize();
}

// Return the height of the capsule
/**
 * @return The height of the capsule shape (in meters)
 */
inline decimal CapsuleShape::getHeight() const {
    return mHalfHeight + mHalfHeight;
}

// Set the height of the capsule
/// Note that you might want to recompute the inertia tensor and center of mass of the body
/// after changing the height of the collision shape
/**
 * @param height The height of the capsule (in meters)
 */
inline void CapsuleShape::setHeight(decimal height) {

    assert(height > decimal(0.0));
    mHalfHeight = height * decimal(0.5);

    notifyColliderAboutChangedSize();
}

// Return the number of bytes used by the collision shape
inline size_t CapsuleShape::getSizeInBytes() const {
    return sizeof(CapsuleShape);
}

// Return the local bounds of the shape in x, y and z directions
// This method is used to compute the AABB of the box
/**
 * @param min The minimum bounds of the shape in local-space coordinates
 * @param max The maximum bounds of the shape in local-space coordinates
 */
inline void CapsuleShape::getLocalBounds(Vector3& min, Vector3& max) const {

    // Maximum bounds
    max.x = mMargin;
    max.y = mHalfHeight + mMargin;
    max.z = mMargin;

    // Minimum bounds
    min.x = -mMargin;
    min.y = -max.y;
    min.z = min.x;
}

// Compute and return the volume of the collision shape
inline decimal CapsuleShape::getVolume() const {
    return reactphysics3d::PI * mMargin * mMargin * (decimal(4.0) * mMargin / decimal(3.0) + decimal(2.0) * mHalfHeight);
}

// Return true if the collision shape is a polyhedron
inline bool CapsuleShape::isPolyhedron() const {
    return false;
}

// Return a local support point in a given direction without the object margin.
/// A capsule is the convex hull of two spheres S1 and S2. The support point in the direction "d"
/// of the convex hull of a set of convex objects is the support point "p" in the set of all
/// support points from all the convex objects with the maximum dot product with the direction "d".
/// Therefore, in this method, we compute the support points of both top and bottom spheres of
/// the capsule and return the point with the maximum dot product with the direction vector. Note
/// that the object margin is implicitly the radius and height of the capsule.
inline Vector3 CapsuleShape::getLocalSupportPointWithoutMargin(const Vector3& direction) const {

    // Support point top sphere
    decimal dotProductTop = mHalfHeight * direction.y;

    // Support point bottom sphere
    decimal dotProductBottom = -mHalfHeight * direction.y;

    // Return the point with the maximum dot product
    if (dotProductTop > dotProductBottom) {
        return Vector3(0, mHalfHeight, 0);
    }
    else {
        return Vector3(0, -mHalfHeight, 0);
    }
}

// Return the string representation of the shape
inline std::string CapsuleShape::to_string() const {
    return "CapsuleShape{halfHeight=" + std::to_string(mHalfHeight) + ", radius=" + std::to_string(getRadius()) + "}";
}

}

#endif
