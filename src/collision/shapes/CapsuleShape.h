/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2010-2013 Daniel Chappuis                                       *
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
#include "CollisionShape.h"
#include "../../mathematics/mathematics.h"

// ReactPhysics3D namespace
namespace reactphysics3d {

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
class CapsuleShape : public CollisionShape {

    private :

        // -------------------- Attributes -------------------- //

        /// Radius of the two spheres of the capsule
        decimal mRadius;

        /// Half height of the capsule (height = distance between the centers of the two spheres)
        decimal mHalfHeight;

        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        CapsuleShape(const CapsuleShape& shape);

        /// Private assignment operator
        CapsuleShape& operator=(const CapsuleShape& shape);

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        CapsuleShape(decimal radius, decimal height);

        /// Destructor
        virtual ~CapsuleShape();

        /// Allocate and return a copy of the object
        virtual CapsuleShape* clone(void* allocatedMemory) const;

        /// Return the radius of the capsule
        decimal getRadius() const;

        /// Return the height of the capsule
        decimal getHeight() const;

        /// Return the number of bytes used by the collision shape
        virtual size_t getSizeInBytes() const;

        /// Return a local support point in a given direction with the object margin.
        virtual Vector3 getLocalSupportPointWithMargin(const Vector3& direction) const;

        /// Return a local support point in a given direction without the object margin
        virtual Vector3 getLocalSupportPointWithoutMargin(const Vector3& direction) const;

        /// Return the local bounds of the shape in x, y and z directions
        virtual void getLocalBounds(Vector3& min, Vector3& max) const;

        /// Return the local inertia tensor of the collision shape
        virtual void computeLocalInertiaTensor(Matrix3x3& tensor, decimal mass) const;

        /// Test equality between two capsule shapes
        virtual bool isEqualTo(const CollisionShape& otherCollisionShape) const;

        /// Create a proxy collision shape for the collision shape
        virtual ProxyShape* createProxyShape(MemoryAllocator& allocator, CollisionBody* body,
                                             const Transform& transform, decimal mass);
};

// Class ProxyCapsuleShape
/**
 * The proxy collision shape for a capsule shape.
 */
class ProxyCapsuleShape : public ProxyShape {

    private:

        // -------------------- Attributes -------------------- //

        /// Pointer to the actual collision shape
        CapsuleShape* mCollisionShape;


        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        ProxyCapsuleShape(const ProxyCapsuleShape& proxyShape);

        /// Private assignment operator
        ProxyCapsuleShape& operator=(const ProxyCapsuleShape& proxyShape);

        /// Return the non-const collision shape
        virtual CollisionShape* getInternalCollisionShape() const;

    public:

        // -------------------- Methods -------------------- //

        /// Constructor
        ProxyCapsuleShape(CapsuleShape* shape, CollisionBody* body,
                          const Transform& transform, decimal mass);

        /// Destructor
        ~ProxyCapsuleShape();

        /// Return the collision shape
        virtual const CollisionShape* getCollisionShape() const;

        /// Return the number of bytes used by the proxy collision shape
        virtual size_t getSizeInBytes() const;

        /// Return a local support point in a given direction with the object margin
        virtual Vector3 getLocalSupportPointWithMargin(const Vector3& direction);

        /// Return a local support point in a given direction without the object margin
        virtual Vector3 getLocalSupportPointWithoutMargin(const Vector3& direction);

        /// Return the current collision shape margin
        virtual decimal getMargin() const;
};

/// Allocate and return a copy of the object
inline CapsuleShape* CapsuleShape::clone(void* allocatedMemory) const {
    return new (allocatedMemory) CapsuleShape(*this);
}

// Get the radius of the capsule
inline decimal CapsuleShape::getRadius() const {
    return mRadius;
}

// Return the height of the capsule
inline decimal CapsuleShape::getHeight() const {
    return mHalfHeight + mHalfHeight;
}

// Return the number of bytes used by the collision shape
inline size_t CapsuleShape::getSizeInBytes() const {
    return sizeof(CapsuleShape);
}

// Return the local bounds of the shape in x, y and z directions
// This method is used to compute the AABB of the box
inline void CapsuleShape::getLocalBounds(Vector3& min, Vector3& max) const {

    // Maximum bounds
    max.x = mRadius;
    max.y = mHalfHeight + mRadius;
    max.z = mRadius;

    // Minimum bounds
    min.x = -mRadius;
    min.y = -max.y;
    min.z = min.x;
}

// Test equality between two capsule shapes
inline bool CapsuleShape::isEqualTo(const CollisionShape& otherCollisionShape) const {
    const CapsuleShape& otherShape = dynamic_cast<const CapsuleShape&>(otherCollisionShape);
    return (mRadius == otherShape.mRadius && mHalfHeight == otherShape.mHalfHeight);
}

// Create a proxy collision shape for the collision shape
inline ProxyShape* CapsuleShape::createProxyShape(MemoryAllocator& allocator, CollisionBody* body,
                                                  const Transform& transform, decimal mass) {
    return new (allocator.allocate(sizeof(ProxyCapsuleShape))) ProxyCapsuleShape(this, body,
                                                                           transform, mass);
}

// Return the non-const collision shape
inline CollisionShape* ProxyCapsuleShape::getInternalCollisionShape() const {
    return mCollisionShape;
}

// Return the collision shape
inline const CollisionShape* ProxyCapsuleShape::getCollisionShape() const {
    return mCollisionShape;
}

// Return the number of bytes used by the proxy collision shape
inline size_t ProxyCapsuleShape::getSizeInBytes() const {
    return sizeof(ProxyCapsuleShape);
}

// Return a local support point in a given direction with the object margin
inline Vector3 ProxyCapsuleShape::getLocalSupportPointWithMargin(const Vector3& direction)  {
    return mCollisionShape->getLocalSupportPointWithMargin(direction);
}

// Return a local support point in a given direction without the object margin
inline Vector3 ProxyCapsuleShape::getLocalSupportPointWithoutMargin(const Vector3& direction) {
    return mCollisionShape->getLocalSupportPointWithoutMargin(direction);
}

// Return the current object margin
inline decimal ProxyCapsuleShape::getMargin() const {
    return mCollisionShape->getMargin();
}

}

#endif
