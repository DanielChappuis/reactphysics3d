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

#ifndef REACTPHYSICS3D_BOX_SHAPE_H
#define REACTPHYSICS3D_BOX_SHAPE_H

// Libraries
#include <cfloat>
#include "CollisionShape.h"
#include "../../mathematics/mathematics.h"


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
class BoxShape : public CollisionShape {

    private :

        // -------------------- Attributes -------------------- //

        /// Extent sizes of the box in the x, y and z direction
        Vector3 mExtent;

        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        BoxShape(const BoxShape& shape);

        /// Private assignment operator
        BoxShape& operator=(const BoxShape& shape);

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        BoxShape(const Vector3& extent, decimal margin = OBJECT_MARGIN);

        /// Destructor
        virtual ~BoxShape();

        /// Allocate and return a copy of the object
        virtual BoxShape* clone(void* allocatedMemory) const;

        /// Return the extents of the box
        Vector3 getExtent() const;

        /// Return the local bounds of the shape in x, y and z directions
        virtual void getLocalBounds(Vector3& min, Vector3& max) const;

        /// Return the number of bytes used by the collision shape
        virtual size_t getSizeInBytes() const;

        /// Return a local support point in a given direction with the object margin
        virtual Vector3 getLocalSupportPointWithMargin(const Vector3& direction) const;

        /// Return a local support point in a given direction without the object margin
        virtual Vector3 getLocalSupportPointWithoutMargin(const Vector3& direction) const;

        /// Return the local inertia tensor of the collision shape
        virtual void computeLocalInertiaTensor(Matrix3x3& tensor, decimal mass) const;

        /// Test equality between two box shapes
        virtual bool isEqualTo(const CollisionShape& otherCollisionShape) const;

        /// Create a proxy collision shape for the collision shape
        virtual ProxyShape* createProxyShape(MemoryAllocator& allocator, CollisionBody* body,
                                             const Transform& transform, decimal mass) const;
};

// Class ProxyBoxShape
/**
 * The proxy collision shape for a box shape.
 */
class ProxyBoxShape : public ProxyShape {

    private:

        // -------------------- Attributes -------------------- //

        /// Pointer to the actual collision shape
        const BoxShape* mCollisionShape;


        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        ProxyBoxShape(const ProxyBoxShape& proxyShape);

        /// Private assignment operator
        ProxyBoxShape& operator=(const ProxyBoxShape& proxyShape);

    public:

        // -------------------- Methods -------------------- //

        /// Constructor
        ProxyBoxShape(const BoxShape* shape, CollisionBody* body,
                      const Transform& transform, decimal mass);

        /// Destructor
        ~ProxyBoxShape();

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

// Allocate and return a copy of the object
inline BoxShape* BoxShape::clone(void* allocatedMemory) const {
    return new (allocatedMemory) BoxShape(*this);
}

// Return the extents of the box
inline Vector3 BoxShape::getExtent() const {
    return mExtent + Vector3(mMargin, mMargin, mMargin);
}

// Return the local bounds of the shape in x, y and z directions
/// This method is used to compute the AABB of the box
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

// Return a local support point in a given direction with the object margin
inline Vector3 BoxShape::getLocalSupportPointWithMargin(const Vector3& direction) const {

    assert(mMargin > 0.0);
    
    return Vector3(direction.x < 0.0 ? -mExtent.x - mMargin : mExtent.x + mMargin,
                   direction.y < 0.0 ? -mExtent.y - mMargin : mExtent.y + mMargin,
                   direction.z < 0.0 ? -mExtent.z - mMargin : mExtent.z + mMargin);
}

// Return a local support point in a given direction without the objec margin
inline Vector3 BoxShape::getLocalSupportPointWithoutMargin(const Vector3& direction) const {

    return Vector3(direction.x < 0.0 ? -mExtent.x : mExtent.x,
                   direction.y < 0.0 ? -mExtent.y : mExtent.y,
                   direction.z < 0.0 ? -mExtent.z : mExtent.z);
}

// Test equality between two box shapes
inline bool BoxShape::isEqualTo(const CollisionShape& otherCollisionShape) const {
    const BoxShape& otherShape = dynamic_cast<const BoxShape&>(otherCollisionShape);
    return (mExtent == otherShape.mExtent);
}

// Create a proxy collision shape for the collision shape
inline ProxyShape* BoxShape::createProxyShape(MemoryAllocator& allocator, CollisionBody* body,
                                              const Transform& transform, decimal mass) const {
    return new (allocator.allocate(sizeof(ProxyBoxShape))) ProxyBoxShape(this, body,
                                                                         transform, mass);
}

// Return the collision shape
inline const CollisionShape* ProxyBoxShape::getCollisionShape() const {
    return mCollisionShape;
}

// Return the number of bytes used by the proxy collision shape
inline size_t ProxyBoxShape::getSizeInBytes() const {
    return sizeof(ProxyBoxShape);
}

// Return a local support point in a given direction with the object margin
inline Vector3 ProxyBoxShape::getLocalSupportPointWithMargin(const Vector3& direction) {
    return mCollisionShape->getLocalSupportPointWithMargin(direction);
}

// Return a local support point in a given direction without the object margin
inline Vector3 ProxyBoxShape::getLocalSupportPointWithoutMargin(const Vector3& direction) {
    return mCollisionShape->getLocalSupportPointWithoutMargin(direction);
}

// Return the current object margin
inline decimal ProxyBoxShape::getMargin() const {
    return mCollisionShape->getMargin();
}

}

#endif
