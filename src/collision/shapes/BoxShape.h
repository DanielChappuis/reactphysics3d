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
 * rigid body gives an orientation and a position to the box.
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
        const Vector3& getExtent() const;

        /// Return the local extents in x,y and z direction.
        virtual Vector3 getLocalExtents() const;

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
};

// Allocate and return a copy of the object
inline BoxShape* BoxShape::clone(void* allocatedMemory) const {
    return new (allocatedMemory) BoxShape(*this);
}

// Return the extents of the box
inline const Vector3& BoxShape::getExtent() const {
    return mExtent + Vector3(mMargin, mMargin, mMargin);
}

// Return the local extents of the box (half-width) in x,y and z local direction.
/// This method is used to compute the AABB of the box
inline Vector3 BoxShape::getLocalExtents() const {
    return mExtent + Vector3(mMargin, mMargin, mMargin);
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

}

#endif
