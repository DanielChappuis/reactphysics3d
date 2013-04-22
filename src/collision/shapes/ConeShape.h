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

#ifndef REACTPHYSICS3D_CONE_SHAPE_H
#define REACTPHYSICS3D_CONE_SHAPE_H

// Libraries
#include "CollisionShape.h"
#include "../../mathematics/mathematics.h"

/// ReactPhysics3D namespace
namespace reactphysics3d {

// Class ConeShape
/**
 * This class represents a cone collision shape centered at the
 * origin and alligned with the Y axis. The cone is defined
 * by its height and by the radius of its base. The center of the
 * cone is at the half of the height. The "transform" of the
 * corresponding rigid body gives an orientation and a position
 * to the cone.
 */
class ConeShape : public CollisionShape {

    private :

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
        
    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        ConeShape(decimal mRadius, decimal height);

        /// Destructor
        virtual ~ConeShape();

        /// Allocate and return a copy of the object
        virtual ConeShape* clone(void* allocatedMemory) const;

        /// Return the radius
        decimal getRadius() const;

        /// Set the radius
        void setRadius(decimal radius);

        /// Return the height
        decimal getHeight() const;

        /// Set the height
        void setHeight(decimal height);

        /// Return the number of bytes used by the collision shape
        virtual size_t getSizeInBytes() const;

        /// Return a local support point in a given direction with the object margin
        virtual Vector3 getLocalSupportPointWithMargin(const Vector3& direction) const;

        /// Return a local support point in a given direction without the object margin
        virtual Vector3 getLocalSupportPointWithoutMargin(const Vector3& direction) const;

        /// Return the local extents in x,y and z direction
        virtual Vector3 getLocalExtents(decimal margin=0.0) const;

        /// Return the local inertia tensor of the collision shape
        virtual void computeLocalInertiaTensor(Matrix3x3& tensor, decimal mass) const;

        /// Return the margin distance around the shape
        virtual decimal getMargin() const;

        /// Test equality between two cone shapes
        virtual bool isEqualTo(const CollisionShape& otherCollisionShape) const;

#ifdef VISUAL_DEBUG
        /// Draw the sphere (only for testing purpose)
        virtual void draw() const;
#endif
};

// Allocate and return a copy of the object
inline ConeShape* ConeShape::clone(void* allocatedMemory) const {
    return new (allocatedMemory) ConeShape(*this);
}

// Return the radius
inline decimal ConeShape::getRadius() const {
    return mRadius;
}

// Set the radius
inline void ConeShape::setRadius(decimal radius) {
    mRadius = radius;

    // Update sine of the semi-angle at the apex point
    mSinTheta = radius / (sqrt(radius * radius + 4 * mHalfHeight * mHalfHeight));
}

// Return the height
inline decimal ConeShape::getHeight() const {
    return decimal(2.0) * mHalfHeight;
}

// Set the height
inline void ConeShape::setHeight(decimal height) {
    mHalfHeight = height * decimal(0.5);

    // Update the sine of the semi-angle at the apex point
    mSinTheta = mRadius / (sqrt(mRadius * mRadius + height * height));
}

// Return the number of bytes used by the collision shape
inline size_t ConeShape::getSizeInBytes() const {
    return sizeof(ConeShape);
}

// Return the local extents in x,y and z direction
inline Vector3 ConeShape::getLocalExtents(decimal margin) const {
    return Vector3(mRadius + margin, mHalfHeight + margin, mRadius + margin);
}

// Return the local inertia tensor of the collision shape
inline void ConeShape::computeLocalInertiaTensor(Matrix3x3& tensor, decimal mass) const {
    decimal rSquare = mRadius * mRadius;
    decimal diagXZ = decimal(0.15) * mass * (rSquare + mHalfHeight);
    tensor.setAllValues(diagXZ, 0.0, 0.0,
                        0.0, decimal(0.3) * mass * rSquare,
                        0.0, 0.0, 0.0, diagXZ);
}

// Return the margin distance around the shape
inline decimal ConeShape::getMargin() const {
    return OBJECT_MARGIN;
}

// Test equality between two cone shapes
inline bool ConeShape::isEqualTo(const CollisionShape& otherCollisionShape) const {
    const ConeShape& otherShape = dynamic_cast<const ConeShape&>(otherCollisionShape);
    return (mRadius == otherShape.mRadius && mHalfHeight == otherShape.mHalfHeight);
}

}

#endif
