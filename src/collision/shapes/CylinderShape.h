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

#ifndef CYLINDER_SHAPE_H
#define CYLINDER_SHAPE_H

// Libraries
#include "CollisionShape.h"
#include "../../mathematics/mathematics.h"


// ReactPhysics3D namespace
namespace reactphysics3d {

/*  -------------------------------------------------------------------
    Class CylinderShape :
        This class represents a cylinder collision shape around the Y axis
        and centered at the origin. The cylinder is defined by its height
        and the radius of its base. The "transform" of the corresponding
        rigid body gives an orientation and a position to the cylinder.
    -------------------------------------------------------------------
*/
class CylinderShape : public CollisionShape {

    private :

        // -------------------- Attributes -------------------- //

        // Radius of the base
        decimal mRadius;

        // Half height of the cone
        decimal mHalfHeight;

        // -------------------- Methods -------------------- //

        // Private copy-constructor
        CylinderShape(const CylinderShape& shape);

        // Private assignment operator
        CylinderShape& operator=(const CylinderShape& shape);

    public :

        // -------------------- Methods -------------------- //

        // Constructor
        CylinderShape(decimal radius, decimal height);

        // Destructor
        virtual ~CylinderShape();

        // Return the radius
        decimal getRadius() const;

        // Set the radius
        void setRadius(decimal mRadius);

        // Return the height
        decimal getHeight() const;

        // Set the height
        void setHeight(decimal height);

        // Return a local support point in a given direction with the object margin
        virtual Vector3 getLocalSupportPointWithMargin(const Vector3& direction) const;

        // Return a local support point in a given direction without the object margin
        virtual Vector3 getLocalSupportPointWithoutMargin(const Vector3& direction) const;

        // Return the local extents in x,y and z direction
        virtual Vector3 getLocalExtents(decimal margin=0.0) const;

        // Return the local inertia tensor of the collision shape
        virtual void computeLocalInertiaTensor(Matrix3x3& tensor, decimal mass) const;

        // Return the margin distance around the shape
        virtual decimal getMargin() const;

#ifdef VISUAL_DEBUG
        // Draw the sphere (only for testing purpose)
        virtual void draw() const;
#endif
};

// Return the radius
inline decimal CylinderShape::getRadius() const {
    return mRadius;
}

// Set the radius
inline void CylinderShape::setRadius(decimal radius) {
    this->mRadius = radius;
}

// Return the height
inline decimal CylinderShape::getHeight() const {
    return mHalfHeight * 2.0;
}

// Set the height
inline void CylinderShape::setHeight(decimal height) {
    mHalfHeight = height * decimal(0.5);
}

// Return the local extents in x,y and z direction
inline Vector3 CylinderShape::getLocalExtents(decimal margin) const {
    return Vector3(mRadius + margin, mHalfHeight + margin, mRadius + margin);
}

// Return the local inertia tensor of the cylinder
inline void CylinderShape::computeLocalInertiaTensor(Matrix3x3& tensor, decimal mass) const {
    decimal height = decimal(2.0) * mHalfHeight;
    decimal diag = (decimal(1.0) / decimal(12.0)) * mass * (3 * mRadius * mRadius + height * height);
    tensor.setAllValues(diag, 0.0, 0.0, 0.0,
                        decimal(0.5) * mass * mRadius * mRadius, 0.0,
                        0.0, 0.0, diag);
}

// Return the margin distance around the shape
inline decimal CylinderShape::getMargin() const {
   return OBJECT_MARGIN;
}

}

#endif

