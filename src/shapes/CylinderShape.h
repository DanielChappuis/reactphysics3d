/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2010-2012 Daniel Chappuis                                       *
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
#include "Shape.h"
#include "../mathematics/mathematics.h"


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
class CylinderShape : public Shape {
    private :
        double radius;              // Radius of the base
        double halfHeight;          // Half height of the cone

    public :
        CylinderShape(double radius, double height);        // Constructor
        virtual ~CylinderShape();                           // Destructor

        double getRadius() const;                                                                   // Return the radius
        void setRadius(double radius);                                                              // Set the radius
        double getHeight() const;                                                                   // Return the height
        void setHeight(double height);                                                              // Set the height
        virtual Vector3 getLocalSupportPoint(const Vector3& direction, double margin=0.0) const;    // Return a support point in a given direction
        virtual Vector3 getLocalExtents(double margin=0.0) const;                                   // Return the local extents in x,y and z direction
        virtual void computeLocalInertiaTensor(Matrix3x3& tensor, double mass) const;               // Return the local inertia tensor of the shape

#ifdef VISUAL_DEBUG
        virtual void draw() const;                              // Draw the sphere (only for testing purpose)
#endif
};

// Return the radius
inline double CylinderShape::getRadius() const {
    return radius;
}

// Set the radius
inline void CylinderShape::setRadius(double radius) {
    this->radius = radius;
}

// Return the height
inline double CylinderShape::getHeight() const {
    return halfHeight * 2.0;
}

// Set the height
inline void CylinderShape::setHeight(double height) {
    this->halfHeight = height / 2.0;
}

// Return the local extents in x,y and z direction
inline Vector3 CylinderShape::getLocalExtents(double margin) const {
    return Vector3(radius + margin, halfHeight + margin, radius + margin);
}

// Return the local inertia tensor of the cylinder
inline void CylinderShape::computeLocalInertiaTensor(Matrix3x3& tensor, double mass) const {
    double height = 2.0 * halfHeight;
    double diag = (1.0 / 12.0) * mass * (3 * radius * radius + height * height);
    tensor.setAllValues(diag, 0.0, 0.0, 0.0, 0.5 * mass * radius * radius, 0.0, 0.0, 0.0, diag);
}

}; // End of the ReactPhysics3D namespace

#endif

