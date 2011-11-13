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

#ifndef SPHERE_SHAPE_H
#define SPHERE_SHAPE_H

// Libraries
#include "Shape.h"
#include "../mathematics/mathematics.h"

// ReactPhysics3D namespace
namespace reactphysics3d {

/*  -------------------------------------------------------------------
    Class SphereShape :
        This class represents a sphere collision shape that is centered
        at the origin and defined by its radius.
    -------------------------------------------------------------------
*/
class SphereShape : public Shape {
    private :
        double radius;              // Radius of the sphere

    public :
        SphereShape(double radius);                 // Constructor
        virtual ~SphereShape();                     // Destructor

        double getRadius() const;                                                                   // Return the radius of the sphere
        void setRadius(double radius);                                                              // Set the radius of the sphere
        virtual Vector3 getLocalSupportPoint(const Vector3& direction, double margin=0.0) const;    // Return a local support point in a given direction
        virtual Vector3 getLocalExtents(double margin=0.0) const;                                   // Return the local extents in x,y and z direction
        virtual void computeLocalInertiaTensor(Matrix3x3& tensor, double mass) const;               // Return the local inertia tensor of the shape

#ifdef VISUAL_DEBUG
            virtual void draw() const;                              // Draw the sphere (only for testing purpose)
#endif
};

// Get the radius of the sphere
inline double SphereShape::getRadius() const {
    return radius;
}

// Set the radius of the sphere
inline void SphereShape::setRadius(double radius) {
    this->radius = radius;
}

// Return a local support point in a given direction
inline Vector3 SphereShape::getLocalSupportPoint(const Vector3& direction, double margin) const {
    assert(margin >= 0.0);
    double length = direction.length();

    // If the direction vector is not the zero vector
    if (length > 0.0) {
        // Return the support point of the sphere in the given direction
        return (radius + margin) * direction.getUnit();
    }

    // If the direction vector is the zero vector we return a point on the
    // boundary of the sphere
    return Vector3(0, radius + margin, 0);
}

// Return the local extents of the shape (half-width) in x,y and z local direction
// This method is used to compute the AABB of the box
inline Vector3 SphereShape::getLocalExtents(double margin) const {
    return Vector3(radius + margin, radius + margin, radius + margin);
}

// Return the local inertia tensor of the sphere
inline void SphereShape::computeLocalInertiaTensor(Matrix3x3& tensor, double mass) const {
    double diag = 0.4 * mass * radius * radius;
    tensor.setAllValues(diag, 0.0, 0.0, 0.0, diag, 0.0, 0.0, 0.0, diag);
}

}; // End of the ReactPhysics3D namespace

#endif