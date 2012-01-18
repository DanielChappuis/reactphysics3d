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

#ifndef SPHERE_COLLIDER_H
#define SPHERE_COLLIDER_H

// Libraries
#include "Collider.h"
#include "../mathematics/mathematics.h"

// ReactPhysics3D namespace
namespace reactphysics3d {

/*  -------------------------------------------------------------------
    Class SphereCollider :
        This class represents a sphere collider that is centered
        at the origin and defined by its radius.
    -------------------------------------------------------------------
*/
class SphereCollider : public Collider {
    private :
        decimal radius;              // Radius of the sphere

    public :
        SphereCollider(decimal radius);                 // Constructor
        virtual ~SphereCollider();                     // Destructor

        decimal getRadius() const;                                                                  // Return the radius of the sphere
        void setRadius(decimal radius);                                                             // Set the radius of the sphere
        virtual Vector3 getLocalSupportPoint(const Vector3& direction, decimal margin=0.0) const;   // Return a local support point in a given direction
        virtual Vector3 getLocalExtents(decimal margin=0.0) const;                                  // Return the local extents in x,y and z direction
        virtual void computeLocalInertiaTensor(Matrix3x3& tensor, decimal mass) const;              // Return the local inertia tensor of the collider

#ifdef VISUAL_DEBUG
            virtual void draw() const;                              // Draw the sphere (only for testing purpose)
#endif
};

// Get the radius of the sphere
inline decimal SphereCollider::getRadius() const {
    return radius;
}

// Set the radius of the sphere
inline void SphereCollider::setRadius(decimal radius) {
    this->radius = radius;
}

// Return a local support point in a given direction
inline Vector3 SphereCollider::getLocalSupportPoint(const Vector3& direction, decimal margin) const {
    assert(margin >= 0.0);
    decimal length = direction.length();

    // If the direction vector is not the zero vector
    if (length > 0.0) {
        // Return the support point of the sphere in the given direction
        return (radius + margin) * direction.getUnit();
    }

    // If the direction vector is the zero vector we return a point on the
    // boundary of the sphere
    return Vector3(0, radius + margin, 0);
}

// Return the local extents of the collider (half-width) in x,y and z local direction
// This method is used to compute the AABB of the box
inline Vector3 SphereCollider::getLocalExtents(decimal margin) const {
    return Vector3(radius + margin, radius + margin, radius + margin);
}

// Return the local inertia tensor of the sphere
inline void SphereCollider::computeLocalInertiaTensor(Matrix3x3& tensor, decimal mass) const {
    decimal diag = 0.4 * mass * radius * radius;
    tensor.setAllValues(diag, 0.0, 0.0, 0.0, diag, 0.0, 0.0, 0.0, diag);
}

}; // End of the ReactPhysics3D namespace

#endif