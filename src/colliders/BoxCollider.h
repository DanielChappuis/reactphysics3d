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

#ifndef BOX_COLLIDER_H
#define BOX_COLLIDER_H

// Libraries
#include <cfloat>
#include "Collider.h"
#include "../mathematics/mathematics.h"


// ReactPhysics3D namespace
namespace reactphysics3d {

/*  -------------------------------------------------------------------
    Class BoxCollider :
        This class represents a 3D box collider. Those axis are unit length.
        The three extents are half-widths of the box along the three
        axis x, y, z local axis. The "transform" of the corresponding
        rigid body gives an orientation and a position to the box.
    -------------------------------------------------------------------
*/
class BoxCollider : public Collider {
    private :
        Vector3 extent;           // Extent sizes of the box in the x, y and z direction

    public :
        BoxCollider(const Vector3& extent);        // Constructor
        virtual ~BoxCollider();                     // Destructor

        const Vector3& getExtent() const;                                                           // Return the extents of the box
        void setExtent(const Vector3& extent);                                                      // Set the extents of the box
        virtual Vector3 getLocalExtents(decimal margin=0.0) const;                                  // Return the local extents in x,y and z direction
        virtual Vector3 getLocalSupportPoint(const Vector3& direction, decimal margin=0.0) const;   // Return a local support point in a given direction
        virtual void computeLocalInertiaTensor(Matrix3x3& tensor, decimal mass) const;              // Return the local inertia tensor of the collider

#ifdef VISUAL_DEBUG
            virtual void draw() const;                                                                      // Draw the Box (only for testing purpose)
#endif
};

// Return the extents of the box
inline const Vector3& BoxCollider::getExtent() const {
    return extent;
}

 // Set the extents of the box
inline void BoxCollider::setExtent(const Vector3& extent) {
    this->extent = extent;
}

// Return the local extents of the box (half-width) in x,y and z local direction
// This method is used to compute the AABB of the box
inline Vector3 BoxCollider::getLocalExtents(decimal margin) const {
    return extent + Vector3(margin, margin, margin);
}

// Return a local support point in a given direction
inline Vector3 BoxCollider::getLocalSupportPoint(const Vector3& direction, decimal margin) const {
    assert(margin >= 0.0);
    
    return Vector3(direction.getX() < 0.0 ? -extent.getX()-margin : extent.getX()+margin,
                    direction.getY() < 0.0 ? -extent.getY()-margin : extent.getY()+margin,
                    direction.getZ() < 0.0 ? -extent.getZ()-margin : extent.getZ()+margin);
}

}; // End of the ReactPhysics3D namespace

#endif
