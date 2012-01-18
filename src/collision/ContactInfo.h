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

#ifndef CONTACT_INFO_H
#define	CONTACT_INFO_H

// Libraries
#include "../shapes/BoxCollider.h"
#include "../mathematics/mathematics.h"

// ReactPhysics3D namespace
namespace reactphysics3d {

/*  -------------------------------------------------------------------
    Structure ContactInfo :
       This structure contains informations about a collision contact
       computed durring the narow phase collision detection. Those
       informations are use to compute the contact set for a contact
       between two bodies.
    -------------------------------------------------------------------
*/
struct ContactInfo {
    public:
        Body* const body1;              // Pointer to the first body of the contact
        Body* const body2;              // Pointer to the second body of the contact
        const Vector3 normal;           // Normal vector the the collision contact in world space
        const decimal penetrationDepth;  // Penetration depth of the contact
        const Vector3 localPoint1;      // Contact point of body 1 in local space of body 1
        const Vector3 localPoint2;      // Contact point of body 2 in local space of body 2
        const Vector3 worldPoint1;      // Contact point of body 1 in world space
        const Vector3 worldPoint2;      // Contact point of body 2 in world space
        
        ContactInfo(Body* body1, Body* body2, const Vector3& normal, decimal penetrationDepth,
                    const Vector3& localPoint1, const Vector3& localPoint2,
                    const Transform& transform1, const Transform& transform2);    // Constructor for GJK
};

} // End of the ReactPhysics3D namespace

#endif

