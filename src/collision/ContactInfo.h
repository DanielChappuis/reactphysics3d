/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2010 Daniel Chappuis                                            *
*********************************************************************************
*                                                                               *
* Permission is hereby granted, free of charge, to any person obtaining a copy  *
* of this software and associated documentation files (the "Software"), to deal *
* in the Software without restriction, including without limitation the rights  *
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell     *
* copies of the Software, and to permit persons to whom the Software is         *
* furnished to do so, subject to the following conditions:                      *
*                                                                               *
* The above copyright notice and this permission notice shall be included in    *
* all copies or substantial portions of the Software.                           *
*                                                                               *
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR    *
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,      *
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE   *
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER        *
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, *
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN     *
* THE SOFTWARE.                                                                 *
********************************************************************************/

#ifndef CONTACT_INFO_H
#define	CONTACT_INFO_H

// Libraries
#include "../shapes/BoxShape.h"
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
        const Vector3 localPoint1;      // Contact point of body 1 in local space of body 1
        const Vector3 localPoint2;      // Contact point of body 2 in local space of body 2
        const Vector3 worldPoint1;      // Contact point of body 1 in world space
        const Vector3 worldPoint2;      // Contact point of body 2 in world space
        const Vector3 normal;           // Normal vector the the collision contact in world space
        const double penetrationDepth;  // Penetration depth of the contact
        
        ContactInfo(Body* body1, Body* body2, const Vector3& normal, double penetrationDepth,
                    const Vector3& localPoint1, const Vector3& localPoint2,
                    const Transform& transform1, const Transform& transform2);    // Constructor for GJK
};

} // End of the ReactPhysics3D namespace

#endif

