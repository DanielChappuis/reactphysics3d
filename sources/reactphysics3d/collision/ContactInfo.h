/***************************************************************************
* Copyright (C) 2009      Daniel Chappuis                                  *
****************************************************************************
* This file is part of ReactPhysics3D.                                     *
*                                                                          *
* ReactPhysics3D is free software: you can redistribute it and/or modify   *
* it under the terms of the GNU Lesser General Public License as published *
* by the Free Software Foundation, either version 3 of the License, or     *
* (at your option) any later version.                                      *
*                                                                          *
* ReactPhysics3D is distributed in the hope that it will be useful,        *
* but WITHOUT ANY WARRANTY; without even the implied warranty of           *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the             *
* GNU Lesser General Public License for more details.                      *
*                                                                          *
* You should have received a copy of the GNU Lesser General Public License *
* along with ReactPhysics3D. If not, see <http://www.gnu.org/licenses/>.   *
***************************************************************************/

#ifndef CONTACTINFO_H
#define	CONTACTINFO_H

// Libraries
#include "../body/OBB.h"
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
        // TODO : Use polymorphism here (change OBB into BoundingVolume to be more general)
        const OBB* const obb1;                   // Body pointer of the first bounding volume
        const OBB* const obb2;                   // Body pointer of the second bounding volume
        const Vector3D normal;                   // Normal vector the the collision contact
        const double penetrationDepth;           // Penetration depth of the contact
        
        ContactInfo(const OBB* const obb1, const OBB* const obb2, const Vector3D& normal, double penetrationDepth);                                                                // Constructor
};

} // End of the ReactPhysics3D namespace

#endif

