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

#ifndef CONTACTCACHINGINFO_H
#define	CONTACTCACHINGINFO_H

// Libraries
#include "../body/OBB.h"

// ReactPhysics3D namespace
namespace reactphysics3d {

/*  -------------------------------------------------------------------
    Structure ContactCachingInfo :
       This structure contains informations used to cache the last lambda
       value of each contact constraint. The last lambda value is used
       as the init lambda value in the constraint solver in the next
       time step to improve the convergence rate of the constraint solver.
    -------------------------------------------------------------------
*/
struct ContactCachingInfo {
    public:
        // TODO : Use polymorphism here (change OBB into BoundingVolume to be more general)
        Body* body1;                            // Body pointer of the first bounding volume
        Body* body2;                            // Body pointer of the second bounding volume
        Vector3D position;                       // Contact point
        double lambda;                           // Last lambda value for the constraint

        ContactCachingInfo(Body* body1, Body* body2, const Vector3D& position, double lambda);   // Constructor
};

} // End of the ReactPhysics3D namespace

#endif

