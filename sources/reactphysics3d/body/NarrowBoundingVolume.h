/****************************************************************************
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

#ifndef NARROW_BOUNDING_VOLUME_H
#define NARROW_BOUNDING_VOLUME_H

// Libraries
#include "BoundingVolume.h"
#include "AABB.h"


// ReactPhysics3D namespace
namespace reactphysics3d {

/*  -------------------------------------------------------------------
    Class NarrowBoundingVolume :
        This class represents the volume that contains a rigid body
        This volume will be used to compute the narrow-phase collision
        detection.
    -------------------------------------------------------------------
*/
class NarrowBoundingVolume : public BoundingVolume {
    protected :

    public :
        NarrowBoundingVolume();                   // Constructor
        virtual ~NarrowBoundingVolume();          // Destructor

        virtual AABB* computeAABB() const=0;      // Return the corresponding AABB
};

}

#endif