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

#ifndef SEPARATINGAXISAABB_H
#define SEPARATINGAXISAABB_H

// Libraries
#include "BroadPhaseAlgorithm.h"
#include "../constraint/Contact.h"

// ReactPhysics3D namespace
namespace reactphysics3d {

/*  -------------------------------------------------------------------
    Class SeparatingAxisAABB :
        This class implements a broad-phase collision detection
        algorithm. This algorithm is uses a separating axis technique
        with axis aligned bounding box (AABB) to check if two bounding
        volumes can colide or not.
    -------------------------------------------------------------------
*/
class SeparatingAxisAABB : public BroadPhaseAlgorithm {
    private :

    public :
        SeparatingAxisAABB();           // Constructor
        ~SeparatingAxisAABB();          // Destructor

        virtual bool testCollisionPair(const BoundingVolume* const boundingVolume1, const BoundingVolume* const boundingVolume2);     // Return true if the two AABB of the bodies intersect
};

} // End of the ReactPhysics3D namespace

#endif

