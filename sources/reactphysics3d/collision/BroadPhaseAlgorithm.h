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

#ifndef BROADPHASEALGORITHM_H
#define BROADPHASEALGORITHM_H

// Libraries
#include "../body/BoundingVolume.h"

// Namespace ReactPhysics3D
namespace reactphysics3d {

/*  --------------------------------------------------------------------
    Class BroadPhaseAlgorithm :
        This class is an abstract class that represents an algorithm
        used to perform the broad-phase of a collision detection. The
        goal of the broad-phase algorithm is to compute the bodies that
        can collide. But it's important to understand that the
        broad-phase doesn't compute only body pairs that can collide but
        also pairs of body that doesn't collide but are very close. The
        goal of the broad-phase is to remove pairs of body that cannot
        collide in order to avoid to much bodies to be tested in the
        narrow-phase.
    --------------------------------------------------------------------
*/
class BroadPhaseAlgorithm {
    private :

    public :
        BroadPhaseAlgorithm();              // Constructor
        virtual ~BroadPhaseAlgorithm();     // Destructor

        virtual bool testCollisionPair(const BoundingVolume& boundingVolume1, const BoundingVolume& boundingVolume2)=0;  // Return true is the two bounding volume can collide
};

} // End of reactphysics3d namespace

#endif

