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
        goal of the broad-phase algorithm is to compute the pair of bodies
        that can collide. But it's important to understand that the
        broad-phase doesn't compute only body pairs that can collide but
        could also pairs of body that doesn't collide but are very close.
        The goal of the broad-phase is to remove pairs of body that cannot
        collide in order to avoid to much bodies to be tested in the
        narrow-phase.
    --------------------------------------------------------------------
*/
class BroadPhaseAlgorithm {
    protected :

    public :
        BroadPhaseAlgorithm();              // Constructor
        virtual ~BroadPhaseAlgorithm();     // Destructor

        virtual void computePossibleCollisionPairs(std::vector<Body*> addedBodies, std::vector<Body*> removedBodies,
                                                   std::vector<std::pair<const Body*, const Body* > >& possibleCollisionPairs)=0;  // Compute the possible collision pairs of bodies
};

} // End of reactphysics3d namespace

#endif

