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

#ifndef COLLISIONDETECTION_H
#define COLLISIONDETECTION_H

// Libraries
#include "BroadPhaseAlgorithm.h"
#include "NarrowPhaseAlgorithm.h"
#include "../body/Body.h"
#include "../engine/CollisionWorld.h"
#include <vector>
#include <utility>

// ReactPhysics3D namespace
namespace reactphysics3d {

/*  -------------------------------------------------------------------
    Class CollisionDetection :
        This class computes the collision detection algorithms. We first
        perfom a broad-phase algorithm to know wich pairs of bodies can
        collide and then we run a narrow-phase algorithm to compute the
        collision contacts between bodies.
    -------------------------------------------------------------------
*/
class CollisionDetection {
    private :
        std::vector<(std::pair<Body*, Body*>)> possibleCollisionPairList;     // List that contains the possible collision pairs of bodies
        BroadPhaseAlgorithm* broadPhaseAlgorithm;                            // Broad-phase algorithm
        NarrowPhaseAlgorithm* narrowPhaseAlgorithm;                          // Narrow-phase algorithm

        void computePossibleCollisionPairs();   // Compute all the possible collisions pairs of bodies (broad-phase)
        void computeCollisionContacts();        // Compute all collision contacts between bodies (narrow-phase)

    public :
        CollisionDetection();       // Constructor
        ~CollisionDetection();      // Destructor

        void computeCollisionDetection(CollisionWorld& collisionWorld);     // Compute the collision detection
};

} // End of the ReactPhysics3D namespace

#endif
