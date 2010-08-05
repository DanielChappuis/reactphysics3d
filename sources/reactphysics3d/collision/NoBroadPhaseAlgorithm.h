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

#ifndef NOBROADPHASEALGORITHM_H
#define NOBROADPHASEALGORITHM_H

// Libraries
#include "BroadPhaseAlgorithm.h"
#include <algorithm>

// Namespace ReactPhysics3D
namespace reactphysics3d {

/*  --------------------------------------------------------------------
    Class NoBroadPhaseAlgorithm :
        This class implements a broad-phase algorithm that does nothing.
        It should be use if we don't want to perform a broad-phase for
        the collision detection.
    --------------------------------------------------------------------
*/
class NoBroadPhaseAlgorithm : public BroadPhaseAlgorithm {
    protected :
        std::vector<Body*> bodies;     // All bodies of the engine

    public :
        NoBroadPhaseAlgorithm();              // Constructor
        virtual ~NoBroadPhaseAlgorithm();     // Destructor

        virtual void computePossibleCollisionPairs(std::vector<Body*> addedBodies, std::vector<Body*> removedBodies,
                                                   std::vector<std::pair<const Body*, const Body* > >& possibleCollisionPairs);     // Compute the possible collision pairs of bodies
};


// Compute the possible collision pairs of bodies
// The arguments "addedBodies" and "removedBodies" are respectively the set
// of bodies that have been added and removed since the last broad-phase
// computation. Before the call, the argument "possibleCollisionPairs"
// correspond to the possible colliding pairs of bodies from the last broad-phase
// computation. This methods computes the current possible collision pairs of
// bodies and update the "possibleCollisionPairs" argument. This broad-phase
// algorithm doesn't do anything and therefore the "possibleCollisionPairs" set
// must contains all the possible pairs of bodies
inline void NoBroadPhaseAlgorithm::computePossibleCollisionPairs(std::vector<Body*> addedBodies, std::vector<Body*> removedBodies,
                                                                 std::vector<std::pair<const Body*, const Body* > >& possibleCollisionPairs) {
    // Add the new bodies
    for (std::vector<Body*>::iterator it = addedBodies.begin(); it < addedBodies.end(); it++) {
        bodies.push_back(*it);
    }

    // Remove the bodies to be removed
    for (std::vector<Body*>::iterator it = removedBodies.begin(); it < removedBodies.end(); it++) {
        bodies.erase(std::find(bodies.begin(), bodies.end(), *it));
    }

    // If the set of bodies have been changed
    if (addedBodies.size() + removedBodies.size() > 0) {
        // Recompute all the possible pairs of bodies
        possibleCollisionPairs.clear();
        for (std::vector<Body*>::iterator it1 = addedBodies.begin(); it1 < addedBodies.end(); it1++) {
            for (std::vector<Body*>::iterator it2 = addedBodies.begin(); it2 < addedBodies.end(); it2++) {
                if (*it1 != *it2) {
                    possibleCollisionPairs.push_back(std::make_pair(*it1, *it2));
                }
            }
        }
    }
}

} // End of reactphysics3d namespace

#endif


