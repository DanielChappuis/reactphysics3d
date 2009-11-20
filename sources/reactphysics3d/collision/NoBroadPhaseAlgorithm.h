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

// Namespace ReactPhysics3D
namespace reactphysics3d {

/*  --------------------------------------------------------------------
    Class BroadPhaseAlgorithm :
        This class implements a broad-phase algorithm that does nothing.
        It should be use if we don't want to perform a broad-phase for
        the collision detection.
    --------------------------------------------------------------------
*/
class NoBroadPhaseAlgorithm : public BroadPhaseAlgorithm {
    private :

    public :
        NoBroadPhaseAlgorithm();              // Constructor
        virtual ~NoBroadPhaseAlgorithm();     // Destructor

        virtual bool testCollisionPair(const BoundingVolume* const boundingVolume1, const BoundingVolume* const boundingVolume2);  // Return true if the two bounding volume can collide
};

// Method that always returns true (meaning that the two bounding volumes can collide).
// We don't want to perform the broad-phase and therefore this method always returns true.
inline bool NoBroadPhaseAlgorithm::testCollisionPair(const BoundingVolume* const boundingVolume1, const BoundingVolume* const boundingVolume2) {
    return true;
}

} // End of reactphysics3d namespace

#endif


