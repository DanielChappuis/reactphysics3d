/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2010-2012 Daniel Chappuis                                       *
*********************************************************************************
*                                                                               *
* This software is provided 'as-is', without any express or implied warranty.   *
* In no event will the authors be held liable for any damages arising from the  *
* use of this software.                                                         *
*                                                                               *
* Permission is granted to anyone to use this software for any purpose,         *
* including commercial applications, and to alter it and redistribute it        *
* freely, subject to the following restrictions:                                *
*                                                                               *
* 1. The origin of this software must not be misrepresented; you must not claim *
*    that you wrote the original software. If you use this software in a        *
*    product, an acknowledgment in the product documentation would be           *
*    appreciated but is not required.                                           *
*                                                                               *
* 2. Altered source versions must be plainly marked as such, and must not be    *
*    misrepresented as being the original software.                             *
*                                                                               *
* 3. This notice may not be removed or altered from any source distribution.    *
*                                                                               *
********************************************************************************/

#ifndef NO_BROAD_PHASE_ALGORITHM_H
#define NO_BROAD_PHASE_ALGORITHM_H

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
        NoBroadPhaseAlgorithm(CollisionDetection& collisionDetection);  // Constructor
        virtual ~NoBroadPhaseAlgorithm();                               // Destructor

        virtual void computePossibleCollisionPairs();                   // Compute the possible collision pairs of bodies
        virtual void notifyAddedBodies(std::vector<Body*> bodies);      // Notify the broad-phase algorithm about new bodies in the physics world
        virtual void notifyRemovedBodies(std::vector<Body*> bodies);    // Notify the broad-phase algorithm about removed bodies in the physics world
};

} // End of reactphysics3d namespace

#endif


