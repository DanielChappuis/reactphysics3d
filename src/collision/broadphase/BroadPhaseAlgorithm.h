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

#ifndef BROAD_PHASE_ALGORITHM_H
#define BROAD_PHASE_ALGORITHM_H

// Libraries
#include <vector>
#include "../../body/RigidBody.h"

// Namespace ReactPhysics3D
namespace reactphysics3d {

// Declarations
class CollisionDetection;
    
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
        CollisionDetection& collisionDetection;  // Reference to the collision detection object
        
    public :
        BroadPhaseAlgorithm(CollisionDetection& collisionDetection);    // Constructor
        virtual ~BroadPhaseAlgorithm();                                 // Destructor

        virtual void computePossibleCollisionPairs()=0;                     // Compute the possible collision pairs of bodies
        virtual void notifyAddedBodies(std::vector<RigidBody*> bodies)=0;   // Notify the broad-phase algorithm about new bodies in the physics world
        virtual void notifyRemovedBodies(std::vector<RigidBody*> bodies)=0; // Notify the broad-phase algorithm about removed bodies in the physics world
};

} // End of reactphysics3d namespace

#endif

