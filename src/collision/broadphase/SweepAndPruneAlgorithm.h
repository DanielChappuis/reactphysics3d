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

#ifndef SWEEP_AND_PRUNE_ALGORITHM_H
#define SWEEP_AND_PRUNE_ALGORITHM_H

// Libraries
#include "BroadPhaseAlgorithm.h"
#include "../../colliders/AABB.h"


// Namespace ReactPhysics3D
namespace reactphysics3d {
    
/*  --------------------------------------------------------------------
    Class SAPAlgorithm :
        This class implements the Sweep and Prune (SAP) broad-phase
        algorithm. This algorithm uses the AABB bounding-volume of the
        bodies and keep a sorted representation of the intervals of the
        bodies' AABB on the three x. y and z axis. Given this sorted
        representation, we can efficiently compute the set of possible
        colliding pairs of bodies. At each broad-phase computation, we
        should sort the AABB according to the axis (x, y or z) with the
        largest variance of the AABBs positions in order that the sorted
        AABB's set does not change a lot between each computations. To do
        so, we compute at each time the variance of each axis and select
        the axis (sortAxis) with the largest variance for the next
        broad-phase computation.
    --------------------------------------------------------------------
*/
class SweepAndPruneAlgorithm : public BroadPhaseAlgorithm {
    protected :
        std::vector<const AABB*> sortedAABBs;   // Sorted set of AABB of the bodies on one of the x.y or z axis
        static unsigned short int sortAxis;     // Current sorting axis (0 for x, 1 for y, 2 for z axis)

        static bool compareAABBs(const AABB* a, const AABB* b);     // Static method that compare two AABBs (in order to sort them)
        

    public :
        SweepAndPruneAlgorithm(CollisionDetection& collisionDetection);   // Constructor
        virtual ~SweepAndPruneAlgorithm();                                // Destructor

        virtual void computePossibleCollisionPairs();                   // Compute the possible collision pairs of bodies
        virtual void notifyAddedBodies(std::vector<RigidBody*> bodies);      // Notify the broad-phase algorithm about new bodies in the physics world
        virtual void notifyRemovedBodies(std::vector<RigidBody*> bodies);    // Notify the broad-phase algorithm about removed bodies in the physics world
};

// Static method that compare two AABBs. This method will be used to compare to AABBs
// in order to sort them with the sort() function to obtain the sortedAABBs set.
// This method must return true if the AABB "a" goes before the AABB "b". We
// consider that "a" goes before "b" if the minimum value of "a" on the current
// sorting axis (sortAxis) is smaller than the minimum value of "b" on this same
// axis.
inline bool SweepAndPruneAlgorithm::compareAABBs(const AABB* a, const AABB* b) {
    return (a->getMinCoordinates()[sortAxis] < b->getMinCoordinates()[sortAxis]);
}

} // End of reactphysics3d namespace

#endif


