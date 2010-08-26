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

#ifndef SAPALGORITHM_H
#define SAPALGORITHM_H

// Libraries
#include "BroadPhaseAlgorithm.h"
#include "../body/AABB.h"

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
class SAPAlgorithm : public BroadPhaseAlgorithm {
    protected :
        std::vector<const AABB*> sortedAABBs;   // Sorted set of AABB of the bodies on one of the x.y or z axis
        static unsigned short int sortAxis;     // Current sorting axis (0 for x, 1 for y, 2 for z axis)

        static bool compareAABBs(const AABB* a, const AABB* b);     // Static method that compare two AABBs (in order to sort them)
        void removeBodiesAABB(std::vector<Body*> bodies);           // Remove the AABB representation of a given set of bodies from the sortedAABBs set
        void addBodiesAABB(std::vector<Body*> bodies);              // Add the AABB representation of a given set of bodies in the sortedAABBs set

    public :
        SAPAlgorithm();                                             // Constructor
        virtual ~SAPAlgorithm();                                    // Destructor

        virtual void computePossibleCollisionPairs(std::vector<Body*> addedBodies, std::vector<Body*> removedBodies,
                                                   std::vector<std::pair<const Body*, const Body* > >& possibleCollisionPairs);     // Compute the possible collision pairs of bodies
};

// Static method that compare two AABBs. This method will be used to compare to AABBs
// in order to sort them with the sort() function to obtain the sortedAABBs set.
// This method must return true if the AABB "a" goes before the AABB "b". We
// consider that "a" goes before "b" if the minimum value of "a" on the current
// sorting axis (sortAxis) is smaller than the minimum value of "b" on this same
// axis.
inline bool SAPAlgorithm::compareAABBs(const AABB* a, const AABB* b) {
    return (a->getMinValueOnAxis(sortAxis) < b->getMinValueOnAxis(sortAxis));
}

} // End of reactphysics3d namespace

#endif


