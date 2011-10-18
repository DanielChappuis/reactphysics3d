/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2010 Daniel Chappuis                                            *
*********************************************************************************
*                                                                               *
* Permission is hereby granted, free of charge, to any person obtaining a copy  *
* of this software and associated documentation files (the "Software"), to deal *
* in the Software without restriction, including without limitation the rights  *
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell     *
* copies of the Software, and to permit persons to whom the Software is         *
* furnished to do so, subject to the following conditions:                      *
*                                                                               *
* The above copyright notice and this permission notice shall be included in    *
* all copies or substantial portions of the Software.                           *
*                                                                               *
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR    *
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,      *
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE   *
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER        *
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, *
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN     *
* THE SOFTWARE.                                                                 *
********************************************************************************/

#ifndef SAP_ALGORITHM_H
#define SAP_ALGORITHM_H

// Libraries
#include "BroadPhaseAlgorithm.h"
#include "../shapes/AABB.h"

// TODO : Rename this class SweepAndPruneAlgorithm

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
        

    public :
        SAPAlgorithm(CollisionDetection& collisionDetection);   // Constructor
        virtual ~SAPAlgorithm();                                // Destructor

        virtual void computePossibleCollisionPairs();                   // Compute the possible collision pairs of bodies
        virtual void notifyAddedBodies(std::vector<Body*> bodies);      // Notify the broad-phase algorithm about new bodies in the physics world
        virtual void notifyRemovedBodies(std::vector<Body*> bodies);    // Notify the broad-phase algorithm about removed bodies in the physics world
};

// Static method that compare two AABBs. This method will be used to compare to AABBs
// in order to sort them with the sort() function to obtain the sortedAABBs set.
// This method must return true if the AABB "a" goes before the AABB "b". We
// consider that "a" goes before "b" if the minimum value of "a" on the current
// sorting axis (sortAxis) is smaller than the minimum value of "b" on this same
// axis.
inline bool SAPAlgorithm::compareAABBs(const AABB* a, const AABB* b) {
    return (a->getMinCoordinates()[sortAxis] < b->getMinCoordinates()[sortAxis]);
}

} // End of reactphysics3d namespace

#endif


