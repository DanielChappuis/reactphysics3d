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

