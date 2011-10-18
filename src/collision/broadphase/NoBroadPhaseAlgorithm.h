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


