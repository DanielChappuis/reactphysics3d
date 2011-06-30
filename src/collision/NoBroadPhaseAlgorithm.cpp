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

// Libraries
#include "NoBroadPhaseAlgorithm.h"

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor
NoBroadPhaseAlgorithm::NoBroadPhaseAlgorithm() {

}

// Destructor
NoBroadPhaseAlgorithm::~NoBroadPhaseAlgorithm() {

}

// Compute the possible collision pairs of bodies
// The arguments "addedBodies" and "removedBodies" are respectively the set
// of bodies that have been added and removed since the last broad-phase
// computation. Before the call, the argument "possibleCollisionPairs"
// correspond to the possible colliding pairs of bodies from the last broad-phase
// computation. This methods computes the current possible collision pairs of
// bodies and update the "possibleCollisionPairs" argument. This broad-phase
// algorithm doesn't do anything and therefore the "possibleCollisionPairs" set
// must contains all the possible pairs of bodies
void NoBroadPhaseAlgorithm::computePossibleCollisionPairs(std::vector<Body*> addedBodies, std::vector<Body*> removedBodies,
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
