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
#include "../CollisionDetection.h"

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;
using namespace std;

// Constructor
NoBroadPhaseAlgorithm::NoBroadPhaseAlgorithm(CollisionDetection& collisionDetection)
                      : BroadPhaseAlgorithm(collisionDetection) {

}

// Destructor
NoBroadPhaseAlgorithm::~NoBroadPhaseAlgorithm() {

}

// Compute the possible collision pairs of bodies. This broad-phase algorithm
// doesn't do anything

void NoBroadPhaseAlgorithm::computePossibleCollisionPairs() {
    // For each pair of bodies
    for (vector<Body*>::iterator it1 = bodies.begin(); it1 != bodies.end(); it1++) {
        for (vector<Body*>::iterator it2 = it1+1; it2 != bodies.end(); it2++) {
            collisionDetection.broadPhaseNotifyOverlappingPair(*it1, *it2);
        }
    }
}

// Notify the broad-phase algorithm about new bodies in the physics world
void NoBroadPhaseAlgorithm::notifyAddedBodies(vector<Body*> addedBodies) {
    // Add the new bodies
    for (vector<Body*>::iterator it = addedBodies.begin(); it < addedBodies.end(); it++) {
        bodies.push_back(*it);
    }
}   

// Notify the broad-phase algorithm about removed bodies in the physics world
void NoBroadPhaseAlgorithm::notifyRemovedBodies(vector<Body*> removedBodies) {
    // Remove the bodies to be removed
    for (vector<Body*>::iterator it = removedBodies.begin(); it < removedBodies.end(); it++) {
        bodies.erase(std::find(bodies.begin(), bodies.end(), *it));
    }
} 
