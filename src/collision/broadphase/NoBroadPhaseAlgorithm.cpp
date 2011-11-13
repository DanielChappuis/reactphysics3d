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
