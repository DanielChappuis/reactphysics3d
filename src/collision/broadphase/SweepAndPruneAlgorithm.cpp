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
#include "SweepAndPruneAlgorithm.h"
#include "../CollisionDetection.h"
#include <algorithm>

// Namespaces
using namespace reactphysics3d;
using namespace std;

// Initialize the static attributes
unsigned short int SweepAndPruneAlgorithm::sortAxis = 0;

// Constructor
SweepAndPruneAlgorithm::SweepAndPruneAlgorithm(CollisionDetection& collisionDetection)
             :BroadPhaseAlgorithm(collisionDetection) {

}

// Destructor
SweepAndPruneAlgorithm::~SweepAndPruneAlgorithm() {

}

// Notify the broad-phase algorithm about new bodies in the physics world
// This method removes the AABB representation of a given set of bodies from the sortedAABBs set
void SweepAndPruneAlgorithm::notifyRemovedBodies(vector<RigidBody*> bodies) {
    vector<const AABB*>::iterator elemToRemove;
    const AABB* aabb;

    // Removed the AABB of the bodies that have been removed
    for (vector<RigidBody*>::iterator it = bodies.begin(); it != bodies.end(); ++it) {
        aabb = (*it)->getAABB();
        assert(aabb);
        elemToRemove = find(sortedAABBs.begin(), sortedAABBs.end(), aabb);
        assert((*elemToRemove) == aabb);
        sortedAABBs.erase(elemToRemove);
    }
}

// Notify the broad-phase algorithm about new bodies in the physics world
// This method adds the AABB representation of a given body in the sortedAABBs set
void SweepAndPruneAlgorithm::notifyAddedBodies(vector<RigidBody*> bodies) {
    const AABB* aabb;
    
    for (vector<RigidBody*>::iterator it = bodies.begin(); it != bodies.end(); ++it) {
        aabb = 0;
        aabb = (*it)->getAABB();
        assert(aabb);
        sortedAABBs.push_back(aabb);
    }
}

// This method computes the possible collision pairs of bodies and notify
// the collision detection object about overlapping pairs using the
// broadPhaseNotifyOverlappingPair() method from the CollisionDetection class
void SweepAndPruneAlgorithm::computePossibleCollisionPairs() {
    double variance[3];                             // Variance of the distribution of the AABBs on the three x, y and z axis
    double esperance[] = {0.0, 0.0, 0.0};           // Esperance of the distribution of the AABBs on the three x, y and z axis
    double esperanceSquare[] = {0.0, 0.0, 0.0};     // Esperance of the square of the distribution values of the AABBs on the three x, y and z axis
    vector<const AABB*>::iterator it;               // Iterator on the sortedAABBs set
    vector<const AABB*>::iterator it2;              // Second iterator
    Vector3 center3D;                               // Center of the current AABB
    double center[3];                               // Coordinates of the center of the current AABB
    int i;
    const Body* body;                               // Body pointer on the body corresponding to an AABB
    uint nbAABBs = sortedAABBs.size();              // Number of AABBs

    // Sort the set of AABBs
    sort(sortedAABBs.begin(), sortedAABBs.end(), compareAABBs);

    // Sweep the sorted set of AABBs
    for (vector<const AABB*>::iterator it = sortedAABBs.begin(); it != sortedAABBs.end(); ++it) {

        // If the collision of the AABB's corresponding body is disabled
        if (!(*it)->getBodyPointer()->getIsCollisionEnabled()) {
            // Go to the next AABB to test
            continue;
        }

        // Center of the current AABB
        center3D = (*it)->getCenter();
        center[0] = center3D.getX();
        center[1] = center3D.getY();
        center[2] = center3D.getZ();

        // Update the esperance and esperanceSquare values to compute the variance
        for (i=0; i<3; i++) {
            esperance[i] += center[i];
            esperanceSquare[i] += center[i] * center[i];
        }

        // Test collision against all possible overlapping AABBs following the current one
        for (it2 = it + 1; it2 != sortedAABBs.end(); it2++) {
            // Stop when the tested AABBs are beyond the end of the current AABB
            if ((*it2)->getMinCoordinates()[sortAxis] > (*it)->getMaxCoordinates()[sortAxis]) {
                break;
            }

            body = (*it2)->getBodyPointer();

            // Test if both AABBs overlap
            if (body->getIsCollisionEnabled() && (*it)->testCollision(*(*it2))) {
                // Notify the collision detection object about this current overlapping pair of bodies
                collisionDetection.broadPhaseNotifyOverlappingPair((*it)->getBodyPointer(), (*it2)->getBodyPointer());
            }
        }
    }

    // Compute the variance of the distribution of the AABBs on the three x,y and z axis
    for (i=0; i<3; i++) {
        variance[i] = esperanceSquare[i] - esperance[i] * esperance[i] / nbAABBs;
    }

    // Update the sorted axis according to the axis with the largest variance
    sortAxis = 0;
    if (variance[1] > variance[0]) sortAxis = 1;
    if (variance[2] > variance[sortAxis]) sortAxis = 2;
}


