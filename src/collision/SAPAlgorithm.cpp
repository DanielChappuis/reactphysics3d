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
#include "SAPAlgorithm.h"
#include <algorithm>

// Namespaces
using namespace reactphysics3d;
using namespace std;

// Initialize the static attributes
unsigned short int SAPAlgorithm::sortAxis = 0;

// Constructor
SAPAlgorithm::SAPAlgorithm() {

}

// Destructor
SAPAlgorithm::~SAPAlgorithm() {

}

// Remove the AABB representation of a given set of bodies from the sortedAABBs set
void SAPAlgorithm::removeBodiesAABB(vector<Body*> bodies) {
    vector<const AABB*>::iterator elemToRemove;
    const AABB* aabb;

    // Removed the AABB of the bodies that have been removed
    for (vector<Body*>::iterator it = bodies.begin(); it != bodies.end(); ++it) {
        aabb = dynamic_cast<const AABB*>((*it)->getBroadBoundingVolume());
        assert(aabb);
        elemToRemove = find(sortedAABBs.begin(), sortedAABBs.end(), aabb);
        assert((*elemToRemove) == aabb);
        sortedAABBs.erase(elemToRemove);
    }
}

// Add the AABB representation of a given body in the sortedAABBs set
void SAPAlgorithm::addBodiesAABB(vector<Body*> bodies) {
    const AABB* aabb;
    
    for (vector<Body*>::iterator it = bodies.begin(); it != bodies.end(); ++it) {
        aabb = 0;
        aabb = dynamic_cast<const AABB*>((*it)->getBroadBoundingVolume());
        assert(aabb);
        sortedAABBs.push_back(aabb);
    }
}

// Compute the possible collision pairs of bodies
// The arguments "addedBodies" and "removedBodies" are respectively the set
// of bodies that have been added and removed since the last broad-phase
// computation. Before the call, the argument "possibleCollisionPairs"
// correspond to the possible colliding pairs of bodies from the last broad-phase
// computation. This methods computes the current possible collision pairs of
// bodies and update the "possibleCollisionPairs" argument.
void SAPAlgorithm::computePossibleCollisionPairs(vector<Body*> addedBodies, vector<Body*> removedBodies,
                                                 vector<pair<const Body*, const Body* > >& possibleCollisionPairs) {
    double variance[3];                             // Variance of the distribution of the AABBs on the three x, y and z axis
    double esperance[] = {0.0, 0.0, 0.0};           // Esperance of the distribution of the AABBs on the three x, y and z axis
    double esperanceSquare[] = {0.0, 0.0, 0.0};     // Esperance of the square of the distribution values of the AABBs on the three x, y and z axis
    vector<const AABB*>::iterator it;               // Iterator on the sortedAABBs set
    vector<const AABB*>::iterator it2;              // Second iterator
    Vector3D center3D;                              // Center of the current AABB
    double center[3];                               // Coordinates of the center of the current AABB
    int i;
    const Body* body;                               // Body pointer on the body corresponding to an AABB
    uint nbAABBs = sortedAABBs.size();              // Number of AABBs

    // Removed the bodies to remove
    removeBodiesAABB(removedBodies);

    // Add the bodies to add
    addBodiesAABB(addedBodies);

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
        for (it2 = it + 1; it2 != sortedAABBs.end(); ++it2) {
            // Stop when the tested AABBs are beyond the end of the current AABB
            if ((*it2)->getMinValueOnAxis(sortAxis) > (*it)->getMaxValueOnAxis(sortAxis)) {
                break;
            }

            body = (*it2)->getBodyPointer();

            // Test if both AABBs overlap
            if (body->getIsCollisionEnabled() && (*it)->testCollision(*(*it2))) {
                // Add the current pair of AABBs into the possibleCollisionPairs set
                possibleCollisionPairs.push_back(make_pair((*it)->getBodyPointer(), (*it2)->getBodyPointer()));
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


