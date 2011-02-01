
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
#include "GJKAlgorithm.h"
#include "Simplex.h"
#include "../constraint/Contact.h"
#include "../constants.h"
#include <algorithm>
#include <cmath>
#include <cfloat>
#include <cassert>


// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor
GJKAlgorithm::GJKAlgorithm() : lastSeparatingAxis(1.0, 1.0, 1.0) {
    // TODO : Check if we can initialize the last separating axis as above

    
}

// Destructor
GJKAlgorithm::~GJKAlgorithm() {

}

// Return true and compute a contact info if the two bounding volume collide.
// The method returns false if there is no collision between the two bounding volumes.
bool GJKAlgorithm::testCollision(const NarrowBoundingVolume* const boundingVolume1,
                                 const NarrowBoundingVolume* const boundingVolume2,
                                 ContactInfo*& contactInfo) {
    Vector3D suppA;             // Support point of object A
    Vector3D suppB;             // Support point of object B
    Vector3D w;                 // Support point of Minkowski difference A-B
    Vector3D pA;                // Closest point of object A
    Vector3D pB;                // Closest point of object B
    double vDotw;
    double prevDistSquare;

    assert(boundingVolume1 != boundingVolume2);

    // Initialize the margin (sum of margins of both objects)
    double margin = 2 * OBJECT_MARGIN;
    double marginSquare = margin * margin;
    assert(margin > 0.0);

    // Create a simplex set
    Simplex simplex;

    // Get the last point V (last separating axis)
    Vector3D v = lastSeparatingAxis;

    // Initialize the upper bound for the square distance
    double distSquare = DBL_MAX;
    
    do {
        // Compute the support points for object A and B
        suppA = boundingVolume1->getSupportPoint(v.getOpposite());
        suppB = boundingVolume2->getSupportPoint(v);

        // Compute the support point for the Minkowski difference A-B
        w = suppA - suppB;
        
        vDotw = v.scalarProduct(w);
        
        // If the enlarge objects (with margins) do not intersect
        if (vDotw > 0.0 && vDotw * vDotw > distSquare * marginSquare) {
            // No intersection, we return false
            return false;
        }

        // If the objects intersect only in the margins
        if (simplex.isPointInSimplex(w) || distSquare - vDotw <= distSquare * REL_ERROR_SQUARE) {
            // Compute the closet points of both objects (without the margins)
            simplex.computeClosestPointsOfAandB(pA, pB);

            // Project those two points on the margins to have the closest points of both
            // object with the margins
            double dist = sqrt(distSquare);
            assert(dist > 0.0);
            pA = pA - (OBJECT_MARGIN / dist) * v;
            pB = pB + (OBJECT_MARGIN / dist) * v;

            // Compute the contact info
            Vector3D normal = (pB - pA).getUnit();
            double penetrationDepth = margin - dist;
            assert(penetrationDepth > 0.0);
            contactInfo = new ContactInfo(boundingVolume1->getBodyPointer(), boundingVolume2->getBodyPointer(),
                                          normal, penetrationDepth, pA, pB);

            // There is an intersection, therefore we return true
            return true;
        }

        // Add the current point to the simplex
        simplex.addPoint(w, suppA, suppB);

        // If the simplex is affinely dependent
        if (simplex.isAffinelyDependent()) {
            // Compute the closet points of both objects (without the margins)
            simplex.computeClosestPointsOfAandB(pA, pB);

            // Project those two points on the margins to have the closest points of both
            // object with the margins
            double dist = sqrt(distSquare);
            assert(dist > 0.0);
            pA = pA - (OBJECT_MARGIN / dist) * v;
            pB = pB + (OBJECT_MARGIN / dist) * v;

            // Compute the contact info
            Vector3D normal = (pB - pA).getUnit();
            double penetrationDepth = margin - dist;
            assert(penetrationDepth > 0.0);
            contactInfo = new ContactInfo(boundingVolume1->getBodyPointer(), boundingVolume2->getBodyPointer(),
                                          normal, penetrationDepth, pA, pB);

            // There is an intersection, therefore we return true
            return true;
        }

        // Compute the point of the simplex closest to the origin
        // If the computation of the closest point fail
        if (!simplex.computeClosestPoint(v)) {
            // Compute the closet points of both objects (without the margins)
            simplex.computeClosestPointsOfAandB(pA, pB);

            // Project those two points on the margins to have the closest points of both
            // object with the margins
            double dist = sqrt(distSquare);
            assert(dist > 0.0);
            pA = pA - (OBJECT_MARGIN / dist) * v;
            pB = pB + (OBJECT_MARGIN / dist) * v;

            // Compute the contact info
            Vector3D normal = (pB - pA).getUnit();
            double penetrationDepth = margin - dist;
            assert(penetrationDepth > 0.0);
            contactInfo = new ContactInfo(boundingVolume1->getBodyPointer(), boundingVolume2->getBodyPointer(),
                                          normal, penetrationDepth, pA, pB);

            // There is an intersection, therefore we return true
            return true;
        }

        // Store and update the squared distance of the closest point
        prevDistSquare = distSquare;
        distSquare = v.scalarProduct(v);

        // If the distance to the closest point doesn't improve a lot
        if (prevDistSquare - distSquare <= MACHINE_EPSILON * prevDistSquare) {
            simplex.backupClosestPointInSimplex(v);
            
            // Get the new squared distance
            distSquare = v.scalarProduct(v);

            // Compute the closet points of both objects (without the margins)
            simplex.computeClosestPointsOfAandB(pA, pB);

            // Project those two points on the margins to have the closest points of both
            // object with the margins
            double dist = sqrt(distSquare);
            assert(dist > 0.0);
            pA = pA - (OBJECT_MARGIN / dist) * v;
            pB = pB + (OBJECT_MARGIN / dist) * v;

            // Compute the contact info
            Vector3D normal = (pB - pA).getUnit();
            double penetrationDepth = margin - dist;
            assert(penetrationDepth > 0.0);
            contactInfo = new ContactInfo(boundingVolume1->getBodyPointer(), boundingVolume2->getBodyPointer(),
                                          normal, penetrationDepth, pA, pB);

            // There is an intersection, therefore we return true
            return true;
        }
        
    } while(!simplex.isFull() && distSquare > MACHINE_EPSILON * simplex.getMaxLengthSquareOfAPoint());

    // The objects (without margins) intersect
}


