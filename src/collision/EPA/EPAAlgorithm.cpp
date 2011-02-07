/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2011 Daniel Chappuis                                            *
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
#include "EPAAlgorithm.h"

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor
EPAAlgorithm::EPAAlgorithm() {

}

// Destructor
EPAAlgorithm::~EPAAlgorithm() {
    
}

// Compute the penetration depth with the EPA algorithms
// This method computes the penetration depth and contact points between two
// enlarged objects (with margin) where the original objects (without margin)
// intersect. An initial simplex that contains origin has been computed with
// GJK algorithm. The EPA Algorithm will extend this simplex polytope to find
// the correct penetration depth
bool EPAAlgorithm::computePenetrationDepthAndContactPoints(Simplex simplex, const NarrowBoundingVolume* const boundingVolume1,
                                                           const NarrowBoundingVolume* const boundingVolume2,
                                                           Vector3D& v, ContactInfo*& contactInfo) {
    Vector3D suppPointsA[MAX_SUPPORT_POINTS];       // Support points of object A in local coordinates
    Vector3D suppPointsB[MAX_SUPPORT_POINTS];       // Support points of object B in local coordinates
    Vector3D points[MAX_SUPPORT_POINTS];            // Current points

    // TODO : Check that we call all the supportPoint() function with a margin

    // Get the simplex computed previously by the GJK algorithm
    unsigned int nbVertices = simplex.getSimplex(suppPointsA, suppPointsB, points);

    // Compute the tolerance
    double tolerance = MACHINE_EPSILON * simplex.getMaxLengthSquareOfAPoint();

    // Number of triangles in the polytope
    unsigned int nbTriangles = 0;

    // Select an action according to the number of points in the simplex computed with GJK algorithm
    switch(nbVertices) {
        case 1:
            // Only one point in the simplex (which should be the origin). We have a touching contact
            // with zero penetration depth. We drop that kind of contact. Therefore, we return false
            return false;

        case 2: {
            
        }
    }
}
