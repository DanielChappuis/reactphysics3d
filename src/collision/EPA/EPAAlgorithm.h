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

#ifndef EPAAlgorithm_H
#define EPAAlgorithm_H

// Libraries
#include "../GJK/Simplex.h"
#include "../body/NarrowBoundingVolume.h"
#include "ContactInfo.h"
#include "../mathematics/mathematics.h"

// ReactPhysics3D namespace
namespace reactphysics3d {

// Constants
const unsigned int MAX_SUPPORT_POINTS = 100;    // Maximum number of support points of the polytope
const unsigned int MAX_FACETS = 200;            // Maximum number of facets of the polytope

/*  -------------------------------------------------------------------
    Class EPAAlgorithm :
        This class is the implementation of the Expanding Polytope Algorithm (EPA).
        The EPA algorithm computes the penetration depth and contact points between
        two enlarged objects (with margin) where the original objects (without margin)
        intersect. The penetration depth of a pair of intersecting objects A and B is
        the length of a point on the boundary of the Minkowski sum (A-B) closest to the
        origin. The goal of the EPA algorithm is to start with an initial simplex polytope
        that contains the origin and expend it in order to find the point on the boundary
        of (A-B) that is closest to the origin. An initial simplex that contains origin
        has been computed wit GJK algorithm. The EPA Algorithm will extend this simplex
        polytope to find the correct penetration depth.
    -------------------------------------------------------------------
*/
class EPAAlgorithm {
    private:
        

        bool isOrigininInTetrahedron(const Vector3D& p1, const Vector3D& p2,
                                     const Vector3D& p3, const Vector3D& p4) const; // Return true if the origin is in the tetrahedron

    public:
        EPAAlgorithm();         // Constructor
        ~EPAAlgorithm();        // Destructor

        bool computePenetrationDepthAndContactPoints(Simplex simplex, const NarrowBoundingVolume* const boundingVolume1,
                                                     const NarrowBoundingVolume* const boundingVolume2,
                                                     Vector3D& v, ContactInfo*& contactInfo);                         // Compute the penetration depth with EPA algorithm
};

} // End of ReactPhysics3D namespace

#endif

