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

#ifndef EPA_ALGORITHM_H
#define EPA_ALGORITHM_H

// Libraries
#include "../GJK/Simplex.h"
#include "../../body/Shape.h"
#include "../ContactInfo.h"
#include "../../mathematics/mathematics.h"
#include "TriangleEPA.h"
#include <algorithm>

// ReactPhysics3D namespace
namespace reactphysics3d {

// Constants
const unsigned int MAX_SUPPORT_POINTS = 100;    // Maximum number of support points of the polytope
const unsigned int MAX_FACETS = 200;            // Maximum number of facets of the polytope


// Class TriangleComparison that allow the comparison of two triangles in the heap
// The comparison between two triangles is made using their square distance to the closest
// point to the origin. The goal is that in the heap, the first triangle is the one with the
// smallest square distance.
class TriangleComparison {
    public:
        // Comparison operator
        bool operator()(const TriangleEPA* face1, const TriangleEPA* face2) {
            return (face1->getDistSquare() > face2->getDistSquare());
        }
};


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
        polytope to find the correct penetration depth. The implementation of the EPA
        algorithm is based on the book "Collision Detection in 3D Environments".
    -------------------------------------------------------------------
*/
class EPAAlgorithm {
    private:
        TriangleComparison triangleComparison;           // Triangle comparison operator

        void addFaceCandidate(TriangleEPA* triangle, TriangleEPA** heap,
                              uint& nbTriangles, double upperBoundSquarePenDepth);      // Add a triangle face in the candidate triangle heap
        int isOriginInTetrahedron(const Vector3D& p1, const Vector3D& p2,
                                  const Vector3D& p3, const Vector3D& p4) const;        // Decide if the origin is in the tetrahedron

    public:
        EPAAlgorithm();         // Constructor
        ~EPAAlgorithm();        // Destructor

        bool computePenetrationDepthAndContactPoints(Simplex simplex, const Shape* shape1, const Transform& transform1,
                                                     const Shape* shape2, const Transform& transform2,
                                                     Vector3D& v, ContactInfo*& contactInfo);                         // Compute the penetration depth with EPA algorithm
};

// Add a triangle face in the candidate triangle heap in the EPA algorithm
inline void EPAAlgorithm::addFaceCandidate(TriangleEPA* triangle, TriangleEPA** heap,
                                           uint& nbTriangles, double upperBoundSquarePenDepth) {
    
    // If the closest point of the affine hull of triangle points is internal to the triangle and
    // if the distance of the closest point from the origin is at most the penetration depth upper bound
    if (triangle->isClosestPointInternalToTriangle() && triangle->getDistSquare() <= upperBoundSquarePenDepth) {
        // Add the triangle face to the list of candidates
        heap[nbTriangles] = triangle;
        nbTriangles++;
        std::push_heap(&heap[0], &heap[nbTriangles], triangleComparison);
    }
}

} // End of ReactPhysics3D namespace

#endif

