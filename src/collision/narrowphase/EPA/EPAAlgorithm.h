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

#ifndef EPA_ALGORITHM_H
#define EPA_ALGORITHM_H

// Libraries
#include "../GJK/Simplex.h"
#include "../../../shapes/Collider.h"
#include "../../ContactInfo.h"
#include "../../../mathematics/mathematics.h"
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
                              uint& nbTriangles, decimal upperBoundSquarePenDepth);      // Add a triangle face in the candidate triangle heap
        int isOriginInTetrahedron(const Vector3& p1, const Vector3& p2,
                                  const Vector3& p3, const Vector3& p4) const;        // Decide if the origin is in the tetrahedron

    public:
        EPAAlgorithm();         // Constructor
        ~EPAAlgorithm();        // Destructor

        bool computePenetrationDepthAndContactPoints(Simplex simplex, const Collider* collider1, const Transform& transform1,
                                                     const Collider* collider2, const Transform& transform2,
                                                     Vector3& v, ContactInfo*& contactInfo);                         // Compute the penetration depth with EPA algorithm
};

// Add a triangle face in the candidate triangle heap in the EPA algorithm
inline void EPAAlgorithm::addFaceCandidate(TriangleEPA* triangle, TriangleEPA** heap,
                                           uint& nbTriangles, decimal upperBoundSquarePenDepth) {
    
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

