/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2016 Daniel Chappuis                                       *
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

#ifndef REACTPHYSICS3D_EPA_ALGORITHM_H
#define REACTPHYSICS3D_EPA_ALGORITHM_H

// Libraries
#include "collision/narrowphase/GJK/Simplex.h"
#include "collision/shapes/CollisionShape.h"
#include "collision/CollisionShapeInfo.h"
#include "constraint/ContactPoint.h"
#include "collision/narrowphase/NarrowPhaseAlgorithm.h"
#include "mathematics/mathematics.h"
#include "TriangleEPA.h"
#include "memory/MemoryAllocator.h"
#include <algorithm>

/// ReactPhysics3D namespace
namespace reactphysics3d {

// ---------- Constants ---------- //

/// Maximum number of support points of the polytope
const unsigned int MAX_SUPPORT_POINTS = 100;

/// Maximum number of facets of the polytope
const unsigned int MAX_FACETS = 200;


// Class TriangleComparison
/**
 * This class allows the comparison of two triangles in the heap
 * The comparison between two triangles is made using their square distance to the closest
 * point to the origin. The goal is that in the heap, the first triangle is the one with the
 * smallest square distance.
 */
class TriangleComparison {

    public:

        /// Comparison operator
        bool operator()(const TriangleEPA* face1, const TriangleEPA* face2) {
            return (face1->getDistSquare() > face2->getDistSquare());
        }
};


// Class EPAAlgorithm
/**
 * This class is the implementation of the Expanding Polytope Algorithm (EPA).
 * The EPA algorithm computes the penetration depth and contact points between
 * two enlarged objects (with margin) where the original objects (without margin)
 * intersect. The penetration depth of a pair of intersecting objects A and B is
 * the length of a point on the boundary of the Minkowski sum (A-B) closest to the
 * origin. The goal of the EPA algorithm is to start with an initial simplex polytope
 * that contains the origin and expend it in order to find the point on the boundary
 * of (A-B) that is closest to the origin. An initial simplex that contains origin
 * has been computed wit GJK algorithm. The EPA Algorithm will extend this simplex
 * polytope to find the correct penetration depth. The implementation of the EPA
 * algorithm is based on the book "Collision Detection in 3D Environments".
 */
class EPAAlgorithm {

    private:

        // -------------------- Attributes -------------------- //

        /// Reference to the memory allocator
        MemoryAllocator* mMemoryAllocator;

        /// Triangle comparison operator
        TriangleComparison mTriangleComparison;
        
        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        EPAAlgorithm(const EPAAlgorithm& algorithm);

        /// Private assignment operator
        EPAAlgorithm& operator=(const EPAAlgorithm& algorithm);

        /// Add a triangle face in the candidate triangle heap
        void addFaceCandidate(TriangleEPA* triangle, TriangleEPA** heap, uint& nbTriangles,
                              decimal upperBoundSquarePenDepth);

        /// Decide if the origin is in the tetrahedron.
        int isOriginInTetrahedron(const Vector3& p1, const Vector3& p2,
                                  const Vector3& p3, const Vector3& p4) const;

    public:

        // -------------------- Methods -------------------- //

        /// Constructor
        EPAAlgorithm();

        /// Destructor
        ~EPAAlgorithm();

        /// Initalize the algorithm
        void init(MemoryAllocator* memoryAllocator);

        /// Compute the penetration depth with EPA algorithm.
        void computePenetrationDepthAndContactPoints(const Simplex& simplex,
                                                     CollisionShapeInfo shape1Info,
                                                     const Transform& transform1,
                                                     CollisionShapeInfo shape2Info,
                                                     const Transform& transform2,
                                                     Vector3& v,
                                                    NarrowPhaseCallback* narrowPhaseCallback);
};

// Add a triangle face in the candidate triangle heap in the EPA algorithm
inline void EPAAlgorithm::addFaceCandidate(TriangleEPA* triangle, TriangleEPA** heap,
                                           uint& nbTriangles, decimal upperBoundSquarePenDepth) {
    
    // If the closest point of the affine hull of triangle
    // points is internal to the triangle and if the distance
    // of the closest point from the origin is at most the
    // penetration depth upper bound
    if (triangle->isClosestPointInternalToTriangle() &&
        triangle->getDistSquare() <= upperBoundSquarePenDepth) {

        // Add the triangle face to the list of candidates
        heap[nbTriangles] = triangle;
        nbTriangles++;
        std::push_heap(&heap[0], &heap[nbTriangles], mTriangleComparison);
    }
}

// Initalize the algorithm
inline void EPAAlgorithm::init(MemoryAllocator* memoryAllocator) {
    mMemoryAllocator = memoryAllocator;
}

}

#endif

