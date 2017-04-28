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

#ifndef REACTPHYSICS3D_SAT_ALGORITHM_H
#define REACTPHYSICS3D_SAT_ALGORITHM_H

// Libraries
#include "collision/ContactManifoldInfo.h"
#include "collision/NarrowPhaseInfo.h"
#include "collision/shapes/ConvexPolyhedronShape.h"

/// ReactPhysics3D namespace
namespace reactphysics3d {

// Class SATAlgorithm
class SATAlgorithm {

    private :

        // -------------------- Attributes -------------------- //

        // -------------------- Methods -------------------- //

        /// Return true if the arcs AB and CD on the Gauss Map intersect
        bool testGaussMapArcsIntersect(const Vector3& a, const Vector3& b,
                                       const Vector3& c, const Vector3& d) const;

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        SATAlgorithm() = default;

        /// Destructor
        ~SATAlgorithm() = default;

        /// Deleted copy-constructor
        SATAlgorithm(const SATAlgorithm& algorithm) = delete;

        /// Deleted assignment operator
        SATAlgorithm& operator=(const SATAlgorithm& algorithm) = delete;

        /// Test collision between a sphere and a convex mesh
        bool testCollisionSphereVsConvexPolyhedron(const NarrowPhaseInfo* narrowPhaseInfo, ContactManifoldInfo& contactManifoldInfo) const;

        /// Test collision between a capsule and a convex mesh
        bool testCollisionCapsuleVsConvexPolyhedron(const NarrowPhaseInfo* narrowPhaseInfo, ContactManifoldInfo& contactManifoldInfo) const;

        /// Compute the two contact points between a polyhedron and a capsule when the separating axis is a face normal of the polyhedron
        void computeCapsulePolyhedronFaceContactPoints(uint referenceFaceIndex, decimal capsuleRadius, const ConvexPolyhedronShape* polyhedron,
                                                       decimal penetrationDepth, const Transform& polyhedronToCapsuleTransform,
                                                       const Vector3& normalWorld, const Vector3& separatingAxisCapsuleSpace,
                                                       const Vector3& capsuleSegAPolyhedronSpace, const Vector3& capsuleSegBPolyhedronSpace,
                                                       ContactManifoldInfo& contactManifoldInfo, bool isCapsuleShape1) const;

        // This method returns true if an edge of a polyhedron and a capsule forms a face of the Minkowski Difference
        bool isMinkowskiFaceCapsuleVsEdge(const Vector3& capsuleSegment, const Vector3& edgeAdjacentFace1Normal,
                                          const Vector3& edgeAdjacentFace2Normal) const;

        /// Test collision between a triangle and a convex mesh
        bool testCollisionTriangleVsConvexMesh(const NarrowPhaseInfo* narrowPhaseInfo, ContactManifoldInfo& contactManifoldInfo) const;

        /// Test collision between two convex meshes
        bool testCollisionConvexMeshVsConvexMesh(const NarrowPhaseInfo* narrowPhaseInfo, ContactManifoldInfo& contactManifoldInfo) const;

};

}

#endif
