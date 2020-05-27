/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2020 Daniel Chappuis                                       *
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
#include <reactphysics3d/decimal.h>
#include <reactphysics3d/collision/HalfEdgeStructure.h>

/// ReactPhysics3D namespace
namespace reactphysics3d {

// Declarations
class CapsuleShape;
class SphereShape;
class ContactManifoldInfo;
struct NarrowPhaseInfoBatch;
class ConvexPolyhedronShape;
class MemoryAllocator;
class Profiler;

// Class SATAlgorithm
/**
 * This class implements the Separating Axis Theorem algorithm (SAT).
 * This algorithm is used to find the axis of minimum penetration between two convex polyhedra.
 * If none is found, the objects are separated. Otherwise, the two objects are
 * in contact and we use clipping to get the contact points.
 */
class SATAlgorithm {

    private :

        // -------------------- Attributes -------------------- //

        /// Relative and absolute bias used to make sure the SAT algorithm returns the same penetration axis between frames
        /// when there are multiple separating axis with almost the same penetration depth. The goal is to
        /// make sure the contact manifold does not change too much between frames for better stability.
        static const decimal SEPARATING_AXIS_RELATIVE_TOLERANCE;
        static const decimal SEPARATING_AXIS_ABSOLUTE_TOLERANCE;

        /// True means that if two shapes were colliding last time (previous frame) and are still colliding
        /// we use the previous (minimum penetration depth) axis to clip the colliding features and we don't
        /// recompute a new (minimum penetration depth) axis. This value must be true for a dynamic simulation
        /// because it uses temporal coherence and clip the colliding features with the previous
        /// axis (this is good for stability). However, when we use the testCollision() methods, the penetration
        /// depths might be very large and we always want the current true axis with minimum penetration depth.
        /// In this case, this value must be set to false. Consider the following situation. Two shapes start overlaping
        /// with "x" being the axis of minimum penetration depth. Then, if the shapes move but are still penetrating,
        /// it is possible that the axis of minimum penetration depth changes for axis "y" for instance. If this value
        /// is true, we will always use the axis of the previous collision test and therefore always report that the
        /// penetrating axis is "x" even if it has changed to axis "y" during the collision. This is not what we want
        /// when we call the testCollision() methods.
        bool mClipWithPreviousAxisIfStillColliding;

        /// Memory allocator
        MemoryAllocator& mMemoryAllocator;

#ifdef IS_RP3D_PROFILING_ENABLED

		/// Pointer to the profiler
		Profiler* mProfiler;

#endif

        // -------------------- Methods -------------------- //

        /// Return true if two edges of two polyhedrons build a minkowski face (and can therefore be a separating axis)
        bool testEdgesBuildMinkowskiFace(const ConvexPolyhedronShape* polyhedron1, const HalfEdgeStructure::Edge& edge1,
                                         const ConvexPolyhedronShape* polyhedron2, const HalfEdgeStructure::Edge& edge2,
                                         const Transform& polyhedron1ToPolyhedron2) const;

        /// Return true if the arcs AB and CD on the Gauss Map intersect
        bool testGaussMapArcsIntersect(const Vector3& a, const Vector3& b,
                                       const Vector3& c, const Vector3& d,
                                       const Vector3& bCrossA, const Vector3& dCrossC) const;

        /// Compute and return the distance between the two edges in the direction of the candidate separating axis
        decimal computeDistanceBetweenEdges(const Vector3& edge1A, const Vector3& edge2A,
                                            const Vector3& polyhedron1Centroid, const Vector3& polyhedron2Centroid,
                                            const Vector3& edge1Direction, const Vector3& edge2Direction,
                                            bool isShape1Triangle, Vector3& outSeparatingAxis) const;

        /// Return the penetration depth between two polyhedra along a face normal axis of the first polyhedron
        decimal testSingleFaceDirectionPolyhedronVsPolyhedron(const ConvexPolyhedronShape* polyhedron1,
                                                              const ConvexPolyhedronShape* polyhedron2,
                                                              const Transform& polyhedron1ToPolyhedron2,
                                                              uint faceIndex) const;


        /// Test all the normals of a polyhedron for separating axis in the polyhedron vs polyhedron case
        decimal testFacesDirectionPolyhedronVsPolyhedron(const ConvexPolyhedronShape* polyhedron1, const ConvexPolyhedronShape* polyhedron2,
                                                        const Transform& polyhedron1ToPolyhedron2, uint& minFaceIndex) const;

        /// Compute the penetration depth between a face of the polyhedron and a sphere along the polyhedron face normal direction
        decimal computePolyhedronFaceVsSpherePenetrationDepth(uint faceIndex, const ConvexPolyhedronShape* polyhedron,
                                                              const SphereShape* sphere, const Vector3& sphereCenter) const;

        /// Compute the penetration depth between the face of a polyhedron and a capsule along the polyhedron face normal direction
        decimal computePolyhedronFaceVsCapsulePenetrationDepth(uint polyhedronFaceIndex, const ConvexPolyhedronShape* polyhedron,
                                                               const CapsuleShape* capsule, const Transform& polyhedronToCapsuleTransform,
                                                               Vector3& outFaceNormalCapsuleSpace) const;

        /// Compute the penetration depth when the separating axis is the cross product of polyhedron edge and capsule inner segment
        decimal computeEdgeVsCapsuleInnerSegmentPenetrationDepth(const ConvexPolyhedronShape* polyhedron, const CapsuleShape* capsule,
                                                                 const Vector3& capsuleSegmentAxis, const Vector3& edgeVertex1,
                                                                 const Vector3& edgeDirectionCapsuleSpace,
                                                                 const Transform& polyhedronToCapsuleTransform, Vector3& outAxis) const;

        /// Compute the contact points between two faces of two convex polyhedra.
        bool computePolyhedronVsPolyhedronFaceContactPoints(bool isMinPenetrationFaceNormalPolyhedron1, const ConvexPolyhedronShape* polyhedron1,
                                                            const ConvexPolyhedronShape* polyhedron2, const Transform& polyhedron1ToPolyhedron2,
                                                            const Transform& polyhedron2ToPolyhedron1, uint minFaceIndex,
                                                            NarrowPhaseInfoBatch& narrowPhaseInfoBatch, uint batchIndex) const;


    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        SATAlgorithm(bool clipWithPreviousAxisIfStillColliding, MemoryAllocator& memoryAllocator);

        /// Destructor
        ~SATAlgorithm() = default;

        /// Deleted copy-constructor
        SATAlgorithm(const SATAlgorithm& algorithm) = delete;

        /// Deleted assignment operator
        SATAlgorithm& operator=(const SATAlgorithm& algorithm) = delete;

        /// Test collision between a sphere and a convex mesh
        bool testCollisionSphereVsConvexPolyhedron(NarrowPhaseInfoBatch& narrowPhaseInfoBatch,
                                                   uint batchStartIndex, uint batchNbItems) const;

        /// Test collision between a capsule and a convex mesh
        bool testCollisionCapsuleVsConvexPolyhedron(NarrowPhaseInfoBatch& narrowPhaseInfoBatch, uint batchIndex) const;

        /// Compute the two contact points between a polyhedron and a capsule when the separating axis is a face normal of the polyhedron
        bool computeCapsulePolyhedronFaceContactPoints(uint referenceFaceIndex, decimal capsuleRadius, const ConvexPolyhedronShape* polyhedron,
                                                       decimal penetrationDepth, const Transform& polyhedronToCapsuleTransform,
                                                       Vector3& normalWorld, const Vector3& separatingAxisCapsuleSpace,
                                                       const Vector3& capsuleSegAPolyhedronSpace, const Vector3& capsuleSegBPolyhedronSpace,
                                                       NarrowPhaseInfoBatch& narrowPhaseInfoBatch, uint batchIndex, bool isCapsuleShape1) const;

        // This method returns true if an edge of a polyhedron and a capsule forms a face of the Minkowski Difference
        bool isMinkowskiFaceCapsuleVsEdge(const Vector3& capsuleSegment, const Vector3& edgeAdjacentFace1Normal,
                                          const Vector3& edgeAdjacentFace2Normal) const;

        /// Test collision between two convex meshes
        bool testCollisionConvexPolyhedronVsConvexPolyhedron(NarrowPhaseInfoBatch& narrowPhaseInfoBatch, uint batchStartIndex, uint batchNbItems) const;

#ifdef IS_RP3D_PROFILING_ENABLED

		/// Set the profiler
		void setProfiler(Profiler* profiler);

#endif

};

#ifdef IS_RP3D_PROFILING_ENABLED

// Set the profiler
inline void SATAlgorithm::setProfiler(Profiler* profiler) {

	mProfiler = profiler;
}

#endif

}

#endif
