/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2019 Daniel Chappuis                                       *
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
#include "SATAlgorithm.h"
#include "constraint/ContactPoint.h"
#include "collision/PolyhedronMesh.h"
#include "collision/shapes/CapsuleShape.h"
#include "collision/shapes/SphereShape.h"
#include "engine/OverlappingPair.h"
#include "collision/shapes/TriangleShape.h"
#include "collision/NarrowPhaseInfo.h"
#include "configuration.h"
#include "utils/Profiler.h"
#include <cassert>

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Static variables initialization
const decimal SATAlgorithm::SEPARATING_AXIS_RELATIVE_TOLERANCE = decimal(1.002);
const decimal SATAlgorithm::SEPARATING_AXIS_ABSOLUTE_TOLERANCE = decimal(0.0005);

// Constructor
SATAlgorithm::SATAlgorithm(MemoryAllocator& memoryAllocator) : mMemoryAllocator(memoryAllocator) {

#ifdef IS_PROFILING_ACTIVE
        mProfiler = nullptr;
#endif

}

// Test collision between a sphere and a convex mesh
bool SATAlgorithm::testCollisionSphereVsConvexPolyhedron(NarrowPhaseInfo* narrowPhaseInfo, bool reportContacts) const {

    RP3D_PROFILE("SATAlgorithm::testCollisionSphereVsConvexPolyhedron()", mProfiler);

    bool isSphereShape1 = narrowPhaseInfo->collisionShape1->getType() == CollisionShapeType::SPHERE;

    assert(narrowPhaseInfo->collisionShape1->getType() == CollisionShapeType::CONVEX_POLYHEDRON ||
           narrowPhaseInfo->collisionShape2->getType() == CollisionShapeType::CONVEX_POLYHEDRON);
    assert(narrowPhaseInfo->collisionShape1->getType() == CollisionShapeType::SPHERE ||
           narrowPhaseInfo->collisionShape2->getType() == CollisionShapeType::SPHERE);

    // Get the capsule collision shapes
    const SphereShape* sphere = static_cast<const SphereShape*>(isSphereShape1 ? narrowPhaseInfo->collisionShape1 : narrowPhaseInfo->collisionShape2);
    const ConvexPolyhedronShape* polyhedron = static_cast<const ConvexPolyhedronShape*>(isSphereShape1 ? narrowPhaseInfo->collisionShape2 : narrowPhaseInfo->collisionShape1);

    const Transform& sphereToWorldTransform = isSphereShape1 ? narrowPhaseInfo->shape1ToWorldTransform : narrowPhaseInfo->shape2ToWorldTransform;
    const Transform& polyhedronToWorldTransform = isSphereShape1 ? narrowPhaseInfo->shape2ToWorldTransform : narrowPhaseInfo->shape1ToWorldTransform;

    // Get the transform from sphere local-space to polyhedron local-space
    const Transform worldToPolyhedronTransform = polyhedronToWorldTransform.getInverse();
    const Transform sphereToPolyhedronSpaceTransform = worldToPolyhedronTransform * sphereToWorldTransform;

    // Transform the center of the sphere into the local-space of the convex polyhedron
    const Vector3 sphereCenter = sphereToPolyhedronSpaceTransform.getPosition();

    // Minimum penetration depth
    decimal minPenetrationDepth = DECIMAL_LARGEST;
    uint minFaceIndex = 0;

    // For each face of the convex mesh
    for (uint f = 0; f < polyhedron->getNbFaces(); f++) {

        // Compute the penetration depth of the shapes along the face normal direction
        decimal penetrationDepth = computePolyhedronFaceVsSpherePenetrationDepth(f, polyhedron, sphere, sphereCenter);

        // If the penetration depth is negative, we have found a separating axis
        if (penetrationDepth <= decimal(0.0)) {

            return false;
        }

        // Check if we have found a new minimum penetration axis
        if (penetrationDepth < minPenetrationDepth) {
            minPenetrationDepth = penetrationDepth;
            minFaceIndex = f;
        }
    }

    if (reportContacts) {

        const Vector3 minFaceNormal = polyhedron->getFaceNormal(minFaceIndex);
        Vector3 minFaceNormalWorld = polyhedronToWorldTransform.getOrientation() * minFaceNormal;
        Vector3 contactPointSphereLocal = sphereToWorldTransform.getInverse().getOrientation() * (-minFaceNormalWorld * sphere->getRadius());
        Vector3 contactPointPolyhedronLocal = sphereCenter + minFaceNormal * (minPenetrationDepth - sphere->getRadius());

        Vector3 normalWorld = isSphereShape1 ? -minFaceNormalWorld : minFaceNormalWorld;

        // Compute smooth triangle mesh contact if one of the two collision shapes is a triangle
        TriangleShape::computeSmoothTriangleMeshContact(narrowPhaseInfo->collisionShape1, narrowPhaseInfo->collisionShape2,
                                                        isSphereShape1 ? contactPointSphereLocal : contactPointPolyhedronLocal,
                                                        isSphereShape1 ? contactPointPolyhedronLocal : contactPointSphereLocal,
                                                        narrowPhaseInfo->shape1ToWorldTransform, narrowPhaseInfo->shape2ToWorldTransform,
                                                        minPenetrationDepth, normalWorld);

        // Create the contact info object
        narrowPhaseInfo->addContactPoint(normalWorld, minPenetrationDepth,
                                         isSphereShape1 ? contactPointSphereLocal : contactPointPolyhedronLocal,
                                         isSphereShape1 ? contactPointPolyhedronLocal : contactPointSphereLocal);
    }

    return true;
}

// Compute the penetration depth between a face of the polyhedron and a sphere along the polyhedron face normal direction
decimal SATAlgorithm::computePolyhedronFaceVsSpherePenetrationDepth(uint faceIndex, const ConvexPolyhedronShape* polyhedron,
                                                                    const SphereShape* sphere, const Vector3& sphereCenter) const {

    RP3D_PROFILE("SATAlgorithm::computePolyhedronFaceVsSpherePenetrationDepth)", mProfiler);

    // Get the face
    const HalfEdgeStructure::Face& face = polyhedron->getFace(faceIndex);

    // Get the face normal
    const Vector3 faceNormal = polyhedron->getFaceNormal(faceIndex);

    Vector3 sphereCenterToFacePoint = polyhedron->getVertexPosition(face.faceVertices[0]) - sphereCenter;
    decimal penetrationDepth = sphereCenterToFacePoint.dot(faceNormal) + sphere->getRadius();

    return penetrationDepth;
}

// Test collision between a capsule and a convex mesh
bool SATAlgorithm::testCollisionCapsuleVsConvexPolyhedron(NarrowPhaseInfo* narrowPhaseInfo, bool reportContacts) const {

    RP3D_PROFILE("SATAlgorithm::testCollisionCapsuleVsConvexPolyhedron()", mProfiler);

    bool isCapsuleShape1 = narrowPhaseInfo->collisionShape1->getType() == CollisionShapeType::CAPSULE;

    assert(narrowPhaseInfo->collisionShape1->getType() == CollisionShapeType::CONVEX_POLYHEDRON ||
           narrowPhaseInfo->collisionShape2->getType() == CollisionShapeType::CONVEX_POLYHEDRON);
    assert(narrowPhaseInfo->collisionShape1->getType() == CollisionShapeType::CAPSULE ||
           narrowPhaseInfo->collisionShape2->getType() == CollisionShapeType::CAPSULE);

    // Get the collision shapes
    const CapsuleShape* capsuleShape = static_cast<const CapsuleShape*>(isCapsuleShape1 ? narrowPhaseInfo->collisionShape1 : narrowPhaseInfo->collisionShape2);
    const ConvexPolyhedronShape* polyhedron = static_cast<const ConvexPolyhedronShape*>(isCapsuleShape1 ? narrowPhaseInfo->collisionShape2 : narrowPhaseInfo->collisionShape1);

    const Transform capsuleToWorld = isCapsuleShape1 ? narrowPhaseInfo->shape1ToWorldTransform : narrowPhaseInfo->shape2ToWorldTransform;
    const Transform polyhedronToWorld = isCapsuleShape1 ? narrowPhaseInfo->shape2ToWorldTransform : narrowPhaseInfo->shape1ToWorldTransform;

    const Transform polyhedronToCapsuleTransform = capsuleToWorld.getInverse() * polyhedronToWorld;

    // Compute the end-points of the inner segment of the capsule
    const Vector3 capsuleSegA(0, -capsuleShape->getHeight() * decimal(0.5), 0);
    const Vector3 capsuleSegB(0, capsuleShape->getHeight() * decimal(0.5), 0);
    const Vector3 capsuleSegmentAxis = capsuleSegB - capsuleSegA;

    // Minimum penetration depth
    decimal minPenetrationDepth = DECIMAL_LARGEST;
    uint minFaceIndex = 0;
    bool isMinPenetrationFaceNormal = false;
    Vector3 separatingAxisCapsuleSpace;
    Vector3 separatingPolyhedronEdgeVertex1;
    Vector3 separatingPolyhedronEdgeVertex2;

    // For each face of the convex mesh
    for (uint f = 0; f < polyhedron->getNbFaces(); f++) {

        Vector3 outFaceNormalCapsuleSpace;

        // Compute the penetration depth
        const decimal penetrationDepth = computePolyhedronFaceVsCapsulePenetrationDepth(f, polyhedron, capsuleShape,
                                                polyhedronToCapsuleTransform,
                                                outFaceNormalCapsuleSpace);

        // If the penetration depth is negative, we have found a separating axis
        if (penetrationDepth <= decimal(0.0)) {

            return false;
        }

        // Check if we have found a new minimum penetration axis
        if (penetrationDepth < minPenetrationDepth) {
            minPenetrationDepth = penetrationDepth;
            minFaceIndex = f;
            isMinPenetrationFaceNormal = true;
            separatingAxisCapsuleSpace = outFaceNormalCapsuleSpace;
        }
    }

    // For each direction that is the cross product of the capsule inner segment and an edge of the polyhedron
    for (uint e = 0; e < polyhedron->getNbHalfEdges(); e += 2) {

        // Get an edge from the polyhedron (convert it into the capsule local-space)
        const HalfEdgeStructure::Edge& edge = polyhedron->getHalfEdge(e);
        const Vector3 edgeVertex1 = polyhedron->getVertexPosition(edge.vertexIndex);
        const Vector3 edgeVertex2 = polyhedron->getVertexPosition(polyhedron->getHalfEdge(edge.nextEdgeIndex).vertexIndex);
        const Vector3 edgeDirectionCapsuleSpace = polyhedronToCapsuleTransform.getOrientation() * (edgeVertex2 - edgeVertex1);

        const HalfEdgeStructure::Edge& twinEdge = polyhedron->getHalfEdge(edge.twinEdgeIndex);
        const Vector3 adjacentFace1Normal = polyhedronToCapsuleTransform.getOrientation() * polyhedron->getFaceNormal(edge.faceIndex);
        const Vector3 adjacentFace2Normal = polyhedronToCapsuleTransform.getOrientation() * polyhedron->getFaceNormal(twinEdge.faceIndex);

        // Check using the Gauss Map if this edge cross product can be as separating axis
        if (isMinkowskiFaceCapsuleVsEdge(capsuleSegmentAxis, adjacentFace1Normal, adjacentFace2Normal)) {

            Vector3 outAxis;

            // Compute the penetration depth
            const decimal penetrationDepth = computeEdgeVsCapsuleInnerSegmentPenetrationDepth(polyhedron, capsuleShape,
                                                  capsuleSegmentAxis, edgeVertex1,
                                                  edgeDirectionCapsuleSpace,
                                                  polyhedronToCapsuleTransform,
                                                  outAxis);

            // If the penetration depth is negative, we have found a separating axis
            if (penetrationDepth <= decimal(0.0)) {

                return false;
            }

            // Check if we have found a new minimum penetration axis
            if (penetrationDepth < minPenetrationDepth) {
                minPenetrationDepth = penetrationDepth;
                isMinPenetrationFaceNormal = false;
                separatingAxisCapsuleSpace = outAxis;
                separatingPolyhedronEdgeVertex1 = edgeVertex1;
                separatingPolyhedronEdgeVertex2 = edgeVertex2;
            }
        }
    }

    // Convert the inner capsule segment points into the polyhedron local-space
    const Transform capsuleToPolyhedronTransform = polyhedronToCapsuleTransform.getInverse();
    const Vector3 capsuleSegAPolyhedronSpace = capsuleToPolyhedronTransform * capsuleSegA;
    const Vector3 capsuleSegBPolyhedronSpace = capsuleToPolyhedronTransform * capsuleSegB;

    Vector3 normalWorld = capsuleToWorld.getOrientation() * separatingAxisCapsuleSpace;
    if (isCapsuleShape1) {
        normalWorld = -normalWorld;
    }
    const decimal capsuleRadius = capsuleShape->getRadius();

    // If the separating axis is a face normal
    // We need to clip the inner capsule segment with the adjacent faces of the separating face
    if (isMinPenetrationFaceNormal) {

        if (reportContacts) {

            return computeCapsulePolyhedronFaceContactPoints(minFaceIndex, capsuleRadius, polyhedron, minPenetrationDepth,
                                                      polyhedronToCapsuleTransform, normalWorld, separatingAxisCapsuleSpace,
                                                      capsuleSegAPolyhedronSpace, capsuleSegBPolyhedronSpace,
                                                      narrowPhaseInfo, isCapsuleShape1);
        }
    }
    else {   // The separating axis is the cross product of a polyhedron edge and the inner capsule segment

        if (reportContacts) {

            // Compute the closest points between the inner capsule segment and the
            // edge of the polyhedron in polyhedron local-space
            Vector3 closestPointPolyhedronEdge, closestPointCapsuleInnerSegment;
            computeClosestPointBetweenTwoSegments(capsuleSegAPolyhedronSpace, capsuleSegBPolyhedronSpace,
                                                  separatingPolyhedronEdgeVertex1, separatingPolyhedronEdgeVertex2,
                                                  closestPointCapsuleInnerSegment, closestPointPolyhedronEdge);

            // Project closest capsule inner segment point into the capsule bounds
            Vector3 contactPointCapsule = (polyhedronToCapsuleTransform * closestPointCapsuleInnerSegment) - separatingAxisCapsuleSpace * capsuleRadius;

            // Compute smooth triangle mesh contact if one of the two collision shapes is a triangle
            TriangleShape::computeSmoothTriangleMeshContact(narrowPhaseInfo->collisionShape1, narrowPhaseInfo->collisionShape2,
                                                        isCapsuleShape1 ? contactPointCapsule : closestPointPolyhedronEdge,
                                                        isCapsuleShape1 ? closestPointPolyhedronEdge : contactPointCapsule,
                                                        narrowPhaseInfo->shape1ToWorldTransform, narrowPhaseInfo->shape2ToWorldTransform,
                                                        minPenetrationDepth, normalWorld);

            // Create the contact point
            narrowPhaseInfo->addContactPoint(normalWorld, minPenetrationDepth,
                                                isCapsuleShape1 ? contactPointCapsule : closestPointPolyhedronEdge,
                                                isCapsuleShape1 ? closestPointPolyhedronEdge : contactPointCapsule);
        }
    }

    return true;
}

// Compute the penetration depth when the separating axis is the cross product of polyhedron edge and capsule inner segment
decimal SATAlgorithm::computeEdgeVsCapsuleInnerSegmentPenetrationDepth(const ConvexPolyhedronShape* polyhedron, const CapsuleShape* capsule,
                                                                       const Vector3& capsuleSegmentAxis, const Vector3& edgeVertex1,
                                                                       const Vector3& edgeDirectionCapsuleSpace,
                                                                       const Transform& polyhedronToCapsuleTransform, Vector3& outAxis) const {

    RP3D_PROFILE("SATAlgorithm::computeEdgeVsCapsuleInnerSegmentPenetrationDepth)", mProfiler);

    decimal penetrationDepth = DECIMAL_LARGEST;

    // Compute the axis to test (cross product between capsule inner segment and polyhedron edge)
    outAxis = capsuleSegmentAxis.cross(edgeDirectionCapsuleSpace);

    // Skip separating axis test if polyhedron edge is parallel to the capsule inner segment
    if (outAxis.lengthSquare() >= decimal(0.00001)) {

        const Vector3 polyhedronCentroid = polyhedronToCapsuleTransform * polyhedron->getCentroid();
        const Vector3 pointOnPolyhedronEdge = polyhedronToCapsuleTransform * edgeVertex1;

        // Swap axis direction if necessary such that it points out of the polyhedron
        if (outAxis.dot(pointOnPolyhedronEdge - polyhedronCentroid) < 0) {
            outAxis = -outAxis;
        }

        outAxis.normalize();

        // Compute the penetration depth
        const Vector3 capsuleSupportPoint = capsule->getLocalSupportPointWithMargin(-outAxis);
        const Vector3 capsuleSupportPointToEdgePoint = pointOnPolyhedronEdge - capsuleSupportPoint;
        penetrationDepth = capsuleSupportPointToEdgePoint.dot(outAxis);
    }

    return penetrationDepth;
}

// Compute the penetration depth between the face of a polyhedron and a capsule along the polyhedron face normal direction
decimal SATAlgorithm::computePolyhedronFaceVsCapsulePenetrationDepth(uint polyhedronFaceIndex, const ConvexPolyhedronShape* polyhedron,
                                                                     const CapsuleShape* capsule, const Transform& polyhedronToCapsuleTransform,
                                                                     Vector3& outFaceNormalCapsuleSpace) const {

    RP3D_PROFILE("SATAlgorithm::computePolyhedronFaceVsCapsulePenetrationDepth", mProfiler);

    // Get the face
    const HalfEdgeStructure::Face& face = polyhedron->getFace(polyhedronFaceIndex);

    // Get the face normal
    const Vector3 faceNormal = polyhedron->getFaceNormal(polyhedronFaceIndex);

    // Compute the penetration depth (using the capsule support in the direction opposite to the face normal)
    outFaceNormalCapsuleSpace = polyhedronToCapsuleTransform.getOrientation() * faceNormal;
    const Vector3 capsuleSupportPoint = capsule->getLocalSupportPointWithMargin(-outFaceNormalCapsuleSpace);
    const Vector3 pointOnPolyhedronFace = polyhedronToCapsuleTransform * polyhedron->getVertexPosition(face.faceVertices[0]);
    const Vector3 capsuleSupportPointToFacePoint =  pointOnPolyhedronFace - capsuleSupportPoint;
    const decimal penetrationDepth = capsuleSupportPointToFacePoint.dot(outFaceNormalCapsuleSpace);

    return penetrationDepth;
}

// Compute the two contact points between a polyhedron and a capsule when the separating
// axis is a face normal of the polyhedron
bool SATAlgorithm::computeCapsulePolyhedronFaceContactPoints(uint referenceFaceIndex, decimal capsuleRadius, const ConvexPolyhedronShape* polyhedron,
                                                             decimal penetrationDepth, const Transform& polyhedronToCapsuleTransform,
                                                             Vector3& normalWorld, const Vector3& separatingAxisCapsuleSpace,
                                                             const Vector3& capsuleSegAPolyhedronSpace, const Vector3& capsuleSegBPolyhedronSpace,
                                                             NarrowPhaseInfo* narrowPhaseInfo, bool isCapsuleShape1) const {

    RP3D_PROFILE("SATAlgorithm::computeCapsulePolyhedronFaceContactPoints", mProfiler);

    const HalfEdgeStructure::Face& face = polyhedron->getFace(referenceFaceIndex);

    // Get the face normal
    Vector3 faceNormal = polyhedron->getFaceNormal(referenceFaceIndex);

    uint firstEdgeIndex = face.edgeIndex;
    uint edgeIndex = firstEdgeIndex;

    List<Vector3> planesPoints(mMemoryAllocator, 2);
    List<Vector3> planesNormals(mMemoryAllocator, 2);

    // For each adjacent edge of the separating face of the polyhedron
    do {

        const HalfEdgeStructure::Edge& edge = polyhedron->getHalfEdge(edgeIndex);
        const HalfEdgeStructure::Edge& twinEdge = polyhedron->getHalfEdge(edge.twinEdgeIndex);

        // Compute the edge vertices and edge direction
        Vector3 edgeV1 = polyhedron->getVertexPosition(edge.vertexIndex);
        Vector3 edgeV2 = polyhedron->getVertexPosition(twinEdge.vertexIndex);
        Vector3 edgeDirection = edgeV2 - edgeV1;

        // Compute the normal of the clipping plane for this edge
        // The clipping plane is perpendicular to the edge direction and the reference face normal
        Vector3 clipPlaneNormal = faceNormal.cross(edgeDirection);

        // Construct a clipping plane for each adjacent edge of the separating face of the polyhedron
        planesPoints.add(polyhedron->getVertexPosition(edge.vertexIndex));
        planesNormals.add(clipPlaneNormal);

        edgeIndex = edge.nextEdgeIndex;

    } while(edgeIndex != firstEdgeIndex);

    // First we clip the inner segment of the capsule with the four planes of the adjacent faces
    List<Vector3> clipSegment = clipSegmentWithPlanes(capsuleSegAPolyhedronSpace, capsuleSegBPolyhedronSpace, planesPoints, planesNormals, mMemoryAllocator);

	// Project the two clipped points into the polyhedron face
	const Vector3 delta = faceNormal * (penetrationDepth - capsuleRadius);

    bool contactFound = false;

	// For each of the two clipped points
    for (uint i = 0; i<clipSegment.size(); i++) {

		// Compute the penetration depth of the two clipped points (to filter out the points that does not correspond to the penetration depth)
		const decimal clipPointPenDepth = (planesPoints[0] - clipSegment[i]).dot(faceNormal);

		// If the clipped point is one that produce this penetration depth, we keep it
		if (clipPointPenDepth > penetrationDepth - capsuleRadius - decimal(0.001)) {

            contactFound = true;

            Vector3 contactPointPolyhedron = clipSegment[i] + delta;

			// Project the clipped point into the capsule bounds
            Vector3 contactPointCapsule = (polyhedronToCapsuleTransform * clipSegment[i]) - separatingAxisCapsuleSpace * capsuleRadius;



            // Compute smooth triangle mesh contact if one of the two collision shapes is a triangle
            TriangleShape::computeSmoothTriangleMeshContact(narrowPhaseInfo->collisionShape1, narrowPhaseInfo->collisionShape2,
                                                        isCapsuleShape1 ? contactPointCapsule : contactPointPolyhedron,
                                                        isCapsuleShape1 ? contactPointPolyhedron : contactPointCapsule,
                                                        narrowPhaseInfo->shape1ToWorldTransform, narrowPhaseInfo->shape2ToWorldTransform,
                                                        penetrationDepth, normalWorld);


			// Create the contact point
            narrowPhaseInfo->addContactPoint(normalWorld, penetrationDepth,
                                             isCapsuleShape1 ? contactPointCapsule : contactPointPolyhedron,
                                             isCapsuleShape1 ? contactPointPolyhedron : contactPointCapsule);
		}
	}

    return contactFound;
}

// This method returns true if an edge of a polyhedron and a capsule forms a
// face of the Minkowski Difference. This test is used to know if two edges
// (one edge of the polyhedron vs the inner segment of the capsule in this case)
// have to be test as a possible separating axis
bool SATAlgorithm::isMinkowskiFaceCapsuleVsEdge(const Vector3& capsuleSegment, const Vector3& edgeAdjacentFace1Normal,
                                                const Vector3& edgeAdjacentFace2Normal) const {

    // Return true if the arc on the Gauss Map corresponding to the polyhedron edge
    // intersect the unit circle plane corresponding to capsule Gauss Map
    return capsuleSegment.dot(edgeAdjacentFace1Normal) * capsuleSegment.dot(edgeAdjacentFace2Normal) < decimal(0.0);
}

// Test collision between two convex polyhedrons
bool SATAlgorithm::testCollisionConvexPolyhedronVsConvexPolyhedron(NarrowPhaseInfo* narrowPhaseInfo, bool reportContacts) const {

    RP3D_PROFILE("SATAlgorithm::testCollisionConvexPolyhedronVsConvexPolyhedron()", mProfiler);

    assert(narrowPhaseInfo->collisionShape1->getType() == CollisionShapeType::CONVEX_POLYHEDRON);
    assert(narrowPhaseInfo->collisionShape2->getType() == CollisionShapeType::CONVEX_POLYHEDRON);

    const ConvexPolyhedronShape* polyhedron1 = static_cast<const ConvexPolyhedronShape*>(narrowPhaseInfo->collisionShape1);
    const ConvexPolyhedronShape* polyhedron2 = static_cast<const ConvexPolyhedronShape*>(narrowPhaseInfo->collisionShape2);

    const Transform polyhedron1ToPolyhedron2 = narrowPhaseInfo->shape2ToWorldTransform.getInverse() * narrowPhaseInfo->shape1ToWorldTransform;
    const Transform polyhedron2ToPolyhedron1 = polyhedron1ToPolyhedron2.getInverse();

    decimal minPenetrationDepth = DECIMAL_LARGEST;
    uint minFaceIndex = 0;
    bool isMinPenetrationFaceNormal = false;
    bool isMinPenetrationFaceNormalPolyhedron1 = false;
    uint minSeparatingEdge1Index = 0;
    uint minSeparatingEdge2Index = 0;
    Vector3 separatingEdge1A, separatingEdge1B;
    Vector3 separatingEdge2A, separatingEdge2B;
    const bool isShape1Triangle = polyhedron1->getName() == CollisionShapeName::TRIANGLE;

    LastFrameCollisionInfo* lastFrameCollisionInfo = narrowPhaseInfo->getLastFrameCollisionInfo();

    // If the last frame collision info is valid and was also using SAT algorithm
    if (lastFrameCollisionInfo->isValid && lastFrameCollisionInfo->wasUsingSAT) {

        // We perform temporal coherence, we check if there is still an overlapping along the previous minimum separating
        // axis. If it is the case, we directly report the collision without executing the whole SAT algorithm again. If
        // the shapes are still separated along this axis, we directly exit with no collision.

        // If the previous separating axis (or axis with minimum penetration depth)
        // was a face normal of polyhedron 1
        if (lastFrameCollisionInfo->satIsAxisFacePolyhedron1) {

            decimal penetrationDepth = testSingleFaceDirectionPolyhedronVsPolyhedron(polyhedron1, polyhedron2, polyhedron1ToPolyhedron2,
                                                 lastFrameCollisionInfo->satMinAxisFaceIndex);

            // If the previous axis was a separating axis and is still a separating axis in this frame
            if (!lastFrameCollisionInfo->wasColliding && penetrationDepth <= decimal(0.0)) {

                // Return no collision without running the whole SAT algorithm
                return false;
            }

            // The two shapes were overlapping in the previous frame and still seem to overlap in this one
            if (lastFrameCollisionInfo->wasColliding && penetrationDepth > decimal(0.0)) {

                minPenetrationDepth = penetrationDepth;
                minFaceIndex = lastFrameCollisionInfo->satMinAxisFaceIndex;
                isMinPenetrationFaceNormal = true;
                isMinPenetrationFaceNormalPolyhedron1 = true;

                // Compute the contact points between two faces of two convex polyhedra.
                if(computePolyhedronVsPolyhedronFaceContactPoints(isMinPenetrationFaceNormalPolyhedron1, polyhedron1, polyhedron2,
                                          polyhedron1ToPolyhedron2, polyhedron2ToPolyhedron1, minFaceIndex,
                                          narrowPhaseInfo, minPenetrationDepth)) {

                    lastFrameCollisionInfo->satIsAxisFacePolyhedron1 = isMinPenetrationFaceNormalPolyhedron1;
                    lastFrameCollisionInfo->satIsAxisFacePolyhedron2 = !isMinPenetrationFaceNormalPolyhedron1;
                    lastFrameCollisionInfo->satMinAxisFaceIndex = minFaceIndex;

                    // The shapes are still overlapping in the previous axis (the contact manifold is not empty).
                    // Therefore, we can return without running the whole SAT algorithm
                    return true;
                }

                // The contact manifold is empty. Therefore, we have to run the whole SAT algorithm again
            }
        }
        else if (lastFrameCollisionInfo->satIsAxisFacePolyhedron2) { // If the previous separating axis (or axis with minimum penetration depth)
                                   // was a face normal of polyhedron 2

            decimal penetrationDepth = testSingleFaceDirectionPolyhedronVsPolyhedron(polyhedron2, polyhedron1, polyhedron2ToPolyhedron1,
                                                 lastFrameCollisionInfo->satMinAxisFaceIndex);

            // If the previous axis was a separating axis and is still a separating axis in this frame
            if (!lastFrameCollisionInfo->wasColliding && penetrationDepth <= decimal(0.0)) {

                // Return no collision without running the whole SAT algorithm
                return false;
            }

            // The two shapes were overlapping in the previous frame and still seem to overlap in this one
            if (lastFrameCollisionInfo->wasColliding && penetrationDepth > decimal(0.0)) {

                minPenetrationDepth = penetrationDepth;
                minFaceIndex = lastFrameCollisionInfo->satMinAxisFaceIndex;
                isMinPenetrationFaceNormal = true;
                isMinPenetrationFaceNormalPolyhedron1 = false;

                // Compute the contact points between two faces of two convex polyhedra.
                if(computePolyhedronVsPolyhedronFaceContactPoints(isMinPenetrationFaceNormalPolyhedron1, polyhedron1, polyhedron2,
                                          polyhedron1ToPolyhedron2, polyhedron2ToPolyhedron1, minFaceIndex,
                                          narrowPhaseInfo, minPenetrationDepth)) {

                    lastFrameCollisionInfo->satIsAxisFacePolyhedron1 = isMinPenetrationFaceNormalPolyhedron1;
                    lastFrameCollisionInfo->satIsAxisFacePolyhedron2 = !isMinPenetrationFaceNormalPolyhedron1;
                    lastFrameCollisionInfo->satMinAxisFaceIndex = minFaceIndex;

                    // The shapes are still overlapping in the previous axis (the contact manifold is not empty).
                    // Therefore, we can return without running the whole SAT algorithm
                    return true;
                }

                // The contact manifold is empty. Therefore, we have to run the whole SAT algorithm again
            }
        }
        else {   // If the previous separating axis (or axis with minimum penetration depth) was the cross product of two edges

            const HalfEdgeStructure::Edge& edge1 = polyhedron1->getHalfEdge(lastFrameCollisionInfo->satMinEdge1Index);
            const HalfEdgeStructure::Edge& edge2 = polyhedron2->getHalfEdge(lastFrameCollisionInfo->satMinEdge2Index);

            const Vector3 edge1A = polyhedron1ToPolyhedron2 * polyhedron1->getVertexPosition(edge1.vertexIndex);
            const Vector3 edge1B = polyhedron1ToPolyhedron2 * polyhedron1->getVertexPosition(polyhedron1->getHalfEdge(edge1.nextEdgeIndex).vertexIndex);
            const Vector3 edge1Direction = edge1B - edge1A;
            const Vector3 edge2A = polyhedron2->getVertexPosition(edge2.vertexIndex);
            const Vector3 edge2B = polyhedron2->getVertexPosition(polyhedron2->getHalfEdge(edge2.nextEdgeIndex).vertexIndex);
            const Vector3 edge2Direction = edge2B - edge2A;

            // If the two edges build a minkowski face (and the cross product is
            // therefore a candidate for separating axis
            if (testEdgesBuildMinkowskiFace(polyhedron1, edge1, polyhedron2, edge2, polyhedron1ToPolyhedron2)) {

                Vector3 separatingAxisPolyhedron2Space;

                // Compute the penetration depth along the previous axis
                const Vector3 polyhedron1Centroid = polyhedron1ToPolyhedron2 * polyhedron1->getCentroid();
                decimal penetrationDepth = computeDistanceBetweenEdges(edge1A, edge2A, polyhedron1Centroid, polyhedron2->getCentroid(),
                           edge1Direction, edge2Direction, isShape1Triangle, separatingAxisPolyhedron2Space);

                // If the shapes were not overlapping in the previous frame and are still not
                // overlapping in the current one
                if (!lastFrameCollisionInfo->wasColliding && penetrationDepth <= decimal(0.0)) {

                    // We have found a separating axis without running the whole SAT algorithm
                    return false;
                }

                // If the shapes were overlapping on the previous axis and still seem to overlap in this frame
                if (lastFrameCollisionInfo->wasColliding && penetrationDepth > decimal(0.0) &&
                    penetrationDepth < DECIMAL_LARGEST) {

                    // Compute the closest points between the two edges (in the local-space of poylhedron 2)
                    Vector3 closestPointPolyhedron1Edge, closestPointPolyhedron2Edge;
                    computeClosestPointBetweenTwoSegments(edge1A, edge1B, edge2A, edge2B,
                          closestPointPolyhedron1Edge, closestPointPolyhedron2Edge);

                    // Here we try to project the closest point on edge1 onto the segment of edge 2 to see if
                    // the projected point falls onto the segment. We also try to project the closest point
                    // on edge 2 to see if it falls onto the segment of edge 1. If one of the point does not
                    // fall onto the opposite segment, it means the edges are not colliding (the contact manifold
                    // is empty). Therefore, we need to run the whole SAT algorithm again.
                    const Vector3 vec1 = closestPointPolyhedron1Edge - edge2A;
                    const Vector3 vec2 = closestPointPolyhedron2Edge - edge1A;
                    const decimal edge1LengthSquare = edge1Direction.lengthSquare();
                    const decimal edge2LengthSquare = edge2Direction.lengthSquare();
                    decimal t1 = vec1.dot(edge2Direction) / edge2LengthSquare;
                    decimal t2 = vec2.dot(edge1Direction) / edge1LengthSquare;
                    if (t1 >= decimal(0.0) && t1 <= decimal(1) && t2 >= decimal(0.0) && t2 <= decimal(1.0)) {

                        // Compute the contact point on polyhedron 1 edge in the local-space of polyhedron 1
                        Vector3 closestPointPolyhedron1EdgeLocalSpace = polyhedron2ToPolyhedron1 * closestPointPolyhedron1Edge;

                        // Compute the world normal
                        // We use the direction from the centroid to the edge of the shape that is not a triangle
                        // to avoid possible degeneracies when axis direction is orthogonal to triangle normal
                        Vector3 normal;
                        if (isShape1Triangle) {
                           normal = polyhedron2->getCentroid() - closestPointPolyhedron2Edge;
                        }
                        else {
                           normal = polyhedron1ToPolyhedron2.getOrientation() * ((polyhedron2ToPolyhedron1 * closestPointPolyhedron1Edge) - polyhedron1->getCentroid());
                        }

                        Vector3 normalWorld = narrowPhaseInfo->shape2ToWorldTransform.getOrientation() * normal.getUnit();

                        // Compute smooth triangle mesh contact if one of the two collision shapes is a triangle
                        TriangleShape::computeSmoothTriangleMeshContact(narrowPhaseInfo->collisionShape1, narrowPhaseInfo->collisionShape2,
                        closestPointPolyhedron1EdgeLocalSpace, closestPointPolyhedron2Edge,
                        narrowPhaseInfo->shape1ToWorldTransform, narrowPhaseInfo->shape2ToWorldTransform,
                        penetrationDepth, normalWorld);

                        // Create the contact point
                        narrowPhaseInfo->addContactPoint(normalWorld, penetrationDepth,
                         closestPointPolyhedron1EdgeLocalSpace, closestPointPolyhedron2Edge);

                        // The shapes are overlapping on the previous axis (the contact manifold is not empty). Therefore
                        // we return without running the whole SAT algorithm
                        return true;
                    }

                    // The contact manifold is empty. Therefore, we have to run the whole SAT algorithm again
                }
            }
        }
    }

    minPenetrationDepth = DECIMAL_LARGEST;
    isMinPenetrationFaceNormal = false;

    // Test all the face normals of the polyhedron 1 for separating axis
    uint faceIndex1;
    decimal penetrationDepth1 = testFacesDirectionPolyhedronVsPolyhedron(polyhedron1, polyhedron2, polyhedron1ToPolyhedron2, faceIndex1);
    if (penetrationDepth1 <= decimal(0.0)) {

        lastFrameCollisionInfo->satIsAxisFacePolyhedron1 = true;
        lastFrameCollisionInfo->satIsAxisFacePolyhedron2 = false;
        lastFrameCollisionInfo->satMinAxisFaceIndex = faceIndex1;

        // We have found a separating axis
        return false;
    }

    // Test all the face normals of the polyhedron 2 for separating axis
    uint faceIndex2;
    decimal penetrationDepth2 = testFacesDirectionPolyhedronVsPolyhedron(polyhedron2, polyhedron1, polyhedron2ToPolyhedron1, faceIndex2);
    if (penetrationDepth2 <= decimal(0.0)) {

        lastFrameCollisionInfo->satIsAxisFacePolyhedron1 = false;
        lastFrameCollisionInfo->satIsAxisFacePolyhedron2 = true;
        lastFrameCollisionInfo->satMinAxisFaceIndex = faceIndex2;

        // We have found a separating axis
        return false;
    }

    // Here we know that we have found penetration along both axis of a face of polyhedron1 and a face of
    // polyhedron2. If the two penetration depths are almost the same, we need to make sure we always prefer
    // one axis to the other for consistency between frames. This is to prevent the contact manifolds to switch
    // from one reference axis to the other for a face to face resting contact for instance. This is better for
    // stability. To do this, we use a relative and absolute bias to move penetrationDepth2 a little bit to the right.
    // Now if:
    //  penetrationDepth1 < penetrationDepth2: Nothing happens and we use axis of polygon 1
    //  penetrationDepth1 ~ penetrationDepth2: Until penetrationDepth1 becomes significantly less than penetrationDepth2 we still use axis of polygon 1
    //  penetrationDepth1 >> penetrationDepth2: penetrationDepth2 is now significantly less than penetrationDepth1 and we use polygon 2 axis
    if (penetrationDepth1 < penetrationDepth2 * SEPARATING_AXIS_RELATIVE_TOLERANCE + SEPARATING_AXIS_ABSOLUTE_TOLERANCE) {

        // We use penetration axis of polygon 1
        isMinPenetrationFaceNormal = true;
        minPenetrationDepth = std::min(penetrationDepth1, penetrationDepth2);
        minFaceIndex = faceIndex1;
        isMinPenetrationFaceNormalPolyhedron1 = true;
    }
    else {

        // We use penetration axis of polygon 2
        isMinPenetrationFaceNormal = true;
        minPenetrationDepth = std::min(penetrationDepth1, penetrationDepth2);
        minFaceIndex = faceIndex2;
        isMinPenetrationFaceNormalPolyhedron1 = false;
    }

    // Test the cross products of edges of polyhedron 1 with edges of polyhedron 2 for separating axis
    for (uint i=0; i < polyhedron1->getNbHalfEdges(); i += 2) {

        // Get an edge of polyhedron 1
        const HalfEdgeStructure::Edge& edge1 = polyhedron1->getHalfEdge(i);

        const Vector3 edge1A = polyhedron1ToPolyhedron2 * polyhedron1->getVertexPosition(edge1.vertexIndex);
        const Vector3 edge1B = polyhedron1ToPolyhedron2 * polyhedron1->getVertexPosition(polyhedron1->getHalfEdge(edge1.nextEdgeIndex).vertexIndex);
        const Vector3 edge1Direction = edge1B - edge1A;

        for (uint j=0; j < polyhedron2->getNbHalfEdges(); j += 2) {

            // Get an edge of polyhedron 2
            const HalfEdgeStructure::Edge& edge2 = polyhedron2->getHalfEdge(j);

            const Vector3 edge2A = polyhedron2->getVertexPosition(edge2.vertexIndex);
            const Vector3 edge2B = polyhedron2->getVertexPosition(polyhedron2->getHalfEdge(edge2.nextEdgeIndex).vertexIndex);
            const Vector3 edge2Direction = edge2B - edge2A;

            // If the two edges build a minkowski face (and the cross product is
            // therefore a candidate for separating axis
            if (testEdgesBuildMinkowskiFace(polyhedron1, edge1, polyhedron2, edge2, polyhedron1ToPolyhedron2)) {

                Vector3 separatingAxisPolyhedron2Space;

                // Compute the penetration depth
                const Vector3 polyhedron1Centroid = polyhedron1ToPolyhedron2 * polyhedron1->getCentroid();
                decimal penetrationDepth = computeDistanceBetweenEdges(edge1A, edge2A, polyhedron1Centroid, polyhedron2->getCentroid(),
                           edge1Direction, edge2Direction, isShape1Triangle, separatingAxisPolyhedron2Space);

                if (penetrationDepth <= decimal(0.0)) {

                    lastFrameCollisionInfo->satIsAxisFacePolyhedron1 = false;
                    lastFrameCollisionInfo->satIsAxisFacePolyhedron2 = false;
                    lastFrameCollisionInfo->satMinEdge1Index = i;
                    lastFrameCollisionInfo->satMinEdge2Index = j;

                    // We have found a separating axis
                    return false;
                }

                if (penetrationDepth < minPenetrationDepth) {

                    minPenetrationDepth = penetrationDepth;
                    isMinPenetrationFaceNormalPolyhedron1 = false;
                    isMinPenetrationFaceNormal = false;
                    minSeparatingEdge1Index = i;
                    minSeparatingEdge2Index = j;
                    separatingEdge1A = edge1A;
                    separatingEdge1B = edge1B;
                    separatingEdge2A = edge2A;
                    separatingEdge2B = edge2B;
                }
            }
        }
    }

    // Here we know the shapes are overlapping on a given minimum separating axis.
    // Now, we will clip the shapes along this axis to find the contact points

    assert(minPenetrationDepth > decimal(0.0));
    assert((isMinPenetrationFaceNormal && minFaceIndex >= 0) || !isMinPenetrationFaceNormal);

    // If the minimum separating axis is a face normal
    if (isMinPenetrationFaceNormal) {

        if (reportContacts) {

            // Compute the contact points between two faces of two convex polyhedra.
            bool contactsFound = computePolyhedronVsPolyhedronFaceContactPoints(isMinPenetrationFaceNormalPolyhedron1, polyhedron1,
                                                                                polyhedron2, polyhedron1ToPolyhedron2, polyhedron2ToPolyhedron1,
                                                                                minFaceIndex, narrowPhaseInfo, minPenetrationDepth);
			
			// There should be clipping points here. If it is not the case, it might be
			// because of a numerical issue
			if (!contactsFound) {
				
                lastFrameCollisionInfo->satIsAxisFacePolyhedron1 = isMinPenetrationFaceNormalPolyhedron1;
                lastFrameCollisionInfo->satIsAxisFacePolyhedron2 = !isMinPenetrationFaceNormalPolyhedron1;
                lastFrameCollisionInfo->satMinAxisFaceIndex = minFaceIndex;

				// Return no collision
				return false;
			}
        }

        lastFrameCollisionInfo->satIsAxisFacePolyhedron1 = isMinPenetrationFaceNormalPolyhedron1;
        lastFrameCollisionInfo->satIsAxisFacePolyhedron2 = !isMinPenetrationFaceNormalPolyhedron1;
        lastFrameCollisionInfo->satMinAxisFaceIndex = minFaceIndex;
    }
    else {    // If we have an edge vs edge contact

        if (reportContacts) {

            // Compute the closest points between the two edges (in the local-space of poylhedron 2)
            Vector3 closestPointPolyhedron1Edge, closestPointPolyhedron2Edge;
            computeClosestPointBetweenTwoSegments(separatingEdge1A, separatingEdge1B, separatingEdge2A, separatingEdge2B,
                                                  closestPointPolyhedron1Edge, closestPointPolyhedron2Edge);

            // Compute the contact point on polyhedron 1 edge in the local-space of polyhedron 1
            Vector3 closestPointPolyhedron1EdgeLocalSpace = polyhedron2ToPolyhedron1 * closestPointPolyhedron1Edge;

            // Compute the world normal
            // We use the direction from the centroid to the edge of the shape that is not a triangle
            // to avoid possible degeneracies when axis direction is orthogonal to triangle normal
            Vector3 normal;
            if (isShape1Triangle) {
               normal = polyhedron2->getCentroid() - closestPointPolyhedron2Edge;
            }
            else {
               normal = polyhedron1ToPolyhedron2.getOrientation() * ((polyhedron2ToPolyhedron1 * closestPointPolyhedron1Edge) - polyhedron1->getCentroid());
            }
            const Vector3 unitNormal = normal.getUnit();
            assert(unitNormal.length() > decimal(0.7));
            Vector3 normalWorld = narrowPhaseInfo->shape2ToWorldTransform.getOrientation() * unitNormal;
            assert(normalWorld.length() > decimal(0.7));

            // Compute smooth triangle mesh contact if one of the two collision shapes is a triangle
            TriangleShape::computeSmoothTriangleMeshContact(narrowPhaseInfo->collisionShape1, narrowPhaseInfo->collisionShape2,
                                                            closestPointPolyhedron1EdgeLocalSpace, closestPointPolyhedron2Edge,
                                                            narrowPhaseInfo->shape1ToWorldTransform, narrowPhaseInfo->shape2ToWorldTransform,
                                                            minPenetrationDepth, normalWorld);

            assert(normalWorld.length() > decimal(0.7));

            // Create the contact point
            narrowPhaseInfo->addContactPoint(normalWorld, minPenetrationDepth,
                                             closestPointPolyhedron1EdgeLocalSpace, closestPointPolyhedron2Edge);
        }

        lastFrameCollisionInfo->satIsAxisFacePolyhedron1 = false;
        lastFrameCollisionInfo->satIsAxisFacePolyhedron2 = false;
        lastFrameCollisionInfo->satMinEdge1Index = minSeparatingEdge1Index;
        lastFrameCollisionInfo->satMinEdge2Index = minSeparatingEdge2Index;
    }

    return true;
}

// Compute the contact points between two faces of two convex polyhedra.
/// The method returns true if contact points have been found
bool SATAlgorithm::computePolyhedronVsPolyhedronFaceContactPoints(bool isMinPenetrationFaceNormalPolyhedron1,
                                                                  const ConvexPolyhedronShape* polyhedron1, const ConvexPolyhedronShape* polyhedron2,
                                                                  const Transform& polyhedron1ToPolyhedron2, const Transform& polyhedron2ToPolyhedron1,
                                                                  uint minFaceIndex, NarrowPhaseInfo* narrowPhaseInfo, decimal minPenetrationDepth) const {

    RP3D_PROFILE("SATAlgorithm::computePolyhedronVsPolyhedronFaceContactPoints", mProfiler);

    const ConvexPolyhedronShape* referencePolyhedron = isMinPenetrationFaceNormalPolyhedron1 ? polyhedron1 : polyhedron2;
    const ConvexPolyhedronShape* incidentPolyhedron = isMinPenetrationFaceNormalPolyhedron1 ? polyhedron2 : polyhedron1;
    const Transform& referenceToIncidentTransform = isMinPenetrationFaceNormalPolyhedron1 ? polyhedron1ToPolyhedron2 : polyhedron2ToPolyhedron1;
    const Transform& incidentToReferenceTransform = isMinPenetrationFaceNormalPolyhedron1 ? polyhedron2ToPolyhedron1 : polyhedron1ToPolyhedron2;

    assert(minPenetrationDepth > decimal(0.0));

    const Vector3 axisReferenceSpace = referencePolyhedron->getFaceNormal(minFaceIndex);
    const Vector3 axisIncidentSpace = referenceToIncidentTransform.getOrientation() * axisReferenceSpace;

    // Compute the world normal
    Vector3 normalWorld = isMinPenetrationFaceNormalPolyhedron1 ? narrowPhaseInfo->shape1ToWorldTransform.getOrientation() * axisReferenceSpace :
                                    -(narrowPhaseInfo->shape2ToWorldTransform.getOrientation() * axisReferenceSpace);

    // Get the reference face
    const HalfEdgeStructure::Face& referenceFace = referencePolyhedron->getFace(minFaceIndex);

    // Find the incident face on the other polyhedron (most anti-parallel face)
    uint incidentFaceIndex = incidentPolyhedron->findMostAntiParallelFace(axisIncidentSpace);

    // Get the incident face
    const HalfEdgeStructure::Face& incidentFace = incidentPolyhedron->getFace(incidentFaceIndex);

    uint nbIncidentFaceVertices = static_cast<uint>(incidentFace.faceVertices.size());
    List<Vector3> polygonVertices(mMemoryAllocator, nbIncidentFaceVertices);   // Vertices to clip of the incident face
    List<Vector3> planesNormals(mMemoryAllocator, nbIncidentFaceVertices);     // Normals of the clipping planes
    List<Vector3> planesPoints(mMemoryAllocator, nbIncidentFaceVertices);      // Points on the clipping planes

    // Get all the vertices of the incident face (in the reference local-space)
    for (uint i=0; i < incidentFace.faceVertices.size(); i++) {
        const Vector3 faceVertexIncidentSpace = incidentPolyhedron->getVertexPosition(incidentFace.faceVertices[i]);
        polygonVertices.add(incidentToReferenceTransform * faceVertexIncidentSpace);
    }

    // Get the reference face clipping planes
    uint currentEdgeIndex = referenceFace.edgeIndex;
    uint firstEdgeIndex = currentEdgeIndex;
    do {

        // Get the adjacent edge
        const HalfEdgeStructure::Edge& edge = referencePolyhedron->getHalfEdge(currentEdgeIndex);

        // Get the twin edge
        const HalfEdgeStructure::Edge& twinEdge = referencePolyhedron->getHalfEdge(edge.twinEdgeIndex);

        // Compute the edge vertices and edge direction
        Vector3 edgeV1 = referencePolyhedron->getVertexPosition(edge.vertexIndex);
        Vector3 edgeV2 = referencePolyhedron->getVertexPosition(twinEdge.vertexIndex);
        Vector3 edgeDirection = edgeV2 - edgeV1;

        // Compute the normal of the clipping plane for this edge
        // The clipping plane is perpendicular to the edge direction and the reference face normal
        Vector3 clipPlaneNormal = axisReferenceSpace.cross(edgeDirection);

        planesNormals.add(clipPlaneNormal);
        planesPoints.add(edgeV1);

        // Go to the next adjacent edge of the reference face
        currentEdgeIndex = edge.nextEdgeIndex;

    } while (currentEdgeIndex != firstEdgeIndex);

    assert(planesNormals.size() > 0);
    assert(planesNormals.size() == planesPoints.size());

    // Clip the reference faces with the adjacent planes of the reference face
    List<Vector3> clipPolygonVertices = clipPolygonWithPlanes(polygonVertices, planesPoints, planesNormals, mMemoryAllocator);

    // We only keep the clipped points that are below the reference face
    const Vector3 referenceFaceVertex = referencePolyhedron->getVertexPosition(referencePolyhedron->getHalfEdge(firstEdgeIndex).vertexIndex);
    bool contactPointsFound = false;
    for (uint i=0; i<clipPolygonVertices.size(); i++) {

        // Compute the penetration depth of this contact point (can be different from the minPenetration depth which is
        // the maximal penetration depth of any contact point for this separating axis
        decimal penetrationDepth = (referenceFaceVertex - clipPolygonVertices[i]).dot(axisReferenceSpace);

        // If the clip point is bellow the reference face
        if (penetrationDepth > decimal(0.0)) {

            contactPointsFound = true;

            Vector3 outWorldNormal = normalWorld;

            // Convert the clip incident polyhedron vertex into the incident polyhedron local-space
            Vector3 contactPointIncidentPolyhedron = referenceToIncidentTransform * clipPolygonVertices[i];

            // Project the contact point onto the reference face
            Vector3 contactPointReferencePolyhedron = projectPointOntoPlane(clipPolygonVertices[i], axisReferenceSpace, referenceFaceVertex);

            // Compute smooth triangle mesh contact if one of the two collision shapes is a triangle
            TriangleShape::computeSmoothTriangleMeshContact(narrowPhaseInfo->collisionShape1, narrowPhaseInfo->collisionShape2,
                                    isMinPenetrationFaceNormalPolyhedron1 ? contactPointReferencePolyhedron : contactPointIncidentPolyhedron,
                                    isMinPenetrationFaceNormalPolyhedron1 ? contactPointIncidentPolyhedron : contactPointReferencePolyhedron,
                                    narrowPhaseInfo->shape1ToWorldTransform, narrowPhaseInfo->shape2ToWorldTransform,
                                    penetrationDepth, outWorldNormal);

            // Create a new contact point
            narrowPhaseInfo->addContactPoint(outWorldNormal, penetrationDepth,
                             isMinPenetrationFaceNormalPolyhedron1 ? contactPointReferencePolyhedron : contactPointIncidentPolyhedron,
                             isMinPenetrationFaceNormalPolyhedron1 ? contactPointIncidentPolyhedron : contactPointReferencePolyhedron);
        }
    }

    return contactPointsFound;
}


// Compute and return the distance between the two edges in the direction of the candidate separating axis
decimal SATAlgorithm::computeDistanceBetweenEdges(const Vector3& edge1A, const Vector3& edge2A,
                                                  const Vector3& polyhedron1Centroid, const Vector3& polyhedron2Centroid,
                                                  const Vector3& edge1Direction, const Vector3& edge2Direction,
                                                  bool isShape1Triangle, Vector3& outSeparatingAxisPolyhedron2Space) const {

    RP3D_PROFILE("SATAlgorithm::computeDistanceBetweenEdges", mProfiler);

    // If the two edges are parallel
    if (areParallelVectors(edge1Direction, edge2Direction)) {

        // Return a large penetration depth to skip those edges
        return DECIMAL_LARGEST;
    }

    // Compute the candidate separating axis (cross product between two polyhedrons edges)
    Vector3 axis = edge1Direction.cross(edge2Direction).getUnit();

    // Make sure the axis direction is going from first to second polyhedron
    decimal dotProd;
    if (isShape1Triangle) {

        // The shape 1 is a triangle. It is safer to use a vector from
        // centroid to edge of the shape 2 because for a triangle, we
        // can have a vector that is orthogonal to the axis

        dotProd = axis.dot(edge2A - polyhedron2Centroid);
    }
    else {

        // The shape 2 might be a triangle. It is safer to use a vector from
        // centroid to edge of the shape 2 because for a triangle, we
        // can have a vector that is orthogonal to the axis

        dotProd = axis.dot(polyhedron1Centroid - edge1A);
    }
    if (dotProd > decimal(0.0)) {
        axis = -axis;
    }

    outSeparatingAxisPolyhedron2Space = axis;

    // Compute and return the distance between the edges
    return -axis.dot(edge2A - edge1A);
}


// Return the penetration depth between two polyhedra along a face normal axis of the first polyhedron
decimal SATAlgorithm::testSingleFaceDirectionPolyhedronVsPolyhedron(const ConvexPolyhedronShape* polyhedron1,
                                                                    const ConvexPolyhedronShape* polyhedron2,
                                                                    const Transform& polyhedron1ToPolyhedron2,
                                                                    uint faceIndex) const {

    RP3D_PROFILE("SATAlgorithm::testSingleFaceDirectionPolyhedronVsPolyhedron", mProfiler);

    const HalfEdgeStructure::Face& face = polyhedron1->getFace(faceIndex);

    // Get the face normal
    const Vector3 faceNormal = polyhedron1->getFaceNormal(faceIndex);

    // Convert the face normal into the local-space of polyhedron 2
    const Vector3 faceNormalPolyhedron2Space = polyhedron1ToPolyhedron2.getOrientation() * faceNormal;

    // Get the support point of polyhedron 2 in the inverse direction of face normal
    const Vector3 supportPoint = polyhedron2->getLocalSupportPointWithoutMargin(-faceNormalPolyhedron2Space);

    // Compute the penetration depth
    const Vector3 faceVertex = polyhedron1ToPolyhedron2 * polyhedron1->getVertexPosition(face.faceVertices[0]);
    decimal penetrationDepth = (faceVertex - supportPoint).dot(faceNormalPolyhedron2Space);

    return penetrationDepth;
}

// Test all the normals of a polyhedron for separating axis in the polyhedron vs polyhedron case
decimal SATAlgorithm::testFacesDirectionPolyhedronVsPolyhedron(const ConvexPolyhedronShape* polyhedron1,
                                                               const ConvexPolyhedronShape* polyhedron2,
                                                               const Transform& polyhedron1ToPolyhedron2,
                                                               uint& minFaceIndex) const {

    RP3D_PROFILE("SATAlgorithm::testFacesDirectionPolyhedronVsPolyhedron", mProfiler);

    decimal minPenetrationDepth = DECIMAL_LARGEST;

    // For each face of the first polyhedron
    for (uint f = 0; f < polyhedron1->getNbFaces(); f++) {

        decimal penetrationDepth = testSingleFaceDirectionPolyhedronVsPolyhedron(polyhedron1, polyhedron2,
                                                                                 polyhedron1ToPolyhedron2, f);

        // If the penetration depth is negative, we have found a separating axis
        if (penetrationDepth <= decimal(0.0)) {
            minFaceIndex = f;
            return penetrationDepth;
        }

        // Check if we have found a new minimum penetration axis
        if (penetrationDepth < minPenetrationDepth) {
            minPenetrationDepth = penetrationDepth;
            minFaceIndex = f;
        }
    }

    return minPenetrationDepth;
}


// Return true if two edges of two polyhedrons build a minkowski face (and can therefore be a separating axis)
bool SATAlgorithm::testEdgesBuildMinkowskiFace(const ConvexPolyhedronShape* polyhedron1, const HalfEdgeStructure::Edge& edge1,
                                               const ConvexPolyhedronShape* polyhedron2, const HalfEdgeStructure::Edge& edge2,
                                               const Transform& polyhedron1ToPolyhedron2) const {

    RP3D_PROFILE("SATAlgorithm::testEdgesBuildMinkowskiFace", mProfiler);

    const Vector3 a = polyhedron1ToPolyhedron2.getOrientation() * polyhedron1->getFaceNormal(edge1.faceIndex);
    const Vector3 b = polyhedron1ToPolyhedron2.getOrientation() * polyhedron1->getFaceNormal(polyhedron1->getHalfEdge(edge1.twinEdgeIndex).faceIndex);

    const Vector3 c = polyhedron2->getFaceNormal(edge2.faceIndex);
    const Vector3 d = polyhedron2->getFaceNormal(polyhedron2->getHalfEdge(edge2.twinEdgeIndex).faceIndex);

    // Compute b.cross(a) using the edge direction
    const Vector3 edge1Vertex1 = polyhedron1->getVertexPosition(edge1.vertexIndex);
    const Vector3 edge1Vertex2 = polyhedron1->getVertexPosition(polyhedron1->getHalfEdge(edge1.twinEdgeIndex).vertexIndex);
    const Vector3 bCrossA = polyhedron1ToPolyhedron2.getOrientation() * (edge1Vertex1 - edge1Vertex2);

    // Compute d.cross(c) using the edge direction
    const Vector3 edge2Vertex1 = polyhedron2->getVertexPosition(edge2.vertexIndex);
    const Vector3 edge2Vertex2 = polyhedron2->getVertexPosition(polyhedron2->getHalfEdge(edge2.twinEdgeIndex).vertexIndex);
    const Vector3 dCrossC = edge2Vertex1 - edge2Vertex2;

    // Test if the two arcs of the Gauss Map intersect (therefore forming a minkowski face)
    // Note that we negate the normals of the second polyhedron because we are looking at the
    // Gauss map of the minkowski difference of the polyhedrons
    return testGaussMapArcsIntersect(a, b, -c, -d, bCrossA, dCrossC);
}


// Return true if the arcs AB and CD on the Gauss Map (unit sphere) intersect
/// This is used to know if the edge between faces with normal A and B on first polyhedron
/// and edge between faces with normal C and D on second polygon create a face on the Minkowski
/// sum of both polygons. If this is the case, it means that the cross product of both edges
/// might be a separating axis.
bool SATAlgorithm::testGaussMapArcsIntersect(const Vector3& a, const Vector3& b, const Vector3& c, const Vector3& d,
                                             const Vector3& bCrossA, const Vector3& dCrossC) const {

    RP3D_PROFILE("SATAlgorithm::testGaussMapArcsIntersect", mProfiler);

    const decimal cba = c.dot(bCrossA);
    const decimal dba = d.dot(bCrossA);
    const decimal adc = a.dot(dCrossC);
    const decimal bdc = b.dot(dCrossC);

    return cba * dba < decimal(0.0) && adc * bdc < decimal(0.0) && cba * bdc > decimal(0.0);
}
