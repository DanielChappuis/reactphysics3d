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

// Libraries
#include "SATAlgorithm.h"
#include "constraint/ContactPoint.h"
#include "collision/PolyhedronMesh.h"
#include "collision/shapes/CapsuleShape.h"
#include "collision/shapes/SphereShape.h"
#include "configuration.h"
#include "engine/Profiler.h"
#include <algorithm>
#include <cmath>
#include <cfloat>
#include <cassert>

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Test collision between a sphere and a convex mesh
bool SATAlgorithm::testCollisionSphereVsConvexPolyhedron(const NarrowPhaseInfo* narrowPhaseInfo, ContactManifoldInfo& contactManifoldInfo) const {

    bool isSphereShape1 = narrowPhaseInfo->collisionShape1->getType() == CollisionShapeType::SPHERE;

    assert(isSphereShape1 || narrowPhaseInfo->collisionShape1->getType() == CollisionShapeType::CONVEX_POLYHEDRON);
    assert(!isSphereShape1 || narrowPhaseInfo->collisionShape1->getType() == CollisionShapeType::CONVEX_POLYHEDRON);
    assert(!isSphereShape1 || narrowPhaseInfo->collisionShape2->getType() == CollisionShapeType::SPHERE);

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

        // Get the face
        HalfEdgeStructure::Face face = polyhedron->getFace(f);

        // Get the face normal
        const Vector3 faceNormal = polyhedron->getFaceNormal(f);

        Vector3 sphereCenterToFacePoint = polyhedron->getVertexPosition(face.faceVertices[0]) - sphereCenter;
        decimal penetrationDepth = sphereCenterToFacePoint.dot(faceNormal) + sphere->getRadius();

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

    const Vector3 minFaceNormal = polyhedron->getFaceNormal(minFaceIndex);
    Vector3 normalWorld = -(polyhedronToWorldTransform.getOrientation() * minFaceNormal);
    const Vector3 contactPointSphereLocal = sphereToWorldTransform.getInverse() * normalWorld * sphere->getRadius();
    const Vector3 contactPointPolyhedronLocal = sphereCenter + minFaceNormal * (minPenetrationDepth - sphere->getRadius());

    if (!isSphereShape1) {
        normalWorld = -normalWorld;
    }

    // Create the contact info object
    contactManifoldInfo.addContactPoint(normalWorld, minPenetrationDepth,
                                        isSphereShape1 ? contactPointSphereLocal : contactPointPolyhedronLocal,
                                        isSphereShape1 ? contactPointPolyhedronLocal : contactPointSphereLocal);

    return true;
}

// Test collision between a capsule and a convex mesh
bool SATAlgorithm::testCollisionCapsuleVsConvexPolyhedron(const NarrowPhaseInfo* narrowPhaseInfo, ContactManifoldInfo& contactManifoldInfo) const {

    bool isCapsuleShape1 = narrowPhaseInfo->collisionShape1->getType() == CollisionShapeType::CAPSULE;

    assert(isCapsuleShape1 || narrowPhaseInfo->collisionShape1->getType() == CollisionShapeType::CONVEX_POLYHEDRON);
    assert(!isCapsuleShape1 || narrowPhaseInfo->collisionShape1->getType() == CollisionShapeType::CONVEX_POLYHEDRON);
    assert(!isCapsuleShape1 || narrowPhaseInfo->collisionShape2->getType() == CollisionShapeType::CAPSULE);

    // Get the collision shapes
    const CapsuleShape* capsuleShape = static_cast<const CapsuleShape*>(isCapsuleShape1 ? narrowPhaseInfo->collisionShape1 : narrowPhaseInfo->collisionShape2);
    const ConvexPolyhedronShape* polyhedron = static_cast<const ConvexPolyhedronShape*>(isCapsuleShape1 ? narrowPhaseInfo->collisionShape2 : narrowPhaseInfo->collisionShape1);

    const Transform capsuleToWorld = isCapsuleShape1 ? narrowPhaseInfo->shape1ToWorldTransform : narrowPhaseInfo->shape2ToWorldTransform;
    const Transform polyhedronToWorld = isCapsuleShape1 ? narrowPhaseInfo->shape2ToWorldTransform : narrowPhaseInfo->shape1ToWorldTransform;

    const Transform polyhedronToCapsuleTransform = capsuleToWorld.getInverse() * polyhedronToWorld;

    // Minimum penetration depth
    decimal minPenetrationDepth = DECIMAL_LARGEST;
    uint minFaceIndex = 0;
    uint minEdgeIndex = 0;
    bool isMinPenetrationFaceNormal = false;
    Vector3 separatingAxisCapsuleSpace;
    Vector3 separatingPolyhedronEdgeVertex1;
    Vector3 separatingPolyhedronEdgeVertex2;

    // For each face of the convex mesh
    for (uint f = 0; f < polyhedron->getNbFaces(); f++) {

        // Get the face
        HalfEdgeStructure::Face face = polyhedron->getFace(f);

        // Get the face normal
        const Vector3 faceNormal = polyhedron->getFaceNormal(f);

        // Compute the penetration depth (using the capsule support in the direction opposite to the face normal)
        const Vector3 faceNormalCapsuleSpace = polyhedronToCapsuleTransform.getOrientation() * faceNormal;
        const Vector3 capsuleSupportPoint = capsuleShape->getLocalSupportPointWithMargin(-faceNormalCapsuleSpace, nullptr);
        const Vector3 pointOnPolyhedronFace = polyhedronToCapsuleTransform * polyhedron->getVertexPosition(face.faceVertices[0]);
        const Vector3 capsuleSupportPointToFacePoint =  pointOnPolyhedronFace - capsuleSupportPoint;
        const decimal penetrationDepth = capsuleSupportPointToFacePoint.dot(faceNormal);

        // If the penetration depth is negative, we have found a separating axis
        if (penetrationDepth <= decimal(0.0)) {
            return false;
        }

        // Check if we have found a new minimum penetration axis
        if (penetrationDepth < minPenetrationDepth) {
            minPenetrationDepth = penetrationDepth;
            minFaceIndex = f;
            isMinPenetrationFaceNormal = true;
            separatingAxisCapsuleSpace = faceNormalCapsuleSpace;
        }
    }

    // Compute the end-points of the inner segment of the capsule
    const Vector3 capsuleSegA(0, -capsuleShape->getHeight() * decimal(0.5), 0);
    const Vector3 capsuleSegB(0, capsuleShape->getHeight() * decimal(0.5), 0);
    const Vector3 capsuleSegmentAxis = capsuleSegB - capsuleSegA;

    // For each direction that is the cross product of the capsule inner segment and
    // an edge of the polyhedron
    for (uint e = 0; e < polyhedron->getNbHalfEdges(); e += 2) {

        // Get an edge from the polyhedron (convert it into the capsule local-space)
        HalfEdgeStructure::Edge edge = polyhedron->getHalfEdge(e);
        const Vector3 edgeVertex1 = polyhedron->getVertexPosition(edge.vertexIndex);
        const Vector3 edgeVertex2 = polyhedron->getVertexPosition(polyhedron->getHalfEdge(edge.nextEdgeIndex).vertexIndex);
        const Vector3 edgeDirectionCapsuleSpace = polyhedronToCapsuleTransform.getOrientation() * (edgeVertex2 - edgeVertex1);

        HalfEdgeStructure::Edge twinEdge = polyhedron->getHalfEdge(edge.twinEdgeIndex);
        const Vector3 adjacentFace1Normal = polyhedronToCapsuleTransform.getOrientation() * polyhedron->getFaceNormal(edge.faceIndex);
        const Vector3 adjacentFace2Normal = polyhedronToCapsuleTransform.getOrientation() * polyhedron->getFaceNormal(twinEdge.faceIndex);

        // Check using the Gauss Map if this edge cross product can be as separating axis
        if (isMinkowskiFaceCapsuleVsEdge(capsuleSegmentAxis, adjacentFace1Normal, adjacentFace2Normal)) {

            // Compute the axis to test (cross product between capsule inner segment and polyhedron edge)
            Vector3 axis = capsuleSegmentAxis.cross(edgeDirectionCapsuleSpace);

            // Skip separating axis test if polyhedron edge is parallel to the capsule inner segment
            if (axis.lengthSquare() >= decimal(0.00001)) {

                const Vector3 polyhedronCentroid = polyhedronToCapsuleTransform * polyhedron->getCentroid();
                const Vector3 pointOnPolyhedronEdge = polyhedronToCapsuleTransform * edgeVertex1;

                // Swap axis direction if necessary such that it points out of the polyhedron
                if (axis.dot(pointOnPolyhedronEdge - polyhedronCentroid) < 0) {
                    axis = -axis;
                }

                axis.normalize();

                // Compute the penetration depth
                const Vector3 capsuleSupportPoint = capsuleShape->getLocalSupportPointWithMargin(-axis, nullptr);
                const Vector3 capsuleSupportPointToEdgePoint = pointOnPolyhedronEdge - capsuleSupportPoint;
                const decimal penetrationDepth = capsuleSupportPointToEdgePoint.dot(axis);

                // If the penetration depth is negative, we have found a separating axis
                if (penetrationDepth <= decimal(0.0)) {
                    return false;
                }

                // Check if we have found a new minimum penetration axis
                if (penetrationDepth < minPenetrationDepth) {
                    minPenetrationDepth = penetrationDepth;
                    minEdgeIndex = e;
                    isMinPenetrationFaceNormal = false;
                    separatingAxisCapsuleSpace = axis;
                    separatingPolyhedronEdgeVertex1 = edgeVertex1;
                    separatingPolyhedronEdgeVertex2 = edgeVertex2;
                }
            }
        }
    }

    // Convert the inner capsule segment points into the polyhedron local-space
    const Transform capsuleToPolyhedronTransform = polyhedronToCapsuleTransform.getInverse();
    const Vector3 capsuleSegAPolyhedronSpace = capsuleToPolyhedronTransform * capsuleSegA;
    const Vector3 capsuleSegBPolyhedronSpace = capsuleToPolyhedronTransform * capsuleSegB;

    const Vector3 normalWorld = capsuleToWorld.getOrientation() * separatingAxisCapsuleSpace;
    const decimal capsuleRadius = capsuleShape->getRadius();

    // If the separating axis is a face normal
    // We need to clip the inner capsule segment with the adjacent faces of the separating face
    if (isMinPenetrationFaceNormal) {

        computeCapsulePolyhedronFaceContactPoints(minFaceIndex, capsuleRadius, polyhedron, minPenetrationDepth,
                                                  polyhedronToCapsuleTransform, normalWorld, separatingAxisCapsuleSpace,
                                                  capsuleSegAPolyhedronSpace, capsuleSegBPolyhedronSpace,
                                                  contactManifoldInfo, isCapsuleShape1);
    }
    else {   // The separating axis is the cross product of a polyhedron edge and the inner capsule segment

        // Compute the closest points between the inner capsule segment and the
        // edge of the polyhedron in polyhedron local-space
        Vector3 closestPointPolyhedronEdge, closestPointCapsuleInnerSegment;
        computeClosestPointBetweenTwoSegments(capsuleSegAPolyhedronSpace, capsuleSegBPolyhedronSpace,
                                              separatingPolyhedronEdgeVertex1, separatingPolyhedronEdgeVertex2,
                                              closestPointCapsuleInnerSegment, closestPointPolyhedronEdge);


        // Project closest capsule inner segment point into the capsule bounds
        const Vector3 contactPointCapsule = (polyhedronToCapsuleTransform * closestPointCapsuleInnerSegment) - separatingAxisCapsuleSpace * capsuleRadius;

        // Create the contact point
        contactManifoldInfo.addContactPoint(normalWorld, minPenetrationDepth,
                                            isCapsuleShape1 ? contactPointCapsule : closestPointPolyhedronEdge,
                                            isCapsuleShape1 ? closestPointPolyhedronEdge : contactPointCapsule);
    }

    return true;
}

// Compute the two contact points between a polyhedron and a capsule when the separating
// axis is a face normal of the polyhedron
void SATAlgorithm::computeCapsulePolyhedronFaceContactPoints(uint referenceFaceIndex, decimal capsuleRadius, const ConvexPolyhedronShape* polyhedron,
                                                             decimal penetrationDepth, const Transform& polyhedronToCapsuleTransform,
                                                             const Vector3& normalWorld, const Vector3& separatingAxisCapsuleSpace,
                                                             const Vector3& capsuleSegAPolyhedronSpace, const Vector3& capsuleSegBPolyhedronSpace,
                                                             ContactManifoldInfo& contactManifoldInfo, bool isCapsuleShape1) const {

    HalfEdgeStructure::Face face = polyhedron->getFace(referenceFaceIndex);
    uint firstEdgeIndex = face.edgeIndex;
    uint edgeIndex = firstEdgeIndex;

    std::vector<Vector3> planesPoints;
    std::vector<Vector3> planesNormals;

    // For each adjacent edge of the separating face of the polyhedron
    do {
        HalfEdgeStructure::Edge edge = polyhedron->getHalfEdge(edgeIndex);
        HalfEdgeStructure::Edge twinEdge = polyhedron->getHalfEdge(edge.twinEdgeIndex);

        // Construct a clippling plane for each adjacent edge of the separting face of the polyhedron
        planesPoints.push_back(polyhedron->getVertexPosition(edge.vertexIndex));
        planesNormals.push_back(polyhedron->getFaceNormal(twinEdge.faceIndex));

        edgeIndex = edge.nextEdgeIndex;

    } while(edgeIndex != firstEdgeIndex);

    // First we clip the inner segment of the capsule with the four planes of the adjacent faces
    std::vector<Vector3> clipSegment = clipSegmentWithPlanes(capsuleSegAPolyhedronSpace, capsuleSegBPolyhedronSpace,
                                                             planesPoints, planesNormals);

    // Project the two clipped points into the polyhedron face
    const Vector3 faceNormal = polyhedron->getFaceNormal(referenceFaceIndex);
    const Vector3 contactPoint1Polyhedron = clipSegment[0] + faceNormal * (penetrationDepth - capsuleRadius);
    const Vector3 contactPoint2Polyhedron = clipSegment[1] + faceNormal * (penetrationDepth - capsuleRadius);

    // Project the two clipped points into the capsule bounds
    const Vector3 contactPoint1Capsule = (polyhedronToCapsuleTransform * clipSegment[0]) - separatingAxisCapsuleSpace * capsuleRadius;
    const Vector3 contactPoint2Capsule = (polyhedronToCapsuleTransform * clipSegment[1]) - separatingAxisCapsuleSpace * capsuleRadius;

    // Create the contact points
    contactManifoldInfo.addContactPoint(normalWorld, penetrationDepth,
                                        isCapsuleShape1 ? contactPoint1Capsule : contactPoint1Polyhedron,
                                        isCapsuleShape1 ? contactPoint1Polyhedron : contactPoint1Capsule);
    contactManifoldInfo.addContactPoint(normalWorld, penetrationDepth,
                                        isCapsuleShape1 ? contactPoint2Capsule : contactPoint2Polyhedron,
                                        isCapsuleShape1 ? contactPoint2Polyhedron : contactPoint2Capsule);
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

// Test collision between a triangle and a convex mesh
bool SATAlgorithm::testCollisionTriangleVsConvexMesh(const NarrowPhaseInfo* narrowPhaseInfo, ContactManifoldInfo& contactManifoldInfo) const {

    assert(narrowPhaseInfo->collisionShape1->getType() == CollisionShapeType::TRIANGLE);
    assert(narrowPhaseInfo->collisionShape2->getType() == CollisionShapeType::CONVEX_POLYHEDRON);
}

// Test collision between two convex meshes
bool SATAlgorithm::testCollisionConvexMeshVsConvexMesh(const NarrowPhaseInfo* narrowPhaseInfo, ContactManifoldInfo& contactManifoldInfo) const {

    assert(narrowPhaseInfo->collisionShape1->getType() == CollisionShapeType::CONVEX_POLYHEDRON);
    assert(narrowPhaseInfo->collisionShape2->getType() == CollisionShapeType::CONVEX_POLYHEDRON);
}


// Return true if the arcs AB and CD on the Gauss Map (unit sphere) intersect
/// This is used to know if the edge between faces with normal A and B on first polyhedron
/// and edge between faces with normal C and D on second polygon create a face on the Minkowski
/// sum of both polygons. If this is the case, it means that the cross product of both edges
/// might be a separating axis.
bool SATAlgorithm::testGaussMapArcsIntersect(const Vector3& a, const Vector3& b,
                                             const Vector3& c, const Vector3& d) const {
    const Vector3 bCrossA = b.cross(a);
    const Vector3 dCrossC = d.cross(c);

    const decimal cba = c.dot(bCrossA);
    const decimal dba = d.dot(bCrossA);
    const decimal adc = a.dot(dCrossC);
    const decimal bdc = b.dot(dCrossC);

    return cba * dba < decimal(0.0) && adc * bdc < decimal(0.0) && cba * bdc > decimal(0.0);
}
