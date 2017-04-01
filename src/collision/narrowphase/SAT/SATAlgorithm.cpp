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
#include "collision/shapes/ConvexPolyhedronShape.h"
#include "collision/shapes/SphereShape.h"
#include "configuration.h"
#include "engine/Profiler.h"
#include <algorithm>
#include <cmath>
#include <cfloat>
#include <cassert>

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

bool SATAlgorithm::testCollision(const NarrowPhaseInfo* narrowPhaseInfo, ContactManifoldInfo& contactManifoldInfo) {

    assert(narrowPhaseInfo->collisionShape2->getType() == CollisionShapeType::CONVEX_POLYHEDRON);

    switch (narrowPhaseInfo->collisionShape1->getType()) {
        case CollisionShapeType::CONVEX_POLYHEDRON:
            return testCollisionConvexMeshVsConvexMesh(narrowPhaseInfo, contactManifoldInfo);
        case CollisionShapeType::SPHERE:
            return testCollisionSphereVsConvexPolyhedron(narrowPhaseInfo, contactManifoldInfo);
        case CollisionShapeType::CAPSULE:
            return testCollisionCapsuleVsConvexMesh(narrowPhaseInfo, contactManifoldInfo);
        case CollisionShapeType::TRIANGLE:
            return testCollisionTriangleVsConvexMesh(narrowPhaseInfo, contactManifoldInfo);
        default: assert(false);
    }

    return false;
}

// Test collision between a sphere and a convex mesh
bool SATAlgorithm::testCollisionSphereVsConvexPolyhedron(const NarrowPhaseInfo* narrowPhaseInfo, ContactManifoldInfo& contactManifoldInfo) const {

    assert(narrowPhaseInfo->collisionShape1->getType() == CollisionShapeType::SPHERE);
    assert(narrowPhaseInfo->collisionShape2->getType() == CollisionShapeType::CONVEX_POLYHEDRON);

    // Get the capsule collision shapes
    const SphereShape* sphere = static_cast<const SphereShape*>(narrowPhaseInfo->collisionShape1);
    const ConvexPolyhedronShape* polyhedron = static_cast<const ConvexPolyhedronShape*>(narrowPhaseInfo->collisionShape2);


    // Get the transform from sphere local-space to polyhedron local-space
    const Transform sphereToPolyhedronSpaceTransform = narrowPhaseInfo->shape2ToWorldTransform.getInverse() *
                                                       narrowPhaseInfo->shape1ToWorldTransform;

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
    const Vector3 normalWorld = -(narrowPhaseInfo->shape2ToWorldTransform.getOrientation() * minFaceNormal);
    const Vector3 contactPointSphereLocal = narrowPhaseInfo->shape1ToWorldTransform.getInverse() * normalWorld * sphere->getRadius();
    const Vector3 contactPointPolyhedronLocal = sphereCenter + minFaceNormal * (minPenetrationDepth - sphere->getRadius());

    // Create the contact info object
    contactManifoldInfo.addContactPoint(normalWorld, minPenetrationDepth, contactPointSphereLocal, contactPointPolyhedronLocal);

    return true;
}

// Test collision between a capsule and a convex mesh
bool SATAlgorithm::testCollisionCapsuleVsConvexMesh(const NarrowPhaseInfo* narrowPhaseInfo, ContactManifoldInfo& contactManifoldInfo) const {

    assert(narrowPhaseInfo->collisionShape1->getType() == CollisionShapeType::CAPSULE);
    assert(narrowPhaseInfo->collisionShape2->getType() == CollisionShapeType::CONVEX_POLYHEDRON);
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
