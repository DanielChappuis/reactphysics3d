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
            return testCollisionSphereVsConvexMesh(narrowPhaseInfo, contactManifoldInfo);
        case CollisionShapeType::CAPSULE:
            return testCollisionCapsuleVsConvexMesh(narrowPhaseInfo, contactManifoldInfo);
        case CollisionShapeType::TRIANGLE:
            return testCollisionTriangleVsConvexMesh(narrowPhaseInfo, contactManifoldInfo);
        default: assert(false);
    }

    return false;
}

// Test collision between a sphere and a convex mesh
bool SATAlgorithm::testCollisionSphereVsConvexMesh(const NarrowPhaseInfo* narrowPhaseInfo, ContactManifoldInfo& contactManifoldInfo) const {

}

// Test collision between a capsule and a convex mesh
bool SATAlgorithm::testCollisionCapsuleVsConvexMesh(const NarrowPhaseInfo* narrowPhaseInfo, ContactManifoldInfo& contactManifoldInfo) const {

}

// Test collision between a triangle and a convex mesh
bool SATAlgorithm::testCollisionTriangleVsConvexMesh(const NarrowPhaseInfo* narrowPhaseInfo, ContactManifoldInfo& contactManifoldInfo) const {

}

// Test collision between two convex meshes
bool SATAlgorithm::testCollisionConvexMeshVsConvexMesh(const NarrowPhaseInfo* narrowPhaseInfo, ContactManifoldInfo& contactManifoldInfo) const {

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
