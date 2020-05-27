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

// Libraries
#include <reactphysics3d/collision/narrowphase/SphereVsConvexPolyhedronAlgorithm.h>
#include <reactphysics3d/collision/narrowphase/GJK/GJKAlgorithm.h>
#include <reactphysics3d/collision/narrowphase/SAT/SATAlgorithm.h>
#include <reactphysics3d/collision/narrowphase/NarrowPhaseInfoBatch.h>

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Compute the narrow-phase collision detection between a sphere and a convex polyhedron
// This technique is based on the "Robust Contact Creation for Physics Simulations" presentation
// by Dirk Gregorius.
bool SphereVsConvexPolyhedronAlgorithm::testCollision(NarrowPhaseInfoBatch& narrowPhaseInfoBatch, uint batchStartIndex, uint batchNbItems,
                                                      bool clipWithPreviousAxisIfStillColliding, MemoryAllocator& memoryAllocator) {

    // First, we run the GJK algorithm
    GJKAlgorithm gjkAlgorithm;

    bool isCollisionFound = false;

#ifdef IS_RP3D_PROFILING_ENABLED


        gjkAlgorithm.setProfiler(mProfiler);

#endif

    List<GJKAlgorithm::GJKResult> gjkResults(memoryAllocator);
    gjkAlgorithm.testCollision(narrowPhaseInfoBatch, batchStartIndex, batchNbItems, gjkResults);
    assert(gjkResults.size() == batchNbItems);

    // For each item in the batch
    for (uint batchIndex = batchStartIndex; batchIndex < batchStartIndex + batchNbItems; batchIndex++) {

        assert(narrowPhaseInfoBatch.collisionShapes1[batchIndex]->getType() == CollisionShapeType::CONVEX_POLYHEDRON ||
            narrowPhaseInfoBatch.collisionShapes2[batchIndex]->getType() == CollisionShapeType::CONVEX_POLYHEDRON);
        assert(narrowPhaseInfoBatch.collisionShapes1[batchIndex]->getType() == CollisionShapeType::SPHERE ||
            narrowPhaseInfoBatch.collisionShapes2[batchIndex]->getType() == CollisionShapeType::SPHERE);

        // Get the last frame collision info
        LastFrameCollisionInfo* lastFrameCollisionInfo = narrowPhaseInfoBatch.lastFrameCollisionInfos[batchIndex];

        lastFrameCollisionInfo->wasUsingGJK = true;
        lastFrameCollisionInfo->wasUsingSAT = false;

        // If we have found a contact point inside the margins (shallow penetration)
        if (gjkResults[batchIndex] == GJKAlgorithm::GJKResult::COLLIDE_IN_MARGIN) {

            // Return true
            narrowPhaseInfoBatch.isColliding[batchIndex] = true;
            isCollisionFound = true;
            continue;
        }

        // If we have overlap even without the margins (deep penetration)
        if (gjkResults[batchIndex] == GJKAlgorithm::GJKResult::INTERPENETRATE) {

            // Run the SAT algorithm to find the separating axis and compute contact point
            SATAlgorithm satAlgorithm(clipWithPreviousAxisIfStillColliding, memoryAllocator);

#ifdef IS_RP3D_PROFILING_ENABLED


            satAlgorithm.setProfiler(mProfiler);

#endif

            isCollisionFound |= satAlgorithm.testCollisionSphereVsConvexPolyhedron(narrowPhaseInfoBatch, batchIndex, 1);

            lastFrameCollisionInfo->wasUsingGJK = false;
            lastFrameCollisionInfo->wasUsingSAT = true;

            continue;
        }
    }

    return isCollisionFound;
}
