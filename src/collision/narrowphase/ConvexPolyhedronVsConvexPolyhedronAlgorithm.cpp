/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2024 Daniel Chappuis                                       *
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
#include <reactphysics3d/collision/narrowphase/ConvexPolyhedronVsConvexPolyhedronAlgorithm.h>
#include <reactphysics3d/collision/narrowphase/GJK/GJKAlgorithm.h>
#include <reactphysics3d/collision/narrowphase/SAT/SATAlgorithm.h>
#include <reactphysics3d/collision/narrowphase/NarrowPhaseInfoBatch.h>

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Compute the narrow-phase collision detection between two convex polyhedra
// This technique is based on the "Robust Contact Creation for Physics Simulations" presentation
// by Dirk Gregorius.
bool ConvexPolyhedronVsConvexPolyhedronAlgorithm::testCollision(NarrowPhaseInfoBatch &narrowPhaseInfoBatch,
                                                                uint32 batchStartIndex, uint32 batchNbItems,
                                                                bool clipWithPreviousAxisIfStillColliding, MemoryAllocator& memoryAllocator) {

    // Run the SAT algorithm to find the separating axis and compute contact point
    SATAlgorithm satAlgorithm(clipWithPreviousAxisIfStillColliding, memoryAllocator);

#ifdef IS_RP3D_PROFILING_ENABLED


	satAlgorithm.setProfiler(mProfiler);

#endif

    bool isCollisionFound = satAlgorithm.testCollisionConvexPolyhedronVsConvexPolyhedron(narrowPhaseInfoBatch, batchStartIndex, batchNbItems);

    for (uint32 batchIndex = batchStartIndex; batchIndex < batchStartIndex + batchNbItems; batchIndex++) {

        // Get the last frame collision info
        LastFrameCollisionInfo* lastFrameCollisionInfo = narrowPhaseInfoBatch.narrowPhaseInfos[batchIndex].lastFrameCollisionInfo;

        lastFrameCollisionInfo->wasUsingSAT = true;
        lastFrameCollisionInfo->wasUsingGJK = false;
    }

    return isCollisionFound;
}
