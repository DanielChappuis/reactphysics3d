/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2018 Daniel Chappuis                                       *
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
#include "ConvexPolyhedronVsConvexPolyhedronAlgorithm.h"
#include "GJK/GJKAlgorithm.h"
#include "SAT/SATAlgorithm.h"
#include "collision/NarrowPhaseInfo.h"

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Compute the narrow-phase collision detection between two convex polyhedra
// This technique is based on the "Robust Contact Creation for Physics Simulations" presentation
// by Dirk Gregorius.
bool ConvexPolyhedronVsConvexPolyhedronAlgorithm::testCollision(NarrowPhaseInfo* narrowPhaseInfo, bool reportContacts,
                                                                MemoryAllocator& memoryAllocator) {

    // Run the SAT algorithm to find the separating axis and compute contact point
    SATAlgorithm satAlgorithm(memoryAllocator);

#ifdef IS_PROFILING_ACTIVE

	satAlgorithm.setProfiler(mProfiler);

#endif

    // Get the last frame collision info
    LastFrameCollisionInfo* lastFrameCollisionInfo = narrowPhaseInfo->getLastFrameCollisionInfo();

    bool isColliding = satAlgorithm.testCollisionConvexPolyhedronVsConvexPolyhedron(narrowPhaseInfo, reportContacts);

    lastFrameCollisionInfo->wasUsingSAT = true;
    lastFrameCollisionInfo->wasUsingGJK = false;

    return isColliding;
}
