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

#ifndef REACTPHYSICS3D_CAPSULE_VS_CONVEX_POLYHEDRON_ALGORITHM_H
#define	REACTPHYSICS3D_CAPSULE_VS_CONVEX_POLYHEDRON_ALGORITHM_H

// Libraries
#include <reactphysics3d/collision/narrowphase/NarrowPhaseAlgorithm.h>

/// Namespace ReactPhysics3D
namespace reactphysics3d {

// Declarations
class ContactPoint;

// Class CapsuleVsConvexPolyhedronAlgorithm
/**
 * This class is used to compute the narrow-phase collision detection
 * between a capsule and a convex polyhedron. The capsule is basically
 * a line segment with a margin around it. First we run the GJK algorithm.
 * If GJK reports separation, we are done. If the objects overlap inside the
 * capsule margin (radius), it will also report contact points and normal.
 * However, if GJK report penetration of the capsule inner segment within
 * the polyhedron, we run the SAT algorithm to get the contact points and
 * normal.
 * This is based on the "Robust Contact Creation for Physics Simulation"
 * presentation by Dirk Gregorius.
 */
class CapsuleVsConvexPolyhedronAlgorithm : public NarrowPhaseAlgorithm {

    protected :

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        CapsuleVsConvexPolyhedronAlgorithm() = default;

        /// Destructor
        virtual ~CapsuleVsConvexPolyhedronAlgorithm() override = default;

        /// Deleted copy-constructor
        CapsuleVsConvexPolyhedronAlgorithm(const CapsuleVsConvexPolyhedronAlgorithm& algorithm) = delete;

        /// Deleted assignment operator
        CapsuleVsConvexPolyhedronAlgorithm& operator=(const CapsuleVsConvexPolyhedronAlgorithm& algorithm) = delete;

        /// Compute the narrow-phase collision detection between a capsule and a polyhedron
        bool testCollision(NarrowPhaseInfoBatch& narrowPhaseInfoBatch, uint batchStartIndex,
                           uint batchNbItems, bool clipWithPreviousAxisIfStillColliding,
                           MemoryAllocator& memoryAllocator);
};

}

#endif

