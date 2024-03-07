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

#ifndef REACTPHYSICS3D_SPHERE_VS_SPHERE_ALGORITHM_H
#define	REACTPHYSICS3D_SPHERE_VS_SPHERE_ALGORITHM_H

// Libraries
#include <reactphysics3d/collision/narrowphase/NarrowPhaseAlgorithm.h>

/// Namespace ReactPhysics3D
namespace reactphysics3d {

// Declarations
class ContactPoint;
struct NarrowPhaseInfoBatch;

// Class SphereVsSphereAlgorithm
/**
 * This class is used to compute the narrow-phase collision detection
 * between two sphere collision shapes. This algorithm finds the contact
 * point and contact normal between two spheres if they are colliding.
 * This case is simple, we do not need to use GJK or SAT algorithm. We
 * directly compute the contact points if any.
 */
class SphereVsSphereAlgorithm : public NarrowPhaseAlgorithm {

    protected :

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        SphereVsSphereAlgorithm() = default;

        /// Destructor
        virtual ~SphereVsSphereAlgorithm() override = default;

        /// Deleted copy-constructor
        SphereVsSphereAlgorithm(const SphereVsSphereAlgorithm& algorithm) = delete;

        /// Deleted assignment operator
        SphereVsSphereAlgorithm& operator=(const SphereVsSphereAlgorithm& algorithm) = delete;

        /// Compute a contact info if the two bounding volume collide
        bool testCollision(NarrowPhaseInfoBatch& narrowPhaseInfoBatch, uint32 batchStartIndex,
                           uint32 batchNbItems, MemoryAllocator& memoryAllocator);
};

}

#endif

