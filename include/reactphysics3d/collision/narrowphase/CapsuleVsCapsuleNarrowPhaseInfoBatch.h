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

#ifndef REACTPHYSICS3D_CAPSULE_VS_CAPSULE_NARROW_PHASE_INFO_BATCH_H
#define REACTPHYSICS3D_CAPSULE_VS_CAPSULE_NARROW_PHASE_INFO_BATCH_H

// Libraries
#include <reactphysics3d/collision/narrowphase/NarrowPhaseInfoBatch.h>

/// Namespace ReactPhysics3D
namespace reactphysics3d {

// Struct CapsuleVsCapsuleNarrowPhaseInfoBatch
/**
 * This structure collects all the potential collisions from the middle-phase algorithm
 * that have to be tested during narrow-phase collision detection. This class collects all the
 * capsule vs capsule collision detection tests.
 */
struct CapsuleVsCapsuleNarrowPhaseInfoBatch : public NarrowPhaseInfoBatch {

    public:

        /// List of radiuses for the first capsules
        List<decimal> capsule1Radiuses;

        /// List of radiuses for the second capsules
        List<decimal> capsule2Radiuses;

        /// List of heights for the first capsules
        List<decimal> capsule1Heights;

        /// List of heights for the second capsules
        List<decimal> capsule2Heights;

        /// Constructor
        CapsuleVsCapsuleNarrowPhaseInfoBatch(MemoryAllocator& allocator, OverlappingPairs& overlappingPairs);

        /// Destructor
        virtual ~CapsuleVsCapsuleNarrowPhaseInfoBatch() override = default;

        /// Add shapes to be tested during narrow-phase collision detection into the batch
        virtual void addNarrowPhaseInfo(uint64 pairId, uint64 pairIndex, Entity collider1, Entity collider2, CollisionShape* shape1,
                                        CollisionShape* shape2, const Transform& shape1Transform,
                                        const Transform& shape2Transform, bool needToReportContacts, MemoryAllocator& shapeAllocator) override;

        // Initialize the containers using cached capacity
        virtual void reserveMemory() override;

        /// Clear all the objects in the batch
        virtual void clear() override;
};

}

#endif

