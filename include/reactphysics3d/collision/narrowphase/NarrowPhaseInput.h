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

#ifndef REACTPHYSICS3D_NARROW_PHASE_INPUT_H
#define REACTPHYSICS3D_NARROW_PHASE_INPUT_H

// Libraries
#include <reactphysics3d/containers/List.h>
#include <reactphysics3d/collision/narrowphase/NarrowPhaseInfoBatch.h>
#include <reactphysics3d/collision/narrowphase/SphereVsSphereNarrowPhaseInfoBatch.h>
#include <reactphysics3d/collision/narrowphase/CapsuleVsCapsuleNarrowPhaseInfoBatch.h>
#include <reactphysics3d/collision/narrowphase/SphereVsCapsuleNarrowPhaseInfoBatch.h>

/// Namespace ReactPhysics3D
namespace reactphysics3d {

// Declarations
class OverlappingPair;
class CollisionShape;
struct LastFrameCollisionInfo;
struct ContactPointInfo;
class NarrowPhaseAlgorithm;
enum class NarrowPhaseAlgorithmType;
class Transform;
struct Vector3;

// Class NarrowPhaseInput
/**
 * This structure contains everything that is needed to perform the narrow-phase
 * collision detection.
 */
class NarrowPhaseInput {

    private:

        SphereVsSphereNarrowPhaseInfoBatch mSphereVsSphereBatch;
        SphereVsCapsuleNarrowPhaseInfoBatch mSphereVsCapsuleBatch;
        CapsuleVsCapsuleNarrowPhaseInfoBatch mCapsuleVsCapsuleBatch;
        NarrowPhaseInfoBatch mSphereVsConvexPolyhedronBatch;
        NarrowPhaseInfoBatch mCapsuleVsConvexPolyhedronBatch;
        NarrowPhaseInfoBatch mConvexPolyhedronVsConvexPolyhedronBatch;

    public:

        /// Constructor
        NarrowPhaseInput(MemoryAllocator& allocator, OverlappingPairs& overlappingPairs);

        /// Add shapes to be tested during narrow-phase collision detection into the batch
        void addNarrowPhaseTest(uint64 pairId, uint64 pairIndex, Entity collider1, Entity collider2, CollisionShape* shape1,
                        CollisionShape* shape2, const Transform& shape1Transform,
                        const Transform& shape2Transform, NarrowPhaseAlgorithmType narrowPhaseAlgorithmType, bool reportContacts,
                        MemoryAllocator& shapeAllocator);

        /// Get a reference to the sphere vs sphere batch
        SphereVsSphereNarrowPhaseInfoBatch& getSphereVsSphereBatch();

        /// Get a reference to the sphere vs capsule batch
        SphereVsCapsuleNarrowPhaseInfoBatch& getSphereVsCapsuleBatch();

        /// Get a reference to the capsule vs capsule batch
        CapsuleVsCapsuleNarrowPhaseInfoBatch& getCapsuleVsCapsuleBatch();

        /// Get a reference to the sphere vs convex polyhedron batch
        NarrowPhaseInfoBatch& getSphereVsConvexPolyhedronBatch();

        /// Get a reference to the capsule vs convex polyhedron batch
        NarrowPhaseInfoBatch& getCapsuleVsConvexPolyhedronBatch();

        /// Get a reference to the convex polyhedron vs convex polyhedron batch
        NarrowPhaseInfoBatch& getConvexPolyhedronVsConvexPolyhedronBatch();

        /// Reserve memory for the containers with cached capacity
        void reserveMemory();

        /// Clear
        void clear();
};


// Get a reference to the sphere vs sphere batch contacts
inline SphereVsSphereNarrowPhaseInfoBatch& NarrowPhaseInput::getSphereVsSphereBatch() {
    return mSphereVsSphereBatch;
}

// Get a reference to the sphere vs capsule batch contacts
inline SphereVsCapsuleNarrowPhaseInfoBatch& NarrowPhaseInput::getSphereVsCapsuleBatch() {
   return mSphereVsCapsuleBatch;
}

// Get a reference to the capsule vs capsule batch contacts
inline CapsuleVsCapsuleNarrowPhaseInfoBatch& NarrowPhaseInput::getCapsuleVsCapsuleBatch() {
   return mCapsuleVsCapsuleBatch;
}

// Get a reference to the sphere vs convex polyhedron batch contacts
inline NarrowPhaseInfoBatch& NarrowPhaseInput::getSphereVsConvexPolyhedronBatch() {
   return mSphereVsConvexPolyhedronBatch;
}

// Get a reference to the capsule vs convex polyhedron batch contacts
inline NarrowPhaseInfoBatch& NarrowPhaseInput::getCapsuleVsConvexPolyhedronBatch() {
   return mCapsuleVsConvexPolyhedronBatch;
}

// Get a reference to the convex polyhedron vs convex polyhedron batch contacts
inline NarrowPhaseInfoBatch& NarrowPhaseInput::getConvexPolyhedronVsConvexPolyhedronBatch() {
   return mConvexPolyhedronVsConvexPolyhedronBatch;
}

}
#endif
