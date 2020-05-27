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
#include <reactphysics3d/collision/narrowphase/SphereVsCapsuleNarrowPhaseInfoBatch.h>
#include <reactphysics3d/collision/shapes/SphereShape.h>
#include <reactphysics3d/collision/shapes/CapsuleShape.h>

using namespace reactphysics3d;

// Constructor
SphereVsCapsuleNarrowPhaseInfoBatch::SphereVsCapsuleNarrowPhaseInfoBatch(MemoryAllocator& allocator,
                                                                         OverlappingPairs& overlappingPairs)
      : NarrowPhaseInfoBatch(allocator, overlappingPairs), isSpheresShape1(allocator), sphereRadiuses(allocator), capsuleRadiuses(allocator),
        capsuleHeights(allocator) {

}

// Add shapes to be tested during narrow-phase collision detection into the batch
void SphereVsCapsuleNarrowPhaseInfoBatch::addNarrowPhaseInfo(uint64 pairId, uint64 pairIndex, Entity collider1, Entity collider2, CollisionShape* shape1, CollisionShape* shape2,
                                                            const Transform& shape1Transform, const Transform& shape2Transform,
                                                            bool needToReportContacts, MemoryAllocator& shapeAllocator) {

    NarrowPhaseInfoBatch::addNarrowPhaseInfo(pairId, pairIndex, collider1, collider2, shape1, shape2, shape1Transform,
                                             shape2Transform, needToReportContacts, shapeAllocator);

    bool isSphereShape1 = shape1->getType() == CollisionShapeType::SPHERE;
    isSpheresShape1.add(isSphereShape1);

    assert(isSphereShape1 || shape1->getType() == CollisionShapeType::CAPSULE);

    // Get the collision shapes
    const SphereShape* sphereShape = static_cast<const SphereShape*>(isSphereShape1 ? shape1 : shape2);
    const CapsuleShape* capsuleShape = static_cast<const CapsuleShape*>(isSphereShape1 ? shape2 : shape1);

    sphereRadiuses.add(sphereShape->getRadius());
    capsuleRadiuses.add(capsuleShape->getRadius());
    capsuleHeights.add(capsuleShape->getHeight());
}

// Initialize the containers using cached capacity
void SphereVsCapsuleNarrowPhaseInfoBatch::reserveMemory() {

    NarrowPhaseInfoBatch::reserveMemory();

    isSpheresShape1.reserve(mCachedCapacity);
    sphereRadiuses.reserve(mCachedCapacity);
    capsuleRadiuses.reserve(mCachedCapacity);
    capsuleHeights.reserve(mCachedCapacity);
}

// Clear all the objects in the batch
void SphereVsCapsuleNarrowPhaseInfoBatch::clear() {

    // Note that we clear the following containers and we release their allocated memory. Therefore,
    // if the memory allocator is a single frame allocator, the memory is deallocated and will be
    // allocated in the next frame at a possibly different location in memory (remember that the
    // location of the allocated memory of a single frame allocator might change between two frames)

    NarrowPhaseInfoBatch::clear();

    isSpheresShape1.clear(true);
    sphereRadiuses.clear(true);
    capsuleRadiuses.clear(true);
    capsuleHeights.clear(true);
}
