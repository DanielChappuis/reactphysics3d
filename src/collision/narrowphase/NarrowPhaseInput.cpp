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
#include "collision/narrowphase/NarrowPhaseInput.h"
#include "collision/narrowphase/CollisionDispatch.h"

using namespace reactphysics3d;

/// Constructor
NarrowPhaseInput::NarrowPhaseInput(MemoryAllocator& allocator)
    :mSphereVsSphereBatch(allocator), mSphereVsCapsuleBatch(allocator), mCapsuleVsCapsuleBatch(allocator),
     mSphereVsConvexPolyhedronBatch(allocator), mCapsuleVsConvexPolyhedronBatch(allocator),
     mConvexPolyhedronVsConvexPolyhedronBatch(allocator) {

}

// Add shapes to be tested during narrow-phase collision detection into the batch
void NarrowPhaseInput::addNarrowPhaseTest(OverlappingPair* pair, CollisionShape* shape1, CollisionShape* shape2,
                                          const Transform& shape1Transform, const Transform& shape2Transform,
                                          NarrowPhaseAlgorithmType narrowPhaseAlgorithmType, MemoryAllocator& shapeAllocator) {

    switch (narrowPhaseAlgorithmType) {
        case NarrowPhaseAlgorithmType::SphereVsSphere:
            mSphereVsSphereBatch.addNarrowPhaseInfo(pair, shape1, shape2, shape1Transform, shape2Transform, shapeAllocator);
            break;
        case NarrowPhaseAlgorithmType::SphereVsCapsule:
            mSphereVsCapsuleBatch.addNarrowPhaseInfo(pair, shape1, shape2, shape1Transform, shape2Transform, shapeAllocator);
            break;
        case NarrowPhaseAlgorithmType::CapsuleVsCapsule:
            mCapsuleVsCapsuleBatch.addNarrowPhaseInfo(pair, shape1, shape2, shape1Transform, shape2Transform, shapeAllocator);
            break;
        case NarrowPhaseAlgorithmType::SphereVsConvexPolyhedron:
            mSphereVsConvexPolyhedronBatch.addNarrowPhaseInfo(pair, shape1, shape2, shape1Transform, shape2Transform, shapeAllocator);
            break;
        case NarrowPhaseAlgorithmType::CapsuleVsConvexPolyhedron:
            mCapsuleVsConvexPolyhedronBatch.addNarrowPhaseInfo(pair, shape1, shape2, shape1Transform, shape2Transform, shapeAllocator);
            break;
        case NarrowPhaseAlgorithmType::ConvexPolyhedronVsConvexPolyhedron:
            mConvexPolyhedronVsConvexPolyhedronBatch.addNarrowPhaseInfo(pair, shape1, shape2, shape1Transform, shape2Transform, shapeAllocator);
            break;
        case NarrowPhaseAlgorithmType::None:
            // Must never happen
            assert(false);
            break;
    }
}

// Clear
void NarrowPhaseInput::clear() {

    mSphereVsSphereBatch.clear();
    mSphereVsCapsuleBatch.clear();
    mCapsuleVsCapsuleBatch.clear();
    mSphereVsConvexPolyhedronBatch.clear();
    mCapsuleVsConvexPolyhedronBatch.clear();
    mConvexPolyhedronVsConvexPolyhedronBatch.clear();
}
