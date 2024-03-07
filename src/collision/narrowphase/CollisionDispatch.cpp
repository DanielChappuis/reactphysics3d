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
#include <reactphysics3d/collision/narrowphase/CollisionDispatch.h>

using namespace reactphysics3d;

// Constructor
CollisionDispatch::CollisionDispatch(MemoryAllocator& allocator) : mAllocator(allocator) {

    // Make sure to allocate memory with size that is multiple integral of alignment
    mSphereVsSphereAllocatedSize = std::ceil(sizeof(SphereVsSphereAlgorithm) / float(GLOBAL_ALIGNMENT)) * GLOBAL_ALIGNMENT;
    mSphereVsCapsuleAllocatedSize = std::ceil(sizeof(SphereVsCapsuleAlgorithm) / float(GLOBAL_ALIGNMENT)) * GLOBAL_ALIGNMENT;
    mCapsuleVsCapsuleAllocatedSize = std::ceil(sizeof(CapsuleVsCapsuleAlgorithm) / float(GLOBAL_ALIGNMENT)) * GLOBAL_ALIGNMENT;
    mSphereVsConvexPolyAllocatedSize = std::ceil(sizeof(SphereVsConvexPolyhedronAlgorithm) / float(GLOBAL_ALIGNMENT)) * GLOBAL_ALIGNMENT;
    mCapsuleVsConvexPolyAllocatedSize = std::ceil(sizeof(CapsuleVsConvexPolyhedronAlgorithm) / float(GLOBAL_ALIGNMENT)) * GLOBAL_ALIGNMENT;
    mConvexPolyVsConvexPolyAllocatedSize = std::ceil(sizeof(ConvexPolyhedronVsConvexPolyhedronAlgorithm) / float(GLOBAL_ALIGNMENT)) * GLOBAL_ALIGNMENT;

    // Create the default narrow-phase algorithms
    mSphereVsSphereAlgorithm = new (allocator.allocate(mSphereVsSphereAllocatedSize)) SphereVsSphereAlgorithm();
    mSphereVsCapsuleAlgorithm = new (allocator.allocate(mSphereVsCapsuleAllocatedSize)) SphereVsCapsuleAlgorithm();
    mCapsuleVsCapsuleAlgorithm = new (allocator.allocate(mCapsuleVsCapsuleAllocatedSize)) CapsuleVsCapsuleAlgorithm();
    mSphereVsConvexPolyhedronAlgorithm = new (allocator.allocate(mSphereVsConvexPolyAllocatedSize)) SphereVsConvexPolyhedronAlgorithm();
    mCapsuleVsConvexPolyhedronAlgorithm = new (allocator.allocate(mCapsuleVsConvexPolyAllocatedSize)) CapsuleVsConvexPolyhedronAlgorithm();
    mConvexPolyhedronVsConvexPolyhedronAlgorithm = new (allocator.allocate(mConvexPolyVsConvexPolyAllocatedSize)) ConvexPolyhedronVsConvexPolyhedronAlgorithm();

    // Fill in the collision matrix
    fillInCollisionMatrix();
}

// Destructor
CollisionDispatch::~CollisionDispatch() {

    // Release allocated memory
    if (mIsSphereVsSphereDefault) {
        mAllocator.release(mSphereVsSphereAlgorithm, mSphereVsSphereAllocatedSize);
    }
    if (mIsSphereVsCapsuleDefault) {
        mAllocator.release(mSphereVsCapsuleAlgorithm, mSphereVsCapsuleAllocatedSize);
    }
    if (mIsCapsuleVsCapsuleDefault) {
        mAllocator.release(mCapsuleVsCapsuleAlgorithm, mCapsuleVsCapsuleAllocatedSize);
    }
    if (mIsSphereVsConvexPolyhedronDefault) {
        mAllocator.release(mSphereVsConvexPolyhedronAlgorithm, mSphereVsConvexPolyAllocatedSize);
    }
    if (mIsCapsuleVsConvexPolyhedronDefault) {
        mAllocator.release(mCapsuleVsConvexPolyhedronAlgorithm, mCapsuleVsConvexPolyAllocatedSize);
    }
    if (mIsConvexPolyhedronVsConvexPolyhedronDefault) {
        mAllocator.release(mConvexPolyhedronVsConvexPolyhedronAlgorithm, mConvexPolyVsConvexPolyAllocatedSize);
    }
}

// Select and return the narrow-phase collision detection algorithm to
// use between two types of collision shapes.
NarrowPhaseAlgorithmType CollisionDispatch::selectAlgorithm(int type1, int type2) {

    CollisionShapeType shape1Type = static_cast<CollisionShapeType>(type1);
    CollisionShapeType shape2Type = static_cast<CollisionShapeType>(type2);

    if (type1 > type2) {
        return NarrowPhaseAlgorithmType::NoCollisionTest;
    }
    // Sphere vs Sphere algorithm
    if (shape1Type == CollisionShapeType::SPHERE && shape2Type == CollisionShapeType::SPHERE) {
        return NarrowPhaseAlgorithmType::SphereVsSphere;
    }
    // Sphere vs Capsule algorithm
    if (shape1Type == CollisionShapeType::SPHERE && shape2Type == CollisionShapeType::CAPSULE) {
        return NarrowPhaseAlgorithmType::SphereVsCapsule;
    }
    // Capsule vs Capsule algorithm
    if (shape1Type == CollisionShapeType::CAPSULE && shape2Type == CollisionShapeType::CAPSULE) {
        return NarrowPhaseAlgorithmType::CapsuleVsCapsule;
    }
    // Sphere vs Convex Polyhedron algorithm
    if (shape1Type == CollisionShapeType::SPHERE && shape2Type == CollisionShapeType::CONVEX_POLYHEDRON) {
        return NarrowPhaseAlgorithmType::SphereVsConvexPolyhedron;
    }
    // Capsule vs Convex Polyhedron algorithm
    if (shape1Type == CollisionShapeType::CAPSULE && shape2Type == CollisionShapeType::CONVEX_POLYHEDRON) {
        return NarrowPhaseAlgorithmType::CapsuleVsConvexPolyhedron;
    }
    // Convex Polyhedron vs Convex Polyhedron algorithm
    if (shape1Type == CollisionShapeType::CONVEX_POLYHEDRON &&
        shape2Type == CollisionShapeType::CONVEX_POLYHEDRON) {
        return NarrowPhaseAlgorithmType::ConvexPolyhedronVsConvexPolyhedron;
    }

    return NarrowPhaseAlgorithmType::NoCollisionTest;
}

// Set the Sphere vs Sphere narrow-phase collision detection algorithm
void CollisionDispatch::setSphereVsSphereAlgorithm(SphereVsSphereAlgorithm* algorithm) {

    if (mIsSphereVsSphereDefault) {
        mAllocator.release(mSphereVsSphereAlgorithm, sizeof(SphereVsSphereAlgorithm));
        mIsSphereVsSphereDefault = false;
    }

    mSphereVsSphereAlgorithm = algorithm;

    fillInCollisionMatrix();
}

// Set the Sphere vs Capsule narrow-phase collision detection algorithm
void CollisionDispatch::setSphereVsCapsuleAlgorithm(SphereVsCapsuleAlgorithm* algorithm) {

    if (mIsSphereVsCapsuleDefault) {
        mAllocator.release(mSphereVsCapsuleAlgorithm, sizeof(SphereVsCapsuleAlgorithm));
        mIsSphereVsCapsuleDefault = false;
    }

    mSphereVsCapsuleAlgorithm = algorithm;

    fillInCollisionMatrix();
}

// Set the Capsule vs Capsule narrow-phase collision detection algorithm
void CollisionDispatch::setCapsuleVsCapsuleAlgorithm(CapsuleVsCapsuleAlgorithm* algorithm) {

    if (mIsCapsuleVsCapsuleDefault) {
        mAllocator.release(mCapsuleVsCapsuleAlgorithm, sizeof(CapsuleVsCapsuleAlgorithm));
        mIsCapsuleVsCapsuleDefault = false;
    }

    mCapsuleVsCapsuleAlgorithm = algorithm;

    fillInCollisionMatrix();
}

// Set the Sphere vs Convex Polyhedron narrow-phase collision detection algorithm
void CollisionDispatch::setSphereVsConvexPolyhedronAlgorithm(SphereVsConvexPolyhedronAlgorithm* algorithm) {

    if (mIsSphereVsConvexPolyhedronDefault) {
        mAllocator.release(mSphereVsConvexPolyhedronAlgorithm, sizeof(SphereVsConvexPolyhedronAlgorithm));
        mIsSphereVsConvexPolyhedronDefault = false;
    }

    mSphereVsConvexPolyhedronAlgorithm = algorithm;

    fillInCollisionMatrix();
}

// Set the Capsule vs Convex Polyhedron narrow-phase collision detection algorithm
void CollisionDispatch::setCapsuleVsConvexPolyhedronAlgorithm(CapsuleVsConvexPolyhedronAlgorithm* algorithm) {

    if (mIsCapsuleVsConvexPolyhedronDefault) {
        mAllocator.release(mCapsuleVsConvexPolyhedronAlgorithm, sizeof(CapsuleVsConvexPolyhedronAlgorithm));
        mIsCapsuleVsConvexPolyhedronDefault = false;
    }

    mCapsuleVsConvexPolyhedronAlgorithm = algorithm;

    fillInCollisionMatrix();
}

// Set the Convex Polyhedron vs Convex Polyhedron narrow-phase collision detection algorithm
void CollisionDispatch::setConvexPolyhedronVsConvexPolyhedronAlgorithm(ConvexPolyhedronVsConvexPolyhedronAlgorithm* algorithm) {

    if (mIsConvexPolyhedronVsConvexPolyhedronDefault) {
        mAllocator.release(mConvexPolyhedronVsConvexPolyhedronAlgorithm, sizeof(ConvexPolyhedronVsConvexPolyhedronAlgorithm));
        mIsConvexPolyhedronVsConvexPolyhedronDefault = false;
    }

    mConvexPolyhedronVsConvexPolyhedronAlgorithm = algorithm;

    fillInCollisionMatrix();
}


// Fill-in the collision detection matrix
void CollisionDispatch::fillInCollisionMatrix() {

    // For each possible type of collision shape
    for (int i=0; i<NB_COLLISION_SHAPE_TYPES; i++) {
        for (int j=0; j<NB_COLLISION_SHAPE_TYPES; j++) {
            mCollisionMatrix[i][j] = selectAlgorithm(i, j);
        }
    }
}

// Return the corresponding narrow-phase algorithm type to use for two collision shapes
NarrowPhaseAlgorithmType CollisionDispatch::selectNarrowPhaseAlgorithm(const CollisionShapeType& shape1Type,
                                                                       const CollisionShapeType& shape2Type) const {

    RP3D_PROFILE("CollisionDispatch::selectNarrowPhaseAlgorithm()", mProfiler);

    uint32 shape1Index = static_cast<uint32>(shape1Type);
    uint shape2Index = static_cast<uint32>(shape2Type);

    // Swap the shape types if necessary
    if (shape1Index > shape2Index) {
        return mCollisionMatrix[shape2Index][shape1Index];
    }

    return mCollisionMatrix[shape1Index][shape2Index];
}



