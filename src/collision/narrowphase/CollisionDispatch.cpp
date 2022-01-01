/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2022 Daniel Chappuis                                       *
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

    // Create the default narrow-phase algorithms
    mSphereVsSphereAlgorithm = new (allocator.allocate(sizeof(SphereVsSphereAlgorithm))) SphereVsSphereAlgorithm();
    mSphereVsCapsuleAlgorithm = new (allocator.allocate(sizeof(SphereVsCapsuleAlgorithm))) SphereVsCapsuleAlgorithm();
    mCapsuleVsCapsuleAlgorithm = new (allocator.allocate(sizeof(CapsuleVsCapsuleAlgorithm))) CapsuleVsCapsuleAlgorithm();
    mSphereVsConvexPolyhedronAlgorithm = new (allocator.allocate(sizeof(SphereVsConvexPolyhedronAlgorithm))) SphereVsConvexPolyhedronAlgorithm();
    mCapsuleVsConvexPolyhedronAlgorithm = new (allocator.allocate(sizeof(CapsuleVsConvexPolyhedronAlgorithm))) CapsuleVsConvexPolyhedronAlgorithm();
    mConvexPolyhedronVsConvexPolyhedronAlgorithm = new (allocator.allocate(sizeof(ConvexPolyhedronVsConvexPolyhedronAlgorithm))) ConvexPolyhedronVsConvexPolyhedronAlgorithm();

    // Fill in the collision matrix
    fillInCollisionMatrix();
}

// Destructor
CollisionDispatch::~CollisionDispatch() {

    // Release allocated memory
    if (mIsSphereVsSphereDefault) {
        mAllocator.release(mSphereVsSphereAlgorithm, sizeof(SphereVsSphereAlgorithm));
    }
    if (mIsSphereVsCapsuleDefault) {
        mAllocator.release(mSphereVsCapsuleAlgorithm, sizeof(SphereVsCapsuleAlgorithm));
    }
    if (mIsCapsuleVsCapsuleDefault) {
        mAllocator.release(mCapsuleVsCapsuleAlgorithm, sizeof(CapsuleVsCapsuleAlgorithm));
    }
    if (mIsSphereVsConvexPolyhedronDefault) {
        mAllocator.release(mSphereVsConvexPolyhedronAlgorithm, sizeof(SphereVsConvexPolyhedronAlgorithm));
    }
    if (mIsCapsuleVsConvexPolyhedronDefault) {
        mAllocator.release(mCapsuleVsConvexPolyhedronAlgorithm, sizeof(CapsuleVsConvexPolyhedronAlgorithm));
    }
    if (mIsConvexPolyhedronVsConvexPolyhedronDefault) {
        mAllocator.release(mConvexPolyhedronVsConvexPolyhedronAlgorithm, sizeof(ConvexPolyhedronVsConvexPolyhedronAlgorithm));
    }
}

// Select and return the narrow-phase collision detection algorithm to
// use between two types of collision shapes.
NarrowPhaseAlgorithmType CollisionDispatch::selectAlgorithm(int type1, int type2) {

    CollisionShapeType shape1Type = static_cast<CollisionShapeType>(type1);
    CollisionShapeType shape2Type = static_cast<CollisionShapeType>(type2);

    if (type1 > type2) {
        return NarrowPhaseAlgorithmType::None;
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

    return NarrowPhaseAlgorithmType::None;
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



