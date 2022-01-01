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

#ifndef REACTPHYSICS3D_COLLISION_DISPATCH_H
#define	REACTPHYSICS3D_COLLISION_DISPATCH_H

// Libraries
#include <reactphysics3d/collision/narrowphase/SphereVsSphereAlgorithm.h>
#include <reactphysics3d/collision/narrowphase/SphereVsConvexPolyhedronAlgorithm.h>
#include <reactphysics3d/collision/narrowphase/SphereVsCapsuleAlgorithm.h>
#include <reactphysics3d/collision/narrowphase/CapsuleVsCapsuleAlgorithm.h>
#include <reactphysics3d/collision/narrowphase/CapsuleVsConvexPolyhedronAlgorithm.h>
#include <reactphysics3d/collision/narrowphase/ConvexPolyhedronVsConvexPolyhedronAlgorithm.h>
#include <reactphysics3d/collision/shapes/CollisionShape.h>

namespace reactphysics3d {

/// Enumeration for the type of narrow-phase
/// collision detection algorithm
enum class NarrowPhaseAlgorithmType {
    None,
    SphereVsSphere,
    SphereVsCapsule,
    CapsuleVsCapsule,
    SphereVsConvexPolyhedron,
    CapsuleVsConvexPolyhedron,
    ConvexPolyhedronVsConvexPolyhedron
};

// Class CollisionDispatch
/**
 * This is the collision dispatch configuration use in ReactPhysics3D.
 * Collision dispatching decides which collision
 * algorithm to use given two types of colliders.
 */
class CollisionDispatch {

    protected:

        /// Memory allocator
        MemoryAllocator& mAllocator;

        /// True if the sphere vs sphere algorithm is the default one
        bool mIsSphereVsSphereDefault = true;

        /// True if the capsule vs capsule algorithm is the default one
        bool mIsCapsuleVsCapsuleDefault = true;

        /// True if the sphere vs capsule algorithm is the default one
        bool mIsSphereVsCapsuleDefault = true;

        /// True if the sphere vs convex polyhedron algorithm is the default one
        bool mIsSphereVsConvexPolyhedronDefault = true;

        /// True if the capsule vs convex polyhedron algorithm is the default one
        bool mIsCapsuleVsConvexPolyhedronDefault = true;

        /// True if the convex polyhedron vs convex polyhedron algorithm is the default one
        bool mIsConvexPolyhedronVsConvexPolyhedronDefault = true;

        /// Sphere vs Sphere collision algorithm
        SphereVsSphereAlgorithm* mSphereVsSphereAlgorithm;

        /// Capsule vs Capsule collision algorithm
        CapsuleVsCapsuleAlgorithm* mCapsuleVsCapsuleAlgorithm;

        /// Sphere vs Capsule collision algorithm
        SphereVsCapsuleAlgorithm* mSphereVsCapsuleAlgorithm;

        /// Sphere vs Convex Polyhedron collision algorithm
        SphereVsConvexPolyhedronAlgorithm* mSphereVsConvexPolyhedronAlgorithm;

        /// Capsule vs Convex Polyhedron collision algorithm
        CapsuleVsConvexPolyhedronAlgorithm* mCapsuleVsConvexPolyhedronAlgorithm;

        /// Convex Polyhedron vs Convex Polyhedron collision algorithm
        ConvexPolyhedronVsConvexPolyhedronAlgorithm* mConvexPolyhedronVsConvexPolyhedronAlgorithm;

        /// Collision detection matrix (algorithms to use)
        NarrowPhaseAlgorithmType mCollisionMatrix[NB_COLLISION_SHAPE_TYPES][NB_COLLISION_SHAPE_TYPES];

        /// Select and return the narrow-phase collision detection algorithm to
        /// use between two types of collision shapes.
        NarrowPhaseAlgorithmType selectAlgorithm(int type1, int type2);

#ifdef IS_RP3D_PROFILING_ENABLED

    /// Pointer to the profiler
    Profiler* mProfiler;

#endif

    public:

        /// Constructor
        CollisionDispatch(MemoryAllocator& allocator);

        /// Destructor
        ~CollisionDispatch();

        /// Set the Sphere vs Sphere narrow-phase collision detection algorithm
        void setSphereVsSphereAlgorithm(SphereVsSphereAlgorithm* algorithm);

        /// Get the Sphere vs Sphere narrow-phase collision detection algorithm
        SphereVsSphereAlgorithm* getSphereVsSphereAlgorithm();

        /// Set the Sphere vs Capsule narrow-phase collision detection algorithm
        void setSphereVsCapsuleAlgorithm(SphereVsCapsuleAlgorithm* algorithm);

        /// Get the Sphere vs Capsule narrow-phase collision detection algorithm
        SphereVsCapsuleAlgorithm* getSphereVsCapsuleAlgorithm();

        /// Set the Capsule vs Capsule narrow-phase collision detection algorithm
        void setCapsuleVsCapsuleAlgorithm(CapsuleVsCapsuleAlgorithm* algorithm);

        /// Get the Capsule vs Capsule narrow-phase collision detection algorithm
        CapsuleVsCapsuleAlgorithm* getCapsuleVsCapsuleAlgorithm();

        /// Set the Sphere vs Convex Polyhedron narrow-phase collision detection algorithm
        void setSphereVsConvexPolyhedronAlgorithm(SphereVsConvexPolyhedronAlgorithm* algorithm);

        /// Get the Sphere vs Convex Polyhedron narrow-phase collision detection algorithm
        SphereVsConvexPolyhedronAlgorithm* getSphereVsConvexPolyhedronAlgorithm();

        /// Set the Capsule vs Convex Polyhedron narrow-phase collision detection algorithm
        void setCapsuleVsConvexPolyhedronAlgorithm(CapsuleVsConvexPolyhedronAlgorithm* algorithm);

        /// Get the Capsule vs Convex Polyhedron narrow-phase collision detection algorithm
        CapsuleVsConvexPolyhedronAlgorithm* getCapsuleVsConvexPolyhedronAlgorithm();

        /// Set the Convex Polyhedron vs Convex Polyhedron narrow-phase collision detection algorithm
        void setConvexPolyhedronVsConvexPolyhedronAlgorithm(ConvexPolyhedronVsConvexPolyhedronAlgorithm* algorithm);

        /// Get the Convex Polyhedron vs Convex Polyhedron narrow-phase collision detection algorithm
        ConvexPolyhedronVsConvexPolyhedronAlgorithm* getConvexPolyhedronVsConvexPolyhedronAlgorithm();

        /// Fill-in the collision detection matrix
        void fillInCollisionMatrix();

        /// Return the corresponding narrow-phase algorithm type to use for two collision shapes
        NarrowPhaseAlgorithmType selectNarrowPhaseAlgorithm(const CollisionShapeType& shape1Type,
                                                            const CollisionShapeType& shape2Type) const;

#ifdef IS_RP3D_PROFILING_ENABLED

		/// Set the profiler
        void setProfiler(Profiler* profiler);

#endif

};

// Get the Sphere vs Sphere narrow-phase collision detection algorithm
RP3D_FORCE_INLINE SphereVsSphereAlgorithm* CollisionDispatch::getSphereVsSphereAlgorithm() {
    return mSphereVsSphereAlgorithm;
}

// Get the Sphere vs Capsule narrow-phase collision detection algorithm
RP3D_FORCE_INLINE SphereVsCapsuleAlgorithm* CollisionDispatch::getSphereVsCapsuleAlgorithm() {
    return mSphereVsCapsuleAlgorithm;
}

// Get the Capsule vs Capsule narrow-phase collision detection algorithm
RP3D_FORCE_INLINE CapsuleVsCapsuleAlgorithm* CollisionDispatch::getCapsuleVsCapsuleAlgorithm() {
    return mCapsuleVsCapsuleAlgorithm;
}

// Get the Sphere vs Convex Polyhedron narrow-phase collision detection algorithm
RP3D_FORCE_INLINE SphereVsConvexPolyhedronAlgorithm* CollisionDispatch::getSphereVsConvexPolyhedronAlgorithm() {
    return mSphereVsConvexPolyhedronAlgorithm;
}

// Get the Capsule vs Convex Polyhedron narrow-phase collision detection algorithm
RP3D_FORCE_INLINE CapsuleVsConvexPolyhedronAlgorithm* CollisionDispatch::getCapsuleVsConvexPolyhedronAlgorithm() {
   return mCapsuleVsConvexPolyhedronAlgorithm;
}

// Get the Convex Polyhedron vs Convex Polyhedron narrow-phase collision detection algorithm
RP3D_FORCE_INLINE ConvexPolyhedronVsConvexPolyhedronAlgorithm* CollisionDispatch::getConvexPolyhedronVsConvexPolyhedronAlgorithm() {
    return mConvexPolyhedronVsConvexPolyhedronAlgorithm;
}

#ifdef IS_RP3D_PROFILING_ENABLED

// Set the profiler
RP3D_FORCE_INLINE void CollisionDispatch::setProfiler(Profiler* profiler) {

    mProfiler = profiler;
    mSphereVsSphereAlgorithm->setProfiler(profiler);
    mCapsuleVsCapsuleAlgorithm->setProfiler(profiler);
    mSphereVsCapsuleAlgorithm->setProfiler(profiler);
    mSphereVsConvexPolyhedronAlgorithm->setProfiler(profiler);
    mCapsuleVsConvexPolyhedronAlgorithm->setProfiler(profiler);
    mConvexPolyhedronVsConvexPolyhedronAlgorithm->setProfiler(profiler);
}

#endif

}

#endif



