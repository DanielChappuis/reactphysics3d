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

#ifndef REACTPHYSICS3D_DEFAULT_COLLISION_DISPATCH_H
#define	REACTPHYSICS3D_DEFAULT_COLLISION_DISPATCH_H

// Libraries
#include "CollisionDispatch.h"
#include "SphereVsSphereAlgorithm.h"
#include "SphereVsConvexPolyhedronAlgorithm.h"
#include "SphereVsCapsuleAlgorithm.h"
#include "CapsuleVsCapsuleAlgorithm.h"
#include "CapsuleVsConvexPolyhedronAlgorithm.h"
#include "ConvexPolyhedronVsConvexPolyhedronAlgorithm.h"

namespace reactphysics3d {

// Class DefaultCollisionDispatch
/**
 * This is the default collision dispatch configuration use in ReactPhysics3D.
 * Collision dispatching decides which collision
 * algorithm to use given two types of proxy collision shapes.
 */
class DefaultCollisionDispatch : public CollisionDispatch {

    protected:

        /// Sphere vs Sphere collision algorithm
        SphereVsSphereAlgorithm mSphereVsSphereAlgorithm;

        /// Capsule vs Capsule collision algorithm
        CapsuleVsCapsuleAlgorithm mCapsuleVsCapsuleAlgorithm;

        /// Sphere vs Capsule collision algorithm
        SphereVsCapsuleAlgorithm mSphereVsCapsuleAlgorithm;

        /// Sphere vs Convex Polyhedron collision algorithm
        SphereVsConvexPolyhedronAlgorithm mSphereVsConvexPolyhedronAlgorithm;

        /// Capsule vs Convex Polyhedron collision algorithm
        CapsuleVsConvexPolyhedronAlgorithm mCapsuleVsConvexPolyhedronAlgorithm;

        /// Convex Polyhedron vs Convex Polyhedron collision algorithm
        ConvexPolyhedronVsConvexPolyhedronAlgorithm mConvexPolyhedronVsConvexPolyhedronAlgorithm;

    public:

        /// Constructor
        DefaultCollisionDispatch() = default;

        /// Destructor
        virtual ~DefaultCollisionDispatch() override = default;

        /// Select and return the narrow-phase collision detection algorithm to
        /// use between two types of collision shapes.
        virtual NarrowPhaseAlgorithm* selectAlgorithm(int type1, int type2) override;

#ifdef IS_PROFILING_ACTIVE

		/// Set the profiler
		virtual void setProfiler(Profiler* profiler) override;

#endif

};

#ifdef IS_PROFILING_ACTIVE

// Set the profiler
inline void DefaultCollisionDispatch::setProfiler(Profiler* profiler) {

	CollisionDispatch::setProfiler(profiler);

	mSphereVsSphereAlgorithm.setProfiler(profiler);
	mCapsuleVsCapsuleAlgorithm.setProfiler(profiler);
	mSphereVsCapsuleAlgorithm.setProfiler(profiler);
	mSphereVsConvexPolyhedronAlgorithm.setProfiler(profiler);
	mCapsuleVsConvexPolyhedronAlgorithm.setProfiler(profiler);
	mConvexPolyhedronVsConvexPolyhedronAlgorithm.setProfiler(profiler);
}

#endif

}

#endif



