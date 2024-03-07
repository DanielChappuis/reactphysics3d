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

#ifndef REACTPHYSICS3D_NARROW_PHASE_ALGORITHM_H
#define REACTPHYSICS3D_NARROW_PHASE_ALGORITHM_H

// Libraries
#include <reactphysics3d/configuration.h>

/// Namespace ReactPhysics3D
namespace reactphysics3d {

class CollisionDetectionSystem;
struct ContactManifoldInfo;
class DefaultPoolAllocator;
class OverlappingPair;
struct NarrowPhaseInfoBatch;
struct ContactPointInfo;
class Profiler;
class MemoryAllocator;

// Class NarrowPhaseCallback
/**
 * This abstract class is the base class for a narrow-phase collision
 * callback class.
 */
class NarrowPhaseCallback {

    public:

        virtual ~NarrowPhaseCallback() = default;

        /// Called by a narrow-phase collision algorithm when a new contact has been found
        virtual void notifyContact(OverlappingPair* overlappingPair,
                                   const ContactPointInfo& contactInfo)=0;

};

// Class NarrowPhaseAlgorithm
/**
 * This abstract class is the base class for a  narrow-phase collision
 * detection algorithm. The goal of the narrow phase algorithm is to
 * compute information about the contact between two colliders.
 */
class NarrowPhaseAlgorithm {

    protected :

        // -------------------- Attributes -------------------- //

#ifdef IS_RP3D_PROFILING_ENABLED

		/// Pointer to the profiler
		Profiler* mProfiler;

#endif

    public :


        // -------------------- Methods -------------------- //

        /// Constructor
        NarrowPhaseAlgorithm() = default;

        /// Destructor
        virtual ~NarrowPhaseAlgorithm() = default;

        /// Deleted copy-constructor
        NarrowPhaseAlgorithm(const NarrowPhaseAlgorithm& algorithm) = delete;

        /// Deleted assignment operator
        NarrowPhaseAlgorithm& operator=(const NarrowPhaseAlgorithm& algorithm) = delete;

#ifdef IS_RP3D_PROFILING_ENABLED

		/// Set the profiler
		void setProfiler(Profiler* profiler);

#endif

};

#ifdef IS_RP3D_PROFILING_ENABLED

// Set the profiler
RP3D_FORCE_INLINE void NarrowPhaseAlgorithm::setProfiler(Profiler* profiler) {
	mProfiler = profiler;
}

#endif

}

#endif


