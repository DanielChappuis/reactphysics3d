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

#ifndef REACTPHYSICS3D_GJK_ALGORITHM_H
#define REACTPHYSICS3D_GJK_ALGORITHM_H

// Libraries
#include <reactphysics3d/decimal.h>
#include <reactphysics3d/configuration.h>

/// ReactPhysics3D namespace
namespace reactphysics3d {

// Declarations
class ContactManifoldInfo;
struct NarrowPhaseInfoBatch;
class ConvexShape;
class Profiler;
class VoronoiSimplex;
template<typename T> class List;

// Constants
constexpr decimal REL_ERROR = decimal(1.0e-3);
constexpr decimal REL_ERROR_SQUARE = REL_ERROR * REL_ERROR;
constexpr int MAX_ITERATIONS_GJK_RAYCAST = 32;

// Class GJKAlgorithm
/**
 * This class implements a narrow-phase collision detection algorithm. This
 * algorithm uses the ISA-GJK algorithm. This
 * implementation is based on the implementation discussed in the book
 * "Collision Detection in Interactive 3D Environments" by Gino van den Bergen.
 * This method implements the Hybrid Technique for calculating the
 * penetration depth. The two objects are enlarged with a small margin. If
 * the object intersects in their margins, the penetration depth is quickly
 * computed using the GJK algorithm on the original objects (without margin).
 * If the original objects (without margin) intersect, we exit GJK and run
 * the SAT algorithm to get contacts and collision data.
 */
class GJKAlgorithm {

    private :

        // -------------------- Attributes -------------------- //

#ifdef IS_RP3D_PROFILING_ENABLED

		/// Pointer to the profiler
		Profiler* mProfiler;

#endif

        // -------------------- Methods -------------------- //

    public :

        enum class GJKResult {
            SEPARATED,              // The two shapes are separated outside the margin
            COLLIDE_IN_MARGIN,      // The two shapes overlap only in the margin (shallow penetration)
            INTERPENETRATE          // The two shapes overlap event without the margin (deep penetration)
        };

        // -------------------- Methods -------------------- //

        /// Constructor
        GJKAlgorithm() = default;

        /// Destructor
        ~GJKAlgorithm() = default;

        /// Deleted copy-constructor
        GJKAlgorithm(const GJKAlgorithm& algorithm) = delete;

        /// Deleted assignment operator
        GJKAlgorithm& operator=(const GJKAlgorithm& algorithm) = delete;

        /// Compute a contact info if the two bounding volumes collide.
        void testCollision(NarrowPhaseInfoBatch& narrowPhaseInfoBatch, uint batchStartIndex,
                           uint batchNbItems, List<GJKResult>& gjkResults);

#ifdef IS_RP3D_PROFILING_ENABLED

		/// Set the profiler
		void setProfiler(Profiler* profiler);

#endif

};

#ifdef IS_RP3D_PROFILING_ENABLED

// Set the profiler
inline void GJKAlgorithm::setProfiler(Profiler* profiler) {
	mProfiler = profiler;
}

#endif

}

#endif
