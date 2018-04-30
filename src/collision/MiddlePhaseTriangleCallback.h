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

#ifndef REACTPHYSICS3D_MIDDLE_PHASE_TRIANGLE_CALLBACK_H
#define REACTPHYSICS3D_MIDDLE_PHASE_TRIANGLE_CALLBACK_H

#include "configuration.h"
#include "collision/shapes/ConcaveShape.h"

/// Namespace ReactPhysics3D
namespace reactphysics3d {

// Libraries

// Declarations
class ConcaveShape;
class OverlappingPair;
class NarrowPhaseAlgorithm;
class ProxyShape;
class MemoryAllocator;
class Profiler;
struct NarrowPhaseInfo;
struct Vector3;

// Class ConvexVsTriangleCallback
/**
 * This class is used to report a collision between the triangle
 * of a concave mesh shape and a convex shape during the
 * middle-phase algorithm.
 */
class MiddlePhaseTriangleCallback : public TriangleCallback {

    protected:

        /// Broadphase overlapping pair
        OverlappingPair* mOverlappingPair;

        /// Pointer to the concave proxy shape
        ProxyShape* mConcaveProxyShape;

        /// Pointer to the convex proxy shape
        ProxyShape* mConvexProxyShape;

        /// Pointer to the concave collision shape
        const ConcaveShape* mConcaveShape;

        /// Reference to the single-frame memory allocator
        MemoryAllocator& mAllocator;

#ifdef IS_PROFILING_ACTIVE

		/// Pointer to the profiler
		Profiler* mProfiler;

#endif

    public:

        /// Pointer to the first element of the linked-list of narrow-phase info
        NarrowPhaseInfo* narrowPhaseInfoList;

        /// Constructor
        MiddlePhaseTriangleCallback(OverlappingPair* overlappingPair,
                                    ProxyShape* concaveProxyShape,
                                    ProxyShape* convexProxyShape, const ConcaveShape* concaveShape,
                                    MemoryAllocator& allocator)
            :mOverlappingPair(overlappingPair), mConcaveProxyShape(concaveProxyShape),
             mConvexProxyShape(convexProxyShape), mConcaveShape(concaveShape),
             mAllocator(allocator), narrowPhaseInfoList(nullptr) {

        }

        /// Test collision between a triangle and the convex mesh shape
        virtual void testTriangle(const Vector3* trianglePoints, const Vector3* verticesNormals, uint shapeId) override;

#ifdef IS_PROFILING_ACTIVE

		/// Set the profiler
		void setProfiler(Profiler* profiler) {
			mProfiler = profiler;
		}

#endif

};

}

#endif
