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

#ifndef REACTPHYSICS3D_CONVEX_POLYHEDRON_VS_CONVEX_POLYHEDRON_ALGORITHM_H
#define	REACTPHYSICS3D_CONVEX_POLYHEDRON_VS_CONVEX_POLYHEDRON_ALGORITHM_H

// Libraries
#include "NarrowPhaseAlgorithm.h"

/// Namespace ReactPhysics3D
namespace reactphysics3d {

// Declarations
class Body;
class ContactPoint;

// Class ConvexPolyhedronVsConvexPolyhedronAlgorithm
/**
 * This class is used to compute the narrow-phase collision detection
 * between two convex polyhedra. Here we do not use the GJK algorithm but
 * we run the SAT algorithm to get the contact points and normal.
 * This is based on the "Robust Contact Creation for Physics Simulation"
 * presentation by Dirk Gregorius.
 */
class ConvexPolyhedronVsConvexPolyhedronAlgorithm : public NarrowPhaseAlgorithm {

    protected :

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        ConvexPolyhedronVsConvexPolyhedronAlgorithm() = default;

        /// Destructor
        virtual ~ConvexPolyhedronVsConvexPolyhedronAlgorithm() override = default;

        /// Deleted copy-constructor
        ConvexPolyhedronVsConvexPolyhedronAlgorithm(const ConvexPolyhedronVsConvexPolyhedronAlgorithm& algorithm) = delete;

        /// Deleted assignment operator
        ConvexPolyhedronVsConvexPolyhedronAlgorithm& operator=(const ConvexPolyhedronVsConvexPolyhedronAlgorithm& algorithm) = delete;

        /// Compute the narrow-phase collision detection between two convex polyhedra
        virtual bool testCollision(NarrowPhaseInfo* narrowPhaseInfo, bool reportContacts, MemoryAllocator& memoryAllocator) override;
};

}

#endif

