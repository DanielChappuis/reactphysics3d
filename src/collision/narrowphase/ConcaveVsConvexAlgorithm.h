/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2015 Daniel Chappuis                                       *
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

#ifndef REACTPHYSICS3D_CONCAVE_VS_CONVEX_ALGORITHM_H
#define	REACTPHYSICS3D_CONCAVE_VS_CONVEX_ALGORITHM_H

// Libraries
#include "NarrowPhaseAlgorithm.h"

/// Namespace ReactPhysics3D
namespace reactphysics3d {

// Class ConcaveVsConvexAlgorithm
/**
 * This class is used to compute the narrow-phase collision detection
 * between a concave collision shape and a convex collision shape. The idea is
 * to use the GJK collision detection algorithm to compute the collision between
 * the convex shape and each of the triangles in the concave shape.
 */
class ConcaveVsConvexAlgorithm : public NarrowPhaseAlgorithm {

    protected :

        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        ConcaveVsConvexAlgorithm(const ConcaveVsConvexAlgorithm& algorithm);

        /// Private assignment operator
        ConcaveVsConvexAlgorithm& operator=(const ConcaveVsConvexAlgorithm& algorithm);

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        ConcaveVsConvexAlgorithm();

        /// Destructor
        virtual ~ConcaveVsConvexAlgorithm();

        /// Return true and compute a contact info if the two bounding volume collide
        virtual bool testCollision(ProxyShape* collisionShape1, ProxyShape* collisionShape2,
                                   ContactPointInfo*& contactInfo);
};

}

#endif

