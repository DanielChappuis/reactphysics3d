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

#ifndef REACTPHYSICS3D_SPHERE_VS_SPHERE_ALGORITHM_H
#define	REACTPHYSICS3D_SPHERE_VS_SPHERE_ALGORITHM_H

// Libraries
#include "reactphysics3d/body/Body.h"
#include "reactphysics3d/collision/narrowphase/NarrowPhaseAlgorithm.h"
#include "reactphysics3d/constraint/ContactPoint.h"

/// ReactPhysics3D Namespace
namespace reactphysics3d {

// Class SphereVsSphereAlgorithm
/**
 * This class is used to compute the narrow-phase collision detection
 * between two sphere collision shapes.
 */
class SphereVsSphereAlgorithm : public NarrowPhaseAlgorithm {

    protected :

        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        SphereVsSphereAlgorithm(const SphereVsSphereAlgorithm& algorithm);

        /// Private assignment operator
        SphereVsSphereAlgorithm& operator=(const SphereVsSphereAlgorithm& algorithm);
        
    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        SphereVsSphereAlgorithm(MemoryAllocator& memoryAllocator);

        /// Destructor
        virtual ~SphereVsSphereAlgorithm();

        /// Return true and compute a contact info if the two bounding volume collide
        virtual bool testCollision(ProxyShape* collisionShape1, ProxyShape* collisionShape2,
                                   ContactPointInfo*& contactInfo);
};

}

#endif

