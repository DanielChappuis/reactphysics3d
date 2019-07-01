/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2019 Daniel Chappuis                                       *
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

#ifndef REACTPHYSICS3D_SPHERE_VS_CAPSULE_ALGORITHM_H
#define	REACTPHYSICS3D_SPHERE_VS_CAPSULE_ALGORITHM_H

// Libraries
#include "NarrowPhaseAlgorithm.h"


/// Namespace ReactPhysics3D
namespace reactphysics3d {

// Declarations
class Body;
class ContactPoint;

// Class SphereVsCapsuleAlgorithm
/**
 * This class is used to compute the narrow-phase collision detection
 * between a sphere collision shape and a capsule collision shape.
 * For this case, we do not use GJK or SAT algorithm. We directly compute the
 * contact points and contact normal. This is based on the "Robust Contact
 * Creation for Physics Simulation" presentation by Dirk Gregorius.
 */
class SphereVsCapsuleAlgorithm : public NarrowPhaseAlgorithm {

    protected :

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
		SphereVsCapsuleAlgorithm() = default;

        /// Destructor
        virtual ~SphereVsCapsuleAlgorithm() override = default;

        /// Deleted copy-constructor
		SphereVsCapsuleAlgorithm(const SphereVsCapsuleAlgorithm& algorithm) = delete;

        /// Deleted assignment operator
		SphereVsCapsuleAlgorithm& operator=(const SphereVsCapsuleAlgorithm& algorithm) = delete;

        /// Compute the narrow-phase collision detection between a sphere and a capsule
        virtual bool testCollision(NarrowPhaseInfo* narrowPhaseInfo, bool reportContacts, MemoryAllocator& memoryAllocator) override;
};

}

#endif

