/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2016 Daniel Chappuis                                       *
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
#include "NarrowPhaseAlgorithm.h"

namespace reactphysics3d {

// Class CollisionDispatch
/**
 * This is the abstract base class for dispatching the narrow-phase
 * collision detection algorithm. Collision dispatching decides which collision
 * algorithm to use given two types of proxy collision shapes.
 */
class CollisionDispatch {

    protected:

    public:

        /// Constructor
        CollisionDispatch() {}

        /// Destructor
        virtual ~CollisionDispatch() {}

        /// Initialize the collision dispatch configuration
        virtual void init(CollisionDetection* collisionDetection,
                          MemoryAllocator* memoryAllocator) {

        }

        /// Select and return the narrow-phase collision detection algorithm to
        /// use between two types of collision shapes.
        virtual NarrowPhaseAlgorithm* selectAlgorithm(int shape1Type,
                                                      int shape2Type)=0;
};

}

#endif

