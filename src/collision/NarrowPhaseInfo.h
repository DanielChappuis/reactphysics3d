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

#ifndef REACTPHYSICS3D_NARROW_PHASE_INFO_H
#define REACTPHYSICS3D_NARROW_PHASE_INFO_H

// Libraries
#include "shapes/CollisionShape.h"

/// Namespace ReactPhysics3D
namespace reactphysics3d {

class OverlappingPair;

// Class NarrowPhaseInfo
/**
 * This structure regroups different things about a collision shape. This is
 * used to pass information about a collision shape to a collision algorithm.
 */
struct NarrowPhaseInfo {

    public:

        /// Broadphase overlapping pair
        OverlappingPair* overlappingPair;

        /// Pointer to the first collision shape to test collision with
        const CollisionShape* collisionShape1;

        /// Pointer to the second collision shape to test collision with
        const CollisionShape* collisionShape2;

        /// Transform that maps from collision shape 1 local-space to world-space
        Transform shape1ToWorldTransform;

        /// Transform that maps from collision shape 2 local-space to world-space
        Transform shape2ToWorldTransform;

        /// Cached collision data of the proxy shape
        // TODO : Check if we can use separating axis in OverlappingPair instead of cachedCollisionData1 and cachedCollisionData2
        void** cachedCollisionData1;

        /// Cached collision data of the proxy shape
        // TODO : Check if we can use separating axis in OverlappingPair instead of cachedCollisionData1 and cachedCollisionData2
        void** cachedCollisionData2;

        /// Pointer to the next element in the linked list
        NarrowPhaseInfo* next;

        /// Constructor
        NarrowPhaseInfo(OverlappingPair* pair, const CollisionShape* shape1,
                        const CollisionShape* shape2, const Transform& shape1Transform,
                        const Transform& shape2Transform, void** cachedData1, void** cachedData2)
              : overlappingPair(pair), collisionShape1(shape1), collisionShape2(shape2),
                shape1ToWorldTransform(shape1Transform), shape2ToWorldTransform(shape2Transform),
                cachedCollisionData1(cachedData1), cachedCollisionData2(cachedData2), next(nullptr) {

        }
};

}

#endif

