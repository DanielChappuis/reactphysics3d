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

#ifndef REACTPHYSICS3D_COLLISION_SHAPE_INFO_H
#define REACTPHYSICS3D_COLLISION_SHAPE_INFO_H

// Libraries
#include "shapes/CollisionShape.h"

/// Namespace ReactPhysics3D
namespace reactphysics3d {

class OverlappingPair;

// Class CollisionShapeInfo
/**
 * This structure regroups different things about a collision shape. This is
 * used to pass information about a collision shape to a collision algorithm.
 */
struct CollisionShapeInfo {

    public:

        /// Broadphase overlapping pair
        OverlappingPair* overlappingPair;

        /// Proxy shape
        ProxyShape* proxyShape;

        /// Pointer to the collision shape
        const CollisionShape* collisionShape;

        /// Transform that maps from collision shape local-space to world-space
        Transform shapeToWorldTransform;

        /// Cached collision data of the proxy shape
        void** cachedCollisionData;

        /// Constructor
        CollisionShapeInfo(ProxyShape* proxyCollisionShape, const CollisionShape* shape,
                           const Transform& shapeLocalToWorldTransform, OverlappingPair* pair,
                           void** cachedData)
              : overlappingPair(pair), proxyShape(proxyCollisionShape), collisionShape(shape),
                shapeToWorldTransform(shapeLocalToWorldTransform),
                cachedCollisionData(cachedData) {

        }
};

}

#endif

