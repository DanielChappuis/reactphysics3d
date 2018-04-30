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

#ifndef REACTPHYSICS3D_NARROW_PHASE_INFO_H
#define REACTPHYSICS3D_NARROW_PHASE_INFO_H

// Libraries
#include "engine/OverlappingPair.h"

/// Namespace ReactPhysics3D
namespace reactphysics3d {

// Declarations
class CollisionShape;
struct LastFrameCollisionInfo;
class ContactManifoldInfo;
struct ContactPointInfo;

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
        CollisionShape* collisionShape1;

        /// Pointer to the second collision shape to test collision with
        CollisionShape* collisionShape2;

        /// Transform that maps from collision shape 1 local-space to world-space
        Transform shape1ToWorldTransform;

        /// Transform that maps from collision shape 2 local-space to world-space
        Transform shape2ToWorldTransform;

        /// Linked-list of contact points created during the narrow-phase
        ContactPointInfo* contactPoints;

        /// Pointer to the next element in the linked list
        NarrowPhaseInfo* next;

        /// Memory allocator for the collision shape (Used to release TriangleShape memory in destructor)
        MemoryAllocator& collisionShapeAllocator;

        /// Constructor
        NarrowPhaseInfo(OverlappingPair* pair, CollisionShape* shape1,
                        CollisionShape* shape2, const Transform& shape1Transform,
                        const Transform& shape2Transform, MemoryAllocator& shapeAllocator);

        /// Destructor
        ~NarrowPhaseInfo();

        /// Add a new contact point
        void addContactPoint(const Vector3& contactNormal, decimal penDepth,
                             const Vector3& localPt1, const Vector3& localPt2);

        /// Create a new potential contact manifold into the overlapping pair using current contact points
        void addContactPointsAsPotentialContactManifold();

        /// Reset the remaining contact points
        void resetContactPoints();

        /// Get the last collision frame info for temporal coherence
        LastFrameCollisionInfo* getLastFrameCollisionInfo() const;
};

// Get the last collision frame info for temporal coherence
inline LastFrameCollisionInfo* NarrowPhaseInfo::getLastFrameCollisionInfo() const {
    return overlappingPair->getLastFrameCollisionInfo(collisionShape1->getId(), collisionShape2->getId());
}

}

#endif

