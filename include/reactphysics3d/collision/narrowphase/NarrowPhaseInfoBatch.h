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

#ifndef REACTPHYSICS3D_NARROW_PHASE_INFO_BATCH_H
#define REACTPHYSICS3D_NARROW_PHASE_INFO_BATCH_H

// Libraries
#include <reactphysics3d/engine/OverlappingPairs.h>

/// Namespace ReactPhysics3D
namespace reactphysics3d {

// Declarations
class CollisionShape;
struct LastFrameCollisionInfo;
class ContactManifoldInfo;
struct ContactPointInfo;

// Struct NarrowPhaseInfoBatch
/**
 * This abstract structure collects all the potential collisions from the middle-phase algorithm
 * that have to be tested during narrow-phase collision detection. There is an implementation of
 * this class for each kind of collision detection test. For instance, one for sphere vs sphere,
 * one for sphere vs capsule, ...
 */
struct NarrowPhaseInfoBatch {

    protected:

        /// Memory allocator
        MemoryAllocator& mMemoryAllocator;

        /// Reference to all the broad-phase overlapping pairs
        OverlappingPairs& mOverlappingPairs;

        /// Cached capacity
        uint mCachedCapacity = 0;

    public:

        /// List of Broadphase overlapping pairs ids
        List<uint64> overlappingPairIds;

        /// List of pointers to the first colliders to test collision with
        List<Entity> colliderEntities1;

        /// List of pointers to the second colliders to test collision with
        List<Entity> colliderEntities2;

        /// List of pointers to the first collision shapes to test collision with
        List<CollisionShape*> collisionShapes1;

        /// List of pointers to the second collision shapes to test collision with
        List<CollisionShape*> collisionShapes2;

        /// List of transforms that maps from collision shape 1 local-space to world-space
        List<Transform> shape1ToWorldTransforms;

        /// List of transforms that maps from collision shape 2 local-space to world-space
        List<Transform> shape2ToWorldTransforms;

        /// True for each pair of objects that we need to report contacts (false for triggers for instance)
        List<bool> reportContacts;

        /// Result of the narrow-phase collision detection test
        List<bool> isColliding;

        /// List of contact points created during the narrow-phase
        List<List<ContactPointInfo*>> contactPoints;

        /// Memory allocators for the collision shape (Used to release TriangleShape memory in destructor)
        List<MemoryAllocator*> collisionShapeAllocators;

        /// Collision infos of the previous frame
        List<LastFrameCollisionInfo*> lastFrameCollisionInfos;

        /// Constructor
        NarrowPhaseInfoBatch(MemoryAllocator& allocator, OverlappingPairs& overlappingPairs);

        /// Destructor
        virtual ~NarrowPhaseInfoBatch();

        /// Return the number of objects in the batch
        uint getNbObjects() const;

        /// Add shapes to be tested during narrow-phase collision detection into the batch
        virtual void addNarrowPhaseInfo(uint64 pairId, uint64 pairIndex, Entity collider1, Entity collider2, CollisionShape* shape1,
                                CollisionShape* shape2, const Transform& shape1Transform,
                                const Transform& shape2Transform, bool needToReportContacts, MemoryAllocator& shapeAllocator);

        /// Add a new contact point
        virtual void addContactPoint(uint index, const Vector3& contactNormal, decimal penDepth,
                             const Vector3& localPt1, const Vector3& localPt2);

        /// Reset the remaining contact points
        void resetContactPoints(uint index);

        // Initialize the containers using cached capacity
        virtual void reserveMemory();

        /// Clear all the objects in the batch
        virtual void clear();
};

/// Return the number of objects in the batch
inline uint NarrowPhaseInfoBatch::getNbObjects() const {
    return overlappingPairIds.size();
}

}

#endif

