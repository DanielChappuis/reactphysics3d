/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2024 Daniel Chappuis                                       *
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
#include <reactphysics3d/collision/ContactPointInfo.h>
#include <reactphysics3d/configuration.h>

/// Namespace ReactPhysics3D
namespace reactphysics3d {

// Declarations
class CollisionShape;
struct LastFrameCollisionInfo;
struct ContactManifoldInfo;
struct ContactPointInfo;

// Struct NarrowPhaseInfoBatch
/**
 * This structure collects all the potential collisions from the middle-phase algorithm
 * that have to be tested during narrow-phase collision detection.
 */
struct NarrowPhaseInfoBatch {

    // Struct NarrowPhaseInfo
    /**
     * A potential collision between two colliders from the middle-phase algorithm
     * that have to be tested during narrow-phase collision detection.
     */
    struct NarrowPhaseInfo {

        /// Broadphase overlapping pairs ids
        uint64 overlappingPairId;

        /// Entity of the first collider to test collision with
        Entity colliderEntity1;

        /// Entity of the second collider to test collision with
        Entity colliderEntity2;

        /// Collision info of the previous frame
        LastFrameCollisionInfo* lastFrameCollisionInfo;

        /// Memory allocator for the collision shape (Used to release TriangleShape memory in destructor)
        MemoryAllocator* collisionShapeAllocator;

        /// Shape local to world transform of sphere 1
        Transform shape1ToWorldTransform;

        /// Shape local to world transform of sphere 2
        Transform shape2ToWorldTransform;

        /// Pointer to the first collision shapes to test collision with
        CollisionShape* collisionShape1;

        /// Pointer to the second collision shapes to test collision with
        CollisionShape* collisionShape2;

        /// True if we need to report contacts (false for triggers for instance)
        bool reportContacts;

        /// Result of the narrow-phase collision detection test
        bool isColliding;

        /// Number of contact points
        uint8 nbContactPoints;

        /// Array of contact points created during the narrow-phase
        ContactPointInfo contactPoints[NB_MAX_CONTACT_POINTS_IN_NARROWPHASE_INFO];

        /// Constructor
        NarrowPhaseInfo(uint64 pairId, Entity collider1, Entity collider2, LastFrameCollisionInfo* lastFrameInfo, MemoryAllocator& shapeAllocator,
                             const Transform& shape1ToWorldTransform, const Transform& shape2ToWorldTransform, CollisionShape* shape1,
                             CollisionShape* shape2, bool needToReportContacts)
                      : overlappingPairId(pairId), colliderEntity1(collider1), colliderEntity2(collider2), lastFrameCollisionInfo(lastFrameInfo),
                         collisionShapeAllocator(&shapeAllocator), shape1ToWorldTransform(shape1ToWorldTransform),
                         shape2ToWorldTransform(shape2ToWorldTransform), collisionShape1(shape1),
                        collisionShape2(shape2), reportContacts(needToReportContacts), isColliding(false), nbContactPoints(0) {

        }
    };

    protected:

        /// Reference to the memory allocator
        MemoryAllocator& mMemoryAllocator;

        /// Reference to all the broad-phase overlapping pairs
        OverlappingPairs& mOverlappingPairs;

        /// Cached capacity
        uint32 mCachedCapacity = 0;

        /// TriangleShape allocated size
        static const size_t mTriangleShapeAllocatedSize;

    public:

        /// For each collision test, we keep some meta data
        Array<NarrowPhaseInfo> narrowPhaseInfos;

        /// Constructor
        NarrowPhaseInfoBatch(OverlappingPairs& overlappingPairs, MemoryAllocator& allocator);

        /// Destructor
        ~NarrowPhaseInfoBatch();

        /// Add shapes to be tested during narrow-phase collision detection into the batch
        void addNarrowPhaseInfo(uint64 pairId, Entity collider1, Entity collider2, CollisionShape* shape1,
                                                      CollisionShape* shape2, const Transform& shape1Transform, const Transform& shape2Transform,
                                                      bool needToReportContacts, LastFrameCollisionInfo* lastFrameInfo, MemoryAllocator& shapeAllocator);

        /// Return the number of objects in the batch
        uint32 getNbObjects() const;

        /// Add a new contact point
        void addContactPoint(uint32 index, const Vector3& contactNormal, decimal penDepth,
                             const Vector3& localPt1, const Vector3& localPt2);

        /// Reset the remaining contact points
        void resetContactPoints(uint32 index);

        // Initialize the containers using cached capacity
        void reserveMemory();

        /// Clear all the objects in the batch
        void clear();
};

/// Return the number of objects in the batch
RP3D_FORCE_INLINE uint32 NarrowPhaseInfoBatch::getNbObjects() const {
    return static_cast<uint32>(narrowPhaseInfos.size());
}

// Add shapes to be tested during narrow-phase collision detection into the batch
RP3D_FORCE_INLINE void NarrowPhaseInfoBatch::addNarrowPhaseInfo(uint64 pairId, Entity collider1, Entity collider2, CollisionShape* shape1,
                                              CollisionShape* shape2, const Transform& shape1Transform, const Transform& shape2Transform,
                                              bool needToReportContacts, LastFrameCollisionInfo* lastFrameInfo, MemoryAllocator& shapeAllocator) {

    // Create a meta data object
    narrowPhaseInfos.emplace(pairId, collider1, collider2, lastFrameInfo, shapeAllocator, shape1Transform, shape2Transform, shape1, shape2, needToReportContacts);
}

// Add a new contact point
RP3D_FORCE_INLINE void NarrowPhaseInfoBatch::addContactPoint(uint32 index, const Vector3& contactNormal, decimal penDepth, const Vector3& localPt1, const Vector3& localPt2) {

    assert(penDepth > decimal(0.0));

    if (narrowPhaseInfos[index].nbContactPoints < NB_MAX_CONTACT_POINTS_IN_NARROWPHASE_INFO) {

        assert(contactNormal.length() > 0.8f);

        // Add it into the array of contact points
        narrowPhaseInfos[index].contactPoints[narrowPhaseInfos[index].nbContactPoints].normal = contactNormal;
        narrowPhaseInfos[index].contactPoints[narrowPhaseInfos[index].nbContactPoints].penetrationDepth = penDepth;
        narrowPhaseInfos[index].contactPoints[narrowPhaseInfos[index].nbContactPoints].localPoint1 = localPt1;
        narrowPhaseInfos[index].contactPoints[narrowPhaseInfos[index].nbContactPoints].localPoint2 = localPt2;
        narrowPhaseInfos[index].nbContactPoints++;
    }
}

// Reset the remaining contact points
RP3D_FORCE_INLINE void NarrowPhaseInfoBatch::resetContactPoints(uint32 index) {
    narrowPhaseInfos[index].nbContactPoints = 0;
}

}

#endif

