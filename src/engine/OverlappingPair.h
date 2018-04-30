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

#ifndef REACTPHYSICS3D_OVERLAPPING_PAIR_H
#define	REACTPHYSICS3D_OVERLAPPING_PAIR_H

// Libraries
#include "collision/ContactManifoldSet.h"
#include "collision/ProxyShape.h"
#include "containers/Map.h"
#include "containers/Pair.h"
#include "containers/containers_common.h"

/// ReactPhysics3D namespace
namespace reactphysics3d {

// Declarations
struct NarrowPhaseInfo;
class CollisionShape;

// Structure LastFrameCollisionInfo
/**
 * This structure contains collision info about the last frame.
 * This is used for temporal coherence between frames.
 */
struct LastFrameCollisionInfo {

    /// True if we have information about the previous frame
    bool isValid;

    /// True if the frame info is obsolete (the collision shape are not overlapping in middle phase)
    bool isObsolete;

    /// True if the two shapes were colliding in the previous frame
    bool wasColliding;

    /// True if we were using GJK algorithm to check for collision in the previous frame
    bool wasUsingGJK;

    /// True if we were using SAT algorithm to check for collision in the previous frame
    bool wasUsingSAT;

    // ----- GJK Algorithm -----

    /// Previous separating axis
    Vector3 gjkSeparatingAxis;

    // SAT Algorithm
    bool satIsAxisFacePolyhedron1;
    bool satIsAxisFacePolyhedron2;
    uint satMinAxisFaceIndex;
    uint satMinEdge1Index;
    uint satMinEdge2Index;

    /// Constructor
    LastFrameCollisionInfo() {

        isValid = false;
        isObsolete = false;
        wasColliding = false;
        wasUsingSAT = false;
        wasUsingGJK = false;

        gjkSeparatingAxis = Vector3(0, 1, 0);
    }
};

// Class OverlappingPair
/**
 * This class represents a pair of two proxy collision shapes that are overlapping
 * during the broad-phase collision detection. It is created when
 * the two proxy collision shapes start to overlap and is destroyed when they do not
 * overlap anymore. This class contains a contact manifold that
 * store all the contact points between the two bodies.
 */
class OverlappingPair {

    public:

        using OverlappingPairId = Pair<uint, uint>;
        using ShapeIdPair = Pair<uint, uint>;

    private:

        // -------------------- Attributes -------------------- //

        /// Set of persistent contact manifolds
        ContactManifoldSet mContactManifoldSet;

        /// Linked-list of potential contact manifold
        ContactManifoldInfo* mPotentialContactManifolds;

        /// Persistent memory allocator
        MemoryAllocator& mPersistentAllocator;

        /// Memory allocator used to allocated memory for the ContactManifoldInfo and ContactPointInfo
        MemoryAllocator& mTempMemoryAllocator;

        /// Temporal coherence collision data for each overlapping collision shapes of this pair.
        /// Temporal coherence data store collision information about the last frame.
        /// If two convex shapes overlap, we have a single collision data but if one shape is concave,
        /// we might have collision data for several overlapping triangles. The key in the map is the
        /// shape Ids of the two collision shapes.
        Map<ShapeIdPair, LastFrameCollisionInfo*> mLastFrameCollisionInfos;

        /// World settings
        const WorldSettings& mWorldSettings;

    public:

        // -------------------- Methods -------------------- //

        /// Constructor
        OverlappingPair(ProxyShape* shape1, ProxyShape* shape2,  MemoryAllocator& persistentMemoryAllocator,
                        MemoryAllocator& temporaryMemoryAllocator, const WorldSettings& worldSettings);

        /// Destructor
        ~OverlappingPair();

        /// Deleted copy-constructor
        OverlappingPair(const OverlappingPair& pair) = delete;

        /// Deleted assignment operator
        OverlappingPair& operator=(const OverlappingPair& pair) = delete;
        
        /// Return the pointer to first proxy collision shape
        ProxyShape* getShape1() const;

        /// Return the pointer to second body
        ProxyShape* getShape2() const;

        /// Return the last frame collision info
        LastFrameCollisionInfo* getLastFrameCollisionInfo(ShapeIdPair& shapeIds);

        /// Return the a reference to the contact manifold set
        const ContactManifoldSet& getContactManifoldSet();

        /// Clear all the potential contact manifolds
        void clearPotentialContactManifolds();

        /// Add potential contact-points from narrow-phase into potential contact manifolds
        void addPotentialContactPoints(NarrowPhaseInfo* narrowPhaseInfo);

        /// Add a contact to the contact manifold
        void addContactManifold(const ContactManifoldInfo* contactManifoldInfo);

        /// Return a reference to the temporary memory allocator
        MemoryAllocator& getTemporaryAllocator();

        /// Return true if one of the shapes of the pair is a concave shape
        bool hasConcaveShape() const;

		/// Return true if the overlapping pair has contact manifolds with contacts
		bool hasContacts() const;

        /// Return a pointer to the first potential contact manifold in the linked-list
        ContactManifoldInfo* getPotentialContactManifolds();

        /// Reduce the number of contact points of all the potential contact manifolds
        void reducePotentialContactManifolds();

        /// Make the contact manifolds and contact points obsolete
        void makeContactsObsolete();

        /// Clear the obsolete contact manifold and contact points
        void clearObsoleteManifoldsAndContactPoints();

        /// Reduce the contact manifolds that have too many contact points
        void reduceContactManifolds();

        /// Add a new last frame collision info if it does not exist for the given shapes already
        void addLastFrameInfoIfNecessary(uint shapeId1, uint shapeId2);

        /// Return the last frame collision info for a given pair of shape ids
        LastFrameCollisionInfo* getLastFrameCollisionInfo(uint shapeId1, uint shapeId2) const;

        /// Delete all the obsolete last frame collision info
        void clearObsoleteLastFrameCollisionInfos();

        /// Make all the last frame collision infos obsolete
        void makeLastFrameCollisionInfosObsolete();

        /// Return the pair of bodies index
        static OverlappingPairId computeID(ProxyShape* shape1, ProxyShape* shape2);

        /// Return the pair of bodies index of the pair
        static bodyindexpair computeBodiesIndexPair(CollisionBody* body1, CollisionBody* body2);

        // -------------------- Friendship -------------------- //

        friend class DynamicsWorld;
};

// Return the pointer to first body
inline ProxyShape* OverlappingPair::getShape1() const {
    return mContactManifoldSet.getShape1();
}          

// Return the pointer to second body
inline ProxyShape* OverlappingPair::getShape2() const {
    return mContactManifoldSet.getShape2();
}                

// Add a contact to the contact manifold
inline void OverlappingPair::addContactManifold(const ContactManifoldInfo* contactManifoldInfo) {
    mContactManifoldSet.addContactManifold(contactManifoldInfo);
}

// Return the last frame collision info for a given shape id or nullptr if none is found
inline LastFrameCollisionInfo* OverlappingPair::getLastFrameCollisionInfo(ShapeIdPair& shapeIds) {
    Map<ShapeIdPair, LastFrameCollisionInfo*>::Iterator it = mLastFrameCollisionInfos.find(shapeIds);
    if (it != mLastFrameCollisionInfos.end()) {
        return it->second;
    }

    return nullptr;
}

// Return the contact manifold
inline const ContactManifoldSet& OverlappingPair::getContactManifoldSet() {
    return mContactManifoldSet;
}

// Make the contact manifolds and contact points obsolete
inline void OverlappingPair::makeContactsObsolete() {

    mContactManifoldSet.makeContactsObsolete();
}

// Return the pair of bodies index
inline OverlappingPair::OverlappingPairId OverlappingPair::computeID(ProxyShape* shape1, ProxyShape* shape2) {
    assert(shape1->getBroadPhaseId() >= 0 && shape2->getBroadPhaseId() >= 0);

    // Construct the pair of body index
    OverlappingPairId pairID = shape1->getBroadPhaseId() < shape2->getBroadPhaseId() ?
                             OverlappingPairId(shape1->getBroadPhaseId(), shape2->getBroadPhaseId()) :
                             OverlappingPairId(shape2->getBroadPhaseId(), shape1->getBroadPhaseId());
    assert(pairID.first != pairID.second);
    return pairID;
}

// Return the pair of bodies index
inline bodyindexpair OverlappingPair::computeBodiesIndexPair(CollisionBody* body1,
                                                             CollisionBody* body2) {

    // Construct the pair of body index
    bodyindexpair indexPair = body1->getId() < body2->getId() ?
                                 bodyindexpair(body1->getId(), body2->getId()) :
                                 bodyindexpair(body2->getId(), body1->getId());
    assert(indexPair.first != indexPair.second);
    return indexPair;
}

// Return a reference to the temporary memory allocator
inline MemoryAllocator& OverlappingPair::getTemporaryAllocator() {
    return mTempMemoryAllocator;
}

// Return true if one of the shapes of the pair is a concave shape
inline bool OverlappingPair::hasConcaveShape() const {
    return !getShape1()->getCollisionShape()->isConvex() ||
           !getShape2()->getCollisionShape()->isConvex();
}

// Return true if the overlapping pair has contact manifolds with contacts
inline bool OverlappingPair::hasContacts() const {
	return mContactManifoldSet.getContactManifolds() != nullptr;
}

// Return a pointer to the first potential contact manifold in the linked-list
inline ContactManifoldInfo* OverlappingPair::getPotentialContactManifolds() {
    return mPotentialContactManifolds;
}

// Clear the obsolete contact manifold and contact points
inline void OverlappingPair::clearObsoleteManifoldsAndContactPoints() {
    mContactManifoldSet.clearObsoleteManifoldsAndContactPoints();
}

// Reduce the contact manifolds that have too many contact points
inline void OverlappingPair::reduceContactManifolds() {
   mContactManifoldSet.reduce();
}

// Return the last frame collision info for a given pair of shape ids
inline LastFrameCollisionInfo* OverlappingPair::getLastFrameCollisionInfo(uint shapeId1, uint shapeId2) const {
    return mLastFrameCollisionInfos[ShapeIdPair(shapeId1, shapeId2)];
}

}

#endif

