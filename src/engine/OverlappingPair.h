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

#ifndef REACTPHYSICS3D_OVERLAPPING_PAIR_H
#define	REACTPHYSICS3D_OVERLAPPING_PAIR_H

// Libraries
#include "collision/ContactManifoldSet.h"
#include "collision/ProxyShape.h"
#include "collision/shapes/CollisionShape.h"

/// ReactPhysics3D namespace
namespace reactphysics3d {

// Type for the overlapping pair ID
using overlappingpairid = std::pair<uint, uint>;

// Structure LastFrameCollisionInfo
/**
 * This structure contains collision info about the last frame.
 * This is used for temporal coherence between frames.
 */
struct LastFrameCollisionInfo {

    /// True if we have information about the previous frame
    bool isValid;

    /// True if the two shapes were colliding in the previous frame
    bool wasColliding;

    /// True if we were using GJK algorithm to check for collision in the previous frame
    bool wasUsingGJK;

    /// True if we were using SAT algorithm to check for collision in the previous frame
    bool wasUsingSAT;

    /// True if there was a narrow-phase collision
    /// in the previous frame
    bool wasCollidingLastFrame;

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
        wasColliding = false;
        wasUsingSAT = false;
        wasUsingGJK = false;
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

    private:

        // -------------------- Attributes -------------------- //

        /// Set of persistent contact manifolds
        ContactManifoldSet mContactManifoldSet;

        /// Collision information about the last frame (for temporal coherence)
        LastFrameCollisionInfo mLastFrameCollisionInfo;

        /// Linked-list of potential contact manifold
        ContactManifoldInfo* mPotentialContactManifolds;

        /// Memory allocator used to allocated memory for the ContactManifoldInfo and ContactPointInfo
        Allocator& mTempMemoryAllocator;

    public:

        // -------------------- Methods -------------------- //

        /// Constructor
        OverlappingPair(ProxyShape* shape1, ProxyShape* shape2,
                        Allocator& memoryAllocator, Allocator& temporaryMemoryAllocator);

        /// Destructor
        ~OverlappingPair() = default;

        /// Deleted copy-constructor
        OverlappingPair(const OverlappingPair& pair) = delete;

        /// Deleted assignment operator
        OverlappingPair& operator=(const OverlappingPair& pair) = delete;
        
        /// Return the pointer to first proxy collision shape
        ProxyShape* getShape1() const;

        /// Return the pointer to second body
        ProxyShape* getShape2() const;

        /// Return the last frame collision info
        LastFrameCollisionInfo& getLastFrameCollisionInfo();

        /// Return the number of contacts in the cache
        uint getNbContactPoints() const;

        /// Return the a reference to the contact manifold set
        const ContactManifoldSet& getContactManifoldSet();

        /// Clear all the potential contact manifolds
        void clearPotentialContactManifolds();

        /// Add potential contact-points from narrow-phase into potential contact manifolds
        void addPotentialContactPoints(NarrowPhaseInfo* narrowPhaseInfo);

        /// Add a contact to the contact manifold
        void addContactManifold(const ContactManifoldInfo* contactManifoldInfo);

        /// Return a reference to the temporary memory allocator
        Allocator& getTemporaryAllocator();

        /// Return true if one of the shapes of the pair is a concave shape
        bool hasConcaveShape() const;

        /// Return a pointer to the first potential contact manifold in the linked-list
        ContactManifoldInfo* getPotentialContactManifolds();

        /// Reduce the number of contact points of all the potential contact manifolds
        void reducePotentialContactManifolds();

        /// Make the contact manifolds and contact points obselete
        void makeContactsObselete();

        /// Clear the obselete contact manifold and contact points
        void clearObseleteManifoldsAndContactPoints();

        /// Return the pair of bodies index
        static overlappingpairid computeID(ProxyShape* shape1, ProxyShape* shape2);

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

// Return the last frame collision info
inline LastFrameCollisionInfo& OverlappingPair::getLastFrameCollisionInfo() {
    return mLastFrameCollisionInfo;
}

// Return the number of contact points in the contact manifold
inline uint OverlappingPair::getNbContactPoints() const {
    return mContactManifoldSet.getTotalNbContactPoints();
}

// Return the contact manifold
inline const ContactManifoldSet& OverlappingPair::getContactManifoldSet() {
    return mContactManifoldSet;
}

// Make the contact manifolds and contact points obselete
inline void OverlappingPair::makeContactsObselete() {

    mContactManifoldSet.makeContactsObselete();
}

// Return the pair of bodies index
inline overlappingpairid OverlappingPair::computeID(ProxyShape* shape1, ProxyShape* shape2) {
    assert(shape1->mBroadPhaseID >= 0 && shape2->mBroadPhaseID >= 0);

    // Construct the pair of body index
    overlappingpairid pairID = shape1->mBroadPhaseID < shape2->mBroadPhaseID ?
                             std::make_pair(shape1->mBroadPhaseID, shape2->mBroadPhaseID) :
                             std::make_pair(shape2->mBroadPhaseID, shape1->mBroadPhaseID);
    assert(pairID.first != pairID.second);
    return pairID;
}

// Return the pair of bodies index
inline bodyindexpair OverlappingPair::computeBodiesIndexPair(CollisionBody* body1,
                                                             CollisionBody* body2) {

    // Construct the pair of body index
    bodyindexpair indexPair = body1->getID() < body2->getID() ?
                                 std::make_pair(body1->getID(), body2->getID()) :
                                 std::make_pair(body2->getID(), body1->getID());
    assert(indexPair.first != indexPair.second);
    return indexPair;
}

// Return a reference to the temporary memory allocator
inline Allocator& OverlappingPair::getTemporaryAllocator() {
    return mTempMemoryAllocator;
}

// Return true if one of the shapes of the pair is a concave shape
inline bool OverlappingPair::hasConcaveShape() const {
    return !getShape1()->getCollisionShape()->isConvex() ||
           !getShape2()->getCollisionShape()->isConvex();
}

// Return a pointer to the first potential contact manifold in the linked-list
inline ContactManifoldInfo* OverlappingPair::getPotentialContactManifolds() {
    return mPotentialContactManifolds;
}

// Clear the obselete contact manifold and contact points
inline void OverlappingPair::clearObseleteManifoldsAndContactPoints() {
    mContactManifoldSet.clearObseleteManifoldsAndContactPoints();
}

}

#endif

