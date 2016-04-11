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
typedef std::pair<uint, uint> overlappingpairid;

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

        /// Cached previous separating axis
        Vector3 mCachedSeparatingAxis;
        
        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        OverlappingPair(const OverlappingPair& pair);

        /// Private assignment operator
        OverlappingPair& operator=(const OverlappingPair& pair);

    public:

        // -------------------- Methods -------------------- //

        /// Constructor
        OverlappingPair(ProxyShape* shape1, ProxyShape* shape2,
                        int nbMaxContactManifolds, MemoryAllocator& memoryAllocator);

        /// Destructor
        ~OverlappingPair();
        
        /// Return the pointer to first proxy collision shape
        ProxyShape* getShape1() const;

        /// Return the pointer to second body
        ProxyShape* getShape2() const;

        /// Add a contact to the contact cache
        void addContact(ContactPoint* contact);

        /// Update the contact cache
        void update();

        /// Return the cached separating axis
        Vector3 getCachedSeparatingAxis() const;

        /// Set the cached separating axis
        void setCachedSeparatingAxis(const Vector3& axis);

        /// Return the number of contacts in the cache
        uint getNbContactPoints() const;

        /// Return the a reference to the contact manifold set
        const ContactManifoldSet& getContactManifoldSet();

        /// Clear the contact points of the contact manifold
        void clearContactPoints();

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
inline void OverlappingPair::addContact(ContactPoint* contact) {
    mContactManifoldSet.addContactPoint(contact);
}

// Update the contact manifold
inline void OverlappingPair::update() {
    mContactManifoldSet.update();
}

// Return the cached separating axis
inline Vector3 OverlappingPair::getCachedSeparatingAxis() const {
    return mCachedSeparatingAxis;
}

// Set the cached separating axis
inline void OverlappingPair::setCachedSeparatingAxis(const Vector3& axis) {
    mCachedSeparatingAxis = axis;
}


// Return the number of contact points in the contact manifold
inline uint OverlappingPair::getNbContactPoints() const {
    return mContactManifoldSet.getTotalNbContactPoints();
}

// Return the contact manifold
inline const ContactManifoldSet& OverlappingPair::getContactManifoldSet() {
    return mContactManifoldSet;
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

// Clear the contact points of the contact manifold
inline void OverlappingPair::clearContactPoints() {
   mContactManifoldSet.clear();
}

}

#endif

