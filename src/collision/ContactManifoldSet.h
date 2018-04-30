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

#ifndef REACTPHYSICS3D_CONTACT_MANIFOLD_SET_H
#define REACTPHYSICS3D_CONTACT_MANIFOLD_SET_H

namespace reactphysics3d {

// Declarations
class ContactManifold;
class ContactManifoldInfo;
class ProxyShape;
class MemoryAllocator;
struct WorldSettings;
class CollisionShape;

// Constants
const int MAX_MANIFOLDS_IN_CONTACT_MANIFOLD_SET = 3;   // Maximum number of contact manifolds in the set
const int CONTACT_CUBEMAP_FACE_NB_SUBDIVISIONS = 3;    // N Number for the N x N subdivisions of the cubemap

// Class ContactManifoldSet
/**
 * This class represents a set of one or several contact manifolds. Typically a
 * convex/convex collision will have a set with a single manifold and a convex-concave
 * collision can have more than one manifolds. Note that a contact manifold can
 * contains several contact points.
 */
class ContactManifoldSet {

    private:

        // -------------------- Attributes -------------------- //

        /// Maximum number of contact manifolds in the set
        int mNbMaxManifolds;

        /// Current number of contact manifolds in the set
        int mNbManifolds;

        /// Pointer to the first proxy shape of the contact
        ProxyShape* mShape1;

        /// Pointer to the second proxy shape of the contact
        ProxyShape* mShape2;

        /// Reference to the memory allocator for the contact manifolds
        MemoryAllocator& mMemoryAllocator;

        /// Contact manifolds of the set
        ContactManifold* mManifolds;

        /// World settings
        const WorldSettings& mWorldSettings;

        // -------------------- Methods -------------------- //

        /// Create a new contact manifold and add it to the set
        void createManifold(const ContactManifoldInfo* manifoldInfo);

        // Return the contact manifold with a similar contact normal.
        ContactManifold* selectManifoldWithSimilarNormal(const ContactManifoldInfo* contactManifold) const;

        /// Remove a contact manifold that is the least optimal (smaller penetration depth)
        void removeNonOptimalManifold();

        /// Update a previous similar manifold with a new one
        void updateManifoldWithNewOne(ContactManifold* oldManifold, const ContactManifoldInfo* newManifold);

        /// Return the maximum number of contact manifolds allowed between to collision shapes
        int computeNbMaxContactManifolds(const CollisionShape* shape1, const CollisionShape* shape2);

        /// Clear the contact manifold set
        void clear();

        /// Delete a contact manifold
        void removeManifold(ContactManifold* manifold);

    public:

        // -------------------- Methods -------------------- //

        /// Constructor
        ContactManifoldSet(ProxyShape* shape1, ProxyShape* shape2,
                           MemoryAllocator& memoryAllocator, const WorldSettings& worldSettings);

        /// Destructor
        ~ContactManifoldSet();

        /// Add a contact manifold in the set
        void addContactManifold(const ContactManifoldInfo* contactManifoldInfo);

        /// Return the first proxy shape
        ProxyShape* getShape1() const;

        /// Return the second proxy shape
        ProxyShape* getShape2() const;

        /// Return the number of manifolds in the set
        int getNbContactManifolds() const;

        /// Return a pointer to the first element of the linked-list of contact manifolds
        ContactManifold* getContactManifolds() const;

        /// Make all the contact manifolds and contact points obsolete
        void makeContactsObsolete();

        /// Return the total number of contact points in the set of manifolds
        int getTotalNbContactPoints() const;

        /// Clear the obsolete contact manifolds and contact points
        void clearObsoleteManifoldsAndContactPoints();

        // Remove some contact manifolds and contact points if there are too many of them
        void reduce();
};

// Return the first proxy shape
inline ProxyShape* ContactManifoldSet::getShape1() const {
    return mShape1;
}

// Return the second proxy shape
inline ProxyShape* ContactManifoldSet::getShape2() const {
    return mShape2;
}

// Return the number of manifolds in the set
inline int ContactManifoldSet::getNbContactManifolds() const {
    return mNbManifolds;
}

// Return a pointer to the first element of the linked-list of contact manifolds
inline ContactManifold* ContactManifoldSet::getContactManifolds() const {
    return mManifolds;
}

}

#endif

