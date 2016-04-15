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

#ifndef REACTPHYSICS3D_CONTACT_MANIFOLD_SET_H
#define REACTPHYSICS3D_CONTACT_MANIFOLD_SET_H

// Libraries
#include "ContactManifold.h"

namespace reactphysics3d {

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

        /// Reference to the memory allocator
        MemoryAllocator& mMemoryAllocator;

        /// Contact manifolds of the set
        ContactManifold* mManifolds[MAX_MANIFOLDS_IN_CONTACT_MANIFOLD_SET];

        // -------------------- Methods -------------------- //

        /// Create a new contact manifold and add it to the set
        void createManifold(short normalDirectionId);

        /// Remove a contact manifold from the set
        void removeManifold(int index);

        // Return the index of the contact manifold with a similar average normal.
        int selectManifoldWithSimilarNormal(short int normalDirectionId) const;

        // Map the normal vector into a cubemap face bucket (a face contains 4x4 buckets)
        // Each face of the cube is divided into 4x4 buckets. This method maps the
        // normal vector into of the of the bucket and returns a unique Id for the bucket
        short int computeCubemapNormalId(const Vector3& normal) const;

    public:

        // -------------------- Methods -------------------- //

        /// Constructor
        ContactManifoldSet(ProxyShape* shape1, ProxyShape* shape2,
                           MemoryAllocator& memoryAllocator, int nbMaxManifolds);

        /// Destructor
        ~ContactManifoldSet();

        /// Return the first proxy shape
        ProxyShape* getShape1() const;

        /// Return the second proxy shape
        ProxyShape* getShape2() const;

        /// Add a contact point to the manifold set
        void addContactPoint(ContactPoint* contact);

        /// Update the contact manifolds
        void update();

        /// Clear the contact manifold set
        void clear();

        /// Return the number of manifolds in the set
        int getNbContactManifolds() const;

        /// Return a given contact manifold
        ContactManifold* getContactManifold(int index) const;

        /// Return the total number of contact points in the set of manifolds
        int getTotalNbContactPoints() const;
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

// Return a given contact manifold
inline ContactManifold* ContactManifoldSet::getContactManifold(int index) const {
    assert(index >= 0 && index < mNbManifolds);
    return mManifolds[index];
}

// Return the total number of contact points in the set of manifolds
inline int ContactManifoldSet::getTotalNbContactPoints() const {
    int nbPoints = 0;
    for (int i=0; i<mNbManifolds; i++) {
        nbPoints += mManifolds[i]->getNbContactPoints();
    }
    return nbPoints;
}

}

#endif

