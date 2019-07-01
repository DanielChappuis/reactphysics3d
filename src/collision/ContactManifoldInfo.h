/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2019 Daniel Chappuis                                       *
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

#ifndef REACTPHYSICS3D_CONTACT_MANIFOLD_INFO_H
#define	REACTPHYSICS3D_CONTACT_MANIFOLD_INFO_H

// Libraries
#include "configuration.h"

/// ReactPhysics3D namespace
namespace reactphysics3d {

// Declarations
class MemoryAllocator;
struct ContactPointInfo;
class Transform;

// Constants
const int8 MAX_CONTACT_POINTS_IN_MANIFOLD = 4;   // Maximum number of contacts in the manifold

// Class ContactManifoldInfo
/**
 * This class is used to collect the list of ContactPointInfo that come
 * from a collision test between two shapes.
 */
class ContactManifoldInfo {

    private:

        // -------------------- Attributes -------------------- //

        /// Linked list with all the contact points
        ContactPointInfo* mContactPointsList;

        /// Number of contact points in the manifold
        uint mNbContactPoints;

        /// Next element in the linked-list of contact manifold info
        ContactManifoldInfo* mNext;

        /// Reference the the memory allocator where the contact point infos have been allocated
        MemoryAllocator& mAllocator;

    public:

        // -------------------- Methods -------------------- //

        /// Constructor
        ContactManifoldInfo(MemoryAllocator& allocator);

        /// Destructor
        ~ContactManifoldInfo();

        /// Deleted Copy-constructor
        ContactManifoldInfo(const ContactManifoldInfo& contactManifold) = delete;

        /// Deleted assignment operator
        ContactManifoldInfo& operator=(const ContactManifoldInfo& contactManifold) = delete;

        /// Add a new contact point into the manifold
        void addContactPoint(ContactPointInfo* contactPointInfo);

        /// Remove all the contact points
        void reset();

        /// Get the first contact point info of the linked list of contact points
        ContactPointInfo* getFirstContactPointInfo() const;

        /// Return the largest penetration depth among its contact points
        decimal getLargestPenetrationDepth() const;

        /// Return the pointer to the next manifold info in the linked-list
        ContactManifoldInfo* getNext();

        /// Reduce the number of contact points of the currently computed manifold
        void reduce(const Transform& shape1ToWorldTransform);

        // Friendship
        friend class OverlappingPair;
        friend class CollisionDetection;
};

// Get the first contact point info of the linked list of contact points
inline ContactPointInfo* ContactManifoldInfo::getFirstContactPointInfo() const {
    return mContactPointsList;
}

// Return the pointer to the next manifold info in the linked-list
inline ContactManifoldInfo* ContactManifoldInfo::getNext() {
    return mNext;
}

}
#endif

