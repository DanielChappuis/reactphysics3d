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

#ifndef REACTPHYSICS3D_CONTACT_MANIFOLD_INFO_H
#define	REACTPHYSICS3D_CONTACT_MANIFOLD_INFO_H

// Libraries
#include "collision/ContactPointInfo.h"
#include "memory/Allocator.h"

/// ReactPhysics3D namespace
namespace reactphysics3d {


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

        /// Memory allocator used to allocate contact points
        Allocator& mAllocator;

        // -------------------- Methods -------------------- //


    public:

        // -------------------- Methods -------------------- //

        /// Constructor
        ContactManifoldInfo(Allocator& allocator) : mContactPointsList(nullptr), mAllocator(allocator) {}

        /// Destructor
        ~ContactManifoldInfo() {

            // Remove all the contact points
            reset();
        }

        /// Deleted copy-constructor
        ContactManifoldInfo(const ContactManifoldInfo& contactManifold) = delete;

        /// Deleted assignment operator
        ContactManifoldInfo& operator=(const ContactManifoldInfo& contactManifold) = delete;

        /// Add a new contact point into the manifold
        void addContactPoint(const Vector3& contactNormal, decimal penDepth,
                             const Vector3& localPt1, const Vector3& localPt2) {

            assert(penDepth > decimal(0.0));

            // Create the contact point info
            ContactPointInfo* contactPointInfo = new (mAllocator.allocate(sizeof(ContactPointInfo)))
                    ContactPointInfo(contactNormal, penDepth, localPt1, localPt2);

            // Add it into the linked list of contact points
            contactPointInfo->next = mContactPointsList;
            mContactPointsList = contactPointInfo;
        }

        /// Remove all the contact points
        void reset() {

            // Delete all the contact points in the linked list
            ContactPointInfo* element = mContactPointsList;
            while(element != nullptr) {
                ContactPointInfo* elementToDelete = element;
                element = element->next;

                // Delete the current element
                mAllocator.release(elementToDelete, sizeof(ContactPointInfo));
            }

            mContactPointsList = nullptr;
        }

        /// Get the first contact point info of the linked list of contact points
        ContactPointInfo* getFirstContactPointInfo() const {
            return mContactPointsList;
        }

        /// Reduce the number of points in the contact manifold
        void reduce() {

            // TODO : Implement this (do not forget to deallocate removed points)
        }

        /// Return the largest penetration depth among the contact points
        decimal getLargestPenetrationDepth() const {

            decimal maxDepth = decimal(0.0);
            ContactPointInfo* element = mContactPointsList;
            while(element != nullptr) {

                if (element->penetrationDepth > maxDepth) {
                    maxDepth = element->penetrationDepth;
                }

                element = element->next;
            }

            return maxDepth;
        }
};

}
#endif

