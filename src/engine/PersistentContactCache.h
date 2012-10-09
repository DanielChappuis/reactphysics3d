/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2010-2012 Daniel Chappuis                                       *
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

#ifndef PERSISTENT_CONTACT_CACHE_H
#define	PERSISTENT_CONTACT_CACHE_H

// Libraries
#include <vector>
#include "../body/Body.h"
#include "../constraint/Contact.h"

// ReactPhysics3D namespace
namespace reactphysics3d {

// Constants
const uint MAX_CONTACTS_IN_CACHE = 4;   // Maximum number of contacts in the persistent cache


/*  -------------------------------------------------------------------
    Class PersistentContactCache :
        This class represents a cache of at most 4 contact points between
        two given bodies. The contacts between two bodies are added one
        after the other in the cache. When the cache is full, we have
        to remove one point. The idea is to keep the point with the deepest
        penetration depth and also to keep the points producing the larger
        area (for a more stable contact manifold). The new added point is
        always kept. This kind of persistent cache has been explained for
        instance in the presentation from Erin Catto about contact manifolds
        at the GDC 2007 conference.
    -------------------------------------------------------------------
*/
class PersistentContactCache {

    private:

        // -------------------- Attributes -------------------- //

        // Pointer to the first body
        Body* const mBody1;

        // Pointer to the second body
        Body* const mBody2;

        // Contacts in the cache
        Contact* mContacts[MAX_CONTACTS_IN_CACHE];

        // Number of contacts in the cache
        uint mNbContacts;

        // Reference to the memory pool with the contacts
        MemoryPool<Contact>& mMemoryPoolContacts;

        // -------------------- Methods -------------------- //

        // Private copy-constructor
        PersistentContactCache(const PersistentContactCache& persistentContactCache);

        // Private assignment operator
        PersistentContactCache& operator=(const PersistentContactCache& persistentContactCache);

        // Return the index of maximum area
        int getMaxArea(decimal area0, decimal area1, decimal area2, decimal area3) const;

        // Return the index of the contact with the larger penetration depth
        int getIndexOfDeepestPenetration(Contact* newContact) const;

        // Return the index that will be removed
        int getIndexToRemove(int indexMaxPenetration, const Vector3& newPoint) const;

        // Remove a contact from the cache
        void removeContact(int index);

        // Return true if two vectors are approximatively equal
        bool isApproxEqual(const Vector3& vector1, const Vector3& vector2) const;
        
    public:

        // -------------------- Methods -------------------- //

        // Constructor
        PersistentContactCache(Body* const mBody1, Body* const mBody2,
                               MemoryPool<Contact>& mMemoryPoolContacts);

        // Destructor
        ~PersistentContactCache();

        // Add a contact
        void addContact(Contact* contact);

        // Update the contact cache
        void update(const Transform& transform1, const Transform& transform2);

        // Clear the cache
        void clear();

        // Return the number of contacts in the cache
        uint getNbContacts() const;

        // Return a contact of the cache
        Contact* getContact(uint index) const;
};

// Return the number of contacts in the cache
inline uint PersistentContactCache::getNbContacts() const {
    return mNbContacts;
} 

// Return a contact of the cache
inline Contact* PersistentContactCache::getContact(uint index) const {
    assert(index >= 0 && index < mNbContacts);
    return mContacts[index];
}  

// Return true if two vectors are approximatively equal
inline bool PersistentContactCache::isApproxEqual(const Vector3& vector1,
                                                  const Vector3& vector2) const {
    const decimal epsilon = 0.1;
    return (approxEqual(vector1.getX(), vector2.getX(), epsilon) &&
            approxEqual(vector1.getY(), vector2.getY(), epsilon) &&
            approxEqual(vector1.getZ(), vector2.getZ(), epsilon));
}  

}
#endif

