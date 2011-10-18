/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2011 Daniel Chappuis                                            *
*********************************************************************************
*                                                                               *
* Permission is hereby granted, free of charge, to any person obtaining a copy  *
* of this software and associated documentation files (the "Software"), to deal *
* in the Software without restriction, including without limitation the rights  *
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell     *
* copies of the Software, and to permit persons to whom the Software is         *
* furnished to do so, subject to the following conditions:                      *
*                                                                               *
* The above copyright notice and this permission notice shall be included in    *
* all copies or substantial portions of the Software.                           *
*                                                                               *
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR    *
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,      *
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE   *
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER        *
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, *
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN     *
* THE SOFTWARE.                                                                 *
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
const uint MAX_CONTACTS_IN_CACHE = 4;           // Maximum number of contacts in the persistent cache

// TODO : Move this class in collision/ folder

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
        Body* const body1;                          // Pointer to the first body
        Body* const body2;                          // Pointer to the second body
        Contact* contacts[MAX_CONTACTS_IN_CACHE];   // Contacts in the cache
        uint nbContacts;                            // Number of contacts in the cache
        MemoryPool<Contact>& memoryPoolContacts;    // Reference to the memory pool with the contacts

        int getMaxArea(double area0, double area1, double area2, double area3) const;   // Return the index of maximum area
        int getIndexOfDeepestPenetration(Contact* newContact) const;                    // Return the index of the contact with the larger penetration depth
        int getIndexToRemove(int indexMaxPenetration, const Vector3& newPoint) const;   // Return the index that will be removed
        void removeContact(int index);                                                  // Remove a contact from the cache
        bool isApproxEqual(const Vector3& vector1, const Vector3& vector2) const;       // Return true if two vectors are approximatively equal    
        
    public:
        PersistentContactCache(Body* const body1, Body* const body2, MemoryPool<Contact>& memoryPoolContacts);  // Constructor
        ~PersistentContactCache();                                                                              // Destructor
        void addContact(Contact* contact);                                                                      // Add a contact
        void update(const Transform& transform1, const Transform& transform2);                                  // Update the contact cache
        void clear();                                                                                           // Clear the cache
        uint getNbContacts() const;                                                                             // Return the number of contacts in the cache
        Contact* getContact(uint index) const;                                                                  // Return a contact of the cache                           
};

// Return the number of contacts in the cache
inline uint PersistentContactCache::getNbContacts() const {
    return nbContacts;
} 

// Return a contact of the cache
inline Contact* PersistentContactCache::getContact(uint index) const {
    assert(index >= 0 && index < nbContacts);
    return contacts[index];
}  

// Return true if two vectors are approximatively equal
inline bool PersistentContactCache::isApproxEqual(const Vector3& vector1, const Vector3& vector2) const {
    const double epsilon = 0.1;
    return (approxEqual(vector1.getX(), vector2.getX(), epsilon) &&
            approxEqual(vector1.getY(), vector2.getY(), epsilon) &&
            approxEqual(vector1.getZ(), vector2.getZ(), epsilon));
}  

}
#endif

