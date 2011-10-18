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

#ifndef OVERLAPPING_PAIR_H
#define	OVERLAPPING_PAIR_H

// Libraries
#include "PersistentContactCache.h"

// ReactPhysics3D namespace
namespace reactphysics3d {

/*  -------------------------------------------------------------------
    Class OverlappingPair :
        This class represents a pair of two bodies that are overlapping
        during the broad-phase collision detection. It is created when
        the two bodies start to overlap and is destroy when they do not 
        overlap anymore. This class contains the cache with all the
        current contacts between the bodies.
    -------------------------------------------------------------------
*/
class OverlappingPair {
    private:
        Body* const body1;                      // Pointer to the first body of the contact
        Body* const body2;                      // Pointer to the second body of the contact
        PersistentContactCache contactsCache;   // Persistent contact cache
        Vector3 cachedSeparatingAxis;           // Cached previous separating axis
        
    public:
        OverlappingPair(Body* body1, Body* body2, MemoryPool<Contact>& memoryPoolContacts);     // Constructor
        ~OverlappingPair();                                                                     // Destructor
        
        Body* const getBody1() const;                 // Return the pointer to first body
        Body* const getBody2() const;                 // Return the pointer to second body
        void addContact(Contact* contact);            // Add a contact to the contact cache
        void update();                                // Update the contact cache
        Vector3 getCachedSeparatingAxis() const;      // Return the cached separating axis
        uint getNbContacts() const;                   // Return the number of contacts in the cache
        Contact* getContact(uint index) const;        // Return a contact of the cache                           
};

// Return the pointer to first body
inline Body* const OverlappingPair::getBody1() const {
    return body1;
}          

// Return the pointer to second body
inline Body* const OverlappingPair::getBody2() const {
    return body2;
}                

// Add a contact to the contact cache
inline void OverlappingPair::addContact(Contact* contact) {
    contactsCache.addContact(contact);
}  

// Update the contact cache
inline void OverlappingPair::update() {
    contactsCache.update(body1->getTransform(), body2->getTransform());
}                                

// Return the cached separating axis
inline Vector3 OverlappingPair::getCachedSeparatingAxis() const {
    return cachedSeparatingAxis;
}      


// Return the number of contacts in the cache
inline uint OverlappingPair::getNbContacts() const {
    return contactsCache.getNbContacts();
}

// Return a contact of the cache    
inline Contact* OverlappingPair::getContact(uint index) const {
    return contactsCache.getContact(index);
}         

} // End of the ReactPhysics3D namespace

#endif

