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

