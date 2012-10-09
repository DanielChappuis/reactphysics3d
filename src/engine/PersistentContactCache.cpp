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

// Libraries
#include "PersistentContactCache.h"

using namespace reactphysics3d;

// Constructor
PersistentContactCache::PersistentContactCache(Body* const body1, Body* const body2, MemoryPool<Contact>& memoryPoolContacts)
                       : mBody1(body1), mBody2(body2), mNbContacts(0), mMemoryPoolContacts(memoryPoolContacts) {
    
}

// Destructor
PersistentContactCache::~PersistentContactCache() {
    clear();
}

// Add a contact in the cache
void PersistentContactCache::addContact(Contact* contact) {

    int indexNewContact = mNbContacts;
	
	// For contact already in the cache
	for (uint i=0; i<mNbContacts; i++) {
		// Check if the new point point does not correspond to a same contact point
		// already in the cache. If it's the case, we do not add the new contact
		if (isApproxEqual(contact->getLocalPointOnBody1(), mContacts[i]->getLocalPointOnBody1())) {
			// Delete the new contact
			contact->Contact::~Contact();
			mMemoryPoolContacts.freeObject(contact);
			
			return;
		}
	}
    
    // If the contact cache is full
    if (mNbContacts == MAX_CONTACTS_IN_CACHE) {
        int indexMaxPenetration = getIndexOfDeepestPenetration(contact);
        int indexToRemove = getIndexToRemove(indexMaxPenetration, contact->getLocalPointOnBody1());
        removeContact(indexToRemove);
        indexNewContact = indexToRemove;
    }

    // Add the new contact in the cache
    mContacts[indexNewContact] = contact;
    mNbContacts++;
}

// Remove a contact from the cache
void PersistentContactCache::removeContact(int index) {
    assert(index >= 0 && index < mNbContacts);
    assert(mNbContacts > 0);
	
	// Call the destructor explicitly and tell the memory pool that
	// the corresponding memory block is now free
	mContacts[index]->Contact::~Contact();
	mMemoryPoolContacts.freeObject(mContacts[index]);
	
    // If we don't remove the last index
    if (index < mNbContacts - 1) {
        mContacts[index] = mContacts[mNbContacts - 1];
    }

    mNbContacts--;
}

// Update the contact cache
// First the world space coordinates of the current contacts in the cache are recomputed from
// the corresponding transforms of the bodies because they have moved. Then we remove the contacts
// with a negative penetration depth (meaning that the bodies are not penetrating anymore) and also
// the contacts with a too large distance between the contact points in the plane orthogonal to the
// contact normal
void PersistentContactCache::update(const Transform& transform1, const Transform& transform2) {
    if (mNbContacts == 0) return;

    // Update the world coordinates and penetration depth of the contacts in the cache
    for (int i=0; i<mNbContacts; i++) {
        mContacts[i]->setWorldPointOnBody1(transform1 * mContacts[i]->getLocalPointOnBody1());
        mContacts[i]->setWorldPointOnBody2(transform2 * mContacts[i]->getLocalPointOnBody2());
        mContacts[i]->setPenetrationDepth((mContacts[i]->getWorldPointOnBody1() - mContacts[i]->getWorldPointOnBody2()).dot(mContacts[i]->getNormal()));
    }

    // Remove the contacts that don't represent very well the persistent contact
    for (int i=mNbContacts-1; i>=0; i--) {
        assert(i>= 0 && i < mNbContacts);
        
        // Remove the contacts with a negative penetration depth (meaning that the bodies are not penetrating anymore)
        if (mContacts[i]->getPenetrationDepth() <= 0.0) {
            removeContact(i);
        }
        else {
            // Compute the distance of the two contact points in the place orthogonal to the contact normal
            Vector3 projOfPoint1 = mContacts[i]->getWorldPointOnBody1() - mContacts[i]->getNormal() * mContacts[i]->getPenetrationDepth();
            Vector3 projDifference = mContacts[i]->getWorldPointOnBody2() - projOfPoint1;

            // If the orthogonal distance is larger than the valid distance threshold, we remove the contact
            if (projDifference.lengthSquare() > PERSISTENT_CONTACT_DIST_THRESHOLD * PERSISTENT_CONTACT_DIST_THRESHOLD) {
                removeContact(i);
            }
        }
    }    
}

// Return the index of the contact with the larger penetration depth. This
// corresponding contact will be kept in the cache. The method returns -1 is
// the new contact is the deepest.
int PersistentContactCache::getIndexOfDeepestPenetration(Contact* newContact) const {
    assert(mNbContacts == MAX_CONTACTS_IN_CACHE);
    int indexMaxPenetrationDepth = -1;
    decimal maxPenetrationDepth = newContact->getPenetrationDepth();

    // For each contact in the cache
    for (uint i=0; i<mNbContacts; i++) {
        // If the current contact has a larger penetration depth
        if (mContacts[i]->getPenetrationDepth() > maxPenetrationDepth) {
            maxPenetrationDepth = mContacts[i]->getPenetrationDepth();
            indexMaxPenetrationDepth = i;
        }
    }

    // Return the index of largest penetration depth
    return indexMaxPenetrationDepth;
}

// Return the index that will be removed. The index of the contact with the larger penetration
// depth is given as a parameter. This contact won't be removed. Given this contact, we compute
// the different area and we want to keep the contacts with the largest area. The new point is also
// kept.
int PersistentContactCache::getIndexToRemove(int indexMaxPenetration, const Vector3& newPoint) const {
    assert(mNbContacts == MAX_CONTACTS_IN_CACHE);
    decimal area0 = 0.0;       // Area with contact 1,2,3 and newPoint
    decimal area1 = 0.0;       // Area with contact 0,2,3 and newPoint
    decimal area2 = 0.0;       // Area with contact 0,1,3 and newPoint
    decimal area3 = 0.0;       // Area with contact 0,1,2 and newPoint

    if (indexMaxPenetration != 0) {
        // Compute the area
        Vector3 vector1 = newPoint - mContacts[1]->getLocalPointOnBody1();
        Vector3 vector2 = mContacts[3]->getLocalPointOnBody1() - mContacts[2]->getLocalPointOnBody1();
        Vector3 crossProduct = vector1.cross(vector2);
        area0 = crossProduct.lengthSquare();
    }
    if (indexMaxPenetration != 1) {
        // Compute the area
        Vector3 vector1 = newPoint - mContacts[0]->getLocalPointOnBody1();
        Vector3 vector2 = mContacts[3]->getLocalPointOnBody1() - mContacts[2]->getLocalPointOnBody1();
        Vector3 crossProduct = vector1.cross(vector2);
        area1 = crossProduct.lengthSquare();
    }
    if (indexMaxPenetration != 2) {
        // Compute the area
        Vector3 vector1 = newPoint - mContacts[0]->getLocalPointOnBody1();
        Vector3 vector2 = mContacts[3]->getLocalPointOnBody1() - mContacts[1]->getLocalPointOnBody1();
        Vector3 crossProduct = vector1.cross(vector2);
        area2 = crossProduct.lengthSquare();
    }
    if (indexMaxPenetration != 3) {
        // Compute the area
        Vector3 vector1 = newPoint - mContacts[0]->getLocalPointOnBody1();
        Vector3 vector2 = mContacts[2]->getLocalPointOnBody1() - mContacts[1]->getLocalPointOnBody1();
        Vector3 crossProduct = vector1.cross(vector2);
        area3 = crossProduct.lengthSquare();
    }
    
    // Return the index of the contact to remove
    return getMaxArea(area0, area1, area2, area3);
}

// Return the index of maximum area
int PersistentContactCache::getMaxArea(decimal area0, decimal area1, decimal area2, decimal area3) const {
    if (area0 < area1) {
        if (area1 < area2) {
            if (area2 < area3) return 3;
            else return 2;
        }
        else {
            if (area1 < area3) return 3;
            else return 1;
        }
    }
    else {
        if (area0 < area2) {
            if (area2 < area3) return 3;
            else return 2;
        }
        else {
            if (area0 < area3) return 3;
            else return 0;
        }
    }
}

// Clear the cache
void PersistentContactCache::clear() {
    for (uint i=0; i<mNbContacts; i++) {
		
		// Call the destructor explicitly and tell the memory pool that
		// the corresponding memory block is now free
		mContacts[i]->Contact::~Contact();
		mMemoryPoolContacts.freeObject(mContacts[i]);
    }
    mNbContacts = 0;
}