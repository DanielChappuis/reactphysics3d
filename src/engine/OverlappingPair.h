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
#include "ContactManifold.h"

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

        // -------------------- Attributes -------------------- //

        // Pointer to the first body of the contact
        CollisionBody* const mBody1;

        // Pointer to the second body of the contact
        CollisionBody* const mBody2;

        // Persistent contact manifold
        ContactManifold mContactManifold;

        // Cached previous separating axis
        Vector3 mCachedSeparatingAxis;
        
        // -------------------- Methods -------------------- //

        // Private copy-constructor
        OverlappingPair(const OverlappingPair& pair);

        // Private assignment operator
        OverlappingPair& operator=(const OverlappingPair& pair);

    public:

        // -------------------- Methods -------------------- //

        // Constructor
        OverlappingPair(CollisionBody* body1, CollisionBody* body2,
                        MemoryPool<ContactPoint>& memoryPoolContacts);

        // Destructor
        ~OverlappingPair();
        
        // Return the pointer to first body
        CollisionBody* const getBody1() const;

        // Return the pointer to second body
        CollisionBody* const getBody2() const;

        // Add a contact to the contact cache
        void addContact(ContactPoint* contact);

        // Update the contact cache
        void update();

        // Return the cached separating axis
        Vector3 getCachedSeparatingAxis() const;

        // Set the cached separating axis
        void setCachedSeparatingAxis(const Vector3& axis);

        // Return the number of contacts in the cache
        uint getNbContactPoints() const;

        // Return the contact manifold
        ContactManifold* getContactManifold();
};

// Return the pointer to first body
inline CollisionBody* const OverlappingPair::getBody1() const {
    return mBody1;
}          

// Return the pointer to second body
inline CollisionBody* const OverlappingPair::getBody2() const {
    return mBody2;
}                

// Add a contact to the contact manifold
inline void OverlappingPair::addContact(ContactPoint* contact) {
    mContactManifold.addContactPoint(contact);
}

// Update the contact manifold
inline void OverlappingPair::update() {
    mContactManifold.update(mBody1->getTransform(), mBody2->getTransform());
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
    return mContactManifold.getNbContactPoints();
}

// Return the contact manifold
inline ContactManifold* OverlappingPair::getContactManifold() {
    return &mContactManifold;
}

} // End of the ReactPhysics3D namespace

#endif

