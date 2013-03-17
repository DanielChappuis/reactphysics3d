/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2010-2013 Daniel Chappuis                                       *
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

#ifndef CONTACT_MANIFOLD_H
#define	CONTACT_MANIFOLD_H

// Libraries
#include <vector>
#include "../body/Body.h"
#include "../constraint/ContactPoint.h"

/// ReactPhysics3D namespace
namespace reactphysics3d {

// Constants
const uint MAX_CONTACT_POINTS_IN_MANIFOLD = 4;   // Maximum number of contacts in the manifold

// Class ContactManifold
/**
 * This class represents the set of contact points between two bodies.
 * The contact manifold is implemented in a way to cache the contact
 * points among the frames for better stability following the
 * "Contact Generation" presentation of Erwin Coumans at GDC 2010
 * conference (bullet.googlecode.com/files/GDC10_Coumans_Erwin_Contact.pdf).
 * Some code of this class is based on the implementation of the
 * btPersistentManifold class from Bullet physics engine (www.http://bulletphysics.org).
 * The contacts between two bodies are added one after the other in the cache.
 * When the cache is full, we have to remove one point. The idea is to keep
 * the point with the deepest penetration depth and also to keep the
 * points producing the larger area (for a more stable contact manifold).
 * The new added point is always kept.
 */
class ContactManifold {

    private:

        // -------------------- Attributes -------------------- //

        /// Pointer to the first body
        Body* const mBody1;

        /// Pointer to the second body
        Body* const mBody2;

        /// Contact points in the manifold
        ContactPoint* mContactPoints[MAX_CONTACT_POINTS_IN_MANIFOLD];

        /// Number of contacts in the cache
        uint mNbContactPoints;

        /// First friction vector of the contact manifold
        Vector3 mFrictionVector1;

        /// Second friction vector of the contact manifold
        Vector3 mFrictionVector2;

        /// First friction constraint accumulated impulse
        decimal mFrictionImpulse1;

        /// Second friction constraint accumulated impulse
        decimal mFrictionImpulse2;

        /// Twist friction constraint accumulated impulse
        decimal mFrictionTwistImpulse;

        /// Reference to the memory pool with the contacts
        MemoryPool<ContactPoint>& mMemoryPoolContacts;

        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        ContactManifold(const ContactManifold& contactManifold);

        /// Private assignment operator
        ContactManifold& operator=(const ContactManifold& contactManifold);

        /// Return the index of maximum area
        int getMaxArea(decimal area0, decimal area1, decimal area2, decimal area3) const;

        /// Return the index of the contact with the larger penetration depth.
        int getIndexOfDeepestPenetration(ContactPoint* newContact) const;

        /// Return the index that will be removed.
        int getIndexToRemove(int indexMaxPenetration, const Vector3& newPoint) const;

        /// Remove a contact point from the manifold
        void removeContactPoint(uint index);

        /// Return true if two vectors are approximatively equal
        bool isApproxEqual(const Vector3& vector1, const Vector3& vector2) const;
        
    public:

        // -------------------- Methods -------------------- //

        /// Constructor
        ContactManifold(Body* const mBody1, Body* const mBody2,
                        MemoryPool<ContactPoint>& mMemoryPoolContacts);

        /// Destructor
        ~ContactManifold();

        /// Add a contact point to the manifold
        void addContactPoint(ContactPoint* contact);

        /// Update the contact manifold.
        void update(const Transform& transform1, const Transform& transform2);

        /// Clear the contact manifold
        void clear();

        /// Return the number of contact points in the manifold
        uint getNbContactPoints() const;

        /// Return the first friction vector at the center of the contact manifold
        const Vector3& getFrictionVector1() const;

        /// set the first friction vector at the center of the contact manifold
        void setFrictionVector1(const Vector3& mFrictionVector1);

        /// Return the second friction vector at the center of the contact manifold
        const Vector3& getFrictionVector2() const;

        /// set the second friction vector at the center of the contact manifold
        void setFrictionVector2(const Vector3& mFrictionVector2);

        /// Return the first friction accumulated impulse
        decimal getFrictionImpulse1() const;

        /// Set the first friction accumulated impulse
        void setFrictionImpulse1(decimal frictionImpulse1);

        /// Return the second friction accumulated impulse
        decimal getFrictionImpulse2() const;

        /// Set the second friction accumulated impulse
        void setFrictionImpulse2(decimal frictionImpulse2);

        /// Return the friction twist accumulated impulse
        decimal getFrictionTwistImpulse() const;

        /// Set the friction twist accumulated impulse
        void setFrictionTwistImpulse(decimal frictionTwistImpulse);

        /// Return a contact point of the manifold
        ContactPoint* getContactPoint(uint index) const;
};

// Return the number of contact points in the manifold
inline uint ContactManifold::getNbContactPoints() const {
    return mNbContactPoints;
}

// Return the first friction vector at the center of the contact manifold
inline const Vector3& ContactManifold::getFrictionVector1() const {
    return mFrictionVector1;
}

// set the first friction vector at the center of the contact manifold
inline void ContactManifold::setFrictionVector1(const Vector3& frictionVector1) {
    mFrictionVector1 = frictionVector1;
}

// Return the second friction vector at the center of the contact manifold
inline const Vector3& ContactManifold::getFrictionVector2() const {
    return mFrictionVector2;
}

// set the second friction vector at the center of the contact manifold
inline void ContactManifold::setFrictionVector2(const Vector3& frictionVector2) {
    mFrictionVector2 = frictionVector2;
}

// Return the first friction accumulated impulse
inline decimal ContactManifold::getFrictionImpulse1() const {
    return mFrictionImpulse1;
}

// Set the first friction accumulated impulse
inline void ContactManifold::setFrictionImpulse1(decimal frictionImpulse1) {
    mFrictionImpulse1 = frictionImpulse1;
}

// Return the second friction accumulated impulse
inline decimal ContactManifold::getFrictionImpulse2() const {
    return mFrictionImpulse2;
}

// Set the second friction accumulated impulse
inline void ContactManifold::setFrictionImpulse2(decimal frictionImpulse2) {
    mFrictionImpulse2 = frictionImpulse2;
}

// Return the friction twist accumulated impulse
inline decimal ContactManifold::getFrictionTwistImpulse() const {
    return mFrictionTwistImpulse;
}

// Set the friction twist accumulated impulse
inline void ContactManifold::setFrictionTwistImpulse(decimal frictionTwistImpulse) {
    mFrictionTwistImpulse = frictionTwistImpulse;
}

// Return a contact point of the manifold
inline ContactPoint* ContactManifold::getContactPoint(uint index) const {
    assert(index >= 0 && index < mNbContactPoints);
    return mContactPoints[index];
}

}
#endif

