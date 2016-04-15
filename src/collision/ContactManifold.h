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

#ifndef REACTPHYSICS3D_CONTACT_MANIFOLD_H
#define	REACTPHYSICS3D_CONTACT_MANIFOLD_H

// Libraries
#include <vector>
#include "body/CollisionBody.h"
#include "collision/ProxyShape.h"
#include "constraint/ContactPoint.h"
#include "memory/MemoryAllocator.h"

/// ReactPhysics3D namespace
namespace reactphysics3d {

// Constants
const uint MAX_CONTACT_POINTS_IN_MANIFOLD = 4;   // Maximum number of contacts in the manifold

// Class declarations
class ContactManifold;

// Structure ContactManifoldListElement
/**
 * This structure represents a single element of a linked list of contact manifolds
 */
struct ContactManifoldListElement {

    public:

        // -------------------- Attributes -------------------- //

        /// Pointer to the actual contact manifold
        ContactManifold* contactManifold;

        /// Next element of the list
        ContactManifoldListElement* next;

        // -------------------- Methods -------------------- //

        /// Constructor
        ContactManifoldListElement(ContactManifold* initContactManifold,
                                   ContactManifoldListElement* initNext)
                                  :contactManifold(initContactManifold), next(initNext) {

        }
};

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

        /// Pointer to the first proxy shape of the contact
        ProxyShape* mShape1;

        /// Pointer to the second proxy shape of the contact
        ProxyShape* mShape2;

        /// Contact points in the manifold
        ContactPoint* mContactPoints[MAX_CONTACT_POINTS_IN_MANIFOLD];

        /// Normal direction Id (Unique Id representing the normal direction)
        short int mNormalDirectionId;

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

        /// Accumulated rolling resistance impulse
        Vector3 mRollingResistanceImpulse;

        /// True if the contact manifold has already been added into an island
        bool mIsAlreadyInIsland;

        /// Reference to the memory allocator
        MemoryAllocator& mMemoryAllocator;

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

        /// Return true if the contact manifold has already been added into an island
        bool isAlreadyInIsland() const;
        
    public:

        // -------------------- Methods -------------------- //

        /// Constructor
        ContactManifold(ProxyShape* shape1, ProxyShape* shape2,
                        MemoryAllocator& memoryAllocator, short int normalDirectionId);

        /// Destructor
        ~ContactManifold();

        /// Return a pointer to the first proxy shape of the contact
        ProxyShape* getShape1() const;

        /// Return a pointer to the second proxy shape of the contact
        ProxyShape* getShape2() const;

        /// Return a pointer to the first body of the contact manifold
        CollisionBody* getBody1() const;

        /// Return a pointer to the second body of the contact manifold
        CollisionBody* getBody2() const;

        /// Return the normal direction Id
        short int getNormalDirectionId() const;

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

        /// Set the accumulated rolling resistance impulse
        void setRollingResistanceImpulse(const Vector3& rollingResistanceImpulse);

        /// Return a contact point of the manifold
        ContactPoint* getContactPoint(uint index) const;

        /// Return the normalized averaged normal vector
        Vector3 getAverageContactNormal() const;

        /// Return the largest depth of all the contact points
        decimal getLargestContactDepth() const;

        // -------------------- Friendship -------------------- //

        friend class DynamicsWorld;
        friend class Island;
        friend class CollisionBody;
};

// Return a pointer to the first proxy shape of the contact
inline ProxyShape* ContactManifold::getShape1() const {
    return mShape1;
}

// Return a pointer to the second proxy shape of the contact
inline ProxyShape* ContactManifold::getShape2() const {
    return mShape2;
}

// Return a pointer to the first body of the contact manifold
inline CollisionBody* ContactManifold::getBody1() const {
    return mShape1->getBody();
}

// Return a pointer to the second body of the contact manifold
inline CollisionBody* ContactManifold::getBody2() const {
    return mShape2->getBody();
}

// Return the normal direction Id
inline short int ContactManifold::getNormalDirectionId() const {
    return mNormalDirectionId;
}

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

// Set the accumulated rolling resistance impulse
inline void ContactManifold::setRollingResistanceImpulse(const Vector3& rollingResistanceImpulse) {
    mRollingResistanceImpulse = rollingResistanceImpulse;
}

// Return a contact point of the manifold
inline ContactPoint* ContactManifold::getContactPoint(uint index) const {
    assert(index < mNbContactPoints);
    return mContactPoints[index];
}

// Return true if the contact manifold has already been added into an island
inline bool ContactManifold::isAlreadyInIsland() const {
    return mIsAlreadyInIsland;
}

// Return the normalized averaged normal vector
inline Vector3 ContactManifold::getAverageContactNormal() const {
    Vector3 averageNormal;

    for (uint i=0; i<mNbContactPoints; i++) {
        averageNormal += mContactPoints[i]->getNormal();
    }

    return averageNormal.getUnit();
}

// Return the largest depth of all the contact points
inline decimal ContactManifold::getLargestContactDepth() const {
    decimal largestDepth = 0.0f;

    for (uint i=0; i<mNbContactPoints; i++) {
        decimal depth = mContactPoints[i]->getPenetrationDepth();
        if (depth > largestDepth) {
            largestDepth = depth;
        }
    }

    return largestDepth;
}

}
#endif

