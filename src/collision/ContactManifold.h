/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2018 Daniel Chappuis                                       *
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
#include "collision/ProxyShape.h"

/// ReactPhysics3D namespace
namespace reactphysics3d {

// Class declarations
class ContactManifold;
class ContactManifoldInfo;
struct ContactPointInfo;
class CollisionBody;
class ContactPoint;
class DefaultPoolAllocator;

// Structure ContactManifoldListElement
/**
 * This structure represents a single element of a linked list of contact manifolds
 */
struct ContactManifoldListElement {

    private:

        // -------------------- Attributes -------------------- //

        /// Pointer to a contact manifold with contact points
        ContactManifold* mContactManifold;

        /// Next element of the list
        ContactManifoldListElement* mNext;

   public:

        // -------------------- Methods -------------------- //

        /// Constructor
        ContactManifoldListElement(ContactManifold* contactManifold,
                                   ContactManifoldListElement* next)
                                  :mContactManifold(contactManifold), mNext(next) {

        }

        /// Return the contact manifold
        ContactManifold* getContactManifold() {
            return mContactManifold;
        }

        /// Return the next element in the linked-list
        ContactManifoldListElement* getNext() {
            return mNext;
        }
};

// Class ContactManifold
/**
 * This class represents a set of contact points between two bodies that
 * all have a similar contact normal direction. Usually, there is a single
 * contact manifold when two convex shapes are in contact. However, when
 * a convex shape collides with a concave shape, there might be several
 * contact manifolds with different normal directions.
 * The contact manifold is implemented in a way to cache the contact
 * points among the frames for better stability (warm starting of the
 * contact solver)
 */
class ContactManifold {

    private:

        // -------------------- Attributes -------------------- //

        /// Pointer to the first proxy shape of the contact
        ProxyShape* mShape1;

        /// Pointer to the second proxy shape of the contact
        ProxyShape* mShape2;

        /// Contact points in the manifold
        ContactPoint* mContactPoints;

        /// Number of contacts in the cache
        int8 mNbContactPoints;

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

        /// Pointer to the next contact manifold in the linked-list
        ContactManifold* mNext;

        /// Pointer to the previous contact manifold in linked-list
        ContactManifold* mPrevious;

        /// True if the contact manifold is obsolete
        bool mIsObsolete;

        /// World settings
        const WorldSettings& mWorldSettings;

        // -------------------- Methods -------------------- //

        /// Return true if the contact manifold has already been added into an island
        bool isAlreadyInIsland() const;

        /// Set the pointer to the next element in the linked-list
        void setNext(ContactManifold* nextManifold);

        /// Return true if the manifold is obsolete
        bool getIsObsolete() const;

        /// Set to true to make the manifold obsolete
        void setIsObsolete(bool isObselete, bool setContactPoints);

        /// Clear the obsolete contact points
        void clearObsoleteContactPoints();

        /// Return the contact normal direction Id of the manifold
        short getContactNormalId() const;

        /// Return the largest depth of all the contact points
        decimal getLargestContactDepth() const;

        /// set the first friction vector at the center of the contact manifold
        void setFrictionVector1(const Vector3& mFrictionVector1);

        /// set the second friction vector at the center of the contact manifold
        void setFrictionVector2(const Vector3& mFrictionVector2);

        /// Set the first friction accumulated impulse
        void setFrictionImpulse1(decimal frictionImpulse1);

        /// Set the second friction accumulated impulse
        void setFrictionImpulse2(decimal frictionImpulse2);

        /// Add a contact point
        void addContactPoint(const ContactPointInfo* contactPointInfo);

        /// Make sure we do not have too much contact points by keeping only the best ones
        void reduce();

        /// Remove a contact point that is not optimal (with a small penetration depth)
        void removeNonOptimalContactPoint();

        /// Remove a contact point
        void removeContactPoint(ContactPoint* contactPoint);

        /// Set the friction twist accumulated impulse
        void setFrictionTwistImpulse(decimal frictionTwistImpulse);

        /// Set the accumulated rolling resistance impulse
        void setRollingResistanceImpulse(const Vector3& rollingResistanceImpulse);

        /// Set the pointer to the previous element in the linked-list
        void setPrevious(ContactManifold* previousManifold);

        /// Return the first friction vector at the center of the contact manifold
        const Vector3& getFrictionVector1() const;

        /// Return the second friction vector at the center of the contact manifold
        const Vector3& getFrictionVector2() const;

        /// Return the first friction accumulated impulse
        decimal getFrictionImpulse1() const;

        /// Return the second friction accumulated impulse
        decimal getFrictionImpulse2() const;

        /// Return the friction twist accumulated impulse
        decimal getFrictionTwistImpulse() const;

    public:

        // -------------------- Methods -------------------- //

        /// Constructor
        ContactManifold(const ContactManifoldInfo* manifoldInfo, ProxyShape* shape1, ProxyShape* shape2,
                        MemoryAllocator& memoryAllocator, const WorldSettings& worldSettings);

        /// Destructor
        ~ContactManifold();

        /// Deleted copy-constructor
        ContactManifold(const ContactManifold& contactManifold) = delete;

        /// Deleted assignment operator
        ContactManifold& operator=(const ContactManifold& contactManifold) = delete;

        /// Return a pointer to the first proxy shape of the contact
        ProxyShape* getShape1() const;

        /// Return a pointer to the second proxy shape of the contact
        ProxyShape* getShape2() const;

        /// Return a pointer to the first body of the contact manifold
        CollisionBody* getBody1() const;

        /// Return a pointer to the second body of the contact manifold
        CollisionBody* getBody2() const;

        /// Return the number of contact points in the manifold
        int8 getNbContactPoints() const;

        /// Return a pointer to the first contact point of the manifold
        ContactPoint* getContactPoints() const;

        /// Return a pointer to the previous element in the linked-list
        ContactManifold* getPrevious() const;

        /// Return a pointer to the next element in the linked-list
        ContactManifold* getNext() const;

        // -------------------- Friendship -------------------- //

        friend class DynamicsWorld;
        friend class Island;
        friend class CollisionBody;
        friend class ContactManifoldSet;
        friend class ContactSolver;
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

// Return the number of contact points in the manifold
inline int8 ContactManifold::getNbContactPoints() const {
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

// Return a pointer to the first contact point of the manifold
inline ContactPoint* ContactManifold::getContactPoints() const {
    return mContactPoints;
}

// Return true if the contact manifold has already been added into an island
inline bool ContactManifold::isAlreadyInIsland() const {
    return mIsAlreadyInIsland;
}

// Return a pointer to the previous element in the linked-list
inline ContactManifold* ContactManifold::getPrevious() const {
    return mPrevious;
}

// Set the pointer to the previous element in the linked-list
inline void ContactManifold::setPrevious(ContactManifold* previousManifold) {
    mPrevious = previousManifold;
}

// Return a pointer to the next element in the linked-list
inline ContactManifold* ContactManifold::getNext() const {
    return mNext;
}

// Set the pointer to the next element in the linked-list
inline void ContactManifold::setNext(ContactManifold* nextManifold) {
    mNext = nextManifold;
}

// Return true if the manifold is obsolete
inline bool ContactManifold::getIsObsolete() const {
    return mIsObsolete;
}

}

#endif

