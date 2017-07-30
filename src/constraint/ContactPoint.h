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

#ifndef REACTPHYSICS3D_CONTACT_POINT_H
#define REACTPHYSICS3D_CONTACT_POINT_H

// Libraries
#include "body/CollisionBody.h"
#include "collision/NarrowPhaseInfo.h"
#include "collision/ContactPointInfo.h"
#include "configuration.h"
#include "mathematics/mathematics.h"

/// ReactPhysics3D namespace
namespace reactphysics3d {

// Class ContactPoint
/**
 * This class represents a collision contact point between two
 * bodies in the physics engine.
 */
class ContactPoint {

    private :

        // -------------------- Attributes -------------------- //

        /// Normalized normal vector of the contact (from body1 toward body2) in world space
        Vector3 mNormal;

        /// Penetration depth
        decimal mPenetrationDepth;

        /// Contact point on body 1 in local space of body 1
        Vector3 mLocalPointOnBody1;

        /// Contact point on body 2 in local space of body 2
        Vector3 mLocalPointOnBody2;

        /// Contact point on body 1 in world space
        Vector3 mWorldPointOnBody1;

        /// Contact point on body 2 in world space
        Vector3 mWorldPointOnBody2;

        /// True if the contact is a resting contact (exists for more than one time step)
        bool mIsRestingContact;

        /// Cached penetration impulse
        decimal mPenetrationImpulse;

        /// True if the contact point is obselete
        bool mIsObselete;

        /// Pointer to the next contact point in the linked-list
        ContactPoint* mNext;

        // -------------------- Methods -------------------- //

        /// Update the contact point with a new one that is similar (very close)
        void update(const ContactPointInfo* contactInfo, const Transform& body1Transform,
                    const Transform& body2Transform);

        /// Return true if the contact point is similar (close enougth) to another given contact point
        bool isSimilarWithContactPoint(const ContactPointInfo* contactPoint) const;

        /// Set the cached penetration impulse
        void setPenetrationImpulse(decimal impulse);


        /// Set the mIsRestingContact variable
        void setIsRestingContact(bool isRestingContact);

        /// Set to true to make the contact point obselete
        void setIsObselete(bool isObselete);

        /// Set the next contact point in the linked list
        void setNext(ContactPoint* next);

        /// Return true if the contact point is obselete
        bool getIsObselete() const;

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        ContactPoint(const ContactPointInfo* contactInfo, const Transform& body1Transform,
                     const Transform& body2Transform);

        /// Destructor
        ~ContactPoint() = default;

        /// Deleted copy-constructor
        ContactPoint(const ContactPoint& contact) = delete;

        /// Deleted assignment operator
        ContactPoint& operator=(const ContactPoint& contact) = delete;

        /// Return the normal vector of the contact
        Vector3 getNormal() const;

        /// Return the contact local point on body 1
        Vector3 getLocalPointOnBody1() const;

        /// Return the contact local point on body 2
        Vector3 getLocalPointOnBody2() const;

        /// Return the contact world point on body 1
        Vector3 getWorldPointOnBody1() const;

        /// Return the contact world point on body 2
        Vector3 getWorldPointOnBody2() const;

        /// Return the cached penetration impulse
        decimal getPenetrationImpulse() const;

        /// Return true if the contact is a resting contact
        bool getIsRestingContact() const;

        /// Return the next contact point in the linked list
        ContactPoint* getNext() const;

        /// Return the penetration depth
        decimal getPenetrationDepth() const;

        /// Return the number of bytes used by the contact point
        size_t getSizeInBytes() const;

        // Friendship
        friend class ContactManifold;
        friend class ContactManifoldSet;
        friend class ContactSolver;
};

// Return the normal vector of the contact
inline Vector3 ContactPoint::getNormal() const {
    return mNormal;
}

// Return the contact point on body 1
inline Vector3 ContactPoint::getLocalPointOnBody1() const {
    return mLocalPointOnBody1;
}

// Return the contact point on body 2
inline Vector3 ContactPoint::getLocalPointOnBody2() const {
    return mLocalPointOnBody2;
}

// Return the contact world point on body 1
inline Vector3 ContactPoint::getWorldPointOnBody1() const {
    return mWorldPointOnBody1;
}

// Return the contact world point on body 2
inline Vector3 ContactPoint::getWorldPointOnBody2() const {
    return mWorldPointOnBody2;
}

// Return the cached penetration impulse
inline decimal ContactPoint::getPenetrationImpulse() const {
    return mPenetrationImpulse;
}

// Return true if the contact point is similar (close enougth) to another given contact point
inline bool ContactPoint::isSimilarWithContactPoint(const ContactPointInfo* localContactPointBody1) const {
    return (localContactPointBody1->localPoint1 - mLocalPointOnBody1).lengthSquare() <= (PERSISTENT_CONTACT_DIST_THRESHOLD *
            PERSISTENT_CONTACT_DIST_THRESHOLD);
}

// Set the cached penetration impulse
inline void ContactPoint::setPenetrationImpulse(decimal impulse) {
    mPenetrationImpulse = impulse;
}

// Return true if the contact is a resting contact
inline bool ContactPoint::getIsRestingContact() const {
    return mIsRestingContact;
}

// Set the mIsRestingContact variable
inline void ContactPoint::setIsRestingContact(bool isRestingContact) {
    mIsRestingContact = isRestingContact;
}

// Return true if the contact point is obselete
inline bool ContactPoint::getIsObselete() const {
    return mIsObselete;
}

// Set to true to make the contact point obselete
inline void ContactPoint::setIsObselete(bool isObselete) {
    mIsObselete = isObselete;
}

// Return the next contact point in the linked list
inline ContactPoint* ContactPoint::getNext() const {
   return mNext;
}

// Set the next contact point in the linked list
inline void ContactPoint::setNext(ContactPoint* next) {
    mNext = next;
}

// Return the penetration depth of the contact
inline decimal ContactPoint::getPenetrationDepth() const {
    return mPenetrationDepth;
}

// Return the number of bytes used by the contact point
inline size_t ContactPoint::getSizeInBytes() const {
    return sizeof(ContactPoint);
}

}

#endif
