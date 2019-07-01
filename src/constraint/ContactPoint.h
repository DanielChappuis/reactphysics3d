/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2019 Daniel Chappuis                                       *
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
#include "configuration.h"
#include "mathematics/mathematics.h"
#include "collision/ContactPointInfo.h"

/// ReactPhysics3D namespace
namespace reactphysics3d {

// Declarations
class CollisionBody;

struct NarrowPhaseInfo;

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

        /// Contact point on proxy shape 1 in local-space of proxy shape 1
        Vector3 mLocalPointOnShape1;

        /// Contact point on proxy shape 2 in local-space of proxy shape 2
        Vector3 mLocalPointOnShape2;

        /// True if the contact is a resting contact (exists for more than one time step)
        bool mIsRestingContact;

        /// Cached penetration impulse
        decimal mPenetrationImpulse;

        /// True if the contact point is obsolete
        bool mIsObsolete;

        /// Pointer to the next contact point in the double linked-list
        ContactPoint* mNext;

        /// Pointer to the previous contact point in the double linked-list
        ContactPoint* mPrevious;

        /// World settings
        const WorldSettings& mWorldSettings;

        // -------------------- Methods -------------------- //

        /// Update the contact point with a new one that is similar (very close)
        void update(const ContactPointInfo* contactInfo);

        /// Return true if the contact point is similar (close enougth) to another given contact point
        bool isSimilarWithContactPoint(const ContactPointInfo* contactPoint) const;

        /// Set the cached penetration impulse
        void setPenetrationImpulse(decimal impulse);


        /// Set the mIsRestingContact variable
        void setIsRestingContact(bool isRestingContact);

        /// Set to true to make the contact point obsolete
        void setIsObsolete(bool isObselete);

        /// Set the next contact point in the linked list
        void setNext(ContactPoint* next);

        /// Set the previous contact point in the linked list
        void setPrevious(ContactPoint* previous);

        /// Return true if the contact point is obsolete
        bool getIsObsolete() const;

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        ContactPoint(const ContactPointInfo* contactInfo, const WorldSettings& worldSettings);

        /// Destructor
        ~ContactPoint() = default;

        /// Deleted copy-constructor
        ContactPoint(const ContactPoint& contact) = delete;

        /// Deleted assignment operator
        ContactPoint& operator=(const ContactPoint& contact) = delete;

        /// Return the normal vector of the contact
        Vector3 getNormal() const;

        /// Return the contact point on the first proxy shape in the local-space of the proxy shape
        const Vector3& getLocalPointOnShape1() const;

        /// Return the contact point on the second proxy shape in the local-space of the proxy shape
        const Vector3& getLocalPointOnShape2() const;

        /// Return the cached penetration impulse
        decimal getPenetrationImpulse() const;

        /// Return true if the contact is a resting contact
        bool getIsRestingContact() const;

        /// Return the previous contact point in the linked list
        inline ContactPoint* getPrevious() const;

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
/**
 * @return The contact normal
 */
inline Vector3 ContactPoint::getNormal() const {
    return mNormal;
}

// Return the contact point on the first proxy shape in the local-space of the proxy shape
/**
 * @return The contact point on the first proxy shape in the local-space of the proxy shape
 */
inline const Vector3& ContactPoint::getLocalPointOnShape1() const {
    return mLocalPointOnShape1;
}

// Return the contact point on the second proxy shape in the local-space of the proxy shape
/**
 * @return The contact point on the second proxy shape in the local-space of the proxy shape
 */
inline const Vector3& ContactPoint::getLocalPointOnShape2() const {
    return mLocalPointOnShape2;
}

// Return the cached penetration impulse
/**
 * @return The penetration impulse
 */
inline decimal ContactPoint::getPenetrationImpulse() const {
    return mPenetrationImpulse;
}

// Return true if the contact point is similar (close enougth) to another given contact point
inline bool ContactPoint::isSimilarWithContactPoint(const ContactPointInfo* localContactPointBody1) const {
    return (localContactPointBody1->localPoint1 - mLocalPointOnShape1).lengthSquare() <= (mWorldSettings.persistentContactDistanceThreshold *
            mWorldSettings.persistentContactDistanceThreshold);
}

// Set the cached penetration impulse
/**
 * @param impulse Penetration impulse
 */
inline void ContactPoint::setPenetrationImpulse(decimal impulse) {
    mPenetrationImpulse = impulse;
}

// Return true if the contact is a resting contact
/**
 * @return True if it is a resting contact
 */
inline bool ContactPoint::getIsRestingContact() const {
    return mIsRestingContact;
}

// Set the mIsRestingContact variable
/**
 * @param isRestingContact True if it is a resting contact
 */
inline void ContactPoint::setIsRestingContact(bool isRestingContact) {
    mIsRestingContact = isRestingContact;
}

// Return true if the contact point is obsolete
/**
 * @return True if the contact is obsolete
 */
inline bool ContactPoint::getIsObsolete() const {
    return mIsObsolete;
}

// Set to true to make the contact point obsolete
/**
 * @param isObsolete True if the contact is obsolete
 */
inline void ContactPoint::setIsObsolete(bool isObsolete) {
    mIsObsolete = isObsolete;
}

// Return the next contact point in the linked list
/**
 * @return A pointer to the next contact point in the linked-list of points
 */
inline ContactPoint* ContactPoint::getNext() const {
   return mNext;
}

// Set the next contact point in the linked list
/**
 * @param next Pointer to the next contact point in the linked-list of points
 */
inline void ContactPoint::setNext(ContactPoint* next) {
    mNext = next;
}

// Return the previous contact point in the linked list
/**
 * @return A pointer to the previous contact point in the linked-list of points
 */
inline ContactPoint* ContactPoint::getPrevious() const {
   return mPrevious;
}

// Set the previous contact point in the linked list
/**
 * @param previous Pointer to the previous contact point in the linked-list of points
 */
inline void ContactPoint::setPrevious(ContactPoint* previous) {
    mPrevious = previous;
}

// Return the penetration depth of the contact
/**
 * @return the penetration depth (in meters)
 */
inline decimal ContactPoint::getPenetrationDepth() const {
    return mPenetrationDepth;
}

// Return the number of bytes used by the contact point
/**
 * @return The size of the contact point in memory (in bytes)
 */
inline size_t ContactPoint::getSizeInBytes() const {
    return sizeof(ContactPoint);
}

}

#endif
