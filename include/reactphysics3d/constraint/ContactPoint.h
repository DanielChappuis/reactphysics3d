/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2020 Daniel Chappuis                                       *
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
#include <reactphysics3d/configuration.h>
#include <reactphysics3d/mathematics/mathematics.h>
#include <reactphysics3d/collision/ContactPointInfo.h>

/// ReactPhysics3D namespace
namespace reactphysics3d {

// Declarations
class CollisionBody;

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

        /// Contact point on collider 1 in local-space of collider 1
        Vector3 mLocalPointOnShape1;

        /// Contact point on collider 2 in local-space of collider 2
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

        /// Persistent contact distance threshold;
        decimal mPersistentContactDistanceThreshold;

        // -------------------- Methods -------------------- //

        /// Update the contact point with a new one that is similar (very close)
        void update(const ContactPointInfo* contactInfo);

        /// Return true if the contact point is similar (close enougth) to another given contact point
        bool isSimilarWithContactPoint(const ContactPointInfo* contactPoint) const;

        /// Set the cached penetration impulse
        void setPenetrationImpulse(decimal impulse);


        /// Set the mIsRestingContact variable
        void setIsRestingContact(bool isRestingContact);

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        ContactPoint(const ContactPointInfo* contactInfo, decimal persistentContactDistanceThreshold);

        /// Constructor
        ContactPoint(const ContactPointInfo& contactInfo, decimal persistentContactDistanceThreshold);

        /// Destructor
        ~ContactPoint() = default;

        /// Copy-constructor
        ContactPoint(const ContactPoint& contact) = default;

        /// Assignment operator
        ContactPoint& operator=(const ContactPoint& contact) = default;

        /// Return the normal vector of the contact
        const Vector3& getNormal() const;

        /// Return the contact point on the first collider in the local-space of the collider
        const Vector3& getLocalPointOnShape1() const;

        /// Return the contact point on the second collider in the local-space of the collider
        const Vector3& getLocalPointOnShape2() const;

        /// Return the cached penetration impulse
        decimal getPenetrationImpulse() const;

        /// Return true if the contact is a resting contact
        bool getIsRestingContact() const;

        /// Return the penetration depth
        decimal getPenetrationDepth() const;

        /// Return the number of bytes used by the contact point
        size_t getSizeInBytes() const;

        // Friendship
        friend class ContactManifold;
        friend class ContactManifoldSet;
        friend class ContactSolverSystem;
        friend class CollisionDetectionSystem;
};

// Return the normal vector of the contact
/**
 * @return The contact normal
 */
inline const Vector3& ContactPoint::getNormal() const {
    return mNormal;
}

// Return the contact point on the first collider in the local-space of the collider
/**
 * @return The contact point on the first collider in the local-space of the collider
 */
inline const Vector3& ContactPoint::getLocalPointOnShape1() const {
    return mLocalPointOnShape1;
}

// Return the contact point on the second collider in the local-space of the collider
/**
 * @return The contact point on the second collider in the local-space of the collider
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
    return (localContactPointBody1->localPoint1 - mLocalPointOnShape1).lengthSquare() <= (mPersistentContactDistanceThreshold *
            mPersistentContactDistanceThreshold);
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
