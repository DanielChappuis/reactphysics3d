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
#include "collision/CollisionShapeInfo.h"
#include "configuration.h"
#include "mathematics/mathematics.h"
#include "configuration.h"

/// ReactPhysics3D namespace
namespace reactphysics3d {

// Structure ContactPointInfo
/**
 * This structure contains informations about a collision contact
 * computed during the narrow-phase collision detection. Those
 * informations are used to compute the contact set for a contact
 * between two bodies.
 */
struct ContactPointInfo {

    private:

        // -------------------- Methods -------------------- //

    public:

        // -------------------- Attributes -------------------- //

        // TODO : Check if we really need the shape1, shape2, collisionShape1 and collisionShape2 fields

        /// First proxy shape of the contact
        ProxyShape* shape1;

        /// Second proxy shape of the contact
        ProxyShape* shape2;

        /// First collision shape
        const CollisionShape* collisionShape1;

        /// Second collision shape
        const CollisionShape* collisionShape2;

        /// Normalized normal vector of the collision contact in world space
        Vector3 normal;

        /// Penetration depth of the contact
        decimal penetrationDepth;

        /// Contact point of body 1 in local space of body 1
        Vector3 localPoint1;

        /// Contact point of body 2 in local space of body 2
        Vector3 localPoint2;

        // -------------------- Methods -------------------- //

        /// Constructor
        ContactPointInfo(ProxyShape* proxyShape1, ProxyShape* proxyShape2, const CollisionShape* collShape1,
                         const CollisionShape* collShape2, const Vector3& normal, decimal penetrationDepth,
                         const Vector3& localPoint1, const Vector3& localPoint2)
            : shape1(proxyShape1), shape2(proxyShape2), collisionShape1(collShape1), collisionShape2(collShape2),
              normal(normal), penetrationDepth(penetrationDepth), localPoint1(localPoint1),
              localPoint2(localPoint2) {

        }
};

// Class ContactPoint
/**
 * This class represents a collision contact point between two
 * bodies in the physics engine.
 */
class ContactPoint {

    private :

        // -------------------- Attributes -------------------- //

        /// Normalized normal vector of the contact (from body1 toward body2) in world space
        const Vector3 mNormal;

        /// Penetration depth
        decimal mPenetrationDepth;

        /// Contact point on body 1 in local space of body 1
        const Vector3 mLocalPointOnBody1;

        /// Contact point on body 2 in local space of body 2
        const Vector3 mLocalPointOnBody2;

        /// Contact point on body 1 in world space
        Vector3 mWorldPointOnBody1;

        /// Contact point on body 2 in world space
        Vector3 mWorldPointOnBody2;

        /// True if the contact is a resting contact (exists for more than one time step)
        bool mIsRestingContact;

        /// Cached penetration impulse
        decimal mPenetrationImpulse;

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        ContactPoint(const ContactPointInfo& contactInfo);

        /// Destructor
        ~ContactPoint() = default;

        /// Deleted copy-constructor
        ContactPoint(const ContactPoint& contact) = delete;

        /// Deleted assignment operator
        ContactPoint& operator=(const ContactPoint& contact) = delete;

        /// Update the world contact points
        void updateWorldContactPoints(const Transform& body1Transform, const Transform& body2Transform);

        /// Update the penetration depth
        void updatePenetrationDepth();

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

        /// Set the cached penetration impulse
        void setPenetrationImpulse(decimal impulse);

        /// Return true if the contact is a resting contact
        bool getIsRestingContact() const;

        /// Set the mIsRestingContact variable
        void setIsRestingContact(bool isRestingContact);

        /// Return the penetration depth
        decimal getPenetrationDepth() const;

        /// Return the number of bytes used by the contact point
        size_t getSizeInBytes() const;
};

// Update the world contact points
inline void ContactPoint::updateWorldContactPoints(const Transform& body1Transform, const Transform& body2Transform) {
    mWorldPointOnBody1 = body1Transform * mLocalPointOnBody1;
    mWorldPointOnBody2 = body2Transform * mLocalPointOnBody2;
}

// Update the penetration depth
inline void ContactPoint::updatePenetrationDepth() {
    mPenetrationDepth = (mWorldPointOnBody1 - mWorldPointOnBody2).dot(mNormal);
}

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
