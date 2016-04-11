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

        /// First rigid body of the contact
        CollisionBody* mBody1;

        /// Second rigid body of the contact
        CollisionBody* mBody2;

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

        /// Two orthogonal vectors that span the tangential friction plane
        Vector3 mFrictionVectors[2];

        /// Cached penetration impulse
        decimal mPenetrationImpulse;

        /// Cached first friction impulse
        decimal mFrictionImpulse1;

        /// Cached second friction impulse
        decimal mFrictionImpulse2;

        /// Cached rolling resistance impulse
        Vector3 mRollingResistanceImpulse;
        
        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        ContactPoint(const ContactPoint& contact);

        /// Private assignment operator
        ContactPoint& operator=(const ContactPoint& contact);

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        ContactPoint(const ContactPointInfo& contactInfo);

        /// Destructor
        ~ContactPoint();

        /// Return the reference to the body 1
        CollisionBody* getBody1() const;

        /// Return the reference to the body 2
        CollisionBody* getBody2() const;

        /// Return the normal vector of the contact
        Vector3 getNormal() const;

        /// Set the penetration depth of the contact
        void setPenetrationDepth(decimal penetrationDepth);

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

        /// Return the cached first friction impulse
        decimal getFrictionImpulse1() const;

        /// Return the cached second friction impulse
        decimal getFrictionImpulse2() const;

        /// Return the cached rolling resistance impulse
        Vector3 getRollingResistanceImpulse() const;

        /// Set the cached penetration impulse
        void setPenetrationImpulse(decimal impulse);

        /// Set the first cached friction impulse
        void setFrictionImpulse1(decimal impulse);

        /// Set the second cached friction impulse
        void setFrictionImpulse2(decimal impulse);

        /// Set the cached rolling resistance impulse
        void setRollingResistanceImpulse(const Vector3& impulse);

        /// Set the contact world point on body 1
        void setWorldPointOnBody1(const Vector3& worldPoint);

        /// Set the contact world point on body 2
        void setWorldPointOnBody2(const Vector3& worldPoint);

        /// Return true if the contact is a resting contact
        bool getIsRestingContact() const;

        /// Set the mIsRestingContact variable
        void setIsRestingContact(bool isRestingContact);

        /// Get the first friction vector
        Vector3 getFrictionVector1() const;

        /// Set the first friction vector
        void setFrictionVector1(const Vector3& frictionVector1);

        /// Get the second friction vector
        Vector3 getFrictionVector2() const;

        /// Set the second friction vector
        void setFrictionVector2(const Vector3& frictionVector2);

        /// Return the penetration depth
        decimal getPenetrationDepth() const;

        /// Return the number of bytes used by the contact point
        size_t getSizeInBytes() const;
};

// Return the reference to the body 1
inline CollisionBody* ContactPoint::getBody1() const {
    return mBody1;
}

// Return the reference to the body 2
inline CollisionBody* ContactPoint::getBody2() const {
    return mBody2;
}

// Return the normal vector of the contact
inline Vector3 ContactPoint::getNormal() const {
    return mNormal;
}

// Set the penetration depth of the contact
inline void ContactPoint::setPenetrationDepth(decimal penetrationDepth) {
    this->mPenetrationDepth = penetrationDepth;
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

// Return the cached first friction impulse
inline decimal ContactPoint::getFrictionImpulse1() const {
    return mFrictionImpulse1;
}

// Return the cached second friction impulse
inline decimal ContactPoint::getFrictionImpulse2() const {
    return mFrictionImpulse2;
}

// Return the cached rolling resistance impulse
inline Vector3 ContactPoint::getRollingResistanceImpulse() const {
    return mRollingResistanceImpulse;
}

// Set the cached penetration impulse
inline void ContactPoint::setPenetrationImpulse(decimal impulse) {
    mPenetrationImpulse = impulse;
}

// Set the first cached friction impulse
inline void ContactPoint::setFrictionImpulse1(decimal impulse) {
    mFrictionImpulse1 = impulse;
}

// Set the second cached friction impulse
inline void ContactPoint::setFrictionImpulse2(decimal impulse) {
    mFrictionImpulse2 = impulse;
}

// Set the cached rolling resistance impulse
inline void ContactPoint::setRollingResistanceImpulse(const Vector3& impulse) {
    mRollingResistanceImpulse = impulse;
}

// Set the contact world point on body 1
inline void ContactPoint::setWorldPointOnBody1(const Vector3& worldPoint) {
    mWorldPointOnBody1 = worldPoint;
}

// Set the contact world point on body 2
inline void ContactPoint::setWorldPointOnBody2(const Vector3& worldPoint) {
    mWorldPointOnBody2 = worldPoint;
}

// Return true if the contact is a resting contact
inline bool ContactPoint::getIsRestingContact() const {
    return mIsRestingContact;
}

// Set the mIsRestingContact variable
inline void ContactPoint::setIsRestingContact(bool isRestingContact) {
    mIsRestingContact = isRestingContact;
}

// Get the first friction vector
inline Vector3 ContactPoint::getFrictionVector1() const {
    return mFrictionVectors[0];
}

// Set the first friction vector
inline void ContactPoint::setFrictionVector1(const Vector3& frictionVector1) {
    mFrictionVectors[0] = frictionVector1;
}

// Get the second friction vector
inline Vector3 ContactPoint::getFrictionVector2() const {
    return mFrictionVectors[1];
}

// Set the second friction vector
inline void ContactPoint::setFrictionVector2(const Vector3& frictionVector2) {
    mFrictionVectors[1] = frictionVector2;
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
