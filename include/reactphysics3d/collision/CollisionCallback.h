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

#ifndef REACTPHYSICS3D_COLLISION_CALLBACK_H
#define REACTPHYSICS3D_COLLISION_CALLBACK_H

// Libraries
#include <reactphysics3d/containers/List.h>
#include <reactphysics3d/collision/ContactPair.h>
#include <reactphysics3d/constraint/ContactPoint.h>

/// ReactPhysics3D namespace
namespace reactphysics3d {

// Declarations
class OverlappingPair;
class ContactManifold;
class CollisionBody;
class Collider;
class MemoryManager;

// Class CollisionCallback
/**
 * This abstract class can be used to register a callback for collision test queries.
 * You should implement your own class inherited from this one and implement
 * the notifyContact() method. This method will called each time a contact
 * point is reported.
 */
class CollisionCallback {

    public:

        // Class ContactPoint
        /**
         * This class represents a contact point between two colliders of the physics world.
         */
        class ContactPoint {

            private:

                // -------------------- Attributes -------------------- //

                const reactphysics3d::ContactPoint& mContactPoint;

                // -------------------- Methods -------------------- //

                /// Constructor
                ContactPoint(const reactphysics3d::ContactPoint& contactPoint);

            public:

                // -------------------- Methods -------------------- //

                /// Copy constructor
                ContactPoint(const ContactPoint& contactPoint) = default;

                /// Assignment operator
                ContactPoint& operator=(const ContactPoint& contactPoint) = delete;

                /// Destructor
                ~ContactPoint() = default;

                /// Return the penetration depth
                /**
                 * @return The penetration depth between the two colliders at this contact point
                 */
                decimal getPenetrationDepth() const;

                /// Return the world-space contact normal
                /**
                 * @return The world-space contact normal
                 */
                const Vector3& getWorldNormal() const;

                /// Return the contact point on the first collider in the local-space of the first collider
                /**
                 * @return The local-space contact point on the first collider
                 */
                const Vector3& getLocalPointOnCollider1() const;

                /// Return the contact point on the second collider in the local-space of the second collider
                /**
                 * @return The local-space contact point on the second collider
                 */
                const Vector3& getLocalPointOnCollider2() const;

                // -------------------- Friendship -------------------- //

                friend class CollisionCallback;
        };

        // Class ContactPair
        /**
         * This class represents the contact between two colliders of the physics world.
         * A contact pair contains a list of contact points.
         */
        class ContactPair {

            public:

                /// Enumeration EventType that describes the type of contact event
                enum class EventType {

                    /// This contact is a new contact between the two
                    /// colliders (the colliders where not touching in the previous frame)
                    ContactStart,

                    /// The two colliders were already touching in the previous frame and this is a new or updated contact
                    ContactStay,

                    /// The two colliders were in contact in the previous frame and are not in contact anymore
                    ContactExit
                };

            private:

                // -------------------- Attributes -------------------- //

                const reactphysics3d::ContactPair& mContactPair;

                /// Pointer to the contact points
                List<reactphysics3d::ContactPoint>* mContactPoints;

                /// Reference to the physics world
                PhysicsWorld& mWorld;

                /// True if this is a lost contact pair (contact pair colliding in previous frame but not in current one)
                bool mIsLostContactPair;

                // -------------------- Methods -------------------- //

                /// Constructor
                ContactPair(const reactphysics3d::ContactPair& contactPair, List<reactphysics3d::ContactPoint>* contactPoints,
                            PhysicsWorld& world, bool mIsLostContactPair);

            public:

                // -------------------- Methods -------------------- //

                /// Copy constructor
                ContactPair(const ContactPair& contactPair) = default;

                /// Assignment operator
                ContactPair& operator=(const ContactPair& contactPair) = delete;

                /// Destructor
                ~ContactPair() = default;

                /// Return the number of contact points in the contact pair
                /**
                 * @return The number of contact points in the contact pair
                 */
                uint getNbContactPoints() const;

                /// Return a given contact point
                /**
                 * @param index Index of the contact point to retrieve
                 * @return A contact point object
                 */
                ContactPoint getContactPoint(uint index) const;

                /// Return a pointer to the first body in contact
                /**
                 * @return A pointer to the first colliding body of the pair
                 */
                CollisionBody* getBody1() const;

                /// Return a pointer to the second body in contact
                /**
                 * @return A pointer to the second colliding body of the pair
                 */
                CollisionBody* getBody2() const;

                /// Return a pointer to the first collider in contact (in body 1)
                /**
                 * @return A pointer to the first collider of the contact pair
                 */
                Collider* getCollider1() const;

                /// Return a pointer to the second collider in contact (in body 2)
                /**
                 * @return A pointer to the second collider of the contact pair
                 */
                Collider* getCollider2() const;

                /// Return the corresponding type of event for this contact pair
                /**
                 * @return The type of contact event for this contact pair
                 */
                EventType getEventType() const;

                // -------------------- Friendship -------------------- //

                friend class CollisionCallback;
        };

        // Class CallbackData
        /**
         * This class contains data about contacts between bodies
         */
        class CallbackData {

            private:

                // -------------------- Attributes -------------------- //

                /// Pointer to the list of contact pairs (contains contacts and triggers events)
                List<reactphysics3d::ContactPair>* mContactPairs;

                /// Pointer to the list of contact manifolds
                List<ContactManifold>* mContactManifolds;

                /// Pointer to the contact points
                List<reactphysics3d::ContactPoint>* mContactPoints;

                /// Pointer to the list of lost contact pairs (contains contacts and triggers events)
                List<reactphysics3d::ContactPair>& mLostContactPairs;

                /// List of indices of the mContactPairs list that are contact events (not overlap/triggers)
                List<uint> mContactPairsIndices;

                /// List of indices of the mLostContactPairs list that are contact events (not overlap/triggers)
                List<uint> mLostContactPairsIndices;

                /// Reference to the physics world
                PhysicsWorld& mWorld;

                // -------------------- Methods -------------------- //

                /// Constructor
                CallbackData(List<reactphysics3d::ContactPair>* contactPairs, List<ContactManifold>* manifolds,
                             List<reactphysics3d::ContactPoint>* contactPoints, List<reactphysics3d::ContactPair>& lostContactPairs,
                             PhysicsWorld& world);

                /// Deleted copy constructor
                CallbackData(const CallbackData& callbackData) = delete;

                /// Deleted assignment operator
                CallbackData& operator=(const CallbackData& callbackData) = delete;

                /// Destructor
                ~CallbackData() = default;

            public:

                // -------------------- Methods -------------------- //

                /// Return the number of contact pairs
                /**
                 * @return The number of contact pairs
                 */
                uint getNbContactPairs() const;

                /// Return a given contact pair
                /**
                 * @param index Index of the contact pair to retrieve
                 * @return A contact pair object
                 */
                ContactPair getContactPair(uint index) const;

                // -------------------- Friendship -------------------- //

                friend class CollisionDetectionSystem;
        };

        /// Destructor
        virtual ~CollisionCallback() = default;

        /// This method is called when some contacts occur
        virtual void onContact(const CallbackData& callbackData)=0;
};

// Return the number of contact pairs (there is a single contact pair between two bodies in contact)
/**
 * @return The number of contact pairs
 */
inline uint CollisionCallback::CallbackData::getNbContactPairs() const {
    return mContactPairsIndices.size() + mLostContactPairsIndices.size();
}

// Return the number of contact points in the contact pair
/**
 * @return The number of contact points
 */
inline uint CollisionCallback::ContactPair::getNbContactPoints() const {
   return mContactPair.nbToTalContactPoints;
}

// Return the penetration depth between the two bodies in contact
/**
 * @return The penetration depth (larger than zero)
 */
inline decimal CollisionCallback::ContactPoint::getPenetrationDepth() const {
   return mContactPoint.getPenetrationDepth();
}

// Return the world-space contact normal (vector from first body toward second body)
/**
 * @return The contact normal direction at the contact point (in world-space)
 */
inline const Vector3& CollisionCallback::ContactPoint::getWorldNormal() const {
   return mContactPoint.getNormal();
}

// Return the contact point on the first collider in the local-space of the first collider
/**
 * @return The contact point in the local-space of the first collider (from body1) in contact
 */
inline const Vector3& CollisionCallback::ContactPoint::getLocalPointOnCollider1() const {
   return mContactPoint.getLocalPointOnShape1();
}

// Return the contact point on the second collider in the local-space of the second collider
/**
 * @return The contact point in the local-space of the second collider (from body2) in contact
 */
inline const Vector3& CollisionCallback::ContactPoint::getLocalPointOnCollider2() const {
   return mContactPoint.getLocalPointOnShape2();
}

}

#endif
