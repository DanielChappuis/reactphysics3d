/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2022 Daniel Chappuis                                       *
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

#ifndef REACTPHYSICS3D_OVERLAP_CALLBACK_H
#define REACTPHYSICS3D_OVERLAP_CALLBACK_H

// Libraries
#include <reactphysics3d/containers/Array.h>
#include <reactphysics3d/collision/ContactPair.h>

/// ReactPhysics3D namespace
namespace reactphysics3d {

// Declarations
class CollisionBody;
class PhysicsWorld;
class Collider;
struct Entity;

// Class OverlapCallback
/**
 * This class can be used to register a callback for collision overlap queries between bodies.
 * You should implement your own class inherited from this one and implement the onOverlap() method.
 */
class OverlapCallback {

    public:

        // Class OverlapPair
        /**
         * This class represents the contact between two colliders of the physics world.
         */
        class OverlapPair {

            public:

                /// Enumeration EventType that describes the type of overlapping event
                enum class EventType {

                    /// This overlap is a new overlap between the two
                    /// colliders (the colliders where not overlapping in the previous frame)
                    OverlapStart,

                    /// The two colliders were already overlapping in the previous frame and this is a new or updated overlap
                    OverlapStay,

                    /// The two colliders were overlapping in the previous frame and are not overlapping anymore
                    OverlapExit
                };

            private:

                // -------------------- Attributes -------------------- //

                /// Contact pair
                ContactPair& mContactPair;

                /// Reference to the physics world
                PhysicsWorld& mWorld;

                /// True if the pair were overlapping in the previous frame but not in the current one
                bool mIsLostOverlapPair;

                // -------------------- Methods -------------------- //

                /// Constructor
                OverlapPair(ContactPair& contactPair, reactphysics3d::PhysicsWorld& world, bool isLostOverlappingPair);

            public:

                // -------------------- Methods -------------------- //

                /// Copy constructor
                OverlapPair(const OverlapPair& contactPair) = default;

                /// Assignment operator
                OverlapPair& operator=(const OverlapPair& contactPair) = default;

                /// Destructor
                ~OverlapPair() = default;

                /// Return a pointer to the first collider in contact
                Collider* getCollider1() const;

                /// Return a pointer to the second collider in contact
                Collider* getCollider2() const;

                /// Return a pointer to the first body in contact
                CollisionBody* getBody1() const;

                /// Return a pointer to the second body in contact
                CollisionBody* getBody2() const;

                /// Return the corresponding type of event for this overlapping pair
                EventType getEventType() const;

                // -------------------- Friendship -------------------- //

                friend class OverlapCallback;
        };

        // Class CallbackData
        /**
         * This class contains data about overlap between bodies
         */
        class CallbackData {

            private:

                // -------------------- Attributes -------------------- //

                /// Reference to the array of contact pairs (contains contacts and triggers events)
                Array<ContactPair>& mContactPairs;

                /// Reference to the array of lost contact pairs (contains contacts and triggers events)
                Array<ContactPair>& mLostContactPairs;

                /// Array of indices of the mContactPairs array that are overlap/triggers events (not contact events)
                Array<uint64> mContactPairsIndices;

                /// Array of indices of the mLostContactPairs array that are overlap/triggers events (not contact events)
                Array<uint64> mLostContactPairsIndices;

                /// Reference to the physics world
                PhysicsWorld& mWorld;

                // -------------------- Methods -------------------- //

                /// Constructor
                CallbackData(Array<ContactPair>& contactPairs, Array<ContactPair>& lostContactPairs, bool onlyReportTriggers, PhysicsWorld& world);

                /// Deleted copy constructor
                CallbackData(const CallbackData& callbackData) = delete;

                /// Deleted assignment operator
                CallbackData& operator=(const CallbackData& callbackData) = delete;

                /// Destructor
                ~CallbackData() = default;

            public:

                // -------------------- Methods -------------------- //

                /// Return the number of overlapping pairs of bodies
                uint32 getNbOverlappingPairs() const;

                /// Return a given overlapping pair of bodies
                OverlapPair getOverlappingPair(uint32 index) const;

                // -------------------- Friendship -------------------- //

                friend class CollisionDetectionSystem;
        };

        /// Destructor
        virtual ~OverlapCallback() {

        }

        /// This method will be called to report bodies that overlap
        virtual void onOverlap(CallbackData& callbackData)=0;
};

// Return the number of overlapping pairs of bodies
RP3D_FORCE_INLINE uint32 OverlapCallback::CallbackData::getNbOverlappingPairs() const {
    return static_cast<uint32>(mContactPairsIndices.size() + mLostContactPairsIndices.size());
}

// Return a given overlapping pair of bodies
/// Note that the returned OverlapPair object is only valid during the call of the CollisionCallback::onOverlap()
/// method. Therefore, you need to get contact data from it and make a copy. Do not make a copy of the OverlapPair
/// object itself because it won't be valid after the CollisionCallback::onOverlap() call.
RP3D_FORCE_INLINE OverlapCallback::OverlapPair OverlapCallback::CallbackData::getOverlappingPair(uint32 index) const {

    assert(index < getNbOverlappingPairs());

    if (index < mContactPairsIndices.size()) {
        // Return a contact pair
        return OverlapCallback::OverlapPair((mContactPairs)[mContactPairsIndices[index]],  mWorld, false);
    }
    else {

        // Return a lost contact pair
        return OverlapCallback::OverlapPair(mLostContactPairs[mLostContactPairsIndices[index - mContactPairsIndices.size()]], mWorld, true);
    }
}

}

#endif
