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

#ifndef REACTPHYSICS3D_OVERLAP_CALLBACK_H
#define REACTPHYSICS3D_OVERLAP_CALLBACK_H

// Libraries
#include <reactphysics3d/containers/List.h>

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

            private:

                // -------------------- Attributes -------------------- //

                /// Pair of overlapping body entities
                Pair<Entity, Entity>& mOverlapPair;

                /// Reference to the physics world
                PhysicsWorld& mWorld;

                // -------------------- Methods -------------------- //

                /// Constructor
                OverlapPair(Pair<Entity, Entity>& overlapPair, reactphysics3d::PhysicsWorld& world);

            public:

                // -------------------- Methods -------------------- //

                /// Copy constructor
                OverlapPair(const OverlapPair& contactPair) = default;

                /// Assignment operator
                OverlapPair& operator=(const OverlapPair& contactPair) = default;

                /// Destructor
                ~OverlapPair() = default;

                /// Return a pointer to the first body in contact
                CollisionBody* getBody1() const;

                /// Return a pointer to the second body in contact
                CollisionBody* getBody2() const;

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

                List<Pair<Entity, Entity>>& mOverlapBodies;

                /// Reference to the physics world
                PhysicsWorld& mWorld;

                // -------------------- Methods -------------------- //

                /// Constructor
                CallbackData(List<Pair<Entity, Entity>>& overlapColliders, PhysicsWorld& world);

                /// Deleted copy constructor
                CallbackData(const CallbackData& callbackData) = delete;

                /// Deleted assignment operator
                CallbackData& operator=(const CallbackData& callbackData) = delete;

                /// Destructor
                ~CallbackData() = default;

            public:

                // -------------------- Methods -------------------- //

                /// Return the number of overlapping pairs of bodies
                uint getNbOverlappingPairs() const;

                /// Return a given overlapping pair of bodies
                OverlapPair getOverlappingPair(uint index) const;

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
inline uint OverlapCallback::CallbackData::getNbOverlappingPairs() const {
    return mOverlapBodies.size();
}

// Return a given overlapping pair of bodies
/// Note that the returned OverlapPair object is only valid during the call of the CollisionCallback::onOverlap()
/// method. Therefore, you need to get contact data from it and make a copy. Do not make a copy of the OverlapPair
/// object itself because it won't be valid after the CollisionCallback::onOverlap() call.
inline OverlapCallback::OverlapPair OverlapCallback::CallbackData::getOverlappingPair(uint index) const {

    assert(index < getNbOverlappingPairs());

    return OverlapPair(mOverlapBodies[index], mWorld);
}

}

#endif
