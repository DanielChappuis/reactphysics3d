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

#ifndef REACTPHYSICS3D_ISLAND_H
#define REACTPHYSICS3D_ISLAND_H

// Libraries
#include <reactphysics3d/constraint/Joint.h>

namespace reactphysics3d {

// Declarations
class RigidBody;
class SingleFrameAllocator;
class ContactManifold;

// Class Island
/**
 * An island represent an isolated group of awake bodies that are connected with each other by
 * some contraints (contacts or joints).
 */
class Island {

    private:

        // -------------------- Attributes -------------------- //

        /// Array with all the bodies of the island
        RigidBody** mBodies;

        /// Array with all the contact manifolds between bodies of the island
        ContactManifold** mContactManifolds;

        /// Current number of bodies in the island
        uint32 mNbBodies;

        /// Current number of contact manifold in the island
        uint32 mNbContactManifolds;

    public:

        // -------------------- Methods -------------------- //

        /// Constructor
        Island(uint32 nbMaxBodies, uint32 nbMaxContactManifolds, MemoryManager& memoryManager);

        /// Destructor
        ~Island();

        /// Deleted assignment operator
        Island& operator=(const Island& island) = delete;

        /// Deleted copy-constructor
        Island(const Island& island) = delete;

        /// Add a body into the island
        void addBody(RigidBody* body);

        /// Add a contact manifold into the island
        void addContactManifold(ContactManifold* contactManifold);

        /// Add a joint into the island
        void addJoint(Joint* joint);

        /// Return the number of bodies in the island
        uint32 getNbBodies() const;

        /// Return the number of contact manifolds in the island
        uint32 getNbContactManifolds() const;

        /// Return the number of joints in the island
        uint32 getNbJoints() const;

        /// Return a pointer to the array of bodies
        RigidBody** getBodies();

        /// Return a pointer to the array of contact manifolds
        ContactManifold** getContactManifolds();

        // -------------------- Friendship -------------------- //

        friend class PhysicsWorld;
};

// Add a body into the island
RP3D_FORCE_INLINE void Island::addBody(RigidBody* body) {
    assert(!body->isSleeping());
    mBodies[mNbBodies] = body;
    mNbBodies++;
}

// Add a contact manifold into the island
RP3D_FORCE_INLINE void Island::addContactManifold(ContactManifold* contactManifold) {
    mContactManifolds[mNbContactManifolds] = contactManifold;
    mNbContactManifolds++;
}

// Return the number of bodies in the island
RP3D_FORCE_INLINE uint32 Island::getNbBodies() const {
    return mNbBodies;
}

// Return the number of contact manifolds in the island
RP3D_FORCE_INLINE uint32 Island::getNbContactManifolds() const {
    return mNbContactManifolds;
}

// Return a pointer to the array of bodies
RP3D_FORCE_INLINE RigidBody** Island::getBodies() {
    return mBodies;
}

// Return a pointer to the array of contact manifolds
RP3D_FORCE_INLINE ContactManifold** Island::getContactManifolds() {
    return mContactManifolds;
}

}

#endif
