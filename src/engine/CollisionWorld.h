/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2010-2013 Daniel Chappuis                                       *
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

#ifndef COLLISION_WORLD_H
#define COLLISION_WORLD_H

// Libraries
#include <vector>
#include <set>
#include <algorithm>
#include "../mathematics/mathematics.h"
#include "Profiler.h"
#include "../body/CollisionBody.h"
#include "OverlappingPair.h"
#include "../collision/CollisionDetection.h"
#include "../constraint/Constraint.h"
#include "../constraint/ContactPoint.h"
#include "../memory/MemoryPool.h"

/// Namespace reactphysics3d
namespace reactphysics3d {

// Class CollisionWorld
/**
 * This class represent a world where it is possible to move bodies
 * by hand and to test collision between each other. In this kind of
 * world, the bodies movement is not computed using the laws of physics.
 */
class CollisionWorld {

    protected :

        // -------------------- Attributes -------------------- //

        /// Reference to the collision detection
        CollisionDetection mCollisionDetection;

        /// All the bodies (rigid and soft) of the world
        std::set<CollisionBody*> mBodies;

        /// Broad-phase overlapping pairs of bodies
        std::map<bodyindexpair, OverlappingPair*>  mOverlappingPairs;

        /// Current body ID
        bodyindex mCurrentBodyID;

        /// Memory pool
        MemoryPool<CollisionBody> mMemoryPoolCollisionBodies;

        /// List of free ID for rigid bodies
        std::vector<luint> mFreeBodiesIDs;

        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        CollisionWorld(const CollisionWorld& world);

        /// Private assignment operator
        CollisionWorld& operator=(const CollisionWorld& world);

        /// Notify the world about a new broad-phase overlapping pair
        virtual void notifyAddedOverlappingPair(const BroadPhasePair* addedPair);

        /// Notify the world about a removed broad-phase overlapping pair
        virtual void notifyRemovedOverlappingPair(const BroadPhasePair* removedPair);

        /// Notify the world about a new narrow-phase contact
        virtual void notifyNewContact(const BroadPhasePair* pair, const ContactInfo* contactInfo);

        /// Update the overlapping pair
        virtual void updateOverlappingPair(const BroadPhasePair* pair);

        /// Return the next available body ID
        bodyindex computeNextAvailableBodyID();

    public :

        // ----- Methods ----- //

        /// Constructor
        CollisionWorld();

        /// Destructor
        virtual ~CollisionWorld();

        /// Return an iterator to the beginning of the bodies of the physics world
        std::set<CollisionBody*>::iterator getBodiesBeginIterator();

        /// Return an iterator to the end of the bodies of the physics world
        std::set<CollisionBody*>::iterator getBodiesEndIterator();

        /// Create a collision body
        CollisionBody* createCollisionBody(const Transform& transform,
                                           CollisionShape* collisionShape);

        /// Destroy a collision body
        void destroyCollisionBody(CollisionBody* collisionBody);

        // ----- Friends ----- //

        friend class CollisionDetection;
};

// Return an iterator to the beginning of the bodies of the physics world
inline std::set<CollisionBody*>::iterator CollisionWorld::getBodiesBeginIterator() {
    return mBodies.begin();
}

// Return an iterator to the end of the bodies of the physics world
inline std::set<CollisionBody*>::iterator CollisionWorld::getBodiesEndIterator() {
    return mBodies.end();
}

}

 #endif
