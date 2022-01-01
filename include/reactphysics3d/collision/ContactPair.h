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

#ifndef REACTPHYSICS3D_OVERLAPPING_PAIR_CONTACT_H
#define REACTPHYSICS3D_OVERLAPPING_PAIR_CONTACT_H

// Libraries
#include <reactphysics3d/mathematics/mathematics.h>
#include <reactphysics3d/configuration.h>
#include <reactphysics3d/engine/OverlappingPairs.h>

/// ReactPhysics3D namespace
namespace reactphysics3d {

// Structure ContactPair
/**
 * This structure represents a pair of shapes that are in contact during narrow-phase.
 */
struct ContactPair {

    public:

        // -------------------- Attributes -------------------- //

        /// Overlapping pair Id
        uint64 pairId;

        /// Number of potential contact manifolds
        uint8 nbPotentialContactManifolds;

        /// Indices of the potential contact manifolds
        uint32 potentialContactManifoldsIndices[NB_MAX_POTENTIAL_CONTACT_MANIFOLDS];

        /// Entity of the first body of the contact
        Entity body1Entity;

        /// Entity of the second body of the contact
        Entity body2Entity;

        /// Entity of the first collider of the contact
        Entity collider1Entity;

        /// Entity of the second collider of the contact
        Entity collider2Entity;

        /// True if the manifold is already in an island
        bool isAlreadyInIsland;

        /// Index of the contact pair in the array of pairs
        uint32 contactPairIndex;

        /// Index of the first contact manifold in the array
        uint32 contactManifoldsIndex;

        /// Number of contact manifolds
        uint32 nbContactManifolds;

        /// Index of the first contact point in the array of contact points
        uint32 contactPointsIndex;

        /// Total number of contact points in all the manifolds of the contact pair
        uint32 nbToTalContactPoints;

        /// True if the colliders of the pair were already colliding in the previous frame
        bool collidingInPreviousFrame;

        /// True if one of the two involved colliders is a trigger
        bool isTrigger;

        // -------------------- Methods -------------------- //

        /// Constructor
        ContactPair(uint64 pairId, Entity body1Entity, Entity body2Entity, Entity collider1Entity,
                    Entity collider2Entity, uint32 contactPairIndex, bool collidingInPreviousFrame, bool isTrigger)
            : pairId(pairId), nbPotentialContactManifolds(0), potentialContactManifoldsIndices{0}, body1Entity(body1Entity), body2Entity(body2Entity),
              collider1Entity(collider1Entity), collider2Entity(collider2Entity),
              isAlreadyInIsland(false), contactPairIndex(contactPairIndex), contactManifoldsIndex(0), nbContactManifolds(0),
              contactPointsIndex(0), nbToTalContactPoints(0), collidingInPreviousFrame(collidingInPreviousFrame), isTrigger(isTrigger) {

        }

        // Remove a potential manifold at a given index in the array
        void removePotentialManifoldAtIndex(uint32 index) {
            assert(index < nbPotentialContactManifolds);

            potentialContactManifoldsIndices[index] = potentialContactManifoldsIndices[nbPotentialContactManifolds - 1];
            nbPotentialContactManifolds--;
        }
};

}

#endif
