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

#ifndef REACTPHYSICS3D_BROAD_PHASE_ALGORITHM_H
#define REACTPHYSICS3D_BROAD_PHASE_ALGORITHM_H

// Libraries
#include <vector>
#include "../../body/CollisionBody.h"
#include "PairManager.h"

/// Namespace ReactPhysics3D
namespace reactphysics3d {

// Declarations
class CollisionDetection;

// Class BroadPhaseAlgorithm
/**
 * This class is an abstract class that represents an algorithm
 * used to perform the broad-phase of a collision detection. The
 * goal of the broad-phase algorithm is to compute the pair of bodies
 * that can collide. But it's important to understand that the
 * broad-phase doesn't compute only body pairs that can collide but
 * could also pairs of body that doesn't collide but are very close.
 * The goal of the broad-phase is to remove pairs of body that cannot
 * collide in order to avoid to much bodies to be tested in the
 * narrow-phase.
 */
class BroadPhaseAlgorithm {

    protected :

        // -------------------- Attributes -------------------- //

        /// Pair manager containing the overlapping pairs
        PairManager mPairManager;

        /// Reference to the collision detection object
        CollisionDetection& mCollisionDetection;
        
        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        BroadPhaseAlgorithm(const BroadPhaseAlgorithm& algorithm);

        /// Private assignment operator
        BroadPhaseAlgorithm& operator=(const BroadPhaseAlgorithm& algorithm);

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        BroadPhaseAlgorithm(CollisionDetection& collisionDetection);

        /// Destructor
        virtual ~BroadPhaseAlgorithm();
        
        /// Notify the broad-phase about a new object in the world
        virtual void addObject(CollisionBody* body, const AABB& aabb)=0;

        /// Notify the broad-phase about an object that has been removed from the world
        virtual void removeObject(CollisionBody* body)=0;

        /// Notify the broad-phase that the AABB of an object has changed
        virtual void updateObject(CollisionBody* body, const AABB& aabb)=0;

        /// Return a pointer to the first active pair (used to iterate over the active pairs)
        BodyPair* beginOverlappingPairsPointer() const;

        /// Return a pointer to the last active pair (used to iterate over the active pairs)
        BodyPair* endOverlappingPairsPointer() const;
};

// Return a pointer to the first active pair (used to iterate over the overlapping pairs)
inline BodyPair* BroadPhaseAlgorithm::beginOverlappingPairsPointer() const {
    return mPairManager.beginOverlappingPairsPointer();
}                                                           

// Return a pointer to the last active pair (used to iterate over the overlapping pairs)
inline BodyPair* BroadPhaseAlgorithm::endOverlappingPairsPointer() const {
   return mPairManager.endOverlappingPairsPointer();
}   

}

#endif

