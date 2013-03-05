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

#ifndef COLLISION_DETECTION_H
#define COLLISION_DETECTION_H

// Libraries
#include "../body/CollisionBody.h"
#include "broadphase/BroadPhaseAlgorithm.h"
#include "BroadPhasePair.h"
#include "../memory/MemoryPool.h"
#include "narrowphase/GJK/GJKAlgorithm.h"
#include "narrowphase/SphereVsSphereAlgorithm.h"
#include "ContactInfo.h"
#include <vector>
#include <map>
#include <set>
#include <utility>
#include <iostream>     // TODO : Delete this


/// ReactPhysics3D namespace
namespace reactphysics3d {

// Declarations
class BroadPhaseAlgorithm;
class CollisionWorld;

// Class CollisionDetection
/**
 * This class computes the collision detection algorithms. We first
 * perform a broad-phase algorithm to know which pairs of bodies can
 * collide and then we run a narrow-phase algorithm to compute the
 * collision contacts between bodies.
 */
class CollisionDetection {

    private :

        // -------------------- Attributes -------------------- //

        /// Pointer to the physics world
        CollisionWorld* mWorld;

        /// Broad-phase overlapping pairs
        std::map<bodyindexpair, BroadPhasePair*> mOverlappingPairs;

        /// Broad-phase algorithm
        BroadPhaseAlgorithm* mBroadPhaseAlgorithm;

        /// Narrow-phase GJK algorithm
        GJKAlgorithm mNarrowPhaseGJKAlgorithm;

        /// Narrow-phase Sphere vs Sphere algorithm
        SphereVsSphereAlgorithm mNarrowPhaseSphereVsSphereAlgorithm;

        /// Memory pool for contactinfo
        MemoryPool<ContactInfo> mMemoryPoolContactInfos;

        /// Memory pool for broad-phase pairs
        MemoryPool<BroadPhasePair> mMemoryPoolBroadPhasePairs;

        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        CollisionDetection(const CollisionDetection& collisionDetection);

        /// Private assignment operator
        CollisionDetection& operator=(const CollisionDetection& collisionDetection);

        /// Compute the broad-phase collision detection
        void computeBroadPhase();

        /// Compute the narrow-phase collision detection
        bool computeNarrowPhase();

        /// Select the narrow phase algorithm to use given two collision shapes
        NarrowPhaseAlgorithm& SelectNarrowPhaseAlgorithm(CollisionShape* collisionShape1,
                                                         CollisionShape* collisionShape2);
   
    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        CollisionDetection(CollisionWorld* world);

        /// Destructor
        ~CollisionDetection();

        /// Add a body to the collision detection
        void addBody(CollisionBody* body);

        /// Remove a body from the collision detection
        void removeBody(CollisionBody* body);

        /// Compute the collision detection
        bool computeCollisionDetection();

        /// Allow the broadphase to notify the collision detection about a new overlapping pair.
        void broadPhaseNotifyAddedOverlappingPair(BodyPair* pair);

        /// Allow the broadphase to notify the collision detection about a removed overlapping pair
        void broadPhaseNotifyRemovedOverlappingPair(BodyPair* pair);
};

// Select the narrow-phase collision algorithm to use given two collision shapes
inline NarrowPhaseAlgorithm& CollisionDetection::SelectNarrowPhaseAlgorithm(
                             CollisionShape* collisionShape1, CollisionShape* collisionShape2) {
    
    // Sphere vs Sphere algorithm
    if (collisionShape1->getType() == SPHERE && collisionShape2->getType() == SPHERE) {
        return mNarrowPhaseSphereVsSphereAlgorithm;
    }
    else {   // GJK algorithm
        return mNarrowPhaseGJKAlgorithm;
    }
}  

// Add a body to the collision detection
inline void CollisionDetection::addBody(CollisionBody* body) {
    
    // Add the body to the broad-phase
    mBroadPhaseAlgorithm->addObject(body, *(body->getAABB()));
}  

// Remove a body from the collision detection
inline void CollisionDetection::removeBody(CollisionBody* body) {
    
    // Remove the body from the broad-phase
    mBroadPhaseAlgorithm->removeObject(body);
}                                                    

}

#endif
