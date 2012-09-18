/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2010-2012 Daniel Chappuis                                       *
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
#include "OverlappingPair.h"
#include "broadphase/BroadPhaseAlgorithm.h"
#include "../memory/MemoryPool.h"
#include "narrowphase/GJK/GJKAlgorithm.h"
#include "narrowphase/SphereVsSphereAlgorithm.h"
#include "ContactInfo.h"
#include <vector>
#include <map>
#include <set>
#include <utility>
#include <iostream>     // TODO : Delete this


// ReactPhysics3D namespace
namespace reactphysics3d {

// Declarations
class BroadPhaseAlgorithm;
class PhysicsWorld;

/*  -------------------------------------------------------------------
    Class CollisionDetection :
        This class computes the collision detection algorithms. We first
        perform a broad-phase algorithm to know which pairs of bodies can
        collide and then we run a narrow-phase algorithm to compute the
        collision contacts between bodies.
    -------------------------------------------------------------------
*/
class CollisionDetection {

    private :
        PhysicsWorld* world;                                                            // Pointer to the physics world        
        std::map<std::pair<bodyindex, bodyindex>, OverlappingPair*>  overlappingPairs;  // Broad-phase overlapping pairs of bodies
        std::set<std::pair<bodyindex, bodyindex> > currentStepOverlappingPairs;         // Overlapping pairs of bodies at the current collision detection step
        std::set<std::pair<bodyindex, bodyindex> > lastStepOverlappingPairs;            // Overlapping pairs of bodies at the last collision detection step
        BroadPhaseAlgorithm* broadPhaseAlgorithm;                                       // Broad-phase algorithm
        GJKAlgorithm narrowPhaseGJKAlgorithm;                                           // Narrow-phase GJK algorithm
        SphereVsSphereAlgorithm narrowPhaseSphereVsSphereAlgorithm;                     // Narrow-phase Sphere vs Sphere algorithm
        MemoryPool<Contact> memoryPoolContacts;                                         // Memory pool for the contacts
        MemoryPool<OverlappingPair> memoryPoolOverlappingPairs;                         // Memory pool for the overlapping pairs
        MemoryPool<ContactInfo> memoryPoolContactInfos;                                 // Memory pool for the contact info
        
        void computeBroadPhase();                                                       // Compute the broad-phase collision detection
        bool computeNarrowPhase();                                                      // Compute the narrow-phase collision detection
        NarrowPhaseAlgorithm& SelectNarrowPhaseAlgorithm(CollisionShape* collisionShape1,
                                                         CollisionShape* collisionShape2);          // Select the narrow phase algorithm to use given two collision shapes
   
    public :
        CollisionDetection(PhysicsWorld* physicsWorld);                                 // Constructor
        ~CollisionDetection();                                                          // Destructor
        
        void addBody(CollisionBody* body);                                                       // Add a body to the collision detection
        void removeBody(CollisionBody* body);                                                    // Remove a body from the collision detection
        OverlappingPair* getOverlappingPair(bodyindex body1ID, bodyindex body2ID);      // Return an overlapping pair or null
        bool computeCollisionDetection();                                               // Compute the collision detection
        void broadPhaseNotifyAddedOverlappingPair(const BroadPhasePair* pair);          // Allow the broadphase to notify the collision detection about a new overlapping pair
        void broadPhaseNotifyRemovedOverlappingPair(const BroadPhasePair* pair);        // Allow the broadphase to notify the collision detection about a removed overlapping pair
};

// Return an overlapping pair of bodies according to the given bodies ID
// The method returns null if the pair of bodies is not overlapping    
inline OverlappingPair* CollisionDetection::getOverlappingPair(bodyindex body1ID, bodyindex body2ID) {
    std::pair<bodyindex, bodyindex> pair = (body1ID < body2ID) ? std::make_pair(body1ID, body2ID) : std::make_pair(body2ID, body1ID);
    if (overlappingPairs.count(pair) == 1) {
        return overlappingPairs[pair];
    }
    return 0;
}

// Select the narrow-phase collision algorithm to use given two collision shapes
inline NarrowPhaseAlgorithm& CollisionDetection::SelectNarrowPhaseAlgorithm(CollisionShape* collisionShape1, CollisionShape* collisionShape2) {
    
    // Sphere vs Sphere algorithm
    if (collisionShape1->getType() == SPHERE && collisionShape2->getType() == SPHERE) {
        return narrowPhaseSphereVsSphereAlgorithm;
    }
    else {   // GJK algorithm
        return narrowPhaseGJKAlgorithm; 
    }
}  

// Add a body to the collision detection
inline void CollisionDetection::addBody(CollisionBody* body) {
    
    // Add the body to the broad-phase
    broadPhaseAlgorithm->addObject(body, *(body->getAABB()));
}  

// Remove a body from the collision detection
inline void CollisionDetection::removeBody(CollisionBody* body) {
    
    // Remove the body from the broad-phase
    broadPhaseAlgorithm->removeObject(body);
}                                                    

} // End of the ReactPhysics3D namespace

#endif
