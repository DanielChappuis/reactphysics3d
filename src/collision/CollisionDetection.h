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
#include "../body/Body.h"
#include "OverlappingPair.h"
#include "../engine/PhysicsWorld.h"
#include "../memory/MemoryPool.h"
#include "narrowphase/GJK/GJKAlgorithm.h"
#include "narrowphase/SphereVsSphereAlgorithm.h"
#include "ContactInfo.h"
#include <vector>
#include <map>
#include <set>
#include <utility>


// ReactPhysics3D namespace
namespace reactphysics3d {

// Declarations
class BroadPhaseAlgorithm;
    
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
        std::map<std::pair<luint, luint>, OverlappingPair*>  overlappingPairs;          // Broad-phase overlapping pairs of bodies 
        std::set<std::pair<luint, luint> > currentStepOverlappingPairs;                 // Overlapping pairs of bodies at the current collision detection step
        std::set<std::pair<luint, luint> > lastStepOverlappingPairs;                    // Overlapping pairs of bodies at the last collision detection step
        BroadPhaseAlgorithm* broadPhaseAlgorithm;                                       // Broad-phase algorithm
        GJKAlgorithm narrowPhaseGJKAlgorithm;                                           // Narrow-phase GJK algorithm
        SphereVsSphereAlgorithm narrowPhaseSphereVsSphereAlgorithm;                     // Narrow-phase Sphere vs Sphere algorithm
        MemoryPool<Contact> memoryPoolContacts;                                         // Memory pool for the contacts
        MemoryPool<OverlappingPair> memoryPoolOverlappingPairs;                         // Memory pool for the overlapping pairs
        
        void computeBroadPhase();                                                       // Compute the broad-phase collision detection
        bool computeNarrowPhase();                                                      // Compute the narrow-phase collision detection
        NarrowPhaseAlgorithm& SelectNarrowPhaseAlgorithm(Collider* collider1,
                                                               Collider* collider2);    // Select the narrow phase algorithm to use given two colliders
   
    public :
        CollisionDetection(PhysicsWorld* physicsWorld);                                 // Constructor
        ~CollisionDetection();                                                          // Destructor
                                                                                           
        OverlappingPair* getOverlappingPair(luint body1ID, luint body2ID);              // Return an overlapping pair or null     
        bool computeCollisionDetection();                                               // Compute the collision detection
        void broadPhaseNotifyOverlappingPair(Body* body1, Body* body2);                 // Allow the broadphase to notify the collision detection about an overlapping pair
};

// Return an overlapping pair of bodies according to the given bodies ID
// The method returns null if the pair of bodies is not overlapping    
inline OverlappingPair* CollisionDetection::getOverlappingPair(luint body1ID, luint body2ID) {
    std::pair<luint, luint> pair = (body1ID < body2ID) ? std::make_pair(body1ID, body2ID) : std::make_pair(body2ID, body1ID);
    if (overlappingPairs.count(pair) == 1) {
        return overlappingPairs[pair];
    }
    return NULL;
}

// Select the narrow-phase collision algorithm to use given two colliders
inline NarrowPhaseAlgorithm& CollisionDetection::SelectNarrowPhaseAlgorithm(Collider* collider1, Collider* collider2) {
    
    // Sphere vs Sphere algorithm
    if (collider1->getType() == SPHERE && collider2->getType() == SPHERE) {
        return narrowPhaseSphereVsSphereAlgorithm;
    }
    else {   // GJK algorithm
        return narrowPhaseGJKAlgorithm; 
    }
}   

} // End of the ReactPhysics3D namespace

#endif
