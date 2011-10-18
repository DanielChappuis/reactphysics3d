/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2010 Daniel Chappuis                                            *
*********************************************************************************
*                                                                               *
* Permission is hereby granted, free of charge, to any person obtaining a copy  *
* of this software and associated documentation files (the "Software"), to deal *
* in the Software without restriction, including without limitation the rights  *
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell     *
* copies of the Software, and to permit persons to whom the Software is         *
* furnished to do so, subject to the following conditions:                      *
*                                                                               *
* The above copyright notice and this permission notice shall be included in    *
* all copies or substantial portions of the Software.                           *
*                                                                               *
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR    *
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,      *
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE   *
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER        *
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, *
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN     *
* THE SOFTWARE.                                                                 *
********************************************************************************/

#ifndef COLLISION_DETECTION_H
#define COLLISION_DETECTION_H

// Libraries
#include "../body/Body.h"
#include "OverlappingPair.h"
#include "../engine/PhysicsWorld.h"
#include "../memory/MemoryPool.h"
#include "ContactInfo.h"
#include <vector>
#include <map>
#include <set>
#include <utility>


// ReactPhysics3D namespace
namespace reactphysics3d {

// Declarations
class BroadPhaseAlgorithm;
class NarrowPhaseAlgorithm;
    
/*  -------------------------------------------------------------------
    Class CollisionDetection :
        This class computes the collision detection algorithms. We first
        perfom a broad-phase algorithm to know which pairs of bodies can
        collide and then we run a narrow-phase algorithm to compute the
        collision contacts between bodies.
    -------------------------------------------------------------------
*/
class CollisionDetection {
    private :
        PhysicsWorld* world;                                                    // Pointer to the physics world        
        std::map<std::pair<luint, luint>, OverlappingPair*>  overlappingPairs;  // Broad-phase overlapping pairs of bodies 
        std::set<std::pair<luint, luint> > currentStepOverlappingPairs;         // Overlapping pairs of bodies at the current collision detection step
        std::set<std::pair<luint, luint> > lastStepOverlappingPairs;            // Overlapping pairs of bodies at the last collision detection step
        BroadPhaseAlgorithm* broadPhaseAlgorithm;                               // Broad-phase algorithm
        NarrowPhaseAlgorithm* narrowPhaseAlgorithm;                             // Narrow-phase algorithm
        MemoryPool<Contact> memoryPoolContacts;                                 // Memory pool for the contacts
        MemoryPool<OverlappingPair> memoryPoolOverlappingPairs;                 // Memory pool for the overlapping pairs
        
        void computeBroadPhase();                                               // Compute the broad-phase collision detection
        bool computeNarrowPhase();                                              // Compute the narrow-phase collision detection
        
    public :
        CollisionDetection(PhysicsWorld* physicsWorld);                             // Constructor
        ~CollisionDetection();                                                      // Destructor
                                                                                            
        OverlappingPair* getOverlappingPair(luint body1ID, luint body2ID);          // Return an overlapping pair or null     
        bool computeCollisionDetection();                                           // Compute the collision detection
        void broadPhaseNotifyOverlappingPair(Body* body1, Body* body2);             // Allow the broadphase to notify the collision detection about an overlapping pair
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

} // End of the ReactPhysics3D namespace

#endif
