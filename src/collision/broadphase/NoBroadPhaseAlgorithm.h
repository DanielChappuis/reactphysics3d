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

#ifndef NO_BROAD_PHASE_ALGORITHM_H
#define NO_BROAD_PHASE_ALGORITHM_H

// Libraries
#include "BroadPhaseAlgorithm.h"
#include <algorithm>
#include <set>
#include <iostream>

// Namespace ReactPhysics3D
namespace reactphysics3d {

/*  --------------------------------------------------------------------
    Class NoBroadPhaseAlgorithm :
        This class implements a broad-phase algorithm that does nothing.
        It should be use if we don't want to perform a broad-phase for
        the collision detection.
    --------------------------------------------------------------------
*/
class NoBroadPhaseAlgorithm : public BroadPhaseAlgorithm {
    protected :
        std::set<Body*> bodies;                                         // All bodies of the world

    public :
        NoBroadPhaseAlgorithm(CollisionDetection& collisionDetection);  // Constructor
        virtual ~NoBroadPhaseAlgorithm();                               // Destructor
        
        virtual void addObject(Body* body, const AABB& aabb);         // Notify the broad-phase about a new object in the world
        virtual void removeObject(Body* body);                        // Notify the broad-phase about an object that has been removed from the world
        virtual void updateObject(Body* body, const AABB& aabb);      // Notify the broad-phase that the AABB of an object has changed
};

        
// Notify the broad-phase about a new object in the world
inline void NoBroadPhaseAlgorithm::addObject(Body* body, const AABB& aabb) {
    
    std::cout << "New body in broadphase with id=" << body->getID() << std::endl;
    
    // For each body that is already in the world
    for (std::set<Body*>::iterator it = bodies.begin(); it != bodies.end(); ++it) {
        
        // Add an overlapping pair with the new body
        pairManager.addPair(*it, body);
    }
    
    // Add the new body into the list of bodies
    bodies.insert(body);
}   

// Notify the broad-phase about an object that has been removed from the world
inline void NoBroadPhaseAlgorithm::removeObject(Body* body) {
    
    // For each body that is in the world
    for (std::set<Body*>::iterator it = bodies.begin(); it != bodies.end(); ++it) {
        
        if ((*it)->getID() != body->getID()) {
            
           // Remove the overlapping pair with the new body
           pairManager.removePair((*it)->getID(), body->getID());
        }
    }
    
    // Remove the body from the broad-phase
    bodies.erase(body);
}  
        
// Notify the broad-phase that the AABB of an object has changed
inline void NoBroadPhaseAlgorithm::updateObject(Body* body, const AABB& aabb) {
    // Do nothing
    return;
}     

} // End of reactphysics3d namespace

#endif


