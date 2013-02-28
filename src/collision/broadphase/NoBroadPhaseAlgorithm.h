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

        // -------------------- Attributes -------------------- //

        // All bodies of the world
        std::set<CollisionBody*> mBodies;

        // -------------------- Methods -------------------- //

        // Private copy-constructor
        NoBroadPhaseAlgorithm(const NoBroadPhaseAlgorithm& algorithm);

        // Private assignment operator
        NoBroadPhaseAlgorithm& operator=(const NoBroadPhaseAlgorithm& algorithm);

    public :

        // -------------------- Methods -------------------- //

        // Constructor
        NoBroadPhaseAlgorithm(CollisionDetection& collisionDetection);

        // Destructor
        virtual ~NoBroadPhaseAlgorithm();
        
        // Notify the broad-phase about a new object in the world
        virtual void addObject(CollisionBody* body, const AABB& aabb);

        // Notify the broad-phase about an object that has been removed from the world
        virtual void removeObject(CollisionBody* body);

        // Notify the broad-phase that the AABB of an object has changed
        virtual void updateObject(CollisionBody* body, const AABB& aabb);
};

        
// Notify the broad-phase about a new object in the world
inline void NoBroadPhaseAlgorithm::addObject(CollisionBody* body, const AABB& aabb) {
        
    // For each body that is already in the world
    for (std::set<CollisionBody*>::iterator it = mBodies.begin(); it != mBodies.end(); ++it) {
        
        // Add an overlapping pair with the new body
        mPairManager.addPair(*it, body);
    }
    
    // Add the new body into the list of bodies
    mBodies.insert(body);
}   

// Notify the broad-phase about an object that has been removed from the world
inline void NoBroadPhaseAlgorithm::removeObject(CollisionBody* body) {
    
    // For each body that is in the world
    for (std::set<CollisionBody*>::iterator it = mBodies.begin(); it != mBodies.end(); ++it) {
        
        if ((*it)->getID() != body->getID()) {
            
           // Remove the overlapping pair with the new body
           mPairManager.removePair((*it)->getID(), body->getID());
        }
    }
    
    // Remove the body from the broad-phase
    mBodies.erase(body);
}  
        
// Notify the broad-phase that the AABB of an object has changed
inline void NoBroadPhaseAlgorithm::updateObject(CollisionBody* body, const AABB& aabb) {
    // Do nothing
    return;
}     

} // End of reactphysics3d namespace

#endif


