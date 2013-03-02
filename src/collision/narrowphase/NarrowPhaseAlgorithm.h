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

#ifndef NARROW_PHASE_ALGORITHM_H
#define NARROW_PHASE_ALGORITHM_H

// Libraries
#include "../../body/Body.h"
#include "../ContactInfo.h"
#include "../broadphase/PairManager.h"
#include "../../memory/MemoryPool.h"
#include "../BroadPhasePair.h"


// Namespace ReactPhysics3D
namespace reactphysics3d {
    

/*  -------------------------------------------------------------------
    Class NarrowPhaseAlgorithm :
        This class is an abstract class that represents an algorithm
        used to perform the narrow-phase of a collision detection. The
        goal of the narrow phase algorithm is to compute contact
        informations of a collision between two bodies.
    -------------------------------------------------------------------
*/
class NarrowPhaseAlgorithm {

    protected :

        // -------------------- Attributes -------------------- //

        // Reference to the memory pool
        MemoryPool<ContactInfo>& mMemoryPoolContactInfos;

        // Overlapping pair of the bodies currently tested for collision
        BroadPhasePair* mCurrentOverlappingPair;
        
        // -------------------- Methods -------------------- //

        // Private copy-constructor
        NarrowPhaseAlgorithm(const NarrowPhaseAlgorithm& algorithm);

        // Private assignment operator
        NarrowPhaseAlgorithm& operator=(const NarrowPhaseAlgorithm& algorithm);

    public :

        // -------------------- Methods -------------------- //

        // Constructor
        NarrowPhaseAlgorithm(MemoryPool<ContactInfo>& memoryPool);

        // Destructor
        virtual ~NarrowPhaseAlgorithm();
        
        // Set the current overlapping pair of bodies
        void setCurrentOverlappingPair(BroadPhasePair* overlappingPair);

        // Return true and compute a contact info if the two bounding volume collide
        virtual bool testCollision(const CollisionShape* collisionShape1,
                                   const Transform& transform1,
                                   const CollisionShape* collisionShape2,
                                   const Transform& transform2,
                                   ContactInfo*& contactInfo)=0;
};

// Set the current overlapping pair of bodies
inline void NarrowPhaseAlgorithm::setCurrentOverlappingPair(BroadPhasePair *overlappingPair) {
    mCurrentOverlappingPair = overlappingPair;
}      

}

#endif


