/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2016 Daniel Chappuis                                       *
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

#ifndef REACTPHYSICS3D_NARROW_PHASE_ALGORITHM_H
#define REACTPHYSICS3D_NARROW_PHASE_ALGORITHM_H

// Libraries
#include "body/Body.h"
#include "constraint/ContactPoint.h"
#include "memory/MemoryAllocator.h"
#include "engine/OverlappingPair.h"
#include "collision/CollisionShapeInfo.h"

/// Namespace ReactPhysics3D
namespace reactphysics3d {

class CollisionDetection;

// Class NarrowPhaseCallback
/**
 * This abstract class is the base class for a narrow-phase collision
 * callback class.
 */
class NarrowPhaseCallback {

    public:

        /// Called by a narrow-phase collision algorithm when a new contact has been found
        virtual void notifyContact(OverlappingPair* overlappingPair,
                                   const ContactPointInfo& contactInfo)=0;

};

// Class NarrowPhaseAlgorithm
/**
 * This abstract class is the base class for a  narrow-phase collision
 * detection algorithm. The goal of the narrow phase algorithm is to
 * compute information about the contact between two proxy shapes.
 */
class NarrowPhaseAlgorithm {

    protected :

        // -------------------- Attributes -------------------- //

        /// Pointer to the collision detection object
        CollisionDetection* mCollisionDetection;

        /// Pointer to the memory allocator
        MemoryAllocator* mMemoryAllocator;

        /// Overlapping pair of the bodies currently tested for collision
        OverlappingPair* mCurrentOverlappingPair;
        
        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        NarrowPhaseAlgorithm(const NarrowPhaseAlgorithm& algorithm);

        /// Private assignment operator
        NarrowPhaseAlgorithm& operator=(const NarrowPhaseAlgorithm& algorithm);

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        NarrowPhaseAlgorithm();

        /// Destructor
        virtual ~NarrowPhaseAlgorithm();

        /// Initalize the algorithm
        virtual void init(CollisionDetection* collisionDetection, MemoryAllocator* memoryAllocator);
        
        /// Set the current overlapping pair of bodies
        void setCurrentOverlappingPair(OverlappingPair* overlappingPair);

        /// Compute a contact info if the two bounding volume collide
        virtual void testCollision(const CollisionShapeInfo& shape1Info,
                                   const CollisionShapeInfo& shape2Info,
                                   NarrowPhaseCallback* narrowPhaseCallback)=0;
};

// Set the current overlapping pair of bodies
inline void NarrowPhaseAlgorithm::setCurrentOverlappingPair(OverlappingPair* overlappingPair) {
    mCurrentOverlappingPair = overlappingPair;
}      

}

#endif


